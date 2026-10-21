// SPDX-License-Identifier: GPL-2.0-only
/*
 * HiSilicon Kirin 990 DisplayPort / HDMI external display output driver.
 *
 * The Kirin 990 Display Subsystem (DSS v510) outputs to external displays
 * through a Synopsys DesignWare Cores DP Transmitter (DW DP TX) at 0xf8500000,
 * an integrated USB 3.1 / DP Combo PHY at 0xf8481000 (2 lanes dedicated to DP),
 * and an on-board Realtek RTD2172 DP-to-HDMI bridge chip (I2C address 0x35).
 *
 * Bridge power rails (EC_HDMI_VDD_ON and EC_1V1_EN) are switched by the
 * Embedded Controller (huawei-echub) via commands 0x53/0x57 (on) and 0x56/0x52 (off).
 * Hotplug detection is handled via GPIO 282 (fa8b7000.gpio pin 2).
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/mfd/huawei-echub.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include <drm/display/drm_dp_helper.h>
#include <drm/display/drm_hdmi_audio_helper.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder.h>
#include <drm/drm_managed.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include <sound/hdmi-codec.h>

#include "kirin990_drv.h"

/* DWC DP TX hardware registers (relative to 0xf8500000) */
#define DW_DP_VERSION_NUMBER		0x0000
#define DW_DP_VERSION_TYPE		0x0004
#define DW_DP_ID			0x0008
#define   DW_DP_ID_EXPECTED		0x900116c3

#define DW_DP_CCTL			0x0200
#define   CCTL_FORCE_HPD		BIT(4)
#define   CCTL_ENHANCE_FRAMING		BIT(1)
#define   CCTL_SCRAMBLE_DIS		BIT(0)

#define DW_DP_SOFT_RESET_CTRL		0x0204
#define   RESET_VIDEO			BIT(5)
#define   RESET_AUX			BIT(4)
#define   RESET_AUDIO			BIT(3)
#define   RESET_HDCP			BIT(2)
#define   RESET_PHY			BIT(1)
#define   RESET_CONTROLLER		BIT(0)
#define   RESET_ALL			0x3f

#define DW_DP_VSAMPLE_CTRL		0x0300
#define   VSAMPLE_VIDEO_STREAM_EN	BIT(5)

#define DW_DP_VINPUT_POLARITY		0x030c
#define   POL_HSYNC			BIT(1)
#define   POL_VSYNC			BIT(0)

#define DW_DP_VIDEO_CONFIG1		0x0310
#define DW_DP_VIDEO_CONFIG2		0x0314
#define DW_DP_VIDEO_CONFIG3		0x0318
#define DW_DP_VIDEO_CONFIG4		0x031c
#define DW_DP_VIDEO_CONFIG5		0x0320

#define DW_DP_AUDIO_CONFIG		0x0400
#define   AUDIO_CONFIG_MUTE		BIT(15)
#define   AUDIO_CONFIG_SPDIF		BIT(0)

#define DW_DP_SDP_CONFIG		0x0500
#define   SDP_TIMESTAMP_EN		BIT(0)
#define   SDP_AUDIO_STREAM_EN		BIT(1)
#define   SDP_INFOFRAME_EN		BIT(2)
#define   SDP_VIDEO_STREAM_EN		BIT(3)

#define DW_DP_SDP_CONFIG2		0x0504
#define   SDP2_AUDIO_STREAM_EN		BIT(1)

#define DW_DP_AUDIO_INFOFRAME_HEADER	0x0600
#define DW_DP_AUDIO_INFOFRAME_DATA0	0x0604
#define DW_DP_AUDIO_INFOFRAME_DATA1	0x0608
#define DW_DP_AUDIO_INFOFRAME_DATA2	0x060c

#define DW_DP_LINK_BW_SET		0x0500
#define   BW_1620MBPS			0x06
#define   BW_2700MBPS			0x0a
#define   BW_5400MBPS			0x14

#define DW_DP_LANE_COUNT_SET		0x0504
#define   LANES_2			0x02

#define DW_DP_PHY_CONFIG		0x0a00

#define DW_DP_AUX_CMD			0x0b00
#define   AUX_CMD_TYPE			GENMASK(31, 28)
#define   AUX_ADDR			GENMASK(27, 8)
#define   AUX_I2C_ADDR_ONLY		BIT(4)
#define   AUX_LEN_REQ			GENMASK(3, 0)

#define DW_DP_AUX_STATUS		0x0b04
#define   AUX_STATUS_BUSY		BIT(16)
#define   AUX_STATUS_TIMEOUT		BIT(17)
#define   AUX_STATUS_BYTES_READ		GENMASK(23, 19)
#define   AUX_STATUS_REPLY_CODE		GENMASK(7, 4)
#define     AUX_REPLY_ACK		0
#define     AUX_REPLY_NACK		1
#define     AUX_REPLY_DEFER		2

#define DW_DP_AUX_DATA0			0x0b08

/* PERI_CRG PPLL7 for DP pixel clock */
#define PERI_CRG_PPLL7_CTRL0		0x530
#define PERI_CRG_PPLL7_CTRL1		0x534

/* DSS Secondary Display Pipe (SDP) control */
#define DSS_SDP_PIPE_SW_EN		0x7e024
#define DSS_SDP_LDI_EN			0x7f024

struct kirin_dp {
	struct kirin_dss *dss;
	void __iomem *base;
	void __iomem *phy_ctrl;
	struct clk *clk_16m;
	struct clk *pclk;
	struct clk *aclk;
	struct clk *clk_ldi1;
	struct gpio_desc *hpd_gpio;
	int hpd_irq;
	struct delayed_work hpd_work;
	struct drm_dp_aux aux;
	struct drm_encoder encoder;
	struct drm_connector connector;
	const struct drm_edid *edid;
	bool connected;
	bool powered;
	bool enabled;
};

#define to_kirin_dp_connector(x) container_of(x, struct kirin_dp, connector)
#define to_kirin_dp_encoder(x)   container_of(x, struct kirin_dp, encoder)

struct ppll7_div {
	u8 total;
	u8 postdiv1;
	u8 postdiv2;
};

/* Kirin 990 PPLL7 postdiv table for video pixel clocks */
static const struct ppll7_div ppll7_div_table[] = {
	{  1, 1, 1 }, {  2, 2, 1 }, {  3, 3, 1 }, {  4, 4, 1 },
	{  5, 5, 1 }, {  6, 6, 1 }, {  7, 7, 1 }, {  8, 4, 2 },
	{  9, 3, 3 }, { 10, 5, 2 }, { 12, 4, 3 }, { 14, 7, 2 },
	{ 15, 5, 3 }, { 16, 4, 4 }, { 20, 5, 4 }, { 21, 7, 3 },
	{ 24, 6, 4 }, { 25, 5, 5 }, { 30, 6, 5 }, { 36, 6, 6 },
	{ 42, 7, 6 }, { 49, 7, 7 },
};

static void kirin_dp_set_pixel_clock(struct kirin_dp *dp, unsigned long rate_hz)
{
	struct kirin_dss *k = dp->dss;
	u64 vco_target;
	const struct ppll7_div *best = &ppll7_div_table[0];
	u32 fbkdiv, frac, ctrl0, ctrl1, rem;
	int i;

	if (!k->peri_crg || !rate_hz)
		return;

	/* Select divider to keep VCO in [800 MHz, 1600 MHz] */
	for (i = 0; i < ARRAY_SIZE(ppll7_div_table); i++) {
		u64 vco = (u64)rate_hz * ppll7_div_table[i].total;
		if (vco >= 800000000ULL && vco <= 1600000000ULL) {
			best = &ppll7_div_table[i];
			break;
		}
	}

	vco_target = (u64)rate_hz * best->total;
	fbkdiv = div_u64_rem(vco_target, 19200000, &rem);
	frac = (u32)div_u64((u64)rem << 24, 19200000);

	ctrl0 = (fbkdiv << 8) | (best->postdiv1 << 20) | (best->postdiv2 << 23);
	ctrl1 = (frac & 0xffffff) | BIT(25);

	writel(ctrl0, k->peri_crg + PERI_CRG_PPLL7_CTRL0);
	writel(ctrl1, k->peri_crg + PERI_CRG_PPLL7_CTRL1);
}

static void kirin_dp_aux_write_data(struct kirin_dp *dp, const u8 *buffer, size_t size)
{
	size_t i, j;

	for (i = 0; i < DIV_ROUND_UP(size, 4); i++) {
		size_t num = min_t(size_t, size - i * 4, 4);
		u32 val = 0;

		for (j = 0; j < num; j++)
			val |= (u32)buffer[i * 4 + j] << (j * 8);

		writel(val, dp->base + DW_DP_AUX_DATA0 + i * 4);
	}
}

static void kirin_dp_aux_read_data(struct kirin_dp *dp, u8 *buffer, size_t size)
{
	size_t i, j;

	for (i = 0; i < DIV_ROUND_UP(size, 4); i++) {
		size_t num = min_t(size_t, size - i * 4, 4);
		u32 val = readl(dp->base + DW_DP_AUX_DATA0 + i * 4);

		for (j = 0; j < num; j++)
			buffer[i * 4 + j] = (val >> (j * 8)) & 0xff;
	}
}

static ssize_t kirin_dp_aux_transfer(struct drm_dp_aux *aux, struct drm_dp_aux_msg *msg)
{
	struct kirin_dp *dp = container_of(aux, struct kirin_dp, aux);
	u32 cmd, status, reply;
	int ret;

	if (WARN_ON(msg->size > 16))
		return -E2BIG;

	/* Write payload into AUX_DATA registers if write request */
	switch (msg->request & ~DP_AUX_I2C_MOT) {
	case DP_AUX_NATIVE_WRITE:
	case DP_AUX_I2C_WRITE:
	case DP_AUX_I2C_WRITE_STATUS_UPDATE:
		kirin_dp_aux_write_data(dp, msg->buffer, msg->size);
		break;
	case DP_AUX_NATIVE_READ:
	case DP_AUX_I2C_READ:
		break;
	default:
		return -EINVAL;
	}

	/* Build AUX command word according to DWC DP TX hardware spec */
	cmd = FIELD_PREP(AUX_CMD_TYPE, msg->request);
	cmd |= FIELD_PREP(AUX_ADDR, msg->address);
	if (msg->size > 0)
		cmd |= FIELD_PREP(AUX_LEN_REQ, msg->size - 1);
	else
		cmd |= AUX_I2C_ADDR_ONLY;

	writel(cmd, dp->base + DW_DP_AUX_CMD);

	/* Wait for completion (BUSY bit 16 clears when done) */
	ret = readl_poll_timeout(dp->base + DW_DP_AUX_STATUS, status,
				 !(status & AUX_STATUS_BUSY), 20, 25000);
	if (ret || (status & AUX_STATUS_TIMEOUT))
		return -ETIMEDOUT;

	reply = FIELD_GET(AUX_STATUS_REPLY_CODE, status);
	switch (reply) {
	case AUX_REPLY_ACK:
		msg->reply = DP_AUX_NATIVE_REPLY_ACK;
		break;
	case AUX_REPLY_NACK:
		msg->reply = DP_AUX_NATIVE_REPLY_NACK;
		return 0;
	case AUX_REPLY_DEFER:
		msg->reply = DP_AUX_NATIVE_REPLY_DEFER;
		return 0;
	default:
		msg->reply = DP_AUX_NATIVE_REPLY_NACK;
		return 0;
	}

	/* Read payload from AUX_DATA registers if read request */
	if (msg->size > 0) {
		switch (msg->request & ~DP_AUX_I2C_MOT) {
		case DP_AUX_NATIVE_READ:
		case DP_AUX_I2C_READ:
			kirin_dp_aux_read_data(dp, msg->buffer, msg->size);
			break;
		default:
			break;
		}
	}

	return msg->size;
}

static void kirin_dp_power_on(struct kirin_dp *dp)
{
	u32 val;

	if (dp->powered)
		return;

	/* Power on RTD2172 bridge rails via EC */
	huawei_echub_hdmi_power(true);

	/* Enable controller and SDP clocks */
	clk_prepare_enable(dp->clk_16m);
	clk_prepare_enable(dp->pclk);
	clk_prepare_enable(dp->aclk);
	clk_prepare_enable(dp->clk_ldi1);

	/* Soft reset DP core */
	writel(RESET_ALL, dp->base + DW_DP_SOFT_RESET_CTRL);
	usleep_range(20, 50);
	writel(0, dp->base + DW_DP_SOFT_RESET_CTRL);

	/* Take PHY out of power down */
	val = readl(dp->base + DW_DP_PHY_CONFIG);
	writel(val & ~BIT(25), dp->base + DW_DP_PHY_CONFIG);

	dp->powered = true;
}

static void kirin_dp_power_off(struct kirin_dp *dp)
{
	if (!dp->powered || dp->enabled)
		return;

	clk_disable_unprepare(dp->clk_ldi1);
	clk_disable_unprepare(dp->aclk);
	clk_disable_unprepare(dp->pclk);
	clk_disable_unprepare(dp->clk_16m);

	huawei_echub_hdmi_power(false);
	dp->powered = false;
}

static void kirin_dp_hpd_work(struct work_struct *work)
{
	struct kirin_dp *dp = container_of(to_delayed_work(work), struct kirin_dp, hpd_work);
	bool hpd = false;

	if (dp->hpd_gpio)
		hpd = gpiod_get_value_cansleep(dp->hpd_gpio) > 0;

	if (hpd) {
		kirin_dp_power_on(dp);
		dp->connected = true;
	} else {
		dp->connected = false;
		kirin_dp_power_off(dp);
	}

	drm_connector_hdmi_audio_plugged_notify(&dp->connector, dp->connected);
	drm_helper_hpd_irq_event(&dp->dss->drm);
}

static irqreturn_t kirin_dp_hpd_irq_handler(int irq, void *data)
{
	struct kirin_dp *dp = data;

	/* Debounce for 50 ms */
	mod_delayed_work(system_wq, &dp->hpd_work, msecs_to_jiffies(50));
	return IRQ_HANDLED;
}

static enum drm_connector_status kirin_dp_connector_detect(struct drm_connector *connector,
							   bool force)
{
	struct kirin_dp *dp = to_kirin_dp_connector(connector);

	if (dp->hpd_gpio) {
		int val = gpiod_get_value_cansleep(dp->hpd_gpio);
		if (val <= 0) {
			dp->connected = false;
			kirin_dp_power_off(dp);
			return connector_status_disconnected;
		}
	}

	kirin_dp_power_on(dp);
	dp->connected = true;
	return connector_status_connected;
}

static int kirin_dp_connector_get_modes(struct drm_connector *connector)
{
	struct kirin_dp *dp = to_kirin_dp_connector(connector);
	int count = 0;

	if (!dp->connected)
		return 0;

	kirin_dp_power_on(dp);

	if (dp->edid) {
		drm_edid_free(dp->edid);
		dp->edid = NULL;
	}

	/* Read EDID through AUX channel (I2C-over-AUX to RTD2172) */
	dp->edid = drm_edid_read_ddc(connector, &dp->aux.ddc);
	if (dp->edid) {
		drm_edid_connector_update(connector, dp->edid);
		count = drm_edid_connector_add_modes(connector);
	}

	if (!count)
		count = drm_add_modes_noedid(connector, 1920, 1080);

	return count;
}

static const struct drm_connector_funcs kirin_dp_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.detect = kirin_dp_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int kirin_dp_audio_prepare(struct drm_connector *connector,
				  struct hdmi_codec_daifmt *fmt,
				  struct hdmi_codec_params *hparms)
{
	struct kirin_dp *dp = to_kirin_dp_connector(connector);
	u32 conf = 0, ch_mask, sf = 0, if_data;

	if (!dp->powered || !dp->enabled)
		return -ENODEV;

	/* Soft reset audio engine */
	writel(RESET_AUDIO, dp->base + DW_DP_SOFT_RESET_CTRL);
	udelay(10);
	writel(0, dp->base + DW_DP_SOFT_RESET_CTRL);

	if (fmt->fmt == HDMI_SPDIF)
		conf |= AUDIO_CONFIG_SPDIF;

	switch (hparms->channels) {
	case 1 ... 2:
		ch_mask = 0x1;
		break;
	case 3 ... 4:
		ch_mask = 0x3;
		break;
	case 5 ... 6:
		ch_mask = 0x7;
		break;
	case 7 ... 8:
	default:
		ch_mask = 0xf;
		break;
	}
	conf |= (ch_mask << 1);
	conf |= ((hparms->sample_width & 0x1f) << 5);
	conf |= (((hparms->channels - 1) & 0x7) << 12);
	conf |= (0x11 << 24); /* Core I2S + FIFO watermark */

	/* Write audio config (unmuted: bit 15 is 0) */
	writel(conf, dp->base + DW_DP_AUDIO_CONFIG);

	switch (hparms->sample_rate) {
	case 32000:  sf = 1; break;
	case 44100:  sf = 2; break;
	case 48000:  sf = 3; break;
	case 88200:  sf = 4; break;
	case 96000:  sf = 5; break;
	case 176400: sf = 6; break;
	case 192000: sf = 7; break;
	default:     sf = 0; break;
	}

	if_data = ((hparms->channels - 1) & 0x7) |
		  (sf << 10) |
		  (hparms->cea.channel_allocation << 16);

	writel(0x441b8400, dp->base + DW_DP_AUDIO_INFOFRAME_HEADER);
	writel(if_data, dp->base + DW_DP_AUDIO_INFOFRAME_DATA0);
	writel(0, dp->base + DW_DP_AUDIO_INFOFRAME_DATA1);
	writel(0, dp->base + DW_DP_AUDIO_INFOFRAME_DATA2);

	/* Enable Audio Timestamp SDP, Audio Stream SDP, Audio InfoFrame SDP */
	writel(readl(dp->base + DW_DP_SDP_CONFIG) |
	       SDP_TIMESTAMP_EN | SDP_AUDIO_STREAM_EN | SDP_INFOFRAME_EN,
	       dp->base + DW_DP_SDP_CONFIG);
	writel(readl(dp->base + DW_DP_SDP_CONFIG2) | SDP2_AUDIO_STREAM_EN,
	       dp->base + DW_DP_SDP_CONFIG2);

	drm_info(&dp->dss->drm, "DP audio configured: %d Hz, %d ch, %d bits (if_data=0x%08x)\n",
		 hparms->sample_rate, hparms->channels, hparms->sample_width, if_data);
	return 0;
}

static void kirin_dp_audio_shutdown(struct drm_connector *connector)
{
	struct kirin_dp *dp = to_kirin_dp_connector(connector);

	if (!dp->base)
		return;

	writel(readl(dp->base + DW_DP_SDP_CONFIG) &
	       ~(SDP_TIMESTAMP_EN | SDP_AUDIO_STREAM_EN | SDP_INFOFRAME_EN),
	       dp->base + DW_DP_SDP_CONFIG);
	writel(readl(dp->base + DW_DP_SDP_CONFIG2) & ~SDP2_AUDIO_STREAM_EN,
	       dp->base + DW_DP_SDP_CONFIG2);
	writel(readl(dp->base + DW_DP_AUDIO_CONFIG) | AUDIO_CONFIG_MUTE,
	       dp->base + DW_DP_AUDIO_CONFIG);

	writel(RESET_AUDIO, dp->base + DW_DP_SOFT_RESET_CTRL);
	udelay(10);
	writel(0, dp->base + DW_DP_SOFT_RESET_CTRL);
	drm_info(&dp->dss->drm, "DP audio shutdown\n");
}

static int kirin_dp_audio_mute_stream(struct drm_connector *connector,
				      bool enable, int direction)
{
	struct kirin_dp *dp = to_kirin_dp_connector(connector);
	u32 val;

	if (!dp->base)
		return 0;

	val = readl(dp->base + DW_DP_AUDIO_CONFIG);
	if (enable)
		val |= AUDIO_CONFIG_MUTE;
	else
		val &= ~AUDIO_CONFIG_MUTE;
	writel(val, dp->base + DW_DP_AUDIO_CONFIG);
	return 0;
}

static const struct drm_connector_hdmi_audio_funcs kirin_dp_audio_funcs = {
	.prepare = kirin_dp_audio_prepare,
	.shutdown = kirin_dp_audio_shutdown,
	.mute_stream = kirin_dp_audio_mute_stream,
};

static const struct drm_connector_helper_funcs kirin_dp_connector_helper_funcs = {
	.get_modes = kirin_dp_connector_get_modes,
};

static void kirin_dp_encoder_enable(struct drm_encoder *encoder,
				    struct drm_atomic_state *state)
{
	struct kirin_dp *dp = to_kirin_dp_encoder(encoder);
	struct drm_crtc *crtc = encoder->crtc;
	const struct drm_display_mode *mode;
	u32 hactive, hblank, vactive, vblank;

	if (!crtc || !crtc->state)
		return;

	mode = &crtc->state->adjusted_mode;

	kirin_dp_power_on(dp);
	dp->enabled = true;

	/* Configure pixel clock in PPLL7 */
	kirin_dp_set_pixel_clock(dp, mode->clock * 1000UL);

	/* Enable SDP Pipe switch and LDI in DSS */
	dss_rmw(dp->dss, DSS_SDP_PIPE_SW_EN, BIT(0), BIT(0));
	dss_rmw(dp->dss, DSS_SDP_LDI_EN, BIT(0), BIT(0));

	/* Configure 2 lanes at 2.7 Gbps (HBR) */
	writel(BW_2700MBPS, dp->base + DW_DP_LINK_BW_SET);
	writel(LANES_2, dp->base + DW_DP_LANE_COUNT_SET);

	/* Configure video timings */
	hactive = mode->hdisplay;
	hblank = mode->htotal - mode->hdisplay;
	vactive = mode->vdisplay;
	vblank = mode->vtotal - mode->vdisplay;

	writel((hactive << 16) | (hblank & 0xffff), dp->base + DW_DP_VIDEO_CONFIG1);
	writel((vblank << 16) | (vactive & 0xffff), dp->base + DW_DP_VIDEO_CONFIG2);
	writel(((mode->hsync_end - mode->hsync_start) << 16) |
	       ((mode->hsync_start - mode->hdisplay) & 0xffff),
	       dp->base + DW_DP_VIDEO_CONFIG3);
	writel(((mode->vsync_end - mode->vsync_start) << 16) |
	       ((mode->vsync_start - mode->vdisplay) & 0xffff),
	       dp->base + DW_DP_VIDEO_CONFIG4);

	/* Enable video stream in 24bpp RGB */
	writel(VSAMPLE_VIDEO_STREAM_EN, dp->base + DW_DP_VSAMPLE_CTRL);
	drm_info(&dp->dss->drm, "DP/HDMI enabled: %ux%u@%uHz\n",
		 mode->hdisplay, mode->vdisplay, drm_mode_vrefresh(mode));
}

static void kirin_dp_encoder_disable(struct drm_encoder *encoder,
				     struct drm_atomic_state *state)
{
	struct kirin_dp *dp = to_kirin_dp_encoder(encoder);

	/* Stop video stream */
	writel(0, dp->base + DW_DP_VSAMPLE_CTRL);

	/* Disable SDP LDI in DSS */
	dss_rmw(dp->dss, DSS_SDP_PIPE_SW_EN, BIT(0), 0);
	dss_rmw(dp->dss, DSS_SDP_LDI_EN, BIT(0), 0);

	dp->enabled = false;
	kirin_dp_power_off(dp);
	drm_info(&dp->dss->drm, "DP/HDMI disabled\n");
}

static const struct drm_encoder_funcs kirin_dp_encoder_funcs = {
};

static const struct drm_encoder_helper_funcs kirin_dp_encoder_helper_funcs = {
	.atomic_enable = kirin_dp_encoder_enable,
	.atomic_disable = kirin_dp_encoder_disable,
};

int kirin_dp_init(struct kirin_dss *k)
{
	struct device *dev = k->drm.dev;
	struct kirin_dp *dp;
	int ret;

	dp = drmm_kzalloc(&k->drm, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	dp->dss = k;
	dp->base = devm_ioremap(dev, 0xf8500000UL, 0x1000);
	if (!dp->base)
		return -ENOMEM;

	dp->phy_ctrl = devm_ioremap(dev, 0xf8481000UL, 0x1000);

	dp->clk_16m = devm_clk_get_optional(dev, "clk_dpctrl_16m");
	dp->pclk = devm_clk_get_optional(dev, "pclk_dpctrl");
	dp->aclk = devm_clk_get_optional(dev, "aclk_dpctrl");
	dp->clk_ldi1 = devm_clk_get_optional(dev, "clk_ldi1");

	/* Hotplug detection GPIO (GPIO 282) */
	dp->hpd_gpio = devm_gpiod_get_optional(dev, "hpd", GPIOD_IN);
	if (IS_ERR(dp->hpd_gpio))
		dp->hpd_gpio = NULL;

	INIT_DELAYED_WORK(&dp->hpd_work, kirin_dp_hpd_work);

	/* AUX channel setup */
	dp->aux.name = "kirin-dp-aux";
	dp->aux.dev = dev;
	dp->aux.drm_dev = &k->drm;
	dp->aux.transfer = kirin_dp_aux_transfer;
	ret = drm_dp_aux_register(&dp->aux);
	if (ret) {
		drm_warn(&k->drm, "failed to register DP AUX: %d\n", ret);
		return ret;
	}

	/* DRM Encoder */
	ret = drmm_encoder_init(&k->drm, &dp->encoder, &kirin_dp_encoder_funcs,
				DRM_MODE_ENCODER_TMDS, "kirin-dp");
	if (ret)
		goto err_aux;

	dp->encoder.possible_crtcs = drm_crtc_mask(&k->crtc);
	drm_encoder_helper_add(&dp->encoder, &kirin_dp_encoder_helper_funcs);

	/* DRM Connector */
	ret = drmm_connector_init(&k->drm, &dp->connector, &kirin_dp_connector_funcs,
				  DRM_MODE_CONNECTOR_HDMIA, &dp->aux.ddc);
	if (ret)
		goto err_aux;

	drm_connector_helper_add(&dp->connector, &kirin_dp_connector_helper_funcs);
	dp->connector.polled = DRM_CONNECTOR_POLL_HPD;
	ret = drm_connector_attach_encoder(&dp->connector, &dp->encoder);
	if (ret)
		goto err_aux;

	/* Register HDMI/DP audio codec */
	ret = drm_connector_hdmi_audio_init(&dp->connector, dev,
					    &kirin_dp_audio_funcs, 8, 0, false, -1);
	if (ret)
		drm_warn(&k->drm, "failed to initialize HDMI audio codec: %d\n", ret);
	else
		drm_connector_hdmi_audio_plugged_notify(&dp->connector, dp->connected);

	/* Setup HPD IRQ */
	if (dp->hpd_gpio) {
		dp->hpd_irq = gpiod_to_irq(dp->hpd_gpio);
		if (dp->hpd_irq > 0) {
			ret = devm_request_threaded_irq(dev, dp->hpd_irq, NULL,
							kirin_dp_hpd_irq_handler,
							IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
							IRQF_ONESHOT,
							"kirin-dp-hpd", dp);
			if (ret)
				drm_warn(&k->drm, "failed to request HPD IRQ: %d\n", ret);
		}
	}

	k->dp = dp;
	drm_info(&k->drm, "DP/HDMI output initialized (HPD GPIO %s)\n",
		 dp->hpd_gpio ? "present" : "absent");
	return 0;

err_aux:
	drm_dp_aux_unregister(&dp->aux);
	return ret;
}

void kirin_dp_remove(struct kirin_dss *k)
{
	struct kirin_dp *dp = k->dp;

	if (!dp)
		return;

	cancel_delayed_work_sync(&dp->hpd_work);
	if (dp->connector.hdmi_audio.codec_pdev) {
		platform_device_unregister(dp->connector.hdmi_audio.codec_pdev);
		dp->connector.hdmi_audio.codec_pdev = NULL;
	}
	drm_dp_aux_unregister(&dp->aux);
	kirin_dp_power_off(dp);
	if (dp->edid) {
		drm_edid_free(dp->edid);
		dp->edid = NULL;
	}
	k->dp = NULL;
}
