// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei L410 DisplayPort / HDMI audio machine driver.
 *
 * Connects the dummy CPU DAI / platform with the hdmi-audio-codec registered
 * by the Kirin 990 DisplayPort driver, exposing the digital HDMI audio card.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "hi6405_drv.h"

struct hisi_dp_card_priv {
	struct snd_soc_card card;
	struct snd_soc_dai_link dai_link;
	struct snd_soc_dai_link_component codec_comp;
	struct snd_soc_jack hdmi_jack;
	struct snd_soc_jack_pin hdmi_jack_pin;
};

static int hisi_dp_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_dai *codec_dai = snd_soc_rtd_to_codec(rtd, 0);
	struct snd_soc_component *component = codec_dai->component;
	struct hisi_dp_card_priv *priv = snd_soc_card_get_drvdata(card);
	int ret;

	priv->hdmi_jack_pin.pin = "HDMI Jack";
	priv->hdmi_jack_pin.mask = SND_JACK_LINEOUT;

	ret = snd_soc_card_jack_new_pins(card, "HDMI Jack", SND_JACK_LINEOUT,
					 &priv->hdmi_jack,
					 &priv->hdmi_jack_pin, 1);
	if (ret)
		return ret;

	ret = snd_soc_component_set_jack(component, &priv->hdmi_jack, NULL);
	if (ret)
		dev_warn(card->dev, "failed to set HDMI jack: %d\n", ret);

	return 0;
}

static const struct snd_soc_dapm_widget hisi_dp_widgets[] = {
	SND_SOC_DAPM_LINE("HDMI Jack", NULL),
};

static const struct snd_soc_dapm_route hisi_dp_routes[] = {
	{ "HDMI Jack", NULL, "TX" },
};

static int match_hdmi_codec(struct device *dev, const void *data)
{
	return dev->driver && !strncmp(dev_name(dev), "hdmi-audio-codec", 16);
}

static int hisi_dp_card_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device *codec_dev;
	struct hisi_dp_card_priv *priv;
	int ret;

	codec_dev = bus_find_device(&platform_bus_type, NULL, NULL, match_hdmi_codec);
	if (!codec_dev)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		put_device(codec_dev);
		return -ENOMEM;
	}

	priv->codec_comp.name = devm_kstrdup(dev, dev_name(codec_dev), GFP_KERNEL);
	priv->codec_comp.dai_name = "i2s-hifi";
	put_device(codec_dev);

	if (!priv->codec_comp.name)
		return -ENOMEM;

	priv->dai_link.name = "HDMI";
	priv->dai_link.stream_name = "HDMI Audio";
	priv->dai_link.cpus = &snd_soc_dummy_dlc;
	priv->dai_link.num_cpus = 1;
	priv->dai_link.codecs = &priv->codec_comp;
	priv->dai_link.num_codecs = 1;
	priv->dai_link.platforms = &snd_soc_dummy_dlc;
	priv->dai_link.num_platforms = 1;
	priv->dai_link.init = hisi_dp_dai_init;

	priv->card.dev = dev;
	priv->card.owner = THIS_MODULE;
	priv->card.name = "hisi-hdmi";
	priv->card.driver_name = "hisi-hdmi";
	priv->card.dai_link = &priv->dai_link;
	priv->card.num_links = 1;
	priv->card.dapm_widgets = hisi_dp_widgets;
	priv->card.num_dapm_widgets = ARRAY_SIZE(hisi_dp_widgets);
	priv->card.dapm_routes = hisi_dp_routes;
	priv->card.num_dapm_routes = ARRAY_SIZE(hisi_dp_routes);

	of_property_read_string(dev->of_node, "card_name", &priv->card.name);
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = devm_snd_soc_register_card(dev, &priv->card);
	if (ret)
		return dev_err_probe(dev, ret, "card registration failed\n");

	dev_info(dev, "DP/HDMI audio card '%s' registered\n", priv->card.name);
	return 0;
}

static const struct of_device_id hisi_dp_card_match[] = {
	{ .compatible = "hisilicon,hisi-dp-audio-machine", },
	{ }
};
MODULE_DEVICE_TABLE(of, hisi_dp_card_match);

struct platform_driver hisi_dp_card_driver = {
	.driver = {
		.name = "hisi-dp-card",
		.of_match_table = hisi_dp_card_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = hisi_dp_card_probe,
};
