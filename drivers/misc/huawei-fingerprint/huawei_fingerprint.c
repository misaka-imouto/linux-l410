// SPDX-License-Identifier: GPL-2.0
/*
 * Huawei Qingyun L410 / Kirin 990 Fingerprint Sensor Bridge Driver
 *
 * Provides the Linux kernel bridge for the Fingerprint Cards (FPC) sensor
 * on Huawei Qingyun L410 (KLVU-WDU0B) / Kirin 990.
 *
 * Copyright (c) 2026 Antigravity Authors
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/atomic.h>
#include <linux/mutex.h>

#include "huawei_fingerprint.h"

#define DRIVER_NAME "fingerprint"
#define DEFAULT_RESET_GPIO 199
#define DEFAULT_IRQ_GPIO   234
#define DEFAULT_CS_GPIO    10
#define DEFAULT_SPI_NUM    1
#define DEFAULT_MODULE_ID  "23100501TC38A1109091205JY5332473639302E008A866AA431D1180E"

struct fingerprint_data {
	struct device *dev;
	struct cdev cdev;
	dev_t devno;
	struct class *class;
	struct device *cdev_device;

	struct gpio_desc *gpiod_reset;
	struct gpio_desc *gpiod_irq;
	int reset_gpio;
	int irq_gpio;
	int cs_gpio;
	int spi_num;
	int sensor_type;
	int irq;

	bool irq_enabled;
	atomic_t irq_count;
	wait_queue_head_t waitq;
	struct mutex lock;

	char module_id[128];
	char chip_info[32];
};

static struct fingerprint_data *g_fp_data;

static irqreturn_t fingerprint_irq_thread(int irq, void *dev_id)
{
	struct fingerprint_data *fp = dev_id;

	atomic_inc(&fp->irq_count);
	wake_up_interruptible(&fp->waitq);
	sysfs_notify(&fp->dev->kobj, NULL, "irq");

	return IRQ_HANDLED;
}

static int fingerprint_open(struct inode *inode, struct file *file)
{
	struct fingerprint_data *fp;

	fp = container_of(inode->i_cdev, struct fingerprint_data, cdev);
	file->private_data = fp;

	return 0;
}

static int fingerprint_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t fingerprint_read(struct file *file, char __user *buf,
				size_t count, loff_t *ppos)
{
	return 0;
}

static __poll_t fingerprint_poll(struct file *file, struct poll_table_struct *wait)
{
	struct fingerprint_data *fp = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &fp->waitq, wait);

	if (atomic_read(&fp->irq_count) > 0) {
		atomic_set(&fp->irq_count, 0);
		mask |= EPOLLIN | EPOLLRDNORM;
	}

	return mask;
}

static long fingerprint_ioctl_internal(struct fingerprint_data *fp,
				       unsigned int cmd, unsigned long arg)
{
	int val = 0;
	struct fp_device_info dev_info;

	switch (cmd) {
	case FP_IOC_CMD_ENABLE_IRQ:
		mutex_lock(&fp->lock);
		if (!fp->irq_enabled) {
			enable_irq(fp->irq);
			fp->irq_enabled = true;
		}
		mutex_unlock(&fp->lock);
		return 0;

	case FP_IOC_CMD_DISABLE_IRQ:
		mutex_lock(&fp->lock);
		if (fp->irq_enabled) {
			disable_irq(fp->irq);
			fp->irq_enabled = false;
		}
		mutex_unlock(&fp->lock);
		return 0;

	case FP_IOC_CMD_SET_IPC_WAKELOCKS:
	case FP_IOC_CMD_SEND_UEVENT:
	case FP_IOC_CMD_SEND_SENSORID:
	case FP_IOC_CMD_SEND_SENSORID_UD:
		return 0;

	case FP_IOC_CMD_GET_IRQ_STATUS:
		val = gpiod_get_value(fp->gpiod_irq);
		if (copy_to_user((void __user *)arg, &val, sizeof(val)))
			return -EFAULT;
		return 0;

	case FP_IOC_CMD_SET_POWEROFF:
		mutex_lock(&fp->lock);
		gpiod_set_value(fp->gpiod_reset, 0);
		mutex_unlock(&fp->lock);
		return 0;

	case FP_IOC_CMD_SET_POWERON:
		mutex_lock(&fp->lock);
		gpiod_set_value(fp->gpiod_reset, 1);
		usleep_range(5000, 6000);
		mutex_unlock(&fp->lock);
		return 0;

	case FP_IOC_CMD_GET_DEVICE_INFO:
		memset(&dev_info, 0, sizeof(dev_info));
		dev_info.reset_pin = fp->reset_gpio;
		dev_info.irq_pin = fp->irq_gpio;
		dev_info.spi_cs = fp->cs_gpio;
		dev_info.spi_num = fp->spi_num;
		if (copy_to_user((void __user *)arg, &dev_info, sizeof(dev_info)))
			return -EFAULT;
		return 0;

	default:
		dev_dbg(fp->dev, "unsupported ioctl cmd 0x%x\n", cmd);
		return -ENOTTY;
	}
}

static long fingerprint_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct fingerprint_data *fp = file->private_data;

	return fingerprint_ioctl_internal(fp, cmd, arg);
}

static long fingerprint_compat_ioctl(struct file *file, unsigned int cmd,
				     unsigned long arg)
{
	struct fingerprint_data *fp = file->private_data;

	return fingerprint_ioctl_internal(fp, cmd, arg);
}

static const struct file_operations fingerprint_fops = {
	.owner          = THIS_MODULE,
	.open           = fingerprint_open,
	.release        = fingerprint_release,
	.read           = fingerprint_read,
	.poll           = fingerprint_poll,
	.unlocked_ioctl = fingerprint_ioctl,
	.compat_ioctl   = fingerprint_compat_ioctl,
};

static ssize_t irq_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct fingerprint_data *fp = dev_get_drvdata(dev);
	int val = gpiod_get_value(fp->gpiod_irq);

	return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RO(irq);

static ssize_t irq_enabled_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct fingerprint_data *fp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", fp->irq_enabled ? 1 : 0);
}

static ssize_t irq_enabled_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct fingerprint_data *fp = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&fp->lock);
	if (val && !fp->irq_enabled) {
		enable_irq(fp->irq);
		fp->irq_enabled = true;
	} else if (!val && fp->irq_enabled) {
		disable_irq(fp->irq);
		fp->irq_enabled = false;
	}
	mutex_unlock(&fp->lock);

	return count;
}
static DEVICE_ATTR_RW(irq_enabled);

static ssize_t fingerprint_chip_info_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct fingerprint_data *fp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", fp->chip_info);
}
static DEVICE_ATTR_RO(fingerprint_chip_info);

static ssize_t module_id_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct fingerprint_data *fp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n", fp->module_id);
}
static DEVICE_ATTR_RO(module_id);

static ssize_t nav_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	return sysfs_emit(buf, "0\n");
}
static DEVICE_ATTR_RO(nav);

static ssize_t result_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	return sysfs_emit(buf, "0\n");
}
static DEVICE_ATTR_RO(result);

static ssize_t read_image_flag_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "0\n");
}
static DEVICE_ATTR_RO(read_image_flag);

static ssize_t low_temperature_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "0\n");
}
static DEVICE_ATTR_RO(low_temperature);

static ssize_t test_power_en_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "1\n");
}
static DEVICE_ATTR_RO(test_power_en);

static struct attribute *fingerprint_attrs[] = {
	&dev_attr_irq.attr,
	&dev_attr_irq_enabled.attr,
	&dev_attr_fingerprint_chip_info.attr,
	&dev_attr_module_id.attr,
	&dev_attr_nav.attr,
	&dev_attr_result.attr,
	&dev_attr_read_image_flag.attr,
	&dev_attr_low_temperature.attr,
	&dev_attr_test_power_en.attr,
	NULL,
};
ATTRIBUTE_GROUPS(fingerprint);

static int fingerprint_parse_dt(struct platform_device *pdev,
				struct fingerprint_data *fp)
{
	struct device_node *np = pdev->dev.of_node;
	u32 val;

	if (!np)
		return -ENODEV;

	if (!of_property_read_u32(np, "fingerprint,reset_gpio", &val))
		fp->reset_gpio = val;
	else
		fp->reset_gpio = DEFAULT_RESET_GPIO;

	if (!of_property_read_u32(np, "fingerprint,irq_gpio", &val))
		fp->irq_gpio = val;
	else
		fp->irq_gpio = DEFAULT_IRQ_GPIO;

	if (!of_property_read_u32(np, "fingerprint,cs_gpio", &val))
		fp->cs_gpio = val;
	else
		fp->cs_gpio = DEFAULT_CS_GPIO;

	if (!of_property_read_u32(np, "fingerprint,spi_num", &val))
		fp->spi_num = val;
	else
		fp->spi_num = DEFAULT_SPI_NUM;

	if (!of_property_read_u32(np, "fingerprint,sensor_type", &val))
		fp->sensor_type = val;
	else
		fp->sensor_type = 0x204;

	return 0;
}

static int fingerprint_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fingerprint_data *fp;
	int ret;

	fp = devm_kzalloc(dev, sizeof(*fp), GFP_KERNEL);
	if (!fp)
		return -ENOMEM;

	fp->dev = dev;
	mutex_init(&fp->lock);
	init_waitqueue_head(&fp->waitq);
	atomic_set(&fp->irq_count, 0);
	snprintf(fp->chip_info, sizeof(fp->chip_info), "ff--NN");
	snprintf(fp->module_id, sizeof(fp->module_id), "%s", DEFAULT_MODULE_ID);

	ret = fingerprint_parse_dt(pdev, fp);
	if (ret)
		return ret;

	fp->gpiod_reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(fp->gpiod_reset)) {
		ret = PTR_ERR(fp->gpiod_reset);
		if (ret == -EPROBE_DEFER)
			return ret;
		fp->gpiod_reset = NULL;
	}
	if (!fp->gpiod_reset && gpio_is_valid(fp->reset_gpio)) {
		ret = devm_gpio_request_one(dev, fp->reset_gpio,
					    GPIOF_OUT_INIT_LOW,
					    "fingerprint,reset_gpio");
		if (ret == 0) {
			fp->gpiod_reset = gpio_to_desc(fp->reset_gpio);
		} else {
			dev_warn(dev, "failed to request reset gpio %d: %d\n",
				 fp->reset_gpio, ret);
		}
	}

	fp->gpiod_irq = devm_gpiod_get_optional(dev, "irq", GPIOD_IN);
	if (IS_ERR(fp->gpiod_irq)) {
		ret = PTR_ERR(fp->gpiod_irq);
		if (ret == -EPROBE_DEFER)
			return ret;
		fp->gpiod_irq = NULL;
	}
	if (!fp->gpiod_irq && gpio_is_valid(fp->irq_gpio)) {
		ret = devm_gpio_request_one(dev, fp->irq_gpio,
					    GPIOF_IN,
					    "fingerprint,irq_gpio");
		if (ret == 0) {
			fp->gpiod_irq = gpio_to_desc(fp->irq_gpio);
		} else {
			dev_warn(dev, "failed to request irq gpio %d: %d\n",
				 fp->irq_gpio, ret);
		}
	}

	if (fp->gpiod_irq)
		fp->irq = gpiod_to_irq(fp->gpiod_irq);
	else if (gpio_is_valid(fp->irq_gpio))
		fp->irq = gpio_to_irq(fp->irq_gpio);
	else
		fp->irq = -ENODEV;

	if (fp->irq == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (fp->irq < 0) {
		dev_err(dev, "failed to map irq for gpio %d: %d\n", fp->irq_gpio, fp->irq);
		return fp->irq;
	}

	ret = devm_request_threaded_irq(dev, fp->irq, NULL,
					fingerprint_irq_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					"fingerprint", fp);
	if (ret) {
		dev_err(dev, "failed to request threaded irq %d: %d\n",
			fp->irq, ret);
		return ret;
	}
	fp->irq_enabled = true;

	ret = alloc_chrdev_region(&fp->devno, 0, 1, DRIVER_NAME);
	if (ret) {
		dev_err(dev, "failed to alloc chrdev region: %d\n", ret);
		return ret;
	}

	cdev_init(&fp->cdev, &fingerprint_fops);
	fp->cdev.owner = THIS_MODULE;
	ret = cdev_add(&fp->cdev, fp->devno, 1);
	if (ret) {
		dev_err(dev, "failed to add cdev: %d\n", ret);
		goto err_unregister_chrdev;
	}

	fp->class = class_create("fpsensor");
	if (IS_ERR(fp->class)) {
		ret = PTR_ERR(fp->class);
		dev_err(dev, "failed to create class fpsensor: %d\n", ret);
		goto err_cdev_del;
	}

	fp->cdev_device = device_create(fp->class, dev, fp->devno, NULL,
					DRIVER_NAME);
	if (IS_ERR(fp->cdev_device)) {
		ret = PTR_ERR(fp->cdev_device);
		dev_err(dev, "failed to create device /dev/fingerprint: %d\n", ret);
		goto err_class_destroy;
	}

	platform_set_drvdata(pdev, fp);
	g_fp_data = fp;

	dev_info(dev, "Huawei fingerprint driver probed (rst=%d, irq=%d, spi_cs=%d, spi_num=%d)\n",
		 fp->reset_gpio, fp->irq_gpio, fp->cs_gpio, fp->spi_num);

	return 0;

err_class_destroy:
	class_destroy(fp->class);
err_cdev_del:
	cdev_del(&fp->cdev);
err_unregister_chrdev:
	unregister_chrdev_region(fp->devno, 1);
	return ret;
}

static void fingerprint_remove(struct platform_device *pdev)
{
	struct fingerprint_data *fp = platform_get_drvdata(pdev);

	device_destroy(fp->class, fp->devno);
	class_destroy(fp->class);
	cdev_del(&fp->cdev);
	unregister_chrdev_region(fp->devno, 1);
	g_fp_data = NULL;
}

static const struct of_device_id fingerprint_of_match[] = {
	{ .compatible = "fpc,fingerprint" },
	{ .compatible = "huawei,fingerprint" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fingerprint_of_match);

static struct platform_driver fingerprint_driver = {
	.probe  = fingerprint_probe,
	.remove = fingerprint_remove,
	.driver = {
		.name           = DRIVER_NAME,
		.of_match_table = fingerprint_of_match,
		.dev_groups     = fingerprint_groups,
	},
};

module_platform_driver(fingerprint_driver);

MODULE_AUTHOR("misaka-imouto <misaka-imouto@users.noreply.github.com>");
MODULE_DESCRIPTION("Huawei Kirin 990 / L410 Fingerprint Sensor Driver");
MODULE_LICENSE("GPL");
