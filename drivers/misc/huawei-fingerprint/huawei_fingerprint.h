/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Huawei Qingyun L410 / Kirin 990 Fingerprint Sensor Bridge Driver
 *
 * Copyright (c) 2026 Antigravity Authors
 */

#ifndef _HUAWEI_FINGERPRINT_H_
#define _HUAWEI_FINGERPRINT_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define FP_IOC_MAGIC                'f'

#define FP_IOC_CMD_ENABLE_IRQ       _IO(FP_IOC_MAGIC, 1)    /* 0x6601 */
#define FP_IOC_CMD_DISABLE_IRQ      _IO(FP_IOC_MAGIC, 2)    /* 0x6602 */
#define FP_IOC_CMD_SET_IPC_WAKELOCKS _IO(FP_IOC_MAGIC, 3)   /* 0x6603 */
#define FP_IOC_CMD_SEND_UEVENT      _IO(FP_IOC_MAGIC, 4)    /* 0x6604 */
#define FP_IOC_CMD_SEND_SENSORID    _IO(FP_IOC_MAGIC, 6)    /* 0x6606 */
#define FP_IOC_CMD_GET_IRQ_STATUS   _IO(FP_IOC_MAGIC, 8)    /* 0x6608 */
#define FP_IOC_CMD_SET_POWEROFF     _IO(FP_IOC_MAGIC, 10)   /* 0x660a */
#define FP_IOC_CMD_SEND_SENSORID_UD _IO(FP_IOC_MAGIC, 12)   /* 0x660c */
#define FP_IOC_CMD_SET_POWERON      _IO(FP_IOC_MAGIC, 14)   /* 0x660e */
#define FP_IOC_CMD_GET_DEVICE_INFO  0x4004660f

struct fp_device_info {
	int reset_pin;
	int irq_pin;
	int spi_cs;
	int spi_num;
};

#endif /* _HUAWEI_FINGERPRINT_H_ */
