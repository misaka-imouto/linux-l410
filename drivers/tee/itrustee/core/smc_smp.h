/*
 * smc_smp.h
 *
 * function declaration for sending smc cmd
 *
 * Copyright (c) 2012-2022 Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef SMC_SMP_H
#define SMC_SMP_H

#include <linux/of_device.h>
#include "teek_client_constants.h"
#include "teek_ns_client.h"

#if (KERNEL_VERSION(5, 4, 0) <= LINUX_VERSION_CODE)
#define CURRENT_CPUS_ALLOWED (&current->cpus_mask)
#else
#define CURRENT_CPUS_ALLOWED (&current->cpus_allowed)
#endif

enum tc_ns_cmd_type {
	TC_NS_CMD_TYPE_INVALID = 0,
	TC_NS_CMD_TYPE_NS_TO_SECURE,
	TC_NS_CMD_TYPE_SECURE_TO_NS,
	TC_NS_CMD_TYPE_SECURE_TO_SECURE,
	TC_NS_CMD_TYPE_SECURE_CONFIG = 0xf,
	TC_NS_CMD_TYPE_MAX
};

struct pending_entry {
	atomic_t users;
	struct task_struct *task;
#ifdef CONFIG_TA_AFFINITY
	struct cpumask ca_mask;
	struct cpumask ta_mask;
#endif
	pid_t pid;
	wait_queue_head_t wq;
	atomic_t run;
	struct list_head list;
};

#ifdef CONFIG_BIG_SESSION
#define MAX_SMC_CMD (CONFIG_BIG_SESSION * 10)
#else
#define MAX_SMC_CMD 18
#endif

#ifdef DIV_ROUND_UP
#undef DIV_ROUND_UP
#endif
#define DIV_ROUND_UP(n, d)              (((n) + (d) - 1) / (d))

#define BITS_PER_BYTE                   8

#ifdef BITS_TO_LONGS
#undef BITS_TO_LONGS
#endif
#define BITS_TO_LONGS(nr)               DIV_ROUND_UP(nr, BITS_PER_BYTE * sizeof(uint64_t))

#ifdef BIT_MASK
#undef BIT_MASK
#endif
#define BIT_MASK(nr)                    (1UL << (((uint64_t)(nr)) % sizeof(uint64_t)))

#ifdef BIT_WORD
#undef BIT_WORD
#endif
#define BIT_WORD(nr)                    ((nr) / BITS_PER_LONG)

#ifdef DECLARE_BITMAP
#undef DECLARE_BITMAP
#endif
#define DECLARE_BITMAP(name, bits)      unsigned long name[BITS_TO_LONGS(bits)]

#define SIQ_DUMP_TIMEOUT 1U
#define SIQ_DUMP_SHELL   2U

typedef uint32_t smc_buf_lock_t;

/*
 * On-wire layout of one shared queue entry, as expected by the vendor
 * Kirin 990 4.19 TEEOS.
 *
 * The 4.19 driver copies sizeof(struct tc_ns_smc_cmd) == 102 bytes into
 * g_cmd_data->in[idx] and reads the same amount from g_cmd_data->out[idx]
 * (verified by disassembling the vendor kernel: occupy/smp_smc_send
 * memcpy 0x66 bytes, the queue layout is 3 bitmaps + smc_lock + last_in,
 * in[18], last_out, out[18], so out[] sits at 0x750).  It is not the same
 * struct as our newer one: with CONFIG_AUTH_ENHANCE the vendor tail is
 * token_phys/token_h_phys/pid/params_phys/params_h_phys/eventindex/started
 * and there is no nsid/vmid/res_int/res_chr.  If we use our 109 byte entry
 * the in[]/out[] slots shift by 7 bytes each and TEEOS writes its result to
 * an address the driver never reads.
 */
struct tc_ns_smc_cmd_wire {
	uint8_t      uuid[sizeof(struct tc_uuid)];
	uint8_t      cmd_type;
	unsigned int cmd_id;
	unsigned int dev_file_id;
	unsigned int context_id;
	unsigned int agent_id;
	unsigned int operation_phys;
	unsigned int operation_h_phys;
	unsigned int login_method;
	unsigned int login_data_phy;
	unsigned int login_data_h_addr;
	unsigned int login_data_len;
	unsigned int err_origin;
	int          ret_val;
	unsigned int event_nr;
	unsigned int uid;
	unsigned int ca_pid; /* pid */
	unsigned int token_phys;     /* CONFIG_AUTH_ENHANCE */
	unsigned int token_h_phys;   /* CONFIG_AUTH_ENHANCE */
	unsigned int pid;            /* tgid */
	unsigned int params_phys;    /* CONFIG_AUTH_ENHANCE */
	unsigned int params_h_phys;  /* CONFIG_AUTH_ENHANCE */
	unsigned int eventindex;     /* tee audit event index for upload */
	bool         started;
} __attribute__((__packed__));

struct tc_ns_smc_queue {
	/* set when CA send cmd_in, clear after cmd_out return */
	DECLARE_BITMAP(in_bitmap, MAX_SMC_CMD);
	/* set when gtask get cmd_in, clear after cmd_out return */
	DECLARE_BITMAP(doing_bitmap, MAX_SMC_CMD);
	/* set when gtask get cmd_out, clear after cmd_out return */
	DECLARE_BITMAP(out_bitmap, MAX_SMC_CMD);
	smc_buf_lock_t smc_lock;
	volatile uint32_t last_in;
	struct tc_ns_smc_cmd_wire in[MAX_SMC_CMD];
	volatile uint32_t last_out;
	struct tc_ns_smc_cmd_wire out[MAX_SMC_CMD];
};

#define SYM_NAME_LEN_MAX 16
#define SYM_NAME_LEN_1 7
#define SYM_NAME_LEN_2 4
#define CRASH_REG_NUM 3
#define LOW_FOUR_BITE 4

union crash_inf {
	uint64_t crash_reg[CRASH_REG_NUM];
	struct {
		uint8_t halt_reason : LOW_FOUR_BITE;
		uint8_t app : LOW_FOUR_BITE;
		char sym_name[SYM_NAME_LEN_1];
		uint16_t off;
		uint16_t size;
		uint32_t far;
		uint32_t fault;
		union {
			char sym_name_append[SYM_NAME_LEN_2];
			uint32_t elr;
		};
	} crash_msg;
};

#define RESLEEP_TIMEOUT 15

bool sigkill_pending(struct task_struct *tsk);
int smc_context_init(const struct device *class_dev);
void free_smc_data(void);
int tc_ns_smc_skip_kill(struct tc_ns_smc_cmd *cmd);
int tc_ns_smc(struct tc_ns_smc_cmd *cmd);
int tc_ns_smc_with_no_nr(struct tc_ns_smc_cmd *cmd);
int teeos_log_exception_archive(unsigned int eventid, const char *exceptioninfo);
void set_cmd_send_state(void);
int init_smc_svc_thread(void);
int smc_wakeup_ca(pid_t ca);
int smc_wakeup_broadcast(void);
int smc_shadow_exit(pid_t ca);
int smc_queue_shadow_worker(uint64_t target);
void fiq_shadow_work_func(uint64_t target);
struct pending_entry *find_pending_entry(pid_t pid);
void foreach_pending_entry(void (*func)(struct pending_entry *));
void put_pending_entry(struct pending_entry *pe);
void show_cmd_bitmap(void);
void wakeup_tc_siq(uint32_t siq_mode);
void occupy_clean_cmd_buf(void);
void clr_system_crash_flag(void);
void svc_thread_release(void);
int send_smc_cmd_rebooting(uint32_t cmd_id, const struct tc_ns_smc_cmd *in_cmd);
void send_smc_cmd_buffer(bool tee_is_dead);
int parse_params_from_tee(void);
int smp_smc_send_func(struct tc_ns_smc_cmd *in, bool reuse, bool skip_kill);
void set_abort_cmd_enabled(bool enabled);
void wakeup_pe(struct pending_entry *pe);
bool is_tee_hungtask(struct task_struct *task);
void smc_set_cmd_buffer(void);

#endif
