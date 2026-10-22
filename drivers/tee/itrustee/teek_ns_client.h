/*
 * teek_ns_client.h
 *
 * define structures and IOCTLs.
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
#ifndef TEEK_NS_CLIENT_H
#define TEEK_NS_CLIENT_H

#include <linux/mutex.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/completion.h>
#include "tc_ns_client.h"
#include "tc_ns_log.h"

#define TC_NS_CLIENT_IOC_MAGIC  't'
#define TC_NS_CLIENT_DEV        "tc_ns_client"
#define TC_PRIV_DEV             "tc_private"
#define TC_NS_CVM_DEV           "tc_ns_cvm"
#define TC_NS_CLIENT_DEV_NAME   "/dev/tc_ns_client"

#define EXCEPTION_MEM_SIZE (8*1024) /* mem for exception handling */

#define TSP_REQUEST        0xB2000008
#define TSP_RESPONSE       0xB2000009

#define TSP_REE_SIQ        0xB200000A
#define TSP_CRASH          0xB200000B

#ifdef CONFIG_TEE_UPGRADE
#define TSP_REBOOT         0xB2000012
#define TSP_CPU_ON         0xB2000013
#define TSP_REBOOT_DONE    0xB2000015
#else
#define TSP_REBOOT         0xB200000E
#define TSP_CPU_ON         0xB200000F
#define TSP_REBOOT_DONE    0xB2000010
#endif

#define TSP_PREEMPTED      0xB2000005
#define TC_CALL_GLOBAL     0x01
#define TC_CALL_SYNC       0x02
#define TC_CALL_LOGIN            0x04
#define TEE_REQ_FROM_USER_MODE   0U
#define TEE_REQ_FROM_KERNEL_MODE 1U
#define TEE_PARAM_NUM            4
#define VMALLOC_TYPE             0
#define RESERVED_TYPE            1

/* Max sizes for login info buffer comming from teecd */
#define MAX_PACKAGE_NAME_LEN 255
/* The apk certificate format is as follows:
  * modulus_size(4 bytes) + modulus buffer(512 bytes)
  * + exponent size(4 bytes) + exponent buffer(1 bytes)
  */
#define MAX_PUBKEY_LEN 1024

struct tc_ns_dev_list {
	struct mutex dev_lock; /* for dev_file_list */
	struct list_head dev_file_list;
};

struct tc_uuid {
	uint32_t time_low;
	uint16_t time_mid;
	uint16_t timehi_and_version;
	uint8_t clockseq_and_node[8]; /* clock len is 8 */
};

#define INVALID_MAP_ADDR ((void*)-1)
struct tc_ns_shared_mem {
	void *kernel_addr;
	void *user_addr;
	void *user_addr_ca; /* for ca alloc share mem */
	unsigned int len;
	int mem_type;
	struct list_head head;
	atomic_t usage;
	atomic_t offset;
	bool addr_corrupted; /* if user space modified the memory by partial munmap or other operation */
	bool addr_ca_corrupted;
};

struct tc_ns_service {
	unsigned char uuid[UUID_LEN];
	struct mutex session_lock; /* for session_list */
	struct list_head session_list;
	struct list_head head;
	struct mutex operation_lock; /* for session's open/close */
	atomic_t usage;
	unsigned int nsid;
	unsigned int vmid;
};

#define SERVICES_MAX_COUNT 32 /* service limit can opened on 1 fd */
struct tc_ns_dev_file {
	unsigned int dev_file_id;
	struct mutex service_lock; /* for service_ref[], services[] */
	uint16_t service_ref[SERVICES_MAX_COUNT]; /* a judge if set services[i]=NULL */
	struct tc_ns_service *services[SERVICES_MAX_COUNT];
	struct mutex shared_mem_lock; /* for shared_mem_list */
	struct list_head shared_mem_list;
	struct list_head head;
	/* Device is linked to call from kernel */
	uint8_t kernel_api;
	/* client login info provided by teecd, can be either package name and public
	 * key or uid(for non android services/daemons)
	 * login information can only be set once, dont' allow subsequent calls
	 */
	bool login_setup;
	struct mutex login_setup_lock; /* for login_setup */
#ifdef CONFIG_AUTH_HASH
	bool cainfo_hash_setup;
	struct mutex cainfo_hash_setup_lock;
#endif
	uint32_t pkg_name_len;
	uint8_t pkg_name[MAX_PACKAGE_NAME_LEN];
	uint32_t pub_key_len;
	uint8_t pub_key[
#ifdef CONFIG_TA_GET_CA_CMDLINE
		CA_CMDLINE_SIZE +
#endif
	MAX_PUBKEY_LEN];
	int load_app_flag;
#ifdef CONFIG_CONFIDENTIAL_CONTAINER
	uint32_t nsid;
	uint32_t vmid;
#endif
	struct completion close_comp; /* for kthread close unclosed session */
#ifdef CONFIG_TEE_TELEPORT_SUPPORT
	bool portal_enabled;
#endif
	struct mm_struct *mm;
	struct mutex groupid_lock;
	struct list_head groupid_list;
};

typedef struct {
	struct list_head head;
	unsigned int nsid;
	unsigned int vmid;
} struct_groupid;

union tc_ns_parameter {
	struct {
		unsigned int buffer;
		unsigned int size;
	} memref;
	struct {
		unsigned int a;
		unsigned int b;
	} value;
};

struct tc_ns_login {
	unsigned int method;
	unsigned int mdata;
};

struct tc_ns_operation {
	unsigned int paramtypes;
	union tc_ns_parameter params[TEE_PARAM_NUM];
	unsigned int    buffer_h_addr[TEE_PARAM_NUM];
	struct tc_ns_shared_mem *sharemem[TEE_PARAM_NUM];
	void *mb_buffer[TEE_PARAM_NUM];
};

struct tc_ns_temp_buf {
	void *temp_buffer;
	unsigned int size;
};

enum smc_cmd_type {
	CMD_TYPE_GLOBAL,
	CMD_TYPE_TA,
	CMD_TYPE_TA_AGENT,
	CMD_TYPE_TA2TA_AGENT, /* compatible with TA2TA2TA->AGENT etc. */
	CMD_TYPE_BUILDIN_AGENT,
	CMD_TYPE_RELEASE_AGENT, /* only for release agent */
};

struct tc_ns_smc_cmd {
	uint8_t      uuid[sizeof(struct tc_uuid)];
	/*
	 * The Kirin 990 TEEOS was built against the vendor 4.19 driver, whose
	 * smc command entry has a one byte cmd_type, so cmd_id sits at offset 17
	 * (verified by dumping the struct the 4.19 kernel hands to tc_ns_smc()).
	 * Keeping this a u32 shifts every following field by three bytes on the
	 * shared queue and TEEOS then never completes user initiated commands.
	 */
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
	unsigned int pid;    /* tgid */
	unsigned int nsid;
    unsigned int vmid;  /* smp_smc_send_func will set vmid to 0, only 0 is allowed */
	unsigned int eventindex;     /* tee audit event index for upload */
    unsigned int res_int[3]; /* 3 is limited by size of tc_ns_smc_queue, can't exceed 3 */
	bool started;
    char res_chr[3]; /* 3 is limited by size of tc_ns_smc_queue, can't exceed 3 */
	/*
	 * CONFIG_AUTH_ENHANCE fields the vendor 4.19 queue entry carries after
	 * ca_pid.  Kept at the end of the native struct; smc_cmd_to_wire() maps
	 * them onto token_phys/token_h_phys/params_phys/params_h_phys.
	 */
	unsigned int token_phys;
	unsigned int token_h_phys;
	unsigned int params_phys;
	unsigned int params_h_phys;
} __attribute__((__packed__));

/*
 * @brief
 */
struct tc_wait_data {
	wait_queue_head_t send_cmd_wq;
	int send_wait_flag;
};

#ifdef CONFIG_REGISTER_SHAREDMEM
struct tc_ns_register_sharedmem {
	uint64_t buf;
	uint32_t buf_size;
};
#endif

#define NUM_OF_SO 1
#ifdef CONFIG_CMS_CAHASH_AUTH
#define KIND_OF_SO 1
#else
#define KIND_OF_SO 2
#endif

/*
 * CONFIG_AUTH_ENHANCE, as used by the Kirin 990 4.19 TEEOS.
 *
 * The session parameters are an AES-256-CBC encrypted blob whose key is
 * negotiated with TEEOS through GLOBAL_CMD_ID_GET_SESSION_SECURE_PARAMS.
 * The blob is [head | payload] padded to 16 bytes and followed by the 16
 * byte IV, exactly like the vendor kernel's generate_encrypted_session_
 * secure_params().
 */
#define TOKEN_BUFFER_LEN         42
#define CIPHER_KEY_BYTESIZE      32
#define IV_BYTESIZE              16
#define CIPHER_BLOCK_BYTESIZE    16
#define SCRAMBLING_NUMBER        3
#define AUTH_MAGIC_SIZE          16
#define AUTH_MAGIC_STRING        "Trusted-magic"
#define AUTH_ROOT_KEY_LEN        CIPHER_KEY_BYTESIZE

struct auth_enhance_head {
	int8_t magic[AUTH_MAGIC_SIZE];
	uint32_t payload_len;
};

struct auth_session_crypto_info {
	uint8_t key[CIPHER_KEY_BYTESIZE];
	uint8_t iv[IV_BYTESIZE];
};

struct auth_session_secure_info {
	uint32_t challenge_word;
	uint32_t scrambling[SCRAMBLING_NUMBER];
	struct auth_session_crypto_info crypto_info;
};

struct auth_session_secure_params {
	struct auth_enhance_head head;
	union {
		struct {
			uint32_t challenge_word;
		} ree2tee;
		struct {
			uint32_t scrambling[SCRAMBLING_NUMBER];
			struct auth_session_crypto_info crypto_info;
		} tee2ree;
	} payload;
};

/* sizeof(head)=20, payload.tee2ree=60 -> 80; +16 byte trailing IV = 96 */
#define AUTH_SECURE_PARAMS_SIZE 96
/*
 * Encrypted CA/SO hash blob: payload 64 (SO hash + CA hash), head 20,
 * CMS padding to 96, then the 16 byte IV.
 */
#define AUTH_LOGIN_PAYLOAD_SIZE (MAX_SHA_256_SZ * (NUM_OF_SO + 1))
#define AUTH_LOGIN_BUF_SIZE 112

struct tc_ns_session {
	unsigned int session_id;
	struct list_head head;
	struct tc_wait_data wait_data;
	struct mutex ta_session_lock; /* for open/close/invoke on 1 session */
	struct tc_ns_dev_file *owner;
#ifdef CONFIG_REGISTER_SHAREDMEM
	struct tc_ns_register_sharedmem register_sharedmem;
#endif
	uint8_t auth_hash_buf[MAX_SHA_256_SZ * NUM_OF_SO + MAX_SHA_256_SZ];
	struct auth_session_secure_info secure_info;
	atomic_t usage;
};

struct mb_cmd_pack {
	struct tc_ns_operation operation;
	unsigned char login_data[AUTH_LOGIN_BUF_SIZE];
	unsigned char token[TOKEN_BUFFER_LEN];
	unsigned char secure_params[AUTH_SECURE_PARAMS_SIZE];
};

struct load_img_params {
	struct tc_ns_dev_file *dev_file;
	const char *file_buffer;
	unsigned int file_size;
	struct mb_cmd_pack *mb_pack;
	char *mb_load_mem;
	struct tc_uuid *uuid_return;
	unsigned int mb_load_size;
};

struct tc_call_params {
	struct tc_ns_dev_file *dev;
	struct tc_ns_client_context *context;
	struct tc_ns_session *sess;
	uint8_t flags;
};

struct tc_op_params {
	struct mb_cmd_pack *mb_pack;
	struct tc_ns_smc_cmd *smc_cmd;
	struct tc_ns_temp_buf local_tmpbuf[TEE_PARAM_NUM];
	uint32_t trans_paramtype[TEE_PARAM_NUM];
	bool op_inited;
};

#if (defined CONFIG_EXPORT_BY_GPL) || (KERNEL_VERSION(6, 6, 0) <= LINUX_VERSION_CODE)
#define EXPORT_SYMBOL_TZ EXPORT_SYMBOL_GPL
#else
#define EXPORT_SYMBOL_TZ EXPORT_SYMBOL
#endif

#endif
