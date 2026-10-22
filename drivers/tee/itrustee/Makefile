# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_TZDRIVER) += tzdriver.o

tzdriver-objs := core/smc_smp.o core/tc_client_driver.o core/session_manager.o core/mailbox_mempool.o core/teek_app_load.o
tzdriver-objs += core/agent.o core/gp_ops.o core/mem.o core/cmdmonitor.o core/tzdebug.o core/tz_spi_notify.o core/tz_pm.o core/tee_compat_check.o
tzdriver-objs += auth/auth_base_impl.o auth/client_hash_auth.o tlogger/tlogger.o tlogger/log_pages_cfg.o ko_adapt.o
tzdriver-objs += core/auth_enhance.o
tzdriver-objs += core/reserved_mempool.o
tzdriver-objs += core/shared_mem.o core/smc_abi.o
tzdriver-objs += core/teek_client_api.o
tzdriver-objs += core/tee_info.o
tzdriver-objs += tzdriver_internal/tee_reboot/reboot.o core/tc_cvm_driver.o
tzdriver-objs += tzdriver_internal/whitelist/agent_allowed_ca.o

tzdriver-objs += libboundscheck/src/memcpy_s.o libboundscheck/src/memset_s.o libboundscheck/src/strcpy_s.o libboundscheck/src/strncpy_s.o \
                 libboundscheck/src/memmove_s.o libboundscheck/src/strcat_s.o libboundscheck/src/strncat_s.o libboundscheck/src/strtok_s.o \
                 libboundscheck/src/securecutil.o libboundscheck/src/secureprintoutput_a.o libboundscheck/src/snprintf_s.o libboundscheck/src/vsnprintf_s.o

ccflags-y += -fstack-protector-strong -DCONFIG_TEELOG -DCONFIG_TEECD_AUTH -DCONFIG_PAGES_MEM=y
ccflags-y += -I$(src)/libboundscheck/include/ -I$(src) -I$(src)/auth -I$(src)/core -I$(src)/tzdriver_internal/tee_trace_event
ccflags-y += -I$(src)/tlogger -I$(src)/tzdriver_internal/kthread_affinity -I$(src)/tzdriver_internal/include
ccflags-y += -DCONFIG_CPU_AFF_NR=0 -DCONFIG_NOTIFY_PAGE_ORDER=5 -DCONFIG_512K_LOG_PAGES_MEM
ccflags-y += -DCONFIG_TEE_LOG_ACHIVE_PATH=\"/var/log/tee/last_teemsg\"
ccflags-y += -DNOT_TRIGGER_AP_RESET -DLAST_TEE_MSG_ROOT_GID -DCONFIG_NOCOPY_SHAREDMEM -DCONFIG_REGISTER_SHAREDMEM -DCONFIG_TA_AFFINITY=y -DCONFIG_TA_AFFINITY_CPU_NUMS=8
ccflags-y += -DTEECD_PATH_UID_AUTH_CTX=\"/usr/bin/teecd:0\"
ccflags-y += -DCONFIG_AUTH_SUPPORT_UNAME -DCONFIG_AUTH_HASH
ccflags-y += -DCONFIG_TEE_UPGRADE -DCONFIG_TEE_REBOOT -DCONFIG_CONFIDENTIAL_TEE -DCONFIG_CONFIDENTIAL_CONTAINER
ccflags-y += -I$(src)/tzdriver_internal/tee_reboot
ccflags-y += -DMAILBOX_POOL_COUNT=8 -DLATE_KILL_WHEN_TA_LOAD
ccflags-y += -DCONFIG_DISABLE_SHOW_AGENT_STATUS
ccflags-y += -DSECUREC_EXPORT_KERNEL_SYMBOL=0
