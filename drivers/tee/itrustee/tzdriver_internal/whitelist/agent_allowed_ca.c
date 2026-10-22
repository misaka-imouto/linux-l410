/*
 * agent_allowed_ca.c
 *
 * allowed_ext_agent_ca list and functions
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

#include "agent.h"
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <securec.h>

/*
 * External agents the vendor 4.19 kernel allows.  Extracted from
 * g_allowed_ext_agent_ca in the local kernel-image-4.19.71 (3 entries of
 * char path[512], u32 uid, u32 agent_id).
 */
static struct ca_info g_allowed_ext_agent_ca[] = {
	{ "/vendor/bin/hiaiserver", 1000, 0x53656345 },
	{ "/vendor/bin/hw/vendor.huawei.hardware.biometrics.hwfacerecognize@1.1-service",
	  1000, 0x46616365 },
	{ "/vendor/bin/hw/vendor.huawei.hardware.biometrics.hwfacerecognize@1.1-service",
	  1000, 0x46616345 },
};

int is_allowed_agent_ca(const struct ca_info *ca,
	bool check_agent_id)
{
	uint32_t i;
	struct ca_info *tmp_ca = g_allowed_ext_agent_ca;
	const uint32_t nr = ARRAY_SIZE(g_allowed_ext_agent_ca);

	if (!ca)
		return -EFAULT;

	/*
	 * teecd registers its own internal agents (fs, misc, secfile load,
	 * gtask) with IDs that are not in the 4.19 external-CA table.  Its
	 * path and UID are already checked by check_teecd_auth(), so trust
	 * the teecd process for all of them.
	 */
	if (ca->uid == 0 &&
	    (!strncmp(ca->path, "/vendor/bin/teecd", sizeof("/vendor/bin/teecd")) ||
	     !strncmp(ca->path, "/usr/bin/teecd", sizeof("/usr/bin/teecd"))))
		return 0;

	if (!check_agent_id) {
		for (i = 0; i < nr; i++) {
			if (!strncmp(ca->path, tmp_ca->path,
				strlen(tmp_ca->path) + 1) &&
				ca->uid == tmp_ca->uid)
				return 0;
			tmp_ca++;
		}
	} else {
		for (i = 0; i < nr; i++) {
			if (!strncmp(ca->path, tmp_ca->path,
				strlen(tmp_ca->path) + 1) &&
				ca->uid == tmp_ca->uid &&
				ca->agent_id == tmp_ca->agent_id)
				return 0;
			tmp_ca++;
		}
	}
	tlogd("ca-uid is %u, ca_path is %s, agent id is %x\n", ca->uid,
		ca->path, ca->agent_id);

	return -EACCES;
}

