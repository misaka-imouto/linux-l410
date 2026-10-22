/*
 * auth_enhance.h
 *
 * CONFIG_AUTH_ENHANCE support for the Kirin 990 4.19 TEEOS.
 *
 * Copyright (c) 2012-2022 Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#ifndef AUTH_ENHANCE_H
#define AUTH_ENHANCE_H

#include <linux/types.h>
#include "teek_ns_client.h"

struct tc_call_params;
struct tc_op_params;

void auth_enhance_set_root_key(const uint8_t *key);
bool auth_enhance_root_key_ready(void);

int auth_enhance_get_session_secure_params(struct tc_ns_dev_file *dev_file,
	struct tc_ns_client_context *context, struct tc_ns_session *session);
int auth_enhance_fill_open_session(const struct tc_call_params *call_params,
	struct tc_op_params *op_params);

#endif
