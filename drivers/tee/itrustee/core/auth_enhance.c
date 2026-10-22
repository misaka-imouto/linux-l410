/*
 * auth_enhance.c
 *
 * CONFIG_AUTH_ENHANCE support for the Kirin 990 4.19 TEEOS.
 *
 * The 4.19 vendor driver negotiates a per-session AES-256 key with TEEOS
 * (GLOBAL_CMD_ID_GET_SESSION_SECURE_PARAMS, 0x16) and then sends the open
 * session request with
 *
 *   - token_phys  pointing at 42 zero bytes,
 *   - params_phys pointing at the challenge word encrypted with that key.
 *
 * The root key used to decrypt the GET_SESSION_SECURE_PARAMS response is
 * written by TEEOS into the shared command buffer (in[0] + 4, right after
 * the legacy magic 0) when the driver hands it the buffer at boot.
 *
 * Copyright (c) 2012-2022 Huawei Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#include <linux/types.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/cred.h>
#include <linux/scatterlist.h>
#include <crypto/skcipher.h>
#include <securec.h>

#include "auth_enhance.h"
#include "teek_ns_client.h"
#include "tc_ns_log.h"
#include "smc_smp.h"
#include "mailbox_mempool.h"
#include "tee_compat_check.h"

#define AUTH_SECURE_PLAIN_SIZE \
	(AUTH_SECURE_PARAMS_SIZE - IV_BYTESIZE)

static uint8_t g_root_key[AUTH_ROOT_KEY_LEN];
static bool g_root_key_valid;

void auth_enhance_set_root_key(const uint8_t *key)
{
	if (!key)
		return;

	(void)memcpy_s(g_root_key, sizeof(g_root_key), key, AUTH_ROOT_KEY_LEN);
	g_root_key_valid = true;
	tlogi("auth enhance root key installed\n");
}

bool auth_enhance_root_key_ready(void)
{
	return g_root_key_valid;
}

static int aes_cbc_crypt(uint8_t *out, const uint8_t *in, const uint8_t *iv,
	const uint8_t *key, int size, bool encrypt)
{
	struct crypto_skcipher *tfm = NULL;
	struct skcipher_request *req = NULL;
	struct scatterlist src;
	struct scatterlist dst;
	uint8_t ivbuf[IV_BYTESIZE];
	int ret;

	tfm = crypto_alloc_skcipher("cbc(aes)", 0, 0);
	if (IS_ERR_OR_NULL(tfm)) {
		tloge("crypto_alloc_skcipher failed\n");
		return -EFAULT;
	}
	req = skcipher_request_alloc(tfm, GFP_KERNEL);
	if (!req) {
		crypto_free_skcipher(tfm);
		return -ENOMEM;
	}
	ret = crypto_skcipher_setkey(tfm, key, CIPHER_KEY_BYTESIZE);
	if (ret) {
		tloge("crypto_skcipher_setkey failed %d\n", ret);
		goto out;
	}
	(void)memcpy_s(ivbuf, sizeof(ivbuf), iv, IV_BYTESIZE);
	sg_init_one(&src, in, size);
	sg_init_one(&dst, out, size);
	skcipher_request_set_crypt(req, &src, &dst, size, ivbuf);
	ret = encrypt ? crypto_skcipher_encrypt(req) : crypto_skcipher_decrypt(req);
out:
	skcipher_request_free(req);
	crypto_free_skcipher(tfm);
	return ret;
}

/*
 * Encrypt: out = ciphertext(in_len) || random 16 byte IV (out_len - 16).
 * Decrypt: in = ciphertext(in_len - 16) || IV, out = plaintext(in_len - 16).
 */
static int session_aes_crypt(uint8_t *in, uint32_t in_len, uint8_t *out,
	uint32_t out_len, const uint8_t *key, bool encrypt)
{
	uint32_t src_len;
	uint32_t dest_len;
	uint8_t *iv;

	if (encrypt) {
		if (out_len < in_len + IV_BYTESIZE)
			return -EINVAL;
		src_len = in_len;
		dest_len = out_len - IV_BYTESIZE;
		iv = out + dest_len;
		get_random_bytes(iv, IV_BYTESIZE);
	} else {
		if (in_len < IV_BYTESIZE)
			return -EINVAL;
		src_len = in_len - IV_BYTESIZE;
		dest_len = out_len;
		iv = in + src_len;
	}

	if (!src_len || src_len != dest_len ||
		(src_len % CIPHER_BLOCK_BYTESIZE))
		return -EINVAL;

	return aes_cbc_crypt(out, in, iv, key, (int)src_len, encrypt);
}

static bool secure_params_head_valid(const struct auth_session_secure_params *p)
{
	if (memcmp(p->head.magic, AUTH_MAGIC_STRING,
		sizeof(AUTH_MAGIC_STRING)) != 0)
		return false;
	if (p->head.payload_len != sizeof(p->payload))
		return false;
	return true;
}

int auth_enhance_get_session_secure_params(struct tc_ns_dev_file *dev_file,
	struct tc_ns_client_context *context, struct tc_ns_session *session)
{
	struct tc_ns_smc_cmd smc_cmd = { {0}, 0 };
	struct auth_session_secure_params *req = NULL;
	struct auth_session_secure_params *plain = NULL;
	uint8_t *enc = NULL;
	uint64_t phys;
	int ret;

	if (!dev_file || !context || !session)
		return -EINVAL;
	if (!g_root_key_valid) {
		tloge("no auth enhance root key\n");
		return -EINVAL;
	}

	get_random_bytes(&session->secure_info.challenge_word,
		sizeof(session->secure_info.challenge_word));

	req = mailbox_alloc(AUTH_SECURE_PARAMS_SIZE, MB_FLAG_ZERO);
	if (!req) {
		tloge("alloc secure params failed\n");
		return -ENOMEM;
	}
	req->payload.ree2tee.challenge_word = session->secure_info.challenge_word;

	smc_cmd.cmd_type = CMD_TYPE_GLOBAL;
	(void)memcpy_s(smc_cmd.uuid, sizeof(smc_cmd.uuid), context->uuid, UUID_LEN);
	smc_cmd.cmd_id = GLOBAL_CMD_ID_GET_SESSION_SECURE_PARAMS;
	smc_cmd.dev_file_id = dev_file->dev_file_id;
	smc_cmd.context_id = context->session_id;
	smc_cmd.err_origin = TEEC_ORIGIN_COMMS;
	smc_cmd.uid = current_uid().val;
	smc_cmd.started = context->started;
	phys = mailbox_virt_to_phys((uintptr_t)req);
	smc_cmd.params_phys = (uint32_t)phys;
	smc_cmd.params_h_phys = (uint32_t)(phys >> ADDR_TRANS_NUM);

	ret = tc_ns_smc(&smc_cmd);
	if (ret != 0) {
		tloge("GET_SESSION_SECURE_PARAMS failed ret=0x%x\n", ret);
		ret = -EPERM;
		goto out;
	}

	/*
	 * The mailbox is ioremap'd (CONFIG_PAGES_MEM), so its pages are not
	 * linear-map struct pages; run the skcipher on a normal kmalloc copy.
	 */
	plain = kzalloc(sizeof(*plain), GFP_KERNEL);
	enc = kmalloc(AUTH_SECURE_PARAMS_SIZE, GFP_KERNEL);
	if (!plain || !enc) {
		ret = -ENOMEM;
		goto out;
	}
	(void)memcpy_s(enc, AUTH_SECURE_PARAMS_SIZE, req,
		AUTH_SECURE_PARAMS_SIZE);
	ret = session_aes_crypt(enc, AUTH_SECURE_PARAMS_SIZE,
		(uint8_t *)plain, sizeof(*plain), g_root_key, false);
	if (ret) {
		tloge("decrypt session secure params failed %d\n", ret);
		goto out;
	}
	if (!secure_params_head_valid(plain)) {
		tloge("invalid session secure params head\n");
		ret = -EFAULT;
		goto out;
	}

	(void)memcpy_s(session->secure_info.scrambling,
		sizeof(session->secure_info.scrambling),
		plain->payload.tee2ree.scrambling,
		sizeof(plain->payload.tee2ree.scrambling));
	(void)memcpy_s(&session->secure_info.crypto_info,
		sizeof(session->secure_info.crypto_info),
		&plain->payload.tee2ree.crypto_info,
		sizeof(plain->payload.tee2ree.crypto_info));
	ret = 0;
out:
	kfree(enc);
	kfree(plain);
	mailbox_free(req);
	return ret;
}

int auth_enhance_fill_open_session(const struct tc_call_params *call_params,
	struct tc_op_params *op_params)
{
	struct tc_ns_smc_cmd *smc_cmd = NULL;
	struct mb_cmd_pack *mb_pack = NULL;
	struct auth_session_secure_params *plain = NULL;
	uint8_t *enc = NULL;
	uint64_t phys;
	int ret;

	if (!call_params || !op_params || !op_params->smc_cmd ||
		!op_params->mb_pack)
		return -EINVAL;

	smc_cmd = op_params->smc_cmd;
	mb_pack = op_params->mb_pack;
	if (!call_params->sess) {
		tloge("open session without session\n");
		return -EINVAL;
	}

	/* open session carries a zero filled 42 byte token */
	(void)memset_s(mb_pack->token, sizeof(mb_pack->token), 0,
		sizeof(mb_pack->token));
	phys = mailbox_virt_to_phys((uintptr_t)mb_pack->token);
	smc_cmd->token_phys = (uint32_t)phys;
	smc_cmd->token_h_phys = (uint32_t)(phys >> ADDR_TRANS_NUM);

	plain = kzalloc(sizeof(*plain), GFP_KERNEL);
	if (!plain)
		return -ENOMEM;
	(void)memcpy_s(plain->head.magic, sizeof(plain->head.magic),
		AUTH_MAGIC_STRING, sizeof(AUTH_MAGIC_STRING));
	plain->head.payload_len = sizeof(plain->payload);
	plain->payload.ree2tee.challenge_word =
		call_params->sess->secure_info.challenge_word;

	enc = kmalloc(AUTH_SECURE_PARAMS_SIZE, GFP_KERNEL);
	if (!enc) {
		kfree(plain);
		return -ENOMEM;
	}
	/* mailbox pages are ioremap'd, so encrypt into a normal buffer first */
	ret = session_aes_crypt((uint8_t *)plain, sizeof(*plain),
		enc, AUTH_SECURE_PARAMS_SIZE,
		call_params->sess->secure_info.crypto_info.key, true);
	kfree(plain);
	if (ret) {
		tloge("encrypt open session params failed %d\n", ret);
		kfree(enc);
		return ret;
	}
	(void)memcpy_s(mb_pack->secure_params, sizeof(mb_pack->secure_params),
		enc, AUTH_SECURE_PARAMS_SIZE);
	kfree(enc);

	phys = mailbox_virt_to_phys((uintptr_t)mb_pack->secure_params);
	smc_cmd->params_phys = (uint32_t)phys;
	smc_cmd->params_h_phys = (uint32_t)(phys >> ADDR_TRANS_NUM);
	return 0;
}
