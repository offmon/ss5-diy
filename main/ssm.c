#include "mbedtls/ecdh.h"
#include "mbedtls/cmac.h"
#include "mbedtls/ccm.h"
#include "mbedtls/ecp.h"

#pragma pack(1)
typedef struct {
	int64_t cnt;
	unsigned char n;
	unsigned char random_code[4];
} ccm_iv_t;
#pragma pack()

typedef struct {
	mbedtls_ccm_context ccm;
	unsigned char code[sizeof(((ccm_iv_t*)0)->random_code)];
} token_t;

/*Must call free_token after use token*/
void create_token(token_t *dst_with_code, const unsigned char *k, unsigned char *chk_4_dig){
	unsigned char token[16];
	mbedtls_aes_cmac_prf_128(k, 16, dst_with_code->code, 4, token);

	mbedtls_ccm_init(&dst_with_code->ccm);
	mbedtls_ccm_setkey(&dst_with_code->ccm, MBEDTLS_CIPHER_ID_AES, token, sizeof(token) * 8);

	if(chk_4_dig){
		chk_4_dig[0] = (memcmp(chk_4_dig, token, 4) == 0) ? 1 : 0;
	}
}

static inline void free_token(token_t *token){
	mbedtls_ccm_free(&token->ccm);
}

/* output buffer must be 4byte larger than input buffer */
/* 0 if success */
int ssm_encrypt(token_t *token, int64_t cnt, const unsigned char *in, size_t sz, unsigned char *out){
	ccm_iv_t iv = {.cnt = cnt, .n = 0};
	memcpy(iv.random_code, token->code, sizeof(token->code));
	return mbedtls_ccm_encrypt_and_tag(&token->ccm, sz, (void*)&iv, sizeof(ccm_iv_t), (const void*)"\0", 1, in, out, &out[sz], 4);
}

/* 0 if success */
int ssm_decrypt(token_t *token, int64_t cnt, const unsigned char *in, size_t sz, unsigned char *out){
	ccm_iv_t iv = {.cnt = cnt, .n = 0};
	memcpy(iv.random_code, token->code, sizeof(token->code));
	return mbedtls_ccm_auth_decrypt(&token->ccm, sz - 4, (void*)&iv, sizeof(ccm_iv_t),  (const void*)"\0", 1, in, out, &in[sz - 4], 4);
}

static int f_rng(void * p, unsigned char *out, size_t sz){
	bootloader_fill_random(out, sz);
	return 0;
}

void compute_shared_secret(unsigned char *r_pub_key, unsigned char *pub_key, unsigned char *shared){
	unsigned char key_buff[65] = {0x04}; //must append header
	mbedtls_ecp_group g;
	mbedtls_mpi pk;
	mbedtls_ecp_point pub;

	mbedtls_mpi_init(&pk);
	mbedtls_ecp_point_init(&pub);
	mbedtls_ecp_group_init(&g);
	mbedtls_ecp_group_load(&g, MBEDTLS_ECP_DP_SECP256R1);
	mbedtls_ecdh_gen_public(&g, &pk, &pub, &f_rng, NULL);

	mbedtls_ecp_point app_pub;
	mbedtls_ecp_point_init(&app_pub);
	memcpy(&key_buff[1], r_pub_key, sizeof(key_buff) - 1);
	mbedtls_ecp_point_read_binary(&g, &app_pub, key_buff, sizeof(key_buff));

	mbedtls_mpi sk;
	mbedtls_mpi_init(&sk);
	mbedtls_ecdh_compute_shared(&g, &sk, &app_pub, &pk, f_rng, NULL);

	unsigned char sk_buff[32];
	mbedtls_ecp_point_write_binary(&g, &pub, MBEDTLS_ECP_PF_UNCOMPRESSED, (size_t[]){0}, key_buff, sizeof(key_buff));
	mbedtls_mpi_write_binary(&sk, sk_buff, sizeof(sk_buff));
	memcpy(pub_key, &key_buff[1], sizeof(key_buff) - 1);
	memcpy(shared, sk_buff, 16);

	mbedtls_mpi_free(&pk);
	mbedtls_mpi_free(&sk);
	mbedtls_ecp_point_free(&pub);
	mbedtls_ecp_point_free(&app_pub);
	mbedtls_ecp_group_free(&g);
}

void calc_bda(const unsigned char *uuid, unsigned char *bda){
	unsigned char c[16];
	mbedtls_aes_cmac_prf_128(uuid, 16, (void*)"candy", 5, c);
	c[5] |= 0xC0;
	for(int i = 0; i < 6 ; i++){
		bda[i] = c[5 - i];
	}
}

#include "nvs_flash.h"
/* 0 if success */
int ssm_load_bin(const char *k, void *v, size_t sz){
	nvs_handle_t h_nvs;
	if(nvs_open("SSM", NVS_READONLY, &h_nvs) == ESP_OK){
		int r = (nvs_get_blob(h_nvs, k, v, (size_t []){sz}) == ESP_OK) ? 0 : 1;
		nvs_close(h_nvs);
		return r;
	}
	return 1;
}

void ssm_save_bin(const char *k, const void *v, size_t sz){
	nvs_handle_t h_nvs;
	if(nvs_open("SSM", NVS_READWRITE, &h_nvs) == ESP_OK){
		nvs_set_blob(h_nvs, k, v, sz);
		nvs_close(h_nvs);
	}
}

/* 0 if success */
int ssm_load_key(unsigned char *k){
	return ssm_load_bin("shared_key", k, 16);
}

void ssm_save_key(unsigned char *k){
	ssm_save_bin("shared_key", k, 16);
}
