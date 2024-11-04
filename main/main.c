#include <string.h>
#include <sys/time.h>

#include "driver/gpio.h"
#include "bootloader_random.h"
#include "nvs_flash.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "config.h"
#include "base64.c"
#include "ssm.c"

#define LOG_TAG "APP"
#define MAX_CONNECTION (3)

static QueueHandle_t q_cmd = NULL;
static QueueHandle_t q_led = NULL;
static const unsigned char *stat_mech = MECH_STAT_CLOSE;

static esp_ble_adv_params_t adv_params = {
	.adv_int_min         = 0x20,
	.adv_int_max         = 800,
	.adv_type            = ADV_TYPE_IND,
	.own_addr_type       = BLE_ADDR_TYPE_RANDOM,
	.channel_map         = ADV_CHNL_ALL,
	.adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

void set_adv_data(){
	unsigned char svc_uuid[] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x81, 0xfd, 0x00, 0x00};
	esp_ble_adv_data_t adv_data = {
		.set_scan_rsp = false,
		.include_name = false,
		.include_txpower = false,
		.min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
		.max_interval = 800, //slave connection max interval, Time = max_interval * 1.25 msec
		.appearance = 0x00,
		.service_data_len = 0,
		.p_service_data = NULL,
		.service_uuid_len = sizeof(svc_uuid),
		.p_service_uuid = svc_uuid,
		.flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
	};

	unsigned char uuid[16];
	static char dev_name[sizeof(uuid) * 2] = {0};
	static unsigned char man_data[sizeof(uuid) + 5] = {0x5A,0x05,0x05,0x00};
	if(dev_name[0] == 0){
			sscanf(DEVICE_UUID, "%02hhx%02hhx%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx-%02hhx%02hhx%02hhx%02hhx%02hhx%02hhx", 
			&uuid[0], &uuid[1], &uuid[2], &uuid[3], &uuid[4], &uuid[5], &uuid[6], &uuid[7],
			&uuid[8], &uuid[9], &uuid[10], &uuid[11], &uuid[12], &uuid[13], &uuid[14], &uuid[15]);
			
			base64_encode(uuid, sizeof(uuid), dev_name);
			memcpy(&man_data[5], uuid, sizeof(uuid));

			unsigned char bda[6];
			calc_bda(uuid,bda);
			esp_ble_gap_set_rand_addr(bda);
			esp_log_buffer_hex(LOG_TAG " BDA", bda, sizeof(bda));
	}
	esp_ble_gap_set_device_name(dev_name);

	man_data[4] = (ssm_load_key((unsigned char[16]){}) == 0) ? 1 : 0;

	adv_data.manufacturer_len = sizeof(man_data);
	adv_data.p_manufacturer_data = man_data;

	esp_ble_gap_config_adv_data(&adv_data);

	//adv_data.set_scan_rsp = true;
	//adv_data.manufacturer_len = 0;
	//adv_data.include_name = true,
	//adv_data.service_uuid_len = 0;
	//adv_data.flag = 0;
	//esp_ble_gap_config_adv_data(&adv_data); //scan resp data

	unsigned char dev_name_rsp[sizeof(dev_name) + 2] = {0};
	dev_name_rsp[1] = 0x09;
	dev_name_rsp[0] = strlen(dev_name);
	memcpy(&dev_name_rsp[2], dev_name, dev_name_rsp[0]++);
	esp_ble_gap_config_scan_rsp_data_raw(dev_name_rsp, dev_name_rsp[0] + 1);

	esp_ble_gap_set_device_name("ss5");
}

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)
static uint8_t adv_config_done       = 0;

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param){
	switch (event) {
		case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
			adv_config_done &= (~ADV_CONFIG_FLAG);
			if (adv_config_done == 0){
				esp_ble_gap_start_advertising(&adv_params);
			}
			break;
		case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
		case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
			adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
			if (adv_config_done == 0){
				esp_ble_gap_start_advertising(&adv_params);
			}
			break;
		case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
			break;
		case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
			ESP_LOGI(LOG_TAG, "ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT");
			set_adv_data();
			esp_ble_gap_start_advertising(&adv_params);
			break;
		case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
			ESP_LOGI(LOG_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
				  param->update_conn_params.status,
				  param->update_conn_params.min_int,
				  param->update_conn_params.max_int,
				  param->update_conn_params.conn_int,
				  param->update_conn_params.latency,
				  param->update_conn_params.timeout);
			break;
		default:
			break;
	}
}

typedef struct {
	int is_ready;
	token_t t;
	size_t pos;
	int64_t s_cnt;
	int64_t r_cnt;
	unsigned char buff[512];
	unsigned char ccc[2];
} con_ctx_t;

static void send(con_ctx_t *ctx, esp_gatt_if_t gatts_if, uint16_t conn_id, uint16_t handle, const unsigned char *p, size_t sz){
	unsigned char e[sz + 4];
	if(ctx){
		ssm_encrypt(&ctx->t, ctx->s_cnt, p, sz, e);
		p = e;
		sz += 4;
		ctx->s_cnt++;
	}

	for(size_t i = 0; i < sz ; i += 19){
		unsigned char b[19 + 2];
		b[0] = sizeof(b) - 2;
		b[1] = (i == 0) ? 1 : 0;
		if((sz - i) <= b[0]){
			b[0] = sz - i;
			b[1] |= ctx ? 0b100 : 0b10;
		}
		memcpy(&b[2], &p[i], b[0]);
		b[0] += 1;
		esp_ble_gatts_send_indicate(gatts_if, conn_id, handle, b[0], &b[1], false);
		esp_log_buffer_hex(LOG_TAG " SENT", &b[1], b[0]);
	}
}

static void set_mech_set_params(unsigned char *p){
	if(ssm_load_bin("close_timer", p, sizeof(unsigned int)) == 0){
		p[4] = p[0];
		p[5] = p[1];
	}else{
		p[4] = 0;
		p[5] = 0;
	}
	memcpy(p, &MECH_STAT_CLOSE[4], 2);
	memcpy(&p[2], &MECH_STAT_OPEN[4], 2);
}

#define ESP_GATTS_SSM_BCAST_EVT (0xFFFF)

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
	static esp_gatts_attr_db_t attrs[] = {
		//Primary Service
		{{ESP_GATT_AUTO_RSP}, { ESP_UUID_LEN_16, (static const uint8_t []) {0x00,0x28}, ESP_GATT_PERM_READ, ESP_UUID_LEN_16, ESP_UUID_LEN_16, (static const uint8_t []){0x81, 0xfd} }},
		
		//Write
		{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (static const uint8_t []){0x03,0x28} , ESP_GATT_PERM_READ, 1, 1, (static const uint8_t []){ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR} }},
		{{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (static const uint8_t[]){0x3e, 0x99, 0x76, 0xc6, 0xb4, 0xdb, 0xd3, 0xb6, 0x56, 0x98, 0xae, 0xa5, 0x02, 0x00, 0x86, 0x16}, ESP_GATT_PERM_WRITE, 22, 22, (static uint8_t [22]){0} }},

		//Read
		{{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (static const uint8_t []){0x03,0x28} , ESP_GATT_PERM_READ, 1, 1, (static const uint8_t []){ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY} }},
		{{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_128, (static const uint8_t[]){0x3e, 0x99, 0x76, 0xc6, 0xb4, 0xdb, 0xd3, 0xb6, 0x56, 0x98, 0xae, 0xa5, 0x03, 0x00, 0x86, 0x16}, ESP_GATT_PERM_READ, 22, 22, (static const uint8_t [22]){0} }},
		{{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (static const uint8_t []){0x02,0x29} , ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, 2, 2, (static uint8_t []){0,0} }},
	};
	static uint16_t handles[sizeof(attrs) / sizeof(attrs[0])];
	static con_ctx_t ctx[MAX_CONNECTION];
	static esp_gatt_if_t  gatt_if;
	static SemaphoreHandle_t mtx;

	switch ((int)event) {
		case ESP_GATTS_REG_EVT:{
				set_adv_data();

				esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(attrs, gatts_if, sizeof(attrs) / sizeof(attrs[0]), 0);

				if (create_attr_ret){
					ESP_LOGE(LOG_TAG, "create attr table failed, error code = %x", create_attr_ret);
				}

				for(int i = 0; i < (sizeof(ctx) / sizeof(ctx[0])) ; i++){
					mbedtls_ccm_init(&ctx[i].t.ccm);
					ctx[i].is_ready = 0;
				}

				gatt_if = gatts_if;
				mtx = xSemaphoreCreateBinary();
			}
			break;
		case ESP_GATTS_READ_EVT:
			ESP_LOGI(LOG_TAG, "ESP_GATTS_READ_EVT");
			if(param->read.conn_id < (sizeof(ctx)/sizeof(ctx[0]))){
				if(param->read.handle == handles[5]) { //CCC for Read characteristic
					esp_gatt_rsp_t r;
					r.attr_value.len = sizeof(ctx[param->read.conn_id].ccc);
					r.attr_value.handle = param->read.handle;
					r.attr_value.offset = 0;
					r.attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
					memcpy(r.attr_value.value, ctx[param->read.conn_id].ccc, sizeof(ctx[param->read.conn_id].ccc));
					esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &r);
				}
			}
			break;
		case ESP_GATTS_WRITE_EVT:
			if (!param->write.is_prep){
				// the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
				ESP_LOGI(LOG_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
				esp_log_buffer_hex(LOG_TAG, param->write.value, param->write.len);
				if(param->write.conn_id < (sizeof(ctx)/sizeof(ctx[0]))){
					if(param->write.handle == handles[2]) { //Write characteristic
						if((ctx[param->write.conn_id].pos + param->write.len) <= sizeof(ctx[param->write.conn_id].buff)){
							if(param->write.value[0] & 1){ //bigin
								ctx[param->write.conn_id].pos = 0;
							}
							memcpy(&ctx[param->write.conn_id].buff[ctx[param->write.conn_id].pos], &param->write.value[1], param->write.len - 1);
							ctx[param->write.conn_id].pos += param->write.len - 1;
							if(param->write.value[0] >= 2){ //end
								int err = 1;
								if(param->write.value[0] & 0b100){ //encrypted
									if(ctx[param->write.conn_id].is_ready == 2){
										unsigned char d[sizeof(ctx[param->write.conn_id].buff)];
										if(ssm_decrypt(&ctx[param->write.conn_id].t, ctx[param->write.conn_id].r_cnt, ctx[param->write.conn_id].buff, ctx[param->write.conn_id].pos, d) == 0){
											ctx[param->write.conn_id].r_cnt++;
											esp_log_buffer_hex(LOG_TAG " CMD(DEC)", d, ctx[param->write.conn_id].pos - 4);
											err = 0;
											switch(d[0]){
												case 4: //History
													//Not found
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], (unsigned char[]){0x07, d[0], 0x05}, 3);
													break;
												case 8: //Set time
													struct timeval t = {.tv_sec = 0, .tv_usec = 0};
													memcpy(&t.tv_sec, &d[1], 4);
													settimeofday(&t, NULL);
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], (unsigned char[]){0x07, d[0], 0x00}, 3);
													break;
												case 11: //Set autolock
													//0000 : OFF, in second
													ssm_save_bin("close_timer",&d[1],2);
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], (unsigned char[]){0x07, d[0], 0x00}, 3);
													break;
												case 17: //Set zero point
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], (unsigned char[]){0x07, d[0], 0x00}, 3);
													break;
												case 80: { //Mech Setting
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], (unsigned char[]){0x07, d[0], 0x00}, 3);
													unsigned char ret[8] = {0x08, 0x50};
													set_mech_set_params(&ret[2]);
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], ret, sizeof(ret));
													break;
													}
												case 82: //Close
												case 83: //Open
													xQueueSend(q_cmd, (unsigned int[]){d[0]}, 0);
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], (unsigned char[]){0x07, d[0], 0x00}, 3);
													break;
												case 5: {//FW VER
													unsigned char ret[3 + sizeof(FW_VERSION) - 1] = {0x07, 0x05, 0x00};
													memcpy(&ret[3], FW_VERSION, sizeof(FW_VERSION) - 1);
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], ret, sizeof(ret));
													break;
													}
												case 92: //Set close timing for open sensor
													//0000 : OFF, FFFF : immediately
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], (unsigned char[]){0x07, d[0], 0x00}, 3);
													send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], (unsigned char[]){0x08, d[0], 0x00, 0x00}, 4);
													break;
											}
										}else{
											ESP_LOGE(LOG_TAG, "Decrypt failed");
										}
									}
								}else{
									if(ctx[param->write.conn_id].is_ready == 1){
										switch(ctx[param->write.conn_id].buff[0]){
											case 0x01: //register
												ESP_LOGI(LOG_TAG, "Register");
												err = 0;
												if(ssm_load_key((unsigned char[16]){}) == 0){ //Already registered
													unsigned char ret[] = {0x07,0x01,0x09};
													send(NULL, gatts_if, param->write.conn_id, handles[4], ret, sizeof(ret));
												}else{
													unsigned char ret[80] = {0x07,0x01,0x00};
													unsigned char sk[16];
													//esp_log_buffer_hex(LOG_TAG " APP PUB KEY", &ctx[param->write.conn_id].buff[1], 64);
													//esp_log_buffer_hex(LOG_TAG " TM", &ctx[param->write.conn_id].buff[65], 4); //LE
													struct timeval t = {.tv_sec = 0, .tv_usec = 0};
													memcpy(&t.tv_sec, &ctx[param->write.conn_id].buff[65], 4);
													settimeofday(&t, NULL);
													compute_shared_secret(&ctx[param->write.conn_id].buff[1], &ret[13 + 3], sk);
													memcpy(&ret[3], stat_mech, sizeof(MECH_STAT_CLOSE));
													set_mech_set_params(&ret[10]);
													send(NULL, gatts_if, param->write.conn_id, handles[4], ret, sizeof(ret));
													esp_log_buffer_hex(LOG_TAG " KEY", sk, sizeof(sk));
													ssm_save_key(sk);
													create_token(&ctx[param->write.conn_id].t, sk, NULL);
													ctx[param->write.conn_id].is_ready = 2;
													esp_ble_gap_stop_advertising(); //restart adv
												}
												break;
											case 0x02: { //login
												ESP_LOGI(LOG_TAG, "Login");
												unsigned char sk[16];
												if(ssm_load_key(sk) == 0){
													create_token(&ctx[param->write.conn_id].t, sk, &ctx[param->write.conn_id].buff[1]);
													if(ctx[param->write.conn_id].buff[1]){
														unsigned char ret[7] = {0x07,0x02,0x00};
														struct timeval t;
														gettimeofday(&t, NULL);
														memcpy(&ret[3], &t.tv_sec, 4);
														send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], ret, sizeof(ret));

														unsigned char ret2[8] = {0x08, 0x50};
														unsigned char ret3[sizeof(MECH_STAT_CLOSE) + 2] = {0x08, 0x51};
														unsigned char ret4[] = {0x08, 0x5c, 0, 0};
														set_mech_set_params(&ret2[2]);
														memcpy(&ret3[2], stat_mech, sizeof(MECH_STAT_CLOSE));
														send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], ret2, sizeof(ret2));
														send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], ret3, sizeof(ret3));
														send(&ctx[param->write.conn_id], gatts_if, param->write.conn_id, handles[4], ret4, sizeof(ret4));

														err = 0;
														ctx[param->write.conn_id].is_ready = 2;
													}
												}
												}
												break;
										}
									}
								}

								ctx[param->write.conn_id].pos = 0;

								if(err){
									ESP_LOGE(LOG_TAG, "Error occurred will disconnect.");
									esp_ble_gap_disconnect(param->write.bda);
								}
							}
						}
					}else if(param->write.handle == handles[5]) { //CCC for Read characteristic
						if((param->write.len == sizeof(ctx[param->write.conn_id].ccc)) && (memcmp(param->write.value,"\1\0",2) == 0)){
							ESP_LOGI(LOG_TAG, "CONNECTION INIT");
							if(memcmp(ctx[param->write.conn_id].ccc,"\0\0",2) != 0){
								xSemaphoreTakeFromISR(mtx, (portBASE_TYPE[]){pdFALSE});
								ctx[param->write.conn_id].is_ready = 0;
								free_token(&ctx[param->write.conn_id].t);
								xSemaphoreGiveFromISR(mtx, (portBASE_TYPE[]){pdFALSE});
							}
							unsigned char d[6] = {0x08,14};
							bootloader_fill_random(ctx[param->write.conn_id].t.code,sizeof(ctx[param->write.conn_id].t.code));
							memcpy(&d[2],ctx[param->write.conn_id].t.code,sizeof(ctx[param->write.conn_id].t.code));
							send(NULL, gatts_if, param->write.conn_id, handles[4], d, sizeof(d));
							ctx[param->write.conn_id].is_ready = 1;
						}
						memcpy(ctx[param->write.conn_id].ccc, param->write.value,sizeof(ctx[param->write.conn_id].ccc));
					}
				}
				/* send response when param->write.need_rsp is true*/
				if (param->write.need_rsp){
					//ESP_LOGI(LOG_TAG, "WRITE RESP");
					esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
				}
			}else{
				/* handle prepare write */
			}
			break;
		case ESP_GATTS_EXEC_WRITE_EVT:
			// the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
			ESP_LOGI(LOG_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
			break;
		case ESP_GATTS_MTU_EVT:
			ESP_LOGI(LOG_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
			break;
		case ESP_GATTS_CONF_EVT:
			ESP_LOGI(LOG_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
			break;
		case ESP_GATTS_START_EVT:
			ESP_LOGI(LOG_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
			break;
		case ESP_GATTS_CONNECT_EVT:
			ESP_LOGI(LOG_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
			esp_log_buffer_hex(LOG_TAG " BDA(CONNECTED)", param->connect.remote_bda, 6);
			esp_ble_conn_update_params_t conn_params = {0};
			memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
			/* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
			//conn_params.latency = 0;
			//conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
			//conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
			//conn_params.timeout = 1000;    // timeout = 1000*10ms = 10000ms
			////start sent the update connection parameters to the peer device.
			//esp_ble_gap_update_conn_params(&conn_params);
			esp_ble_gap_start_advertising(&adv_params);
			if(param->connect.conn_id < (sizeof(ctx)/sizeof(ctx[0]))){
				xSemaphoreTakeFromISR(mtx, (portBASE_TYPE[]){pdFALSE});
				ctx[param->connect.conn_id].is_ready = 0;
				free_token(&ctx[param->connect.conn_id].t);				
				xSemaphoreGiveFromISR(mtx, (portBASE_TYPE[]){pdFALSE});

				memset(ctx[param->connect.conn_id].ccc, 0, sizeof(ctx[param->connect.conn_id].ccc));
				ctx[param->connect.conn_id].pos = 0;
				ctx[param->connect.conn_id].s_cnt = 0;
				ctx[param->connect.conn_id].r_cnt = 0;
			}else{
				//Too many connections
				esp_ble_gap_disconnect(param->connect.remote_bda);
			}
			break;
		case ESP_GATTS_DISCONNECT_EVT:
			ESP_LOGI(LOG_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
			if(param->disconnect.conn_id < (sizeof(ctx)/sizeof(ctx[0]))){
				ctx[param->disconnect.conn_id].is_ready = 0;
			}
			esp_ble_gap_start_advertising(&adv_params);
			break;
		case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
				if (param->add_attr_tab.status != ESP_GATT_OK){
					ESP_LOGE(LOG_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
				} else {
					ESP_LOGI(LOG_TAG, "create attribute table successfully, the number handle = %d",param->add_attr_tab.num_handle);
					memcpy(handles, param->add_attr_tab.handles, sizeof(handles));
					esp_ble_gatts_start_service(handles[0]);
				}
				break;
			}
		case ESP_GATTS_SSM_BCAST_EVT: {
				void **p = param;
				for(uint16_t i = 0; i < (sizeof(ctx)/sizeof(ctx[0])) ; i++){
					if(ctx[i].is_ready >= 2){
						xSemaphoreTake(mtx, portMAX_DELAY);
						send(&ctx[i], gatt_if, i, handles[4], p[1], (size_t)p[0]);
						xSemaphoreGive(mtx);
					}
				}
				break;
			}
		case ESP_GATTS_OPEN_EVT:
		case ESP_GATTS_CLOSE_EVT:
		case ESP_GATTS_STOP_EVT:
		case ESP_GATTS_CANCEL_OPEN_EVT:
		case ESP_GATTS_LISTEN_EVT:
		case ESP_GATTS_CONGEST_EVT:
		case ESP_GATTS_UNREG_EVT:
		case ESP_GATTS_DELETE_EVT:
		default:
			break;
	}
}

static void btn_thread(void *){
	gpio_set_direction(PIN_BTN, GPIO_MODE_INPUT);
	gpio_set_pull_mode(PIN_BTN, GPIO_PULLUP_ONLY);

	while(1){
		while(gpio_get_level(PIN_BTN)){
			vTaskDelay(32 / portTICK_PERIOD_MS);
		}

		xQueueSend(q_cmd, (unsigned int[]){0xF000}, 0);

		for(int i = 0; !gpio_get_level(PIN_BTN) ; i++){
			if(i >= (20000 / 32)){
				xQueueSend(q_led, (unsigned int[]){3000}, pdMS_TO_TICKS(1000));
				nvs_flash_erase();
				for(i = 0; (i < (3000 / 50)) || !gpio_get_level(PIN_BTN) ; i++){
					vTaskDelay(32 / portTICK_PERIOD_MS);
				}
				esp_restart();
				while(1){}
			}
			vTaskDelay(32 / portTICK_PERIOD_MS);
		}
	}
}

static void led_thread(void *){
	gpio_set_direction(PIN_ACC_LED, GPIO_MODE_INPUT);
	xQueueSend(q_led, (unsigned int[]){1000}, 0);
	while(1){
		unsigned int d;
		if(xQueueReceive(q_led, (void*)&d, portMAX_DELAY) == pdTRUE) {
			gpio_set_pull_mode(PIN_ACC_LED, GPIO_PULLUP_ONLY);
			vTaskDelay(d / portTICK_PERIOD_MS);
			gpio_set_pull_mode(PIN_ACC_LED, GPIO_PULLDOWN_ONLY);
			vTaskDelay(50 / portTICK_PERIOD_MS);
		}
	}
}

void close_timer(TimerHandle_t xTimer){
	xQueueSend(q_cmd, (unsigned int[]){0xF001}, portMAX_DELAY);
}

void app_main(void){
	esp_err_t ret;

	/* Initialize NVS. */
	ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	ESP_LOGI(LOG_TAG, "SHARED KEY %s", (ssm_load_key((unsigned char[16]){}) == 0) ? "LOADED" : "NOT LOADED");

	on_init();

    q_cmd = xQueueCreate(10, sizeof(unsigned int));
    q_led = xQueueCreate(10, sizeof(unsigned int));

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	bt_cfg.ble_max_conn = MAX_CONNECTION;
	ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(LOG_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) {
		ESP_LOGE(LOG_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_init();
	if (ret) {
		ESP_LOGE(LOG_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_bluedroid_enable();
	if (ret) {
		ESP_LOGE(LOG_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
		return;
	}

	ret = esp_ble_gatts_register_callback(gatts_event_handler);
	if (ret){
		ESP_LOGE(LOG_TAG, "gatts register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gap_register_callback(gap_event_handler);
	if (ret){
		ESP_LOGE(LOG_TAG, "gap register error, error code = %x", ret);
		return;
	}

	ret = esp_ble_gatts_app_register(0);
	if (ret){
		ESP_LOGE(LOG_TAG, "gatts app register error, error code = %x", ret);
		return;
	}

	esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
	if (local_mtu_ret){
		ESP_LOGE(LOG_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
	}

	xTaskCreate(btn_thread, "btn_thread", 2048, NULL, (tskIDLE_PRIORITY + 3), NULL);
	xTaskCreate(led_thread, "led_thread", 2048, NULL, (tskIDLE_PRIORITY + 2), NULL);

	TimerHandle_t tm_h = xTimerCreate("close_timer", 1, pdFALSE, 0, close_timer);

	while(1){
		unsigned int p;
		if(xQueueReceive(q_cmd, (void*)&p, portMAX_DELAY) == pdTRUE) {
			const void *prev_stat = stat_mech;
			if(p == 0xF000){ //Toggle by button
				p = (stat_mech == MECH_STAT_OPEN) ? 82 : 83;
			}
			switch(p){
				case 83: //open
					if(stat_mech != MECH_STAT_OPEN){
						stat_mech = MECH_STAT_OPEN;
						ESP_LOGI(LOG_TAG, "OPEN");
						xQueueSend(q_led, (unsigned int[]){50}, 0);
						on_open_cb();
						unsigned int close_tm = 0;
						if(ssm_load_bin("close_timer", &close_tm, 2) == 0){
							if(close_tm > 0){
								xTimerStop(tm_h, 0);
								xTimerChangePeriod(tm_h, close_tm * 1000 / portTICK_PERIOD_MS, 0);
								xTimerStart(tm_h, 0);
							}
						}
					}
					break;
				case 0xF001: //close timer // thru
				case 82: //close
					if(stat_mech != MECH_STAT_CLOSE){
						stat_mech = MECH_STAT_CLOSE;
						ESP_LOGI(LOG_TAG, "CLOSE");
						xQueueSend(q_led, (unsigned int[]){50}, 0);
						xQueueSend(q_led, (unsigned int[]){50}, 0);
						on_close_cb();
						xTimerStop(tm_h, 0);
					}
					break;
				default:
					continue;
			}
			if(prev_stat != stat_mech){
				unsigned char st[sizeof(MECH_STAT_CLOSE) + 2] = {0x08, 81};
				memcpy(&st[2], stat_mech, sizeof(MECH_STAT_CLOSE));
				gatts_event_handler((esp_gatts_cb_event_t) ESP_GATTS_SSM_BCAST_EVT, NULL, (void*[]){sizeof(st),st});
			}
		}
	}
}
