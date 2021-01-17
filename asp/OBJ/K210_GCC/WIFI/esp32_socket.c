/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2020-2021 by fukuen
 * 
 *  上記著作権者は，以下の(1)～(4)の条件を満たす場合に限り，本ソフトウェ
 *  ア（本ソフトウェアを改変したものを含む．以下同じ）を使用・複製・改
 *  変・再配布（以下，利用と呼ぶ）することを無償で許諾する．
 *  (1) 本ソフトウェアをソースコードの形で利用する場合には，上記の著作
 *      権表示，この利用条件および下記の無保証規定が，そのままの形でソー
 *      スコード中に含まれていること．
 *  (2) 本ソフトウェアを，ライブラリ形式など，他のソフトウェア開発に使
 *      用できる形で再配布する場合には，再配布に伴うドキュメント（利用
 *      者マニュアルなど）に，上記の著作権表示，この利用条件および下記
 *      の無保証規定を掲載すること．
 *  (3) 本ソフトウェアを，機器に組み込むなど，他のソフトウェア開発に使
 *      用できない形で再配布する場合には，次のいずれかの条件を満たすこ
 *      と．
 *    (a) 再配布に伴うドキュメント（利用者マニュアルなど）に，上記の著
 *        作権表示，この利用条件および下記の無保証規定を掲載すること．
 *    (b) 再配布の形態を，別に定める方法によって，TOPPERSプロジェクトに
 *        報告すること．
 *  (4) 本ソフトウェアの利用により直接的または間接的に生じるいかなる損
 *      害からも，上記著作権者およびTOPPERSプロジェクトを免責すること．
 *      また，本ソフトウェアのユーザまたはエンドユーザからのいかなる理
 *      由に基づく請求からも，上記著作権者およびTOPPERSプロジェクトを
 *      免責すること．
 * 
 *  本ソフトウェアは，無保証で提供されているものである．上記著作権者お
 *  よびTOPPERSプロジェクトは，本ソフトウェアに関して，特定の使用目的
 *  に対する適合性も含めて，いかなる保証も行わない．また，本ソフトウェ
 *  アの利用により直接的または間接的に生じたいかなる損害に関しても，そ
 *  の責任を負わない．
 * 
 *  $Id: esp32_socket.c 2416 2020-12-10 08:06:20Z fukuen $
 */

/* 
 *  ESP32_SOCKET プログラム
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "device.h"
#include "esp32_socket.h"

#define printk printf
#define TIMEOUT 1

#define HIGH 0x1
#define LOW  0x0

#define INPUT           0x0
#define OUTPUT          0x3
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0X1

#if defined(TOPPERS_BASE_PLATFORM_STF7)
#define i2c_send(h, a, b, l)    i2c_memwrite((h), (a), 0, 0, (b), (l), 500)
#define i2c_recv(h, a, b, l)    i2c_memread((h), (a), 0, 0, (b), (l), 500)
#else
#define i2c_send(h, a, b, l)    i2c_memwrite((h), (a), 0, 0, (b), (l))
#define i2c_recv(h, a, b, l)    i2c_memread((h), (a), 0, 0, (b), (l))
#endif

char ssid[32] = {0};
uint8_t mac[32] = {0};
esp32_net_t net_dat;
uint8_t cs_num, rst_num, rdy_num;
uint32_t time;

ESP32_Handle_t esp32Handle;

static esp32_params_t *esp32_params_alloc_1param(uint32_t len, uint8_t *buf);
static esp32_params_t *esp32_params_alloc_2param(uint32_t len_0, uint8_t *buf_0, uint32_t len_1, uint8_t *buf_1);
static int8_t esp32_send_command(ESP32_Handle_t *hesp, uint8_t cmd, esp32_params_t *params, uint8_t param_len_16);
static void delete_esp32_params(void *arg);

/*
*  ダイレクトデジタルピン設定
*/
void pinMode(uint8_t Pin, uint8_t dwMode)
{
	int gpionum = gpio_get_gpiohno(Pin, false);
	GPIO_Init_t init = {0};

	syslog_2(LOG_NOTICE, "## pinMode Pin(%d) gpionum(%d) ##", Pin, gpionum);
	if (gpionum >= 0)
	{
		uint8_t function = FUNC_GPIOHS0 + gpionum;
		fpioa_set_function(Pin, function);
		switch(dwMode){
		case INPUT:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_NOPULL;
			break;
		case INPUT_PULLDOWN:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_PULLDOWN;
			break;
		case INPUT_PULLUP:
			init.mode = GPIO_MODE_INPUT;
			init.pull = GPIO_PULLUP;
			break;
		case OUTPUT:
		default:
			init.mode = GPIO_MODE_OUTPUT;
			init.pull = GPIO_PULLDOWN;
			break;
		}
		gpio_setup(TADR_GPIOHS_BASE, &init, (uint8_t)gpionum);
	}
	return ;
}

/*
 *  ダイレクトデジタルピン出力
 */
void digitalWrite(uint8_t Pin, int dwVal)
{
	if (Pin == 0)
		return;

	int8_t gpio_pin = gpio_get_gpiohno(Pin, false);

	if (gpio_pin >= 0)
	{
		gpio_set_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin, dwVal);
	}
}

/*
 *  ダイレクトデジタルピン入力
 */
int digitalRead(uint8_t Pin)
{
	if (Pin == 0)
		return 0;

	int8_t gpio_pin = gpio_get_gpiohno(Pin, false);
	if (gpio_pin >= 0)
	{
		return (int)gpio_get_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin);
	}
	return -1;
}

/*
 *  ミリ秒取得
 */
uint32_t get_millis(void)
{
	SYSTIM tim;
	get_tim(&tim);
	return (uint32_t)(tim);
}

/*
 *  SPI 1 文字送受信
 */
uint8_t esp32_spi_rw(SPI_Handle_t *hspi, uint8_t data)
{
	uint8_t c;
	spi_esp32_transrecv(hspi, 0, &data, 1, &c, 1);
	spi_wait(hspi, 10);
	return c;
}

/*
 *  SPI 文字列送受信
 */
void esp32_spi_rw_len(SPI_Handle_t *hspi, uint8_t *send, uint8_t *recv, uint32_t len)
{
	if (send == NULL && recv == NULL)
	{
		printf(" buffer is null\r\n");
		return;
	}

	//only send
	if (send && recv == NULL)
	{
//	    spi_core_transmit(hspi, 0, send, len);
		spi_esp32_transrecv(hspi, 0, send, len, NULL, 0);
		spi_wait(hspi, 10);
		return;
	}

	//only recv
	if (send == NULL && recv)
	{
//	    spi_core_receive(hspi, 0, recv, (size_t)len);
//	    spi_esp32_transrecv(hspi, 0, NULL, 0, recv, len);
		spi_esp32_transrecv(hspi, 0, recv, len, recv, len);
		spi_wait(hspi, 10);
		return;
	}

	//send and recv
	if (send && recv)
	{
		spi_esp32_transrecv(hspi, 0, send, len, recv, len);
		spi_wait(hspi, 10);
		return;
	}
	return;
}

/*
 *  I2C 1 文字送受信
 */
uint8_t esp32_i2c_rw(I2C_Handle_t *hi2c, uint8_t data)
{
	uint8_t c;
	i2c_recv(hi2c, ESP32_ADDRESS, &c, 1);
	return c;
}

/*
 *  I2C 文字列送受信
 */
void esp32_i2c_rw_len(I2C_Handle_t *hi2c, uint8_t *send, uint8_t *recv, uint32_t len)
{
	if (send == NULL && recv == NULL)
	{
		printf(" buffer is null\r\n");
		return;
	}

	//only send
	if (send)
	{
		uint32_t txindex = 0;
		while (txindex < len)
		{
			for (int i = 0; i < 32; i++)
			{
				if (txindex > len)
					break;
				i2c_send(hi2c, ESP32_ADDRESS, &send[txindex], 1);
				txindex++;
			}
			dly_tsk(1);
		}
	}
	else
	{
	//only recv
		for (int i = 0; i < len; i++)
		{
			i2c_recv(hi2c, ESP32_ADDRESS, &recv[i], 1);
		}
	}
	return;
}

/*
 *  SPI/I2C 1 文字送受信
 */
uint8_t esp32_rw(ESP32_Handle_t *hesp, uint8_t data)
{
	if (hesp->hspi == NULL)
	{
		return esp32_i2c_rw(hesp->hi2c, data);
	}
	else
	{
		dly_tsk(1);
		return esp32_spi_rw(hesp->hspi, data);
	}
}

/*
 *  SPI/I2C 文字列送受信
 */
void esp32_rw_len(ESP32_Handle_t *hesp, uint8_t *send, uint8_t *recv, uint32_t len)
{
	if (hesp->hspi == NULL)
	{
		esp32_i2c_rw_len(hesp->hi2c, send, recv, len);
	}
	else
	{
		dly_tsk(1);
		esp32_spi_rw_len(hesp->hspi, send, recv, len);
	}
	return;
}

/*
 *  ESP32 ハード/ソフトリセット
 */
static void esp32_reset(ESP32_Handle_t *hesp)
{
	PRINTF1("Reset ESP32\r\n");

	//here we sleep 1s
	digitalWrite(cs_num, 1);

	if ((int8_t)rst_num > 0)
	{
		digitalWrite(rst_num, 0);
		dly_tsk(500);
		digitalWrite(rst_num, 1);
		dly_tsk(1000);
	}
	else
	{
		//soft reset
		esp32_send_command(hesp, SOFT_RESET_CMD, NULL, 0);
		dly_tsk(1500);
	}
}

/*
 *  SPI 接続の場合、レディピンが 0 になるのを待機
 *  0 OK
 *  -1 レスポンスなし
 */
int8_t esp32_wait_for_ready(ESP32_Handle_t *hesp)
{
	if (hesp->hspi == NULL)
		return 0;

	PRINTF3("Wait for ESP32 ready\r\n");

	uint32_t tm = get_millis();
	while ((get_millis() - tm) < 10 * 1000) //10s
	{
		if (digitalRead(rdy_num) == 0)
			return 0;

		PRINTF3(".");
		dly_tsk(1);
	}

	PRINTF3("esp32 not responding\r\n");

	return -1;
}

/*
 *  SPI 接続の場合、レディピンが 1 になるのを待機
 *  0 OK
 *  -1 レスポンスなし
 */
int8_t esp32_wait_for_active(ESP32_Handle_t *hesp)
{
	if (hesp->hspi == NULL)
		return 0;

	PRINTF3("Wait for ESP32 active\r\n");

	uint32_t tm = get_millis();
	while ((get_millis() - tm) < 1000) //1s
	{
		if (digitalRead(rdy_num) == 1)
			return 0;

		PRINTF3(".");
		dly_tsk(1); //FIXME
	}

	PRINTF3("esp32 not responding\r\n");

	return -1;
}

#define lc_buf_len 256
uint8_t lc_send_buf[lc_buf_len];
uint8_t lc_buf_flag = 0;

/*
 *  ESP32 にパラメータを含めてコマンドを送信
 *  0 OK
 *  -1 エラー
 */
static int8_t esp32_send_command(ESP32_Handle_t *hesp, uint8_t cmd, esp32_params_t *params, uint8_t param_len_16)
{
	uint32_t packet_len = 0;

	packet_len = 4; // header + end byte
	if (params != NULL)
	{
		for (uint32_t i = 0; i < params->params_num; i++)
		{
			packet_len += params->params[i]->param_len;
			packet_len += 1; // size byte
			if (param_len_16)
				packet_len += 1;
		}
	}
	while (packet_len % 4 != 0)
		packet_len += 1;

	uint8_t *sendbuf = NULL;

	if (packet_len > lc_buf_len)
	{
		sendbuf = (uint8_t *)malloc(sizeof(uint8_t) * packet_len);
		lc_buf_flag = 0;
		if (!sendbuf)
		{
			PRINTF11("%s: malloc error\r\n", __func__);
			return -1;
		}
	}
	else
	{
		sendbuf = lc_send_buf;
		lc_buf_flag = 1;
	}

	sendbuf[0] = START_CMD;
	sendbuf[1] = cmd & ~REPLY_FLAG;
	if (params != NULL)
		sendbuf[2] = params->params_num;
	else
		sendbuf[2] = 0;

	uint32_t ptr = 3;

	if (params != NULL)
	{
		//handle parameters here
		for (uint32_t i = 0; i < params->params_num; i++)
		{
			PRINTF21("\tSending param #%d is %d bytes long\r\n", i, params->params[i]->param_len);

			if (param_len_16)
			{
				sendbuf[ptr] = (uint8_t)((params->params[i]->param_len >> 8) & 0xFF);
				ptr += 1;
			}
			sendbuf[ptr] = (uint8_t)(params->params[i]->param_len & 0xFF);
			ptr += 1;
			memcpy(sendbuf + ptr, params->params[i]->param, params->params[i]->param_len);
			ptr += params->params[i]->param_len;
		}
	}
	sendbuf[ptr] = END_CMD;

	esp32_wait_for_ready(hesp);
	digitalWrite(cs_num, 0);

	uint32_t tm = get_millis();
	esp32_wait_for_active(hesp);

	if ((get_millis() - tm) > 1000)
	{
		PRINTF1("ESP32 timed out on SPI select\r\n");
		digitalWrite(cs_num, 1);
		if (lc_buf_flag == 0)
		{
			free(sendbuf);
			sendbuf = NULL;
		}
		else
		{
			memset(sendbuf, 0, packet_len);
		}
		return -1;
	}

	esp32_rw_len(hesp, sendbuf, NULL, packet_len);
	digitalWrite(cs_num, 1);

#if (ESP32_DEBUG >= 3)
	if (packet_len < 100)
	{
		printk("Wrote buf packet_len --> %d: ", packet_len);
		for (uint32_t i = 0; i < packet_len; i++)
			printk("%02x ", sendbuf[i]);
		printk("\r\n");
	}
#endif
	if (lc_buf_flag == 0)
	{
		free(sendbuf);
		sendbuf = NULL;
	}
	else
	{
		memset(sendbuf, 0, packet_len);
	}
	return 0;
}

/*
 *  ESP32 から 1 バイト受信
 *  戻り データ
 */
uint8_t esp32_read_byte(ESP32_Handle_t *hesp)
{
	uint8_t read = 0x0;

	read = esp32_rw(hesp, 0xff);

	PRINTF31("\t\tRead:%02x\r\n", read);

	return read;
}

/*
 *  ESP32 から複数バイト受信
 */
void esp32_read_bytes(ESP32_Handle_t *hesp, uint8_t *buffer, uint32_t len)
{
	esp32_rw_len(hesp, NULL, buffer, len);

#if (ESP32_DEBUG >= 3)
	if (len < 100)
	{
		printk("\t\tRead:");
		for (uint32_t i = 0; i < len; i++)
			printk("%02x ", *(buffer + i));
		printk("\r\n");
	}
#endif
}

/*
 *  ESP32 から特定の文字が受信されるまで待機、違う文字はスキップする
 *  0 OK
 *  -1 エラー、タイムアウト
 */
int8_t esp32_wait_spc_char(ESP32_Handle_t *hesp, uint8_t want)
{
	uint8_t read = 0x0;
	uint32_t tm = get_millis();

	while ((get_millis() - tm) < 100)
	{
		read = esp32_read_byte(hesp);

		if (read == ERR_CMD)
		{
			PRINTF1("Error response to command\r\n");
			return -1;
		}
		else if (read == want)
			return 0;
	}

	return -1;
}

/*
 *  ESP32 から特定の 1 文字を受信、一致しなければエラー
 *  0 OK
 *  -1 エラー
 */
uint8_t esp32_check_data(ESP32_Handle_t *hesp, uint8_t want)
{
	uint8_t read = esp32_read_byte(hesp);

	if (read != want)
	{
		PRINTF11("Expected %02X but got %02X\r\n", want, read);
		return -1;
	}
	return 0;
}

/*
 *  ESP32 からのレスポンスを受信してパラメータ解析
 *  NULL エラー
 */
esp32_params_t *esp32_wait_response_cmd(ESP32_Handle_t *hesp, uint8_t cmd, uint32_t *num_responses, uint8_t param_len_16)
{
	uint32_t num_of_resp = 0;

	esp32_wait_for_ready(hesp);

	digitalWrite(cs_num, 0);

	uint32_t tm = get_millis();
	esp32_wait_for_active(hesp);

	if ((get_millis() - tm) > 1000)
	{
		PRINTF1("ESP32 timed out on SPI select\r\n");
		digitalWrite(cs_num, 1);
		return NULL;
	}

	// wait DUMMY_CMD when I2C
	if (hesp->hspi == NULL)
	{
		if (esp32_wait_spc_char(hesp, DUMMY_CMD) != 0)
		{
			PRINTF1("esp32_wait_spc_char DUMMY_CMD error\r\n");
			digitalWrite(cs_num, 1);
			return NULL;
		}
	}

	if (esp32_wait_spc_char(hesp, START_CMD) != 0)
	{
		PRINTF1("esp32_wait_spc_char START_CMD error\r\n");
		digitalWrite(cs_num, 1);
		return NULL;
	}

	if (esp32_check_data(hesp, cmd | REPLY_FLAG) != 0)
	{
		PRINTF1("esp32_check_data cmd | REPLY_FLAG error\r\n");
		digitalWrite(cs_num, 1);
		return NULL;
	}

	if (num_responses)
	{
		if (esp32_check_data(hesp, *num_responses) != 0)
		{
			PRINTF1("esp32_check_data num_responses error\r\n");
			digitalWrite(cs_num, 1);
			return NULL;
		}
		num_of_resp = *num_responses;
	}
	else
	{
		num_of_resp = esp32_read_byte(hesp);
	}

	esp32_params_t *params_ret = (esp32_params_t *)malloc(sizeof(esp32_params_t));

	params_ret->del = delete_esp32_params;
	params_ret->params_num = num_of_resp;
	params_ret->params = (void *)malloc(sizeof(void *) * num_of_resp);

	for (uint32_t i = 0; i < num_of_resp; i++)
	{
		params_ret->params[i] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		params_ret->params[i]->param_len = esp32_read_byte(hesp);

		if (param_len_16)
		{
			params_ret->params[i]->param_len <<= 8;
			params_ret->params[i]->param_len |= esp32_read_byte(hesp);
		}

		PRINTF21("\tParameter #%d length is %d\r\n", i, params_ret->params[i]->param_len);

		params_ret->params[i]->param = (uint8_t *)malloc(sizeof(uint8_t) * params_ret->params[i]->param_len);
		esp32_read_bytes(hesp, params_ret->params[i]->param, params_ret->params[i]->param_len);
	}

	if (esp32_check_data(hesp, END_CMD) != 0)
	{
		PRINTF1("esp32_check_data END_CMD error\r\n");
		digitalWrite(cs_num, 1);
		return NULL;
	}

	digitalWrite(cs_num, 1);

	return params_ret;
}

/*
 *  ESP32 にコマンド送信およびレスポンスを受信
 *  NULL エラー
 */
esp32_params_t *esp32_send_command_get_response(ESP32_Handle_t *hesp, uint8_t cmd, esp32_params_t *params, uint32_t *num_resp, uint8_t sent_param_len_16, uint8_t recv_param_len_16)
{
	uint32_t resp_num;

	if (!num_resp)
		resp_num = 1;
	else
		resp_num = *num_resp;

	esp32_send_command(hesp, cmd, params, sent_param_len_16);
	return esp32_wait_response_cmd(hesp, cmd, &resp_num, recv_param_len_16);
}

/*
 *  パラメータメモリ開放
 */
static void delete_esp32_params(void *arg)
{
	esp32_params_t *params = (esp32_params_t *)arg;

	for (uint8_t i = 0; i < params->params_num; i++)
	{
		esp32_param_t *param = params->params[i];
		free(param->param);
		free(param);
	}
	free(params->params);
	free(params);
}

/*
 *  パラメータの構築 1 パラメータ
 */
static esp32_params_t *esp32_params_alloc_1param(uint32_t len, uint8_t *buf)
{
	esp32_params_t *ret = (esp32_params_t *)malloc(sizeof(esp32_params_t));

	ret->del = delete_esp32_params;

	ret->params_num = 1;
	ret->params = (void *)malloc(sizeof(void *) * ret->params_num);
	ret->params[0] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
	ret->params[0]->param_len = len;
	ret->params[0]->param = (uint8_t *)malloc(sizeof(uint8_t) * len);
	memcpy(ret->params[0]->param, buf, len);

	return ret;
}

/*
 *  パラメータの構築 2 パラメータ
 */
static esp32_params_t *esp32_params_alloc_2param(uint32_t len_0, uint8_t *buf_0, uint32_t len_1, uint8_t *buf_1)
{
	esp32_params_t *ret = (esp32_params_t *)malloc(sizeof(esp32_params_t));

	ret->del = delete_esp32_params;

	ret->params_num = 2;
	ret->params = (void *)malloc(sizeof(void *) * ret->params_num);
	//
	ret->params[0] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
	ret->params[0]->param_len = len_0;
	ret->params[0]->param = (uint8_t *)malloc(sizeof(uint8_t) * len_0);
	memcpy(ret->params[0]->param, buf_0, len_0);
	//
	ret->params[1] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
	ret->params[1]->param_len = len_1;
	ret->params[1]->param = (uint8_t *)malloc(sizeof(uint8_t) * len_1);
	memcpy(ret->params[1]->param, buf_1, len_1);

	return ret;
}

/*
 *  ESP32 ファームウェアバージョンの取得
 */
ER esp32_firmware_version(ESP32_Handle_t *hesp, char* fw_version)
{
	PRINTF1("Firmware version\r\n");

	esp32_params_t *resp = esp32_send_command_get_response(hesp, GET_FW_VERSION_CMD, NULL, NULL, 0, 0);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return E_SYS;
	}

	uint8_t ret_len = resp->params[0]->param_len;
	memcpy(fw_version, resp->params[0]->param, ret_len);
	fw_version[ret_len] = 0;

	resp->del(resp);
	return E_OK;
}

/*
 *  ESP32 のデバッグを有効にする
 */
void esp32_set_debug(ESP32_Handle_t *hesp, uint8_t debug)
{
	esp32_params_t *send = esp32_params_alloc_1param(1, &debug);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, SET_DEBUG_CMD, send, NULL, 0, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
	}
	resp->del(resp);
}
/*
 *  時刻取得
 */
uint32_t esp32_get_time(ESP32_Handle_t *hesp)
{
	uint8_t data = 0xff;

	esp32_params_t *send = esp32_params_alloc_1param(1, &data);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, GET_TIME_CMD, send, NULL, 0, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return 0;
	}

	uint8_t ret_len = resp->params[0]->param_len;
	memcpy(&time, resp->params[0]->param, ret_len);

	resp->del(resp);
	return time;
}

/*
 *  証明書設定
 */
void esp32_set_certificate(ESP32_Handle_t *hesp, char *client_ca)
{
	esp32_params_t *send = esp32_params_alloc_1param(strlen((const char*)client_ca), (uint8_t *)client_ca);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, SET_CLIENT_CERT_CMD, send, NULL, 1, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
	}
	resp->del(resp);
}

/*
 *  プライベートキー設定
 */
void esp32_set_private_key(ESP32_Handle_t *hesp, char *private_key)
{
	esp32_params_t *send = esp32_params_alloc_1param(strlen((const char*)private_key), (uint8_t *)private_key);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, SET_CERT_KEY_CMD, send, NULL, 1, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
	}
	resp->del(resp);
}

/*
 *  アクセスポイントの SSID を設定
 *  0 OK
 *  -1 エラー
 */
int8_t esp32_wifi_set_network(ESP32_Handle_t *hesp, uint8_t *ssid)
{
	esp32_params_t *send = esp32_params_alloc_1param(strlen((const char*)ssid), ssid);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, SET_NET_CMD, send, NULL, 0, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return -1;
	}

	if (resp->params[0]->param[0] != 1)
	{
		PRINTF1("Failed to set network\r\n");
		resp->del(resp);
		return -1;
	}

	resp->del(resp);

	return 0;
}

/*
 *  アクセスポイントの SSID とパスフレーズを設定
 *  0 OK
 *  -1 エラー
 */
int8_t esp32_wifi_set_passphrase(ESP32_Handle_t *hesp, uint8_t *ssid, uint8_t *passphrase)
{
	esp32_params_t *send = esp32_params_alloc_2param(strlen((const char*)ssid), ssid, strlen((const char*)passphrase), passphrase);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, SET_PASSPHRASE_CMD, send, NULL, 0, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return -1;
	}

	if (resp->params[0]->param[0] != 1)
	{
		PRINTF11("%s: Failed to set passphrase\r\n", __func__);
		resp->del(resp);
		return -1;
	}

	resp->del(resp);
	return 0;
}

/*
 *  WIFI アクセスポイントへの接続状態
 *  -1 エラー
 *  その他 WL_NO_SHIELD, WL_NO_MODULE (なし)
 *        WL_IDLE_STATUS, WL_NO_SSID_AVAIL, WL_SCAN_COMPLETED,
 *        WL_CONNECTED, WL_CONNECT_FAILED, WL_CONNECTION_LOST, WL_DISCONNECTED,
 *        WL_AP_LISTENING, WL_AP_CONNECTED, WL_AP_FAILED
 */
int8_t esp32_status(ESP32_Handle_t *hesp)
{
	PRINTF2("Connection status\r\n");

	esp32_params_t *resp = esp32_send_command_get_response(hesp, GET_CONN_STATUS_CMD, NULL, NULL, 0, 0);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return -2;
	}
	int8_t ret = (int8_t)resp->params[0]->param[0];

	PRINTF11("Conn Connection: %s\r\n", wlan_enum_to_str(ret));

	resp->del(resp);
	return ret;
}

/*
 *  アクセスポイントへの接続状況
 *  0 接続なし
 *  1 接続中
 */
uint8_t esp32_is_connected(ESP32_Handle_t *hesp)
{
	int8_t stat = esp32_status(hesp);

	if (stat == -2)
	{
		PRINTF11("%s get status error \r\n", __func__);
		esp32_reset(hesp);
		return 2;
	}
	else if (stat == WL_CONNECTED)
		return 0;

	return 1;
}

/*
 *  アクセスポイントへの接続
 *  0 接続成功
 *  -1 接続失敗
 */
int8_t esp32_connect(ESP32_Handle_t *hesp)
{
	PRINTF11("Connect to AP--> ssid: %s password:%s\r\n", hesp->Init.ssid, hesp->Init.pass);
	if (hesp->Init.pass)
		esp32_wifi_set_passphrase(hesp, (uint8_t *)hesp->Init.ssid, (uint8_t *)hesp->Init.pass);
	else
		esp32_wifi_set_network(hesp, (uint8_t *)hesp->Init.ssid);

	int8_t stat = -1;

	for (uint8_t i = 0; i < hesp->Init.retry_times; i++)
	{
		stat = esp32_status(hesp);

		if (stat == -1)
		{
			PRINTF11("%s get status error \r\n", __func__);
			esp32_reset(hesp);
			return -1;
		}
		else if (stat == WL_CONNECTED)
			return 0;
		else if (stat == WL_CONNECT_FAILED)
			return -2;
		dly_tsk(1000);
	}
	stat = esp32_status(hesp);

	if (stat == WL_CONNECT_FAILED || stat == WL_CONNECTION_LOST || stat == WL_DISCONNECTED)
	{
		PRINTF11("Failed to connect to ssid: %s\r\n", ssid);

		return -3;
	}

	if (stat == WL_NO_SSID_AVAIL)
	{
		PRINTF11("No such ssid: %s\r\n", ssid);

		return -4;
	}

	PRINTF11("Unknown error 0x%02X", stat);

	return -5;
}

/*
 *  アクセスポイントから切断
 */
int8_t esp32_disconnect(ESP32_Handle_t *hesp)
{
	esp32_params_t *resp = esp32_send_command_get_response(hesp, DISCONNECT_CMD, NULL, NULL, 0, 0);
	if (resp == NULL)
	{
		return -1;
	}
	int8_t ret = (int8_t)resp->params[0]->param[0];
	resp->del(resp);
	return ret;
}

/*
 *  ホスト名からIPアドレス取得
 *  0 OK
 *  -1 エラー
 */
int esp32_get_host_by_name(ESP32_Handle_t *hesp, uint8_t *hostname, uint8_t *ip)
{
	PRINTF1("*** Get host by name\r\n");

	esp32_params_t *send = esp32_params_alloc_1param(strlen((const char*)hostname), hostname);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, REQ_HOST_BY_NAME_CMD, send, NULL, 0, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
//        return EIO;
		return -2;
	}

	if (resp->params[0]->param[0] != 1)
	{
		PRINTF1("Failed to request hostname\r\n");
		resp->del(resp);
//        return EINVAL;
		return -1;
	}
	resp->del(resp);

	resp = esp32_send_command_get_response(hesp, GET_HOST_BY_NAME_CMD, NULL, NULL, 0, 0);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
//        return EIO;
		return -2;
	}

#if (ESP32_DEBUG >= 2)
	printk("get_host_by_name:%s-->", hostname);
	for (uint8_t i = 0; i < resp->params[0]->param_len; i++)
	{
		printk("%d", resp->params[0]->param[i]);
		if (i < resp->params[0]->param_len - 1)
			printk(".");
	}
	printk("\r\n");
#endif

	memcpy(ip, resp->params[0]->param, resp->params[0]->param_len);
	resp->del(resp);

	return 0;
}

/*
 *  ESP32 から空ソケット番号を取得
 *  0xff エラー
 *  その他 OK
 */
uint8_t esp32_get_socket(ESP32_Handle_t *hesp)
{
	PRINTF1("*** Get socket\r\n");
	esp32_params_t *resp = esp32_send_command_get_response(hesp, GET_SOCKET_CMD, NULL, NULL, 0, 0);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return 0xff;
	}

	uint8_t socket = resp->params[0]->param[0];

	if (socket == 255)
	{
		PRINTF1("No sockets available\r\n");
		resp->del(resp);
		return 0xff;
	}

	PRINTF11("Allocated socket #%d\r\n", socket);

	resp->del(resp);
	return (int16_t)socket;
}

/*
 *  宛先の IP アドレスまたはホスト名を指定してソケットをオープン
 *  宛先タイプ 0 IP アドレス
 *           1 ホスト名 (TCP_MODEではホスト名ではエラーになる)
 *  0 OK
 *  -1 エラー
 */
int8_t esp32_socket_open(ESP32_Handle_t *hesp, uint8_t *dest, uint8_t dest_type, uint16_t port, esp32_socket_mode_enum_t conn_mode)
{
	uint8_t port_arr[2];

	port_arr[0] = (uint8_t)(port >> 8);
	port_arr[1] = (uint8_t)(port);

	PRINTF21("port: 0x%02x 0x%02x\r\n", port_arr[0], port_arr[1]);

	esp32_params_t *send = (esp32_params_t *)malloc(sizeof(esp32_params_t));
	send->del = delete_esp32_params;

	uint32_t param_len = 0;

	if (dest_type)
	{
		send->params_num = 5;
		send->params = (void *)malloc(sizeof(void *) * send->params_num);
		//
		param_len = strlen((const char*)dest);
		send->params[0] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[0]->param_len = param_len;
		send->params[0]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		memcpy(send->params[0]->param, dest, param_len);
		//
		param_len = 4;
		send->params[1] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[1]->param_len = param_len;
		send->params[1]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		memset(send->params[1]->param, 0, param_len);
		//
		param_len = 2;
		send->params[2] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[2]->param_len = param_len;
		send->params[2]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		memcpy(send->params[2]->param, port_arr, param_len);
		//
		param_len = 1;
		send->params[3] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[3]->param_len = param_len;
		send->params[3]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		send->params[3]->param[0] = hesp->socket_num;
		//
		param_len = 1;
		send->params[4] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[4]->param_len = param_len;
		send->params[4]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		send->params[4]->param[0] = conn_mode;
	}
	else
	{
		send->params_num = 4;
		send->params = (void *)malloc(sizeof(void *) * send->params_num);
		//
		param_len = 4;
		send->params[0] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[0]->param_len = param_len;
		send->params[0]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		memcpy(send->params[0]->param, dest, param_len);
		//
		param_len = 2;
		send->params[1] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[1]->param_len = param_len;
		send->params[1]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		memcpy(send->params[1]->param, port_arr, param_len);
		//
		param_len = 1;
		send->params[2] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[2]->param_len = param_len;
		send->params[2]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		send->params[2]->param[0] = hesp->socket_num;
		//
		param_len = 1;
		send->params[3] = (esp32_param_t *)malloc(sizeof(esp32_param_t));
		send->params[3]->param_len = param_len;
		send->params[3]->param = (uint8_t *)malloc(sizeof(uint8_t) * param_len);
		send->params[3]->param[0] = conn_mode;
	}

	esp32_params_t *resp = esp32_send_command_get_response(hesp, START_CLIENT_TCP_CMD, send, NULL, 0, 0);

	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return -1;
	}

	if (resp->params[0]->param[0] != 1)
	{
		PRINTF1("Could not connect to remote server\r\n");
		resp->del(resp);

		return -1;
	}
	resp->del(resp);
	return 0;
}

/*
 *  ソケットの接続状態
 *  -1 エラー
 *  その他 SOCKET_CLOSED, SOCKET_LISTEN,
 *        SOCKET_SYN_SENT, SOCKET_SYN_RCVD, SOCKET_ESTABLISHED, SOCKET_FIN_WAIT_1,
 *        SOCKET_FIN_WAIT_2, SOCKET_CLOSE_WAIT, SOCKET_CLOSING, SOCKET_LAST_ACK,
 *        SOCKET_TIME_WAIT
 */
esp32_socket_enum_t esp32_socket_status(ESP32_Handle_t *hesp)
{
	esp32_params_t *send = esp32_params_alloc_1param(1, &(hesp->socket_num));
	esp32_params_t *resp = esp32_send_command_get_response(hesp, GET_CLIENT_STATE_TCP_CMD, send, NULL, 0, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return 0xff;
	}
	esp32_socket_enum_t ret;

	ret = (esp32_socket_enum_t)resp->params[0]->param[0];

	resp->del(resp);

	PRINTF21("sock stat :%d\r\n", ret);
	return ret;
}

/*
 *  ソケットの接続状態
 *  0 接続なし
 *  1 接続中
 */
uint8_t esp32_socket_connected(ESP32_Handle_t *hesp)
{
	return (esp32_socket_status(hesp) == SOCKET_ESTABLISHED);
}

/*
 *  ソケットに書き込み
 *  0 エラー
 *  送信長 OK
 */
uint32_t esp32_socket_write(ESP32_Handle_t *hesp, uint8_t *buffer, uint16_t len)
{
	esp32_params_t *send = esp32_params_alloc_2param(1, &(hesp->socket_num), len, buffer);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, SEND_DATA_TCP_CMD, send, NULL, 1, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return 0;
	}
	uint16_t sent = ( ((uint16_t)(resp->params[0]->param[1]) << 8) & 0xff00 ) | (uint16_t)(resp->params[0]->param[0]);
	// if (sent != len) //TODO: the firmware is nonblock, so return value maybe < len
	if (sent == 0)
	{
		PRINTF11("Failed to send %d bytes (sent %d)", len, sent);
		resp->del(resp);
		return 0;
	}

	resp->del(resp);
	return sent;
}

/*
 *  ソケットから受信できるデータ長
 */
int esp32_socket_available(ESP32_Handle_t *hesp)
{
	esp32_params_t *send = esp32_params_alloc_1param(1, &(hesp->socket_num));
	esp32_params_t *resp = esp32_send_command_get_response(hesp, AVAIL_DATA_TCP_CMD, send, NULL, 0, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return -1;
	}

	int reply = 0;

	reply = (int)((uint16_t)(resp->params[0]->param[1] << 8) | (uint16_t)(resp->params[0]->param[0]));

	if(reply > 0)
	{
		PRINTF11("ESPSocket: %d bytes available\r\n", reply);
	}

	resp->del(resp);
	return reply;
}

/*
 *  ソケットから読み取り
 */
int esp32_socket_read(ESP32_Handle_t *hesp, uint8_t *buff, uint16_t size)
{
	PRINTF11("Reading %d bytes from ESP socket with status %s\r\n", size, socket_enum_to_str(esp32_socket_status(hesp)));

	uint8_t len[2];

	len[0] = (uint8_t)(size & 0xff);
	len[1] = (uint8_t)((size >> 8) & 0xff);

	PRINTF31("len_0:%02x\tlen_1:%02x\r\n", len[0], len[1]);

	esp32_params_t *send = esp32_params_alloc_2param(1, &(hesp->socket_num), 2, len);
	esp32_params_t *resp = esp32_send_command_get_response(hesp, GET_DATABUF_TCP_CMD, send, NULL, 1, 1);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return -1;
	}

	uint16_t real_read_size = resp->params[0]->param_len;

	if (real_read_size != 0)
		memcpy(buff, resp->params[0]->param, real_read_size);
	resp->del(resp);

	return real_read_size;
}

/*
 *  宛先の IP アドレスまたはホスト名を指定してソケットをオープン
 *  宛先タイプ 0 IP アドレス
 *           1 ホスト名 (TCP_MODEではホスト名ではエラーになる)
 *  0 OK
 *  -1 エラー
 */
int8_t esp32_socket_connect(ESP32_Handle_t *hesp, uint8_t *dest, uint8_t dest_type, uint16_t port, esp32_socket_mode_enum_t conn_mod)
{
	PRINTF11("*** Socket connect mode:%d\r\n", conn_mod);
	int8_t ret = esp32_socket_open(hesp, dest, dest_type, port, conn_mod);
	if ( ret == -2 )
	{
		return -2;
	}
	if(ret == -1)
	{
		return -1;
	}
	if(conn_mod == UDP_MODE)
		return 0;
	uint32_t tm = get_millis();

	while ((get_millis() - tm) < 3 * 1000) //3s
	{
		uint8_t ret = esp32_socket_status(hesp);
		if (ret == SOCKET_ESTABLISHED)
			return 0;
		else if(ret == 0xff) // EIO
		{
			return -2;
		}
		dly_tsk(100);
	}
	return -3;
}

/*
 *  ソケットのクローズ
 *  0 OK
 *  -1 エラー
 */
int8_t esp32_socket_close(ESP32_Handle_t *hesp)
{
	esp32_params_t *send = esp32_params_alloc_1param(1, &(hesp->socket_num));
	esp32_params_t *resp = esp32_send_command_get_response(hesp, STOP_CLIENT_TCP_CMD, send, NULL, 0, 0);
	send->del(send);

	if (resp == NULL)
	{
		PRINTF11("%s: get resp error!\r\n", __func__);
		return -1;
	}

	if (resp->params[0]->param[0] != 1)
	{

		PRINTF1("Failed to close socket\r\n");
		resp->del(resp);
		return -1;
	}
	resp->del(resp);
	return 0;
}

#define ENUM_TO_STR(x) \
	case (x):          \
		return (#x)

char *socket_enum_to_str(esp32_socket_enum_t x)
{
	switch (x)
	{
		ENUM_TO_STR(SOCKET_CLOSED);
		ENUM_TO_STR(SOCKET_LISTEN);
		ENUM_TO_STR(SOCKET_SYN_SENT);
		ENUM_TO_STR(SOCKET_SYN_RCVD);
		ENUM_TO_STR(SOCKET_ESTABLISHED);
		ENUM_TO_STR(SOCKET_FIN_WAIT_1);
		ENUM_TO_STR(SOCKET_FIN_WAIT_2);
		ENUM_TO_STR(SOCKET_CLOSE_WAIT);
		ENUM_TO_STR(SOCKET_CLOSING);
		ENUM_TO_STR(SOCKET_LAST_ACK);
		ENUM_TO_STR(SOCKET_TIME_WAIT);
	}
	return "unknown";
}

char *wlan_enum_to_str(esp32_wlan_enum_t x)
{
	switch (x)
	{
		ENUM_TO_STR(WL_IDLE_STATUS);
		ENUM_TO_STR(WL_NO_SSID_AVAIL);
		ENUM_TO_STR(WL_SCAN_COMPLETED);
		ENUM_TO_STR(WL_CONNECTED);
		ENUM_TO_STR(WL_CONNECT_FAILED);
		ENUM_TO_STR(WL_CONNECTION_LOST);
		ENUM_TO_STR(WL_DISCONNECTED);
		ENUM_TO_STR(WL_AP_LISTENING);
		ENUM_TO_STR(WL_AP_CONNECTED);
		ENUM_TO_STR(WL_AP_FAILED);
		ENUM_TO_STR(WL_NO_SHIELD);
		ENUM_TO_STR(WL_NO_MODULE);
	}
	return "unknown";
}

/*
 *  ESP32 初期設定
 *  parameter1  init: ESP32 初期設定構造体へのポインタ
 *  return ESP32 ハンドラへのポインタ、NULL でエラー
 */
ESP32_Handle_t * esp32_init(ESP32_Init_t *init)
{
	ESP32_Handle_t *hesp;

	if(init == NULL)
		return NULL;

	hesp = &esp32Handle;
	memcpy(&hesp->Init, init, sizeof(ESP32_Init_t));

	hesp->hspi = init->hspi;
	hesp->hi2c = init->hi2c;
	hesp->connection = WL_IDLE_STATUS;
	hesp->status = SOCKET_CLOSED;
	hesp->socket_num = 0xff;
 
	cs_num = 0;
	rst_num = 0;
	rdy_num = 0;

	if (init->hspi == NULL)
	{
		if (init->hi2c == NULL)
			return NULL;
		PRINTF1("ESP32 USE I2C INTERFACE\r\n");
	}
	else
	{
		PRINTF1("ESP32 USE SPI INTERFACE\r\n");

		cs_num = init->cs_num;
		rst_num = init->rst_num;
		rdy_num = init->rdy_num;

		//cs
		pinMode(cs_num, OUTPUT);
		digitalWrite(cs_num, HIGH);

		//ready
		pinMode(rdy_num, INPUT);

		if ((int8_t)rst_num > 0)
		{
			pinMode(rst_num, OUTPUT);
		}
	}
	
	esp32_reset(hesp);

	return hesp;
}

/*
 *  ESP32 終了設定
 *  parameter1  hesp: ESP32 ハンドラへのポインタ
 *  return ERコード
 */
ER esp32_deinit(ESP32_Handle_t *hesp)
{
	if(hesp == NULL)
		return E_PAR;

	hesp->connection = WL_IDLE_STATUS;
	hesp->status = SOCKET_CLOSED;
	hesp->socket_num = 0xff;
	return E_OK;
}
