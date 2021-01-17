/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  $Id: wifitest.c 2416 2020-12-03 08:06:20Z fukuen $
 */

/* 
 *  WIFIテストの本体
 */

#include <kernel.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <sil.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "kernel_cfg.h"
#include <target_syssvc.h>
#include <string.h>
#include <stdio.h>
#include "device.h"
#include "spi.h"
#include "esp32_socket.h"
#include "wifitest.h"

/*
 *  サービスコールのエラーのログ出力
 */
Inline void
svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

#define SWAP_16(x) ((x >> 8 & 0xff) | (x << 8))

static uint32_t heap_area[2*512*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*2*512*1024)
};


/*
 *  メインタスク
 */
void main_task(intptr_t exinf)
{
	SPI_Init_t Init;
	SPI_Handle_t *hspi;
	ESP32_Init_t esp32_initd;
	ESP32_Handle_t *hesp;
	ER_UINT	ercd;

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Sample program starts (exinf = %d).", (int_t) exinf);

	/*
	 *  シリアルポートの初期化
	 *
	 *  システムログタスクと同じシリアルポートを使う場合など，シリアル
	 *  ポートがオープン済みの場合にはここでE_OBJエラーになるが，支障は
	 *  ない．
	 */
	ercd = serial_opn_por(TASK_PORTID);
	if (ercd < 0 && MERCD(ercd) != E_OBJ) {
		syslog(LOG_ERROR, "%s (%d) reported by `serial_opn_por'.",
									itron_strerror(ercd), SERCD(ercd));
	}
	SVC_PERROR(serial_ctl_por(TASK_PORTID,
							(IOCTL_CRLF | IOCTL_FCSND | IOCTL_FCRCV)));

	printf("WIFI TEST START\n");

	/* Set the SPI parameters */
	Init.WorkMode     = SPI_WORK_MODE_0;
	Init.FrameFormat  = SPI_FF_STANDARD;
	Init.DataSize     = 8;
	Init.Prescaler    = 9000000;
	Init.SignBit      = 0;
	Init.InstLength   = 0;
	Init.AddrLength   = 0;
	Init.WaitCycles   = 0;
	Init.IATransMode  = SPI_AITM_STANDARD;
	Init.SclkPin      = SPI_SCK_PIN;
	Init.MosiPin      = SPI_MOSI_PIN;
	Init.MisoPin      = SPI_MISO_PIN;
	Init.SsPin        = -1;
	Init.SsNo         = -1;
	Init.TxDMAChannel = -1;
	Init.RxDMAChannel = -1; //SPI_DMA1_CH;
	Init.semid        = SPI2TRN_SEM;
	Init.semlock      = SPI2LOCK_SEM;
	Init.semdmaid     = SPI2DMARX_SEM;
	hspi = spi_init(SPI2_PORTID, &Init);
	if(hspi == NULL){
		syslog_0(LOG_ERROR, "SPI INIT ERROR");
		slp_tsk();
	}

	/* Set the ESP32 parameters */
	esp32_initd.ssid = "aterm-a7acc0-g";
	esp32_initd.pass = "0c3e7d989e4af";
	esp32_initd.retry_times = 10;
	esp32_initd.cs_num = 25;
	esp32_initd.rst_num = 8;
	esp32_initd.rdy_num = 9;
	esp32_initd.hspi = hspi;
	esp32_initd.hi2c = NULL;
	hesp = esp32_init(&esp32_initd);
	if(hesp == NULL){
		syslog_0(LOG_ERROR, "ESP32 INIT ERROR");
		slp_tsk();
	}

	/* ファームウェアバージョン取得 */
	char fw_version[6];
	esp32_firmware_version(hesp, fw_version);
	printf("Firmware version %s\n", fw_version);

	/* WiFi アクセスポイントに接続 */
	printf("Connecting");
	int status = esp32_connect(hesp);
	while (status != WL_CONNECTED)
	{
		dly_tsk(1000);
		if (status == WL_CONNECT_FAILED)
		{
			status = esp32_connect(hesp);
		}
		else
		{
			status = esp32_status(hesp);
		}
		
		printf(".");
	}
	printf("\nConnected.\n");

	/* サーバーに接続 */
	printf("Connecting to server\n");
	int8_t ip[4];
	status = esp32_get_host_by_name(hesp, "arduino.cc", ip);

	hesp->socket_num = 0; // 0 固定
	status = esp32_socket_connect(hesp, ip, 0, 80, TCP_MODE);
	if (status != 0)
	{
		printf("Connection failed. %d\n", status);
	}

	/* HTTP GET */
	printf("Requesting to server\n");
	char buf[100] = "GET /asciilogo.txt HTTP/1.1\nHost: arduino.cc\nConnection: close\n\n";
	status = esp32_socket_write(hesp, buf, strlen(buf));

	/* レスポンス取得 */
	printf("Waiting response\n");
	while (esp32_socket_connected(hesp))
	{
		int len = esp32_socket_available(hesp);
		if (len > 0)
		{
//			char c[1];
//			status = esp32_socket_read(socket, c, 1);
//			printf(c);

			memset(buf, 0, 100);
			if (len > 30) len = 30;
			status = esp32_socket_read(hesp, (uint8_t *)buf, len);
			printf(buf);
		}
	}

	/* ソケットクローズ */
	status = esp32_socket_close(hesp);

	printf("Finished.\n");

stop_task:
	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Sample program ends.");
//	SVC_PERROR(ext_ker());
}
