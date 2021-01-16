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
 *  上記著作権者は，以下の(1)~(4)の条件を満たす場合に限り，本ソフトウェ
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
 *  $Id: tone.c 2416 2020-12-03 08:06:20Z fukuen $
 */

/* 
 *  I2Sサンプルプログラムの本体
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
#include "device.h"
#include "tone.h"
#include "i2s.h"
#include "sysctl.h"

#define I2S_SCLK_PIN  35
#define I2S_WS_PIN    33
#define I2S_DA_PIN    34
#define MIC_SCLK_PIN  18
#define MIC_WS_PIN    19
#define MIC_DAT_PIN   20

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

static uint32_t heap_area[2*512*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*2*512*1024)
};

#define FRAME_LENGTH 512
uint32_t i2s_rx_buf[FRAME_LENGTH * 2];
uint32_t i2s_tx_buf[FRAME_LENGTH * 2];

#define SAMPLE_RATE 44100
#define PI 3.14159265358979323846
#define VOLUME 10

#define HIGH 0x1
#define LOW  0x0

#define INPUT           0x0
#define OUTPUT          0x3
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0X1

/*
 *  ダイレクトデジタルピン設定
*/
void pinMode(uint8_t Pin, uint8_t dwMode){ 
	int gpionum = gpio_get_gpiohno(Pin, false);
	GPIO_Init_t init = {0};

	syslog_2(LOG_NOTICE, "## pinMode Pin(%d) gpionum(%d) ##", Pin, gpionum);
	if(gpionum >= 0){
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
    int8_t gpio_pin = gpio_get_gpiohno(Pin, false);

    if( gpio_pin >= 0){
        gpio_set_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin, dwVal);
    }
}

/*
 *  正弦波生成
 */
int generateSign(uint16_t frequency)
{
	int16_t len = SAMPLE_RATE / frequency;
	int i = 0, j = 0, k = 0, temp;
	for (j = 0; j < 10; j++) {
		for (i = 0; i < len; i++) {
			temp = (short)(32767 * sinf(2 * PI * i / len));
			i2s_tx_buf[k + 1] = (short)(temp * VOLUME / 10);
			k += 2;
		}
	}
}

/*
 *  メインタスク
 */
void main_task(intptr_t exinf)
{
	I2S_Init_t i2s_initd;
	I2S_Handle_t *hi2s;
	ER_UINT	ercd;
	SYSTIM  tim;

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

	syslog(LOG_NOTICE, "TONE TEST START");

	i2s_initd.RxTxMode = I2S_TRANSMITTER;
	i2s_initd.MclkPin  = -1;
	i2s_initd.SclkPin  = I2S_SCLK_PIN;
	i2s_initd.WsPin    = I2S_WS_PIN;
	i2s_initd.InD0Pin  = -1;
	i2s_initd.InD1Pin  = -1;
	i2s_initd.InD2Pin  = -1;
	i2s_initd.InD3Pin  = -1;
	i2s_initd.OutD0Pin = I2S_DA_PIN;
	i2s_initd.OutD1Pin = -1;
	i2s_initd.OutD2Pin = -1;
	i2s_initd.OutD3Pin = -1;
	i2s_initd.RxChannelMask = 0x00;
	i2s_initd.TxChannelMask = 0x03;
	i2s_initd.RxDMAChannel = -1;
	i2s_initd.TxDMAChannel = I2S_DMA_CH;
	i2s_initd.semdmaid = I2S0DMATX_SEM;
	i2s_initd.word_length = RESOLUTION_16_BIT;
	i2s_initd.word_select_size = SCLK_CYCLES_32;
	i2s_initd.trigger_level = TRIGGER_LEVEL_4;
	i2s_initd.word_mode = STANDARD_MODE;
	i2s_initd.sample_rate = SAMPLE_RATE;
	if((hi2s = i2s_init(I2S1_PORTID, &i2s_initd)) == NULL){
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## I2S ERROR(1) ##");
	}

	int len = generateSign(800);

	// audio PA on
	pinMode(2, OUTPUT);
	digitalWrite(2, HIGH);

	SYSTIM tim_start;
	ercd = get_tim(&tim);
	tim_start = tim;
	while (tim < (tim_start + 3000)){ // 3000ms
		ercd = i2s_send_data(hi2s, i2s_tx_buf, len * 10);
		dly_tsk(1);
		ercd = get_tim(&tim);
	}

	// audio PA off
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);

stop_task:
	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Sample program ends.");
//	SVC_PERROR(ext_ker());
}
