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
 *  �嵭����Ԥϡ��ʲ���(1)���(4)�ξ������������˸¤ꡤ�ܥ��եȥ���
 *  �����ܥ��եȥ���������Ѥ�����Τ�ޤࡥ�ʲ�Ʊ���ˤ���ѡ�ʣ������
 *  �ѡ������ۡʰʲ������ѤȸƤ֡ˤ��뤳�Ȥ�̵���ǵ������롥
 *  (1) �ܥ��եȥ������򥽡��������ɤη������Ѥ�����ˤϡ��嵭������
 *      ��ɽ�����������Ѿ�浪��Ӳ�����̵�ݾڵ��꤬�����Τޤޤη��ǥ���
 *      ����������˴ޤޤ�Ƥ��뤳�ȡ�
 *  (2) �ܥ��եȥ������򡤥饤�֥������ʤɡ�¾�Υ��եȥ�������ȯ�˻�
 *      �ѤǤ�����Ǻ����ۤ�����ˤϡ������ۤ�ȼ���ɥ�����ȡ�����
 *      �ԥޥ˥奢��ʤɡˤˡ��嵭�����ɽ�����������Ѿ�浪��Ӳ���
 *      ��̵�ݾڵ����Ǻܤ��뤳�ȡ�
 *  (3) �ܥ��եȥ������򡤵�����Ȥ߹���ʤɡ�¾�Υ��եȥ�������ȯ�˻�
 *      �ѤǤ��ʤ����Ǻ����ۤ�����ˤϡ����Τ����줫�ξ�����������
 *      �ȡ�
 *    (a) �����ۤ�ȼ���ɥ�����ȡ����Ѽԥޥ˥奢��ʤɡˤˡ��嵭����
 *        �ɽ�����������Ѿ�浪��Ӳ�����̵�ݾڵ����Ǻܤ��뤳�ȡ�
 *    (b) �����ۤη��֤��̤�������ˡ�ˤ�äơ�TOPPERS�ץ������Ȥ�
 *        ��𤹤뤳�ȡ�
 *  (4) �ܥ��եȥ����������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������뤤���ʤ�»
 *      ������⡤�嵭����Ԥ����TOPPERS�ץ������Ȥ����դ��뤳�ȡ�
 *      �ޤ����ܥ��եȥ������Υ桼���ޤ��ϥ���ɥ桼������Τ����ʤ���
 *      ͳ�˴�Ť����ᤫ��⡤�嵭����Ԥ����TOPPERS�ץ������Ȥ�
 *      ���դ��뤳�ȡ�
 * 
 *  �ܥ��եȥ������ϡ�̵�ݾڤ��󶡤���Ƥ����ΤǤ��롥�嵭����Ԥ�
 *  ���TOPPERS�ץ������Ȥϡ��ܥ��եȥ������˴ؤ��ơ�����λ�����Ū
 *  ���Ф���Ŭ������ޤ�ơ������ʤ��ݾڤ�Ԥ�ʤ����ޤ����ܥ��եȥ���
 *  �������Ѥˤ��ľ��Ū�ޤ��ϴ���Ū�������������ʤ�»���˴ؤ��Ƥ⡤��
 *  ����Ǥ�����ʤ���
 * 
 *  $Id: tone.c 2416 2020-12-03 08:06:20Z fukuen $
 */

/* 
 *  I2S����ץ�ץ���������
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
 *  �����ӥ�������Υ��顼�Υ�����
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
 *  �����쥯�ȥǥ�����ԥ�����
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
 *  �����쥯�ȥǥ�����ԥ����
 */
void digitalWrite(uint8_t Pin, int dwVal)
{
    int8_t gpio_pin = gpio_get_gpiohno(Pin, false);

    if( gpio_pin >= 0){
        gpio_set_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin, dwVal);
    }
}

/*
 *  ����������
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
 *  �ᥤ�󥿥���
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
	 *  ���ꥢ��ݡ��Ȥν����
	 *
	 *  �����ƥ����������Ʊ�����ꥢ��ݡ��Ȥ�Ȥ����ʤɡ����ꥢ��
	 *  �ݡ��Ȥ������ץ�Ѥߤξ��ˤϤ�����E_OBJ���顼�ˤʤ뤬���پ��
	 *  �ʤ���
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
