/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
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
 *  $Id: camera.c 2416 2020-12-10 08:06:20Z fukuen $
 */

/* 
 *  CAMERA デモプログラム
 */

#include <kernel.h>
#include <stdlib.h>
#include <t_syslog.h>
#include <t_stdlib.h>
#include <target_syssvc.h>
#include "syssvc/serial.h"
#include "syssvc/syslog.h"
#include "device.h"
#include "sipeed_st7789.h"
#include "sipeed_ov7740.h"
#include "kernel_cfg.h"
#include "camera.h"

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

#ifndef SPI1DMATX_SEM
#define SPI1DMATX_SEM   0
#endif

static uint32_t heap_area[256*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*256*1024)
};

/*
 *  使用する変数の定義
 */
LCD_Handler_t  LcdHandle;
LCD_DrawProp_t DrawProp;
DVP_Handle_t   DvpHandle;
OV7740_t       CameraHandle;

/*
 *  メインタスク
 */
void main_task(intptr_t exinf)
{
	SPI_Init_t Init;
	SPI_Handle_t    *hspi;
	LCD_Handler_t   *hlcd;
	OV7740_t        *hcmr;
	DVP_Handle_t    *hdvp;
	uint16_t        *lcd_buffer;
	ER_UINT	ercd;
	uint32_t count;
	unsigned long atmp;

	SVC_PERROR(syslog_msk_log(LOG_UPTO(LOG_INFO), LOG_UPTO(LOG_EMERG)));
	syslog(LOG_NOTICE, "Camera program starts (exinf = %d).", (int_t) exinf);

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

    select_spi0_dvp_mode(1);

	hcmr = &CameraHandle;
	hcmr->frameSize = FRAMESIZE_QVGA;
	hcmr->pixFormat = PIXFORMAT_RGB565;
	ov7740_getResolition(hcmr, FRAMESIZE_QVGA);
	hcmr->_resetPoliraty  = ACTIVE_HIGH;
	hcmr->_pwdnPoliraty   = ACTIVE_HIGH;
	hcmr->_slaveAddr      = 0x00;
	hcmr->_dataBuffer     = NULL;
	hcmr->_aiBuffer       = NULL;

    // just support RGB565 and YUV442 on k210

    // Initialize the camera bus, 8bit reg
	hdvp = &DvpHandle;
	hcmr->hdvp = hdvp;
	hdvp->Init.Freq         = 24000000;
	hdvp->Init.Width        = hcmr->_width;
	hdvp->Init.Height       = hcmr->_height;
	hdvp->Init.Format       = DVP_FORMAT_RGB;
	hdvp->Init.BurstMode    = DVP_BURST_ENABLE;
	hdvp->Init.AutoMode     = DVP_AUTOMODE_DISABLE;
	hdvp->Init.GMMlen       = 4;

	hdvp->Init.num_sccb_reg = 8;
	hdvp->Init.CMosPClkPin  = 47;
	hdvp->Init.CMosXClkPin  = 46;
	hdvp->Init.CMosHRefPin  = 45;
	hdvp->Init.CMosPwDnPin  = 44;
	hdvp->Init.CMosVSyncPin = 43;
	hdvp->Init.CMosRstPin   = 42;
	hdvp->Init.SccbSClkPin  = 41;
	hdvp->Init.SccbSdaPin   = 40;
	hdvp->Init.IntNo        = INTNO_DVP;
	syslog_1(LOG_NOTICE, "## DvpHandle[%08x] ##", &DvpHandle);

	syslog_3(LOG_NOTICE, "## hcmr->_width(%d) hcmr->_height(%d) size[%08x] ##", hcmr->_width, hcmr->_height, (hcmr->_width * hcmr->_height * 2));
	hcmr->_dataBuffer = (uint32_t*)malloc(hcmr->_width * hcmr->_height * 2); //RGB565
    if(hcmr->_dataBuffer == NULL){
		hcmr->_width = 0;
		hcmr->_height = 0;
		syslog_0(LOG_ERROR, "Can't allocate _dataBuffer !");
		slp_tsk();
    }
	hcmr->_aiBuffer = (uint32_t*)malloc(hcmr->_width * hcmr->_height * 3 + 64 * 1024);   //RGB888
	if(hcmr->_aiBuffer == NULL){
		hcmr->_width = 0;
        hcmr->_height = 0;
		free(hcmr->_dataBuffer);
		hcmr->_dataBuffer = NULL;
		syslog_0(LOG_ERROR, "Can't allocate _aiBuffer !");
		slp_tsk();
	}
	syslog_2(LOG_NOTICE, "## hcmr->_dataBuffer[%08x] hcmr->_aiBuffer[%08x] ##", hcmr->_dataBuffer, hcmr->_aiBuffer);
	atmp = (unsigned long)hcmr->_aiBuffer;
	hdvp->Init.RedAddr    = (uint32_t)atmp;
	atmp = (unsigned long)(hcmr->_aiBuffer + hcmr->_width * hcmr->_height);
	hdvp->Init.GreenAddr  = (uint32_t)atmp;
	atmp = (unsigned long)(hcmr->_aiBuffer + hcmr->_width * hcmr->_height * 2);
	hdvp->Init.BlueAddr   = (uint32_t)atmp;
	atmp = (unsigned long)hcmr->_dataBuffer;
	hdvp->Init.RGBAddr    = (uint32_t)atmp;
    dvp_init(hdvp);

	if(ov7740_sensor_gc_detect(hcmr) == E_OK){
		syslog_0(LOG_NOTICE, "find gc0328 !");
	}
	syslog_1(LOG_NOTICE, "GC0328 id(%d)", ov7740_id(hcmr));
	if(gc0328_reset(hcmr) != E_OK){
		syslog_0(LOG_ERROR, "gc0328 reset error !");
		slp_tsk();
	}
	if(gc0328_set_pixformat(hcmr) != E_OK){
		syslog_0(LOG_ERROR, "set pixformat error !");
		slp_tsk();
	}
	if(gc0328_set_framesize(hcmr) != E_OK){
		syslog_0(LOG_ERROR, "set frame size error !");
		slp_tsk();
	}

	Init.WorkMode     = SPI_WORK_MODE_0;
	Init.FrameFormat  = SPI_FF_OCTAL;
	Init.DataSize     = 8;
	Init.Prescaler    = 15000000;
	Init.SignBit      = 0;
	Init.InstLength   = 8;
	Init.AddrLength   = 0;
	Init.WaitCycles   = 0;
	Init.IATransMode  = SPI_AITM_AS_FRAME_FORMAT;
	Init.SclkPin      = SIPEED_ST7789_SCLK_PIN;
	Init.MosiPin      = SIPEED_ST7789_MOSI_PIN;
	Init.MisoPin      = SIPEED_ST7789_MISO_PIN;
	Init.SsPin        = SIPEED_ST7789_SS_PIN;
	Init.SsNo         = SIPEED_ST7789_SS;
	Init.TxDMAChannel = SIPEED_DMA_CH;
	Init.RxDMAChannel = -1;
	Init.semid        = SPI1TRN_SEM;
	Init.semlock      = SPI1LOCK_SEM;
	Init.semdmaid     = SPI1DMATX_SEM;
	hspi = spi_init(SPI_PORTID, &Init);
	if(hspi == NULL){
		syslog_0(LOG_ERROR, "SPI INIT ERROR");
		slp_tsk();
	}

	hlcd = &LcdHandle;
	hlcd->hspi    = hspi;
//	hlcd->spi_lock= SPI1LOCK_SEM;
	hlcd->dir     = DIR_YX_RLDU;
	hlcd->dcx_pin = SIPEED_ST7789_DCX_PIN;
	hlcd->rst_pin = SIPEED_ST7789_RST_PIN;
	hlcd->cs_sel  = SIPEED_ST7789_SS;
	hlcd->rst_no  = SIPEED_ST7789_RST_GPIONUM;
	hlcd->dcx_no  = SIPEED_ST7789_DCX_GPIONUM;
	DrawProp.hlcd = hlcd;
    lcd_init(hlcd);
	syslog_2(LOG_NOTICE, "width(%d) height(%d)", hlcd->_width, hlcd->_height);
	count = hcmr->_width * hcmr->_height;
	lcd_buffer = (uint16_t *)malloc(count * 2);
	if(lcd_buffer == NULL){
		syslog_0(LOG_ERROR, "no lcd buffer !");
		slp_tsk();
	}
	DrawProp.BackColor = ST7789_BLACK;
	DrawProp.TextColor = ST7789_WHITE;
	lcd_fillScreen(&DrawProp);

	if((ercd = ov7740_activate(hcmr, true)) != E_OK){
		syslog_2(LOG_NOTICE, "ov7740 activate error result(%d) id(%d) ##", ercd, ov7740_id(hcmr));
		slp_tsk();
	}

	for(;;){
		ercd = ov7740_snapshot(hcmr);
		if(ercd == E_OK){
			uint16_t *p = (uint16_t *)hcmr->_dataBuffer;
			uint32_t no;
			for (no = 0; no < count ; no += 2){
				lcd_buffer[no]   = SWAP_16(*(p + 1));
				lcd_buffer[no+1] = SWAP_16(*(p));
				p += 2;
			}
			lcd_drawPicture(hlcd, 0, 0, hcmr->_width, hcmr->_height, lcd_buffer);
		}
	}
	ov7740_activate(hcmr, false);

	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Camera program ends.");
	SVC_PERROR(ext_ker());
	assert(0);
}
