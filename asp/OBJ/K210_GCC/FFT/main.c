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
 *  $Id: main.c 2416 2020-12-03 08:06:20Z fukuen $
 */

/* 
 *  FFTサンプルプログラムの本体
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
#include <math.h>
#include "device.h"
#include "sipeed_st7789.h"
#include "i2s.h"
#include "fft.h"
#include "spi.h"
#include "sysctl.h"
#include "main.h"

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

#ifndef SPI1DMATX_SEM
#define SPI1DMATX_SEM   0
#endif

static uint32_t heap_area[2*512*1024];

intptr_t heap_param[2] = {
	(intptr_t)heap_area,
	(4*2*512*1024)
};
/*
 *  使用する変数の定義
 */
LCD_Handler_t  LcdHandle;
LCD_DrawProp_t DrawProp;
SPI_Init_t Init;
I2S_Init_t i2s_initd;
FFT_Init_t fft_initd;

#define FFT_N 512
#define FRAME_LENGTH 512
uint32_t i2s_rx_buf[FRAME_LENGTH * 2];
uint32_t i2s_tx_buf[FRAME_LENGTH * 2];
uint32_t g_index;
uint32_t i2s_rec_index;
uint8_t i2s_rec_flag;

fft_data_t fft_in_data[FFT_N];
fft_data_t fft_out_data[FFT_N];
complex_hard_t data_hard[FFT_N] = {0};
float hard_power[FFT_N];

#define WIDTH 320
#define HEIGHT 240
uint16_t g_lcd_gram[WIDTH * HEIGHT] __attribute__((aligned(64)));

#define SAMPLE_RATE 16000

#define HIGH 0x1
#define LOW  0x0

#define INPUT           0x0
#define OUTPUT          0x3
#define INPUT_PULLUP    0x2
#define INPUT_PULLDOWN  0X1

/*
 *  ダイレクトデジタルピン設定
*/
void
pinMode(uint8_t Pin, uint8_t dwMode){ 
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
void
digitalWrite(uint8_t Pin, int dwVal){
    int8_t gpio_pin = gpio_get_gpiohno(Pin, false);

    if( gpio_pin >= 0){
        gpio_set_pin(TADR_GPIOHS_BASE, (uint8_t)gpio_pin, dwVal);
    }
}

#define SWAP_16(x) ((x >> 8 & 0xff) | (x << 8))

void update_image_fft(float* hard_power, float pw_max, uint32_t* pImage, uint32_t color, uint32_t bkg_color)
{
    uint32_t bcolor= SWAP_16((bkg_color << 16)) | SWAP_16(bkg_color);
    uint32_t fcolor= SWAP_16((color << 16)) | SWAP_16(color);

    int  h[80];

    int i, x = 0;

    for (i = 0; i < 80; i++)
    {
        h[i]=120*(hard_power[i])/pw_max;

        if (h[i]>120)
            h[i] = 120;
        if (h[i]<0)
            h[i] = 0;
    }

    for (i = 0; i < 80; i++)  // 53* 38640/512 => ~4000Hz
    {
        x=i*2;
        for( int y=0; y<120; y++)
        {
            if( y<(120 - h[i+2]) )
            {
                pImage[x+y*2*160]=bcolor;
                pImage[x+1+y*2*160]=bcolor;
                pImage[x+(y*2+1)*160]=bcolor;
                pImage[x+1+(y*2+1)*160]=bcolor;
            }
            else
            {
                pImage[x+y*2*160]=fcolor;
                pImage[x+1+y*2*160]=fcolor;
                pImage[x+(y*2+1)*160]=bcolor;
                pImage[x+1+(y*2+1)*160]=bcolor;
            }
        }
    }
}

void FFT(FFT_Handle_t *hfft, int offset)
{
    for (int i = 0; i < FFT_N / 2; i++)
    {
        fft_in_data[i].I1 = 0;
        fft_in_data[i].R1 = i2s_rx_buf[4 * i + offset];
        fft_in_data[i].I2 = 0;
        fft_in_data[i].R2 = i2s_rx_buf[4 + i + 2 + offset];
    }
    fft_complex_uint16_dma(hfft, (uint64_t *)fft_in_data, (uint64_t *)fft_out_data);
    for (int i = 0; i < FFT_N / 2; i++)
    {
        data_hard[2 * i].imag = fft_out_data[i].I1;
        data_hard[2 * i].real = fft_out_data[i].R1;
        data_hard[2 * i + 1].imag = fft_out_data[i].I2;
        data_hard[2 * i + 1].real = fft_out_data[i].R2;
    }

    float pmax=10;
    for (int i = 0; i < FFT_N / 2; i++)
    {
        hard_power[i] = sqrt(data_hard[i].real * data_hard[i].real + data_hard[i].imag * data_hard[i].imag);

        //Convert to dBFS
        hard_power[i] = 20*log(2*hard_power[i]/FFT_N);

        if( hard_power[i]>pmax)
            pmax = hard_power[i];
    }
}


/*
 *  メインタスク
 */
void main_task(intptr_t exinf)
{
	SPI_Handle_t  *hspi;
	LCD_Handler_t *hlcd;
	I2S_Handle_t  *hi2s_i;
	I2S_Handle_t  *hi2s_o;
	FFT_Handle_t  *hfft;
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

	syslog(LOG_NOTICE, "FFT DEMO START");

    select_spi0_dvp_mode(1);

	Init.WorkMode     = SPI_WORK_MODE_0;
	Init.FrameFormat  = SPI_FF_OCTAL;
	Init.DataSize     = 8;
	Init.Prescaler    = 20000000;
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
		syslog_0(LOG_ERROR, "## SPI INIT ERROR ##");
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
	i2s_initd.semdmaid = I2SDMATX_SEM;
	i2s_initd.word_length = RESOLUTION_16_BIT;
	i2s_initd.word_select_size = SCLK_CYCLES_32;
	i2s_initd.trigger_level = TRIGGER_LEVEL_4;
	i2s_initd.word_mode = STANDARD_MODE;
	i2s_initd.sample_rate = SAMPLE_RATE;
	if((hi2s_o = i2s_init(I2S1_PORTID, &i2s_initd)) == NULL){
		syslog_0(LOG_ERROR, "## I2S INIT ERROR(1) ##");
		slp_tsk();
	}

	i2s_initd.RxTxMode = I2S_RECEIVER;
	i2s_initd.MclkPin  = -1;
	i2s_initd.SclkPin  = MIC_SCLK_PIN;
	i2s_initd.WsPin    = MIC_WS_PIN;
	i2s_initd.InD0Pin  = -1;
	i2s_initd.InD1Pin  = MIC_DAT_PIN;
	i2s_initd.InD2Pin  = -1;
	i2s_initd.InD3Pin  = -1;
	i2s_initd.OutD0Pin = -1;
	i2s_initd.OutD1Pin = -1;
	i2s_initd.OutD2Pin = -1;
	i2s_initd.OutD3Pin = -1;
	i2s_initd.RxChannelMask = 0x0C;
	i2s_initd.TxChannelMask = 0x00;
	i2s_initd.RxDMAChannel = I2S_DMA1_CH;
	i2s_initd.TxDMAChannel = -1;
	i2s_initd.semdmaid = I2SDMARX_SEM;
	i2s_initd.word_length = RESOLUTION_16_BIT;
	i2s_initd.word_select_size = SCLK_CYCLES_32;
	i2s_initd.trigger_level = TRIGGER_LEVEL_4;
	i2s_initd.word_mode = STANDARD_MODE;
	i2s_initd.sample_rate = SAMPLE_RATE;
	if((hi2s_i = i2s_init(I2S2_PORTID, &i2s_initd)) == NULL){
		syslog_0(LOG_ERROR, "## I2S INIT ERROR(2) ##");
		slp_tsk();
	}

	fft_initd.TxDMAChannel = FFT_DMATX_CH;
	fft_initd.RxDMAChannel = FFT_DMARX_CH;
	fft_initd.shift = FFT_FORWARD_SHIFT;
	fft_initd.direction = FFT_DIR_FORWARD;
	fft_initd.point_num = FFT_N;
	fft_initd.semdmaid = FFTDMARX_SEM;
	if((hfft = fft_init(&fft_initd)) == NULL){
		syslog_0(LOG_ERROR, "## FFT INIT ERROR ##");
		slp_tsk();
	}

	hi2s_o->hdmatx->xfercallback = NULL;
	hi2s_i->hdmatx->xfercallback = NULL;
//	hi2s_i->readcallback = readCallback;
	g_index = 0;
	i2s_rec_index = 0;
	i2s_rec_flag = 0;

	// audio PA off
	pinMode(2, OUTPUT);
	digitalWrite(2, LOW);

	DrawProp.BackColor = ST7789_BLACK;
	DrawProp.TextColor = ST7789_RED;
	lcd_fillScreen(&DrawProp);

	while (1){
		i2s_receive_data(hi2s_i, i2s_rx_buf, FRAME_LENGTH * 2);
		FFT(hfft, 0);
		update_image_fft(hard_power, 140 /*MAX range dBFS*/, (uint32_t *)g_lcd_gram, ST7789_BLUE, ST7789_BLACK);
		lcd_drawPicture(hlcd, 0, 0, WIDTH, HEIGHT, (uint16_t *)g_lcd_gram);
		dly_tsk(1);
	}

stop_task:
	syslog_0(LOG_NOTICE, "## STOP ##");
	slp_tsk();
	syslog(LOG_NOTICE, "Sample program ends.");
//	SVC_PERROR(ext_ker());
}
