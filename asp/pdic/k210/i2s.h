/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
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
 *  @(#) $Id: i2s.h 698 2021-01-15 17:44:42Z fukuen $
 */
/*
 * 
 *  K210 I2Sデバイスドライバの外部宣言
 *
 */

#ifndef _I2S_H_
#define _I2S_H_

#ifdef __cplusplus
 extern "C" {
#endif


/*
 *  I2Sポート定義
 */
#define I2S1_PORTID             1
#define I2S2_PORTID             2
#define I2S3_PORTID             3
#define NUM_I2SPORT             3

#define I2S_MAX_NUM NUM_I2SPORT

typedef enum _i2s_device_number
{
	I2S_DEVICE_0 = 0,
	I2S_DEVICE_1 = 1,
	I2S_DEVICE_2 = 2,
	I2S_DEVICE_MAX
} i2s_device_number_t;

typedef enum _i2s_channel_num
{
	I2S_CHANNEL_0 = 0,
	I2S_CHANNEL_1 = 1,
	I2S_CHANNEL_2 = 2,
	I2S_CHANNEL_3 = 3
} i2s_channel_num_t;

typedef enum _i2s_transmit
{
	I2S_TRANSMITTER = 0,
	I2S_RECEIVER = 1,
	I2S_BOTH = 2
} i2s_transmit_t;

typedef enum _i2s_work_mode
{
	STANDARD_MODE = 1,
	RIGHT_JUSTIFYING_MODE = 2,
	LEFT_JUSTIFYING_MODE = 4
} i2s_work_mode_t;

typedef enum _sclk_gating_cycles
{
	/* Clock gating is diable */
	NO_CLOCK_GATING = 0x0,
	/* Gating after 12 sclk cycles */
	CLOCK_CYCLES_12 = 0x1,
	/* Gating after 16 sclk cycles */
	CLOCK_CYCLES_16 = 0x2,
	/* Gating after 20 sclk cycles */
	CLOCK_CYCLES_20 = 0x3,
	/* Gating after 24 sclk cycles */
	CLOCK_CYCLES_24 = 0x4
} i2s_sclk_gating_cycles_t;

typedef enum _word_select_cycles
{
	/* 16 sclk cycles */
	SCLK_CYCLES_16 = 0x0,
	/* 24 sclk cycles */
	SCLK_CYCLES_24 = 0x1,
	/* 32 sclk cycles */
	SCLK_CYCLES_32 = 0x2
} i2s_word_select_cycles_t;

typedef enum _word_length
{
	/* Ignore the word length */
	IGNORE_WORD_LENGTH = 0x0,
	/* 12-bit data resolution of the receiver */
	RESOLUTION_12_BIT = 0x1,
	/* 16-bit data resolution of the receiver */
	RESOLUTION_16_BIT = 0x2,
	/* 20-bit data resolution of the receiver */
	RESOLUTION_20_BIT = 0x3,
	/* 24-bit data resolution of the receiver */
	RESOLUTION_24_BIT = 0x4,
	/* 32-bit data resolution of the receiver */
	RESOLUTION_32_BIT = 0x5
} i2s_word_length_t;

typedef enum _fifo_threshold
{
	/* Interrupt trigger when FIFO level is 1 */
	TRIGGER_LEVEL_1 = 0x0,
	/* Interrupt trigger when FIFO level is 2 */
	TRIGGER_LEVEL_2 = 0x1,
	/* Interrupt trigger when FIFO level is 3 */
	TRIGGER_LEVEL_3 = 0x2,
	/* Interrupt trigger when FIFO level is 4 */
	TRIGGER_LEVEL_4 = 0x3,
	/* Interrupt trigger when FIFO level is 5 */
	TRIGGER_LEVEL_5 = 0x4,
	/* Interrupt trigger when FIFO level is 6 */
	TRIGGER_LEVEL_6 = 0x5,
	/* Interrupt trigger when FIFO level is 7 */
	TRIGGER_LEVEL_7 = 0x6,
	/* Interrupt trigger when FIFO level is 8 */
	TRIGGER_LEVEL_8 = 0x7,
	/* Interrupt trigger when FIFO level is 9 */
	TRIGGER_LEVEL_9 = 0x8,
	/* Interrupt trigger when FIFO level is 10 */
	TRIGGER_LEVEL_10 = 0x9,
	/* Interrupt trigger when FIFO level is 11 */
	TRIGGER_LEVEL_11 = 0xa,
	/* Interrupt trigger when FIFO level is 12 */
	TRIGGER_LEVEL_12 = 0xb,
	/* Interrupt trigger when FIFO level is 13 */
	TRIGGER_LEVEL_13 = 0xc,
	/* Interrupt trigger when FIFO level is 14 */
	TRIGGER_LEVEL_14 = 0xd,
	/* Interrupt trigger when FIFO level is 15 */
	TRIGGER_LEVEL_15 = 0xe,
	/* Interrupt trigger when FIFO level is 16 */
	TRIGGER_LEVEL_16 = 0xf
} i2s_fifo_threshold_t;

typedef enum _i2s_transfer_mode
{
	I2S_SEND,
	I2S_RECEIVE,
} i2s_transfer_mode_t;

/*
 *  I2S状態定義
 */
#define I2S_STATUS_RESET    0x00			/* I2S リセット状態 */
#define I2S_STATUS_READY    0x01			/* I2S レディ状態 */
#define I2S_STATUS_BUSY     0x02			/* I2S ビジィ状態 */
#define I2S_STATUS_BUSY_TX  0x52			/* I2S 送信中 */
#define I2S_STATUS_BUSY_RX  0x62			/* I2S 受信中 */
#define I2S_STATUS_TIMEOUT  0x80

/*
 *  SPIハードウェア設定構造体
 */

typedef struct _I2S_PortControlBlock{
	unsigned long         base;
	uint32_t              clkenable;
	uint32_t              perireset;
	uint32_t              thresholdctrl;
	uint32_t              thresholdmask;
	uint8_t               thresholdshift;
	uint8_t               dummy;
} I2S_PortControlBlock;

/*
 *  I2Sコンフュギュレーション構造体定義
 */
typedef struct
{
	i2s_transmit_t        RxTxMode;         /* I2S Tx/Rxモード */
	int32_t               MclkPin;			/* I2S MCLK PINNO */
	int32_t               SclkPin;			/* I2S SCLK PINNO */
	int32_t               WsPin;			/* I2S WS PINNO*/
	int32_t               InD0Pin;			/* I2S IN_D0 PINNO*/
	int32_t               InD1Pin;			/* I2S IN_D1 PINNO*/
	int32_t               InD2Pin;			/* I2S IN_D2 PINNO*/
	int32_t               InD3Pin;			/* I2S IN_D3 PINNO*/
	int32_t               OutD0Pin;			/* I2S OUT_D0 PINNO*/
	int32_t               OutD1Pin;			/* I2S OUT_D1 PINNO*/
	int32_t               OutD2Pin;			/* I2S OUT_D2 PINNO*/
	int32_t               OutD3Pin;			/* I2S OUT_D3 PINNO*/
	int32_t               TxChannelMask;	/* I2S Txチャンネルマスク */
	int32_t               RxChannelMask;	/* I2S Rxチャンネルマスク */
	int32_t               TxDMAChannel;		/* I2S TxDMAチャンネル */
	int32_t               RxDMAChannel;		/* I2S RxDMAチャンネル */
	int                   semid;			/* I2S 通信用セマフォ値 */
	int                   semlock;			/* I2S ロックセマフォ値 */
	int                   semdmaid;			/* I2S DMA通信用セマフォ値 */
	i2s_word_length_t     word_length;      /* ワード長 */
	i2s_word_select_cycles_t word_select_size; /* ワード選択サイズ */
	i2s_fifo_threshold_t  trigger_level;    /* FIFOしきい値 */
	i2s_work_mode_t       word_mode;        /* ワードモード */
	uint32_t              sample_rate;      /* サンプリングレート */
}I2S_Init_t;


/*
 *  I2Sハンドラ定義
 */
typedef struct __I2S_Handle_t I2S_Handle_t;
struct __I2S_Handle_t {
	unsigned long              base;
	I2S_Init_t                 Init;		/* I2S初期設定パラメータ */
	uint8_t                    i2s_num;	    /* I2S デバイス番号 */
	void                       (*writecallback)(I2S_Handle_t * hi2s);	/* 送信終了コールバック関数 */
	void                       (*readcallback)(I2S_Handle_t * hi2s);		/* 受信終了コールバック関数 */
	void                       (*errorcallback)(I2S_Handle_t * hi2s);	/* エラーコールバック関数 */
	ID                         i2sid;		/* I2S ID値 */
	DMA_Handle_t               *hdmatx;		/* I2S Tx DMA handle parameters */
	DMA_Handle_t               *hdmarx;		/* I2S Rx DMA handle parameters */
	volatile uint32_t          status;		/* I2S 実行状態 */
	volatile uint32_t          ErrorCode;	/* I2S Error code */
};

extern I2S_Handle_t *
i2s_init(ID portid, I2S_Init_t *ii2s);

extern ER
i2s_deinit(I2S_Handle_t *hi2s);

ER
i2s_receive_data_standard(I2S_Handle_t *hi2s, i2s_channel_num_t channel_num, uint64_t *buf, size_t buf_len);

ER
i2s_receive_data(I2S_Handle_t *hi2s, uint32_t *buf, size_t buf_len);

ER
i2s_send_data_standard(I2S_Handle_t *hi2s, i2s_channel_num_t channel_num, const uint8_t *pcm, size_t buf_len,
				  size_t single_length);

ER
i2s_send_data(I2S_Handle_t *hi2s, const void *buf, size_t buf_len);

uint32_t
i2s_set_sample_rate(I2S_Handle_t *hi2s, uint32_t sample_rate);

ER
i2s_set_dma_divide_16(I2S_Handle_t *hi2s, uint32_t enable);

int
i2s_get_dma_divide_16(I2S_Handle_t *hi2s);


#ifdef __cplusplus
}
#endif

#endif	/* _I2S_H_ */
