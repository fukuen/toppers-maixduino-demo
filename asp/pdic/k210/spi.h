/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
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
 *  @(#) $Id: spi.h 698 2019-12-28 18:11:30Z roi $
 */
/*
 * 
 *  K210 SPIデバイスドライバの外部宣言
 *
 */

#ifndef _SPI_H_
#define _SPI_H_

#ifdef __cplusplus
 extern "C" {
#endif

/*
 *  SPIポート定義
 */
#define SPI1_PORTID             1
#define SPI2_PORTID             2
#define SPI3_PORTID             3
#define SPI4_PORTID             4
#define NUM_SPIPORT             4


/*
 *  SPI状態定義
 */
#define SPI_STATUS_RESET        0x0000	/* SPI未使用状態 */
#define SPI_STATUS_READY        0x0001	/* SPIレディ状態 */
#define SPI_STATUS_ERROR        0x0002	/* SPIエラー状態 */
#define SPI_STATUS_BUSY         0x0004	/* SPI処理中 */

/*
 *  SPI転送モード
 */
#define SPI_XMODE_TX            0x0000	/* 送信モード */
#define SPI_XMODE_RX            0x0001	/* 受信モード */
#define SPI_XMODE_TXRX          0x0002	/* 送受信モード */

/*
 *  SPIエラー定義
 */
#define SPI_ERROR_NONE          0x00000000	/* No error */
#define SPI_ERROR_MODF          0x00000001	/* MODF error */
#define SPI_ERROR_CRC           0x00000002	/* CRC error */
#define SPI_ERROR_OVR           0x00000004	/* OVR error */
#define SPI_ERROR_FRE           0x00000008	/* FRE error */
#define SPI_ERROR_DMA           0x00000010	/* DMA transfer error */
#define SPI_ERROR_TIMEOUT       0x00000020

/*
 *  SPIワークモード定義
 */
#define SPI_WORK_MODE_0         0x00000000
#define SPI_WORK_MODE_1         0x00000001
#define SPI_WORK_MODE_2         0x00000002
#define SPI_WORK_MODE_3         0x00000003

/*
 *  SPIフレームフォーマット定義
 */
#define SPI_FF_STANDARD         0x00000000
#define SPI_FF_DUAL             0x00000001
#define SPI_FF_QUAD             0x00000002
#define SPI_FF_OCTAL            0x00000003

/*
 *  SPIインストラクションアドレスモード
 */
#define SPI_AITM_STANDARD        0x00000000
#define SPI_AITM_ADDR_STANDARD   0x00000001
#define SPI_AITM_AS_FRAME_FORMAT 0x00000002


/*
 *  SPI転送モード定義
 */
#define SPI_TMOD_TRANS_RECV     0x00000000
#define SPI_TMOD_TRANS          0x00000001
#define SPI_TMOD_RECV           0x00000002
#define SPI_TMOD_EEROM          0x00000003

/*
 *  SPI転送データ長定義
 */
#define SPI_TRANS_CHAR          0x01
#define SPI_TRANS_SHORT         0x02
#define SPI_TRANS_INT           0x04

/*
 *  SPI CS選択定義
 */
#define SPI_CHIP_SELECT_0       0x00
#define SPI_CHIP_SELECT_1       0x01
#define SPI_CHIP_SELECT_2       0x02
#define SPI_CHIP_SELECT_3       0x03
#define SPI_CHIP_SELECT_MAX     4

/*
 *  SPI 設定初期設定構造体
 */
typedef struct
{
	uint32_t              WorkMode;
	uint32_t              FrameFormat;
    uint32_t              DataSize;			/* SPI転送データサイズ */
	uint32_t              Prescaler;		/* SPIクロック分周設定 */
	uint32_t              SignBit;			/* SPI MSB/LSB設定 */
	uint32_t              InstLength;		/* SPI Instraction Length */
	uint32_t              AddrLength;		/* SPI Address Length */
	uint32_t              WaitCycles;		/* SPI WaitCycles */
	uint32_t              IATransMode;		/* SPI 転送モード */
	int32_t               SclkPin;			/* SPI SCLK-PIN */
	int32_t               MosiPin;			/* SPI MOSI-PIN */
	int32_t               MisoPin;			/* SPI MISO-PIN */
	int32_t               SsPin;			/* SPI Slave Select-PIN */
	int32_t               SsNo;				/* SPI Slave Select-Number */
	int32_t               TxDMAChannel;		/* SPI TxDMAチャンネル */
	int32_t               RxDMAChannel;		/* SPI RxDMAチャンネル */
	int                   semid;			/* SPI 通信用セマフォ値 */
	int                   semlock;			/* SPI ロックセマフォ値 */
	int                   semdmaid;			/* SPI DMA通信用セマフォ値 */
}SPI_Init_t;

/*
 *  SPIハンドラ
 */
typedef struct _SPI_Handle_t
{
	unsigned long         base;				/* SPI registers base address */
	SPI_Init_t            Init;				/* SPI communication parameters */
	uint8_t               spi_num;
	uint8_t               dfs_offset;
	uint8_t               frf_offset;
	uint8_t               work_mode_offset;
	uint8_t               tmod_offset;
	uint8_t               dummy[3];
	uint8_t               *pTxBuffPtr;		/* Pointer to SPI Tx transfer Buffer */
	uint32_t              TxXferSize;		/* SPI Tx transfer size */
	uint32_t              TxXferCount;		/* SPI Tx Transfer Counter */
	uint8_t               *pRxBuffPtr;		/* Pointer to SPI Rx transfer Buffer */
	uint32_t              RxXferSize;		/* SPI Rx transfer size */
	uint32_t              RxXferCount;		/* SPI Rx Transfer Counter */
	DMA_Handle_t          *hdmatx;			/* SPI Tx DMA handle parameters */
	DMA_Handle_t          *hdmarx;			/* SPI Rx DMA handle parameters */
	uint16_t              xmode;			/* SPI Transfar mode */
	volatile uint16_t     status;			/* SPI communication state */
	volatile uint32_t     ErrorCode;		/* SPI Error code */
}SPI_Handle_t;


extern SPI_Handle_t *spi_init(ID port, const SPI_Init_t *init);
extern ER spi_deinit(SPI_Handle_t *hspi);
extern ER spi_core_transmit(SPI_Handle_t *hspi, int8_t ss_no, uint8_t *pdata, uint16_t length);
extern ER spi_core_transmit_fill(SPI_Handle_t *hspi, int8_t ss_no, const uint32_t *tx_buff, size_t tx_len);
extern ER spi_core_receive(SPI_Handle_t *hspi, int8_t ss_no, void *rx_buff, size_t rx_len);
extern ER spi_core_transrecv(SPI_Handle_t *hspi, int8_t ss_no, const uint8_t *tx_buf, uint8_t *rx_buf, size_t len);
extern ER spi_eerom_transrecv(SPI_Handle_t *hspi, int8_t ss_no, uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len);
extern ER spi_esp32_transrecv(SPI_Handle_t *hspi, int8_t ss_no, uint8_t *tx_buf, size_t tx_len, uint8_t *rx_buf, size_t rx_len);
extern ER spi_transmit(SPI_Handle_t *hspi, uint8_t *pdata, uint16_t length);
extern ER spi_receive(SPI_Handle_t *hspi, uint8_t *pdata, uint16_t length);
extern ER spi_transrecv(SPI_Handle_t *hspi, uint8_t *ptxData, uint8_t *prxData, uint16_t length);
extern ER spi_wait(SPI_Handle_t *hspi, uint32_t timeout);
extern void spi_handler(SPI_Handle_t *hspi);
extern void spi_isr(intptr_t exinf);
extern DMA_Handle_t *spi_dmac_set_single_mode(SPI_Handle_t *hspi, 
                          uint8_t rtx,
						  int8_t  ss_no,
                          const void *src, void *dest, uint8_t src_inc,
                          uint8_t dest_inc,
                          uint8_t dmac_burst_size,
                          uint8_t dmac_trans_width,
                          size_t block_size);
extern ER spi_dmac_wait_done(DMA_Handle_t * hdma);


#ifdef __cplusplus
}
#endif

#endif	/* _SPI_H_ */

