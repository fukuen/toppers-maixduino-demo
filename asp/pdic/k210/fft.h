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
 *  @(#) $Id: fft.h 699 2020-12-26 17:44:27Z fukuen $
 */
#ifndef _FFT_H
#define _FFT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TADR_FFT_INPUT_FIFO		0x0000
#define TADR_FFT_CTRL			0x0008
  #define FFT_CTRL_POINT			0x00000007
  #define FFT_CTRL_MODE				0x00000008
  #define FFT_CTRL_SHIFT			0x00001FF0
  #define FFT_CTRL_ENABLE			0x00002000
  #define FFT_CTRL_DMA_SEND			0x00004000
  #define FFT_CTRL_INPUT_MODE		0x00018000
  #define FFT_CTRL_DATA_MODE		0x00020000
#define TADR_FFT_FIFO_CTRL		0x0010
#define TADR_FFT_INTR_MASK		0x0018
#define TADR_FFT_INTR_CLEAR 	0x0020
#define TADR_FFT_STATUS			0x0028
#define TADR_FFT_STATUS_RAW		0x0030
#define TADR_FFT_OUTPUT_FIFO	0x0038

typedef struct _complex_hard
{
    int16_t real;
    int16_t imag;
} complex_hard_t;

typedef struct _fft_data
{
    int16_t I1;
    int16_t R1;
    int16_t I2;
    int16_t R2;
} fft_data_t;

typedef enum _fft_point
{
    FFT_512,
    FFT_256,
    FFT_128,
    FFT_64,
} fft_point_t;

typedef enum _fft_direction
{
    FFT_DIR_BACKWARD,
    FFT_DIR_FORWARD,
    FFT_DIR_MAX,
} fft_direction_t;

typedef enum _fft_shift
{
    FFT_FORWARD_SHIFT  = 0x0U,
    FFT_BACKWARD_SHIFT = 0x1ffU
} fft_shift_t;

/*
 *  FFT 設定初期設定構造体
 */
typedef struct
{
	int32_t               TxDMAChannel;		/* FFT TxDMAチャンネル */
	int32_t               RxDMAChannel;		/* FFT RxDMAチャンネル */
    uint16_t              shift;
    fft_direction_t       direction;
    size_t                point_num;
	int                   semdmaid;			/* FFT DMA通信用セマフォ値 */
}FFT_Init_t;

/*
 *  FFTハンドラ定義
 */
typedef struct __FFT_Handle {
	FFT_Init_t            Init;				/* FFT communication parameters */
	uint64_t              *input;		    /* Pointer to FFT Tx transfer Buffer */
	uint64_t              *output;		    /* Pointer to FFT Rx transfer Buffer */
	DMA_Handle_t          *hdmatx;		    /* FFT Tx DMA handle parameters */
	DMA_Handle_t          *hdmarx;		    /* FFT Rx DMA handle parameters */
}FFT_Handle_t;

void fft_complex_uint16_dma(FFT_Handle_t * hfft, const uint64_t *input, uint64_t *output);

#ifdef __cplusplus
}
#endif

#endif /* _FFT_H */
