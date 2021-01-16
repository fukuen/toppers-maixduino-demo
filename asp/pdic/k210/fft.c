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
 *  @(#) $Id: fft.c 699 2020-12-26 17:44:27Z fukuen $
 */
/*
 * 
 *  K210 FFTドライバ関数群
 *
 */
#include "kernel_impl.h"
#include <t_syslog.h>
#include <t_stdlib.h>
#include <stddef.h>
#include <string.h>
#include "kendryte-k210.h"
#include "sysctl.h"
#include "device.h"
#include "fft.h"

/*
 *  SIL関数のマクロ定義
 */
#define sil_orw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

FFT_Handle_t fft_handle;
DMA_Handle_t fft_dmarx_handle;
DMA_Handle_t fft_dmatx_handle;

/*
 *  FFT初期設定
 *  parameter1  init: FFT初期設定構造体へのポインタ
 *  return FFTハンドラへのポインタ、NULLでエラー
 */
FFT_Handle_t *
fft_init(const FFT_Init_t *init)
{
	FFT_Handle_t *hfft = &fft_handle;
	DMA_Handle_t *hdmarx = &fft_dmarx_handle;
	DMA_Handle_t *hdmatx = &fft_dmatx_handle;

	if(init == NULL)
		return NULL;

	fft_point_t point = FFT_512;
	switch(init->point_num)
	{
		case 512:
			point = FFT_512;
			break;
		case 256:
			point = FFT_256;
			break;
		case 128:
			point = FFT_128;
			break;
		case 64:
			point = FFT_64;
			break;
		default:
			return NULL;
			break;
	}

	memcpy(&hfft->Init, init, sizeof(FFT_Init_t));

	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_CENT), SYSCTL_CLK_EN_PERI_FFT_CLK_EN);
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_PERI_RESET), SYSCTL_PERI_RESET_FFT_RESET);
	sil_dly_nse(10000);
	sil_andw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_PERI_RESET), SYSCTL_PERI_RESET_FFT_RESET);

	sil_modw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_POINT, point);
	sil_modw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_MODE, ((init->direction) << 3));
	sil_modw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_SHIFT, ((init->shift) << 4));
	sil_orw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_DMA_SEND);
	sil_orw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_ENABLE);
	sil_andw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_INPUT_MODE);
	sil_andw_mem((uint32_t *)(FFT_BASE_ADDR+TADR_FFT_CTRL), FFT_CTRL_DATA_MODE);

	hdmarx->chnum = init->RxDMAChannel;
	hdmarx->xfercallback = NULL;
	hdmarx->errorcallback = NULL;
	hdmarx->Init.Request = DMA_SELECT_FFT_RX_REQ;		/* DMA選択 */
	hdmarx->Init.Direction = DMA_PERIPH_TO_MEMORY;		/* DMA転送方向 */
	hdmarx->Init.SrcMultBlock = DMAC_MULTBLOCK_CONT;	/* ソースマルチブロックタイプ */
	hdmarx->Init.DrcMultBlock = DMAC_MULTBLOCK_CONT;	/* デスティネーションマルチブロックタイプ */
	hdmarx->Init.SrcHandShake = DMAC_HS_HARDWARE;		/* ソースハンドシェイク */
	hdmarx->Init.DrcHandShake = DMAC_HS_SOFTWARE;		/* デスティネーションハンドシェイク */
	hdmarx->Init.SrcHwhsPol = DMAC_HWHS_POLARITY_LOW;	/* ソースハードウェアハンドシェイク極性 */
	hdmarx->Init.DrcHwhsPol = DMAC_HWHS_POLARITY_LOW;	/* デスティネーションハードウェアハンドシェイク極性 */
	hdmarx->Init.Priority = 4;							/* 優先度 */
	hdmarx->Init.SrcMaster = DMAC_MASTER1;				/* ソースマスター設定 */
	hdmarx->Init.DstMaster = DMAC_MASTER2;				/* デスティネーションマスター設定 */
	hdmarx->Init.SrcInc = DMAC_ADDR_NOCHANGE;			/* ソースインクリメント設定 */
	hdmarx->Init.DstInc = DMAC_ADDR_INCREMENT;			/* デスティネーションインクリメント設定 */
	hdmarx->Init.SrcTransWidth = DMAC_TRANS_WIDTH_64;	/* ソース転送幅 */
	hdmarx->Init.DstTransWidth = DMAC_TRANS_WIDTH_64;	/* デスティネーション転送幅 */
	hdmarx->Init.SrcBurstSize = DMAC_MSIZE_4;			/* ソースバーストサイズ */
	hdmarx->Init.DstBurstSize = DMAC_MSIZE_4;			/* デスティネーションバーストサイズ */
	hdmarx->Init.IocBlkTrans = 0;						/* IOCブロック転送 */
	hdmarx->localdata = (void *)hfft;

	dma_init(hdmarx);
	hfft->hdmarx = hdmarx;

	hdmatx->chnum = init->TxDMAChannel;
	hdmatx->xfercallback = NULL;
	hdmatx->errorcallback = NULL;
	hdmatx->Init.Request = DMA_SELECT_FFT_TX_REQ;		/* DMA選択 */
	hdmatx->Init.Direction = DMA_MEMORY_TO_PERIPH;		/* DMA転送方向 */
	hdmatx->Init.SrcMultBlock = DMAC_MULTBLOCK_CONT;	/* ソースマルチブロックタイプ */
	hdmatx->Init.DrcMultBlock = DMAC_MULTBLOCK_CONT;	/* デスティネーションマルチブロックタイプ */
	hdmatx->Init.SrcHandShake = DMAC_HS_SOFTWARE;		/* ソースハンドシェイク */
	hdmatx->Init.DrcHandShake = DMAC_HS_HARDWARE;		/* デスティネーションハンドシェイク */
	hdmatx->Init.SrcHwhsPol = DMAC_HWHS_POLARITY_LOW;	/* ソースハードウェアハンドシェイク極性 */
	hdmatx->Init.DrcHwhsPol = DMAC_HWHS_POLARITY_LOW;	/* デスティネーションハードウェアハンドシェイク極性 */
	hdmatx->Init.Priority = 4;							/* 優先度 */
	hdmatx->Init.SrcMaster = DMAC_MASTER1;				/* ソースマスター設定 */
	hdmatx->Init.DstMaster = DMAC_MASTER2;				/* デスティネーションマスター設定 */
	hdmatx->Init.SrcInc = DMAC_ADDR_INCREMENT;			/* ソースインクリメント設定 */
	hdmatx->Init.DstInc = DMAC_ADDR_NOCHANGE;			/* デスティネーションインクリメント設定 */
	hdmatx->Init.SrcTransWidth = DMAC_TRANS_WIDTH_64;	/* ソース転送幅 */
	hdmatx->Init.DstTransWidth = DMAC_TRANS_WIDTH_64;	/* デスティネーション転送幅 */
	hdmatx->Init.SrcBurstSize = DMAC_MSIZE_4;			/* ソースバーストサイズ */
	hdmatx->Init.DstBurstSize = DMAC_MSIZE_4;			/* デスティネーションバーストサイズ */
	hdmatx->Init.IocBlkTrans = 0;						/* IOCブロック転送 */
	hdmatx->localdata = NULL;

	dma_init(hdmatx);
	hfft->hdmatx = hdmatx;
	return hfft;
}

/*
 *  DMA転送開始関数
 */
DMA_Handle_t *
fft_dmac_set_single_mode(DMA_Handle_t *hdma,
						  const void *src, void *dest,
						  size_t block_size)
{
	dma_reset(hdma);
	dma_start(hdma, (uintptr_t)src, (uintptr_t)dest, block_size);
	return hdma;
}

/*
 *  FFT-DMA転送終了待ち
 */
ER
fft_dmac_wait_idle(DMA_Handle_t * hdma)
{
	FFT_Handle_t *hfft = (FFT_Handle_t *)hdma->localdata;
	ER ercd = E_OK;
	int tick = DMA_TRS_TIMEOUT;

	while((hdma->status == DMA_STATUS_BUSY) && tick > 0){
		if(hfft != NULL && hfft->Init.semdmaid != 0){
	 		ercd = twai_sem(hfft->Init.semdmaid, 5);
		}
		else
			dly_tsk(1);
		tick--;
	}
	dma_end(hdma);
	if(hdma->ErrorCode != 0)
		ercd = E_OBJ;
	else if(tick == 0)
		ercd = E_TMOUT;
	return ercd;
}

/*
 *  FFT
 */
void fft_complex_uint16_dma(FFT_Handle_t * hfft, const uint64_t *input, uint64_t *output)
{
	fft_dmac_set_single_mode(hfft->hdmarx, (void *)(FFT_BASE_ADDR+TADR_FFT_OUTPUT_FIFO), output,
		hfft->Init.point_num>>1);
	fft_dmac_set_single_mode(hfft->hdmatx, input, (void *)(FFT_BASE_ADDR+TADR_FFT_INPUT_FIFO),
		hfft->Init.point_num>>1);
	fft_dmac_wait_idle(hfft->hdmarx);
}
