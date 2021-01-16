/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 *
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2010 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: tone.h 2416 2020-12-23 08:06:20Z fukuen $
 */

/*
 *	I2Sテストのヘッダファイル
 */

/*
 *  ターゲット依存の定義
 */
#include "target_test.h"
#include "i2s.h"

/*
 *  各タスクの優先度の定義
 */

#define MAIN_PRIORITY	5		/* メインタスクの優先度 */
								/* HIGH_PRIORITYより高くすること */

#define HIGH_PRIORITY	9		/* 並行実行されるタスクの優先度 */
#define MID_PRIORITY	10
#define LOW_PRIORITY	11

/*
 *  ターゲットに依存する可能性のある定数の定義
 */

#ifndef TASK_PORTID
#define	TASK_PORTID		1			/* 文字入力するシリアルポートID */
#endif /* TASK_PORTID */

#ifndef STACK_SIZE
#define	STACK_SIZE		8192		/* タスクのスタックサイズ */
#endif /* STACK_SIZE */

#ifndef I2SPORTID
#define I2SPORTID        1
#endif
#define I2S1_PORTID        1 //temp

#if I2SPORTID == 1
#define I2S_PORTID    I2S1_PORTID
#define INHNO_I2SEV   IRQ_VECTOR_I2S0	/* 割込みハンドラ番号 */
#define INTNO_I2SEV   IRQ_VECTOR_I2S0	/* 割込み番号 */
#define INTPRI_I2SEV  -5			/* 割込み優先度 */
#define INTATR_I2SEV  0				/* 割込み属性 */
#else
#define I2S_PORTID    I2S2_PORTID
#define INHNO_I2SEV   IRQ_VECTOR_I2S1	/* 割込みハンドラ番号 */
#define INTNO_I2SEV   IRQ_VECTOR_I2S1	/* 割込み番号 */
#define INTPRI_I2SEV  -5			/* 割込み優先度 */
#define INTATR_I2SEV  0				/* 割込み属性 */
#endif

#define I2S_DMA_CH    DMA_CHANNEL4
#define INHNO_DMATX4  IRQ_VECTOR_DMA4	/* 割込みハンドラ番号 */
#define INTNO_DMATX4  IRQ_VECTOR_DMA4	/* 割込み番号 */
#define INTPRI_DMATX4 -4		/* 割込み優先度 */
#define INTATR_DMATX4 0			/* 割込み属性 */

#define I2S_DMA1_CH   DMA_CHANNEL5
#define INHNO_DMARX5  IRQ_VECTOR_DMA5	/* 割込みハンドラ番号 */
#define INTNO_DMARX5  IRQ_VECTOR_DMA5	/* 割込み番号 */
#define INTPRI_DMARX5 -4		/* 割込み優先度 */
#define INTATR_DMARX5 0			/* 割込み属性 */

#ifndef TOPPERS_MACRO_ONLY

/*
 *  ヒープ領域の設定
 */
extern intptr_t heap_param[2];

/*
 *  関数のプロトタイプ宣言
 */
extern void	main_task(intptr_t exinf);
extern void heap_init(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */
