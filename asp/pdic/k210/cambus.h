/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2008-2011 by Embedded and Real-Time Systems Laboratory
 *              Graduate School of Information Science, Nagoya Univ., JAPAN
 *  Copyright (C) 2015-2019 by TOPPERS PROJECT Educational Working Group.
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
 *  @(#) $Id: cambus.h 698 2020-11-28 22:10:08Z fukuen $
 */
/*
 * 
 *  K210 CAMBUSデバイスドライバの外部宣言
 *
 */

#ifndef _CAMBUS_H_
#define _CAMBUS_H_

#include <kernel.h>
#include <target_syssvc.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAIX_PORTID
#define MAIX_PORTID        3
#endif

#if MAIX_PORTID == 1
#define MAIX_I2C_PORTID    I2C1_PORTID
#define MAIX_INHNO_I2CEV   IRQ_VECTOR_I2C0	/* 割込みハンドラ番号 */
#define MAIX_INTNO_I2CEV   IRQ_VECTOR_I2C0	/* 割込み番号 */
#define MAIX_INTPRI_I2CEV  -5			/* 割込み優先度 */
#define MAIX_INTATR_I2CEV  0				/* 割込み属性 */
#endif
#if MAIX_PORTID == 2
#define MAIX_I2C_PORTID    I2C2_PORTID
#define MAIX_INHNO_I2CEV   IRQ_VECTOR_I2C1	/* 割込みハンドラ番号 */
#define MAIX_INTNO_I2CEV   IRQ_VECTOR_I2C1	/* 割込み番号 */
#define MAIX_INTPRI_I2CEV  -5			/* 割込み優先度 */
#define MAIX_INTATR_I2CEV  0				/* 割込み属性 */
#endif
#if MAIX_PORTID == 3
#define MAIX_I2C_PORTID    I2C3_PORTID
#define MAIX_INHNO_I2CEV   IRQ_VECTOR_I2C2	/* 割込みハンドラ番号 */
#define MAIX_INTNO_I2CEV   IRQ_VECTOR_I2C2	/* 割込み番号 */
#define MAIX_INTPRI_I2CEV  -5			/* 割込み優先度 */
#define MAIX_INTATR_I2CEV  0				/* 割込み属性 */
#endif

#define GC0328_ADDR     (0x42)
//#define GC0328_ADDR     (0x21)

#ifndef TOPPERS_MACRO_ONLY

extern ER cambus_init(int8_t pin_clk, int8_t pin_sda, uint8_t gpio_clk, uint8_t gpio_sda);
extern ER cambus_deinit();
extern int cambus_scan();
extern int cambus_scan_gc0328(void);
extern void cambus_set_writeb_delay(uint32_t delay);
extern ER cambus_writeb(uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data);
extern ER cambus_writeb(uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data);
extern uint8_t cambus_reg_width();

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif

#endif	/* _CAMBUS_H_ */

