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
 *  @(#) $Id: dvp.c 699 2019-11-29 22:10:19Z roi $
 */
/*
 * 
 *  K210 DVPドライバ関数群
 *
 */
#include <stddef.h>
#include <kernel.h>
#include <sil.h>
#include <t_syslog.h>
#include <target_syssvc.h>
#include <kernel_cfg.h>
#include "device.h"
#include "dvp.h"
#include "cambus.h"

/*
 *  SIL関数のマクロ定義
 */
#define sil_orw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) | (b))
#define sil_andw_mem(a, b)		sil_wrw_mem((a), sil_rew_mem(a) & ~(b))
#define sil_modw_mem(a, b, c)	sil_wrw_mem((a), (sil_rew_mem(a) & (~b)) | (c))

#define DVP_CLOCK_REQ           0
#define APB1_CLOCK_REQ          1

#define RESET_DELAY_TIME        200

#define DEAFULT_CLEAR_INT       (DVP_STS_FRAME_START  | DVP_STS_FRAME_START_WE | \
                                 DVP_STS_FRAME_FINISH | DVP_STS_FRAME_FINISH_WE)


static DVP_Handle_t *phdvp;

static uint32_t
dvp_clock_get_freq(uint8_t clock)
{
	uint32_t clk_sel0 = sil_rew_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_CLK_SEL0));
    uint32_t source = 0;
    uint32_t result = 0;
	uint32_t threshold = 0;

	/*
	 *  ACLKの取り出し
	 */
	if((clk_sel0 & SYSCTL_CLK_SEL0_ACLK_SEL) == 0)
		source = SYSCTRL_CLOCK_FREQ_IN0;
	else{
		threshold = (clk_sel0 & SYSCTL_CLK_SEL0_ACLK_SDIVISER) >> 1;
		source = get_pll_clock(0) / (2ULL << threshold);
	}

	if(clock == DVP_CLOCK_REQ){
		threshold = sil_rew_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_TH0)) & SYSCTL_CLK_TH0_DVP_GCLK_THHD;
		threshold >>= 12;
	}
	else	/* APB1 request */
		threshold = (clk_sel0 & SYSCTL_CLK_SEL0_APB1_CLK_SEL) >> 6;
	result = source / (threshold + 1);
	syslog_2(LOG_NOTICE, "## dvp_clock_get_freq req(%d) result(%d) ##", clock, result);
    return result;
}

static void
dvp_sccb_start_transfer(DVP_Handle_t *hdvp)
{
	while(sil_rew_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS)) & DVP_STS_SCCB_EN)
        ;
	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS), (DVP_STS_SCCB_EN | DVP_STS_SCCB_EN_WE));
    while (sil_rew_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS)) & DVP_STS_SCCB_EN)
        ;
}


/*
 *  SCCBレジスタ書込み
 *  parameter1  hdvp: DVPハンドラへのポインタ
 *  parameter2  addr: デバイスアドレス
 *  parameter3  reg_addr: レジスタアドレス
 *  parameter4  reg_data: レジスタデータ
 */
void
dvp_sccb_send_data(DVP_Handle_t *hdvp, uint8_t addr, uint16_t reg_addr, uint8_t reg_data)
{
	uint32_t tmp;

	tmp = sil_rew_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CFG)) & (~DVP_SCCB_BYTE_NUM_MASK);

	(hdvp->Init.num_sccb_reg == 8) ? (tmp |= DVP_SCCB_BYTE_NUM_3) : (tmp |= DVP_SCCB_BYTE_NUM_4);

	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CFG), tmp);

	if (hdvp->Init.num_sccb_reg == 8)
		tmp = DVP_SCCB_WRITE_DATA_ENABLE | addr | (reg_addr << 8) | (reg_data << 16);
	else
		tmp = DVP_SCCB_WRITE_DATA_ENABLE | addr | ((reg_addr >> 8) << 8) | ((reg_addr & 0xff) << 16) | (reg_data << 24);
	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CTL), tmp);
	dvp_sccb_start_transfer(hdvp);
}

/*
 *  SCCBレジスタ読み込み
 *  parameter1  hdvp: DVPハンドラへのポインタ
 *  parameter2  addr: デバイスアドレス
 *  parameter3  reg_addr: レジスタアドレス
 *  return      読み込みデータ
 */
uint8_t dvp_sccb_receive_data(DVP_Handle_t *hdvp, uint8_t addr, uint16_t reg_addr)
{
	uint32_t tmp;

	tmp = sil_rew_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CFG)) & (~DVP_SCCB_BYTE_NUM_MASK);

    if (hdvp->Init.num_sccb_reg == 8)
		tmp |= DVP_SCCB_BYTE_NUM_2;
	else
		tmp |= DVP_SCCB_BYTE_NUM_3;

	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CFG), tmp);

	if(hdvp->Init.num_sccb_reg == 8)
		tmp = DVP_SCCB_WRITE_DATA_ENABLE | addr | (reg_addr << 8);
	else
		tmp = DVP_SCCB_WRITE_DATA_ENABLE | addr | ((reg_addr >> 8) << 8) | ((reg_addr & 0xff) << 16);

	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CTL), tmp);

    dvp_sccb_start_transfer(hdvp);

	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CTL), addr);

    dvp_sccb_start_transfer(hdvp);

	tmp = sil_rew_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CFG));
	return (uint8_t) (tmp >> 24);
}

/*
 *  DVP初期設定
 *  parameter1  hdvp: DVPハンドラへのポインタ
 *  return      ERコード
 */
ER
dvp_init(DVP_Handle_t *hdvp)
{
    uint32_t v_apb1_clk, v_period;

	if(hdvp == NULL)
		return E_PAR;

	if(hdvp->Init.GMMlen > DVP_AXI_GM_MLEN_MASK || hdvp->Init.GMMlen == 0)
		return E_PAR;

	hdvp->base  = DVP_BASE_ADDR;
	hdvp->semid = DVP_SEM;
	fpioa_set_function(hdvp->Init.CMosPClkPin, FUNC_CMOS_PCLK);
	fpioa_set_function(hdvp->Init.CMosXClkPin, FUNC_CMOS_XCLK);
	fpioa_set_function(hdvp->Init.CMosHRefPin, FUNC_CMOS_HREF);
	fpioa_set_function(hdvp->Init.CMosPwDnPin, FUNC_CMOS_PWDN);
	fpioa_set_function(hdvp->Init.CMosVSyncPin, FUNC_CMOS_VSYNC);
	fpioa_set_function(hdvp->Init.CMosRstPin, FUNC_CMOS_RST);
//	fpioa_set_function(hdvp->Init.SccbSClkPin, FUNC_SCCB_SCLK);
//	fpioa_set_function(hdvp->Init.SccbSdaPin, FUNC_SCCB_SDA);
	cambus_init(hdvp->Init.SccbSClkPin, hdvp->Init.SccbSdaPin, 0, 0);

    /* Do a power cycle */
    dvp_dcmi_powerdown(hdvp, false);
    dly_tsk(10);

    dvp_dcmi_powerdown(hdvp, true);
    dly_tsk(100);

	/*
	 *  DVPクロック有効化
	 */
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_CENT), SYSCTL_CLK_EN_CENT_APB1_CLK_EN);
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_PERI), SYSCTL_CLK_EN_PERI_DVP_CLK_EN);

	phdvp = hdvp;

	/*
	 *  DVPリセット
	 */
	sil_orw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_PERI_RESET), SYSCTL_PERI_RESET_DVP_RESET);
	dly_tsk(10);
	sil_andw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_PERI_RESET), SYSCTL_PERI_RESET_DVP_RESET);
	dly_tsk(1);

	/*
	 *  SCCBクロック初期化
	 */
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CFG), (DVP_SCCB_SCL_LCNT_MASK | DVP_SCCB_SCL_HCNT_MASK));

	 // Initialize dvp interface
	v_apb1_clk = dvp_clock_get_freq(APB1_CLOCK_REQ);
	if(v_apb1_clk > (hdvp->Init.Freq * 2))
		v_period = ((v_apb1_clk + hdvp->Init.Freq) / (hdvp->Init.Freq * 2)) - 1;
	else
		v_period = 0;
	if(v_period > 255)
		v_period = 255;

	/*
	 *  DVPリセット
	 */
	sil_andw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_CLK_DIV_MASK);
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), (DVP_CMOS_CLK_ENABLE | v_period));
	/* First power down */
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_POWER_DOWN);
	sil_dly_nse(RESET_DELAY_TIME*1000);
	sil_andw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_POWER_DOWN);
	sil_dly_nse(RESET_DELAY_TIME*1000);

	/* Second reset */
	sil_andw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_RESET);
	sil_dly_nse(RESET_DELAY_TIME*1000);
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_RESET);
	sil_dly_nse(RESET_DELAY_TIME*1000);

	/*
	 *  バーストモード設定
	 */
	sil_modw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), DVP_CFG_BURST_SIZE_4BEATS, hdvp->Init.BurstMode);
	sil_modw_mem((uint32_t *)(hdvp->base+TOFF_DVP_AXI), DVP_AXI_GM_MLEN_MASK, hdvp->Init.GMMlen-1);

	/*
	 *  オートモード設定
	 */
	sil_modw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), DVP_CFG_AUTO_ENABLE, hdvp->Init.AutoMode);

	/*
	 *  出力有効化
	 */
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), DVP_CFG_AI_OUTPUT_ENABLE);
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), DVP_CFG_DISPLAY_OUTPUT_ENABLE);

	/*
	 *  イメージフォーマット、サイズ設定
	 */
	dvp_set_image_format(hdvp);
	dvp_set_image_size(hdvp);	//set QVGA default

	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_R_ADDR), hdvp->Init.RedAddr);
	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_G_ADDR), hdvp->Init.GreenAddr);
	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_B_ADDR), hdvp->Init.BlueAddr);
	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_RGB_ADDR), hdvp->Init.RGBAddr);

	/*
	 *  割込み設定
	 */
	sil_andw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), (DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE));
	dis_int(hdvp->Init.IntNo);
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS), DEAFULT_CLEAR_INT);
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), (DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE));
	hdvp->state = DVP_STATE_INIT;
	return E_OK;
}

/*
 *  DVP終了設定
 *  parameter1  hspi: SPIハンドラへのポインタ
 *  return ERコード
 */
ER
dvp_deinit(DVP_Handle_t *hdvp)
{
	if(hdvp == NULL)
		return E_PAR;

	dis_int(hdvp->Init.IntNo);
	sil_andw_mem((uint32_t *)(TADR_SYSCTL_BASE+TOFF_SYSCTL_CLK_EN_PERI), SYSCTL_CLK_EN_PERI_DVP_CLK_EN);

	phdvp = NULL;
	hdvp->state = DVP_STATE_INIT;
	return E_OK;
}

/*
 *  DVP有効化処理
 *  parameter1  hspi: SPIハンドラへのポインタ
 *  parameter2  run:  1:有効化、0:無効化
 *  return ERコード
 */
ER
dvp_activate(DVP_Handle_t *hdvp, bool_t run)
{
	if(hdvp == NULL)
		return E_PAR;
	if(run){
		hdvp->state = DVP_STATE_READY;
		sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS), DEAFULT_CLEAR_INT);
		ena_int(hdvp->Init.IntNo);
		sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), (DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE));
	}
	else{
		hdvp->state = DVP_STATE_INIT;
		dis_int(hdvp->Init.IntNo);
		sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS), DEAFULT_CLEAR_INT);
		sil_andw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), (DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE));
	}
	return E_OK;
}

/*
 *  DVPイメージフォーマット再設定
 *  parameter1  hdvp:  DVPハンドラへのポインタ
 *              Init.Formatに再設定値
 *  return ERコード
 */
ER
dvp_set_image_format(DVP_Handle_t *hdvp)
{
	if(hdvp == NULL)
		return E_PAR;
	if((hdvp->Init.Format & ~DVP_CFG_FORMAT_MASK) != 0)
		return E_PAR;

	sil_modw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), DVP_CFG_FORMAT_MASK, hdvp->Init.Format);
	return E_OK;
}

/*
 *  DVPイメージサイズ再設定
 *  parameter1  hdvp:  DVPハンドラへのポインタ
 *              Init.Width, Init.Heightに再設定値
 *  return ERコード
 */
ER
dvp_set_image_size(DVP_Handle_t *hdvp)
{
	uint32_t cfg, divw;

	if(hdvp == NULL)
		return E_PAR;
	cfg  = sil_rew_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG));
	cfg &= ~(DVP_CFG_HREF_BURST_NUM_MASK | DVP_CFG_LINE_NUM_MASK);
    cfg |= hdvp->Init.Height << 20;

    if((cfg & DVP_CFG_BURST_SIZE_4BEATS) != 0)
		divw = 4;
	else
		divw = 1;
	cfg |= (hdvp->Init.Width / 8 / divw) << 12;
    sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CFG), cfg);
	return E_OK;
}

/*
 *  SCCBクロック再設定
 *  parameter1  hdvp:  DVPハンドラへのポインタ
 *  parameter2  clk_rate:設定クロック値
 *  return      設定値、0で未設定
 */
uint32_t
dvp_sccb_set_clk_rate(DVP_Handle_t *hdvp, uint32_t clk_rate)
{
	uint32_t sccb_cfg;
	uint32_t v_sccb_freq = dvp_clock_get_freq(APB1_CLOCK_REQ);
	uint16_t v_period_clk_cnt = ((v_sccb_freq+(clk_rate/2)) / clk_rate / 2);

	if(hdvp == NULL)
		return 0;
	if(v_period_clk_cnt > 255){
		return 0;
	}

	sccb_cfg  = sil_rew_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CFG));
    sccb_cfg &= ~(DVP_SCCB_SCL_LCNT_MASK | DVP_SCCB_SCL_HCNT_MASK);
    sccb_cfg |= (v_period_clk_cnt << 8) | (v_period_clk_cnt << 16);
	sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_SCCB_CFG), sccb_cfg);
	return dvp_clock_get_freq(DVP_CLOCK_REQ) / (v_period_clk_cnt * 2);
}

/*
 *  DVP-DCMIリセット設定
 *  parameter1  hdvp:  DVPハンドラへのポインタ
 *  parameter2  reset: 1:reset, 0:relase
 *  return ERコード
 */
ER
dvp_dcmi_reset(DVP_Handle_t *hdvp, bool_t reset)
{
	if(hdvp == NULL)
		return E_PAR;
	if(reset)
		sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_RESET);
	else
		sil_andw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_RESET);
	return E_OK;
}

/*
 *  DVP-DCMIパワーダウン設定
 *  parameter1  hdvp:  DVPハンドラへのポインタ
 *  parameter2  down: 1:down, 0:up
 *  return ERコード
 */
ER
dvp_dcmi_powerdown(DVP_Handle_t *hdvp, bool_t down)
{
	if(hdvp == NULL)
		return E_PAR;
	if(down)
		sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_POWER_DOWN);
	else
		sil_andw_mem((uint32_t *)(hdvp->base+TOFF_DVP_CMOS_CFG), DVP_CMOS_POWER_DOWN);
	return E_OK;
}

/*
 *  DVP割込みハンドラ
 */
void
dvp_handler(void)
{
	DVP_Handle_t *hdvp = phdvp;
	uint32_t istatus, estatus;

	if(hdvp == NULL)
		return;
	istatus = sil_rew_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS));
	if(istatus == 0)
		return;
	estatus = istatus;
	syslog_2(LOG_DEBUG, "sensor_irq istatus[%08x] hdvp->state(%d)", istatus, hdvp->state);
	if((istatus & DVP_STS_FRAME_FINISH) != 0){	//frame end
		estatus |= DVP_STS_FRAME_FINISH_WE;
		hdvp->state = DVP_STATE_FINISH;
		if(hdvp->semid != 0)
			isig_sem(hdvp->semid);
	}
	if((istatus & DVP_STS_FRAME_START) != 0){	//frame start
		estatus |= DVP_STS_FRAME_START_WE;
        if(hdvp->state == DVP_STATE_ACTIVATE){  //only we finish the convert, do transmit again
			/*
			 *  コンバートスタート
			 */
			sil_wrw_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS), DVP_STS_DVP_EN | DVP_STS_DVP_EN_WE);
			hdvp->state = DVP_STATE_STARTED;
		}
	}
	sil_orw_mem((uint32_t *)(hdvp->base+TOFF_DVP_STS), estatus);
}


