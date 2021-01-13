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
 *  @(#) $Id: cambus.c 699 2020-11-29 22:10:19Z fukuen $
 */
/*
 * 
 *  K210 CAMBUSドライバ関数群
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
#include "maix_i2c.h"
#include "cambus.h"

static uint32_t write_bus_delay = 10; //ms
static uint8_t sccb_reg_width = 8;

I2C_Init_t sccb_i2c_initd;
I2C_Handle_t *hi2c;

void cambus_set_writeb_delay(uint32_t delay)
{
    write_bus_delay = delay;
}

ER
sccb_i2c_init(uint8_t pin_clk, uint8_t pin_sda, uint8_t gpio_clk, uint8_t gpio_sda, uint32_t freq)
{
//    fpioa_set_function(pin_clk, FUNC_I2C0_SCLK + (MAIX_I2C_PORTID - 1) * 2);
//    fpioa_set_function(pin_sda, FUNC_I2C0_SDA + (MAIX_I2C_PORTID - 1) * 2);

	sccb_i2c_initd.ClockSpeed      = freq;
	sccb_i2c_initd.OwnAddress1     = 0;
	sccb_i2c_initd.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	sccb_i2c_initd.SclPin          = pin_clk;
	sccb_i2c_initd.SdaPin          = pin_sda;
	sccb_i2c_initd.semid           = MAIX_I2CTRS_SEM;
	sccb_i2c_initd.semlock         = MAIX_I2CLOC_SEM;

	if((hi2c = maix_i2c_init(MAIX_I2C_PORTID, &sccb_i2c_initd)) == NULL){
		/* Initialization Error */
		syslog_0(LOG_ERROR, "## SCCB_I2C ERROR(1) ##");
		return E_SYS;
	}
	syslog_1(LOG_NOTICE, "SCCB_I2C INITAILIZE(%d) !", MAIX_I2C_PORTID);

	hi2c->writecallback = NULL;
	hi2c->readcallback  = NULL;
	hi2c->errorcallback = NULL;

    return E_OK;
}

ER
sccb_i2c_write_byte(uint8_t addr, uint16_t reg, uint8_t reg_len, uint8_t data, uint16_t timeout_ms)
{
    uint8_t tmp[3];
    if(reg_len == 8)
    {
        tmp[0] = reg & 0xFF;
        tmp[1] = data;
		return maix_i2c_memwrite(hi2c, addr, 0, 0, tmp, 2, 10);
    }
    else
    {
        tmp[0] = (reg>>8) & 0xFF;
        tmp[1] = reg&0xFF;
        tmp[2] = data;
		return maix_i2c_memwrite(hi2c, addr, 0, 0, tmp, 3, 10);
    }
    return 0;
}

ER
sccb_i2c_read_byte(uint8_t addr, uint16_t reg, uint8_t reg_len, uint8_t* data, uint16_t timeout_ms)
{
    *data = 0;
    uint8_t tmp[2];
    if(reg_len == 8)
    {
        tmp[0] = reg & 0xFF;
		ER ret = maix_i2c_memwrite(hi2c, addr, 0, 0, tmp, 1, 10);
        if(ret != E_OK)
            return ret;
		ret = maix_i2c_memread(hi2c, addr, 0, 0, data, 1, 10);
        return ret;
    }
    else
    {
        tmp[0] = (reg>>8) & 0xFF;
        tmp[1] = reg&0xFF;
		ER ret = maix_i2c_memwrite(hi2c, addr, 0, 0, tmp, 2, 10);
        if(ret != E_OK)
            return ret;
		ret = maix_i2c_memread(hi2c, addr, 0, 0, data, 1, 10);
        return ret;
    }
    return 0;
}

ER
sccb_i2c_recieve_byte(uint8_t addr, uint8_t* data, uint16_t timeout_ms)
{
	ER ret = maix_i2c_memread(hi2c, addr, 0, 0, data, 1, 10);
    return ret;
}

/*
 *  CAMBUS初期設定
 *  parameter1  pin_clk: SCCB_CLK_PIN
 *  parameter2  pin_sda: SCCB_SDA_PIN
 *  parameter3  gpio_clk: SCCB_CLK_NO
 *  parameter4  gpio_sda: SCCB_SDA_NO
 *  return      ERコード
 */
ER
cambus_init(int8_t pin_clk, int8_t pin_sda, uint8_t gpio_clk, uint8_t gpio_sda)
{
    if(pin_clk<0 || pin_sda<0)
        return E_PAR;
    sccb_i2c_init(pin_clk, pin_sda, gpio_clk, gpio_sda, 100000);
	return E_OK;
}

/*
 *  CAMBUSの無効化
 *  return      ERコード
 */
ER
cambus_deinit(void)
{
    maix_i2c_deinit(hi2c);
	return E_OK;
}

ER
cambus_read_id(uint8_t addr,uint16_t *manuf_id, uint16_t *device_id)
{
    *manuf_id = 0;
    *device_id = 0;
    uint8_t tmp = 0;
    ER ret = E_OK;

    // sccb_i2c_write_byte(addr, 0xFF, sccb_reg_width, 0x01, 10); // for OV2640
    ret |= sccb_i2c_read_byte(addr, 0x1C, sccb_reg_width, &tmp, 100);
	if(ret != E_OK)
		return ret;
    *manuf_id = tmp << 8;
    ret |= sccb_i2c_read_byte(addr, 0x1D, sccb_reg_width, &tmp, 100);
	if(ret != E_OK)
		return ret;
    *manuf_id |= tmp;
    ret |= sccb_i2c_read_byte(addr, 0x0A, sccb_reg_width, &tmp, 100);
	if(ret != E_OK)
		return ret;
    *device_id = tmp << 8;
    ret |= sccb_i2c_read_byte(addr, 0x0B, sccb_reg_width, &tmp, 100);
    *device_id |= tmp;
	return ret;
}

ER
cambus_read16_id(uint8_t addr,uint16_t *manuf_id, uint16_t *device_id)
{
    *manuf_id = 0;
    *device_id = 0;
    uint8_t tmp = 0;
    ER ret = E_OK;
    //TODO: 0x300A 0x300B maybe just for OV3660
    // sccb_i2c_write_byte(addr, 0xFF, sccb_reg_width, 0x01, 10);
    ret |= sccb_i2c_read_byte(addr, 0x300A, sccb_reg_width, &tmp, 100);
    if(ret != E_OK)
        return ret;
    *device_id = tmp << 8;
    ret |= sccb_i2c_read_byte(addr, 0x300B, sccb_reg_width, &tmp, 100);
    *device_id |= tmp;
    // printk("ret:%d %04x %04x\r\n",ret, *manuf_id, *device_id);
	return ret;
}

int cambus_scan()
{
	uint16_t manuf_id = 0;
	uint16_t device_id = 0;
    sccb_reg_width = 8;
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
		if(cambus_read_id(addr ,&manuf_id,&device_id) != E_OK)
            continue;
        if(device_id!=0 && device_id!=0xffff)
        {
            return addr;
        }
    }
    sccb_reg_width = 16;
    for (uint8_t addr=0x08; addr<=0x77; addr++) {
		if( cambus_read16_id(addr ,&manuf_id,&device_id) != E_OK)
            continue;
        if(device_id!=0 && device_id!=0xffff)
        {
            return addr;
        }
    }
    return 0; // not found
}

int cambus_scan_gc0328(void)
{
    uint8_t id;
    sccb_reg_width = 8;
    sccb_i2c_write_byte(GC0328_ADDR, 0xFE, sccb_reg_width, 0x00, 10);
    sccb_i2c_read_byte(GC0328_ADDR, 0xF0, sccb_reg_width, &id, 10);
    if (id != 0x9d)
    {
        // mp_printf(&mp_plat_print, "error gc0328 detect, ret id is 0x%x\r\n", id);
        return 0;
    }
    return id;
}

ER
cambus_readb(uint8_t slv_addr, uint16_t reg_addr, uint8_t *reg_data)
{

    ER ret = E_OK;
    sccb_i2c_read_byte(slv_addr, reg_addr, sccb_reg_width, reg_data, 10);
	if(0xff == *reg_data)
		ret = E_SYS;

    return ret;

}

ER
cambus_writeb(uint8_t slv_addr, uint16_t reg_addr, uint8_t reg_data)
{
    sccb_i2c_write_byte(slv_addr, reg_addr, sccb_reg_width, reg_data, 10);
	dly_tsk(write_bus_delay);
	return E_OK;
}

uint8_t cambus_reg_width()
{
    return sccb_reg_width;
}
