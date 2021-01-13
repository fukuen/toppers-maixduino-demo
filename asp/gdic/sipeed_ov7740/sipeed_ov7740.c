/*
 *  TOPPERS/ASP Kernel
 *      Toyohashi Open Platform for Embedded Real-Time Systems/
 *      Advanced Standard Profile Kernel
 * 
 *  Copyright (C) 2000-2003 by Embedded and Real-Time Systems Laboratory
 *                              Toyohashi Univ. of Technology, JAPAN
 *  Copyright (C) 2004-2012 by Embedded and Real-Time Systems Laboratory
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
 *  $Id: sipeed_ov7740.c 2416 2019-11-29 22:11:42Z roi $
 */
/* 
 *  SIPEED OV7740 CAMERA制御プログラムの本体
 */

#include <kernel.h>
#include <t_syslog.h>
#include <target_syssvc.h>
#include "device.h"

#include "dvp.h"
#include <stdlib.h>
#include "math.h"
#include "sipeed_ov7740.h"
#include "cambus.h"


#define IM_MAX(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a > _b ? _a : _b; })
#define IM_MIN(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })
#define IM_DIV(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _b ? (_a / _b) : 0; })
#define IM_MOD(a,b)     ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _b ? (_a % _b) : 0; })

#ifndef SNAPSHOT_TIMEOUT
#define SNAPDHOT_TIMEOUT    300
#endif


#define SVGA_HSIZE     (800)
#define SVGA_VSIZE     (600)

#define UXGA_HSIZE     (1600)
#define UXGA_VSIZE     (1200)
static const uint8_t ov7740_default[][2] = { //k210 
	{0x47, 0x02}  ,
	{0x17, 0x27}  ,
	{0x04, 0x40}  ,
	{0x1B, 0x81}  ,
	{0x29, 0x17}  ,
	{0x5F, 0x03}  ,
	{0x3A, 0x09}  ,
	{0x33, 0x44}  ,
	{0x68, 0x1A}  ,
	{0x14, 0x38}  ,
	{0x5F, 0x04}  ,
	{0x64, 0x00}  ,
	{0x67, 0x90}  ,
	{0x27, 0x80}  ,
	{0x45, 0x41}  ,
	{0x4B, 0x40}  ,
	{0x36, 0x2f}  ,
	{0x11, 0x00}  ,  // 60fps
	{0x36, 0x3f}  ,
	// {0x0c, 0x12}  , // default YUYV
	{0x12, 0x00}  ,
	{0x17, 0x25}  ,
	{0x18, 0xa0}  ,
	{0x1a, 0xf0}  ,
	{0x31, 0x50}  ,
	{0x32, 0x78}  ,
	{0x82, 0x3f}  ,
	{0x85, 0x08}  ,
	{0x86, 0x02}  ,
	{0x87, 0x01}  ,
	{0xd5, 0x10}  ,
	{0x0d, 0x34}  ,
	{0x19, 0x03}  ,
	{0x2b, 0xf8}  ,
	{0x2c, 0x01}  ,
	{0x53, 0x00}  ,
	{0x89, 0x30}  ,
	{0x8d, 0x30}  ,
	{0x8f, 0x85}  ,
	{0x93, 0x30}  ,
	{0x95, 0x85}  ,
	{0x99, 0x30}  ,
	{0x9b, 0x85}  ,
	{0xac, 0x6E}  ,
	{0xbe, 0xff}  ,
	{0xbf, 0x00}  ,
	{0x38, 0x14}  ,
	{0xe9, 0x00}  ,
	{0x3D, 0x08}  ,
	{0x3E, 0x80}  ,
	{0x3F, 0x40}  ,
	{0x40, 0x7F}  ,
	{0x41, 0x6A}  ,
	{0x42, 0x29}  ,
	{0x49, 0x64}  ,
	{0x4A, 0xA1}  ,
	{0x4E, 0x13}  ,
	{0x4D, 0x50}  ,
	{0x44, 0x58}  ,
	{0x4C, 0x1A}  ,
	{0x4E, 0x14}  ,
	{0x38, 0x11}  ,
	{0x84, 0x70}  ,
	{0,0}

};

#define NUM_BRIGHTNESS_LEVELS (9)
static const uint8_t brightness_regs[NUM_BRIGHTNESS_LEVELS][2] = {
	{0x06, 0x40}, /* -4 */
	{0x06, 0x30}, /* -3 */
	{0x06, 0x20}, /* -2 */
	{0x06, 0x10}, /* -1 */
	{0x0E, 0x00}, /*  0 */
	{0x0E, 0x10}, /* +1 */
	{0x0E, 0x20}, /* +2 */
	{0x0E, 0x30}, /* +3 */
	{0x0E, 0x40}, /* +4 */
};

#define NUM_CONTRAST_LEVELS (9)
static const uint8_t contrast_regs[NUM_CONTRAST_LEVELS][3] = {
	{0x20, 0x10, 0xD0}, /* -4 */
	{0x20, 0x14, 0x80}, /* -3 */
	{0x20, 0x18, 0x48}, /* -2 */
	{0x20, 0x1C, 0x20}, /* -1 */
	{0x20, 0x20, 0x00}, /*  0 */
	{0x20, 0x24, 0x00}, /* +1 */
	{0x20, 0x28, 0x00}, /* +2 */
	{0x20, 0x2C, 0x00}, /* +3 */
	{0x20, 0x30, 0x00}, /* +4 */
};

#define NUM_SATURATION_LEVELS (9)
static const uint8_t saturation_regs[NUM_SATURATION_LEVELS][2] = {
	{0x00, 0x00}, /* -4 */
	{0x10, 0x10}, /* -3 */
	{0x20, 0x20}, /* -2 */
	{0x30, 0x30}, /* -1 */
	{0x40, 0x40}, /*  0 */
	{0x50, 0x50}, /* +1 */
	{0x60, 0x60}, /* +2 */
	{0x70, 0x70}, /* +3 */
	{0x80, 0x80}, /* +4 */
};

static const int resolution[][2] = {
	{0,    0   },
	// C/SIF Resolutions
	{88,   72  },    /* QQCIF     */
	{176,  144 },    /* QCIF      */
	{352,  288 },    /* CIF       */
	{88,   60  },    /* QQSIF     */
	{176,  120 },    /* QSIF      */
	{352,  240 },    /* SIF       */
	// VGA Resolutions
	{40,   30  },    /* QQQQVGA   */
	{80,   60  },    /* QQQVGA    */
	{160,  120 },    /* QQVGA     */
	{320,  240 },    /* QVGA      */
	{640,  480 },    /* VGA       */
	{60,   40  },    /* HQQQVGA   */
	{120,  80  },    /* HQQVGA    */
	{240,  160 },    /* HQVGA     */
	// FFT Resolutions
	{64,   32  },    /* 64x32     */
	{64,   64  },    /* 64x64     */
	{128,  64  },    /* 128x64    */
	{128,  128 },    /* 128x64    */
	// Other
	{128,  160 },    /* LCD       */
	{128,  160 },    /* QQVGA2    */
	{720,  480 },    /* WVGA      */
	{752,  480 },    /* WVGA2     */
	{800,  600 },    /* SVGA      */
	{1280, 1024},    /* SXGA      */
	{1600, 1200},    /* UXGA      */
};

#define OV7740_SET_MIRROR(r, x)   ((r&0xBF)|((x&1)<<6))
#define OV7740_SET_FLIP(r, x)     ((r&0x7F)|((x&1)<<7))
#define OV7740_SET_SP(r, x)     ((r&0xEE)|((x&1)<<4)|(x&1))

static uint8_t gc0328_default_regs[][2] = {
	{0xfe , 0x80},
	{0xfe , 0x80},
	{0xfc , 0x16},
	{0xfc , 0x16},
	{0xfc , 0x16},
	{0xfc , 0x16},
	{0xf1 , 0x00},
	{0xf2 , 0x00},
	{0xfe , 0x00},
	{0x4f , 0x00},
	{0x42 , 0x00},  
	{0x03 , 0x00},  
	{0x04 , 0xc0},  
	{0x77 , 0x62},  
	{0x78 , 0x40},  
	{0x79 , 0x4d},  

	{0xfe , 0x00},
	{0x16 , 0x00},
	{0x17 , 0x14},
	{0x18 , 0x0e},
	{0x19 , 0x06},

	{0x1b , 0x48},
	{0x1f , 0xC8},
	{0x20 , 0x01},
	{0x21 , 0x78},
	{0x22 , 0xb0},
	{0x23 , 0x04},//0x06  20140519 GC0328C
	{0x24 , 0x11}, 
	{0x26 , 0x00},

	//global gain for range 
	{0x70 , 0x85},   

	/////////////banding/////////////
	{0x05 , 0x00},//hb
	{0x06 , 0x6a},//
	{0x07 , 0x00},//vb
	{0x08 , 0x0c},//
	{0xfe , 0x01},//
	{0x29 , 0x00},//anti-flicker step [11:8]
	{0x2a , 0x96},//anti-flicker step [7:0]
	{0xfe , 0x00},//

	///////////////AWB//////////////
	{0xfe , 0x01},
	{0x50 , 0x00},
	{0x4f , 0x00},
	{0x4c , 0x01},
	{0x4f , 0x00},
	{0x4f , 0x00},
	{0x4f , 0x00},
	{0x4f , 0x00},
	{0x4f , 0x00}, 
	{0x4d , 0x30},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x40},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x50},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x60},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x70},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},	
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04}, 
	{0x4e , 0x04},	
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4f , 0x01},
	{0x50 , 0x88},
	{0xfe , 0x00},

	//////////// BLK//////////////////////
	{0xfe , 0x00}, 
	{0x27 , 0xb7},
	{0x28 , 0x7F},
	{0x29 , 0x20},
	{0x33 , 0x20},
	{0x34 , 0x20},
	{0x35 , 0x20},
	{0x36 , 0x20},
	{0x32 , 0x08},
	{0x3b , 0x00}, 
	{0x3c , 0x00},
	{0x3d , 0x00},
	{0x3e , 0x00},
	{0x47 , 0x00},
	{0x48 , 0x00}, 

	//////////// block enable/////////////
	{0x40 , 0x7f}, 
	{0x41 , 0x26}, 
	{0x42 , 0xfb},
	{0x44 , 0x00}, //yuv
	{0x45 , 0x00},
	{0x46 , 0x03},
	{0x4f , 0x01},
	{0x4b , 0x01},
	{0x50 , 0x01}, 

	/////DN & EEINTP/////
	{0x7e , 0x0a}, 
	{0x7f , 0x03}, 
	{0x81 , 0x15}, 
	{0x82 , 0x85},    
	{0x83 , 0x03},
	{0x84 , 0xe5},
	{0x90 , 0xac},    
	{0x92 , 0x02},
	{0x94 , 0x02},
	{0x95 , 0x32},    

	////////////YCP///////////
	{0xd1 , 0x28},
	{0xd2 , 0x28},
	{0xd3 , 0x40},
	{0xdd , 0x58},
	{0xde , 0x36},
	{0xe4 , 0x88},
	{0xe5 , 0x40}, 
	{0xd7 , 0x0e}, 

	///////////rgb gamma ////////////
	{0xfe , 0x00},
	{0xbf , 0x0e},
	{0xc0 , 0x1c},
	{0xc1 , 0x34},
	{0xc2 , 0x48},
	{0xc3 , 0x5a},
	{0xc4 , 0x6e},
	{0xc5 , 0x80},
	{0xc6 , 0x9c},
	{0xc7 , 0xb4},
	{0xc8 , 0xc7},
	{0xc9 , 0xd7},
	{0xca , 0xe3},
	{0xcb , 0xed},
	{0xcc , 0xf2},
	{0xcd , 0xf8},
	{0xce , 0xfd},
	{0xcf , 0xff},

	/////////////Y gamma//////////
	{0xfe , 0x00},
	{0x63 , 0x00},
	{0x64 , 0x05},
	{0x65 , 0x0b},
	{0x66 , 0x19},
	{0x67 , 0x2e},
	{0x68 , 0x40},
	{0x69 , 0x54},
	{0x6a , 0x66},
	{0x6b , 0x86},
	{0x6c , 0xa7},
	{0x6d , 0xc6},
	{0x6e , 0xe4},
	{0x6f , 0xff},

	//////////////ASDE/////////////
	{0xfe , 0x01},
	{0x18 , 0x02},
	{0xfe , 0x00},
	{0x98 , 0x00},    
	{0x9b , 0x20},    
	{0x9c , 0x80},    
	{0xa4 , 0x10},    
	{0xa8 , 0xB0},    
	{0xaa , 0x40},    
	{0xa2 , 0x23},    
	{0xad , 0x01},    

	//////////////abs///////////
	{0xfe , 0x01},
	{0x9c , 0x02},   
	{0x9e , 0xc0}, 
	{0x9f , 0x40},	

	////////////// AEC////////////
	{0x08 , 0xa0},
	{0x09 , 0xe8},
	{0x10 , 0x00},  
	{0x11 , 0x11},   
	{0x12 , 0x10},   
	{0x13 , 0x98},   
	{0x15 , 0xfc},   
	{0x18 , 0x03},
	{0x21 , 0xc0},   
	{0x22 , 0x60},   
	{0x23 , 0x30},
	{0x25 , 0x00},
	{0x24 , 0x14},
	{0x3d , 0x80},
	{0x3e , 0x40},

	////////////////AWB///////////
	{0xfe , 0x01},
	{0x51 , 0x88},
	{0x52 , 0x12},
	{0x53 , 0x80},
	{0x54 , 0x60},
	{0x55 , 0x01},
	{0x56 , 0x02},
	{0x58 , 0x00},
	{0x5b , 0x02},
	{0x5e , 0xa4},
	{0x5f , 0x8a},
	{0x61 , 0xdc},
	{0x62 , 0xdc},
	{0x70 , 0xfc},
	{0x71 , 0x10},
	{0x72 , 0x30},
	{0x73 , 0x0b},
	{0x74 , 0x0b},
	{0x75 , 0x01},
	{0x76 , 0x00},
	{0x77 , 0x40},
	{0x78 , 0x70},
	{0x79 , 0x00},
	{0x7b , 0x00},
	{0x7c , 0x71},
	{0x7d , 0x00},
	{0x80 , 0x70},
	{0x81 , 0x58},
	{0x82 , 0x98},
	{0x83 , 0x60},
	{0x84 , 0x58},
	{0x85 , 0x50},
	{0xfe , 0x00},	

	////////////////LSC////////////////
	{0xfe , 0x01},
	{0xc0 , 0x10},
	{0xc1 , 0x0c},
	{0xc2 , 0x0a},
	{0xc6 , 0x0e},
	{0xc7 , 0x0b},
	{0xc8 , 0x0a},
	{0xba , 0x26},
	{0xbb , 0x1c},
	{0xbc , 0x1d},
	{0xb4 , 0x23},
	{0xb5 , 0x1c},
	{0xb6 , 0x1a},
	{0xc3 , 0x00},
	{0xc4 , 0x00},
	{0xc5 , 0x00},
	{0xc9 , 0x00},
	{0xca , 0x00},
	{0xcb , 0x00},
	{0xbd , 0x00},
	{0xbe , 0x00},
	{0xbf , 0x00},
	{0xb7 , 0x07},
	{0xb8 , 0x05},
	{0xb9 , 0x05},
	{0xa8 , 0x07},
	{0xa9 , 0x06},
	{0xaa , 0x00},
	{0xab , 0x04},
	{0xac , 0x00},
	{0xad , 0x02},
	{0xae , 0x0d},
	{0xaf , 0x05},
	{0xb0 , 0x00},
	{0xb1 , 0x07},
	{0xb2 , 0x03},
	{0xb3 , 0x00},
	{0xa4 , 0x00},
	{0xa5 , 0x00},
	{0xa6 , 0x00},
	{0xa7 , 0x00},
	{0xa1 , 0x3c},
	{0xa2 , 0x50},
	{0xfe , 0x00},

	///////////////CCT ///////////
	{0xb1 , 0x12},
	{0xb2 , 0xf5},
	{0xb3 , 0xfe},
	{0xb4 , 0xe0},
	{0xb5 , 0x15},
	{0xb6 , 0xc8},

	/////skin CC for front //////
	{0xb1 , 0x00},
	{0xb2 , 0x00},
	{0xb3 , 0x05},
	{0xb4 , 0xf0},
	{0xb5 , 0x00},
	{0xb6 , 0x00},
	
	///////////////AWB////////////////
	{0xfe , 0x01},
	{0x50 , 0x00},
	{0xfe , 0x01}, 
	{0x4f , 0x00},
	{0x4c , 0x01},
	{0x4f , 0x00},
	{0x4f , 0x00},
	{0x4f , 0x00}, 
	{0x4d , 0x34},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4e , 0x02},
	{0x4e , 0x02},
	{0x4d , 0x44},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x53},
	{0x4e , 0x00},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4e , 0x04},
	{0x4d , 0x65},
	{0x4e , 0x04},
	{0x4d , 0x73},
	{0x4e , 0x20},
	{0x4d , 0x83},
	{0x4e , 0x20},
	{0x4f , 0x01}, 
	{0x50 , 0x88}, 

	{0xfe , 0x00},
	// window
		//windowing mode
	// {0x09 , 0x00},
	// {0x0a , 0x78},
	// {0x0b , 0x00},
	// {0x0c , 0xa0},
	// {0x0d , 0x00},
	// {0x0e , 0xf8},
	// {0x0f , 0x01},
	// {0x10 , 0x48},
		//crop mode 
	{0x50 , 0x01},
	// {0x51, 0x00},
	// {0x52, 0x78},
	// {0x53, 0x00},
	// {0x54, 0xa0},
	// {0x55, 0x00},
	// {0x56, 0xf0},
	// {0x57, 0x01},
	// {0x58, 0x40},
	//subsample 1/2
	{0x59, 0x22},
	{0x5a, 0x00},
	{0x5b, 0x00},
	{0x5c, 0x00},
	{0x5d, 0x00},
	{0x5e, 0x00},
	{0x5f, 0x00},
	{0x60, 0x00},
	{0x61, 0x00},
	{0x62, 0x00},

	//Exp level
	{0xfe, 0x01},
	{0x2b , 0x02},//exp level 0  30fps => 16fps
	{0x2c , 0x00},//			 
	{0x2d , 0x02},//exp level 1  12.50fps
	{0x2e , 0x00},//			 
	{0x2f , 0x02},//exp level 2  10.00fps
	{0x30 , 0x00},//			 
	{0x31 , 0x02},//exp level 3  7.14fps
	{0x32 , 0x00},//
	{0x33, 0x00},

	/////////output//////// 
	{0xfe , 0x00},	
	{0xf1 , 0x07}, 
	{0xf2 , 0x01}, 

	{0x00, 0x00}
};

static const uint8_t gc0328_qvga_config[][2] = { //k210 
	{0xfe , 0x00},
	// window
		//windowing mode
	// {0x09 , 0x00},
	// {0x0a , 0x78},
	// {0x0b , 0x00},
	// {0x0c , 0xa0},
	// {0x0d , 0x00},
	// {0x0e , 0xf8},
	// {0x0f , 0x01},
	// {0x10 , 0x48},
		//crop mode 
	{0x50 , 0x01},
	// {0x51, 0x00},
	// {0x52, 0x78},
	// {0x53, 0x00},
	// {0x54, 0xa0},
	// {0x55, 0x00},
	// {0x56, 0xf0},
	// {0x57, 0x01},
	// {0x58, 0x40},
	//subsample 1/2
	{0x59, 0x22},
	{0x5a, 0x00},
	{0x5b, 0x00},
	{0x5c, 0x00},
	{0x5d, 0x00},
	{0x5e, 0x00},
	{0x5f, 0x00},
	{0x60, 0x00},
	{0x61, 0x00},
	{0x62, 0x00},

	{0x00, 0x00}
};

static const uint8_t gc0328_vga_config[][2] = { //k210 
	{0xfe, 0x00},
	{0x4b, 0x8b},
	{0x50, 0x01},
	{0x51, 0x00},
	{0x52, 0x00},
	{0x53, 0x00},
	{0x54, 0x00},
	{0x55, 0x01},
	{0x56, 0xe0},
	{0x57, 0x02},
	{0x58, 0x80},
	{0x59, 0x11},
	{0x5a, 0x02},
	{0x5b, 0x00},
	{0x5c, 0x00},
	{0x5d, 0x00},
	{0x5e, 0x00},
	{0x5f, 0x00},
	{0x60, 0x00},
	{0x61, 0x00},
	{0x62, 0x00},

	{0x00, 0x00}
};

static const uint8_t gc0328_yuv422_regs[][2] = {
	{0xfe , 0x00},
	{0x44 , 0x00}, //yuv
	{0x00, 0x00}
};

static const uint8_t gc0328_rgb565_regs[][2] = {
	{0xfe , 0x00},
	{0x44 , 0x06},
	{0x00, 0x00}
};

int Sipeed_OV7740_dvpInitIrq(void);


static int reverse_u32pixel(uint32_t* addr, uint32_t length)
{
  if(NULL == addr)
	return -1;

  uint32_t data;
  uint32_t* pend = addr+length;
  for(;addr<pend;addr++)
  {
	  data = *(addr);
	  *(addr) = ((data & 0x000000FF) << 24) | ((data & 0x0000FF00) << 8) | 
				((data & 0x00FF0000) >> 8) | ((data & 0xFF000000) >> 24) ;
  }  //1.7ms
  
  
  return 0;
}

ER
ov7740_set_hmirror(OV7740_t *hcmr, int enable)
{
	uint8_t reg;
	cambus_readb(hcmr->_slaveAddr, 0x0C, &reg);
	cambus_writeb(hcmr->_slaveAddr, 0x0C, OV7740_SET_MIRROR(reg, enable));

	cambus_readb(hcmr->_slaveAddr, 0x16, &reg);
	cambus_writeb(hcmr->_slaveAddr, 0x16, OV7740_SET_SP(reg, enable));
	return E_OK;
}

ER
ov7740_set_special_effect(OV7740_t *hcmr, int sde)
{
	uint8_t reg;
	switch (sde)
	{
		case 0: // SDE_NORMAL:
			cambus_readb(hcmr->_slaveAddr, 0x81, &reg);
			cambus_writeb(hcmr->_slaveAddr, 0x81, reg & 0xFE);
			cambus_readb(hcmr->_slaveAddr, 0xDA, &reg);
			cambus_writeb(hcmr->_slaveAddr, 0xDA, reg & 0xBF);
			break;
		case 1: // SDE_NEGATIVE:
			cambus_readb(hcmr->_slaveAddr, 0x81, &reg);
			cambus_writeb(hcmr->_slaveAddr, 0x81, reg | 0x01);
			cambus_readb(hcmr->_slaveAddr, 0xDA, &reg);
			cambus_writeb(hcmr->_slaveAddr, 0xDA, reg | 0x40);
			break;
	
		default:
			return E_PAR;
	}
	return E_OK;
}

void
ov7740_choice(OV7740_t *hcmr, int8_t choice_dev)
{
	if (choice_dev == 1) (hcmr->_pwdnPoliraty == ACTIVE_HIGH) ? (dvp_dcmi_powerdown(hcmr->hdvp, true)) : (dvp_dcmi_powerdown(hcmr->hdvp, false));
}

void
ov7740_getResolition(OV7740_t *hcmr, framesize_t frameSize)
{
	hcmr->_width    = resolution[frameSize][0];
	hcmr->_height   = resolution[frameSize][1];
}

ER
ov7740_sensor_ov_detect(OV7740_t *hcmr)
{
	DVP_Handle_t *hdvp = hcmr->hdvp;

	/* Reset the sensor */
	dvp_dcmi_reset(hdvp, true);
	dly_tsk(10);

	dvp_dcmi_reset(hdvp, false);
	dly_tsk(10);

	/* Probe the ov sensor */
	hcmr->_slaveAddr = cambus_scan(hcmr);
	if(hcmr->_slaveAddr == 0){
		/* Sensor has been held in reset,
		   so the reset line is active low */
		hcmr->_resetPoliraty = ACTIVE_LOW;

		/* Pull the sensor out of the reset state,systick_sleep() */
		dvp_dcmi_powerdown(hdvp, false); //
		dly_tsk(10);                    //
		dvp_dcmi_powerdown(hdvp, true);//
		dly_tsk(10);                    //
		dvp_dcmi_reset(hdvp, true);
		dly_tsk(10);

		/* Probe again to set the slave addr */
		hcmr->_slaveAddr = cambus_scan(hcmr);
		if(hcmr->_slaveAddr == 0){
			hcmr->_pwdnPoliraty = ACTIVE_LOW;
			dvp_dcmi_powerdown(hdvp, false);     //
			dly_tsk(10);                        //
//			dvp_dcmi_powerdown(hdvp, true);
//			dly_tsk(10);
			dvp_dcmi_reset(hdvp, true);         //
			dly_tsk(10);                        //

			hcmr->_slaveAddr = cambus_scan(hcmr);
			if(hcmr->_slaveAddr == 0){
				hcmr->_resetPoliraty = ACTIVE_HIGH;
				dvp_dcmi_powerdown(hdvp, true);     //
				dly_tsk(10);                        //
				dvp_dcmi_powerdown(hdvp, false);     //
				dly_tsk(10);                        //
				dvp_dcmi_reset(hdvp, false);
				dly_tsk(10);

				hcmr->_slaveAddr = cambus_scan(hcmr);
				if(hcmr->_slaveAddr == 0){
					//should do something?
					return E_SYS;
				}
			}
		}
	}

	// Clear sensor chip ID.
	hcmr->_id = 0;

	if(hcmr->_slaveAddr == LEPTON_ID){
		hcmr->_id = LEPTON_ID;
		/*set LEPTON xclk rate*/
		/*lepton_init*/
	}
	else{
		// Read ON semi sensor ID.
		cambus_readb(hcmr->_slaveAddr, ON_CHIP_ID, &hcmr->_id);
		if(hcmr->_id == MT9V034_ID){
			/*set MT9V034 xclk rate*/
			/*mt9v034_init*/
		}
		else{	// Read OV sensor ID.
			cambus_readb(hcmr->_slaveAddr, OV_CHIP_ID, &hcmr->_id);
			// Initialize sensor struct.
			switch(hcmr->_id){
			case OV9650_ID:
				/*ov9650_init*/
				break;
			case OV2640_ID:
				// printf("detect ov2640, id:%x\n", _slaveAddr);
				break;
			case OV7740_ID:
				/*ov7740_init*/
				break;
			default:
				// Sensor is not supported.
				return E_SYS;
			}
		}
	}
	return E_OK;
}

ER
ov7740_sensor_gc_detect(OV7740_t *hcmr)
{
	DVP_Handle_t *hdvp = hcmr->hdvp;
	uint8_t id;

	dvp_dcmi_powerdown(hdvp, false);//enable gc0328 要恢? normal 工作模式，需将 PWDN pin 接入低?平即可，同?写入初始化寄存器即可
	dvp_dcmi_reset(hdvp, true);	//reset gc0328
	dly_tsk(10);
	dvp_dcmi_reset(hdvp, false);
	dly_tsk(10);
	id = cambus_scan_gc0328();
	if(0 == id){
		return E_SYS;
	}
	else{
		// printf("[MAIXPY]: gc0328 id = %x\n",id); 
		hcmr->_slaveAddr = GC0328_ADDR;
		hcmr->_id = id;
		cambus_set_writeb_delay(2);
	}
	return E_OK;
}


ER
ov7740_reset(OV7740_t *hcmr)
{
	int i=0;
	const uint8_t (*regs)[2];

	/* Reset all registers */
	cambus_writeb(hcmr->_slaveAddr, 0x12, 0x80);

	/* delay n ms */
	dly_tsk(2);

	i = 0;
	regs = ov7740_default;
	/* Write initial regsiters */
	while (regs[i][0]) {
		cambus_writeb(hcmr->_slaveAddr, regs[i][0], regs[i][1]);
		i++;
	}
	return E_OK;
}

ER
gc0328_reset(OV7740_t *hcmr)
{
	uint16_t index = 0;
	
	cambus_writeb(GC0328_ADDR, 0xfe, 0x01);
	for (index = 0; gc0328_default_regs[index][0]; index++)
	{
		if(gc0328_default_regs[index][0] == 0xff){
			dly_tsk(gc0328_default_regs[index][1]);
			continue;
		}
		if(gc0328_default_regs[index][0] == 0x00){
			continue;
		}
		cambus_writeb(GC0328_ADDR, gc0328_default_regs[index][0], gc0328_default_regs[index][1]);
	}
	return E_OK;
}

ER
ov7740_set_pixformat(OV7740_t *hcmr)
{
	hcmr->hdvp->Init.Format = DVP_FORMAT_YUY;
	dvp_set_image_format(hcmr->hdvp);
	/* delay n ms */
	dly_tsk(30);
	return E_OK;
}

ER
gc0328_set_pixformat(OV7740_t *hcmr)
{
	int i=0;
	const uint8_t (*regs)[2]=NULL;

	/* read pixel format reg */
	switch (hcmr->pixFormat) {
		case PIXFORMAT_RGB565:
			regs = gc0328_rgb565_regs;
			break;
		case PIXFORMAT_YUV422:
		case PIXFORMAT_GRAYSCALE:
			regs = gc0328_yuv422_regs;
			break;
		default:
			return E_PAR;
	}

	/* Write initial regsiters */
	while (regs[i][0]) {
		cambus_writeb(hcmr->_slaveAddr, regs[i][0], regs[i][1]);
		i++;
	}
	switch (hcmr->pixFormat) {
		case PIXFORMAT_RGB565:
			hcmr->hdvp->Init.Format = DVP_CFG_RGB_FORMAT;
			dvp_set_image_format(hcmr->hdvp);
			break;
		case PIXFORMAT_YUV422:
			hcmr->hdvp->Init.Format = DVP_CFG_YUV_FORMAT;
			dvp_set_image_format(hcmr->hdvp);
			break;
		case PIXFORMAT_GRAYSCALE:
			hcmr->hdvp->Init.Format = DVP_CFG_Y_FORMAT;
			dvp_set_image_format(hcmr->hdvp);
			break;
		default:
			return E_PAR;
	}
	/* delay n ms */
	dly_tsk(30);
	return E_OK;
}

ER
ov7740_set_framesize(OV7740_t *hcmr)
{
	uint16_t w = hcmr->_width;
	uint16_t h = hcmr->_height;

	// VGA
	if ((w > 320) || (h > 240))
	{
		cambus_writeb(hcmr->_slaveAddr, 0x31, 0xA0);
		cambus_writeb(hcmr->_slaveAddr, 0x32, 0xF0);
		cambus_writeb(hcmr->_slaveAddr, 0x82, 0x32);
	}
	// QVGA
	else if( ((w <= 320) && (h <= 240)) && ((w > 160) || (h > 120)) )
	{
		cambus_writeb(hcmr->_slaveAddr, 0x31, 0x50);
		cambus_writeb(hcmr->_slaveAddr, 0x32, 0x78);
		cambus_writeb(hcmr->_slaveAddr, 0x82, 0x3F);
	}
	// QQVGA
	else
	{
		cambus_writeb(hcmr->_slaveAddr, 0x31, 0x28);
		cambus_writeb(hcmr->_slaveAddr, 0x32, 0x3c);
		cambus_writeb(hcmr->_slaveAddr, 0x82, 0x3F);
	}

	/* delay n ms */
	dly_tsk(30);
	hcmr->hdvp->Init.Width  = w;
	hcmr->hdvp->Init.Height = h;
	return dvp_set_image_size(hcmr->hdvp);
}

ER
gc0328_set_framesize(OV7740_t *hcmr)
{
	uint16_t w = hcmr->_width;
	uint16_t h = hcmr->_height;

	int i=0;
	const uint8_t (*regs)[2];

	if ((w <= 320) && (h <= 240)) {
		regs = gc0328_qvga_config;
	} else {
		regs = gc0328_vga_config;
	}

	while (regs[i][0]) {
		cambus_writeb(hcmr->_slaveAddr, regs[i][0], regs[i][1]);
//        msleep(1);
		i++;
	}
	/* delay n ms */
//    mp_hal_delay_ms(30);
	dly_tsk(30);
	hcmr->hdvp->Init.Width  = w;
	hcmr->hdvp->Init.Height = h;
	return dvp_set_image_size(hcmr->hdvp);
}

ER
ov7740_activate(OV7740_t *hcmr, bool_t run)
{
	return dvp_activate(hcmr->hdvp, run);
}

int
ov7740_id(OV7740_t *hcmr)
{
	return hcmr->_id;
}

ER
ov7740_snapshot(OV7740_t *hcmr)
{
	DVP_Handle_t *hdvp = hcmr->hdvp;
	int32_t timeout = SNAPDHOT_TIMEOUT;

	//wait for new frame
	hdvp->state = DVP_STATE_ACTIVATE;

	while(hdvp->state != DVP_STATE_FINISH){
		if(--timeout <= 0)
			return E_TMOUT;
		if(hdvp->semid != 0)
			twai_sem(hdvp->semid, 1);
		else
			dly_tsk(1);
	}
	reverse_u32pixel((uint32_t*)hcmr->_dataBuffer, hcmr->_width * hcmr->_height/2);
	return E_OK;
}

ER
ov7740_setInvert(OV7740_t *hcmr, bool_t invert)
{
	if (invert)
	{
		ov7740_set_hmirror(hcmr, 1); 
		ov7740_set_vflip(hcmr, 1);
	}
	else{
		ov7740_set_hmirror(hcmr, 0); 
		ov7740_set_vflip(hcmr, 0);
	}
	return E_OK;
}

ER
ov7740_set_contrast(OV7740_t *hcmr, int level)
{
	uint8_t tmp = 0;

	level += (NUM_CONTRAST_LEVELS / 2);
	if (level < 0 || level >= NUM_CONTRAST_LEVELS) {
		return E_PAR;
	}
	cambus_readb(hcmr->_slaveAddr, 0x81,&tmp);
	tmp |= 0x20;
	cambus_writeb(hcmr->_slaveAddr, 0x81, tmp);
	cambus_readb(hcmr->_slaveAddr, 0xDA,&tmp);
	tmp |= 0x04;
	cambus_writeb(hcmr->_slaveAddr, 0xDA, tmp);
	cambus_writeb(hcmr->_slaveAddr, 0xE1, contrast_regs[level][0]);
	cambus_writeb(hcmr->_slaveAddr, 0xE2, contrast_regs[level][1]);
	cambus_writeb(hcmr->_slaveAddr, 0xE3, contrast_regs[level][2]);
	cambus_readb(hcmr->_slaveAddr, 0xE4,&tmp);
	tmp &= 0xFB;
	cambus_writeb(hcmr->_slaveAddr, 0xE4, tmp);
	return E_OK;
}

ER
ov7740_set_brightness(OV7740_t *hcmr, int level)
{
	uint8_t tmp = 0;

	level += (NUM_BRIGHTNESS_LEVELS / 2);
	if (level < 0 || level >= NUM_BRIGHTNESS_LEVELS) {
		return E_PAR;
	}
	cambus_readb(hcmr->_slaveAddr, 0x81,&tmp);
	tmp |= 0x20;
	cambus_writeb(hcmr->_slaveAddr, 0x81, tmp);
	cambus_readb(hcmr->_slaveAddr, 0xDA,&tmp);
	tmp |= 0x04;
	cambus_writeb(hcmr->_slaveAddr, 0xDA, tmp);
	cambus_writeb(hcmr->_slaveAddr, 0xE4, brightness_regs[level][0]);
	cambus_writeb(hcmr->_slaveAddr, 0xE3, brightness_regs[level][1]);
	return E_OK;
}

ER
ov7740_set_saturation(OV7740_t *hcmr, int level)
{
	uint8_t tmp = 0;

	level += (NUM_SATURATION_LEVELS / 2 );
	if (level < 0 || level >= NUM_SATURATION_LEVELS) {
		return E_PAR;
	}
	cambus_readb(hcmr->_slaveAddr, 0x81,&tmp);
	tmp |= 0x20;
	cambus_writeb(hcmr->_slaveAddr, 0x81, tmp);
	cambus_readb(hcmr->_slaveAddr, 0xDA,&tmp);
	tmp |= 0x02;
	cambus_writeb(hcmr->_slaveAddr, 0xDA, tmp);
	cambus_writeb(hcmr->_slaveAddr, 0xDD, saturation_regs[level][0]);
	cambus_writeb(hcmr->_slaveAddr, 0xDE, saturation_regs[level][1]);
	return E_OK;
}

ER
ov7740_set_gainceiling(OV7740_t *hcmr, gainceiling_t gainceiling)
{
	uint8_t tmp = 0;
	uint8_t ceiling = (uint8_t)gainceiling;
	if(ceiling > GAINCEILING_32X)
		ceiling = GAINCEILING_32X;
	tmp = (ceiling & 0x07) << 4;
	cambus_writeb(hcmr->_slaveAddr, 0x14, tmp);
	return E_OK;
}

ER
ov7740_set_quality(OV7740_t *hcmr, int qs)
{
	return E_OK;
}

ER
ov7740_set_colorbar(OV7740_t *hcmr, bool_t enable)
{
	if(enable)
	{
		cambus_writeb(hcmr->_slaveAddr, 0x38, 0x07);
		cambus_writeb(hcmr->_slaveAddr, 0x84, 0x02);
	}
	else
	{
		cambus_writeb(hcmr->_slaveAddr, 0x38, 0x07);
		cambus_writeb(hcmr->_slaveAddr, 0x84, 0x00);
	}
	return E_OK;
}

ER
ov7740_set_auto_exposure(OV7740_t *hcmr, bool_t enable, int exposure_us)
{
	uint8_t tmp = 0;

	cambus_readb(hcmr->_slaveAddr, 0x13, &tmp);
	if(enable != 0)
	{
		cambus_writeb(hcmr->_slaveAddr, 0x13, tmp | 0x01);
	}
	else
	{
		cambus_writeb(hcmr->_slaveAddr, 0x13, tmp & 0xFE);
		cambus_writeb(hcmr->_slaveAddr, 0x0F, (uint8_t)(exposure_us>>8));
		cambus_writeb(hcmr->_slaveAddr, 0x10, (uint8_t)exposure_us);
	}
	return E_OK;
}

ER
ov7740_get_exposure_us(OV7740_t *hcmr, int *exposure_us)
{
	uint8_t tmp = 0;

	cambus_readb(hcmr->_slaveAddr, 0x0F, &tmp);
	*exposure_us = tmp<<8 & 0xFF00;
	cambus_readb(hcmr->_slaveAddr, 0x10, &tmp);
	*exposure_us = tmp | *exposure_us;
	return E_OK;
}

ER
ov7740_set_auto_whitebal(OV7740_t *hcmr, bool_t enable, float r_gain_db, float g_gain_db, float b_gain_db)
{
	uint8_t tmp = 0;

	cambus_readb(hcmr->_slaveAddr, 0x80, &tmp);
	if(enable != 0)
	{
		cambus_writeb(hcmr->_slaveAddr, 0x80, tmp | 0x14);
	}
	else
	{
		if((uint16_t)r_gain_db!= 0xFFFF && (uint16_t)g_gain_db!=0xFFFF && (uint16_t)b_gain_db!=0xFFFF)
		{
			cambus_writeb(hcmr->_slaveAddr, 0x80, tmp & 0xEF);
			cambus_writeb(hcmr->_slaveAddr, 0x01, (uint8_t)b_gain_db);
			cambus_writeb(hcmr->_slaveAddr, 0x02, (uint8_t)r_gain_db);
			cambus_writeb(hcmr->_slaveAddr, 0x03, (uint8_t)g_gain_db);
		}
		else
		{
			cambus_writeb(hcmr->_slaveAddr, 0x80, tmp & 0xEB);
		}
	}
	return E_OK;
}

ER
ov7740_set_vflip(OV7740_t *hcmr, bool_t enable)
{
	uint8_t reg;
	cambus_readb(hcmr->_slaveAddr, 0x0C, &reg);
	cambus_writeb(hcmr->_slaveAddr, 0x0C, OV7740_SET_FLIP(reg, enable));
	return E_OK;
}

#ifdef USE_GAIN
ER
ov7740_set_auto_gain(OV7740_t *hcmr, bool_t enable, float gain_db, float gain_db_ceiling)
{
	uint8_t tmp = 0;
	uint16_t gain = (uint16_t)gain_db;
	uint8_t ceiling = (uint8_t)gain_db_ceiling;

	cambus_readb(hcmr->_slaveAddr, 0x13, &tmp);
	if(enable != 0)
	{
		cambus_writeb(hcmr->_slaveAddr, 0x13, tmp | 0x04);
	}
	else
	{
		cambus_writeb(hcmr->_slaveAddr, 0x13, tmp & 0xFB);
		if(gain!=0xFFFF && (uint16_t)gain_db_ceiling!=0xFFFF)
		{
			cambus_readb(hcmr->_slaveAddr, 0x15, &tmp);
			tmp = (tmp & 0xFC) | (gain>>8 & 0x03);
			cambus_writeb(hcmr->_slaveAddr, 0x15, tmp);
			tmp = gain & 0xFF;
			cambus_writeb(hcmr->_slaveAddr, 0x00, tmp);
			tmp = (ceiling & 0x07) << 4;
			cambus_writeb(hcmr->_slaveAddr, 0x14, tmp);
		}
	}
	return E_OK;
}

ER
ov7740_get_gain_db(OV7740_t *hcmr, float *gain_db)
{
	uint8_t tmp = 0;
	uint16_t gain;

	cambus_readb(hcmr->_slaveAddr, 0x00, &tmp);
	gain = tmp;
	cambus_readb(hcmr->_slaveAddr, 0x15, &tmp);
	gain |= ((uint16_t)(tmp & 0x03))<<8;
	*gain_db = (float)gain;
	return E_OK;
}
#endif	/* USE_GAIN */

