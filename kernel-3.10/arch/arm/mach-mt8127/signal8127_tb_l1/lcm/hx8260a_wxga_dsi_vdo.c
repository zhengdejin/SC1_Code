/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#include <linux/xlog.h>
#include <mach/mt_pm_ldo.h>
#endif

#include <cust_gpio_usage.h>

#define FRAME_WIDTH  										(800)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID                                  0x06

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

     //unsigned int tmp=0x50;
// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size) 

       


static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[128];
};

static struct LCM_setting_table lcm_initialization_setting[] = {

    /*
Note :

Data ID will depends on the following rule.

count of parameters > 1	=> Data ID = 0x39
count of parameters = 1	=> Data ID = 0x15
count of parameters = 0	=> Data ID = 0x05

Structure Format :

{DCS command, count of parameters, {parameter list}}
{REGFLAG_DELAY, milliseconds of time, {}},

*/
{0x11,0,{0x00}},
{0x29,0,{0x00}},

{REGFLAG_END_OF_TABLE, 0x00, {}}

};

#if 1

//static int vcom = 0x7A;
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;



    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
		/*	  case 0xb6:
			table[i].para_list[0] = vcom;
			table[i].para_list[1] = vcom;
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
			vcom -= 1;
			break;
			*/
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}

#endif
// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
		memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_DISABLED;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

    //params->dsi.mode   = BURST_VDO_MODE; 
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;//SYNC_EVENT_VDO_MODE; //BURST_VDO_MODE; 

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    params->dsi.intermediat_buffer_num = 0;//because DSI/DPI HW design change, this parameters should be 0 when video mode in MT658X; or memory leakage

    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.word_count=800*3;
    params->dsi.vertical_sync_active				= 6;  
    params->dsi.vertical_backporch					= 20; 
    params->dsi.vertical_frontporch					= 20; 
    params->dsi.vertical_active_line				= FRAME_HEIGHT; 
    params->dsi.horizontal_sync_active				= 24;
    params->dsi.horizontal_backporch				= 24;
    params->dsi.horizontal_frontporch				= 72;
    params->dsi.horizontal_active_pixel			= FRAME_WIDTH;


    // Video mode setting		
    //params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
    //params->dsi.pll_select=1;
    params->dsi.PLL_CLOCK = 250;//LCM_DSI_6589_PLL_CLOCK_253_5;//LCM_DSI_6589_PLL_CLOCK_240_5;//LCM_DSI_6589_PLL_CLOCK_227_5;//this value must be in MTK suggested table 227_5

//    params->dsi.pll_div1 = 1;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps  0
//    params->dsi.pll_div2 = 1;		// div2=0,1,2,3;div1_real=1,2,4,4	          	
//    params->dsi.fbk_div = 17;      // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)      	

	
}


static void lcm_init(void)
{
	#ifdef BUILD_LK
    upmu_set_rg_vmch_vosel(1);
    upmu_set_rg_vmch_en(1);
	#else
	    hwPowerOn(MT6323_POWER_LDO_VMCH, VOL_3300, "LCM");
	#endif
    mt_set_gpio_out(GPIO_LCM_PWR,GPIO_OUT_ONE);
    MDELAY(50);

	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
	MDELAY(20);
	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
	MDELAY(20);
	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
    MDELAY(150);
//	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);   //add by yy
#if 0
    unsigned int data_array[16];

    data_array[0] = 0x00B02300;
    dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x20B12300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x43B22300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x28B32300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x73B72300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0xABBA2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0xE8BB2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0xFFBD2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x3FBE2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x50C32300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x14C42300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0xB9C52300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x14C62300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x15C72300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00CA2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x3FCB2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x34CC2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x2CCD2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x2ACE2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x17CF2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0dD02300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0CD12300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x11D22300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x11D32300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x11D42300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0AD52300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x3FD62300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x34D72300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x2CD82300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x2AD92300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x17DA2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0dDB2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0CDC2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x11DD2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x11DE2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x11DF2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0AE02300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x52E52300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x02B02300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x25B12300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00B22300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x11B32300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x12B42300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x09B52300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0BB62300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x05B72300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x07B82300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x01B92300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00BA2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00BB2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00BC2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00BD2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00BE2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00BF2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x03C02300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00C12300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00C22300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00C32300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00C42300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00C52300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00C62300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x25C72300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00C82300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x11C92300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x12CA2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0ACB2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0CCC2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x06CD2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x08CE2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x02CF2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D02300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D12300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D22300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D32300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D42300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D52300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x04D62300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D72300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D82300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00D92300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00DA2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00DB2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00DC2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x03B02300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x00B72300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x31BA2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x31BE2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x0BC42300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x07C62300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x01C72300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x3CCA2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x04CB2300; 
	dsi_set_cmdq(&data_array, 1, 1);
	data_array[0] = 0x05CC2300; 
	dsi_set_cmdq(&data_array, 1, 1);



	data_array[0] = 0x00110500; // Sleep Out
    dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00290500; // Sleep Out
    dsi_set_cmdq(&data_array, 1, 1);
#endif  
	
	unsigned int data_array[16];  

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000B0;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000020B1;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000043B2;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000028B3;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000073B7;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x0000ABBA;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x0000E8BB;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x0000FFBD;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00003FBE;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000050C3;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000014C4;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x0000B9C5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000014C6;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000015C7;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000CA;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00003FCB;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000034CC;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00002CCD;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00002ACE;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000017CF;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000dD0;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000CD1;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000011D2;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000011D3;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000011D4;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000AD5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00003FD6;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000034D7;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00002CD8;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00002AD9;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000017DA;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000dDB;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000CDC;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000011DD;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000011DE;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000011DF;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000AE0;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000052E5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000002B0;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000025B1;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000B2;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000011B3;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000012B4;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000009B5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000BB6;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000005B7;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000007B8;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000001B9;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000BA;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000BB;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000BC;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000BD;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000BE;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000BF;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000003C0;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000C1;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000C2;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000C3;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000C4;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000C5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000C6;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000025C7;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000C8;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000011C9;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000012CA;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000ACB;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000CCC;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000006CD;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000008CE;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000002CF;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D0;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D1;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D2;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D3;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D4;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D5;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000004D6;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D7;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D8;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000D9;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000DA;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000DB;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000DC;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000003B0;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000000B7;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000031BA;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000031BE;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00000BC4;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000007C6;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000001C7;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x00003CCA;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000004CB;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902; 
	data_array[1] = 0x000005CC;
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00110500; // Sleep Out
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(120);
	data_array[0] = 0x00290500; // Sleep Out
	dsi_set_cmdq(&data_array, 1, 1);	
}


static void lcm_suspend(void)
{
#ifdef BUILD_LK
	upmu_set_rg_vmch_en(0);
#else
    hwPowerDown(MT6323_POWER_LDO_VMCH, "LCM");
#endif
	
	mt_set_gpio_out(GPIO_LCM_PWR,GPIO_OUT_ZERO);
    MDELAY(30);

 #if 0
	SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(150);
 #endif
 
 	mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
  
}


static void lcm_resume(void)
{    
 #ifndef BUILD_LK
	lcm_init();
 #endif
	
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00052902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);
	
	data_array[0]= 0x00052902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(&data_array, 3, 1);

	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
	dsi_set_cmdq(&data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(&data_array, 1, 0);

}


static unsigned int lcm_compare_id(void)
{
		unsigned int id=0;
		unsigned char buffer[2];
		unsigned int array[16];  
	
/*	
	#ifdef BUILD_LK
	    upmu_set_rg_vmch_vosel(1);
	    upmu_set_rg_vmch_en(1);
	#else
	    hwPowerOn(MT6323_POWER_LDO_VMCH, VOL_3300, "LCM");
	#endif
	    mt_set_gpio_out(GPIO_LCM_PWR,GPIO_OUT_ONE);
	   
	    mt_set_gpio_mode(GPIO_LCM_RST, GPIO_MODE_00);
	    mt_set_gpio_dir(GPIO_LCM_RST, GPIO_DIR_OUT);
	    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
	    MDELAY(10);
	    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ZERO);
	    MDELAY(10);
	    mt_set_gpio_out(GPIO_LCM_RST, GPIO_OUT_ONE);
	    MDELAY(100);
	    mt_set_gpio_out(GPIO_LCM_PWR,GPIO_OUT_ONE);
        MDELAY(30);
	*/	
		SET_RESET_PIN(1);
		MDELAY(10);
		SET_RESET_PIN(0);
		MDELAY(50);
		SET_RESET_PIN(1);
		MDELAY(120);
		
	    array[0]=0x00043902;//Enable external Command 
        array[1]=0x8012F1B9; 
        dsi_set_cmdq(&array, 2, 1); 
        
		/*array[0]=0x00113902;
		array[1]=0x008213BA;
		array[2]=0x1000c516;
		array[3]=0x03240fff;
		array[4]=0x20252421;
		array[5]=0x00000008;
		dsi_set_cmdq(&array, 6, 1);*/
		
		array[0] = 0x00043700;// return byte number
		dsi_set_cmdq(&array, 1, 1);
		
	    
	    read_reg_v2(0xd0, buffer, 4);
	    id = buffer[0]; 
		
#ifdef BUILD_LK
		printf("=====>compare id for test %s, id = 0x%08x,buffer[1] = 0x%x,buffer[2] = 0x%x,buffer[3] = %x\n", __func__, id,buffer[1],buffer[2],buffer[3]);
#else 
		printk("=====>compare id for test %s, id = 0x%08x,buffer[1] = 0x%x,buffer[2] = 0x%x,buffer[3] = %x\n", __func__, id,buffer[1],buffer[2],buffer[3]);
#endif
	
		return (LCM_ID == id)?1:0;

}

static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
char  buffer[3],buffer1[6],buffer2[3];
	int   array[16],array1[16],array2[16];

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}


	array[0] = 0x00043700;
	dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x09, buffer, 4);
	
	array1[0] = 0x00073700;
	dsi_set_cmdq(array1, 1, 1);
	read_reg_v2(0xB1, buffer1, 7);
	
	printk("esdcheck:0x%x 0x%x 0x%x 0x%x\n",buffer[0],buffer[1],buffer[2],buffer[3]);
	
	if((buffer[0]==0x80)&&(buffer[1]==0x73)&&(buffer[2]==0x4)&&(buffer1[5]==0x11)&&(buffer1[6]==0x11))
	{
		return FALSE;
	}
	else
	{			 
		return TRUE;
	}
 #endif

}

static unsigned int lcm_esd_recover(void)
{
	lcm_init();

	return TRUE;
}

LCM_DRIVER hx8260a_wxga_dsi_vdo_lcm_drv = 
{
    .name		= "hx8260a_wxga_dsi_vdo",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
	//.compare_id     = lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
        //.set_backlight	= lcm_setbacklight,
		//.set_pwm        = lcm_setpwm,
		//.get_pwm        = lcm_getpwm,
        .update         = lcm_update
#endif
    };

