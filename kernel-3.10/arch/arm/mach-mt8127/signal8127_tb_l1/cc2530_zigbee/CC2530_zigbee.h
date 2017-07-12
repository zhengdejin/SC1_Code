
#ifndef __ZIGBEE_CC2530_H__
#define __ZIGBEE_CC2530_H__

#include "cust_gpio_usage.h"

#define ZB_POWER	1	//0	//1

#define uart_flow_control 0

struct zigbee_gpio {
	const char *name;
	unsigned long 	gpio;
	unsigned 	active_level;
};

struct zigbee_platform_data {
	struct zigbee_gpio power_io;
	struct zigbee_gpio reset_io;
	struct zigbee_gpio CFG_io;
	struct zigbee_gpio STATUS_io;
#if ZB_POWER	
	struct zigbee_gpio txd_io;	// zigbee-UART pin
	struct zigbee_gpio rxd_io;	// zigbee-UART pin
#if uart_flow_control
	struct zigbee_gpio rts_io;	// zigbee-UART pin
	struct zigbee_gpio cts_io;	// zigbee-UART pin
#endif
#endif	
};


static struct zigbee_platform_data zigbee_cc2530_info = {
	.power_io = {
		.name = "zb_power_enable",
		.gpio = GPIO_ZB_PWR_EN_PIN,					
		.active_level = 1,						      
	},
	.reset_io = {
		.name = "zb_reset",
		.gpio = GPIO_ZB_RST_PIN,					
		.active_level = 0,			
	},
#if 0	
	.CFG_io = {
		.name = "zb_cfg",
		.gpio = GPIO_ZB_CFG_PIN,					
		.active_level = 0,						      
	},
#endif
	.STATUS_io = {
		.name = "zb_status",
		.gpio = GPIO_ZB_STATUS_PIN,					
		.active_level = 0,						      
	},
	
#if ZB_POWER	
	.txd_io = {
		.name = "zb_uart_rx",
		.gpio = GPIO_ZB_RXD_PIN,					
		.active_level = 1,						      
	},
	.rxd_io = {
		.name = "zb_uart_tx",
		.gpio = GPIO_ZB_TXD_PIN,					
		.active_level = 1,			
	},
	
#if uart_flow_control
	.rts_io = {
		.name = "zb_uart_rts",
		.gpio = GPIO_ZB_RTS_PIN,					
		.active_level = 1,						      
	},
	.cts_io = {
		.name = "zb_uart_cts",
		.gpio = GPIO_ZB_CTS_PIN,					
		.active_level = 1,			
	},
#endif
#endif	
};

static struct platform_device zigbee_cc2530 = {
	.name = "zigbee_cc2530",
	.id	= -1,
	.dev = {
		.platform_data = &zigbee_cc2530_info,
	},
};

#endif

