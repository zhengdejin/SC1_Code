/*
 * Generated by MTK SP DrvGen Version 03.13.6 for MT8127. Copyright MediaTek Inc. (C) 2013.
 * Thu Nov 05 11:01:23 2015
 * Do Not Modify the File.
 */

#ifndef __CUST_GPIO_USAGE_H__
#define __CUST_GPIO_USAGE_H__


#define GPIO_PMIC_EINT_PIN         (GPIO2 | 0x80000000)
#define GPIO_PMIC_EINT_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_PMIC_EINT_PIN_M_EINT   GPIO_MODE_00

#define GPIO_AUD_CLK_MOSI_PIN         (GPIO7 | 0x80000000)
#define GPIO_AUD_CLK_MOSI_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_AUD_CLK_MOSI_PIN_M_CLK   GPIO_MODE_01

#define GPIO_AUD_DAT_MISO_PIN         (GPIO8 | 0x80000000)
#define GPIO_AUD_DAT_MISO_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_AUD_DAT_MISO_PIN_M_AUD_MISO   GPIO_MODE_01

#define GPIO_AUD_DAT_MOSI_PIN         (GPIO9 | 0x80000000)
#define GPIO_AUD_DAT_MOSI_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_AUD_DAT_MOSI_PIN_M_AUD_MOSI   GPIO_MODE_01

#define GPIO_ZB_RXD_PIN         (GPIO14 | 0x80000000)
#define GPIO_ZB_RXD_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_ZB_RXD_PIN_M_CLK   GPIO_MODE_05
#define GPIO_ZB_RXD_PIN_M_KROW   GPIO_MODE_06
#define GPIO_ZB_RXD_PIN_M_URXD   GPIO_MODE_01

#define GPIO_ZB_TXD_PIN         (GPIO15 | 0x80000000)
#define GPIO_ZB_TXD_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_ZB_TXD_PIN_M_KROW   GPIO_MODE_06
#define GPIO_ZB_TXD_PIN_M_UTXD   GPIO_MODE_01

#define GPIO_SD_PWR_EN_PIN         (GPIO23 | 0x80000000)
#define GPIO_SD_PWR_EN_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_SD_PWR_EN_PIN_M_PWM   GPIO_MODE_01

#define GPIO_SWCHARGER_EN_PIN         (GPIO24 | 0x80000000)
#define GPIO_SWCHARGER_EN_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_SWCHARGER_EN_PIN_M_CLK   GPIO_MODE_01
#define GPIO_SWCHARGER_EN_PIN_M_KCOL   GPIO_MODE_06
#define GPIO_SWCHARGER_EN_PIN_CLK     CLK_OUT0
#define GPIO_SWCHARGER_EN_PIN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_IR_TRANS_DSM_PIN         (GPIO26 | 0x80000000)
#define GPIO_IR_TRANS_DSM_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_IR_TRANS_DSM_PIN_M_CLK   GPIO_MODE_01
#define GPIO_IR_TRANS_DSM_PIN_CLK     CLK_OUT2
#define GPIO_IR_TRANS_DSM_PIN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_IR_PWR_PIN         (GPIO28 | 0x80000000)
#define GPIO_IR_PWR_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_IR_PWR_PIN_M_KCOL   GPIO_MODE_06

#define GPIO_ZB_RST_PIN         (GPIO31 | 0x80000000)
#define GPIO_ZB_RST_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_ZB_RST_PIN_M_CLK   GPIO_MODE_01
#define GPIO_ZB_RST_PIN_CLK     CLK_OUT4
#define GPIO_ZB_RST_PIN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_ZB_STATUS_PIN         (GPIO32 | 0x80000000)
#define GPIO_ZB_STATUS_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_ZB_STATUS_PIN_M_CLK   GPIO_MODE_01
#define GPIO_ZB_STATUS_PIN_CLK     CLK_OUT5
#define GPIO_ZB_STATUS_PIN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_OTG_DRVVBUS_PIN         (GPIO35 | 0x80000000)
#define GPIO_OTG_DRVVBUS_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_OTG_DRVVBUS_PIN_M_KROW   GPIO_MODE_01

#define GPIO_KPD_KCOL0_PIN         (GPIO36 | 0x80000000)
#define GPIO_KPD_KCOL0_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_KPD_KCOL0_PIN_M_KCOL   GPIO_MODE_01

#define GPIO_KPD_KCOL1_PIN         (GPIO37 | 0x80000000)
#define GPIO_KPD_KCOL1_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_KPD_KCOL1_PIN_M_KCOL   GPIO_MODE_01

#define GPIO_OTG_IDDIG_EINT_PIN         (GPIO38 | 0x80000000)
#define GPIO_OTG_IDDIG_EINT_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_OTG_IDDIG_EINT_PIN_M_KCOL   GPIO_MODE_01
#define GPIO_OTG_IDDIG_EINT_PIN_M_IDDIG   GPIO_MODE_02

#define GPIO_ZB_PWR_EN_PIN         (GPIO43 | 0x80000000)
#define GPIO_ZB_PWR_EN_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_ZB_PWR_EN_PIN_M_CLK   GPIO_MODE_01
#define GPIO_ZB_PWR_EN_PIN_M_KROW   GPIO_MODE_03
#define GPIO_ZB_PWR_EN_PIN_M_PWM   GPIO_MODE_02
#define GPIO_ZB_PWR_EN_PIN_CLK     CLK_OUT4
#define GPIO_ZB_PWR_EN_PIN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_LCM_BL_EN         (GPIO44 | 0x80000000)
#define GPIO_LCM_BL_EN_M_GPIO   GPIO_MODE_00
#define GPIO_LCM_BL_EN_M_CLK   GPIO_MODE_01
#define GPIO_LCM_BL_EN_M_KCOL   GPIO_MODE_03
#define GPIO_LCM_BL_EN_M_PWM   GPIO_MODE_02
#define GPIO_LCM_BL_EN_CLK     CLK_OUT5
#define GPIO_LCM_BL_EN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_ACCDET_EINT_PIN         (GPIO46 | 0x80000000)
#define GPIO_ACCDET_EINT_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_ACCDET_EINT_PIN_M_EINT   GPIO_MODE_00

#define GPIO_MSDC1_INSI         (GPIO47 | 0x80000000)
#define GPIO_MSDC1_INSI_M_GPIO   GPIO_MODE_00
#define GPIO_MSDC1_INSI_M_EINT   GPIO_MODE_00
#define GPIO_MSDC1_INSI_NCE     1

#define GPIO_CTP_EINT_PIN         (GPIO48 | 0x80000000)
#define GPIO_CTP_EINT_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_CTP_EINT_PIN_M_EINT   GPIO_MODE_00
#define GPIO_CTP_EINT_PIN_NCE     0

#define GPIO_ALS_EINT_PIN         (GPIO49 | 0x80000000)
#define GPIO_ALS_EINT_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_ALS_EINT_PIN_M_CLK   GPIO_MODE_03
#define GPIO_ALS_EINT_PIN_M_EINT   GPIO_MODE_00
#define GPIO_ALS_EINT_PIN_CLK     CLK_OUT0
#define GPIO_ALS_EINT_PIN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_GYRO_EINT_PIN         (GPIO50 | 0x80000000)
#define GPIO_GYRO_EINT_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_GYRO_EINT_PIN_M_CLK   GPIO_MODE_02
#define GPIO_GYRO_EINT_PIN_M_EINT   GPIO_MODE_00
#define GPIO_GYRO_EINT_PIN_CLK     CLK_OUT3
#define GPIO_GYRO_EINT_PIN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_IR_SHIFT_ON         (GPIO51 | 0x80000000)
#define GPIO_IR_SHIFT_ON_M_GPIO   GPIO_MODE_00
#define GPIO_IR_SHIFT_ON_M_CLK   GPIO_MODE_04
#define GPIO_IR_SHIFT_ON_M_UCTS   GPIO_MODE_01
#define GPIO_IR_SHIFT_ON_CLK     CLK_OUT1
#define GPIO_IR_SHIFT_ON_FREQ    GPIO_CLKSRC_NONE

#define GPIO_CTP_RST_PIN         (GPIO52 | 0x80000000)
#define GPIO_CTP_RST_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_CTP_RST_PIN_M_CLK   GPIO_MODE_04
#define GPIO_CTP_RST_PIN_CLK     CLK_OUT2
#define GPIO_CTP_RST_PIN_FREQ    GPIO_CLKSRC_NONE

#define GPIO_CAMERA_CMRST1_PIN         (GPIO53 | 0x80000000)
#define GPIO_CAMERA_CMRST1_PIN_M_GPIO   GPIO_MODE_00

#define GPIO_CAMERA_CMPDN1_PIN         (GPIO54 | 0x80000000)
#define GPIO_CAMERA_CMPDN1_PIN_M_GPIO   GPIO_MODE_00

#define GPIO_CAMERA_CMRST_PIN         (GPIO55 | 0x80000000)
#define GPIO_CAMERA_CMRST_PIN_M_GPIO   GPIO_MODE_00

#define GPIO_CAMERA_CMPDN_PIN         (GPIO56 | 0x80000000)
#define GPIO_CAMERA_CMPDN_PIN_M_GPIO   GPIO_MODE_00

#define GPIO_I2C1_SDA_PIN         (GPIO57 | 0x80000000)
#define GPIO_I2C1_SDA_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_I2C1_SDA_PIN_M_SDA   GPIO_MODE_01

#define GPIO_I2C1_SCA_PIN         (GPIO58 | 0x80000000)
#define GPIO_I2C1_SCA_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_I2C1_SCA_PIN_M_SCL   GPIO_MODE_01

#define GPIO_I2C0_SDA_PIN         (GPIO75 | 0x80000000)
#define GPIO_I2C0_SDA_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_I2C0_SDA_PIN_M_SDA   GPIO_MODE_01

#define GPIO_I2C0_SCA_PIN         (GPIO76 | 0x80000000)
#define GPIO_I2C0_SCA_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_I2C0_SCA_PIN_M_SCL   GPIO_MODE_01

#define GPIO_UART_URXD1_PIN         (GPIO79 | 0x80000000)
#define GPIO_UART_URXD1_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_UART_URXD1_PIN_M_URXD   GPIO_MODE_01

#define GPIO_UART_UTXD1_PIN         (GPIO80 | 0x80000000)
#define GPIO_UART_UTXD1_PIN_M_GPIO   GPIO_MODE_00
#define GPIO_UART_UTXD1_PIN_M_UTXD   GPIO_MODE_01

#define GPIO_LCM_PWR         (GPIO83 | 0x80000000)
#define GPIO_LCM_PWR_M_GPIO   GPIO_MODE_00

#define GPIO_LCM_RST         (GPIO142 | 0x80000000)
#define GPIO_LCM_RST_M_GPIO   GPIO_MODE_00


/*Output for default variable names*/
/*@XXX_XX_PIN in gpio.cmp          */




/*Output for default variable names*/
/*@XXX_XX_PIN in gpio.cmp          */



#endif /* __CUST_GPIO_USAGE_H__ */


