
#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/types.h>
#include <linux/platform_device.h>
#include <asm/uaccess.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <mach/mt_gpio.h>

#include "CC2530_zigbee.h"

#define ZIGBEE_DEBUG
#define ZIGBEE_SYS_DEBUG

#ifdef  ZIGBEE_DEBUG
#define ZIGBEE_DBG(x...)	printk(KERN_INFO x)
#else
#define ZIGBEE_DBG(x...)
#endif

//static DEFINE_SPINLOCK(zigbee_cc2530_lock);
static struct zigbee_gpio power_pin, reset_pin;
#if ZB_POWER
static struct zigbee_gpio txd_pin, rxd_pin;
#if uart_flow_control
	static struct zigbee_gpio rts_pin, cts_pin;
#endif
#endif
static atomic_t status_flag;

static void zigbee_gpio_request(unsigned long pin,const char *label){
       			switch (pin){
			 	case GPIO_ZB_PWR_EN_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_PWR_EN_PIN_M_GPIO);
					break;
				case GPIO_ZB_RST_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_RST_PIN_M_GPIO);
					break;
				case GPIO_ZB_RXD_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_RXD_PIN_M_GPIO);
					break;
				case GPIO_ZB_TXD_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_TXD_PIN_M_GPIO);
					break;
				#if uart_flow_control
				case GPIO_ZB_RTS_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_RTS_PIN_M_GPIO);
					break;
				case GPIO_ZB_CTS_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_CTS_PIN_M_GPIO);
					break;
				#endif
				
				default:
					ZIGBEE_DEBUG("no this pin");
					break;
       			}
	
}

static void zigbee_gpio_direction_output(unsigned long pin,unsigned level){ 
                 mt_set_gpio_dir(pin, GPIO_DIR_OUT);
                 mt_set_gpio_out(pin, level);
 }

static void zigbee_gpio_direction_input(unsigned long pin) {   
                mt_set_gpio_dir(pin, GPIO_DIR_IN);
}
                                       
static void iomux_mode_to_gpio(unsigned long pin)			
{
				switch (pin){
				case GPIO_ZB_RXD_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_RXD_PIN_M_GPIO);
					break;
				case GPIO_ZB_TXD_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_TXD_PIN_M_GPIO);
					break;
				#if uart_flow_control
				case GPIO_ZB_RTS_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_RTS_PIN_M_GPIO);
					break;
				case GPIO_ZB_CTS_PIN:
					mt_set_gpio_mode(pin,GPIO_ZB_CTS_PIN_M_GPIO);
					break;
				#endif
				
				default:
					ZIGBEE_DEBUG("no this pin");
					break;
			}
}

static void iomux_mode_to_uart(unsigned long pin)	{
            switch (pin){
			case GPIO_ZB_RXD_PIN:
				mt_set_gpio_mode(pin,GPIO_ZB_RXD_PIN_M_URXD);
				break;
			case GPIO_ZB_TXD_PIN:
				mt_set_gpio_mode(pin,GPIO_ZB_TXD_PIN_M_UTXD);
				break;		
			#if uart_flow_control
			case GPIO_ZB_CTS_PIN:
				mt_set_gpio_mode(pin,GPIO_ZB_CTS_PIN_M_UCTS);
				break;
			case GPIO_ZB_RTS_PIN:
				mt_set_gpio_mode(pin,GPIO_ZB_RTS_PIN_M_URTS);
				break;
			#endif	
			default:
				ZIGBEE_DEBUG("no this pin");
				break;
            }
}


#ifdef ZIGBEE_SYS_DEBUG
static struct class *zigbee_power_class;
static ssize_t zigbee_power_read(struct class *cls, struct class_attribute *attr, char *_buf)
{
	int count = 0;
		
	if (atomic_read(&status_flag)){
		count = sprintf(_buf, "%s", "zigbee_on\n");
	}
	else{
		count = sprintf(_buf, "%s", "zigbee_off\n");
	}
	 
	return count;
}

static ssize_t zigbee_power_write(struct class *cls, struct class_attribute *attr, const char *_buf, size_t _count)
{
	int poweren = 0;
	poweren = simple_strtol(_buf, NULL, 10);
	ZIGBEE_DBG("%s: poweren = %d\n", __func__, poweren);
	if (poweren > 0) {
		atomic_set(&status_flag, 1);
		zigbee_gpio_direction_output(power_pin.gpio, power_pin.active_level);
		zigbee_gpio_direction_output(reset_pin.gpio, !reset_pin.active_level);
		mdelay(1);
		zigbee_gpio_direction_output(reset_pin.gpio, reset_pin.active_level);
		mdelay(1);
		zigbee_gpio_direction_output(reset_pin.gpio, !reset_pin.active_level);
		mdelay(1);
#if ZB_POWER	
		// switch to UART pin
		iomux_mode_to_uart(txd_pin.gpio);
		iomux_mode_to_uart(rxd_pin.gpio);
#endif
	}
	else {
		atomic_set(&status_flag, 0);
		zigbee_gpio_direction_output(power_pin.gpio, !power_pin.active_level);
		zigbee_gpio_direction_input(reset_pin.gpio);
#if ZB_POWER	
		// switch to GPIO pin and set to low level 
		iomux_mode_to_gpio(txd_pin.gpio);
		zigbee_gpio_request(txd_pin.gpio, txd_pin.name);
		zigbee_gpio_direction_input(txd_pin.gpio);
		iomux_mode_to_gpio(rxd_pin.gpio);
		zigbee_gpio_request(rxd_pin.gpio, rxd_pin.name);	
		zigbee_gpio_direction_input(rxd_pin.gpio);
#endif	
	}
		 
	return _count;
}

static CLASS_ATTR(power, 0666, zigbee_power_read, zigbee_power_write);
#endif


static int zigbee_cc2530__open(struct inode *inode, struct file *file)
{
	ZIGBEE_DBG("%s\n", __FUNCTION__);
	
	atomic_set(&status_flag, 1);	
	zigbee_gpio_direction_output(power_pin.gpio, power_pin.active_level);	
	zigbee_gpio_direction_output(reset_pin.gpio, !reset_pin.active_level);
	mdelay(1);
	zigbee_gpio_direction_output(reset_pin.gpio, reset_pin.active_level);
	mdelay(1);	
	zigbee_gpio_direction_output(reset_pin.gpio, !reset_pin.active_level);
	
#if ZB_POWER	
	// switch to UART pin
	iomux_mode_to_uart(txd_pin.gpio);
	iomux_mode_to_uart(rxd_pin.gpio);
#endif
	mdelay(1);
	
	return 0;
}

static int zigbee_cc2530__release(struct inode *inode, struct file *file)
{
	ZIGBEE_DBG("%s\n", __FUNCTION__);
	
	atomic_set(&status_flag, 0);
	zigbee_gpio_direction_output(power_pin.gpio, !power_pin.active_level);
	zigbee_gpio_direction_input(reset_pin.gpio);
	
#if ZB_POWER	
	// switch to GPIO pin and set to low level 
	iomux_mode_to_gpio(txd_pin.gpio);
	zigbee_gpio_request(txd_pin.gpio, txd_pin.name);
	zigbee_gpio_direction_input(txd_pin.gpio);
	iomux_mode_to_gpio(rxd_pin.gpio);
	zigbee_gpio_request(rxd_pin.gpio, rxd_pin.name);	
	zigbee_gpio_direction_input(rxd_pin.gpio);
#endif	
	return 0;
}

static long zigbee_cc2530__ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	ZIGBEE_DBG("%s\n",__FUNCTION__);
	return 0;
}

static struct file_operations zigbee_cc2530__fops = {
	.owner = THIS_MODULE,
	.open = zigbee_cc2530__open,
	.release = zigbee_cc2530__release,
	.unlocked_ioctl = zigbee_cc2530__ioctl,
};

static struct miscdevice zigbee_cc2530__misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "zigbee_cc2530",
	.fops = &zigbee_cc2530__fops
};

static int zigbee_cc2530__probe(struct platform_device *pdev)
{
	struct zigbee_platform_data *pdata = pdev->dev.platform_data;
	int result = 0;
	ZIGBEE_DBG("%s\n", __FUNCTION__);

	power_pin = pdata->power_io;
	reset_pin = pdata->reset_io;
#if ZB_POWER	
	txd_pin = pdata->txd_io;
	rxd_pin = pdata->rxd_io;
#if uart_flow_control
	rts_pin = pdata->rts_io;
	cts_pin = pdata->cts_io;
#endif
#endif

	// request GPIO: ZB_POWER_ENABLE
	zigbee_gpio_request(power_pin.gpio, power_pin.name);
	
	// request GPIO: ZB_RESET
	zigbee_gpio_request(reset_pin.gpio, reset_pin.name);

#if ZB_POWER	
	// request GPIO: ZIGBEE_TXD
	iomux_mode_to_gpio(txd_pin.gpio);
	zigbee_gpio_request(txd_pin.gpio, txd_pin.name);
	
	// request GPIO: ZIGBEE_RXD
	iomux_mode_to_gpio(rxd_pin.gpio);
	zigbee_gpio_request(rxd_pin.gpio, rxd_pin.name);
	
#if uart_flow_control
	// request GPIO: ZIGBEE_RTS
	iomux_mode_to_gpio(rts_pin.gpio);
	zigbee_gpio_request(rts_pin.gpio, rts_pin.name);
	
	// request GPIO: ZIGBEE_CTS	
	iomux_mode_to_gpio(cts_pin.gpio);
	zigbee_gpio_request(cts_pin.gpio, cts_pin.name);
#endif

#endif

	atomic_set(&status_flag, 0);
	zigbee_gpio_direction_output(power_pin.gpio, !power_pin.active_level);
	zigbee_gpio_direction_input(reset_pin.gpio);

#if ZB_POWER	
	zigbee_gpio_direction_input(txd_pin.gpio);
	zigbee_gpio_direction_input(rxd_pin.gpio);
#if uart_flow_control
	zigbee_gpio_direction_input(rts_pin.gpio);
	zigbee_gpio_direction_input(cts_pin.gpio);
#endif
#endif

	result = misc_register(&zigbee_cc2530__misc);
	if(result){
		goto exit_misc_device_register_failed;
	}	

#ifdef ZIGBEE_SYS_DEBUG
	zigbee_power_class = NULL;
	zigbee_power_class = class_create(THIS_MODULE, "zigbee_power");
	result = IS_ERR(zigbee_power_class);
	if (result) {   
		printk("Create class zigbee_power_class failed.\n");
		goto exit_class_register_failed;
	}
	result = class_create_file(zigbee_power_class, &class_attr_power);
	if (result) {   
		printk("Create class zigbee_power_class failed.\n");
		goto exit_class_register_failed;
	}
#endif

	return 0;

exit_misc_device_register_failed:
exit_class_register_failed:
	
	return result;
}

static int zigbee_cc2530__remove(struct platform_device *pdev)
{
	return 0;
}

static int zigbee_cc2530__suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int zigbee_cc2530__resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver zigbee_cc2530__driver = {
	.probe = zigbee_cc2530__probe,
	.remove = zigbee_cc2530__remove,
	.suspend = zigbee_cc2530__suspend,
	.resume = zigbee_cc2530__resume,
	.driver = {
		.name	= "zigbee_cc2530",
		.owner	= THIS_MODULE,
	},
};

static int __init zigbee_cc2530__init(void)
{
	platform_device_register(&zigbee_cc2530);
	
	return platform_driver_register(&zigbee_cc2530__driver);
}

static void __exit zigbee_cc2530__exit(void)
{
	platform_driver_unregister(&zigbee_cc2530__driver);
}

late_initcall(zigbee_cc2530__init);
module_exit(zigbee_cc2530__exit);

MODULE_DESCRIPTION ("zigbee cc2530 driver");
MODULE_LICENSE("GPL");

