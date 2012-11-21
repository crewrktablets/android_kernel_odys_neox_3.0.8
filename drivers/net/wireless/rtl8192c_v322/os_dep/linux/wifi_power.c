/*
 * wifi_power.c
 *
 * Power control for WIFI module.
 * 
 * Yongle Lai @ Rockchip
 *
 * There are Power supply and Power Up/Down controls for WIFI typically.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/jiffies.h>

#include <mach/gpio.h>
#include <mach/iomux.h>
#include "wifi_power.h"
#define GPIO_WIFI_POWER       RK29_PIN6_PC0

/*
 * rtw_channel_plan : The initialization parameter of wifi channel,
 * 					  Allow number is "0" "2" and "5".
 *					  0 => 11 ( channel 1 ~ 11 is SCAN_ACTIVE )
 *					  2 => 13 ( channel 1 ~ 13 is SCAN_ACTIVE )
 *					  5 => 14 ( channel 1 ~ 14 is SCAN_ACTIVE )
 *					  default number is "2".
 */
char init_channel_plan = 2;

#if (WIFI_GPIO_POWER_CONTROL == 1)

/*
 * GPIO to control LDO/DCDC.
 *
 * 用于控制WIFI的电源，通常是3.3V和1.8V，可能1.2V也在其中。
 *
 * 如果是扩展IO，请参考下面的例子:
 *   POWER_USE_EXT_GPIO, 0, 0, 0, PCA9554_Pin1, GPIO_HIGH
 */
struct wifi_power power_gpio = 
{
//	POWER_NOT_USE_GPIO, 0, 0, 0, 0, 0 
#ifdef WIFI_POWER_USE_MUX
		POWER_USE_GPIO, POWER_GPIO_IOMUX, 
		GPIO_WIFI_POWER_MUX_NAME, GPIO_WIFI_POWER_GPIO_MODE, GPIO_WIFI_POWER, GPIO_HIGH 
#else
		POWER_USE_GPIO, 0, 
		0, 0, GPIO_WIFI_POWER, GPIO_HIGH 
#endif
};

/*
 * 在WIFI被上电前，会调用这个函数。
 */
void wifi_turn_on_callback(void)
{
}

/*
 * 在WIFI被下电后，会调用这个函数。
 */
void wifi_turn_off_callback(void)
{
}

/*
 * If external GPIO chip such as PCA9554 is being used, please
 * implement the following 2 function.
 *
 * id:   is GPIO identifier, such as GPIOPortF_Pin0, or external 
 *       name defined in struct wifi_power.
 * sens: the value should be set to GPIO, usually is GPIO_HIGH or GPIO_LOW.
 *
 * 如果有用扩展GPIO来控制WIFI，请实现下面的函数:
 * 函数的功能是：控制指定的IO口id，使其状态切换为要求的sens状态。
 * id  : 是IO的标识号，以整数的形式标识。
 * sens: 是要求的IO状态，为高或低。
 */
void wifi_extgpio_operation(u8 id, u8 sens)
{
	//pca955x_gpio_direction_output(id, sens);
}

/*
 * 在系统中如果要调用WIFI的IO控制，将WIFI下电，可以调用如下接口：
 *   void rockchip_wifi_shutdown(void);
 * 但注意需要在宏WIFI_GPIO_POWER_CONTROL的控制下。
 */

#endif /* WIFI_GPIO_POWER_CONTROL */

