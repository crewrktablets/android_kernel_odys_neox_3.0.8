/*
 * rk29_m911.h
 *
 * Overview:  
 *
 * Copyright (c) 2011, YiFang Digital
 *
 * Version:  1.0
 * Created:  02/22/2011 03:36:04 PM
 * Author:  zqqu <zqqu@yifangdigital.com>
 * Company:  YiFang Digital
 * History:
 *
 * 
 */
/***************************************************
 *
 *				    LCD  
 *
 **************************************************/

#define LCD_TXD_PIN          INVALID_GPIO
#define LCD_CLK_PIN          INVALID_GPIO
#define LCD_CS_PIN           INVALID_GPIO

#define FB_ID                       0
#define FB_DISPLAY_ON_PIN           RK29_PIN6_PD1
#define FB_LCD_STANDBY_PIN          RK29_PIN6_PD0
#define FB_LCD_CABC_EN_PIN          INVALID_GPIO
#define FB_MCU_FMK_PIN              INVALID_GPIO

#define FB_DISPLAY_ON_VALUE         GPIO_HIGH
#define FB_LCD_STANDBY_VALUE        GPIO_HIGH

#if defined(CONFIG_MACH_RK29_ODYS_NEOX8)
/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#define OUT_CLK		40000000
#define LCDC_ACLK		150000000     //29 lcdc axi DMA

/* Timing */
#define H_PW			1
#define H_BP			46
#define H_VD			800
#define H_FP			210

#define V_PW			3
#define V_BP			23
#define V_VD			600
#define V_FP			12

/* Other */
#define DCLK_POL		0
#define SWAP_RB		0

#define LCD_WIDTH       202
#define LCD_HEIGHT      152

#elif defined(CONFIG_MACH_RK29_ODYS_NEOX7)
/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#define OUT_CLK			32000000
#define LCDC_ACLK		400000000     //29 lcdc axi DMA

/* Timing */
#define H_PW			48
#define H_BP			40
#define H_VD			800
#define H_FP			62

#define V_PW			3
#define V_BP			29
#define V_VD			480
#define V_FP			48

/* Other */
#define DCLK_POL		0
#define SWAP_RB			0

#define LCD_WIDTH       	154
#define LCD_HEIGHT      	87 

#elif defined(CONFIG_MACH_RK29_ODYS_Q)
/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_D888_P666
#define OUT_CLK		 	100000000
#define LCDC_ACLK           	500000000
/* Timing */
#define H_PW			380
#define H_BP			540
#define H_VD			1024
#define H_FP			300

#define V_PW			15
#define V_BP			10
#define V_VD			768
#define V_FP			30

/* Other */
#define DCLK_POL		0 // 
#define SWAP_RB			0

#define LCD_WIDTH   		196// 142  // 202
#define LCD_HEIGHT  		147 //106//  152

#else
// Base 
#define OUT_TYPE		SCREEN_RGB

#define OUT_FACE		OUT_D888_P666  
#define OUT_CLK			40000000
#define LCDC_ACLK        	150000000//312000000           //29 lcdc axi DMA

// Timing 
#define H_PW			1
#define H_BP			46
#define H_VD			800
#define H_FP			210

#define V_PW			3
#define V_BP			23
#define V_VD			480
#define V_FP			12


// Other 
#define DCLK_POL		0
#define SWAP_RB			0

#define LCD_WIDTH       	170
#define LCD_HEIGHT      	128
#endif

/***************************************************
 *
 *				     BACKLIGHG
 *
 **************************************************/
#define PWM_ID            0
#define PWM_MUX_NAME      GPIO1B5_PWM0_NAME
#define PWM_MUX_MODE      GPIO1L_PWM0
#define PWM_MUX_MODE_GPIO GPIO1L_GPIO1B5
#define PWM_EFFECT_VALUE  0
#define PWM_GPIO		RK29_PIN1_PB5

/**the value of MIN_BACKLIGHT_SCALE must be between 0~10*/
#define MIN_BACKLIGHT_SCALE	12



//#define LCD_DISP_ON_PIN

#ifdef  LCD_DISP_ON_PIN
#define BL_EN_MUX_NAME    GPIOF34_UART3_SEL_NAME
#define BL_EN_MUX_MODE    IOMUXB_GPIO1_B34

#define BL_EN_PIN         GPIO0L_GPIO0A5
#define BL_EN_VALUE       GPIO_HIGH
#endif

/***************************************************
 *
 *                      BATTERY 
 *
 **************************************************/
#define DC_DET_EFFECTIVE		1
#define CHG_OK_EFFECTIVE		1
#define GPIO_DC_DET			RK29_PIN4_PA1
#define GPIO_CHG_OK			RK29_PIN4_PA3
#define ADC_ADD_VALUE		1
#define ADC_CLI_VALUE		50
#define CHARGE_FULL_GATE 		4150

//This parameter is for new battery driver//
#define	TIMER_MS_COUNTS		            50	//timers length(ms)

#define	SLOPE_SECOND_COUNTS	            15	//time interval(s) for computing voltage slope
#define	DISCHARGE_MIN_SECOND	        60	//minimum time interval for discharging 1% battery
#define	CHARGE_MIN_SECOND	            90	//minimum time interval for charging 1% battery
#define	CHARGE_MID_SECOND	            160	//time interval for charging 1% battery when battery capacity over 80%
#define	CHARGE_MAX_SECOND	            220	//max time interval for charging 1% battery

#define CHARGE_FULL_DELAY_TIMES         10  //delay time when charging FULL
#define USBCHARGE_IDENTIFY_TIMES        5   //time for identifying USB and Charge
#define STABLE_SECOND					8  //check ok µçÆ½»á»Î¶¯¡£¡£
#define SHUTDOWN_SECOND					20
#define SPEEDLOSE_SECOND                120 //play game rapid down

#define	NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//samling numbers
#define	NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	
#define	NUM_CHARGE_MIN_SAMPLE	        ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MID_SAMPLE	        ((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MAX_SAMPLE	        ((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_STABLE_SAMPLE				((STABLE_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SHUTD0WN_SAMPLE             ((SHUTDOWN_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SPEEDLOSE_SAMPLE  			((SPEEDLOSE_SECOND * 1000) / TIMER_MS_COUNTS)

#define BAT_2V5_VALUE	        2500
#define BATT_MAX_VOL_VALUE	    4190	//voltage of FULL battery
#define	BATT_ZERO_VOL_VALUE     3500	//voltage when poweroff
#define BATT_NOMAL_VOL_VALUE    3800
#define SHUTDOWNVOLTAGE			3400
//define  divider resistors for ADC sampling, units as K
#define BAT_PULL_UP_R           549
#define BAT_PULL_DOWN_R         200


/***************************************************
 *
 *                  CAMERA SENSOR
 *
 **************************************************/
#define CONFIG_SENSOR_0 RK29_CAM_SENSOR_OV5642			/* back camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_0 	    0x78
#define CONFIG_SENSOR_IIC_ADAPTER_ID_0    1
#define CONFIG_SENSOR_ORIENTATION_0       90
#define CONFIG_SENSOR_POWER_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_0       RK29_PIN6_PB7
#define CONFIG_SENSOR_FALSH_PIN_0         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_0 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_0 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_0 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_0 RK29_CAM_FLASHACTIVE_L
#define OV5642_BP_REGULATOR	0x0b	

#define CONFIG_SENSOR_QCIF_FPS_FIXED_0      15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_0      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_0       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_0       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_0      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_0      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_0      30000


//---------------------------------------------------------------------------
#if defined CONFIG_SOC_CAMERA_GT2005
#define CONFIG_SENSOR_1 RK29_CAM_SENSOR_GT2005 /* front camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_1				0x78
#define GT2005_FLIP 1
#elif defined CONFIG_SOC_CAMERA_NT99250
#define CONFIG_SENSOR_1 RK29_CAM_SENSOR_NT99250 /* front camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_1				0x6c
#if defined CONFIG_OLD_NT99250
#define NT99250_X_OFFSET	2
#define NT99250_X_OFFSET	2
#define VAL_306D	0x00
#endif 	
#define NT99250_MIRROR	0
#define NT99250_FLIP		0
#elif defined CONFIG_SOC_CAMERA_FCAM
#define CONFIG_SENSOR_1 RK29_CAM_SENSOR_FCAM /* front camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_1				0xff
#elif defined CONFIG_SOC_CAMERA_GC0307
#define CONFIG_SENSOR_1						RK29_CAM_SENSOR_GC0307	/* front camera sensor */
#define CONFIG_SENSOR_IIC_ADDR_1			0x42
#endif

#define GC0308_MIRROR	0
#define GC0308_FLIP		0

#define CONFIG_SENSOR_IIC_ADAPTER_ID_1    1
#define CONFIG_SENSOR_ORIENTATION_1       270
#define CONFIG_SENSOR_POWER_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_RESET_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERDN_PIN_1       RK29_PIN5_PD7
#define CONFIG_SENSOR_FALSH_PIN_1         INVALID_GPIO
#define CONFIG_SENSOR_POWERACTIVE_LEVEL_1 RK29_CAM_POWERACTIVE_L
#define CONFIG_SENSOR_RESETACTIVE_LEVEL_1 RK29_CAM_RESETACTIVE_L
#define CONFIG_SENSOR_POWERDNACTIVE_LEVEL_1 RK29_CAM_POWERDNACTIVE_H
#define CONFIG_SENSOR_FLASHACTIVE_LEVEL_1 RK29_CAM_FLASHACTIVE_L
#define CONFIG_SENSOR_QCIF_FPS_FIXED_1      15000
#define CONFIG_SENSOR_QVGA_FPS_FIXED_1      15000
#define CONFIG_SENSOR_CIF_FPS_FIXED_1       15000
#define CONFIG_SENSOR_VGA_FPS_FIXED_1       15000
#define CONFIG_SENSOR_480P_FPS_FIXED_1      15000
#define CONFIG_SENSOR_SVGA_FPS_FIXED_1      15000
#define CONFIG_SENSOR_720P_FPS_FIXED_1      30000

/***************************************************
 *
 *                      HDMI
 *
 **************************************************/
#define NO_IOMUX_PINNAME  NULL
#define NO_IO_MUX_MODE		NULL
#define ANX7150_ATTACHED_BUS	1	//attached to I2C1
#define	GPIO_ANX7150_RST	RK29_PIN2_PC7
#define ANX7150_RST_MUX_NAME	NO_IOMUX_PINNAME
#define ANX7150_RST_MUX_MODE	NO_IO_MUX_MODE
