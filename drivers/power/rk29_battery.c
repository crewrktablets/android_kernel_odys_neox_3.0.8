
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/debugfs.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <asm/types.h>
#include <asm/io.h>
#include <asm/delay.h>
#include <asm/mach/time.h>
#include <linux/gpio.h>
#include <linux/adc.h>
#include <mach/board.h>

#define BAT_DBG 0
#if BAT_DBG
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

#if defined (CONFIG_RK2818_M10)
#define BATT_FULL_VALUE	       8400
#define CHARGE_FULL_GATE	8400
#define BATT_NOMAL_VOL_VALUE	8000
#define BATT_VOLTAGE_MAX	8400
#define BATT_VOLTAGE_MIN	6800
#else
#define BATT_FULL_VALUE	       4200
#define BATT_NOMAL_VOL_VALUE	4000
#define BATT_VOLTAGE_MAX	4200
#define BATT_VOLTAGE_MIN	3400//3300//3500
#endif

#define PERCENT				100
#define BATT_LEVEL_FULL		100
#define BATT_LEVEL_EMPTY	0
#define BATT_PRESENT_TRUE	 1
#define BATT_PRESENT_FALSE  0

#define AD_SAMPLE_TIMES	6
#define AC_OFFSET     500
#define PER_MINUTE	600//(60*1000*1000*1000/(TS_POLL_DELAY))


#define AD_NO_BATT_VALE       200
#define AD_NO_DC_VALE         200

#define TS_POLL_DELAY		(100*1000*1000)
#define SEC_NUM				4  ///8
#define PER_SEC_NUM		10  ///10


#if !defined (ADC_CLI_VALUE) 
#define ADC_CLI_VALUE 15	//change 30 to 15 because ADC sampling value is some low on our most board
#endif

static int bat_vol_cnt = 0;  
static int bat_vol_up_cnt = 0; 
static int bat_vol_no_power_cnt = 0;  
static int bat_status =  POWER_SUPPLY_STATUS_UNKNOWN;
static int bat_health = POWER_SUPPLY_HEALTH_GOOD;
static int bat_capacity = BATT_LEVEL_EMPTY;
static int bat_present = BATT_PRESENT_TRUE;
static int bat_voltage =  BATT_NOMAL_VOL_VALUE;
static int ad_sample_current_time = 0;
unsigned int sample_times = 0;			/*count times (report batter status)*/
static int booting_with_charger_cnt = 10;
static int chg_ok_gpio, dc_det_gpio;
static int battery_status_change = 0, sample_num_after_status_change = 0;
static int old_report_bat_capacity = 0;
static int old_bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
static int old_bat_capacity = 0;
static int bat_capacity_array[32] = {0};
static int bat_first_sample = 1;

static int power_state_change = 0;
static int num_after_power_state_change = 0;

static int times_after_first_report = 600; //will not report battery capacity 1 minutesafter first report battery


#if defined (CONFIG_RK2818_M10)
static int batt_step_table[56]={
    6800,6840,6860,6880,6900,6930,6960,6990,7020,7040,
	7060,7090,7120,7150,7180,7200,7220,7240,7260,7280,
	7300,7320,7340,7360,7380,7400,7420,7440,7470,7500,
	7530,7560,7580,7610,7640,7670,7770,7730,7760,7790,
	7820,7850,7880,7910,7940,7970,8000,8030,8060,8090,
	8120,8150,8200,8260,8300,8400	
};

//电池充电数组
static int batt_no_current_step_table[56]={
    6800,6840,6860,6880,6900,6930,6960,6990,7020,7040,
	7060,7090,7120,7150,7180,7200,7220,7240,7260,7280,
	7300,7320,7340,7360,7380,7400,7420,7440,7470,7500,
	7530,7560,7580,7610,7640,7670,7770,7730,7760,7790,
	7820,7850,7880,7910,7940,7970,8000,8030,8060,8090,
	8120,8150,8200,8260,8300,8400	
};
#else
#if defined (CONFIG_MACH_M803) || defined (CONFIG_MACH_M803HD) //battery: AE4169100P6HAS: 6000mAH
static int batt_step_table[15]={
	//3494,3515,3539,3574,3612,3646,3688,3703,3729,3782,3851,3928,4007,4111,4200
	3500,3515,3542,3580,3615,3655,3735,3750,3765,3805,3870,3940,4031,4100,4200
};
static int batt_no_current_step_table[15]={
	3505,3573,3604,3660,3708,3753,3843,3865,3888,3935,4004,4085,4150,4300,6000
};
#elif defined (CONFIG_MACH_M908)  //battery: AE4169100P6HAS: 6000mAH
static int batt_step_table[15]={
	3495,3510,3540,3580,3615,3655,3735,3750,3765,3805,3870,3940,4000,4070,4200
	//3300,3400,3450,3580,3615,3655,3735,3750,3765,3805,3870,3940,4030,4100,4200
};
static int batt_no_current_step_table[15]={
	3505,3728,3760,3795,3820,3870,3945,3962,3980,4010,4075,4130,4180,4300,6000
};
#elif defined (CONFIG_MACH_M911) //battery: 37100100: 4000mAH
static int batt_step_table[15]={
	//3494,3515,3539,3574,3612,3646,3688,3703,3729,3782,3851,3928,4007,4111,4200
	3494,3508,3517,3552,3576,3590,3609,3632,3668,3713,3782,3845,3928,4045,4200
};
static int batt_no_current_step_table[15]={
	//3500,3700,3720,3755,3795,3830,3884,3933,3952,3976,4014,4073,4149,4300,4500
	3500,3700,3720,3755,3759,3774,3805,3863,3900,3930,4000,4073,4149,4300,4500
	//3500,3608,3680,3717,3759,3803,3884,3933,3952,3976,4014,4073,4149,4300,4500 //original data
};
#elif defined (CONFIG_MACH_A7HC)
static int batt_step_table[15]={
	3500,3515,3539,3579,3610,3649,3676,3694,3731,3789,3856,3927,4007,4200,4500
};
static int batt_no_current_step_table[15]={
	3691,3700,3720,3750,3775,3800,3827,3845,3885,3950,4007,4078,4158,4300,6000
};
#elif defined (CONFIG_MACH_A7HTC) || defined(CONFIG_MACH_A70HT3R)  //battery: AE4585102P8HSM: 4300mAH
static int batt_step_table[15]={		
	//3500,3603,3646,3658,3686,3708,3733,3753,3784,3829,3899,3962,4057,4158,4500  //origianl data
	3475,3520,3570,3608,3636,3658,3690,3705,3725,3780,3855,3920,4010,4110,4500
};
static int batt_no_current_step_table[15]={ 
	//3500,3633,3712,3727,3757,3784,3807,3824,3860,3922,3982,4058,4157,4300,6000  //origianl data
	3500,3680,3730,3760,3800,3824,3847,3880,3920,3975,4030,4100,4150,4300,6000
};
#elif defined (CONFIG_MACH_M726) || defined (CONFIG_MACH_M726HN)
static int batt_step_table[15]={
	3495,3505,3510,3535,3560,3585,3635,3685,3735,3785,3828,3870,3925,4050,4500
};
static int batt_no_current_step_table[15]={
	3500,3720,3730,3765,3790,3815,3856,3895,3945,3995,4030,4065,4110,4300,6000
};
#elif defined (CONFIG_MACH_M900) || defined (CONFIG_MACH_M900HD) || defined (CONFIG_MACH_M900HDW)
static int batt_step_table[15]={
	3485,3500,3580,3630,3670,3690,3715,3740,3780,3835,3900,3970,4040,4115,4500
};
static int batt_no_current_step_table[15]={
	3500,3690,3725,3770,3805,3825,3855,3870,3910,3960,4030,4085,4150,4200,6000
};
#elif defined (CONFIG_MACH_M732)
static int batt_step_table[15]={		
	3495,3510,3540,3575,3615,3645,3690,3705,3730,3780,3850,3930,4000,4080,4200 //origianl data
};
static int batt_no_current_step_table[15]={ 
	3500,3725,3750,3780,3820,3845,3885,3900,3925,3975,4050,4110,4170,4300,4500
};
#elif defined (CONFIG_MACH_M737)
static int batt_step_table[15]={		
	3495,3510,3540,3575,3615,3645,3690,3705,3730,3780,3850,3930,4000,4080,4200 //origianl data
};
static int batt_no_current_step_table[15]={ 
	3500,3725,3750,3780,3820,3845,3885,3900,3925,3975,4050,4110,4170,4300,4500
};
#else   //battery: 37100100: 4000mAH  
static int batt_step_table[15]={		
	3494,3515,3540,3575,3615,3645,3690,3705,3730,3780,3850,3930,4007,4080,4200 //origianl data
};
static int batt_no_current_step_table[15]={ 
	//3500,3608,3680,3717,3759,3803,3884,3933,3952,3976,4014,4073,4149,4300,4500 //911 original data
	3500,3715,3740,3775,3800,3830,3875,3890,3905,3950,4020,4090,4150,4300,4500
};

#endif

static int batt_disp_table[15]={
	   0,   2,   5,  10,  15,  20,  30,  40,  50,  60,  70,  80,  90, 100, 100
};
static int batt_disp_table_no_current[15]={
	   0,   2,   5,  10,  15,  20,  30,  40,  50,  60,  70,  80,  90, 100, 100 
};

/*
//battery: AE45100100P6HAS: 4700mAH
3500,3520,3545,3585,3620,3665,3730,3745,3760,3805,3875,3945,4040,4110,4500  //origianl data
3500,3565,3594,3645,3690,3735,3827,3855,3875,3917,3985,4065,4190,4300,6000 //origianl data
*/
#endif


static u16 g_adcbat = 0;	
static int full_flag = 0;
static int full_time_cnt = 0;

struct rk29_battery_data {
	spinlock_t lock;
	struct adc_client *client;
	struct power_supply 	battery;
	struct power_supply	usb;
	struct power_supply	ac;
	struct timer_list adc_timer;	
};


#define APP_BATT_PDEV_NAME		"rk29_battery"

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;


static int get_ac_charge_status(void);

static int rockchip_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);
static int rockchip_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val);
static int rockchip_ac_get_property(struct power_supply *psy, 
					enum power_supply_property psp,
					union power_supply_propval *val);

static enum power_supply_property rockchip_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property rockchip_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *supply_list[] = {
	"battery",
};


static struct power_supply rockchip_power_supplies[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = rockchip_battery_properties,
		.num_properties = ARRAY_SIZE(rockchip_battery_properties),
		.get_property = rockchip_battery_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = rockchip_power_properties,
		.num_properties = ARRAY_SIZE(rockchip_power_properties),
		.get_property = rockchip_usb_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = rockchip_power_properties,
		.num_properties = ARRAY_SIZE(rockchip_power_properties),
		.get_property = rockchip_ac_get_property,
	},
};

static int rockchip_usb_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		#if 0 
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
			val->intval = (charger ==  CHARGER_AC ? 1 : 0);
		else if (psy->type == POWER_SUPPLY_TYPE_USB)
//			val->intval = (charger ==  CHARGER_USB ? 1 : 0);
		else
	            val->intval = 0;
		 #endif 
		  val->intval = 1;
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}

static int rockchip_ac_get_property(struct power_supply *psy, 
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE: 
		val->intval = get_ac_charge_status();
		break;
	default:
		return -EINVAL;
	}
	
	return 0;
}


static int get_ac_charge_status(void)
{
	int ac_level;
#if DC_DET_EFFECTIVE	
	ac_level = gpio_get_value(dc_det_gpio);
#else
	ac_level = !gpio_get_value(dc_det_gpio);
#endif
	//printk("=====ac_level = %d\n", ac_level);

 	if (ac_level) {
		  return 1;
 	} else {
		full_flag = 0;
	 	return 0;
	}
}

static int insert_num = 0;
static int first_sampling = 1;  //the first sampling when booting, but bat_first_sample is the first cycle sample when booting
static int battery_sample_func(int cur_bat_capacity, int first)
{
	int i, average = 0, sum = 0;
	int min_capacity, max_capacity;
	int array_len = sizeof(bat_capacity_array) >> 2;
	if(first_sampling)
	{
		for(i = 0; i < array_len; i++ )
		{
			bat_capacity_array[i] = cur_bat_capacity;
		}
	}
	else
		bat_capacity_array[insert_num] = cur_bat_capacity;
	first_sampling = 0;
	/*
	if(first)
	{
		for(i = 0; i < array_len; i++ )
		{
			sum += bat_capacity_array[i];
		}
		average = sum/array_len;
	}
	else
	*/
	{
		min_capacity = bat_capacity_array[0];
		max_capacity = bat_capacity_array[0];
		for(i = 0; i < array_len; i++ )
		{
			sum += bat_capacity_array[i];
			if(bat_capacity_array[i] < min_capacity)
				min_capacity = bat_capacity_array[i];
			if(bat_capacity_array[i] > max_capacity)
				max_capacity = bat_capacity_array[i];
		}
		average = (sum-max_capacity-min_capacity)/(array_len-2);
	}
	DBG("====>average battery voltage: %d, array_len = %d\n", average, array_len);
	insert_num = (insert_num + 1) % array_len;
	return average;
}

static int battery_full_cnt = 0;
static int last_average_vol = 0;
static int full_cnt_after_wakeup = 0;
static int rockchip_get_battery_status(void)
{
	int  current_vol,i;
	int average_vol;

	//printk("g_adcbat = %d\n", g_adcbat);
	
	if (g_adcbat < AD_NO_BATT_VALE)	/*haven't battery*/ 
	{
		bat_present = BATT_PRESENT_FALSE;	
		goto nobattery;
	}
	bat_present = BATT_PRESENT_TRUE;	/*have battery*/

	/*get charge status*/
	if ( (get_ac_charge_status()) && (!full_flag) ) {/* charging */
#if defined (USE_CHARGE_LED)
		GPIOSetPinLevel(RED_LED_IOPIN, GPIO_LOW); /* red led on */
		GPIOSetPinLevel(GREEN_LED_IOPIN, GPIO_HIGH); /* green led off */
#endif
		bat_status = POWER_SUPPLY_STATUS_CHARGING;
	} else if ( (get_ac_charge_status()) && (full_flag) ) {/* charge full */
#if defined (USE_CHARGE_LED)
		GPIOSetPinLevel(RED_LED_IOPIN, GPIO_HIGH); /* red led off */
		GPIOSetPinLevel(GREEN_LED_IOPIN, GPIO_LOW); /* green led on */
#endif
		bat_status = POWER_SUPPLY_STATUS_FULL;
	} else {/* not charge */
#if defined (USE_CHARGE_LED)
		GPIOSetPinLevel(RED_LED_IOPIN, GPIO_HIGH); /* red led off */
		GPIOSetPinLevel(GREEN_LED_IOPIN, GPIO_HIGH); /* green led off */
#endif
		bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	/*Continuall sampling when first sampling or when wakeup from sleep mode so to report the current battery status*/
	if((!bat_first_sample) && (!power_state_change))
	{
		ad_sample_current_time++;
		if(ad_sample_current_time < AD_SAMPLE_TIMES) 
			return 1;
		ad_sample_current_time = 0;	
	}

	if(power_state_change)
		num_after_power_state_change++;
	/*get present voltage*/
#if defined(CONFIG_MACH_A7HC) || defined(CONFIG_MACH_M700HR)
	current_vol = (g_adcbat * 5000)/1024;		/*current voltage*/
#else
	current_vol = (g_adcbat * 9362)/1024;		/*current voltage*/
#endif

#if defined (ADC_ADD_VALUE)	
	current_vol += ADC_CLI_VALUE; 
#else
	current_vol -= ADC_CLI_VALUE;		/*有些板子采样会偏高，减去大约50mv*/
#endif

	DBG("g_adcbat = %d\n", g_adcbat);
	/*when wakeup from sleep mode, will report the current vol after 1s but not average*/
	if(power_state_change)
	{
		if(last_average_vol == 0)
			last_average_vol = current_vol;
		average_vol = (current_vol + last_average_vol)/2;
		last_average_vol = average_vol;
	}
	else
	{
		average_vol = battery_sample_func(current_vol, bat_first_sample);
		last_average_vol = 0;
	}
	bat_voltage = average_vol;
 
	/*get battery health status*/
	if (batt_step_table[0] >= average_vol)
	{
		if (!gpio_get_value(chg_ok_gpio)){
			bat_health = POWER_SUPPLY_HEALTH_GOOD;	/*current voltage too poor*/
			bat_capacity =  1;
			bat_vol_no_power_cnt = 0;	
		} else {
			bat_vol_no_power_cnt++;
			if (bat_vol_no_power_cnt < 10){
			    bat_capacity = 1;
			    return 1;
			}
			bat_vol_no_power_cnt = 0;
			bat_health = POWER_SUPPLY_HEALTH_GOOD;	/*current voltage too poor*/
			bat_capacity = 0; 
			printk("battery is too poor>>power down!!!");
		}
		return 1;
	} else if ((CHARGE_FULL_GATE <= average_vol) || ((gpio_get_value(chg_ok_gpio))&&(get_ac_charge_status()))) {
		DBG("===chg_ok_gpio is: %d\n", gpio_get_value(chg_ok_gpio));
		if (gpio_get_value(chg_ok_gpio))/* current voltage full */
		{ 
			//printk("===chg_ok_gpio is: %d\n", gpio_get_value(chg_ok_gpio));
			bat_health = POWER_SUPPLY_HEALTH_GOOD;
			bat_vol_no_power_cnt = 0;
			if(full_time_cnt > 5)
			{
				bat_capacity =  BATT_LEVEL_FULL;
				full_flag = 1;
				full_time_cnt = 6;
			}
			full_time_cnt++;
		} 
		else 
		{
			bat_health = POWER_SUPPLY_HEALTH_GOOD;
			bat_vol_no_power_cnt = 0;
			if(CHARGE_FULL_GATE <= average_vol)
			{
				bat_capacity =  90;
			}
			full_flag = 0;
			full_time_cnt = 0;
		}
		return 1;
	}
	bat_vol_no_power_cnt = 0;
	/*When long time charging, the detected battery voltage maybe lower than CHARGE_FULL_GATE in some board as the charging current 
	  come lower. But at this time, the battery capacity should be set as FULL*/
	if(bat_status == POWER_SUPPLY_STATUS_FULL)
	{
		bat_capacity =  BATT_LEVEL_FULL;
		return 1;
	}
	if(power_state_change)
	{
		if((average_vol > batt_no_current_step_table[12] - 100) && ((gpio_get_value(chg_ok_gpio))&&(get_ac_charge_status())))
		{
			full_cnt_after_wakeup++;
			if(full_cnt_after_wakeup > 5)
			{
				full_cnt_after_wakeup = 6;
				full_flag = 1;
				bat_capacity =  BATT_LEVEL_FULL;
				printk("Battery FULL after long charging\n");
			}
			return 1;
		}
	}
	full_cnt_after_wakeup = 0;
	full_time_cnt = 0;
	
	if (get_ac_charge_status()) {
		for (i=0; i<56; i++) {		
			if((batt_no_current_step_table[i]<=average_vol)&&(batt_no_current_step_table[i+1]>average_vol))
				break;		
		}
		bat_capacity = batt_disp_table_no_current[i];
	} else {
	    for(i=0; i<56; i++){		
		    if ((batt_step_table[i] <= average_vol) &&(batt_step_table[i+1] > average_vol))
				break;		
	    }
		bat_capacity = batt_disp_table[i];
	}
	bat_health = POWER_SUPPLY_HEALTH_GOOD;

	DBG("+++++>battery status  = %d\n", bat_status);
	DBG("+++++>chgok status = %d\n", gpio_get_value(chg_ok_gpio));
	DBG("+++++>battery current voltage = %d, average_vol = %d\n", current_vol, average_vol);
	DBG("+++++>battery capacity = %d\n", bat_capacity);
	
	return 1;

nobattery:
	if(get_ac_charge_status())	/*the battery charge*/
		bat_status = POWER_SUPPLY_STATUS_CHARGING ;
	else 
		bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING ;	/*no charge*/

	bat_health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int rockchip_battery_get_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	//DBG("--------%s-->%s-->property_psp%d\n",__FILE__,__FUNCTION__,psp);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = bat_present;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat_status;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = bat_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		/* get power supply */
		val->intval = bat_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		/* Todo return battery level */	
		val->intval = bat_capacity;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val ->intval = bat_voltage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = BATT_VOLTAGE_MAX;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		val->intval = BATT_VOLTAGE_MIN;
		break;
	default:		
		return -EINVAL;
	}
	
	return 0;
}


static void adc_timer_func(unsigned long _data)
{
	struct rk29_battery_data *data = (struct rk29_battery_data *)_data;
	old_bat_status = bat_status;
	old_bat_capacity = bat_capacity;
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	adc_async_read(data->client);
	rockchip_get_battery_status();
	//DBG("\n----> power_state_change==%d\n", power_state_change);
	if((times_after_first_report >=0)&&(!bat_first_sample))
		times_after_first_report--;
	
	/*Make sure the battery capacity stable when plug/unplu AC charger*/
	if(old_bat_status != bat_status)
	{
		battery_status_change = 1;
		sample_num_after_status_change = 0;
		ad_sample_current_time = 0;
	}
	if(bat_first_sample == 1)
	{
		if(sample_num_after_status_change > 4*SEC_NUM * PER_SEC_NUM - 1)
			battery_status_change = 0;
	}
	else
	{
		if(sample_num_after_status_change > 16*SEC_NUM * PER_SEC_NUM - 1)
			battery_status_change = 0;
	}	

	/* To avoid the battery of charging lower than before */
	if ((get_ac_charge_status()) && (bat_capacity < old_bat_capacity)) {	
		if ((old_bat_capacity - bat_capacity)<=20){
		    bat_capacity = old_bat_capacity;
		    bat_vol_up_cnt = 0;
		}else{
		    bat_vol_up_cnt++;
			if(bat_vol_up_cnt > 80)
			    bat_vol_up_cnt = 0;
			else	
			    bat_capacity = old_bat_capacity;
	    }
	}	
	else
		bat_vol_up_cnt = 0;

	/* To avoid the battery of not charging higher than before */
	if ((!get_ac_charge_status()) && (bat_capacity > old_bat_capacity) &&
			 old_bat_capacity != 0) {		
		if ((bat_capacity - old_bat_capacity) <= 20) {
			bat_capacity = old_bat_capacity;
			bat_vol_cnt = 0;
		} else {
			bat_vol_cnt++;
			if(bat_vol_cnt > 40)
			    bat_vol_cnt = 0;
			else	
			    bat_capacity = old_bat_capacity;
		}	
	}
	else
		bat_vol_cnt = 0;
	/*when wakeup from sleep, battery will first report after 1s whatever charger is insterted in or not*/
	if(!power_state_change)
	{
		if(battery_status_change)
		{
			sample_times = 0;
			/*To make sure battery dislay not more than 99% when plug in AC charger*/
			if(get_ac_charge_status()&&(old_report_bat_capacity > 99))
			{
				bat_capacity = 99;
			}
			else
			{
				/*battery capacity must report immediately when battery status changes from CHARGING to FULL, otherwise, 
				  old battery status have to be remain for 1 minutes when battery status changes*/
				if(!full_flag) 
					bat_capacity = old_report_bat_capacity;
			}
			sample_num_after_status_change++;
		}
			
		if ((bat_present == BATT_PRESENT_TRUE) && (old_bat_status != bat_status)) {
			/*set charge status*/
			DBG("\n ====>old_bat_status==%i bat_status==%i\n",
					old_bat_status, bat_status);
			//DBG("====>battery adcbat = %d\n", g_adcbat);
			//DBG("====>battery present = %d\n", bat_present);
			//DBG("====>battery status  = %d\n", bat_status);
			DBG("====>chgok status = %d, full_flag = %d\n", gpio_get_value(chg_ok_gpio), full_flag);
			DBG("====>battery current voltage = %d\n", bat_voltage);
			DBG("====>old_bat_capacity = %d, battery capacity = %d\n", old_bat_capacity, bat_capacity);
			old_report_bat_capacity = bat_capacity;
			power_supply_changed(&data->battery);
			if(bat_first_sample)
				bat_first_sample = 0;
			goto next;
		}
		
		/*will not report battery capacity 6s after battery status changes*/
		if(battery_status_change)
			goto next;
	}
	
	/*To make sure report battery status immediately when system wakeup from sleep*/
	if(!power_state_change)
	{
		num_after_power_state_change = 0;
	}
	else
	{
		ad_sample_current_time = 0;
		sample_times = 0;
	}
	if(num_after_power_state_change > 10)
	{
		power_state_change = 0;
		sample_times = SEC_NUM * PER_SEC_NUM;
		num_after_power_state_change = 0;
		if(!get_ac_charge_status())
			bat_capacity = old_report_bat_capacity;
	}
	sample_times++;
	if ((bat_present == BATT_PRESENT_TRUE) && 
			(sample_times > SEC_NUM * PER_SEC_NUM)) {
		sample_times = 0;
		DBG("\n----> old_bat_status==%i  bat_status==%i\n",
				old_bat_status, bat_status);
		//DBG("---->battery adcbat = %d\n", g_adcbat);
		//DBG("---->battery present = %d\n", bat_present);
		//DBG("---->battery status  = %d\n", bat_status);
		DBG("---->chgok status = %d, full_flag = %d\n", gpio_get_value(chg_ok_gpio), full_flag);
		DBG("---->battery current voltage = %d\n", bat_voltage);
		DBG("---->old_bat_capacity = %d, battery capacity = %d\n\n", old_bat_capacity, bat_capacity);
		if(times_after_first_report >= 0)     //prevent battery capacity drop sharply when booting
			 bat_capacity = old_report_bat_capacity;
		else
			old_report_bat_capacity = bat_capacity;
		power_supply_changed(&data->battery);
		if(bat_first_sample)
			bat_first_sample = 0;
	}

next:
	spin_unlock_irqrestore(&data->lock, flags);
	mod_timer(&data->adc_timer, jiffies + msecs_to_jiffies(100));

}

	
static void callback(struct adc_client *client, void *client_param, int result)
{
	struct rk29_battery_data *ddata = (struct rk29_battery_data *)client_param;

	//if(result > INVALID_ADVALUE && result < EMPTY_ADVALUE)
	g_adcbat = result;
	return;
}

static int rockchip_battery_probe(struct platform_device *pdev)
{
	int  rc,i;
	struct rk29_battery_data  *data;
	struct adc_battery_platform_data *pdata = pdev->dev.platform_data;
	int dect_time = 0;
	int array_len;

	printk("******battery probe start!!\n");
	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data ) {
		rc = -ENOMEM;
		goto fail0;
	}

	dc_det_gpio = pdata->dc_det_gpio;
	rc = gpio_request(pdata->dc_det_gpio, "DC_DET");
	if (rc < 0) {
		pr_err("rk29_battery: failed to request GPIO %d,"
				" error %d\n", pdata->dc_det_gpio, rc);
		//goto fail0;
	}

	rc = gpio_direction_input(pdata->dc_det_gpio);

	if (rc < 0) {
		pr_err("rk29_battery: failed to configure input"
				" direction for GPIO %d, error %d\n",
				pdata->dc_det_gpio, rc);
		gpio_free(pdata->dc_det_gpio);
		goto fail0;
	}

#if DC_DET_EFFECTIVE	
    gpio_pull_updown(pdata->dc_det_gpio, PullDisable);
#else
    gpio_pull_updown(pdata->dc_det_gpio, GPIOPullUp);
#endif
	
#if defined(DC_CURRENT_IN_TWO_MODE)
	rc = gpio_request(GPIO_CURRENT_CONTROL, "DC_CURRENT_CONTROL");
	if (rc < 0) {
		pr_err("rk29_battery: failed to request GPIO %d,"
				" error %d\n", GPIO_CURRENT_CONTROL, rc);
		goto fail0;
	}

	rc = gpio_direction_output(GPIO_CURRENT_CONTROL, 1);
	if (rc < 0) {
		pr_err("rk29_battery: failed to configure input"
				" direction for GPIO %d, error %d\n",
				GPIO_CURRENT_CONTROL, rc);
		gpio_free(GPIO_CURRENT_CONTROL);
		goto fail0;
	}
    gpio_pull_updown(GPIO_CURRENT_CONTROL, 1);
	gpio_set_value(GPIO_CURRENT_CONTROL, 1);
#endif
	printk("======================GPIO_USB_INT: %d===========\n", gpio_get_value(GPIO_USB_INT));
	
	chg_ok_gpio = pdata->chg_ok_gpio;
	rc = gpio_request(pdata->chg_ok_gpio, "CHG_OK");
	if (rc < 0) {
		pr_err("rk29_battery: failed to request GPIO %d,"
				" error %d\n", pdata->chg_ok_gpio, rc);
		goto fail0;
	}
	gpio_pull_updown(pdata->chg_ok_gpio, PullDisable);
	rc = gpio_direction_input(pdata->chg_ok_gpio);
	if (rc < 0) {
		pr_err("rk29_battery: failed to configure input"
				" direction for GPIO %d, error %d\n",
				pdata->chg_ok_gpio, rc);
		gpio_free(pdata->chg_ok_gpio);
		goto fail0;
	}

	platform_set_drvdata(pdev, data);
	data->client = adc_register(pdata->adc_chn, callback, (void *)data);
	if(!data->client) {
		rc = -EINVAL;
		goto fail1;
	}

	spin_lock_init(&data->lock);

	data->battery = rockchip_power_supplies[0];
	data->usb = rockchip_power_supplies[1];
	data->ac = rockchip_power_supplies[2];

	rc = power_supply_register(&pdev->dev, &data ->battery);
	if (rc)
	{
		printk(KERN_ERR "Failed to register battery power supply (%d)\n", rc);
		goto err_battery_fail;
	}
	
	rc = power_supply_register(&pdev->dev, &data ->ac);
	if (rc)
	{
		printk(KERN_ERR "Failed to register ac power supply (%d)\n", rc);
		goto err_ac_fail;
	}
	setup_timer(&data->adc_timer,
			adc_timer_func, (unsigned long)data);
	mod_timer(&data->adc_timer, jiffies + msecs_to_jiffies(100));
	array_len = sizeof(bat_capacity_array)/sizeof(int);
	/*Get the default battery statuc when booting*/
	for(i = 0; i < 100; i++)
	{
		//ad_sample_current_time++;
		if((g_adcbat > 0)/*&&(ad_sample_current_time > (AD_SAMPLE_TIMES-1)) */)
		{
			//printk("=============================\n");
			if(insert_num < array_len)
			{
				rockchip_get_battery_status();
				dect_time++;
			}
			/*read battery status for 2*array_len times to make sure get stable battery cacipty when booting*/
			if(dect_time > array_len + array_len)
				break;
		}
		mdelay(10);	
	}
	rockchip_get_battery_status();
	old_bat_status = bat_status;
	old_bat_capacity = bat_capacity;
	old_report_bat_capacity = bat_capacity;
	DBG("====old_bat_status= %d, old_bat_capacity = %d, old_report_bat_capacity =%d\n", old_bat_status, old_bat_capacity, old_report_bat_capacity);
	//bat_first_sample = 0;
	DBG("====dect_time = %d, array_len= %d\n", dect_time, array_len);
	return 0;

err_battery_fail:
	power_supply_unregister(&data->battery);

err_ac_fail:
	power_supply_unregister(&data->ac);
fail1:
 	platform_set_drvdata(pdev, NULL);
fail0:
	kfree(data);
	return rc;
}


#ifdef CONFIG_PM
static int rockchip_battery_suspend(struct platform_device *pdev, 
		pm_message_t state)
{
	printk("battery suspend!!!!!!!!!!!!!!!!!!!!!!!!\n");
	power_state_change = 1;
#if defined(DC_CURRENT_IN_TWO_MODE)
	gpio_set_value(GPIO_CURRENT_CONTROL, 0);
#endif
	return 0;
}

static int rockchip_battery_resume(struct platform_device *pdev)
{
	printk("battery resume!!!!!!!!!!!!!!!!!!!!!!!!\n");
	//sample_times = (SEC_NUM * PER_SEC_NUM - 5);
	DBG("power_state_change = %d\n", power_state_change);
	struct rk29_battery_data  *data = platform_get_drvdata(pdev);
	mod_timer(&data->adc_timer, jiffies);
#if defined(DC_CURRENT_IN_TWO_MODE)
	gpio_set_value(GPIO_CURRENT_CONTROL, 1);
#endif
	return 0;
}
#else
#define rockchip_battery_suspend NULL
#define rockchip_battery_resume NULL
#endif /* CONFIG_PM */

static struct platform_driver rockchip_battery_driver = {
	.probe	= rockchip_battery_probe,
	.suspend = rockchip_battery_suspend,
	.resume = rockchip_battery_resume,
	.driver	= {
		.name	= APP_BATT_PDEV_NAME,
		.owner	= THIS_MODULE,
	},
};


static int __init rockchip_battery_init(void)
{
	return platform_driver_register(&rockchip_battery_driver);
}

fs_initcall(rockchip_battery_init);
//module_init(rockchip_battery_init);
MODULE_DESCRIPTION("Rockchip Battery Driver");
MODULE_LICENSE("GPL");
