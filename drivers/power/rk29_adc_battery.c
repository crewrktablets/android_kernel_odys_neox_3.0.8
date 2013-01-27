/* drivers/power/rk29_adc_battery.c
 *
 * battery detect driver for the rk2918 
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <mach/gpio.h>
#include <linux/adc.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/slab.h>
#include <linux/syscalls.h>

#include <linux/wakelock.h>

static struct wake_lock batt_wake_lock;

#if 0
#define DBG(x...)   printk(x)
#else
#define DBG(x...)
#endif

int rk29_battery_dbg_level = 0;
module_param_named(dbg_level, rk29_battery_dbg_level, int, 0644);

#define ARNOVA_M16C
//#define ARNOVA_M19

/*******************ÒÔÏÂ²ÎÊý¿ÉÒÔÐÞ¸Ä******************************/
#define	TIMER_MS_COUNTS		            50	//¶¨Ê±Æ÷µÄ³¤¶Èms
//ÒÔÏÂ²ÎÊýÐèÒª¸ù¾ÝÊµ¼Ê²âÊÔµ÷Õû
#define	SLOPE_SECOND_COUNTS	            15	//Í³¼ÆµçÑ¹Ð±ÂÊµÄÊ±¼ä¼ä¸ôs
#define	DISCHARGE_MIN_SECOND	            10			//×î¿ì·Åµçµç1%Ê±¼ä
#define	CHARGE_MIN_SECOND	            45	//×î¿ì³äµçµç1%Ê±¼ä
#define	CHARGE_MID_SECOND	            90	//ÆÕÍ¨³äµçµç1%Ê±¼ä
#define	CHARGE_MAX_SECOND	            250	//×î³¤³äµçµç1%Ê±¼ä
#define CHARGE_FULL_DELAY_TIMES         10  //³äµçÂú¼ì²â·À¶¶Ê±¼ä
#define USBCHARGE_IDENTIFY_TIMES        5   //²åÈëUSB»ìÁ÷£¬pcÊ¶±ð¼ì²âÊ±¼ä

#define	NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//´æ´¢µÄ²ÉÑùµã¸öÊý
#define	NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	//´æ´¢µÄ²ÉÑùµã¸öÊý
#define	NUM_CHARGE_MIN_SAMPLE	        ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    //´æ´¢µÄ²ÉÑùµã¸öÊý
#define	NUM_CHARGE_MID_SAMPLE	        ((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	    //´æ´¢µÄ²ÉÑùµã¸öÊý
#define	NUM_CHARGE_MAX_SAMPLE	        ((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	    //´æ´¢µÄ²ÉÑùµã¸öÊý
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	//³äµçÂú×´Ì¬³ÖÐøÊ±¼ä³¤¶È
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	//³äµçÂú×´Ì¬³ÖÐøÊ±¼ä³¤¶È

#define BAT_2V5_VALUE	                2500

#ifdef ARNOVA_M16C
	#define BAT_VOL_RATIO								(400/100)
#endif

#ifdef ARNOVA_M19
#define  BAT_VOL_RATIO                  6
#endif

#define BATT_MAX_VOL_VALUE              8303//4180	//ÂúµçÊ±µÄµç³ØµçÑ¹	 FOR A7
//#define BATT_ZERO_VOL_VALUE             6800      //¿¿¿¿6.8v
  #define BATT_ZERO_VOL_VALUE             6600      //¿¿¿¿   v 
//#define BATT_NOMAL_VOL_VALUE            3800

//¶¨ÒåADC²ÉÑù·ÖÑ¹µç×è£¬ÒÔÊµ¼ÊÖµÎª×¼£¬µ¥Î»K
#define BAT_PULL_UP_R                   200
#define BAT_PULL_DOWN_R                 200
#define BAT_ADC_TABLE_LEN               11
#define adc_to_voltage(adc_val) ((adc_val * BAT_2V5_VALUE * BAT_VOL_RATIO) / 1024)

static int adc_raw_table_bat[BAT_ADC_TABLE_LEN] = 
{
//         0        10       20        30         40       50         60         70         80        90       	100
//  6.8v ¿¿¿
//     6800,    6889,   6963,    	7046,      7134,     7214,      7323,      7482,      7681,      7892,     8303
      6618,    6825,     6946,     7024,      7077,      7164,     7307,     7475,     7648,     7864,     8313     

};

static int adc_raw_table_ac[BAT_ADC_TABLE_LEN] = 
{
// 6.8v ¿¿¿
//	7050,7136,7310,7393,7481,7561,7670,7829,8028,8239,8650
//6.6v ¿¿¿
      6868,    7075,   7196,      7274,       7327,     7414,      7557,    7725,      7898,    8114,      8563
};

extern int dwc_vbus_status(void);
extern int get_msc_connect_flag(void);

struct rk29_adc_battery_data {
	int irq;
	
	struct timer_list       timer;
	struct work_struct 	    timer_work;
	struct work_struct 	    dcwakeup_work;
	struct work_struct 	    resume_work;
	
	struct rk29_adc_battery_platform_data *pdata;

	int                     full_times;
	
	struct adc_client       *client; 
	int                     adc_val;
	int                     adc_samples[NUM_VOLTAGE_SAMPLE+2];
	
	int                     bat_status;
	int                     bat_status_cnt;
	int                     bat_health;
	int                     bat_present;
	int                     bat_voltage;
	int                     bat_capacity;
	int                     bat_change;
};
static struct rk29_adc_battery_data *gBatteryData;

enum {
	BATTERY_STATUS          = 0,
	BATTERY_HEALTH          = 1,
	BATTERY_PRESENT         = 2,
	BATTERY_CAPACITY        = 3,
	BATTERY_AC_ONLINE       = 4,
	BATTERY_STATUS_CHANGED	= 5,
	AC_STATUS_CHANGED   	= 6,
	BATTERY_INT_STATUS	    = 7,
	BATTERY_INT_ENABLE	    = 8,
};

typedef enum {
	CHARGER_BATTERY = 0,
	CHARGER_USB,
	CHARGER_AC
} charger_type_t;


#define BATT_FILENAME "/data/bat_last_capacity.dat"
#include <linux/fs.h>

static void rk29_adc_battery_capacity_samples(struct rk29_adc_battery_data *bat);
static int rk29_adc_battery_voltage_to_capacity(struct rk29_adc_battery_data *bat, int BatVoltage);
static struct power_supply rk29_battery_supply;

static int rk29_adc_battery_load_capacity(void)
{
    char value[4];
	int* p = (int *)value;
    long fd = sys_open(BATT_FILENAME,O_RDONLY,0);
    
	if(fd < 0)
    {
		printk("rk29_adc_battery_load_capacity: open file /data/bat_last_capacity.dat failed\n");
		return -1;
	}
	
	sys_read(fd,(char __user *)value,4);
	
    sys_close(fd);
    
	return (*p);
}

static void rk29_adc_battery_put_capacity(int loadcapacity)
{
    char value[4];
	int* p = (int *)value;
    long fd = sys_open(BATT_FILENAME,O_CREAT | O_RDWR,0);
    
	if(fd < 0)
    {
		printk("rk29_adc_battery_put_capacity: open file /data/bat_last_capacity.dat failed\n");
		return;
	}
    *p = loadcapacity;
	sys_write(fd, (const char __user *)value, 4);
	
    sys_close(fd);
}

static void rk29_adc_battery_charge_enable(struct rk29_adc_battery_data *bat)
{
    struct rk29_adc_battery_platform_data *pdata = bat->pdata;
    
    if (pdata->charge_set_pin != INVALID_GPIO)
    {
        gpio_direction_output(pdata->charge_set_pin, pdata->charge_set_level);
    }
}

static void rk29_adc_battery_charge_disable(struct rk29_adc_battery_data *bat)
{
    struct rk29_adc_battery_platform_data *pdata = bat->pdata;
    
    if (pdata->charge_set_pin != INVALID_GPIO)
    {
        gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
    }
}

extern int suspend_flag;
static int rk29_adc_battery_get_charge_level(struct rk29_adc_battery_data *bat)
{
    int charge_on = 0;
    struct rk29_adc_battery_platform_data *pdata = bat->pdata;
    
#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
    if (pdata->dc_det_pin != INVALID_GPIO)
    {
        if (gpio_get_value (pdata->dc_det_pin) == pdata->dc_det_level)
        {
            charge_on = 1;
        }
    }
#endif
    
#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
    if (charge_on == 0)
    {
        if (suspend_flag) return;
            
        if (1 == dwc_vbus_status())         //¼ì²âµ½USB²åÈë£¬µ«ÊÇÎÞ·¨Ê¶±ðÊÇ·ñÊÇ³äµçÆ÷
        {                                   //Í¨¹ýÑÓÊ±¼ì²âPCÊ¶±ð±êÖ¾£¬Èç¹û³¬Ê±¼ì²â²»µ½£¬ËµÃ÷ÊÇ³äµç
            if (0 == get_msc_connect_flag())
            {                               //²åÈë³äµçÆ÷Ê±¼ä´óÓÚÒ»¶¨Ê±¼äÖ®ºó£¬¿ªÊ¼½øÈë³äµç×´Ì¬
                if (++gBatUsbChargeCnt >= NUM_USBCHARGE_IDENTIFY_TIMES)
                {
                    gBatUsbChargeCnt = NUM_USBCHARGE_IDENTIFY_TIMES + 1;
                    charge_on = 1;
                }
            }                               //·ñÔò£¬²»½øÈë³äµçÄ£Ê½
        }                   
        else
        {
            gBatUsbChargeCnt = 0;
            if (2 == dwc_vbus_status()) 
            {
                charge_on = 1;
            }
        }
    }
#endif

    return charge_on;
}

int old_charge_level;
static int rk29_adc_battery_status_samples(struct rk29_adc_battery_data *bat)
{
    int charge_level;
    struct rk29_adc_battery_platform_data *pdata = bat->pdata;
    
    charge_level = rk29_adc_battery_get_charge_level(bat);
    
    //¼ì²â³äµç×´Ì¬±ä»¯Çé¿ö
    if (charge_level != old_charge_level)
    {
        old_charge_level = charge_level;
        bat->bat_change  = 1;
        if(charge_level) 
        {            
            rk29_adc_battery_charge_enable(bat);
        }
        else
        {
            rk29_adc_battery_charge_disable(bat);
        }
        bat->bat_status_cnt = 0;        //×´Ì¬±ä»¯¿ªÊ¼¼ÆÊý
    }
    
    //»ñÈ¡ÎÈ¶¨µÄ³äµç×´Ì¬
	if(charge_level == 0)
	{   
	    //Î´³äµç
	    bat->full_times = 0;
        bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	}
	else
	{
	    //³äµç	    
        if (pdata->charge_ok_pin == INVALID_GPIO)
        {
            //Ã»ÓÐcharge_ok_pin£¬¼ì²âÈÝÁ¿
            if (bat->bat_capacity == 100)
            {
                if (bat->bat_status != POWER_SUPPLY_STATUS_FULL)
                {
                    bat->bat_status = POWER_SUPPLY_STATUS_FULL;
                    bat->bat_change  = 1;
                }
            }
            else
            {
                bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
            }
        }
        else
        {
            //ÓÐ³äµç¼ì²â½Ì
            if (gpio_get_value(pdata->charge_ok_pin) != pdata->charge_ok_level)
            {
                //Ã»ÓÐ¼ì²âµ½³äµçÂúµçÆ½±êÖ¾
                bat->full_times = 0;
                bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
            }
            else
            {
                //¼ì²âµ½³äµçÂúµçÆ½±êÖ¾
                bat->full_times++;
                if (bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES) 
                {
                    bat->full_times = NUM_CHARGE_FULL_DELAY_TIMES + 1;
                }

                if ((bat->full_times >= NUM_CHARGE_FULL_DELAY_TIMES) && (bat->bat_capacity >= 99))
    		    {
    		        if (bat->bat_status != POWER_SUPPLY_STATUS_FULL)
                    {
                        bat->bat_status = POWER_SUPPLY_STATUS_FULL;
                        bat->bat_capacity = 100;
                        bat->bat_change  = 1;
                    }
    		    }
    		    else
    		    {
    		        bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
    		    }
            }
        }
    }
    
	return charge_level;
}

int AdcTestvalue = 0;
static int gFlagLoop = 0;
static int *pSamples;
static void rk29_adc_battery_voltage_samples(struct rk29_adc_battery_data *bat)
{
	int value;
	int i,*pStart = bat->adc_samples, num = 0;
	
	value = bat->adc_val;
	AdcTestvalue = value;
    adc_async_read(bat->client);
    
	*pSamples++ = adc_to_voltage(value);
	
	bat->bat_status_cnt++;
	if (bat->bat_status_cnt > NUM_VOLTAGE_SAMPLE)  bat->bat_status_cnt = NUM_VOLTAGE_SAMPLE + 1;
	
	num = pSamples - pStart;
	if (num >= NUM_VOLTAGE_SAMPLE)
	{
	    pSamples = pStart;
	    gFlagLoop = 1;
	}
	if (gFlagLoop == 1)
	{
	    num = NUM_VOLTAGE_SAMPLE;
	}
	value = 0;
	for (i = 0; i < num; i++)
	{
	    value += bat->adc_samples[i];
	}
	bat->bat_voltage = value / num;
	
	/*Ïû³ýÃ«´ÌµçÑ¹*/
	if(bat->bat_voltage >= BATT_MAX_VOL_VALUE + 10)
		bat->bat_voltage = BATT_MAX_VOL_VALUE + 10;
	else if(bat->bat_voltage <= BATT_ZERO_VOL_VALUE - 10)
		bat->bat_voltage = BATT_ZERO_VOL_VALUE - 10;
}

int capacitytmp = 0;
static int rk29_adc_battery_voltage_to_capacity(struct rk29_adc_battery_data *bat, int BatVoltage)
{
    int i = 0;
	int capacity = 0;
	int *p = adc_raw_table_bat;
    
    if (rk29_adc_battery_get_charge_level(bat))
    {
        p = adc_raw_table_ac;
    }
	
	if(BatVoltage >= p[BAT_ADC_TABLE_LEN - 1])
	{
	    //µ±µçÑ¹³¬¹ý×î´óÖµ
	    capacity = 100;
	}	
	else if(BatVoltage <= p[0])
	{
	    //µ±µçÑ¹µÍÓÚ×îÐ¡Öµ
	    capacity = 0;
	}
	else
	{
    	//¼ÆËãÈÝÁ¿
    	for(i = 0; i < BAT_ADC_TABLE_LEN - 1; i++)
        {
    		
    		if((p[i] <= BatVoltage) && (BatVoltage < p[i+1]))
    		{
    			capacity = i * 10 + ((BatVoltage - p[i]) * 10) / (p[i+1] - p[i]);
    			break;
    		}
    	}
    }  
    return capacity;
}

static int gBatCapacityDisChargeCnt = 0;
static int gBatCapacityChargeCnt    = 0;
//static int rk29_adc_battery_get_capacity_ext(int BatVoltage)
static void rk29_adc_battery_capacity_samples(struct rk29_adc_battery_data *bat)
{
	int capacity = 0;
	struct rk29_adc_battery_platform_data *pdata = bat->pdata;
	
    //³ä·Åµç×´Ì¬±ä»¯ºó£¬BufferÌîÂúÖ®Ç°£¬²»¸üÐÂ
	if (bat->bat_status_cnt < NUM_VOLTAGE_SAMPLE)  
	{
	    gBatCapacityDisChargeCnt = 0;
	    gBatCapacityChargeCnt    = 0;
	    return;
	}
	
    capacity = rk29_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
	    
    if (rk29_adc_battery_get_charge_level(bat))
    {
        if (capacity > bat->bat_capacity)
        {
            //Êµ¼Ê²ÉÑùµ½µÄµçÑ¹±ÈÏÔÊ¾µÄµçÑ¹´ó£¬Öð¼¶ÉÏÉý
            if (++gBatCapacityDisChargeCnt >= NUM_CHARGE_MIN_SAMPLE)
            {
                gBatCapacityDisChargeCnt = 0;
                if (bat->bat_capacity < 99)
                {
                    bat->bat_capacity++;
                    bat->bat_change  = 1;
                }
            }
            gBatCapacityChargeCnt = 0;
        }
        else
        {
            gBatCapacityDisChargeCnt = 0;
            gBatCapacityChargeCnt++;
            
            if (pdata->charge_ok_pin != INVALID_GPIO)
            {
                if (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level)
                {
                    //¼ì²âµ½µç³Ø³äÂú±êÖ¾£¬Í¬Ê±³¤Ê±¼äÄÚ³äµçµçÑ¹ÎÞ±ä»¯£¬¿ªÊ¼Æô¶¯¼ÆÊ±³äµç£¬¿ìËÙÉÏÉýÈÝÁ¿
                    if (gBatCapacityChargeCnt >= NUM_CHARGE_MIN_SAMPLE)
                    {
                        gBatCapacityChargeCnt = 0;
                        if (bat->bat_capacity < 99)
                        {
                            bat->bat_capacity++;
                            bat->bat_change  = 1;
                        }
                    }
                }
                else
                {
                    if (capacity > capacitytmp)
                    {
                        //¹ý³ÌÖÐÈç¹ûµçÑ¹ÓÐÔö³¤£¬¶¨Ê±Æ÷¸´Î»£¬·ÀÖ¹¶¨Ê±Æ÷Ä£Äâ³äµç±ÈÊµ¼Ê³äµç¿ì
                        gBatCapacityChargeCnt = 0;
                    }
                    if (/*(bat->bat_capacity >= 80) && */(gBatCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE))
                    {
                        gBatCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);
                        if (bat->bat_capacity < 99)
                        {
                            bat->bat_capacity++;
                            bat->bat_change  = 1;
                        }
                    }
                }
            }
            else
            {
                //Ã»ÓÐ³äµçÂú¼ì²â½Å£¬³¤Ê±¼äÄÚµçÑ¹ÎÞ±ä»¯£¬¶¨Ê±Æ÷Ä£Äâ³äµç
                if (capacity > capacitytmp)
                {
                    //¹ý³ÌÖÐÈç¹ûµçÑ¹ÓÐÔö³¤£¬¶¨Ê±Æ÷¸´Î»£¬·ÀÖ¹¶¨Ê±Æ÷Ä£Äâ³äµç±ÈÊµ¼Ê³äµç¿ì
                    gBatCapacityChargeCnt = 0;
                }
                if (gBatCapacityChargeCnt > NUM_CHARGE_MAX_SAMPLE)
                {
                    gBatCapacityChargeCnt = (NUM_CHARGE_MAX_SAMPLE - NUM_CHARGE_MID_SAMPLE);
                    if (bat->bat_capacity < 100)
                    {
                        bat->bat_capacity++;
                        bat->bat_change  = 1;
                    }
                }
            }            
        }
    }    
    else
    {   
        //·ÅµçÊ±,Ö»ÔÊÐíµçÑ¹ÏÂ½µ
        if (capacity < bat->bat_capacity)
        {
            if (++gBatCapacityDisChargeCnt >= NUM_DISCHARGE_MIN_SAMPLE)
            {
                gBatCapacityDisChargeCnt = 0;
                if (bat->bat_capacity > 0)
                {
                    bat->bat_capacity-- ;
                    bat->bat_change  = 1;
                }
            }
        }
        else
        {
            gBatCapacityDisChargeCnt = 0;
        }
        
        gBatCapacityChargeCnt = 0;
    }
	capacitytmp = capacity;
}

static int poweron_check = 0;
static void rk29_adc_battery_poweron_capacity_check(void)
{
    int new_capacity, old_capacity;
    
    new_capacity = gBatteryData->bat_capacity;
    old_capacity = rk29_adc_battery_load_capacity();
    if ((old_capacity <= 0) || (old_capacity >= 100))
    {
        old_capacity = new_capacity;
    }    
    
    if (gBatteryData->bat_status == POWER_SUPPLY_STATUS_FULL)
    {
        if (new_capacity > 80)
        {
            gBatteryData->bat_capacity = 100;
        }
    }
    else if (gBatteryData->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING)
    {
        //chargeing state
        //ÎÊÌâ£º
        //1£©³¤Ê±¼ä¹Ø»ú·ÅÖÃºó£¬¿ª»úºó¶ÁÈ¡µÄÈÝÁ¿Ô¶Ô¶´óÓÚÊµ¼ÊÈÝÁ¿ÔõÃ´°ì£¿
        //2£©Èç¹û²»ÕâÑù×ö£¬¶ÌÊ±¼ä¹Ø»úÔÙ¿ª»ú£¬Ç°ºóÈÝÁ¿²»Ò»ÖÂÓÖ¸ÃÔõÃ´°ì£¿
        //3£©Ò»ÏÂÄÇÖÖ·½Ê½ºÏÊÊ£¿
        //gBatteryData->bat_capacity = new_capacity;
        gBatteryData->bat_capacity = (new_capacity > old_capacity) ? new_capacity : old_capacity;
    }
    else
    {
        gBatteryData->bat_capacity = (new_capacity < old_capacity) ? new_capacity : old_capacity;
    }
    
    
    printk("capacity = %d, new_capacity = %d, old_capacity = %d\n",gBatteryData->bat_capacity, new_capacity, old_capacity);
    
    gBatteryData->bat_change = 1;
}

unsigned long AdcTestCnt = 0;
static void rk29_adc_battery_timer_work(struct work_struct *work)
{	
	rk29_adc_battery_status_samples(gBatteryData);
	
	if (poweron_check)
	{   
        poweron_check = 0;
        rk29_adc_battery_poweron_capacity_check();
	}
	
	rk29_adc_battery_voltage_samples(gBatteryData);
	rk29_adc_battery_capacity_samples(gBatteryData);
	
	/*update battery parameter after adc and capacity has been changed*/
	if(gBatteryData->bat_change)
	{
	    gBatteryData->bat_change = 0;
	    rk29_adc_battery_put_capacity(gBatteryData->bat_capacity);
		power_supply_changed(&rk29_battery_supply);
	}

	if (rk29_battery_dbg_level)
	{
    	if (++AdcTestCnt >= 20)
    	{
    	    AdcTestCnt = 0;
    	    printk("Status = %d, RealAdcVal = %d, RealVol = %d,gBatVol = %d, gBatCap = %d, RealCapacity = %d, dischargecnt = %d, chargecnt = %d\n", 
    	            gBatteryData->bat_status, AdcTestvalue, adc_to_voltage(AdcTestvalue), 
    	            gBatteryData->bat_voltage, gBatteryData->bat_capacity, capacitytmp, gBatCapacityDisChargeCnt, gBatCapacityChargeCnt);
    	}
    }
	


}

static void rk29_adc_battery_scan_timer(unsigned long data)
{
    gBatteryData->timer.expires  = jiffies + msecs_to_jiffies(TIMER_MS_COUNTS);
	add_timer(&gBatteryData->timer);
	
	schedule_work(&gBatteryData->timer_work);	
}

#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
static int rk29_adc_battery_get_usb_property(struct power_supply *psy, 
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	charger_type_t charger;
	charger =  CHARGER_USB;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_USB)
			val->intval = get_msc_connect_flag();
		printk("%s:%d\n",__FUNCTION__,val->intval);
		break;

	default:
		return -EINVAL;
	}
	
	return 0;

}

static enum power_supply_property rk29_adc_battery_usb_props[] = {
    
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk29_usb_supply = 
{
	.name = "usb",
	.type = POWER_SUPPLY_TYPE_USB,

	.get_property   = rk29_adc_battery_get_usb_property,

    .properties     = rk29_adc_battery_usb_props,
	.num_properties = ARRAY_SIZE(rk29_adc_battery_usb_props),
};
#endif

#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
static irqreturn_t rk29_adc_battery_dc_wakeup(int irq, void *dev_id)
{   
    schedule_work(&gBatteryData->dcwakeup_work);
    return IRQ_HANDLED;
}


static int rk29_adc_battery_get_ac_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	charger_type_t charger;
	charger =  CHARGER_USB;
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS)
		{
			printk("POWER_SUPPLY_TYPE_MAINS\n");
			if (rk29_adc_battery_get_charge_level(gBatteryData)&&(gpio_get_value(RK29_PIN4_PA3) != GPIO_HIGH))
			{
				val->intval = 1;
				}
			else
				{
				val->intval = 0;	
				}
		}
		DBG("%s:%d\n",__FUNCTION__,val->intval);
		break;
		
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property rk29_adc_battery_ac_props[] = 
{
	POWER_SUPPLY_PROP_ONLINE,
};

static struct power_supply rk29_ac_supply = 
{
	.name = "ac",
	.type = POWER_SUPPLY_TYPE_MAINS,

	.get_property   = rk29_adc_battery_get_ac_property,

    .properties     = rk29_adc_battery_ac_props,
	.num_properties = ARRAY_SIZE(rk29_adc_battery_ac_props),
};

static void rk29_adc_battery_dcdet_delaywork(struct work_struct *work)
{
    int ret;
    struct rk29_adc_battery_platform_data *pdata = gBatteryData->pdata;
    int irq      = gpio_to_irq(pdata->dc_det_pin);
    int irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
    
    rk28_send_wakeup_key();
    
    free_irq(irq, NULL);
    ret = request_irq(irq, rk29_adc_battery_dc_wakeup, irq_flag, "rk29_adc_battery", NULL);
	if (ret) {
		free_irq(irq, NULL);
	}
	
	power_supply_changed(&rk29_ac_supply);

    gBatteryData->bat_status_cnt = 0;        //×´Ì¬±ä»¯¿ªÊ¼¼ÆÊý

		wake_lock_timeout(&batt_wake_lock, 30 * HZ);

}


#endif

static int rk29_adc_battery_get_status(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_status);
}

static int rk29_adc_battery_get_health(struct rk29_adc_battery_data *bat)
{
	return POWER_SUPPLY_HEALTH_GOOD;
}

static int rk29_adc_battery_get_present(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_voltage < BATT_MAX_VOL_VALUE) ? 0 : 1;
}

static int rk29_adc_battery_get_voltage(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_voltage );
}

static int rk29_adc_battery_get_capacity(struct rk29_adc_battery_data *bat)
{
	return (bat->bat_capacity);
}

static int rk29_adc_battery_get_property(struct power_supply *psy,
				 enum power_supply_property psp,
				 union power_supply_propval *val)
{		
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = rk29_adc_battery_get_status(gBatteryData);
		DBG("gBatStatus=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = rk29_adc_battery_get_health(gBatteryData);
		DBG("gBatHealth=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = rk29_adc_battery_get_present(gBatteryData);
		DBG("gBatPresent=%d\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val ->intval = rk29_adc_battery_get_voltage(gBatteryData);
		DBG("gBatVoltage=%d\n",val->intval);
		break;
//	case POWER_SUPPLY_PROP_CURRENT_NOW:
//		val->intval = 1100;
//		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = rk29_adc_battery_get_capacity(gBatteryData);
		DBG("gBatCapacity=%d%%\n",val->intval);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;	
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = BATT_MAX_VOL_VALUE;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = BATT_ZERO_VOL_VALUE;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static enum power_supply_property rk29_adc_battery_props[] = {

	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
//	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
};

static struct power_supply rk29_battery_supply = 
{
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,

	.get_property   = rk29_adc_battery_get_property,
	
    .properties     = rk29_adc_battery_props,
	.num_properties = ARRAY_SIZE(rk29_adc_battery_props),
};


#ifdef CONFIG_PM
int suspend_capacity = 0;
static void rk29_adc_battery_resume_check(struct work_struct *work)
{
    int i;
    int level,oldlevel;
    int new_capacity, old_capacity;
    struct rk29_adc_battery_data *bat = gBatteryData;
    
    old_charge_level = -1;
    pSamples = bat->adc_samples;
    
    adc_sync_read(bat->client);                             //start adc sample
    level = oldlevel = rk29_adc_battery_status_samples(bat);//init charge status
    
    for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++)                //0.3 s
    {
        mdelay(1);
        rk29_adc_battery_voltage_samples(bat);              //get voltage
        level = rk29_adc_battery_status_samples(bat);       //check charge status
        if (oldlevel != level)
        {
            oldlevel = level;                               //if charge status changed, reset sample
            i = 0;
        }        
    }
    new_capacity = rk29_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
    old_capacity = suspend_capacity;
    
    if (bat->bat_status != POWER_SUPPLY_STATUS_NOT_CHARGING)
    {
        //chargeing state
        bat->bat_capacity = (new_capacity > old_capacity) ? new_capacity : old_capacity;
    }
    else
    {
        bat->bat_capacity = (new_capacity < old_capacity) ? new_capacity : old_capacity;
    }
    
    printk("rk29_adc_battery_resume: status = %d, voltage = %d, capacity = %d, new_capacity = %d, old_capacity = %d\n",
                                     bat->bat_status, bat->bat_voltage, bat->bat_capacity, new_capacity, old_capacity);
    
    //start timer scan
	schedule_work(&bat->timer_work);
    bat->timer.expires  = jiffies + 10;
	add_timer(&bat->timer);
}

static int rk29_adc_battery_suspend(struct platform_device *dev, pm_message_t state)
{
	/* flush all pending status updates */
	suspend_capacity = gBatteryData->bat_capacity;
	del_timer(&gBatteryData->timer);
	//flush_scheduled_work();
	return 0;
}

static int rk29_adc_battery_resume(struct platform_device *dev)
{
	/* things may have changed while we were away */
	schedule_work(&gBatteryData->resume_work);
	return 0;
}
#else
#define rk29_adc_battery_suspend NULL
#define rk29_adc_battery_resume NULL
#endif


static int rk29_adc_battery_io_init(struct rk29_adc_battery_data *data, struct rk29_adc_battery_platform_data *pdata)
{
    int ret = 0;
    
    data->pdata = pdata;
	
	if (pdata->io_init) 
	{
		pdata->io_init();
	}
	
	//charge control pin
	if (pdata->charge_set_pin != INVALID_GPIO)
	{
    	ret = gpio_request(pdata->charge_set_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto error;
    	}
    	gpio_direction_output(pdata->charge_set_pin, 1 - pdata->charge_set_level);
    }
	
	//dc charge detect pin
	if (pdata->dc_det_pin != INVALID_GPIO)
	{
    	ret = gpio_request(pdata->dc_det_pin, NULL);
    	if (ret) {
    		printk("failed to request dc_det gpio\n");
    		goto error;
    	}
	
    	gpio_pull_updown(pdata->dc_det_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->dc_det_pin);
    	if (ret) {
    		printk("failed to set gpio dc_det input\n");
    		goto error;
    	}
    }
	
	//charge ok detect
	if (pdata->charge_ok_pin != INVALID_GPIO)
	{
        ret = gpio_request(pdata->charge_ok_pin, NULL);
    	if (ret) {
    		printk("failed to request charge_ok gpio\n");
    		goto error;
    	}
	
    	gpio_pull_updown(pdata->charge_ok_pin, GPIOPullUp);//important
    	ret = gpio_direction_input(pdata->charge_ok_pin);
    	if (ret) {
    		printk("failed to set gpio charge_ok input\n");
    		goto error;
    	}
    }
    
    return 0;
error:
    return -1;
}

#define POWER_ON_PIN    RK29_PIN4_PA4
static void rk29_adc_battery_lowpower_check(struct rk29_adc_battery_data *bat)
{
    int i;
    int tmp = 0;
    int level,oldlevel;
    struct rk29_adc_battery_platform_data *pdata = bat->pdata;
    
    printk("%s--%d:\n",__FUNCTION__,__LINE__);
    
    old_charge_level = -1;
    pSamples = bat->adc_samples;
    
    adc_sync_read(bat->client);                             //start adc sample
    level = oldlevel = rk29_adc_battery_status_samples(bat);//init charge status
    
    bat->full_times = 0;
    for (i = 0; i < NUM_VOLTAGE_SAMPLE; i++)                //0.3 s
    {
        mdelay(1);
        rk29_adc_battery_voltage_samples(bat);              //get voltage
        //level = rk29_adc_battery_status_samples(bat);       //check charge status
        level = rk29_adc_battery_get_charge_level(bat);
        if (oldlevel != level)
        {
            oldlevel = level;                               //if charge status changed, reset sample
            i = 0;
        }        
    }
    
    bat->bat_capacity = rk29_adc_battery_voltage_to_capacity(bat, bat->bat_voltage);
    bat->bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
    if (rk29_adc_battery_get_charge_level(bat))
    {
        bat->bat_status = POWER_SUPPLY_STATUS_CHARGING;
        if (pdata->charge_ok_pin != INVALID_GPIO)
        {
            if (gpio_get_value(pdata->charge_ok_pin) == pdata->charge_ok_level)
            {
                bat->bat_status = POWER_SUPPLY_STATUS_FULL;
                bat->bat_capacity = 100;
            }
        }
    }
    
#if 0
    rk29_adc_battery_poweron_capacity_check();
#else
    poweron_check = 1;
#endif

    
    /*******************************************
    //¿ª»ú²ÉÑùµ½µÄµçÑ¹ºÍÉÏ´Î¹Ø»ú±£´æµçÑ¹Ïà²î½Ï´ó£¬ÔõÃ´´¦Àí£¿
    if (bat->bat_capacity > old_capacity)
    {
        if ((bat->bat_capacity - old_capacity) > 20)
        {
            
        }
    }
    else if (bat->bat_capacity < old_capacity)
    {
        if ((old_capacity > bat->bat_capacity) > 20)
        {
            
        }
    }
    *********************************************/
    if (bat->bat_capacity == 0) bat->bat_capacity = 1;
    
    if (bat->bat_voltage <= BATT_ZERO_VOL_VALUE + 150)
    {
        printk("low battery: powerdown\n");
        gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
        tmp = 0;
        while(1)
        {
            if(gpio_get_value(POWER_ON_PIN) == GPIO_HIGH)
		    {
			    gpio_set_value(POWER_ON_PIN,GPIO_LOW);
		    }
		    mdelay(5);
		    if (++tmp > 50) break;
		}
    }
    gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
}

static void rk29_adc_battery_callback(struct adc_client *client, void *param, int result)
{
    gBatteryData->adc_val = result;
	return;
}

static int rk29_adc_battery_probe(struct platform_device *pdev)
{
	int    ret;
	int    irq;
	int    irq_flag;
	struct adc_client                   *client;
	struct rk29_adc_battery_data          *data;
	struct rk29_adc_battery_platform_data *pdata = pdev->dev.platform_data;
	
	printk("%s--%d:\n",__FUNCTION__,__LINE__);

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (data == NULL) {
		ret = -ENOMEM;
		goto err_data_alloc_failed;
	}
	gBatteryData = data;
	platform_set_drvdata(pdev, data);

	ret = rk29_adc_battery_io_init(data, pdata);
    if (ret)
    {
        goto err_io_init;
    }
    
    //register adc for battery sample
	memset(data->adc_samples, 0, sizeof(int)*(NUM_VOLTAGE_SAMPLE + 2));
    client = adc_register(0, rk29_adc_battery_callback, NULL);
    if(!client)
		goto err_adc_register_failed;
    
    //variable init
	data->client  = client;
	data->adc_val = adc_sync_read(client);
	
	//init a timer for adc sample
	//init a delay work for adc timer work
    setup_timer(&data->timer, rk29_adc_battery_scan_timer, (unsigned long)data);
	data->timer.expires  = jiffies + 2000;
	add_timer(&data->timer);

	INIT_WORK(&data->timer_work, rk29_adc_battery_timer_work);
	INIT_WORK(&data->resume_work, rk29_adc_battery_resume_check);
	
#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	//init dc dectet irq & delay work
    if (pdata->dc_det_pin != INVALID_GPIO)
    {
        irq = gpio_to_irq(pdata->dc_det_pin);
        
        irq_flag = gpio_get_value (pdata->dc_det_pin) ? IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING;
    	ret = request_irq(irq, rk29_adc_battery_dc_wakeup, irq_flag, "rk29_adc_battery", NULL);
    	if (ret) {
    		printk("failed to request dc det irq\n");
    		goto err_dcirq_failed;
    	}
    	enable_irq_wake(irq);
    	
    	INIT_WORK(&data->dcwakeup_work, rk29_adc_battery_dcdet_delaywork);
    }
#endif
    
    //Power on Battery detect
	rk29_adc_battery_lowpower_check(data);
    
    //power supply register
    wake_lock_init(&batt_wake_lock, WAKE_LOCK_SUSPEND, "batt_lock");

	ret = power_supply_register(&pdev->dev, &rk29_battery_supply);
	if (ret)
	{
		printk(KERN_INFO "fail to battery power_supply_register\n");
		goto err_battery_failed;
	}
	
#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	ret = power_supply_register(&pdev->dev, &rk29_ac_supply);
	if (ret)
	{
		printk(KERN_INFO "fail to ac power_supply_register\n");
		goto err_ac_failed;
	}
#endif

#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
	ret = power_supply_register(&pdev->dev, &rk29_usb_supply);
	if (ret)
	{
		printk(KERN_INFO "fail to usb power_supply_register\n");
		goto err_usb_failed;
	}
#endif
	
	printk(KERN_INFO "rk29_adc_battery: driver initialized\n");
	
	return 0;
	
#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
err_usb_failed:
	power_supply_unregister(&rk29_usb_supply);
#endif

err_ac_failed:
#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	power_supply_unregister(&rk29_ac_supply);
#endif

err_battery_failed:
	power_supply_unregister(&rk29_battery_supply);
    
err_dcirq_failed:
    free_irq(gpio_to_irq(pdata->dc_det_pin), data);
    
err_adc_register_failed:
err_io_init:    
err_data_alloc_failed:
	kfree(data);

    printk("rk29_adc_battery: error!\n");
    
	return ret;
}

static int rk29_adc_battery_remove(struct platform_device *pdev)
{
	struct rk29_adc_battery_data *data = platform_get_drvdata(pdev);
	struct rk29_adc_battery_platform_data *pdata = pdev->dev.platform_data;
	
#if defined(CONFIG_BATTERY_RK29_USB_CHARGE)
	power_supply_unregister(&rk29_usb_supply);
#endif
#if defined(CONFIG_BATTERY_RK29_AC_CHARGE)
	power_supply_unregister(&rk29_ac_supply);
#endif
	power_supply_unregister(&rk29_battery_supply);

	free_irq(gpio_to_irq(pdata->dc_det_pin), data);

	kfree(data);
	
	return 0;
}

static struct platform_driver rk29_adc_battery_driver = {
	.probe		= rk29_adc_battery_probe,
	.remove		= rk29_adc_battery_remove,
	.suspend	= rk29_adc_battery_suspend,
	.resume		= rk29_adc_battery_resume,
	.driver = {
		.name = "rk2918-battery",
		.owner	= THIS_MODULE,
	}
};

static int __init rk29_adc_battery_init(void)
{
	return platform_driver_register(&rk29_adc_battery_driver);
}

static void __exit rk29_adc_battery_exit(void)
{
	platform_driver_unregister(&rk29_adc_battery_driver);
}

subsys_initcall(rk29_adc_battery_init);
module_exit(rk29_adc_battery_exit);

MODULE_DESCRIPTION("Battery detect driver for the rk2918");
MODULE_AUTHOR("luowei lw@rock-chips.com");
MODULE_LICENSE("GPL");
