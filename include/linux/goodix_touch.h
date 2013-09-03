/*
 * include/linux/goodix_touch.h
 *
 * Copyright (C) 2008 Google, Inc.
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

#ifndef _LINUX_GOODIX_TOUCH_H
#define _LINUX_GOODIX_TOUCH_H

#define GOODIX_I2C_NAME "Goodix_TS"
#define GOODIX_I2C_ADDR 0x55
#define GUITAR_SMALL
//触摸屏的分辨率
#define TOUCH_MAX_HEIGHT 	1024
#define TOUCH_MAX_WIDTH	 	768
//显示屏的分辨率
//#define SCREEN_MAX_HEIGHT	1280
//#define SCREEN_MAX_WIDTH	768

#define MAX_FINGER_NUM	5    //最大支持手指数(<=5)
#define FLAG_UP 		0
#define FLAG_DOWN 	1

#define GOODIX_MULTI_TOUCH
#define GOODIX_TS_DEBUG

struct goodix_i2c_rmi_platform_data {
    uint32_t version;	/* Use this entry for panels with */
    unsigned gpio_shutdown;
    unsigned gpio_irq;
    bool irq_edge; /* 0:rising edge, 1:falling edge */
    bool swap_xy;
    bool xpol;
    bool ypol;
    int xmax;
    int ymax;
    int config_info_len;
    u8 *config_info;
    int config_info_len_bak;		//备用配置信息
    u8 *config_info_bak;
};

#endif /* _LINUX_GOODIX_TOUCH_H */
