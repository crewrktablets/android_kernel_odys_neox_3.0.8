#include <linux/fb.h>
#include <linux/delay.h>
#include "../../rk29_fb.h"
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/board.h>
#include "screen.h"

#if defined(CONFIG_MACH_RK29_ODYS_NEOX8)
/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#define OUT_CLK			40000000
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
#define SWAP_RB			0

#define LCD_WIDTH       	170
#define LCD_HEIGHT      	128 

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

void set_lcd_info(struct rk29fb_screen *screen, struct rk29lcd_info *lcd_info )
{
    /* screen type & face */
    screen->type = OUT_TYPE;
    screen->face = OUT_FACE;

    /* Screen size */
    screen->x_res = H_VD;
    screen->y_res = V_VD;

    screen->width = LCD_WIDTH;
    screen->height = LCD_HEIGHT;

    /* Timing */
    screen->lcdc_aclk = LCDC_ACLK;
    screen->pixclock = OUT_CLK;
	screen->left_margin = H_BP;
	screen->right_margin = H_FP;
	screen->hsync_len = H_PW;
	screen->upper_margin = V_BP;
	screen->lower_margin = V_FP;
	screen->vsync_len = V_PW;

	/* Pin polarity */
	screen->pin_hsync = 0;
	screen->pin_vsync = 0;
	screen->pin_den = 0;
	screen->pin_dclk = DCLK_POL;

	/* Swap rule */
    screen->swap_rb = SWAP_RB;
    screen->swap_rg = 0;
    screen->swap_gb = 0;
    screen->swap_delta = 0;
    screen->swap_dumy = 0;

    /* Operation function*/
    screen->init = NULL;
    screen->standby = NULL;
}



