/*
 * linux/drivers/video/backlight/tps61158_pwm_bl.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
* 	@file	drivers/video/backlight/tps61158_pwm_bl.c
*
* Unless you and Broadcom execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Broadcom software provided under a license
* other than the GPL, without Broadcom's express prior written consent.
*******************************************************************************/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/tps61158_pwm_bl.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/broadcom/lcd.h>
#include <linux/spinlock.h>
#include <linux/broadcom/PowerManager.h>
#include <linux/rtc.h>

int current_intensity;

static int backlight_mode=1;

#define DIMMING_VALUE		8
#define MAX_BRIGHTNESS_VALUE	255
#define MIN_BRIGHTNESS_VALUE	20
#define BACKLIGHT_DEBUG 1
#define BACKLIGHT_SUSPEND 0
#define BACKLIGHT_RESUME 1

#define CABC_FUNCTION_ENABLE

#if BACKLIGHT_DEBUG
#define BLDBG(fmt, args...) printk(fmt, ## args)
#else
#define BLDBG(fmt, args...)
#endif

struct tps61158_bl_data {
	struct platform_device *pdev;
	unsigned int ctrl_pin;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend_desc;
#endif
};

struct brt_value{
	int level;				// Platform setting values
	int tune_level;			// Chip Setting values
};

#if defined (CONFIG_MACH_RHEA_SS_LUCAS)
struct brt_value brt_table[] = {
   { MIN_BRIGHTNESS_VALUE,  32 }, // Min pulse 32
   { 32,  31 },
   { 46,  30 },
   { 60,  29 },  
   { 73,  28 }, 
   { 86,  27 }, 
   { 98,  26 }, 
   { 105,  25 }, 
   { 110,  24 }, 
   { 115,  23 }, 
   { 120,  22 }, 
   { 125,  21 }, 
   { 130,  20 }, 
   { 140,  19 },//default value  
   { 155,  18 },   
   { 165,  17 },
   { 176,  16 }, 
   { 191,  15 }, 
   { 207,  14 },
   { 214,  13 },
   { 221,  12 },
   { 228,  10 },
   { 235,  8 },
   { 242,  7 },
   { 249,  5 },
   { MAX_BRIGHTNESS_VALUE,  5 }, // Max pulse 1
};
#else
struct brt_value brt_table[] = {
   { 20, 1 }, 
   { 25, 2 },
   { 30, 3 },
   { 35, 4 }, 
   { 40, 5 }, 
   { 45, 6 }, 
   { 50, 7 },
   { 55, 8 }, 
   { 60,  32 },
   { 65,  35 }, 
   { 70,  38 },
   { 75,  41 },
   { 80,  44 }, 
   { 85,  47 }, 
   { 90,  50 }, 
   { 95,  53 }, 
   { 100,  56 },   
   { 105,  59 },
   { 110,  62 }, 
   { 115,  65 }, 
   { 120,  68 },
   { 125,  71 },
   { 130,  74 },
   { 135,  77 },
   { 140,  80 },
   { 145,  83 },  
   { 150,  86 },
   { 155, 142 },
   { 160, 143 }, /* default 160 */
   { 165, 144 },
   { 170, 145 },
   { 175, 146 },
   { 180,  111 },
   { 185,  116 }, 
   { 190,  121 },
   { 195,  126 },
   { 200,  131 },
   { 205,  136 },
   { 210,  141 },
   { 215,  146 },
   { 220,  151 },
   { 225, 190 },
   { 230, 200 },
   { 235, 210 },
   { 240, 220 },
   { 245, 230 },
   { 250, 240 },
   { 255, 255 }, 
};
#endif


#ifdef CABC_FUNCTION_ENABLE

enum OUTDOOR {
	OUTDOOR_OFF,
	OUTDOOR_ON,
	OUTDOOR_MAX,
};

enum CABC {
	CABC_OFF,
	CABC_ON,
	CABC_MAX,
};

enum POWER_LUT_LEVEL {
	LUT_LEVEL_MANUAL_AND_INDOOR,
	LUT_LEVEL_OUTDOOR_1,
	LUT_LEVEL_OUTDOOR_2,
	LUT_LEVEL_MAX,
};

struct Vx5b3d_backlight_value {
	const unsigned int max;
	const unsigned int mid;
	const unsigned char low;
	const unsigned char dim;
};

typedef struct Vx5d3b_cabc_info {
	enum OUTDOOR			outdoor;
	enum CABC			cabc;
	enum POWER_LUT_LEVEL		powerLut;

	struct backlight_device		*bd;
	struct lcd_device		*lcd;
	struct Vx5b3d_backlight_value	*vee_lightValue;
	struct device			*dev;
	struct mutex			lock;
	struct mutex			pwr_lock;
	struct mutex			lvds_clk_switch_lock;

	unsigned int			auto_brightness;
	unsigned int			power_lut_idx;
	unsigned int			vee_strenght;
	unsigned int			prevee_strenght;	
	unsigned int			first_count;
	unsigned int			lcd_panel;
	int 				recovery_mode;
	unsigned int			lvds_clk;
	unsigned int			orig_lvds_clk;
	unsigned int			vx5b3d_backlight_frq;	
	int				lvds_clk_switching;
	int				i2cfail;
};

static struct Vx5b3d_backlight_value backlight_table[3] = {
	{	/*BOE/CPT*/
		.max = 235,
		.mid = 135,
		.low = 2,
		.dim = 1,
	}, {	/*SDC*/
		.max = 189,
		.mid = 111,
		.low = 2,
		.dim = 1,
	}, {	/*BOEVE*/
		.max = 235,
		.mid = 135,
		.low = 2,
		.dim = 1,
	}
};

#define V5D3BX_VEESTRENGHT		0x00001f07
#define V5D3BX_VEEDEFAULTVAL		0
#define V5D3BX_DEFAULT_STRENGHT		5
#define V5D3BX_DEFAULT_LOW_STRENGHT	8
#define V5D3BX_DEFAULT_HIGH_STRENGHT	10
#define V5D3BX_MAX_STRENGHT		15

#define V5D3BX_CABCBRIGHTNESSRATIO	815
#define V5D3BX_10KHZ_DEFAULT_RATIO	4707
#define V5D3BX_5P9KHZ_DEFAULT_RATIO	8000
#define V5D3BX_5P9KHZ_50P98_OFFSET	458
#define V5D3BX_5P9KHZ_50P18_OFFSET	329
#define V5D3BX_5P9KHZ_48P52_OFFSET	50

#define AUTOBRIGHTNESS_LIMIT_VALUE	207

#define MIN_BRIGHTNESS			0
#define MAX_BRIGHTNESS_LEVEL		255
#define MID_BRIGHTNESS_LEVEL		195
#define LOW_BRIGHTNESS_LEVEL		20
#define DIM_BRIGHTNESS_LEVEL		19
#define LOW_BATTERY_LEVEL		10
#define MINIMUM_VISIBILITY_LEVEL	30
#define DEFAULT_BRIGHTNESS		MID_BRIGHTNESS_LEVEL
#define LVDS_CLK_48P19Mhz		0
#define LVDS_CLK_50P98Mhz		1
#define LVDS_CLK_50P18Mhz		2
#define LVDS_CLK_48P52Mhz		3

struct Vx5b3d_backlight_value *pwm;
struct class *mdnie_class;
struct Vx5d3b_cabc_info *g_vx5d3b = NULL;
static int lt06_update_brightness(struct Vx5d3b_cabc_info *g_vx5d3b);

extern int ql_backlight_i2c_write(long addr, long val, int data_size);
extern int ql_i2c_release(void);

static ssize_t auto_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct Vx5d3b_cabc_info *Vee_cabc = g_vx5d3b;
	char *pos = buf;

	pos += sprintf(pos, "%d, %d, ", Vee_cabc->auto_brightness, Vee_cabc->power_lut_idx);
	/*
	for (i = 0; i < 5; i++)
		pos += sprintf(pos, "0x%02x, ", power_lut[mdnie->power_lut_idx][0][i]);
	pos += sprintf(pos, "\n");
	*/
	return pos - buf;
}

static ssize_t auto_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct Vx5d3b_cabc_info *Vee_cabc = g_vx5d3b;
	int value = 0, rc = 0, ret = 0;

	rc = strict_strtoul(buf, (unsigned int)0, (unsigned long *)&value);
	
	if (rc < 0)
		return rc;
	else {
		pr_info("auto_brightness[%d] -> [%d]\n",\
				Vee_cabc->auto_brightness, value);

		mutex_lock(&Vee_cabc->lock);
		if (value == 0) {
			printk("vee off [%d]\n", value);
			ret |= ql_backlight_i2c_write(0x710,0x054D000B,1);
			mdelay(1);
			ret |= ql_backlight_i2c_write(0x174,0x0,1);

		} else if (Vee_cabc->auto_brightness != value){
			printk("vee on [%d]\n", value);
			ret |= ql_backlight_i2c_write(0x710,0x054D004B,1);
			mdelay(1);
			ret |= ql_backlight_i2c_write(0x174,0xff,1);
		}
		mutex_unlock(&Vee_cabc->lock);
		
		mdelay(1);

		if (ret < 0)
			pr_info("tc35876x_i2c_write fail [%d] ! \n",ret);

		mutex_lock(&Vee_cabc->lock);

		Vee_cabc->auto_brightness = value;

		Vee_cabc->cabc = (value) ? CABC_ON : CABC_OFF;

				
		lt06_update_brightness(Vee_cabc);
		
		mutex_unlock(&Vee_cabc->lock);
	}
	return size;
}

static DEVICE_ATTR(auto_brightness, 0644, auto_brightness_show, auto_brightness_store);

static ssize_t vee_strenght_store(struct device *dev, struct
device_attribute *attr, const char *buf, size_t size)
{
	struct Vx5d3b_cabc_info *Vee_cabc = g_vx5d3b;

	int value;
	u32 vee_value = 0x00001f07;	
	int rc;
		
	rc = strict_strtoul(buf, (unsigned int) 0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		dev_info(dev, "vee_strenght_store[0x400] - %d, %d\n", \
			Vee_cabc->vee_strenght, value);

		if (Vee_cabc->vee_strenght!= value) {
			Vee_cabc->vee_strenght = value;
			vee_value = vee_value | (value << 27);
			ql_backlight_i2c_write(0x400,vee_value,1);
			ql_i2c_release();//vx5d3b_i2c_release();
			pr_info("vee_strenght value [0x%x]\n",vee_value);			
		}

		return size;
	}
}
static DEVICE_ATTR(vee_strenght, 0664,NULL, vee_strenght_store);

static ssize_t cabc_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct Vx5d3b_cabc_info *Vee_cabc = g_vx5d3b;

	return sprintf(buf, "%d\n", Vee_cabc->cabc);
}
 
static ssize_t cabc_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct Vx5d3b_cabc_info *Vee_cabc = g_vx5d3b;
	unsigned int value = 0;
	int ret = 0;

	if (Vee_cabc->auto_brightness)
		return count;

	ret = strict_strtoul(buf, 0, (unsigned long *)&value);

	dev_info(dev, "%s :: value=%d\n", __func__, value);

	if (value >= CABC_MAX)
		value = CABC_OFF;

	value = (value) ? CABC_ON : CABC_OFF;

	mutex_lock(&Vee_cabc->lock);
	Vee_cabc->cabc = value;
	lt06_update_brightness(Vee_cabc);
	mutex_unlock(&Vee_cabc->lock);

	return count;
}

static struct device_attribute mdnie_attributes[] = {
	__ATTR(cabc, 0664, cabc_show, cabc_store),
	__ATTR_NULL,
};


static int lt06_update_brightness(struct Vx5d3b_cabc_info *g_vx5d3b)
{
	struct Vx5b3d_backlight_value *pwm = g_vx5d3b->vee_lightValue;

	int brightness = g_vx5d3b->bd->props.brightness;

	int vx5b3d_brightness = 0;
	u32 vee_strenght = 0;
	int ret = 0;

	if (g_vx5d3b->bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (g_vx5d3b->bd->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;
	
	if(g_vx5d3b->lvds_clk_switching)
		brightness = 0;

	/*
	register 0x160
	register 0x164
			  value of 0x164
	---> duty ratio = -------------
			  value of 0x160
	*/
	if ((g_vx5d3b->cabc) && (g_vx5d3b->auto_brightness >= 5)) {
		if (brightness <= AUTOBRIGHTNESS_LIMIT_VALUE)
			brightness = AUTOBRIGHTNESS_LIMIT_VALUE;
	}
	/* brightness tuning*/
	if (brightness > MAX_BRIGHTNESS_LEVEL)
		brightness = MAX_BRIGHTNESS_LEVEL;

	if (brightness == LOW_BATTERY_LEVEL)/*Outgoing Quality Control Group issue*/
		brightness = MINIMUM_VISIBILITY_LEVEL;

	if (brightness >= MID_BRIGHTNESS_LEVEL) {
		vx5b3d_brightness  = (brightness - MID_BRIGHTNESS_LEVEL) *
		(pwm->max - pwm->mid) / (MAX_BRIGHTNESS_LEVEL-MID_BRIGHTNESS_LEVEL) + pwm->mid;
	} else if (brightness >= LOW_BRIGHTNESS_LEVEL) {
		vx5b3d_brightness  = (brightness - LOW_BRIGHTNESS_LEVEL) *
		(pwm->mid - pwm->low) / (MID_BRIGHTNESS_LEVEL-LOW_BRIGHTNESS_LEVEL) + pwm->low;
	} else if (brightness >= DIM_BRIGHTNESS_LEVEL) {
		vx5b3d_brightness  = (brightness - DIM_BRIGHTNESS_LEVEL) *
		(pwm->low - pwm->dim) / (LOW_BRIGHTNESS_LEVEL-DIM_BRIGHTNESS_LEVEL) + pwm->dim;
	} else if (brightness > 0)
		vx5b3d_brightness  = pwm->dim;
	else {
		vx5b3d_brightness = 0;
		pr_info("brightness = [%d]: vx5b3d_brightness = [%d]\n",\
			brightness,vx5b3d_brightness);	
	}

	if (g_vx5d3b->cabc) {

		switch (g_vx5d3b->auto_brightness) {

		case	0 ... 3:
			g_vx5d3b->vee_strenght = V5D3BX_DEFAULT_STRENGHT;				
			break;
		case	4 ... 5:
			g_vx5d3b->vee_strenght = V5D3BX_DEFAULT_LOW_STRENGHT;
			break;
		case 	6 ... 8:
			g_vx5d3b->vee_strenght = V5D3BX_DEFAULT_HIGH_STRENGHT;
			break;	
		default:
			g_vx5d3b->vee_strenght = V5D3BX_DEFAULT_STRENGHT;
		}
/*
	if (g_vx5d3b->auto_brightness >= 5)
		g_vx5d3b->vee_strenght = V5D3BX_MAX_STRENGHT;
*/
		vee_strenght = V5D3BX_VEESTRENGHT | ((g_vx5d3b->vee_strenght) << 27);

	if (!(g_vx5d3b->auto_brightness >= 5))
		vx5b3d_brightness = (vx5b3d_brightness * V5D3BX_CABCBRIGHTNESSRATIO) / 1000;

	} else {
		vee_strenght = V5D3BX_VEESTRENGHT | (V5D3BX_VEEDEFAULTVAL << 27);
	}

	/* brightness setting from platform is from 0 to 255 */
	mutex_lock(&g_vx5d3b->pwr_lock);

	if (LVDS_CLK_50P98Mhz == g_vx5d3b->lvds_clk)	
		g_vx5d3b->vx5b3d_backlight_frq = V5D3BX_5P9KHZ_DEFAULT_RATIO + V5D3BX_5P9KHZ_50P98_OFFSET;
	else if (LVDS_CLK_50P18Mhz == g_vx5d3b->lvds_clk)
		g_vx5d3b->vx5b3d_backlight_frq = V5D3BX_5P9KHZ_DEFAULT_RATIO + V5D3BX_5P9KHZ_50P18_OFFSET;		
	else if (LVDS_CLK_48P52Mhz == g_vx5d3b->lvds_clk)
		g_vx5d3b->vx5b3d_backlight_frq = V5D3BX_5P9KHZ_DEFAULT_RATIO + V5D3BX_5P9KHZ_48P52_OFFSET;
	else	/*Default for 48.2Mhz*/
		g_vx5d3b->vx5b3d_backlight_frq = V5D3BX_5P9KHZ_DEFAULT_RATIO;

	if ((g_vx5d3b->prevee_strenght != vee_strenght) && (brightness != 0))
		ret |= ql_backlight_i2c_write(0x400,vee_strenght,1);

	if (!g_vx5d3b->first_count)
		ret |= ql_backlight_i2c_write(0x164,((vx5b3d_brightness * g_vx5d3b->vx5b3d_backlight_frq)/1000),1);

    printk("[vee] vx5b3d_brightness = %d, vee_strenght = %d !!! \n ",vx5b3d_brightness,vee_strenght);

#if 0
	/*backlight duty ration control when device is first backlight on.*/
	if (g_vx5d3b->first_count && brightness != 0) {
		printk("backlight control first...[%d] \n",brightness);
		vx5b3dx_backlightReg_on();
		ret |= ql_backlight_i2c_write(0x164,((vx5b3d_brightness * g_vx5d3b->vx5b3d_backlight_frq)/1000),1);

		if (ret < 0)
			i2cfail_sysfs_check(1);
			/*g_vx5d3b->i2cfail = 1;*/
		g_vx5d3b->first_count = false;
	}
   
#endif

	g_vx5d3b->prevee_strenght = vee_strenght;

	mutex_unlock(&g_vx5d3b->pwr_lock);

	if (ret < 0)
		pr_info("tc35876x_i2c_write fail [%d] ! \n",ret);
    
	return 0;
}

#endif


#define MAX_BRT_STAGE (int)(sizeof(brt_table)/sizeof(struct brt_value))

extern void backlight_control(int brigtness);
extern int ql_backlight_i2c_write(long addr, long val, int data_size);

/* input: intensity in percentage 0% - 100% */
static int tps61158_backlight_update_status(struct backlight_device *bd)
{
	int user_intensity = bd->props.brightness;
 	int tune_level = 0;
	int i;
  
	BLDBG("[BACKLIGHT] tps61158_backlight_update_status ==> user_intensity  : %d\n", user_intensity);
		
	if (bd->props.power != FB_BLANK_UNBLANK)
		user_intensity = 0;

	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		user_intensity = 0;

	if (bd->props.state & BL_CORE_SUSPENDED)
		user_intensity = 0;
		
	if(backlight_mode != BACKLIGHT_RESUME)
	{
		BLDBG("[BACKLIGHT] Returned with invalid backlight mode %d\n", backlight_mode);
		return 0;
	}

	if(user_intensity > 0) {
		if(user_intensity < MIN_BRIGHTNESS_VALUE) {
			tune_level = DIMMING_VALUE; //DIMMING
		} else if (user_intensity == MAX_BRIGHTNESS_VALUE) {
			tune_level = brt_table[MAX_BRT_STAGE-1].tune_level;
		} else {
			for(i = 0; i < MAX_BRT_STAGE; i++) {
				if(user_intensity <= brt_table[i].level ) {
					tune_level = brt_table[i].tune_level;
					break;
				}
			}
		}
	}
		
	BLDBG("[BACKLIGHT] tps61158_backlight_update_status ==> tune_level : %d\n", tune_level);

	//backlight_control(tune_level);
	ql_backlight_i2c_write(0x164,tune_level,1);
	
	return 0;
}

static int tps61158_backlight_get_brightness(struct backlight_device *bl)
{
	BLDBG("[BACKLIGHT] tps61158_backlight_get_brightness\n");
    
	return current_intensity;
}

static struct backlight_ops tps61158_backlight_ops = {
	.update_status	= tps61158_backlight_update_status,
	.get_brightness	= tps61158_backlight_get_brightness,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void tps61158_backlight_earlysuspend(struct early_suspend *desc)
{
	struct timespec ts;
	struct rtc_time tm;
	
	backlight_mode=BACKLIGHT_SUSPEND; 

	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	printk("[%02d:%02d:%02d.%03lu][BACKLIGHT] earlysuspend\n", tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}

static void tps61158_backlight_earlyresume(struct early_suspend *desc)
{
	struct tps61158_bl_data *tps61158 = container_of(desc, struct tps61158_bl_data, early_suspend_desc);
	struct backlight_device *bl = platform_get_drvdata(tps61158->pdev);
	struct timespec ts;
	struct rtc_time tm;
	
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	backlight_mode = BACKLIGHT_RESUME;
	printk("[%02d:%02d:%02d.%03lu][BACKLIGHT] earlyresume\n", tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);

	backlight_update_status(bl);
}
#else
#ifdef CONFIG_PM
static int tps61158_backlight_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct tps61158_bl_data *tps61158 = dev_get_drvdata(&bl->dev);
	  
	BLDBG("[BACKLIGHT] tps61158_backlight_suspend\n");
	      
	return 0;
}
static int tps61158_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
  BLDBG("[BACKLIGHT] tps61158_backlight_resume\n");
	    
	backlight_update_status(bl);
	    
	return 0;
}
#else
#define tps61158_backlight_suspend  NULL
#define tps61158_backlight_resume   NULL
#endif /* CONFIG_PM */
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int tps61158_backlight_probe(struct platform_device *pdev)
{
	struct platform_tps61158_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct tps61158_bl_data *tps61158;
	struct backlight_properties props;
	int ret;
	
#ifdef CABC_FUNCTION_ENABLE
	struct Vx5d3b_cabc_info *vx5d3bInfo;
#endif
	
	BLDBG("[BACKLIGHT] tps61158_backlight_probe\n");
	
	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}
	
	tps61158 = kzalloc(sizeof(*tps61158), GFP_KERNEL);
	if (!tps61158) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}
	
	//tps61158->ctrl_pin = data->ctrl_pin;
    
	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = data->max_brightness;
	props.type = BACKLIGHT_PLATFORM;
	bl = backlight_device_register(pdev->name, &pdev->dev,
			tps61158, &tps61158_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	tps61158->pdev = pdev;
	tps61158->early_suspend_desc.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	tps61158->early_suspend_desc.suspend = tps61158_backlight_earlysuspend;
	tps61158->early_suspend_desc.resume = tps61158_backlight_earlyresume;
	register_early_suspend(&tps61158->early_suspend_desc);
#endif

	bl->props.max_brightness = data->max_brightness;
	bl->props.brightness = data->dft_brightness;
	platform_set_drvdata(pdev, bl);
  //tps61158_backlight_update_status(bl);
    
#ifdef CABC_FUNCTION_ENABLE
	/*For v5b3dx cabc*/
	mdnie_class = class_create(THIS_MODULE, "mdnie");

	if (IS_ERR_OR_NULL(mdnie_class)) {
		pr_err("failed to create mdnie class\n");
	}

	mdnie_class->dev_attrs = mdnie_attributes;
	
	vx5d3bInfo = kzalloc(sizeof(struct Vx5d3b_cabc_info), GFP_KERNEL);

	if (!vx5d3bInfo) {
		pr_err("failed to allocate vx5d3bInfo\n");
		ret = -ENOMEM;
		goto error1;
	}

	vx5d3bInfo->dev = device_create(mdnie_class, NULL, 0, &vx5d3bInfo, "mdnie");

	if (IS_ERR_OR_NULL(vx5d3bInfo->dev)) {
		pr_err("failed to create mdnie device\n");
		ret = -EINVAL;
		goto error2;
	}

#if 0
	/* Register backlight  control */
	vx5d3bInfo->bd = backlight_device_register("panel", vx5d3bInfo->dev,
					vx5d3bInfo, &lt02_backlight_ops, NULL);

	vx5d3bInfo->bd->props.max_brightness = MAX_BRIGHTNESS_LEVEL;
	vx5d3bInfo->bd->props.brightness = DEFAULT_BRIGHTNESS;
	vx5d3bInfo->bd->props.type = BACKLIGHT_RAW;
#endif
	vx5d3bInfo->cabc = CABC_OFF;
	vx5d3bInfo->vee_lightValue = 0;
	vx5d3bInfo->vee_strenght = V5D3BX_VEEDEFAULTVAL;
	vx5d3bInfo->prevee_strenght = 1;
	vx5d3bInfo->auto_brightness = false;
	vx5d3bInfo->first_count = false;
	vx5d3bInfo->vee_lightValue = &backlight_table[1];
	vx5d3bInfo->lvds_clk = LVDS_CLK_48P19Mhz;
 	vx5d3bInfo->orig_lvds_clk = LVDS_CLK_48P19Mhz;
 	vx5d3bInfo->lvds_clk_switching = false;
	vx5d3bInfo->vx5b3d_backlight_frq = V5D3BX_5P9KHZ_DEFAULT_RATIO;
	vx5d3bInfo->i2cfail = 0;


	if (device_create_file(&bl->dev, &dev_attr_auto_brightness) < 0)
		pr_err("Failed to create auto_brightness\n");

	//if (device_create_file(&bl->dev, &dev_attr_vx5b3d_Regread) < 0)
		//pr_err("Failed to create vx5b3d_Regread\n");
	
	//if (device_create_file(&vx5d3bInfo->lcd->dev, &dev_attr_vee_strenght) < 0)
		//pr_info("Failed to create device file for vee_strenght!\n");

	mutex_init(&vx5d3bInfo->lock);
	mutex_init(&vx5d3bInfo->pwr_lock);
	mutex_init(&vx5d3bInfo->lvds_clk_switch_lock);

	g_vx5d3b = vx5d3bInfo;
	g_vx5d3b->bd = bl;

#endif

    
	return 0;
	
err_bl:
	kfree(tps61158);
err_alloc:
	return ret;

#ifdef CABC_FUNCTION_ENABLE
error2:
	kfree(vx5d3bInfo);	
error1:
	class_destroy(mdnie_class);
	return ret;
#endif

}

static int tps61158_backlight_remove(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct tps61158_bl_data *tps61158 = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&tps61158->early_suspend_desc);
#endif

	kfree(tps61158);

	return 0;
}

extern void power_off_LCD(void);
static int tps61158_backlight_shutdown(struct platform_device *pdev)
{
	printk("[BACKLIGHT] tps61158_backlight_shutdown\n");
	ql_backlight_i2c_write(0x164,0x00,1);
	mdelay(100);
    
	power_off_LCD();
	mdelay(100);
	
	return 0;
}

static struct platform_driver tps61158_backlight_driver = {
	.driver		= {
		.name	= "panel",
		.owner	= THIS_MODULE,
	},
	.probe		= tps61158_backlight_probe,
	.remove		= tps61158_backlight_remove,
	.shutdown      = tps61158_backlight_shutdown,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend        = tps61158_backlight_suspend,
	.resume         = tps61158_backlight_resume,
#endif
};

static int __init tps61158_backlight_init(void)
{
	return platform_driver_register(&tps61158_backlight_driver);
}

module_init(tps61158_backlight_init);

static void __exit tps61158_backlight_exit(void)
{
	platform_driver_unregister(&tps61158_backlight_driver);
}

module_exit(tps61158_backlight_exit);
MODULE_DESCRIPTION("tps61158 based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tps61158-backlight");

