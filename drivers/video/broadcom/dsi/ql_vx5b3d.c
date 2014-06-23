/*
 * Copyright (C) Quicklogic 2013
 *
 * Quicklogic BX5B3A MIPI-to-RGB display driver
 * Author : Sunny
 *
 *
 * License terms: GNU General Public License (GPL), version 2.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/suspend.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <mach/pinmux.h>
#include <mach/rdb/brcm_rdb_sysmap.h>
#include <mach/rdb/brcm_rdb_dsi1.h>
#include <mach/io_map.h>
#include "ql_vx5b3d.h"

/* Unique ID allocation */
static struct i2c_client *i2c_quick_client = NULL;

static struct regulator *lvds_31_reg;
static struct regulator *lvds_12_reg;

static struct regulator *disp_33_reg;

#define QL_ID                        0X2300
#define CONTROL_BYTE_I2C_RELEASE     (0x00u)

#define QL_I2C_RELEASE  {\
	CONTROL_BYTE_I2C_RELEASE, \
}

struct QL_VX_INIT_INFO
{
	u16 address;
	u32 data;
};

static struct QL_VX_INIT_INFO ql_initialization_setting[] = {
	0x700 , 0x6C900040 ,
	0x704 , 0x302D1 ,
	0x70C , 0x00004604 ,
	0x710 , 0x54D004B ,
	0x714 , 0x20 ,
	0x718 , 0x00000102 ,
	0x71C , 0xA8002F ,
	0x720 , 0x0 ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
};

static struct QL_VX_INIT_INFO ql_initialization_2_setting[] = {
	0x700 , 0x6C900840 ,
	0x70C , 0x5E56 ,
	0x718 , 0x00000202 ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
	0x120 , 0x5 ,
	0x124 , 0x512C400 ,
	0x128 , 0x103014 ,
	0x12C , 0x89 ,
	0x130 , 0x3C18 ,
	0x134 , 0x15 ,
	0x138 , 0xFF0000 ,
	0x13C , 0x0 ,
	0x140 , 0x10000 ,
	0x174 , 0xff ,
	0x20C , 0x124 ,
	0x21C , 0x0 ,
	0x224 , 0x7 ,
	0x228 , 0x50001 ,
	0x22C , 0xFF03 ,
	0x230 , 0x1 ,
	0x234 , 0xCA033E10 ,
	0x238 , 0x00000060 ,
	0x23C , 0x82E86030 ,
	0x244 , 0x001E0285 ,
	0x258 , 0x40014 ,
	0x158 , 0x0 ,
	0x158 , 0x1 ,
};
static struct QL_VX_INIT_INFO ql_initialization_3_setting[] = {
	0x37C , 0x00001063 ,
	0x380 , 0x82A86030 ,
	0x384 , 0x2861408B ,
	0x388 , 0x00130285 ,
	0x38C , 0x10630009 ,
	0x394 , 0x400B82A8 ,
	0x600 , 0x16CC78C ,
	0x604 , 0x3FFFFFE0 ,
	0x608 , 0xD8C ,
	0x154 , 0x00000000 ,
	0x154 , 0x80000000 ,
};

//Porting: put your pre-standby seq. here.
static struct QL_VX_INIT_INFO ql_initialization_4_setting[] = {
//	0x300, 0x00000000,
//	0x300, 0x00000001,
	0x304, 0xFFFFFFFF,
	0x204, 0xFFFFFFFF,
	0x148, 0xFFFFFFFF,
};

//Porting: put your pre-standby seq. here.
static struct QL_VX_INIT_INFO ql_initialization_5_setting[] = {
	0x114, 0xc6302,
	0x160, 0xff,
	0x164, 0x00, // initially it was 0x7f 
	0x138, 0xff0000,
	0x15C, 0x5,
};
//Porting: put your pre-standby seq. here.
static struct QL_VX_INIT_INFO ql_pre_standby_setting[] = {
	0x15C, 0x00000001,
};

//Porting: put your standby seq. here.
static struct QL_VX_INIT_INFO ql_standby_setting[] = {
	{0x700, 0x24000040},
	{0x704, 0x9F0009},
	{0x70C, 0x600},
	{0x710, 0xC0002},
	{0x714, 0x20},
	{0x718, 0x309},
	{0x154, 0x00000000},
	{0x154, 0x80000000},
	{0x248, 0x10630009},
	{0x378, 0xCA0F3C90},
	{0x37C, 0x00000060},
	{0x38C, 0x10630009},
	{0x608, 0x50F},
};

#define LCD_TYPE_DISPLAY

#ifdef LCD_TYPE_DISPLAY

extern struct device *lcd_dev;

static ssize_t show_lcd_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	char temp[20];
	sprintf(temp, "BOE_BA070WS1\n");
	strcat(buf, temp);
	return strlen(buf);
}

static DEVICE_ATTR(lcd_type, 0444, show_lcd_info, NULL);
#endif

//#define QL_VX_REG_I2C_DIRECT_ACCESS

#ifdef QL_VX_REG_I2C_DIRECT_ACCESS
#define CONTROL_BYTE_DA_WR     (0x0Au)
#define CONTROL_BYTE_DA_RD     (0x0Eu)

#define DA_QL_WRITE  {\
	CONTROL_BYTE_DA_WR, \
        0x00,  /* Address MS */\
        0x00,  /* Address LS */\
        0x00,  /* data LS */\
	0x00, \
	0x00, \
        0x00,  /* data MS */\
}

#define DA_QL_READ  {\
	CONTROL_BYTE_DA_RD, \
        0x00,  /* Address MS */\
        0x00,  /* Address LS */\
        0x00,  /* len MS */\
        0x00,  /* len LS */\
}

static int ql_i2c_read(u32 addr, u32 *val, u32 data_size)
{
	u32 data;
	char buf[] = DA_QL_READ;
	char rx[10];
	int ret = -1;
	int write_size;

	if (i2c_quick_client == NULL) {	/* No global client pointer? */
		printk(KERN_ERR"%s: i2c_quick_client == NULL! \n", __func__);
		return -1;
	}

	buf[1] = (addr >> 8) & 0xff;
	buf[2] = addr & 0xff;
	buf[3] = (data_size >> 8) & 0xff;
	buf[4] = data_size & 0xff;
	write_size = 5;

	/* Read register */
	if ((ret = i2c_master_send(i2c_quick_client, (char*)(&buf[0]),
				write_size )) != write_size) {
		printk(KERN_ERR"%s: DI i2c_master_send DI failed (%d)!\n", __func__, ret);
		return -1;
	}

	/* Return number of bytes or error */
	if ((ret = i2c_master_recv(i2c_quick_client, (char*)(&rx[0]),
			data_size )) != data_size) {
		printk(KERN_ERR"%s: DI i2c_master_recv failed (%d)!\n", __func__, ret);
		return -1;
	}

	data = rx[0];
	if (data_size > 1) 
		data |= (rx[1] << 8);
	if (data_size > 2)
		data |= (rx[2] << 16) | (rx[3] << 24);

	*val = data;

	QL_DBG("r0x%x=0x%x\n",addr,data);

	return 0;
}

static int ql_i2c_write(long addr, long val, int data_size)
{
	int ret = -1;
	int write_size;
	char buf[] = DA_QL_WRITE;

	if (i2c_quick_client == NULL) {	/* No global client pointer? */
		printk(KERN_ERR"%s: i2c_quick_client == NULL! \n", __func__);
		return -1;
	}

	buf[1] = (uint8_t)(addr >> 8);	/* Address MS */
	buf[2] = (uint8_t)addr;		/* Address LS */
	buf[3] = val & 0xff;
	buf[4] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[5] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[6] = (data_size > 3) ? ((val >> 24) & 0xff) : 0;
	write_size = data_size + 3;

	if ((ret = i2c_master_send(i2c_quick_client, (char*)(&buf[0]),
			write_size )) != write_size) {
		printk(KERN_ERR"%s: DI i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}
	
	QL_DBG("w0x%lx=0x%lx\n",addr,val);

	return 0;
}

#else

#define CONTROL_BYTE_GEN       (0x09u)

#define GEN_QL_CSR_OFFSET_LENGTH  {\
	CONTROL_BYTE_GEN, \
        0x29,  /* Data ID */\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x41,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* Length LS */\
        0x00,  /* Length MS */\
}

#define GEN_QL_CSR_WRITE  {\
	CONTROL_BYTE_GEN, \
        0x29,  /* Data ID */\
        0x05,  /* Vendor Id 1 */\
        0x01,  /* Vendor Id 2 */\
        0x40,  /* Vendor Unique Command */\
        0x00,  /* Address LS */\
        0x00,  /* Address MS */\
        0x00,  /* data LS */\
	0x00, \
	0x00, \
        0x00,  /* data MS */\
}

static int ql_i2c_read(u32 addr, u32 *val, u32 data_size)
{
	u32 data;
	char buf[] = GEN_QL_CSR_OFFSET_LENGTH;
	char rx[10];
	int ret = -1;
	int write_size;

	if (i2c_quick_client == NULL) {	/* No global client pointer? */
		printk(KERN_ERR"%s: i2c_quick_client == NULL! \n", __func__);
		return -1;
	}

	buf[5] = addr & 0xff;
	buf[6] = (addr >> 8) & 0xff;
	buf[7] = data_size & 0xff;
	buf[8] = (data_size >> 8) & 0xff;
	write_size = 9;

	if ((ret = i2c_master_send(i2c_quick_client, (char*)(&buf[0]),
			write_size )) != write_size) {
		printk(KERN_ERR"%s: GE i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	// generic read request 0x24 to send generic read command 
	buf[0] = CONTROL_BYTE_GEN;
	buf[1] =    0x24;  /* Data ID */
	buf[2] =    0x05;  /* Vendor Id 1 */
	buf[3] =    0x01;  /* Vendor Id 2 */
	write_size = 4;

	if ((ret = i2c_master_send(i2c_quick_client, (char*)(&buf[0]),
			write_size)) != write_size) {
		printk(KERN_ERR"%s: GE i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	if ((ret = i2c_master_recv(i2c_quick_client, (char*)(&rx[0]),
			data_size)) != data_size) {
		printk(KERN_ERR"%s: GE i2c_master_recv failed (%d)!\n", __func__, ret);
		return -1;
	}

	data = rx[0];
	if (data_size > 1) 
		data |= (rx[1] << 8);
	if (data_size > 2)
		data |= (rx[2] << 16) | (rx[3] << 24);
	*val = data;

	QL_DBG("r0x%x=0x%x\n",addr,data);

	return 0;
}

static int ql_i2c_write(long addr, long val, int data_size)
{
	int write_size;
	int ret = -1;
	char buf[] = GEN_QL_CSR_WRITE;

	if (i2c_quick_client == NULL) {	/* No global client pointer? */
		printk(KERN_ERR"%s: i2c_quick_client == NULL! \n", __func__);
		return -1;
	}

	buf[5] = (uint8_t)addr;  /* Address LS */
	buf[6] = (uint8_t)(addr >> 8);  /* Address MS */
	buf[7] = val & 0xff;
	buf[8] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[9] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[10] = (data_size > 3) ? ((val >> 24) & 0xff) : 0;
	write_size = data_size + 7;

	if ((ret = i2c_master_send(i2c_quick_client, (char*)(&buf[0]),
		write_size )) != write_size) {
		printk(KERN_ERR"%s: GE i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	QL_DBG("w0x%lx=0x%lx\n",addr,val);
	
#if 0
	ql_i2c_read(addr, &val, data_size);

	QL_DBG("r0x%lx=0x%lx\n",addr,val);
#endif
	return 0;
}

int ql_backlight_i2c_write(long addr, long val, int data_size)
{
	int write_size;
	int ret = -1;
	char buf[] = GEN_QL_CSR_WRITE;

	if (i2c_quick_client == NULL) {	/* No global client pointer? */
		printk(KERN_ERR"%s: i2c_quick_client == NULL! \n", __func__);
		return -1;
	}

	buf[5] = (uint8_t)addr;  /* Address LS */
	buf[6] = (uint8_t)(addr >> 8);  /* Address MS */
	buf[7] = val & 0xff;
	buf[8] = (data_size > 1) ? ((val >> 8) & 0xff) : 0;
	buf[9] = (data_size > 2) ? ((val >> 16) & 0xff) : 0;
	buf[10] = (data_size > 3) ? ((val >> 24) & 0xff) : 0;
	write_size = data_size + 7;

	if ((ret = i2c_master_send(i2c_quick_client, (char*)(&buf[0]),
		write_size )) != write_size) {
		printk(KERN_ERR"%s: GE i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	QL_DBG("w0x%lx=0x%lx\n",addr,val);
	
#if 0
	ql_i2c_read(addr, &val, data_size);

	QL_DBG("r0x%lx=0x%lx\n",addr,val);
#endif
	return 0;
}
EXPORT_SYMBOL(ql_backlight_i2c_write);

#endif

int ql_i2c_release(void)
{
	int write_size;
	int ret = -1;
	char buf[] = QL_I2C_RELEASE;

	QL_DBG("+++\n");

	if (i2c_quick_client == NULL)	/* No global client pointer? */
		return -1;

	write_size = 1;

	if ((ret = i2c_master_send(i2c_quick_client, (char*)(&buf[0]),
			write_size)) != write_size) {
		printk(KERN_ERR"%s: i2c_master_send failed (%d)!\n", __func__, ret);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(ql_i2c_release);

static void ql_init_table(struct QL_VX_INIT_INFO *table, unsigned int count)
{
	unsigned int i;
	u16 addr;
	u32 data;

	for(i = 0; i < count; i++) {
		addr = table[i].address;
		data = table[i].data;
		ql_i2c_write(addr, data, 4);
#if 0
		/* if you are using 32kHz sys_clk, and seeing i2c error after the first 0x154 write,
		you may try enable this, and adjust the delay below, now it's 0.5 seconds */
		if ((addr == 0x154) && (data == 0x80000000))
			msleep(500);
#endif
	}

	return;
}

#define DCLK2		"dig_ch1_clk"	/*DCLK2 */
#define DCLK2_FREQ	(19500000)
#define LCD_BLIC_RST_GPIO		(26) 
#define CV_SYS_RST_GPIO		(25)
#define LCD_PWN_GPIO			(24)
//#define LCD_RESET_GPIO			(22)

static int flash_gpio = 0;


static void setClkCont() {
	uint32_t val;
	val = readl(KONA_DSI0_VA + DSI1_PHYC_OFFSET);
	val |= (0x1<<DSI1_PHYC_TX_HSCLK_CONT_SHIFT);
	writel(val, KONA_DSI0_VA + DSI1_PHYC_OFFSET);
}


static int ql_init_external_hw( int on)
{
	unsigned int value;
	int ret = -1;
	struct clk *clock_pre,*clock;
	
	printk( "%s:quickvx power %s\n", __func__, (on ? "on" : "off"));

	if (!flash_gpio){
		if (gpio_request_one(LCD_BLIC_RST_GPIO , 
						GPIOF_DIR_OUT  | GPIOF_INIT_LOW, "lcd_dvdd_en")) {
			printk(KERN_ERR "GPIO lcd_dvdd_en failed\n");
			return -1;
		}

		if (gpio_request_one(LCD_PWN_GPIO , 
							GPIOF_DIR_OUT  | GPIOF_INIT_HIGH, "lcd_pwn")) {
			printk(KERN_ERR "GPIO lcd_pwn failed\n");
			return -1;
		}

		
		if (gpio_request_one(CV_SYS_RST_GPIO ,
					GPIOF_DIR_OUT  | GPIOF_INIT_LOW, "cv_sys_rst")) {
			printk(KERN_ERR "GPIO cv_sys_rst failed\n");
			return -1;
		}

		flash_gpio = 1;
	}	
	clock_pre = clk_get(NULL,"dig_prediv_clk");
	if (IS_ERR_OR_NULL(clock_pre)) {
		printk(KERN_ERR "Unable to get dig_prediv_clk\n");
		return -1;
	}

	clock = clk_get(NULL, DCLK2);
	if (IS_ERR_OR_NULL(clock)) {
		printk(KERN_ERR "Unable to get DCLK2 clock\n");
		return -1;
	}

	
	if (on) {

		regulator_enable(disp_33_reg);
            
		//gpio_set_value(LCD_PWN_GPIO, 1);
		//mdelay(20);
		gpio_set_value(LCD_PWN_GPIO, 1);
		mdelay(20);
		regulator_enable(lvds_31_reg);
		regulator_enable(lvds_12_reg);

		value = clk_enable(clock_pre);
			if (value) {
				printk("Failed to enable dig_prediv_clk clock\n");
				goto e_clk;
			}
		value = clk_set_rate(clock_pre, 78000000);
		if (0){//value) {
			printk("Failed to set clock_pre clock\n");
			goto e_clk_pre_set;
		}

		//step 1 set clk
		printk("enable DCLK2 clock\n");
		value = clk_enable(clock);
		if (value) {
			printk("Failed to enable DCLK2 clock\n");
			goto e_clk_pre_set;
		}
		value = clk_set_rate(clock, DCLK2_FREQ);
		if (value) {
			printk("Failed to set DCLK2 clock\n");
			goto e_clk_set;
		}
		
		mdelay(20); /* 131007 added */
		//step 3 set reset
		gpio_set_value(CV_SYS_RST_GPIO, 1);
		mdelay(20);

		gpio_set_value(LCD_BLIC_RST_GPIO, 1);
		mdelay(20);

	} else {
		ql_backlight_i2c_write(0x164,0x00,1);
		gpio_set_value(CV_SYS_RST_GPIO, 0);

		gpio_set_value(LCD_BLIC_RST_GPIO, 0);

		regulator_disable(lvds_12_reg);
		regulator_disable(lvds_31_reg);

		gpio_set_value(LCD_PWN_GPIO, 0);

		clk_disable(clock);
		clk_disable(clock_pre);
             
		regulator_set_always_on(disp_33_reg,false);

 		mdelay(100);
		regulator_disable(disp_33_reg);
	}
	return 0;

e_clk_set:
	clk_disable(clock);
e_clk_pre_set:
	clk_disable(clock_pre);
e_clk:

	return ret;
}

void power_off_LCD(void)
{
		ql_backlight_i2c_write(0x164,0x00,1);
		gpio_set_value(CV_SYS_RST_GPIO, 0);

		gpio_set_value(LCD_BLIC_RST_GPIO, 0);

		regulator_disable(lvds_12_reg);
		regulator_disable(lvds_31_reg);

		gpio_set_value(LCD_PWN_GPIO, 0);

		//clk_disable(clock);
		//clk_disable(clock_pre);

 		mdelay(100);
		regulator_disable(disp_33_reg);
}
EXPORT_SYMBOL(power_off_LCD);


int quickvx_init(void)
{
#if 1
	u32 val = 0;
	static int i = 0;
#endif
	ql_init_external_hw(1); //power up seq

#if 0
	if (ql_i2c_read(0x4fe, &val, 2) != 0) {
		QL_ERR("Read ID failed!\n");
		return -1;
	}

	if(val != QL_ID) {
		QL_ERR("Wrong ID (0x%x)!\n", val);
		return -1;
	}

	ql_i2c_write(0x448, 0x12345678, 4); 
	ql_i2c_read(0x448, &val, 4);
	
	printk("wangcongqin Test failed (0x%x)!\n", val);
	
	if (val != 0x12345678) {
		QL_ERR("R/W Test failed (0x%x)!\n", val);
		return -1;

	}
#endif
	//Porting : Load init table.
	ql_init_table(ql_initialization_setting, ARRAY_SIZE(ql_initialization_setting));
	mdelay(1);
	ql_init_table(ql_initialization_2_setting, ARRAY_SIZE(ql_initialization_2_setting));
	mdelay(1);
	ql_init_table(ql_initialization_3_setting, ARRAY_SIZE(ql_initialization_3_setting));
	mdelay(1);
	ql_init_table(ql_initialization_4_setting, ARRAY_SIZE(ql_initialization_4_setting));
	mdelay(20);
	ql_init_table(ql_initialization_5_setting, ARRAY_SIZE(ql_initialization_5_setting));
	mdelay(20);
/*
	val =0;
	if (ql_i2c_read(0x204, &val, 4)  >= 0)
		printk("[LVDS] r0x%lx=0x%lx\n",0x204,val);
	val=0;
	if (ql_i2c_read(0x148, &val, 4) >= 0)
		printk("[LVDS] r0x%lx=0x%lx\n",0x148,val);
*/

/* 
 Disable brightnes issue Caused by IBC
 read 4 bytes from address 0x410 to 0x413
 0x15E50300 is read value for 0x410 register
 0x5E50300= 0x15E50300 & 0xefffffff
  */
	ql_backlight_i2c_write(0x410, 0x5E50300,4);
 /*...end*/
 
	ql_i2c_release();

	setClkCont();

	QL_DBG("ok\n");

	return 0;
}
EXPORT_SYMBOL(quickvx_init);

int quickvx_standby(void)
{
	//Porting : put chip in standby. 
	ql_init_table(ql_pre_standby_setting, ARRAY_SIZE(ql_pre_standby_setting));
	mdelay(10);
	ql_init_table(ql_standby_setting, ARRAY_SIZE(ql_standby_setting));
	ql_i2c_release();
	ql_init_external_hw(0);////power down seq
	QL_DBG("ok\n");

	return 0;
}
EXPORT_SYMBOL(quickvx_standby);

static int __devinit qlvx5b3d_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
#ifdef LCD_TYPE_DISPLAY
      int ret;
#endif

	printk("\n ***********#########$$$$$$$$$$ qlvx5b3d_probe\n");
	i2c_quick_client = client;
#if 0
	void __iomem *base;
	base = ioremap(PAD_CTRL_BASE_ADDR, SZ_2K);
	writel(0x100, base + 31 * 4);
	printk( "0x%08x  pad 0x%x\n", readl(base+31* 4), base+31* 4);
	writel(0x100, base + 32 * 4);
	printk( "0x%08x  pad 0x%x\n", readl(base+32* 4), base+32* 4);
	writel(0x03, base + 39 * 4);
	printk( "0x%08x  pad 0x%x\n", readl(base+39* 4), base+39* 4);
	writel(0x03, base + 40 * 4);
	printk( "0x%08x  pad 0x%x\n", readl(base+40* 4), base+40* 4);
	writel(0x103, base + 50 * 4);
	printk( "0x%08x  pad 0x%x\n", readl(base+50* 4), base+50* 4);
#endif
	if (!lvds_31_reg) {
		/*SIM2 LDO */
		lvds_31_reg = regulator_get(NULL, "sim2_vcc");
		regulator_set_voltage(lvds_31_reg,3300000,3300000);
	}
	if (!lvds_12_reg) {
		/*vsr_uc LDO */
		lvds_12_reg = regulator_get(NULL, "vsr_uc");
		regulator_set_voltage(lvds_12_reg,1200000,1200000);
	}

       if (!disp_33_reg) {
		/*vsr_uc LDO */
		disp_33_reg = regulator_get(NULL, "camldo2");
		regulator_set_voltage(disp_33_reg,3300000,3300000);
	}

#ifdef LCD_TYPE_DISPLAY
      	ret = device_create_file(lcd_dev, &dev_attr_lcd_type);
	if (ret < 0)
		printk("Failed to add lcd_type sysfs entries, %d\n",	__LINE__);		
#endif

	return 0;
}

static int qlvx5b3d_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id qlvx5b3d_id[] = {
	{"ql_vx5b3d", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, qlvx5b3d_id);


static struct i2c_driver qlvx5b3d_driver = {
	.driver = {
		   .name = "qlvx5b3d",
	},
	.id_table = qlvx5b3d_id,
	.probe = qlvx5b3d_probe,
	.remove = qlvx5b3d_remove,
};

static int __init qlvx5b3d_init(void)
{
	QL_DBG("+++\n");

	return i2c_add_driver(&qlvx5b3d_driver);
}

static void __exit qlvx5b3d_exit(void)
{
	i2c_del_driver(&qlvx5b3d_driver);
}

module_init(qlvx5b3d_init);
module_exit(qlvx5b3d_exit);

MODULE_DESCRIPTION("ql_vx5b3d Driver");
MODULE_LICENSE("GPL");
