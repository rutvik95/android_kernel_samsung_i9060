/************************************************************************/
/*                                                                      */
/*  Copyright 2013  Broadcom Corporation                                */
/*                                                                      */
/* Unless you and Broadcom execute a separate written software license  */
/* agreement governing use of this software, this software is licensed  */
/* to you under the terms of the GNU General Public License version 2   */
/* (the GPL), available at						*/
/*                                                                      */
/*          http://www.broadcom.com/licenses/GPLv2.php                  */
/*                                                                      */
/*  with the following added to such license:                           */
/*                                                                      */
/*  As a special exception, the copyright holders of this software give */
/*  you permission to link this software with independent modules, and  */
/*  to copy and distribute the resulting executable under terms of your */
/*  choice, provided that you also meet, for each linked independent    */
/*  module, the terms and conditions of the license of that module. An  */
/*  independent module is a module which is not derived from this       */
/*  software.  The special   exception does not apply to any            */
/*  modifications of the software.					*/
/*									*/
/*  Notwithstanding the above, under no circumstances may you combine	*/
/*  this software in any way with any other Broadcom software provided	*/
/*  under a license other than the GPL, without Broadcom's express	*/
/*  prior written consent.						*/
/*									*/
/************************************************************************/

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/stat.h>
#include <linux/delay.h>
#include <asm/page.h>
#include <asm/gpio.h>
#include <mach/pinmux.h>
#include <mach/hawaii_modem_detect.h>

#define BCM_MODEM_DETECT_GPIO_DEBOUNCE		1000
#define BCM_MODEM_DETECT_GPIO_CHARGE_MS		3

struct modem_detect_pin_info {
	enum PIN_NAME gpio_modem_detect_pin_name;
	enum PIN_FUNC gpio_modem_detect_pin_func;
};

static struct modem_detect_pin_info mod_det_pin_info;
static int modem_present = -1;

static ssize_t modem_detect_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	ssize_t ret = -EINTR;
	unsigned int val = modem_present;
	ret = scnprintf(buf, PAGE_SIZE-1, "%u\n", val);
	return ret;
}

static struct kobj_attribute modem_detect_attr = {
	.attr = {
		.name = "modem_detected",
		.mode = S_IRUGO,
	},
	.show = modem_detect_show,
	.store = NULL,
};

static int bcm_modem_detect_gpio_request(enum PIN_NAME pin_cfg_name,
		enum PIN_FUNC pin_cfg_func)
{
	int rc = 0;
	struct pin_config modem_det_pin_cfg;
	struct pin_config modem_det_pin_cfg_orig;
	unsigned int gpio_pin;
	enum PIN_FUNC pin_func_orig;

	if (is_ball_valid(pin_cfg_name)) {

		rc = pinmux_find_gpio(pin_cfg_name, &gpio_pin, &pin_func_orig);
		if (rc < 0) {
			pr_err("%s: Unable to find gpio pin\n", __func__);
			return rc;
		}

		modem_det_pin_cfg.name = pin_cfg_name;
		rc = pinmux_get_pin_config(&modem_det_pin_cfg);
		if (rc < 0) {
			pr_err("%s: Unable to find pin configuration\n",
					__func__);
			return rc;
		}

		/* Keep a copy of original configuration */
		memcpy(&modem_det_pin_cfg_orig, &modem_det_pin_cfg,
									sizeof(struct pin_config));

		modem_det_pin_cfg.reg.b.pull_up = 1;
		modem_det_pin_cfg.reg.b.pull_dn = 0;
		modem_det_pin_cfg.reg.b.drv_sth = DRIVE_STRENGTH_16MA;
		modem_det_pin_cfg.func = pin_cfg_func;

		rc = pinmux_set_pin_config(&modem_det_pin_cfg);
		if (rc < 0) {
			pr_err("%s: Unable to set pin configuration\n",
					__func__);
			return rc;
		}

		rc = gpio_request(gpio_pin, "modem_detect");
		if (rc < 0) {
			pr_err("Unable to request GPIO pin %d\n",
					gpio_pin);
			goto modem_detect_pin_config_restore;
		}

		/* Set to output to charge potentially existing capacitor */
		gpio_direction_output(gpio_pin, 1);

		mdelay(BCM_MODEM_DETECT_GPIO_CHARGE_MS);

		gpio_direction_input(gpio_pin);

		modem_present = gpio_get_value(gpio_pin);

modem_detect_pin_config_restore:
		gpio_free(gpio_pin);
		pinmux_set_pin_config(&modem_det_pin_cfg_orig);

	} else {
		rc = -EINVAL;
		pr_err("%s: Invalid gpio pin name provided %d\n",
				__func__, pin_cfg_name);
	}

	return rc;
}

static int __devinit bcm_modem_detect_pltfm_probe(struct platform_device *pdev)
{
	int rc = 0;
	if (pdev->dev.platform_data) {
		struct board_modem_detect_info *modem_detect_dev =
			pdev->dev.platform_data;

		mod_det_pin_info.gpio_modem_detect_pin_name =
			modem_detect_dev->gpen_pin_name;
		mod_det_pin_info.gpio_modem_detect_pin_func =
			modem_detect_dev->gpen_pin_func;

		rc = bcm_modem_detect_gpio_request(
				mod_det_pin_info.gpio_modem_detect_pin_name,
				mod_det_pin_info.gpio_modem_detect_pin_func);
		if (rc < 0) {
			pr_err("%s: Error evaluating gpio pin\n", __func__);
			return rc;
		}

		rc = sysfs_create_file(firmware_kobj, &modem_detect_attr.attr);
		if (rc != 0) {
			pr_err("%s: Error creating sysfs file entry\n",
					__func__);
			return rc;
		}

	} else {
		rc = -EINVAL;
		pr_err("%s: Unable to find platform data\n", __func__);
	}
	return rc;
}

static int bcm_modem_detect_pltfm_remove(struct platform_device *pdev)
{
	/*Unused */
	return 0;
}

static struct platform_driver bcm_modem_detect_pltfm_driver = {
	.driver = {
		.name = "bcm_modem_detect",
		.owner = THIS_MODULE,
	},
	.probe = bcm_modem_detect_pltfm_probe,
	.remove = __devexit_p(bcm_modem_detect_pltfm_remove),
};

int __init modem_detect_init(void)
{
	int rc = 0;

	rc = platform_driver_register(&bcm_modem_detect_pltfm_driver);
	if (rc != 0)
		pr_err("%s: Error registering modem detect platform driver\n",
				__func__);

	return rc;
}

arch_initcall(modem_detect_init);

