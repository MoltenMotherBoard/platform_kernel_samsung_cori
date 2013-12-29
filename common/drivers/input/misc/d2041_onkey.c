/*
 * d2041_onkey.c: ON Key support for Dialog D2041
 *   
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 *  
 * Author: Dialog Semiconductor Ltd. D. Chen, D. Patel
 *  
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
 
#include <linux/module.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
 
#include <linux/d2041/d2041_reg.h> 
#include <linux/d2041/core.h>

#define DRIVER_NAME "d2041-onkey"

/* DLG start */
static int powerkey_pressed;

int d2041_onkey_check(void)
{
	return powerkey_pressed;
}
EXPORT_SYMBOL(d2041_onkey_check);
/* DLG end */

static irqreturn_t d2041_onkey_event_lo_handler(int irq, void *data)
{
	struct d2041 *d2041 = data;
	struct d2041_onkey *dlg_onkey = &d2041->onkey;

    	dev_info(d2041->dev, "Onkey LO Interrupt Event generated\n");
	input_report_key(dlg_onkey->input, KEY_POWER, 1);
	input_sync(dlg_onkey->input);
	/* DLG start */
	powerkey_pressed = 1;	
	/* DLG end */

	return IRQ_HANDLED;
} 

static irqreturn_t d2041_onkey_event_hi_handler(int irq, void *data)
{
	struct d2041 *d2041 = data;
	struct d2041_onkey *dlg_onkey = &d2041->onkey;

    	dev_info(d2041->dev, "Onkey HI Interrupt Event generated\n");
	input_report_key(dlg_onkey->input, KEY_POWER, 0);
	input_sync(dlg_onkey->input);
	
	/* DLG start */
	powerkey_pressed = 0;
	/* DLG end */
	return IRQ_HANDLED;
} 

static int __devinit d2041_onkey_probe(struct platform_device *pdev)
{
	struct d2041 *d2041 = platform_get_drvdata(pdev);
	struct d2041_onkey *dlg_onkey = &d2041->onkey;
	int ret = 0;
	
	
        dev_info(d2041->dev, "Starting Onkey Driver\n");

        dlg_onkey->input = input_allocate_device();
        if (!dlg_onkey->input) {
		dev_err(&pdev->dev, "failed to allocate data device\n");
		return -ENOMEM;
	}
        
	dlg_onkey->input->evbit[0] = BIT_MASK(EV_KEY);
	dlg_onkey->input->keybit[BIT_WORD(KEY_POWER)] = BIT_MASK(KEY_POWER);
	dlg_onkey->input->name = DRIVER_NAME;
	dlg_onkey->input->phys = "d2041-onkey/input0";
	dlg_onkey->input->dev.parent = &pdev->dev;

	ret = input_register_device(dlg_onkey->input);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register input device,error: %d\n", ret);
		input_free_device(dlg_onkey->input);
		return ret;
	}

    	d2041_register_irq(d2041, D2041_IRQ_ENONKEY_HI, d2041_onkey_event_hi_handler, 
                            0, DRIVER_NAME, d2041);
    	d2041_register_irq(d2041, D2041_IRQ_ENONKEY_LO, d2041_onkey_event_lo_handler, 
                            0, DRIVER_NAME, d2041);                         
	dev_info(d2041->dev, "Onkey Driver registered\n");
	return 0;

}

static int __devexit d2041_onkey_remove(struct platform_device *pdev)
{
	struct d2041 *d2041 = platform_get_drvdata(pdev);
	struct d2041_onkey *dlg_onkey = &d2041->onkey;

	d2041_free_irq(d2041, D2041_IRQ_ENONKEY_LO);
	d2041_free_irq(d2041, D2041_IRQ_ENONKEY_HI);
	input_unregister_device(dlg_onkey->input);
	return 0;
}   

static struct platform_driver d2041_onkey_driver = {
	.probe		= d2041_onkey_probe,
	.remove		= __devexit_p(d2041_onkey_remove),
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	}
};

static int __init d2041_onkey_init(void)
{
	return platform_driver_register(&d2041_onkey_driver);
}

static void __exit d2041_onkey_exit(void)
{
	platform_driver_unregister(&d2041_onkey_driver);
}

module_init(d2041_onkey_init);
module_exit(d2041_onkey_exit);   

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("Onkey driver for the Dialog D2041 PMIC");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
