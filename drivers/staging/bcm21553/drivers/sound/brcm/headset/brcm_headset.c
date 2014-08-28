/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
* 
* 	@file	drivers/sound/brcm/headset/brcm_headset.c
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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <mach/irqs.h>
#include <linux/irq.h>
#include <plat/brcm_headset_pd.h>
#include <plat/syscfg.h>
#include "brcm_headset_hw.h"
#include <mach/gpio.h>
#include <linux/switch.h>
#include <linux/wakelock.h>

#define VALID_RELEASE_REF_TIME 60000000 /* VALID_RELEASE_REF_TIME + Chip Debounce >= 250ms,  ex)  VALID_RELEASE_REF_TIME(140ms) + Chip Debounce = 290ms*/
#define KEY_INTERRUPT_REF_TIME 350000000 /* 350ms */
#define KEY_BEFORE_PRESS_REF_TIME msecs_to_jiffies(50)
#define KEY_PRESS_REF_TIME msecs_to_jiffies(5)
#define TYPE_DETECT_REF_TIME msecs_to_jiffies(100)
#define HEADSET_DETECT_REF_COUNT  10
#define GET_IMSI_REF_TIME	(HZ * 8)  /* 8 sec */
#define WAKE_LOCK_TIME		(HZ * 5)	/* 5 sec */
#define TEST_SIM_IMSI	"999999999999999"

#define REG_ANACR12 0x088800B0
#define REG_INTC_ICR0 0x8810008
#define REG_INTC_IMR0 0x08810000
#define INTC_IRQ12_BIT (1<<12)
#define CLEAR_IRQ_MICON writel((readl(io_p2v(REG_INTC_ICR0)) | INTC_IRQ12_BIT) , io_p2v(REG_INTC_ICR0))
#define ENABLE_IRQ_MICON CLEAR_IRQ_MICON; \
						    writel((readl(io_p2v(REG_INTC_IMR0)) | INTC_IRQ12_BIT) , io_p2v(REG_INTC_IMR0))
#define DISABLE_IRQ_MICON writel((readl(io_p2v(REG_INTC_IMR0)) & ~INTC_IRQ12_BIT) , io_p2v(REG_INTC_IMR0))

#define HEADSET_4_POLE	1
#define HEADSET_3_POLE 	2
#define INIT 3
#define NONE 2
#define PRESS 1
#define RELEASE 0
#define ENABLE 1
#define DISABLE 0

static short KEY_PRESS_THRESHOLD;
static short KEY_3POLE_THRESHOLD;
static short KEY1_THRESHOLD_L;
static short KEY1_THRESHOLD_U;
static short KEY2_THRESHOLD_L;
static short KEY2_THRESHOLD_U;
static short KEY3_THRESHOLD_L;
static short KEY3_THRESHOLD_U;

static unsigned char FactoryMode = DISABLE;

extern unsigned char sync_use_mic; /* Synchronization between audio and headset for MIC bias */
extern void set_button(int value);
extern int auxadc_access(int);
extern int bcm_gpio_set_db_val(unsigned int gpio, unsigned int db_val);
extern SIMLOCK_SIM_DATA_t* GetSIMData(void);

static void determine_state_func(void);
static void type_work_func(struct work_struct *work);
static void input_work_func(struct work_struct *work);

struct mic_t
{
	unsigned char pluging;
	unsigned char keypressing;
	unsigned char key_count[3];
	int headset_state;
	ktime_t hsbtime;
	struct switch_dev sdev;
	struct input_dev* headset_button_idev;
	struct workqueue_struct* headset_workqueue;
	struct delayed_work input_work;
	struct delayed_work type_work;
	struct delayed_work imsi_work;	
	struct wake_lock det_wake_lock;
	struct brcm_headset_pd* headset_pd;
};

static struct mic_t mic;

static inline int Return_valid_key(unsigned char keyarr[3])
{
	if ( keyarr[0] >= keyarr[1] && keyarr[0] >= keyarr[2] )
		return KEY_BCM_HEADSET_BUTTON;
	else if ( keyarr[1] >= keyarr[0] && keyarr[1] >= keyarr[2] )
		return KEY_VOLUMEUP;
	else
		return KEY_VOLUMEDOWN;
}

static void input_work_func(struct work_struct *work)
{
	int adc_value = -1;
	int val = 0;
	ktime_t temptime;

	adc_value = auxadc_access(2);
	val = readl(io_p2v(REG_ANACR12));
//	printk("%s: REG_ANACR2=%d, REG_ANACR12=%d\n", __func__,adc_value , val);

	if(val >= KEY_PRESS_THRESHOLD && adc_value >= KEY1_THRESHOLD_L && adc_value < KEY3_THRESHOLD_U)
	{
		temptime = ktime_get();
		temptime = ktime_sub(temptime, mic.hsbtime);

		if(temptime.tv.nsec < VALID_RELEASE_REF_TIME && mic.keypressing == PRESS)
		{
			if ( adc_value >= KEY1_THRESHOLD_L && adc_value < KEY1_THRESHOLD_U )
			{
				mic.key_count[0]++;
				printk ("KEY_BCM_HEADSET_BUTTON \n");
			}
			else if ( adc_value >= KEY2_THRESHOLD_L && adc_value < KEY2_THRESHOLD_U ) 
			{
				mic.key_count[1]++;
				printk ("KEY_VOLUMEUP \n");
			}
			else if ( adc_value >= KEY3_THRESHOLD_L && adc_value < KEY3_THRESHOLD_U ) 
			{
				mic.key_count[2]++;
				printk ("KEY_VOLUMEDOWN \n");
			}
		}
		else
		{
			if(mic.keypressing == PRESS && (mic.key_count[0] + mic.key_count[1] + mic.key_count[2]))
			{
				input_report_key(mic.headset_button_idev, Return_valid_key(mic.key_count), PRESS);
				input_sync(mic.headset_button_idev);

				set_button(1); 
				mic.keypressing = RELEASE;
			}
		}

		cancel_delayed_work(&(mic.input_work));
		queue_delayed_work(mic.headset_workqueue, &(mic.input_work), KEY_PRESS_REF_TIME);
	}
	else
	{
		if(mic.keypressing == RELEASE && (mic.key_count[0] + mic.key_count[1] + mic.key_count[2]))
		{			
			printk ("%s: RELEASE key_count [%d, %d, %d] \n", __func__,  mic.key_count[0], mic.key_count[1], mic.key_count[2]);
			input_report_key(mic.headset_button_idev, Return_valid_key(mic.key_count), RELEASE);
			input_sync(mic.headset_button_idev);
		}
		else
		{
			printk("%s: NO PRESS\n",  __func__);
		}

		if(FactoryMode == DISABLE)
		{
			board_sysconfig(SYSCFG_AUXMIC, SYSCFG_ENABLE | SYSCFG_DISABLE);
			sync_use_mic = DISABLE;		
		}
		
		set_button(0); 
		mic.keypressing = NONE;
	}
}

/*------------------------------------------------------------------------------
Function name   : hs_buttonisr
Description     : interrupt handler

Return type     : irqreturn_t
------------------------------------------------------------------------------*/
irqreturn_t hs_buttonisr(int irq, void *dev_id)
{
	struct mic_t *p = &mic;
	int val = 0;
	ktime_t temptime;
	
	if(mic.keypressing == INIT)
	{
		temptime = ktime_get();
		temptime = ktime_sub(temptime, mic.hsbtime);
		if(temptime.tv.sec >= 1 || temptime.tv.nsec >= KEY_INTERRUPT_REF_TIME)
			mic.keypressing = NONE;
		else
		{
		 	printk("%s: Initializing HSB ISR\n", __func__ );
			return IRQ_NONE;
		}
	}	

	if(p->pluging ==  ENABLE || p->keypressing != NONE)
	{
		printk("%s: Headset pluging OR keypressing\n", __func__ );
		return IRQ_NONE;
	}

	val = readl(io_p2v(REG_ANACR12));
	if(val < KEY_PRESS_THRESHOLD)
	{
		printk("%s: False button interrupt\n", __func__ );
		return IRQ_NONE;	
	}
	
	if (p->headset_state == HEADSET_4_POLE)
	{	
		p->hsbtime = ktime_get();
		
		board_sysconfig(SYSCFG_AUXMIC, SYSCFG_ENABLE);
		
		memset(mic.key_count, 0, sizeof(mic.key_count));
		p->keypressing = PRESS;
		sync_use_mic = ENABLE;

		cancel_delayed_work(&(mic.input_work));
		queue_delayed_work(mic.headset_workqueue, &(p->input_work), KEY_BEFORE_PRESS_REF_TIME);
	}

	 return IRQ_HANDLED;
}

/* 	1 : SIM_DUAL_FIRST,
	2 : SIM_DUAL_SECOND */
static void getIMSI_work_func(struct work_struct *work)
{
	SIMLOCK_SIM_DATA_t* simdata = GetSIMData();
	
	if(simdata == NULL)
	{
		FactoryMode = DISABLE;
	}
	else
	{
		FactoryMode = strncmp(simdata->imsi_string, TEST_SIM_IMSI, IMSI_DIGITS) == 0 ?  ENABLE : DISABLE;
	}
	
	if(FactoryMode == ENABLE)
	{
		if(mic.headset_state == HEADSET_4_POLE)
			board_sysconfig(SYSCFG_AUXMIC, SYSCFG_ENABLE);
	}
}

static void determine_state_func(void)
{
	if(mic.headset_state)
	{
		board_sysconfig(SYSCFG_HEADSET, SYSCFG_ENABLE);
		board_sysconfig(SYSCFG_AUXMIC, SYSCFG_ENABLE);

		cancel_delayed_work(&(mic.type_work));
		queue_delayed_work(mic.headset_workqueue, &(mic.type_work), TYPE_DETECT_REF_TIME);
	}
	else
	{
		board_sysconfig(SYSCFG_HEADSET, SYSCFG_DISABLE);
		board_sysconfig(SYSCFG_AUXMIC, SYSCFG_DISABLE);
		
		DISABLE_IRQ_MICON;

		mic.pluging = DISABLE;
		mic.keypressing = INIT;
		sync_use_mic = DISABLE;		
		switch_set_state(&mic.sdev, mic.headset_state);
		printk("%s: plugged out\n", __func__);
	}
}

static void type_work_func(struct work_struct *work)
{
	int adc = 0;

	if(mic.headset_state)
	{
		adc = auxadc_access(2);
		if(adc >= KEY_3POLE_THRESHOLD)
		{
			if(FactoryMode == DISABLE)
				board_sysconfig(SYSCFG_AUXMIC, SYSCFG_ENABLE | SYSCFG_DISABLE);

			board_sysconfig(SYSCFG_HEADSET, SYSCFG_ENABLE);
			
			mic.headset_state = HEADSET_4_POLE;
			mic.hsbtime = ktime_get();
			ENABLE_IRQ_MICON;
			printk("%s: 4-pole inserted : ear_adc=%d\n", __func__, adc);
		}
		else
		{
			if(FactoryMode == DISABLE)
				board_sysconfig(SYSCFG_AUXMIC, SYSCFG_DISABLE);
			
			mic.headset_state = HEADSET_3_POLE;
			printk("%s: 3-pole inserted : ear_adc=%d\n", __func__, adc);
		}

		mic.pluging = DISABLE;

		if(FactoryMode == DISABLE)
		sync_use_mic = DISABLE;

		switch_set_state(&mic.sdev, mic.headset_state);
	}
}

/*------------------------------------------------------------------------------
Function name   : hs_isr
Description     : interrupt handler

Return type     : irqreturn_t
------------------------------------------------------------------------------*/
irqreturn_t hs_isr(int irq, void *dev_id)
{
	struct mic_t *p = &mic;
	int pre_data = -1;
	int loopcnt = 0;

	wake_lock_timeout(&p->det_wake_lock, WAKE_LOCK_TIME);
	p->pluging = ENABLE;
	sync_use_mic = ENABLE;

	printk("%s: Before state : %d \n", __func__, p->headset_state);

#if 0
	/* For remove pop-up noise.*/
	if(p->headset_state && (p->keypressing == NONE || p->keypressing == INIT))
	{		
		printk("%s: Remove popup noise\n", __func__);
		board_sysconfig(SYSCFG_HEADSET, SYSCFG_DISABLE);
		board_sysconfig(SYSCFG_AUXMIC, SYSCFG_DISABLE);
	}
#endif
	
	/* debounce headset jack.  don't try to determine the type of
	 * headset until the detect state is true for a while.
	 */
	while (1)
	{
		p->headset_pd->check_hs_state(&(p->headset_state));
		if (pre_data == p->headset_state)
			loopcnt++;
		else
			loopcnt = 0;

		pre_data = p->headset_state;

		if (loopcnt >= HEADSET_DETECT_REF_COUNT)
			break;
		
		msleep(30);
	}

	set_irq_type(p->headset_pd->hsirq, (p->headset_state) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING);
	determine_state_func();

	return IRQ_HANDLED;
}

/*------------------------------------------------------------------------------
Function name   : hs_switchinit
Description     : Register sysfs device for headset
It uses class switch from kernel/common/driver/switch
Return type     : int
------------------------------------------------------------------------------*/
int hs_switchinit(struct mic_t *p)
{
	int result = 0;
	p->sdev.name = "h2w";

	result = switch_dev_register(&p->sdev);
	if (result < 0)
	{
		printk("%s: Failed to register device\n", __func__);
		return result;
	}
	
	INIT_DELAYED_WORK(&(p->type_work), type_work_func);
	
	return 0;
} 

/*------------------------------------------------------------------------------
Function name   : hs_inputdev
Description     : Create and Register input device for headset button
Return type     : int
------------------------------------------------------------------------------*/
int  hs_inputdev(struct mic_t *p)
{
	int result = 0;

	// Allocate struct for input device
	p->headset_button_idev = input_allocate_device();
	if (p->headset_button_idev == NULL)
	{
		printk("%s: headset button: Not enough memory\n", __func__);
		return -1;
	}

	// specify key event type and value for it
	// we have only one button on headset so only one possible
	// value KEY_SEND used here.
	set_bit(EV_KEY, p->headset_button_idev->evbit);
	input_set_capability(p->headset_button_idev, EV_KEY, KEY_BCM_HEADSET_BUTTON);
	input_set_capability(p->headset_button_idev, EV_KEY, KEY_VOLUMEUP);
	input_set_capability(p->headset_button_idev, EV_KEY, KEY_VOLUMEDOWN);

	p->headset_button_idev->name = "bcm_headset";
	p->headset_button_idev->phys = "headset/input0";
	p->headset_button_idev->id.bustype = BUS_HOST;
	p->headset_button_idev->id.vendor = 0x0001;
	p->headset_button_idev->id.product = 0x0100;

	// Register input device for headset
	result = input_register_device(p->headset_button_idev);
	
	if (result) {
		printk("%s: Failed to register device\n", __func__);
		return result;
	}

	INIT_DELAYED_WORK(&(mic.input_work), input_work_func);

	return 0;
}

static int hs_remove(struct platform_device *pdev)
{
	if (mic.headset_pd)
	{
		free_irq(mic.headset_pd->hsirq, NULL);
		free_irq(mic.headset_pd->hsbirq, NULL);
	}
		
	destroy_workqueue(mic.headset_workqueue);	
	
	input_unregister_device(mic.headset_button_idev);
	switch_dev_unregister(&mic.sdev);	
	return 0;
}

static int hs_suspend(struct platform_device *pdev, pm_message_t state)
{
	//Clear IRQ12
	CLEAR_IRQ_MICON;
	printk("%s: Clear HSB irq12 \n", __func__);	
	return 0;
}

static int __init hs_probe(struct platform_device *pdev)
{
	int result = 0;
	mic.pluging = DISABLE;
	mic.keypressing = INIT;

	result = hs_switchinit(&mic);
	if (result < 0)
		return result;

	result = hs_inputdev(&mic);
	if (result < 0)
		goto err;

	INIT_DELAYED_WORK(&(mic.imsi_work), getIMSI_work_func);
	wake_lock_init(&mic.det_wake_lock, WAKE_LOCK_SUSPEND, "brcm_headset_det");
	mic.headset_workqueue = create_singlethread_workqueue("brcm_headset_wq");
	if (mic.headset_workqueue == NULL) {
		printk("%s: Failed to create workqueue\n", __func__);
		goto err1;
	}

	/* check if platform data is defined for a particular board variant */
	if (pdev->dev.platform_data)
	{
		mic.headset_pd = pdev->dev.platform_data;

		KEY_PRESS_THRESHOLD = mic.headset_pd->key_press_threshold;
		KEY_3POLE_THRESHOLD = mic.headset_pd->key_3pole_threshold;
		KEY1_THRESHOLD_L = mic.headset_pd->key1_threshold_l;
		KEY1_THRESHOLD_U = mic.headset_pd->key1_threshold_u;
		KEY2_THRESHOLD_L = mic.headset_pd->key2_threshold_l;
		KEY2_THRESHOLD_U = mic.headset_pd->key2_threshold_u;
		KEY3_THRESHOLD_L = mic.headset_pd->key3_threshold_l;
		KEY3_THRESHOLD_U = mic.headset_pd->key3_threshold_u;
		
		if (gpio_request(mic.headset_pd->hsgpio, "headset detect") < 0)
		{
			printk("%s: Could not reserve headset signal GPIO!\n", __func__);
			goto err2;
		}
		
		gpio_direction_input(mic.headset_pd->hsgpio);
		bcm_gpio_set_db_val(mic.headset_pd->hsgpio, 0x7);
		mic.headset_pd->hsirq = gpio_to_irq(mic.headset_pd->hsgpio);		
	}
    	else
	{
		goto err2;
	}

	/* Set the ANACR2 bit for mic power down */
	board_sysconfig(SYSCFG_AUXMIC, SYSCFG_INIT);
	board_sysconfig(SYSCFG_HEADSET, SYSCFG_INIT);

	/*Fix the audio path is wrong when headset already plugged in the device  then boot device case.*/
	mic.headset_pd->check_hs_state(&mic.headset_state);
	set_irq_type(mic.headset_pd->hsirq, (mic.headset_state) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING);
	determine_state_func();	
	queue_delayed_work(mic.headset_workqueue, &(mic.imsi_work), GET_IMSI_REF_TIME);

	result = request_threaded_irq(mic.headset_pd->hsbirq, NULL, hs_buttonisr,  (IRQF_ONESHOT |IRQF_NO_SUSPEND), "BrcmHeadsetButton",  NULL);
	if(result < 0)
		goto err2;

	DISABLE_IRQ_MICON;

	result = request_threaded_irq(mic.headset_pd->hsirq, NULL, hs_isr, (IRQF_ONESHOT|IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND), "BrcmHeadset",  NULL);
	if(result < 0)
	{
		free_irq(mic.headset_pd->hsbirq, &mic);
		goto err2;
	}
	enable_irq_wake(mic.headset_pd->hsirq);
	set_irq_type(mic.headset_pd->hsirq, (mic.headset_state) ? IRQF_TRIGGER_RISING : IRQF_TRIGGER_FALLING);

	return 0;

err2:  destroy_workqueue(mic.headset_workqueue);	
err1:  input_unregister_device(mic.headset_button_idev);
err: switch_dev_unregister(&mic.sdev);
	return result;
}

static struct platform_driver headset_driver = {
	.probe = hs_probe,
	.remove = hs_remove,
	.suspend = hs_suspend,
	.driver = {
		.name = "bcmheadset",
		.owner = THIS_MODULE,
	},
};

/*------------------------------------------------------------------------------
Function name   : BrcmHeadsetModuleInit
Description     : Initialize the driver

Return type     : int
------------------------------------------------------------------------------*/
int __init BrcmHeadsetModuleInit(void)
{
	return platform_driver_register(&headset_driver);
}

/*------------------------------------------------------------------------------
Function name   : BrcmHeadsetModuleExit
Description     : clean up

Return type     : int
------------------------------------------------------------------------------*/
void __exit BrcmHeadsetModuleExit(void)
{
	return platform_driver_unregister(&headset_driver);
}

module_init(BrcmHeadsetModuleInit);
module_exit(BrcmHeadsetModuleExit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Headset plug and button detection");
