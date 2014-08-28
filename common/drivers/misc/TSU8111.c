/*
 * For TI 8111 micro usb IC
 *
 * Copyright (C) 2009 Samsung Electronics
 * Wonguk Jeong <wonguk.jeong@samsung.com>
 * Minkyu Kang <mk7.kang@samsung.com>
 *
 * Modified by Sumeet Pawnikar <sumeet.p@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mfd/tsu8111/TSU8111.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
//#include <plat/microusbic.h>
//#include <plat/mfp.h>
//#include <plat/vbus.h>

#include <linux/mfd/88pm860x.h>
#include <linux/wakelock.h>

#if defined(CONFIG_SPA)
#include <mach/spa.h>
static void (*spa_external_event)(int, int) = NULL;
#endif

#include <linux/delay.h>
#include <linux/mfd/max8986/max8986.h>


/* TSU8111 I2C registers */
#define TSU8111_REG_DEVID              		0x01
#define TSU8111_REG_CTRL               		0x02
#define TSU8111_REG_INT1               		0x03
#define TSU8111_REG_INT2               		0x04
#define TSU8111_REG_INT1_MASK          	0x05
#define TSU8111_REG_INT2_MASK          	0x06
#define TSU8111_REG_ADC                        	0x07
#define TSU8111_REG_TIMING1            		0x08
#define TSU8111_REG_TIMING2            		0x09
#define TSU8111_REG_DEV_T1             		0x0a
#define TSU8111_REG_DEV_T2             		0x0b
#define TSU8111_REG_BTN1               		0x0c
#define TSU8111_REG_BTN2               		0x0d
#define TSU8111_REG_MANSW1             	0x13
#define TSU8111_REG_MANSW2             	0x14
#define TSU8111_REG_CHGCTRL1			0x20
#define TSU8111_REG_CHGCTRL2			0x21
#define TSU8111_REG_CHGCTRL3			0x22
#define TSU8111_REG_CHGCTRL4			0x23
#define TSU8111_REG_CHG_INT			0x24
#define TSU8111_REG_CHG_INT_MASK		0x25
#define TSU8111_REG_CHG_STATUS		0x26


/* MANSW1 */
#define VAUDIO                 0x90
#define UART                   0x6c
#define AUDIO                  0x48
#define DHOST                  0x24
#define AUTO                   0x0

/*TSU8111 MANSW1*/
#define VAUDIO_9485            0x93
#define AUDIO_9485             0x4B
#define DHOST_9485             0x27

/* MANSW2 */
#define MANSW2_JIG		(1 << 2)

/* Control */
#define SWITCH_OPEN            (1 << 4)
#define RAW_DATA               (1 << 3)
#define MANUAL_SWITCH          (1 << 2)
#define WAIT                   (1 << 1)
#define INT_MASK               (1 << 0)
#define CTRL_MASK              (SWITCH_OPEN | RAW_DATA | MANUAL_SWITCH | WAIT | INT_MASK)
/* Device Type 1*/
#define DEV_USB_OTG            (1 << 7)
#define DEV_DEDICATED_CHG      (1 << 6)
#define DEV_USB_CHG            (1 << 5)
#define DEV_CAR_KIT            (1 << 4)
#define DEV_UART               (1 << 3)
#define DEV_USB                        (1 << 2)
#define DEV_AUDIO_2            (1 << 1)
#define DEV_AUDIO_1            (1 << 0)

#define TSU8111_DEV_T1_HOST_MASK               (DEV_USB_OTG)
#define TSU8111_DEV_T1_USB_MASK                (DEV_USB | DEV_USB_CHG)
#define TSU8111_DEV_T1_UART_MASK       (DEV_UART)
#define TSU8111_DEV_T1_CHARGER_MASK    (DEV_DEDICATED_CHG | DEV_CAR_KIT)
#define TSU8111_DEV_T1_AUDIO_MASK    (DEV_AUDIO_1 | DEV_AUDIO_2)

/* Device Type 2*/
#define DEV_AV                 (1 << 6)
#define DEV_TTY                        (1 << 5)
#define DEV_PPD                        (1 << 4)
#define DEV_JIG_UART_OFF       (1 << 3)
#define DEV_JIG_UART_ON                (1 << 2)
#define DEV_JIG_USB_OFF                (1 << 1)
#define DEV_JIG_USB_ON         (1 << 0)

#define TSU8111_DEV_T2_USB_MASK                (DEV_JIG_USB_OFF | DEV_JIG_USB_ON)
#define TSU8111_DEV_T2_UART_MASK       (DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

#define TSU8111_DEV_T2_JIG_MASK                (/*DEV_JIG_USB_OFF |*/ DEV_JIG_USB_ON | DEV_JIG_UART_OFF | DEV_JIG_UART_ON)

#define DEV_MHL                 (DEV_AV)
#define TSU8111_DEV_T2_MHL_MASK         (DEV_MHL)

#if defined(CONFIG_BOARD_ZANIN)
#define MUSB_IRQ_GPIO 	14
#define MUSB_SDA 		15
#define MUSB_SCL		7
#else
#define MUSB_IRQ_GPIO 	11
#define MUSB_SDA 		12
#define MUSB_SCL		13
#endif

struct muic_event_handler
{
	void (*handler) (int, u32, void *);
	void* data;
};
struct fsa880_muic {
	struct max8986 *max8986;

	/*MUIC state variables */
	u8 jig_status;
	u8 adc_low_status;
	u8 dcd_timeout_status; /*DCD - data contact detect*/
	u8 charger_detect_status;
	u8 dbc_status; /*Dead Batt Charging status */

	u8 charger_type;
	u8 mic_pluggedin;
	u8 mic_btn_pressed;
	u8 mic_insert_adc_val;
	u8 mic_btn_press_adc_val;
	struct muic_event_handler handler[MAX8986_MUIC_EVENT_MAX];
};


#define CALL_EVENT_HANDLER(event,param) if(fsa880_muic->handler[(event)].handler)\
											fsa880_muic->handler[(event)].handler((event),\
												(param),\
												fsa880_muic->handler[(event)].data)


static struct fsa880_muic* fsa880_muic_ptr = NULL;
#if 0
static void tsu8111_read_adc_value(void)
{
	u8 adc=0;
    struct tsu8111_usbsw *usbsw = chip;
    struct i2c_client *client = usbsw->client;
	
	tsu8111_read_reg(client, TSU8111_REG_ADC, &adc);
	printk("[TSU8111] %s: adc is 0x%x\n",__func__,adc);
}
#endif
void fsa880_muic_force_charger_detection()
{

}
EXPORT_SYMBOL(fsa880_muic_force_charger_detection);

int fsa880_muic_register_event_handler(int event,void (*handler) (int, u32, void *),
					void* data)
{
	if(fsa880_muic_ptr)
	{
		if(fsa880_muic_ptr->handler[event].handler == NULL)
		{
			fsa880_muic_ptr->handler[event].handler = handler;
			fsa880_muic_ptr->handler[event].data = data;
		}
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(fsa880_muic_register_event_handler);

extern void fsa880_muic_unregister_event_handler(int event)
{
	if(fsa880_muic_ptr)
	{
		fsa880_muic_ptr->handler[event].handler = NULL;
		fsa880_muic_ptr->handler[event].data = NULL;
	}
}
EXPORT_SYMBOL(fsa880_muic_unregister_event_handler);

pmu_muic_chgtyp fsa880_muic_get_charger_type()
{
	if(fsa880_muic_ptr)
	{
		printk(KERN_INFO "%s:type = %d\n",__func__,fsa880_muic_ptr->charger_type);
		return 	fsa880_muic_ptr->charger_type;
	}
	return PMU_MUIC_CHGTYP_NONE;
}
EXPORT_SYMBOL(fsa880_muic_get_charger_type);

int fsa880_muic_get_jig_status()
{
	if(fsa880_muic_ptr)
	{
		printk(KERN_INFO "%s:jig = %s\n",__func__,(fsa880_muic_ptr->jig_status ? "Inserted":"removed"));
		return 	fsa880_muic_ptr->jig_status;
	}
	return fsa880_muic_ptr->jig_status;
}
EXPORT_SYMBOL(fsa880_muic_get_jig_status);


struct tsu8111_usbsw {
       struct i2c_client               *client;
       struct tsu8111_platform_data    *pdata;
       struct work_struct              work;
       int                             dev1;
       int                             dev2;
       int                             mansw;
	   u8                              id;
};

static struct tsu8111_usbsw *chip;
static struct wake_lock JIGConnect_idle_wake;
static struct wake_lock JIGConnect_suspend_wake;


#ifdef CONFIG_VIDEO_MHL_V1
/*for MHL cable insertion*/
static int isMHLconnected=0;
#endif
static int isDeskdockconnected=0;

static int isProbe=0;
enum {
	muicTypeTI8111 = 1,
	muicTypeFSA880 = 2,
	muicTypeFSA = 3,
};
static int muic_type=0;
static int isManual=0;

static int tsu8111_write_reg(struct i2c_client *client,        u8 reg, u8 data)
{
       int ret = 0;
       u8 buf[2];
       struct i2c_msg msg[1];

       buf[0] = reg;
       buf[1] = data;

       msg[0].addr = client->addr;
       msg[0].flags = 0;
       msg[0].len = 2;
       msg[0].buf = buf;

	   printk("[TSU8111] tsu8111_write_reg   reg[0x%2x] data[0x%2x]\n",buf[0],buf[1]);

       ret = i2c_transfer(client->adapter, msg, 1);
       if (ret != 1) {
               printk("\n [TSU8111] i2c Write Failed (ret=%d) \n", ret);
               return -1;
       }
       
       return ret;
}

static int tsu8111_read_reg(struct i2c_client *client, u8 reg, u8 *data)
{
       int ret = 0;
       u8 buf[1];
       struct i2c_msg msg[2];

       buf[0] = reg;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = buf;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = buf;
		
	printk("[TSU8111] tsu8111_read_reg reg[0x%2x] ", buf[0]);

       ret = i2c_transfer(client->adapter, msg, 2);
       if (ret != 2) {
               printk("\n [TSU8111] i2c Read Failed (ret=%d) \n", ret);
               return -1;
       }
       *data = buf[0];

      printk(" data [0x%2x]   i2c Read success\n",buf[0]);
       return 0;
}

static void tsu8111_read_adc_value(void)
{
	u8 adc=0;
    struct tsu8111_usbsw *usbsw = chip;
    struct i2c_client *client = usbsw->client;
	
	tsu8111_read_reg(client, TSU8111_REG_ADC, &adc);
	printk("[TSU8111] %s: adc is 0x%x\n",__func__,adc);
}


static void tsu8111_id_open(void)
{
	struct tsu8111_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;

	pr_alert("tsu8111 id_open\n");
	tsu8111_write_reg(client, 0x1B, 1);
}

void tsu8111_set_switch(const char *buf)
{
       struct tsu8111_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
       u8 value = 0;
       unsigned int path = 0;

       tsu8111_read_reg(client, TSU8111_REG_CTRL, &value);

       if (!strncmp(buf, "VAUDIO", 6)) {
	   	       if(usbsw->id == 0)
			   	path = VAUDIO_9485;
			   else
			   	path = VAUDIO;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "UART", 4)) {
               path = UART;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "AUDIO", 5)) {
               if(usbsw->id == 0)
			   	path = AUDIO_9485;
			   else
                path = AUDIO;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "DHOST", 5)) {
               path = DHOST;
               value &= ~MANUAL_SWITCH;
       } else if (!strncmp(buf, "AUTO", 4)) {
               path = AUTO;
               value |= MANUAL_SWITCH;
       } else {
               printk(KERN_ERR "Wrong command\n");
               return;
       }

       usbsw->mansw = path;
       tsu8111_write_reg(client, TSU8111_REG_MANSW1, path);
       tsu8111_write_reg(client, TSU8111_REG_CTRL, value);
}
EXPORT_SYMBOL_GPL(tsu8111_set_switch);

ssize_t tsu8111_get_switch(char *buf)
{
struct tsu8111_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
       u8 value;

       tsu8111_read_reg(client, TSU8111_REG_MANSW1, &value);

       if (value == VAUDIO)
               return sprintf(buf, "VAUDIO\n");
       else if (value == UART)
               return sprintf(buf, "UART\n");
       else if (value == AUDIO)
               return sprintf(buf, "AUDIO\n");
       else if (value == DHOST)
               return sprintf(buf, "DHOST\n");
       else if (value == AUTO)
               return sprintf(buf, "AUTO\n");
       else
               return sprintf(buf, "%x", value);
}
EXPORT_SYMBOL_GPL(tsu8111_get_switch);


#ifdef CONFIG_VIDEO_MHL_V1
static void DisableTSU8111Interrupts(void)
{
       struct tsu8111_usbsw *usbsw = chip;
       struct i2c_client *client = usbsw->client;
        printk ("DisableTSU8111Interrupts-2\n");

     tsu8111_write_reg(client, TSU8111_REG_INT1_MASK, 0xFF);
     tsu8111_write_reg(client, TSU8111_REG_INT2_MASK, 0x1F);
	 
} 

static void EnableTSU8111Interrupts(void)
{
	struct tsu8111_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 intr, intr2;

	printk ("Enable TSU8111 Interrupts\n");

     /*clear interrupts*/
     tsu8111_read_reg(client, TSU8111_REG_INT1, &intr);
     tsu8111_read_reg(client, TSU8111_REG_INT2, &intr2);
	 
     tsu8111_write_reg(client, TSU8111_REG_INT1_MASK, 0x00);
     tsu8111_write_reg(client, TSU8111_REG_INT2_MASK, 0x00);

} 
void TSU8111_EnableIntrruptByMHL(bool _bDo)
{
	struct tsu8111_platform_data *pdata = chip->pdata;
	struct i2c_client *client = chip->client;
	char buf[16];

	if(true == _bDo)
	{
		tsu8111_write_reg(client,TSU8111_REG_CTRL, 0x1E);
		EnableTSU8111Interrupts();
	}
	else
	{
		DisableTSU8111Interrupts();
	}

	tsu8111_get_switch(buf);
	printk("[%s] switch status = %s\n",__func__, buf);
}

/*MHL call this function to change VAUDIO path*/
void TSU8111_CheckAndHookAudioDock(void)
{
   struct tsu8111_platform_data *pdata = chip->pdata;
   struct i2c_client *client = chip->client;

   printk("[TSU8111] %s: TSU8111 VAUDIO\n",__func__);
   
   isMHLconnected = 0;
   isDeskdockconnected = 1;
   
   if (pdata->mhl_cb)
   	       pdata->mhl_cb(TSU8111_DETACHED);

   EnableTSU8111Interrupts();

   if(chip->id == 0)
	chip->mansw = VAUDIO_9485;
   else
	chip->mansw = VAUDIO;

   /*make ID change report*/
   tsu8111_write_reg(client,TSU8111_REG_CTRL, 0x16);
   
   if(pdata->deskdock_cb)
           pdata->deskdock_cb(TSU8111_ATTACHED);   

}
EXPORT_SYBMOL_GPL(TSU8111_CheckAndHookAudioDock);
#endif


static ssize_t tsu8111_show_status(struct device *dev,
                                  struct device_attribute *attr,
                                  char *buf)
{
       struct tsu8111_usbsw *usbsw = dev_get_drvdata(dev);
       struct i2c_client *client = usbsw->client;
       u8 devid, ctrl, adc, dev1, dev2, intr;
       u8 intmask1, intmask2, time1, time2, mansw1;

       tsu8111_read_reg(client, TSU8111_REG_DEVID, &devid);
       tsu8111_read_reg(client, TSU8111_REG_CTRL, &ctrl);
       tsu8111_read_reg(client, TSU8111_REG_ADC, &adc);
       tsu8111_read_reg(client, TSU8111_REG_INT1_MASK, &intmask1);
       tsu8111_read_reg(client, TSU8111_REG_INT2_MASK, &intmask2);
       tsu8111_read_reg(client, TSU8111_REG_DEV_T1, &dev1);
       tsu8111_read_reg(client, TSU8111_REG_DEV_T2, &dev2);
       tsu8111_read_reg(client, TSU8111_REG_TIMING1, &time1);
       tsu8111_read_reg(client, TSU8111_REG_TIMING2, &time2);
       tsu8111_read_reg(client, TSU8111_REG_MANSW1, &mansw1);

       tsu8111_read_reg(client, TSU8111_REG_INT1, &intr);
       intr &= 0xffff;

       return sprintf(buf, "Device ID(%02x), CTRL(%02x)\n"
                       "ADC(%02x), DEV_T1(%02x), DEV_T2(%02x)\n"
                       "INT(%04x), INTMASK(%02x, %02x)\n"
                       "TIMING(%02x, %02x), MANSW1(%02x)\n",
                       devid, ctrl, adc, dev1, dev2, intr,
                       intmask1, intmask2, time1, time2, mansw1);
}

static ssize_t tsu8111_show_manualsw(struct device *dev,
               struct device_attribute *attr, char *buf)
{
       return tsu8111_get_switch(buf);

}

static ssize_t tsu8111_set_manualsw(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
       tsu8111_set_switch(buf);
       return count;
}

static ssize_t tsu8111_set_syssleep(struct device *dev,
                                   struct device_attribute *attr,
                                   const char *buf, size_t count)
{
	struct tsu8111_usbsw *usbsw = chip;
	struct i2c_client *client = usbsw->client;
	u8 value = 0;

	if (!strncmp(buf, "1", 1))
	{
		wake_unlock(&JIGConnect_idle_wake);
		wake_unlock(&JIGConnect_suspend_wake);

		if(muic_type==muicTypeFSA880)
		{
			tsu8111_read_reg(client, TSU8111_REG_MANSW2, &value);
			tsu8111_write_reg(client, TSU8111_REG_MANSW2, 0x00);
			msleep(100);
			tsu8111_read_reg(client, TSU8111_REG_CTRL, &value);
			tsu8111_write_reg(client, TSU8111_REG_CTRL, 0x00);
			msleep(100);
		}
		else
		{
			tsu8111_read_reg(client, TSU8111_REG_CTRL, &value);
			value &= ~MANUAL_SWITCH;
			tsu8111_write_reg(client, TSU8111_REG_CTRL, value);
		
		msleep(100);
			tsu8111_read_reg(client, TSU8111_REG_MANSW2, &value);
			value &= ~MANSW2_JIG;
			tsu8111_write_reg(client, TSU8111_REG_MANSW2, value);
			
			msleep(100);
		}
		isManual=1;
	}
	else /*No use case*/
	{
		tsu8111_read_reg(client, TSU8111_REG_CTRL, &value);
		value |= MANUAL_SWITCH;
		tsu8111_write_reg(client, TSU8111_REG_CTRL, value);
	}
	return count;
}
					

static DEVICE_ATTR(status, S_IRUGO, tsu8111_show_status, NULL);
static DEVICE_ATTR(switch, S_IRUGO | S_IWGRP,
               tsu8111_show_manualsw, tsu8111_set_manualsw);
static DEVICE_ATTR(syssleep, S_IWUSR, NULL, tsu8111_set_syssleep);

static struct attribute *tsu8111_attributes[] = {
       &dev_attr_status.attr,
       &dev_attr_switch.attr,
       &dev_attr_syssleep.attr,
       NULL
};

static const struct attribute_group tsu8111_group = {
       .attrs = tsu8111_attributes,
};

static irqreturn_t tsu8111_irq_handler(int irq, void *data)
{
       struct tsu8111_usbsw *usbsw = data;

       if (!work_pending(&usbsw->work)) {
               disable_irq_nosync(irq);
               schedule_work(&usbsw->work);
       }

       return IRQ_HANDLED;
}

/* SW RESET for TI USB:To fix no USB recog problem after jig attach&detach*/
#if 0 // sehyoung
static void TI_SWreset(struct tsu8111_usbsw *usbsw)
{
	struct i2c_client *client = usbsw->client;

	printk("[TSU8111] TI_SWreset ...Start\n");
	disable_irq(client->irq); 

	/*Hold SCL&SDA Low more than 30ms*/
	gpio_direction_output(MUSB_SDA,0);
	gpio_direction_output(MUSB_SCL,0);
	msleep(31);
	/*Make SCL&SDA High again*/
	gpio_direction_output(MUSB_SDA,1);
	gpio_direction_output(MUSB_SCL,1);
	/*Should I make this input setting? Not Sure*/
	//gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO64));
	//gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO66));

	/*Write SOME Init register value again*/
	tsu8111_write_reg(client, TSU8111_REG_INT2_MASK, 0x20);
	tsu8111_write_reg(client, TSU8111_REG_CTRL, 0x1E);

	enable_irq(client->irq); 
	printk("[TSU8111] TI_SWreset ...Done\n");
}
#else
static void TI_SWreset(struct tsu8111_usbsw *usbsw)
{
	printk("[TSU8111] TI_SWreset ...Done\n");
}
#endif
  
 //extern void mv_usb_connect_change(int status); // khMa
static void tsu8111_detect_dev(struct tsu8111_usbsw *usbsw, u8 intr)
{
       u8 val1, val2;// , ctrl,temp;
       struct fsa880_muic *fsa880_muic = fsa880_muic_ptr;
       struct tsu8111_platform_data *pdata = usbsw->pdata;
       struct i2c_client *client = usbsw->client;


	printk("[TSU8111] tsu8111_detect_dev.........\n");
       tsu8111_read_reg(client, TSU8111_REG_DEV_T1, &val1);
       tsu8111_read_reg(client, TSU8111_REG_DEV_T2, &val2);

	if((intr==0x01) &&(val1==0x00) && (val2==0x00) && (isProbe == 0))
	{
		printk("[tsu8111] (intr==0x01) &&(val1==0x00) && (val2==0x00) !!!!!\n");
		tsu8111_read_adc_value();
		return;
	}
       /* Attached */
       if (intr & (1 << 0)) 
	{
               if (val1 & TSU8111_DEV_T1_USB_MASK ||
                       val2 & TSU8111_DEV_T2_USB_MASK) {
			printk("[TSU8111] TSU8111_USB ATTACHED*****\n");

			fsa880_muic->charger_type = PMU_MUIC_CHGTYP_USB;
			CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_CHARGER_TYPE,PMU_MUIC_CHGTYP_USB);
               }

               if (val1 & TSU8111_DEV_T1_CHARGER_MASK) {
			printk("[TSU8111] Charger ATTACHED*****\n");
			fsa880_muic->charger_type = PMU_MUIC_CHGTYP_DEDICATED_CHGR;
			CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_CHARGER_TYPE,PMU_MUIC_CHGTYP_DEDICATED_CHGR);
               }

               if (val2 & TSU8111_DEV_T2_JIG_MASK) {
			printk("[TSU8111] JIG ATTACHED*****\n"); 			   
			wake_lock(&JIGConnect_idle_wake);
			wake_lock(&JIGConnect_suspend_wake);
			fsa880_muic->charger_type = PMU_MUIC_CHGTYP_NONE;
			fsa880_muic->jig_status = 1;
               }
       } 
	else if (intr & (1 << 1)) 
	{    /* DETACH */
		fsa880_muic->charger_type = PMU_MUIC_CHGTYP_NONE;
               if (usbsw->dev1 & TSU8111_DEV_T1_USB_MASK ||
                       usbsw->dev2 & TSU8111_DEV_T2_USB_MASK) {
                       printk("[TSU8111] TSU8111_USB Detached*****\n");
			  CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_CHARGER_TYPE,PMU_MUIC_CHGTYP_NONE);

               }


               if (usbsw->dev1 & TSU8111_DEV_T1_CHARGER_MASK) {
			printk("[TSU8111] Charger Detached*****\n");
			CALL_EVENT_HANDLER(MAX8986_MUIC_EVENT_CHARGER_TYPE,PMU_MUIC_CHGTYP_NONE);
               }

               if (usbsw->dev2 & TSU8111_DEV_T2_JIG_MASK) {      
			printk("[TSU8111] JIG Detached*****\n");			   	
			wake_unlock(&JIGConnect_idle_wake);
			wake_unlock(&JIGConnect_suspend_wake);
			fsa880_muic->jig_status = 0;
               }
       }

       usbsw->dev1 = val1;
       usbsw->dev2 = val2;

       chip->dev1 = val1;
       chip->dev2 = val2;

}

#if 0 // sehyoung
int get_real_usbic_state(void)
{
       struct tsu8111_usbsw *usbsw = chip;
       int ret = MICROUSBIC_NO_DEVICE ;
       u8 val1 = 0;
       u8 val2 = 0;

       /* read real usb ic state
       val1 = chip->dev1;
       val2 = chip->dev2;
       */
       struct i2c_client *client = usbsw->client;
       tsu8111_read_reg(client, TSU8111_REG_DEV_T1, &val1);
       tsu8111_read_reg(client, TSU8111_REG_DEV_T2, &val2);

       if (val1 & TSU8111_DEV_T1_USB_MASK)
               ret = MICROUSBIC_USB_CABLE;
       else if (val1 & TSU8111_DEV_T1_CHARGER_MASK)
               ret = MICROUSBIC_USB_CHARGER;
       else if (val1 & TSU8111_DEV_T1_UART_MASK)
               ret = MICROUSBIC_USB_CHARGER;
	   else if (val1 & TSU8111_DEV_T1_HOST_MASK)
		   	   ret = MICROUSBIC_HOST;

       if (ret ==  MICROUSBIC_NO_DEVICE) {
               if (val2 & DEV_JIG_USB_ON)
				   ret = MICROUSBIC_JIG_USB_ON;
			   else if (val2 & TSU8111_DEV_T2_MHL_MASK)
				   ret = MICROUSBIC_MHL_CHARGER;
       }

       return ret;
}
#endif
static void tsu8111_work_cb(struct work_struct *work)
{
       u8 intr, intr2;//, val1, val2;
       struct tsu8111_usbsw *usbsw =
               container_of(work, struct tsu8111_usbsw, work);
       struct i2c_client *client = usbsw->client;

       /* clear interrupt */
	if(muic_type==muicTypeFSA880)
	{
		tsu8111_read_reg(client, TSU8111_REG_INT1, &intr);
		printk("[FSA880] %s: intr=0x%x \n",__func__,intr);
	}
	else
	{
		tsu8111_read_reg(client, TSU8111_REG_INT1, &intr);
		tsu8111_read_reg(client, TSU8111_REG_INT2, &intr2);	
		printk("[TSU8111] %s: intr=0x%x, intr2=0x%x \n",__func__,intr,intr2);
	}
	intr &= 0xffff;

	enable_irq(client->irq); 

	if(intr== 0x00)
	{	
		printk("[TSU8111] (intr== 0x00) in work_cb !!!!!\n");
		tsu8111_read_adc_value();
		return;
	}

         /* device detection */
       tsu8111_detect_dev(usbsw, intr);
}

static int tsu8111_irq_init(struct tsu8111_usbsw *usbsw)
{
       struct i2c_client *client = usbsw->client;
       int ret, irq = -1;

       INIT_WORK(&usbsw->work, tsu8111_work_cb);

	//ret = gpio_request(mfp_to_gpio(GPIO_IRQ(client->irq)),"TSU8111 irq");
	ret = gpio_request(MUSB_IRQ_GPIO, "TSU8111_irq");
	if(ret)
	{
		dev_err(&client->dev,"%s: Unable to get gpio %d\n", __func__,MUSB_IRQ_GPIO);
		goto gpio_out;
	}
	   //gpio_direction_input(mfp_to_gpio(GPIO_IRQ(client->irq)));
	gpio_direction_input(MUSB_IRQ_GPIO);

       ret = request_irq(client->irq, tsu8111_irq_handler,
                      IRQF_NO_SUSPEND|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_LOW*/,
                       "TSU8111 micro USB", usbsw); /*2. Low level detection*/
       if (ret) {
               dev_err(&client->dev,"%s: Unable to get IRQ %d ,%d\n",__func__,client->irq, ret);
               goto out;
       }

       return 0;
gpio_out:
	   //gpio_free(mfp_to_gpio(GPIO_IRQ(client->irq)));
	   gpio_free(MUSB_IRQ_GPIO);

out:
       return ret;
}

static int __devinit tsu8111_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	//struct regulator *regulator;
       //struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
       struct tsu8111_platform_data *pdata = client->dev.platform_data;
       struct tsu8111_usbsw *usbsw;
   	struct fsa880_muic *fsa880_muic;

	unsigned int data;
	int ret = 0;
	u8 devID;
	
	u8 intr, intr2;
	u8 mansw1;
	unsigned int ctrl = CTRL_MASK;

       printk("[TSU8111] PROBE ......\n");
	   
	isProbe = 1;

	fsa880_muic = kzalloc(sizeof(struct fsa880_muic), GFP_KERNEL);
	if (unlikely(!fsa880_muic))
	{
		pr_err("%s: fsa880_muic memory alloc failed\n", __func__);
		return -ENOMEM;
	}
	fsa880_muic_ptr = fsa880_muic;
	   
       //add for AT Command 
       wake_lock_init(&JIGConnect_idle_wake, WAKE_LOCK_IDLE, "jig_connect_idle_wake");
       wake_lock_init(&JIGConnect_suspend_wake, WAKE_LOCK_SUSPEND, "jig_connect_suspend_wake");
	   
       usbsw = kzalloc(sizeof(struct tsu8111_usbsw), GFP_KERNEL);
       if (!usbsw) {
               dev_err(&client->dev, "failed to allocate driver data\n");
               return -ENOMEM;
       }

       usbsw->client = client;
       usbsw->pdata = client->dev.platform_data;

       chip = usbsw;

       i2c_set_clientdata(client, usbsw);

	   
#if defined(CONFIG_SPA)
		   spa_external_event = spa_get_external_event_handler();
#endif

	/* clear interrupt */
	tsu8111_read_reg(client, TSU8111_REG_INT1, &intr);

	tsu8111_read_reg(client, TSU8111_REG_DEVID, &devID);
	if(devID==0x0a)
		muic_type=muicTypeTI8111;
	else if(devID==0x00)
		muic_type=muicTypeFSA880;
	else
		muic_type=muicTypeFSA;

	if(muic_type==muicTypeFSA880)
	{

	   intr &= 0xffff;
	   /* set control register */
	   tsu8111_write_reg(client, TSU8111_REG_CTRL, 0x04);
	}
	else if(muic_type==muicTypeTI8111)
	{
	   tsu8111_read_reg(client, TSU8111_REG_INT2, &intr2);
	   intr &= 0xffff;

	   /* unmask interrupt (attach/detach only) */
	   ret = tsu8111_write_reg(client, TSU8111_REG_INT1_MASK, 0x00);
	   if (ret < 0)
			  return ret;

	   /*TI USB : not to get Connect Interrupt : no more double interrupt*/
	   ret = tsu8111_write_reg(client, TSU8111_REG_INT2_MASK, 0x20);
	   if (ret < 0)
			  return ret;

	   tsu8111_read_reg(client, TSU8111_REG_MANSW1, &mansw1);
	   usbsw->mansw = mansw1;

	   ctrl &= ~INT_MASK;			  /* Unmask Interrupt */
	   if (usbsw->mansw)
			  ctrl &= ~MANUAL_SWITCH; /* Manual Switching Mode */
	   
	   tsu8111_write_reg(client, TSU8111_REG_CTRL, ctrl);
	}
	else
	   printk("[TSU8111] Error!!!! No Type. Check dev ID(0x01 addr) ......\n");



       ret = tsu8111_irq_init(usbsw);
       if (ret)
               goto tsu8111_probe_fail;

       ret = sysfs_create_group(&client->dev.kobj, &tsu8111_group);
       if (ret) {
               dev_err(&client->dev,
                               "[TSU8111] Creating fsa9480 attribute group failed");
               goto tsu8111_probe_fail2;
       }

       /* device detection */
	tsu8111_detect_dev(usbsw, 1); 
	isProbe = 0;

	/*reset UIC*/
	if(muic_type==muicTypeFSA880)
		tsu8111_write_reg(client, TSU8111_REG_CTRL, 0x04);
	else
	{
		tsu8111_write_reg(client, TSU8111_REG_CTRL, 0x1E);
		/*set timing1 to 100ms*/
		tsu8111_write_reg(client, TSU8111_REG_TIMING1, 0x1);
	}
       printk("[TSU8111] PROBE Done.\n");
       return 0;

tsu8111_probe_fail2:
       if (client->irq)
               free_irq(client->irq, NULL);
tsu8111_probe_fail:
       i2c_set_clientdata(client, NULL);
       kfree(usbsw);
       return ret;
}
static int __devexit tsu8111_remove(struct i2c_client *client)
{
       struct tsu8111_usbsw *usbsw = i2c_get_clientdata(client);
       if (client->irq)
               free_irq(client->irq, NULL);
       i2c_set_clientdata(client, NULL);

       sysfs_remove_group(&client->dev.kobj, &tsu8111_group);
       kfree(usbsw);
       return 0;
}

static int tsu8111_suspend(struct i2c_client *client)
{
	//gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO64),1); // set_value
	//gpio_direction_output(mfp_to_gpio(MFP_PIN_GPIO66),1);

	printk("[TSU8111] tsu8111_suspend  enable_irq_wake...\n");
	enable_irq_wake(client->irq);

       return 0;
}


#ifdef CONFIG_PM
static int tsu8111_resume(struct i2c_client *client)
{
	printk("[TSU8111] tsu8111_resume  disable_irq_wake...\n");
	disable_irq_wake(client->irq);
	if(isManual==1)
	{
		if(muic_type==muicTypeFSA880)
			tsu8111_write_reg(client, TSU8111_REG_CTRL, 0x04);
		else
			tsu8111_write_reg(client, TSU8111_REG_CTRL, 0x1E);

		isManual=0;
	}
		
	return 0;
}
#else
#define tsu8111_resume         NULL
#endif

static const struct i2c_device_id tsu8111_id[] = {
       {"tsu8111", 0},
       {}
};
MODULE_DEVICE_TABLE(i2c, tsu8111_id);

static struct i2c_driver tsu8111_i2c_driver = {
       .driver = {
               .name = "tsu8111",
       },
       .probe = tsu8111_probe,
       .remove = __devexit_p(tsu8111_remove),
//       .suspend=tsu8111_suspend,
//       .resume = tsu8111_resume,
       .id_table = tsu8111_id,
};

static int __init tsu8111_init(void)
{
       return i2c_add_driver(&tsu8111_i2c_driver);
}

module_init(tsu8111_init);

static void __exit tsu8111_exit(void)
{
       i2c_del_driver(&tsu8111_i2c_driver);
}

module_exit(tsu8111_exit);

MODULE_AUTHOR("Wonguk.Jeong <wonguk.jeong@samsung.com>");
MODULE_DESCRIPTION("TSU8111 USB Switch driver");
MODULE_LICENSE("GPL");



