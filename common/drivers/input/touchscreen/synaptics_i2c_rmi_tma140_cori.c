/* drivers/input/keyboard/synaptics_i2c_rmi.c
 *
 * Copyright (C) 2007 Google, Inc.
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

#include <linux/slab.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/device.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/regulator/consumer.h>



#include <linux/firmware.h>
#include <linux/uaccess.h> 
/* firmware - update */

#define MAX_X	240 
#define MAX_Y	320
#define TSP_INT 30
#define TSP_SDA 27
#define TSP_SCL 26

#define MAX_KEYS	2
#define MAX_USING_FINGER_NUM 2

static const int touchkey_keycodes[] = {
			KEY_MENU,
			KEY_BACK,
			//KEY_HOME,
			//KEY_SEARCH,
};

#define TOUCH_ON 1
#define TOUCH_OFF 0

#define TRUE    1
#define FALSE    0

#define I2C_RETRY_CNT	3

//#define __TOUCH_DEBUG__ 1
#define USE_THREADED_IRQ	1

//#define __TOUCH_KEYLED__ 

static struct regulator *touch_regulator=NULL;
#if defined (__TOUCH_KEYLED__)
static struct regulator *touchkeyled_regulator=NULL;
#endif

#if USE_THREADED_IRQ

#else
static struct workqueue_struct *synaptics_wq;
#endif

static struct workqueue_struct *check_ic_wq;

static int touchkey_status[MAX_KEYS];

#define TK_STATUS_PRESS		1
#define TK_STATUS_RELEASE		0

int tsp_irq;
int st_old;

typedef struct
{
	int8_t id;	/*!< (id>>8) + size */
	int8_t status;/////////////IC
	int8_t z;	/*!< dn>0, up=0, none=-1 */
	int16_t x;			/*!< X */
	int16_t y;			/*!< Y */
} report_finger_info_t;

static report_finger_info_t fingerInfo[MAX_USING_FINGER_NUM]={0,};

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	int use_irq;
	struct hrtimer timer;				////////////////////////IC
	struct work_struct  work;
	struct work_struct  work_timer;		////////////////////////IC
	struct early_suspend early_suspend;
};

#if defined (CONFIG_TOUCHSCREEN_MMS128_TASSCOOPER)
static int8_t Synaptics_Connected = 0;
#endif

struct synaptics_ts_data *ts_global;

/* firmware - update */
static int firmware_ret_val = -1;
static int HW_ver = -1;
unsigned char now_tma140_update_luisa = 0;
unsigned char tsp_special_update = 0;

/* touch information*/
unsigned char touch_vendor_id = 0;
unsigned char touch_hw_ver = 0;
unsigned char touch_sw_ver = 0;

// need to verify
#define TSP_VENDER_ID	0xF0
#define TSP_HW_VER1		0x03
#define TSP_SW_VER1		0x07

#define TSP_HW_VER2		0x04
#define TSP_SW_VER2		0x04

#define TSP_HW_VER3		0x05
#define TSP_SW_VER3		0x03

int tsp_irq_num = 0;
int tsp_workqueue_num = 0;
int tsp_threadedirq_num = 0;

int g_touch_info_x = 0;
int g_touch_info_y = 0;
int g_touch_info_press = 0;

static int pre_ta_stat = 0;
int tsp_status=0;
int reset_check = 0;

int firm_update( void );
extern int cypress_update( int );
void set_tsp_for_ta_detect(int);

int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size);

struct touch_trace_data {
	uint32_t time;
	uint16_t x1;
	uint16_t y1;
	uint16_t x2;
	uint16_t y2;
};
#define MAX_TOUCH_TRACE_NUMBER	10000
static int touch_trace_index = 0;
static struct touch_trace_data touch_trace_info[MAX_TOUCH_TRACE_NUMBER];

/* sys fs */
struct class *touch_class;
EXPORT_SYMBOL(touch_class);
struct device *firmware_dev;
EXPORT_SYMBOL(firmware_dev);

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_store( struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t firmware_ret_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);
static ssize_t raw_show_tma140(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_enable_tma140(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t raw_disable_tma140(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t read_rawdata_tma140(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t read_difference_tma140(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t rawdata_pass_fail_tma140(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(firmware	, 0444, firmware_show, NULL);
static DEVICE_ATTR(firmware_ret	, 0444, firmware_ret_show, NULL);
static DEVICE_ATTR(raw, 0444, raw_show_tma140, NULL) ;
static DEVICE_ATTR(raw_enable, 0444, raw_enable_tma140, NULL) ;
static DEVICE_ATTR(raw_disable, 0444, raw_disable_tma140, NULL) ;
static DEVICE_ATTR(rawdata_read, 0444, read_rawdata_tma140, NULL) ;
static DEVICE_ATTR(difference_read, 0444, read_difference_tma140, NULL) ;
static DEVICE_ATTR(raw_value, 0444, rawdata_pass_fail_tma140, NULL) ;

/* sys fs */

static int tsp_testmode = 0;
static int prev_wdog_val = -1;
static int tsp_irq_operation = 0;
static unsigned int touch_present = 0;


#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

extern int bcm_gpio_pull_up(unsigned int gpio, bool up);
extern int bcm_gpio_pull_up_down_enable(unsigned int gpio, bool enable);
extern int set_irq_type(unsigned int irq, unsigned int type);
extern int tsp_charger_type_status;

void TSP_forced_release_forkey(void);

//static struct muti_touch_info g_Mtouch_info[MAX_USING_FINGER_NUM];
//static struct key_info touchkey_status[MAX_KEYS];

#if defined (CONFIG_TOUCHSCREEN_MMS128_TASSCOOPER)
extern int Is_MMS128_Connected(void);

int Is_Synaptics_Connected(void)
{
    return (int) Synaptics_Connected;
}
#endif

void touch_ctrl_regulator(int on_off)
{
	if(on_off==TOUCH_ON)
	{
			regulator_set_voltage(touch_regulator,2900000,2900000);
			regulator_enable(touch_regulator);
#if defined (__TOUCH_KEYLED__)
                     regulator_set_voltage(touchkeyled_regulator,3300000,3300000);
			regulator_enable(touchkeyled_regulator);
#endif
	}
	else
	{
			regulator_disable(touch_regulator);
#if defined (__TOUCH_KEYLED__) 
			regulator_disable(touchkeyled_regulator);
#endif
	}
}
EXPORT_SYMBOL(touch_ctrl_regulator);

int tsp_reset( void )
{
	int ret=1;

      #if defined(__TOUCH_DEBUG__)
	printk("[TSP] %s, %d\n", __func__, __LINE__ );
      #endif 
      
        if(reset_check == 0){

        reset_check = 1;

	touch_ctrl_regulator(0);

	gpio_direction_output( TSP_SCL , 0 ); 
	gpio_direction_output( TSP_SDA , 0 ); 
	gpio_direction_output( TSP_INT , 0 ); 

	msleep(500);

	TSP_forced_release_forkey();

	gpio_direction_output( TSP_SCL , 1 ); 
	gpio_direction_output( TSP_SDA , 1 ); 
	gpio_direction_output( TSP_INT , 1 ); 

	gpio_direction_input(TSP_INT);

	touch_ctrl_regulator(1);

	msleep(100);

        reset_check = 0;
        }
	return ret;
}


static void process_key_event(uint8_t tsk_msg)
{
	int i;
	int keycode= 0;
	int st_new;

        //printk("[TSP] process_key_event : %d\n", tsk_msg);

	if(	tsk_msg	== 0)
	{
		input_report_key(ts_global->input_dev, st_old, 0);
		//printk("[TSP] release keycode: %4d, keypress: %4d\n", st_old, 0);
	}
	else{
	//check each key status
		for(i = 0; i < MAX_KEYS; i++)
		{

		st_new = (tsk_msg>>(i)) & 0x1;
		if (st_new ==1)
		{
		keycode = touchkey_keycodes[i];
		input_report_key(ts_global->input_dev, keycode, 1);
		//printk("[TSP] press keycode: %4d, keypress: %4d\n", keycode, 1);
		}

		st_old = keycode;


		}
	}
}

#if USE_THREADED_IRQ
static irqreturn_t synaptics_ts_work_func(int irq, void *dev_id)
#else
static void synaptics_ts_work_func(struct work_struct *work)
#endif
{
	int ret=0;
	//uint8_t buf[12];// 02h ~ 0Dh
	uint8_t buf[29];// 02h ~ 1Fh
	uint8_t buf_key[1];
	uint8_t i2c_addr = 0x02;
	int i = 0;
	int finger = 0;
	int button_check = 0;

#if USE_THREADED_IRQ
    if(tsp_testmode)
		return IRQ_HANDLED;

	struct synaptics_ts_data *ts = dev_id;
#else
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);
#endif
	
	ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));

	if (ret <= 0) {
		printk("[TSP] i2c failed : ret=%d, ln=%d\n",ret, __LINE__);
		tsp_reset();
		goto work_func_out;
	}
#if 0
	printk("[TSP] buf[0]:%d, buf[1]:%d, buf[2]=%d, buf[3]=%d, buf[4]=%d\n", buf[0], buf[1], buf[2], buf[3], buf[4]);
	printk("[TSP] buf[5]:%d, buf[6]:%d, buf[7]=%d, buf[8]=%d, buf[9]=%d\n", buf[5], buf[6], buf[7], buf[8], buf[9]);
	printk("[TSP] buf[10]:%d, buf[11]:%d, buf[12]=%d, buf[13]=%d, buf[14]=%d\n", buf[10], buf[11], buf[12], buf[13], buf[14]);
	printk("[TSP] buf[15]:%d, buf[16]:%d, buf[17]=%d, buf[18]=%d, buf[19]=%d\n", buf[15], buf[16], buf[17], buf[18], buf[19]);	
	printk("[TSP] buf[20]:%d, buf[21]:%d, buf[22]=%d, buf[23]=%d, buf[24]=%d\n", buf[20], buf[21], buf[22], buf[23], buf[24]);
	printk("[TSP] buf[25]:%d, buf[26]:%d, buf[27]=%d, buf[28]=%d=%d\n", buf[25], buf[26], buf[27], buf[28]);
#endif
	finger = buf[0] & 0x0F;	//number of touch finger
	buf_key[0] = buf[25] & 0x03; //information of touch key
	button_check = buf[0] & 0x40;

	if(button_check == 0)
	{
		fingerInfo[0].x = (buf[1] << 8) |buf[2];
		fingerInfo[0].y = (buf[3] << 8) |buf[4];
		fingerInfo[0].z = buf[5];
		fingerInfo[0].id = buf[6] >>4;

		fingerInfo[1].x = (buf[7] << 8) |buf[8];
		fingerInfo[1].y = (buf[9] << 8) |buf[10];
		fingerInfo[1].z = buf[11];
		fingerInfo[1].id = buf[6] & 0xf;

	/*********************hash
		if ( board_hw_revision >= 0x2 && HW_ver==1 )
		{

			fingerInfo[0].x = 240 - fingerInfo[0].x;
			fingerInfo[0].y = 320 - fingerInfo[0].y;
			fingerInfo[1].x = 240 - fingerInfo[1].x;
			fingerInfo[1].y = 320 - fingerInfo[1].y;
		
			//	fingerInfo[0].x = 320 - fingerInfo[0].x;
			//	fingerInfo[1].y = 480 - fingerInfo[1].y;
		}
	************************/
		//	print message
	//	for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
	//		printk("[TSP] finger[%d].x = %d, finger[%d].y = %d, finger[%d].z = %x, finger[%d].id = %x\n", i, fingerInfo[i].x, i, fingerInfo[i].y, i, fingerInfo[i].z, i, fingerInfo[i].id);

		/* check key event*/
	//	if(fingerInfo[0].status != 1 && fingerInfo[1].status != 1)	//
	//		process_key_event(buf[0]);								//HASHTSK
		//if(finger == 0)
		//{
		//	process_key_event(buf_key[0]);
		//}

		/* check touch event */
		for ( i= 0; i<MAX_USING_FINGER_NUM; i++ )
		{
			//////////////////////////////////////////////////IC
			if(fingerInfo[i].id >=1) // press interrupt
			{
				if(fingerInfo[i].status != -2) // force release
					fingerInfo[i].status = 1;
				else
					fingerInfo[i].status = -2;
			}
			else if(fingerInfo[i].id ==0) // release interrupt (only first finger)
			{
				if(fingerInfo[i].status == 1) // prev status is press
					fingerInfo[i].status = 0;
				else if(fingerInfo[i].status == 0 || fingerInfo[i].status == -2) // release already or force release
					fingerInfo[i].status = -1;				
			}

			if(fingerInfo[i].status < 0) continue;
			//////////////////////////////////////////////////IC
			
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, fingerInfo[i].status);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].z);
			input_mt_sync(ts->input_dev);

	     #if defined(__TOUCH_DEBUG__)
			printk("[TSP] i[%d] id[%d] xyz[%d, %d, %x] status[%x]\n", i, fingerInfo[i].id, fingerInfo[i].x, fingerInfo[i].y, fingerInfo[i].z, fingerInfo[i].status);	
	    #endif
		}
	}
	else
	{
		process_key_event(buf_key[0]);	
	}
	input_sync(ts->input_dev);

	/* ++ due to touch trace ++ */
	touch_trace_info[touch_trace_index].time = jiffies;
	touch_trace_info[touch_trace_index].x1 = fingerInfo[0].x;
	touch_trace_info[touch_trace_index].y1 = fingerInfo[0].y;
	touch_trace_info[touch_trace_index].x2 = fingerInfo[1].x;
	touch_trace_info[touch_trace_index].y2 = fingerInfo[1].y;

	if(touch_trace_index >= MAX_TOUCH_TRACE_NUMBER)
	{
		touch_trace_index = 0;
	}
	/* -- due to touch trace -- */

work_func_out:
	if (ts->use_irq)
	{
		#if USE_THREADED_IRQ

		#else
		enable_irq(ts->client->irq);
		#endif
	}
	
	tsp_irq_operation = 0;
	
	#if USE_THREADED_IRQ
	return IRQ_HANDLED;
	#else

	#endif	
}


int tsp_i2c_read(u8 reg, unsigned char *rbuf, int buf_size)
{
	int i, ret=-1;
	struct i2c_msg rmsg;
	uint8_t start_reg;

	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		rmsg.addr = ts_global->client->addr;
		rmsg.flags = 0;//I2C_M_WR;
		rmsg.len = 1;
		rmsg.buf = &start_reg;
		start_reg = reg;
		
		ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1);

		if(ret >= 0) 
		{
			rmsg.flags = I2C_M_RD;
			rmsg.len = buf_size;
			rmsg.buf = rbuf;
			ret = i2c_transfer(ts_global->client->adapter, &rmsg, 1 );

			if (ret >= 0)
				break; // i2c success
		}

		if( i == (I2C_RETRY_CNT - 1) )
		{
			printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
		}
	}

	return ret;
}



static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	#if USE_THREADED_IRQ
	
	#else
	queue_work(synaptics_wq, &ts_global->work);
	#endif

	hrtimer_start(&ts_global->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}


void TSP_forced_release_forkey(void)
{
	int i, key;
	int temp_value=0;
    
	for(i=0; i<MAX_USING_FINGER_NUM; i++)
	{
		if(fingerInfo[i].z== -1)
			continue;

		input_report_abs(ts_global->input_dev, ABS_MT_TRACKING_ID, i);
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_X, fingerInfo[i].x);
		input_report_abs(ts_global->input_dev, ABS_MT_POSITION_Y, fingerInfo[i].y);
		input_report_abs(ts_global->input_dev, ABS_MT_TOUCH_MAJOR, 0 );
		input_report_abs(ts_global->input_dev, ABS_MT_WIDTH_MAJOR, fingerInfo[i].z);      				
		input_mt_sync(ts_global->input_dev);   

		printk("[TSP] force release\n");

		if(fingerInfo[i].z == 0)
			fingerInfo[i].z = -1;

		temp_value++;
	}

	if(temp_value>0)
		input_sync(ts_global->input_dev);

    
//	for(key = 0; key < MAX_KEYS ; key++)
//	{
//		touchkey_status[key].key_press = RELEASE_KEY;
//		input_report_key(ts_global->input_dev, touchkey_status[key].key_value, touchkey_status[key].key_press);	
//	}
	
}


static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	if(tsp_testmode)
		return IRQ_WAKE_THREAD;

	struct synaptics_ts_data *ts = dev_id;

	tsp_irq_operation = 1;
	
	//printk("[TSP] %s, %d\n", __func__, __LINE__ );
	
	#if USE_THREADED_IRQ

	#else
	disable_irq_nosync(ts->client->irq);
	#endif


	#if USE_THREADED_IRQ
	return IRQ_WAKE_THREAD;
	#else
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
	#endif
}

void set_tsp_for_ta_detect(int state)
{
	
	int i, ret=0;
	uint8_t buf1[2] = {0,};
	uint8_t temp;

	printk("[TSP] %s, %d\n", __func__, __LINE__ );
	
	if(tsp_status==0)
	{	
	if((tsp_testmode == 0) && (tsp_irq_operation == 0 && (now_tma140_update_luisa == 0)))
    {
		if(state)
		{
			printk("[TSP] [1] set_tsp_for_ta_detect!!! state=1\n");
	        
			//buf1[0] = 0x00;//address
			//buf1[1] = 0x01;//data
	        //ret = i2c_master_send(ts_global->client, &buf1, 2);
	    
	        //if(ret<0) 
			//{
			//	printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
			//}

			for (i = 0; i < I2C_RETRY_CNT; i++)
			{
				buf1[0] = 0x01; //address
				ret = i2c_master_send(ts_global->client, buf1, 1);

				if (ret >= 0)
				{
					ret = i2c_master_recv(ts_global->client, buf1, 1);

					if (ret >= 0)
					{
						temp = buf1[0] | 0x04;//0b0000 0100

						buf1[0] = 0x01;//address
						buf1[1] = temp;//data
						ret = i2c_master_send(ts_global->client, buf1, 2);

						if (ret >= 0)
						{
							printk("[TSP] 01h = 0x%x\n", temp);
							break; // i2c success
						}
					}	
				}

				printk("[TSP] %s, %d, fail\n", __func__, __LINE__ );
			}

			pre_ta_stat = 1;
		}
		else
		{
			printk("[TSP] [2] set_tsp_for_ta_detect!!! state=0\n");
	        
			//buf1[0] = 0x00;//address
			//buf1[1] = 0x00;//data
	        //ret = i2c_master_send(ts_global->client, &buf1, 2);
	            
	        //if(ret<0) 
			//{
			//	printk("[TSP] Error code : %d, %d\n", __LINE__, ret );
		    //}

			for (i = 0; i < I2C_RETRY_CNT; i++)
			{
				buf1[0] = 0x01; //address
				ret = i2c_master_send(ts_global->client, buf1, 1);

				if (ret >= 0)
				{
					ret = i2c_master_recv(ts_global->client, buf1, 1);

					if (ret >= 0)
					{
						temp = buf1[0] & 0xFB;//0b1111 1011

						buf1[0] = 0x01;//address
						buf1[1] = temp;//data
						ret = i2c_master_send(ts_global->client, buf1, 2);

						if (ret >= 0)
						{
							printk("[TSP] 01h = 0x%x\n", temp);
							break; // i2c success
						}

					}	
				}

				printk("[TSP] %s, %d, fail\n", __func__, __LINE__ );
			}
			
			pre_ta_stat = 0;
		}
	}
	}
}	
EXPORT_SYMBOL(set_tsp_for_ta_detect);


static void check_ic_work_func(struct work_struct *work_timer)
{
	int ret=0;
	uint8_t i2c_addr;
	uint8_t wdog_val[1];

	struct synaptics_ts_data *ts = container_of(work_timer, struct synaptics_ts_data, work_timer);

	i2c_addr = 0x1F;
	wdog_val[0] = 1;
	   
	if((tsp_testmode == 0) && (tsp_irq_operation == 0 && (now_tma140_update_luisa == 0)))
	{
		ret = tsp_i2c_read( i2c_addr, wdog_val, sizeof(wdog_val));
			
		if(ret >=0)
		{
			//printk("[TSP] prev_wdog_val = %d, wdog_val = %d\n", prev_wdog_val, ((wdog_val[0] & 0xFC) >> 2) );
			
			if(((wdog_val[0] & 0xFC) >> 2) == (uint8_t)prev_wdog_val)
			{
				printk("[TSP] %s tsp_reset counter = %x, prev = %x\n", __func__, ((wdog_val[0] & 0xFC) >> 2), (uint8_t)prev_wdog_val);

				disable_irq(ts_global->client->irq);
				tsp_reset();
				enable_irq(ts_global->client->irq);
				prev_wdog_val = -1;
			}
			else
			{
				prev_wdog_val = (wdog_val[0] & 0xFC) >> 2;
			}			
		}
		else//if(ret < 0)
		{
			disable_irq(ts_global->client->irq);
			tsp_reset();
			enable_irq(ts_global->client->irq);
			printk("[TSP] silabs_ts_work_func : i2c_master_send [%d]\n", ret);			
		}

		if( pre_ta_stat != tsp_charger_type_status )
		{
			set_tsp_for_ta_detect(tsp_charger_type_status);
		}
	}
}

static enum hrtimer_restart synaptics_watchdog_timer_func(struct hrtimer *timer)
{
	//printk("[TSP] %s, %d\n", __func__, __LINE__ );

	queue_work(check_ic_wq, &ts_global->work_timer);
	hrtimer_start(&ts_global->timer, ktime_set(2, 0), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}


int synaptics_ts_check(void)
{
	int ret, i;
	uint8_t buf_tmp[3]={0,0,0};
	int retry = 3;


	ret = tsp_i2c_read( 0x1B, buf_tmp, sizeof(buf_tmp));

	// i2c read retry
	if(ret <= 0)
	{
		for(i=0; i<retry;i++)
		{
			ret=tsp_i2c_read( 0x1B, buf_tmp, sizeof(buf_tmp));

			if(ret > 0)
				break;
		}
	}

	if (ret <= 0) 
	{
		printk("[TSP][Synaptics][%s] %s\n", __func__,"Failed synpatics i2c");
#if defined (CONFIG_TOUCHSCREEN_MMS128_TASSCOOPER)	
		Synaptics_Connected = 0;
#endif
		ret = 0;
	}
	else 
	{
		printk("[TSP][Synaptics][%s] %s\n", __func__,"Passed synpatics i2c");
#if defined (CONFIG_TOUCHSCREEN_MMS128_TASSCOOPER)	
		Synaptics_Connected = 1;
#endif

		printk("[TSP][Synaptics][%s][SlaveAddress : 0x%x][VendorID : 0x%x] [HW : 0x%x] [SW : 0x%x]\n", __func__,ts_global->client->addr, buf_tmp[0], buf_tmp[1], buf_tmp[2]);

		if ( buf_tmp[0] == 0xf0 )//(ts->hw_rev == 0) && (ts->fw_ver == 2))
		{
			ret = 1;
			printk("[TSP][Synaptics][%s] %s\n", __func__,"Passed synaptics_ts_check");
		}
		else
		{
			ret = 0;
			printk("[TSP][Synaptics][%s] %s\n", __func__,"Failed synaptics_ts_check");
		}
		
	}

	return ret;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;

	uint8_t i2c_addr = 0x1B;
  	uint8_t buf[3], buf_tmp[3]={0,0,0};
	uint8_t addr[1];	
	int i;
    	int ret = 0, key = 0;


#if defined (CONFIG_TOUCHSCREEN_MMS128_TASSCOOPER)
    printk("[TSP][Synaptics][%s] %s\n", __func__,"Called");

    if(Is_MMS128_Connected()== 1)
    {
        printk("[TSP][Synaptics][%s] %s\n", __func__,"Melfas already detected !!");

        return -ENXIO;
    }
#endif

	printk("[TSP] %s, %d\n", __func__, __LINE__ );

//	touch_ctrl_regulator(TOUCH_ON);
//	msleep(100);	
	touch_ctrl_regulator(TOUCH_OFF);
	msleep(200);
	touch_ctrl_regulator(TOUCH_ON);
	msleep(100);

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	#if USE_THREADED_IRQ

	#else
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	#endif
	
	INIT_WORK(&ts->work_timer, check_ic_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts_global = ts;

	tsp_irq=client->irq;


	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		printk(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "sec_touchscreen";


	ts->input_dev->keybit[BIT_WORD(KEY_POWER)] |= BIT_MASK(KEY_POWER);
	
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, MAX_X, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, MAX_Y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);


	for(key = 0; key < MAX_KEYS ; key++)
		input_set_capability(ts->input_dev, EV_KEY, touchkey_keycodes[key]);

	// for TSK
	for(key = 0; key < MAX_KEYS ; key++)
		touchkey_status[key] = TK_STATUS_RELEASE;

    
	/* ts->input_dev->name = ts->keypad_info->name; */
	ret = input_register_device(ts->input_dev);
	if (ret) {
		printk(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

    	printk("[TSP] %s, irq=%d\n", __func__, client->irq );

    if (client->irq) {
		#if USE_THREADED_IRQ
		ret = request_threaded_irq(client->irq, synaptics_ts_irq_handler, synaptics_ts_work_func, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->name, ts);
		#else		
		ret = request_irq(client->irq, synaptics_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);
		#endif
		
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}

	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
#if 1
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
#endif
	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	/* sys fs */
	touch_class = class_create(THIS_MODULE, "touch");
	if (IS_ERR(touch_class))
		pr_err("Failed to create class(touch)!\n");

	firmware_dev = device_create(touch_class, NULL, 0, NULL, "firmware");
	if (IS_ERR(firmware_dev))
		pr_err("Failed to create device(firmware)!\n");

	if (device_create_file(firmware_dev, &dev_attr_firmware) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_firmware_ret) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_firmware_ret.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_raw) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_raw_enable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_enable.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_raw_disable) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_disable.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_rawdata_read) < 0)
	   pr_err("Failed to create device file(%s)!\n", dev_attr_rawdata_read.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_difference_read) < 0)
	   pr_err("Failed to create device file(%s)!\n", dev_attr_difference_read.attr.name);
	if (device_create_file(firmware_dev, &dev_attr_raw_value) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_raw_value.attr.name);		

	/* sys fs */

    
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = synaptics_watchdog_timer_func;

#if defined (CONFIG_TOUCHSCREEN_MMS128_TASSCOOPER)
	#if USE_THREADED_IRQ
		msleep(100);
	#endif
	
		ret = synaptics_ts_check();
		if (ret <= 0) {
			 i2c_release_client(client);		
			 touch_ctrl_regulator(TOUCH_OFF);
	
			 ret = -ENXIO;
			 goto err_input_dev_alloc_failed;
		 }
#else
		/* Check point - i2c check - start */	
		//ret = tsp_i2c_read( 0x1B, buf_tmp, sizeof(buf_tmp));
		for (i = 0; i < 2; i++)
		{
			printk("[TSP] %s, %d, send\n", __func__, __LINE__ );
			addr[0] = 0x1B; //address
			ret = i2c_master_send(ts_global->client, addr, 1);
	
			if (ret >= 0)
			{
				printk("[TSP] %s, %d, receive\n", __func__, __LINE__ );
				ret = i2c_master_recv(ts_global->client, buf_tmp, 3);
				if (ret >= 0)
					break; // i2c success
			}
	
			printk("[TSP] %s, %d, fail\n", __func__, __LINE__ );
		}
	
		touch_vendor_id = buf_tmp[0] & 0xF0;
		touch_hw_ver = buf_tmp[1];
		touch_sw_ver = buf_tmp[2];
		printk("[TSP] %s:%d, ver tsp=%x, HW=%x, SW=%x\n", __func__,__LINE__, touch_vendor_id, touch_hw_ver, touch_sw_ver);
#endif
	
		HW_ver = touch_hw_ver;
	
		if(ret >= 0)
		{
			touch_present = 1;
			hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);
		}
		else//if(ret < 0) 
		{
			printk(KERN_ERR "i2c_transfer failed\n");
			printk("[TSP] %s, ln:%d, Failed to register TSP!!!\n\tcheck the i2c line!!!, ret=%d\n", __func__,__LINE__, ret);
	 		//goto err_check_functionality_failed;
		}
		/* Check point - i2c check - end */


#if 0//SET_DOWNLOAD_BY_GPIO
	#if 0
		if (/*(touch_vendor_id == TSP_VENDER_ID)&&*/( touch_hw_ver == TSP_HW_VER1)&&(touch_sw_ver < TSP_SW_VER1))
		{ 
			printk("[TSP] Firmware update start!!\n" );
			now_tma140_update_luisa = 1;
			firm_update();
			now_tma140_update_luisa = 0;
		}
		else if (/*(touch_vendor_id == TSP_VENDER_ID)&&*/( touch_hw_ver == TSP_HW_VER2)&&(touch_sw_ver < TSP_SW_VER2))
		{ 
			printk("[TSP] Firmware update start!!\n" );
			now_tma140_update_luisa = 1;
			firm_update();
			now_tma140_update_luisa = 0;	
		}	
	#endif
		{
			HW_ver = 0x04;
	
			printk("[TSP] Firmware update start!!\n" );
			now_tma140_update_luisa = 1;
			firm_update();
			now_tma140_update_luisa = 0;		
		}
 #endif // SET_DOWNLOAD_BY_GPIO


	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	//else
	//	hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	printk("[TSP] %s+\n", __func__ );
	
	  tsp_status=1; 
	
	if (ts->use_irq)
	{
		disable_irq(client->irq);
	}

	if(touch_present == 1)
		hrtimer_cancel(&ts->timer);
		
	gpio_direction_output( TSP_INT , 0 );
	gpio_direction_output( TSP_SCL , 0 ); 
	gpio_direction_output( TSP_SDA , 0 ); 

	bcm_gpio_pull_up(TSP_INT, false);
	bcm_gpio_pull_up_down_enable(TSP_INT, true);

	#if USE_THREADED_IRQ
	
	#else
	ret = cancel_work_sync(&ts->work);
	
	//if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	//{
	//	enable_irq(client->irq);
	//}
	#endif	

	msleep(20);	
	
	touch_ctrl_regulator(TOUCH_OFF);

    TSP_forced_release_forkey();
	
    printk("[TSP] %s-\n", __func__ );
        
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
    	uint8_t i2c_addr = 0x1B;
	uint8_t buf[3];
#if 0//power_contol needs
	if (ts->power) {
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "synaptics_ts_resume power on failed\n");
	}
#endif
	//synaptics_init_panel(ts);

	gpio_direction_output( TSP_SCL , 1 ); 
	gpio_direction_output( TSP_SDA , 1 ); 
	gpio_direction_output( TSP_INT , 1 ); 

	gpio_direction_input(TSP_INT);
	bcm_gpio_pull_up_down_enable(TSP_INT, false);

	touch_ctrl_regulator(TOUCH_ON);
    
	msleep(100);
		
//	while (ts->use_irq)
//	{
		ret = tsp_i2c_read( i2c_addr, buf, sizeof(buf));
//		if (ret <= 0) {
//			printk("[TSP] %d : i2c_transfer failed\n", __LINE__);
//		}
//		else if	( buf[0] == 0 )
//		{
//			printk("[TSP] %d : maybe unlimited loop! check to re-try code i2c_transfer failed\n", __LINE__);
//			continue;
//		}
//		else
//		{
			//printk("[TSP] %s:%d, ver SW=%x\n", __func__,__LINE__, buf[0] );
			touch_vendor_id = buf[0] & 0xF0;
			touch_hw_ver = buf[1];
			touch_sw_ver = buf[2];
			printk("[TSP] %s:%d, ver tsp=%x, HW=%x, SW=%x\n", __func__,__LINE__, touch_vendor_id, touch_hw_ver, touch_sw_ver);

			enable_irq(client->irq);
//			break;
//		}
//		msleep(20);
//	}

	prev_wdog_val = -1;

	if(tsp_charger_type_status == 1)
	{
		set_tsp_for_ta_detect(tsp_charger_type_status);
	}

	if(touch_present == 1)
		hrtimer_start(&ts->timer, ktime_set(2, 0), HRTIMER_MODE_REL);

  tsp_status=0; 
	printk("[TSP] %s-\n", __func__ );
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ SYNAPTICS_I2C_RMI_NAME, 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#if 1
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= SYNAPTICS_I2C_RMI_NAME,
	},
};

static int __devinit synaptics_ts_init(void)
{
#if defined (CONFIG_TOUCHSCREEN_MMS128_TASSCOOPER)
	printk("[TSP][Synaptics][%s] %s\n", __func__,"Init Func Called");

	if(Is_MMS128_Connected()== 1)
	{
		printk("[TSP][Synaptics][%s] %s\n", __func__,"Melfas already detected !!");

		return -ENXIO;
	}
#endif

	printk("[TSP] %s\n", __func__ );

	gpio_request(TSP_INT, "ts_irq");
	gpio_direction_input(TSP_INT);
	//bcm_gpio_pull_up(TSP_INT, true);
	//bcm_gpio_pull_up_down_enable(TSP_INT, true);
	set_irq_type(GPIO_TO_IRQ(TSP_INT), IRQF_TRIGGER_FALLING);

	//disable BB internal pulls for touch int, scl, sda pin
	bcm_gpio_pull_up_down_enable(TSP_INT, 0);
	bcm_gpio_pull_up_down_enable(TSP_SCL, 0);
	bcm_gpio_pull_up_down_enable(TSP_SDA, 0);

	gpio_direction_output( TSP_SCL , 1 ); 
	gpio_direction_output( TSP_SDA , 1 ); 		
 
	#if USE_THREADED_IRQ

	#else	
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq)
		return -ENOMEM;
	#endif

	check_ic_wq = create_singlethread_workqueue("check_ic_wq");	

	if (!check_ic_wq)
		return -ENOMEM;	

	touch_regulator = regulator_get(NULL,"touch_vcc");
#if defined (__TOUCH_KEYLED__)
	touchkeyled_regulator = regulator_get(NULL,"touch_keyled");
#endif
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	if (touch_regulator) 
	{
       	 regulator_put(touch_regulator);
		 touch_regulator = NULL;
    	}
#if defined (__TOUCH_KEYLED__)
	if (touchkeyled_regulator) 
	{
       	 regulator_put(touchkeyled_regulator);
		 touchkeyled_regulator = NULL;
    	}
#endif
	i2c_del_driver(&synaptics_ts_driver);

	#if USE_THREADED_IRQ

	#else	
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
	#endif

	if (check_ic_wq)
		destroy_workqueue(check_ic_wq);		
}

static ssize_t firmware_show(struct device *dev, struct device_attribute *attr, char *buf)
{
#if 0
	uint8_t i2c_addr = 0x1C;
	uint8_t buf_tmp[2] = {0};
	int phone_ver = 0;

	printk("[TSP] %s\n",__func__);

	 		
	if ( HW_ver == 1  || HW_ver == 2 || HW_ver == 3)
	{
		/* for glass */
		phone_ver = 100;  /* SW Ver.4 - change this value if New firmware be released */ 	
	}
	else
	{
		phone_ver = 200; // Acryl type
		printk("[TSP] %s:%d,HW_ver is wrong!!\n", __func__,__LINE__ );
	}
	
	tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));
	printk("[TSP] %s:%d, ver SW=%x, HW=%x\n", __func__,__LINE__, buf_tmp[1], buf_tmp[0] );

	/* below protocol is defined with App. ( juhwan.jeong@samsung.com )
		The TSP Driver report like XY as decimal.
		The X is the Firmware version what phone has.
		The Y is the Firmware version what TSP has. */

	sprintf(buf, "%d\n", phone_ver + buf_tmp[1]+(buf_tmp[0]*10) );

	return sprintf(buf, "%s", buf );
#endif

	uint8_t i2c_addr = 0x1B;
	uint8_t buf_tmp[3] = {0};
	int phone_ver = 0;

	printk("[TSP] %s\n",__func__);

	tsp_i2c_read( i2c_addr, buf_tmp, sizeof(buf_tmp));

	touch_vendor_id = buf_tmp[0] & 0xF0;
	touch_hw_ver = buf_tmp[1];
	touch_sw_ver = buf_tmp[2];
	printk("[TSP] %s:%d, ver tsp=%x, HW=%x, SW=%x\n", __func__,__LINE__, touch_vendor_id, touch_hw_ver, touch_sw_ver);

	HW_ver = touch_hw_ver;

			
	if ( HW_ver == TSP_HW_VER1 )//touch_hw_ver
	{
		phone_ver = TSP_SW_VER1;	/* change this value if New firmware be released */ 	
	}
	else if ( HW_ver == TSP_HW_VER2 )//touch_hw_ver
	{
		phone_ver = TSP_SW_VER2;	/* change this value if New firmware be released */ 	
	}	
	else if ( HW_ver == TSP_HW_VER3 )//touch_hw_ver
	{
		phone_ver = TSP_SW_VER3;	/* change this value if New firmware be released */ 	
	}	
	else
	{
		printk("[TSP] %s:%d,HW_ver is wrong!!\n", __func__,__LINE__ );
	}

	/* below protocol is defined with App. ( juhwan.jeong@samsung.com )
		The TSP Driver report like XY as decimal.
		The X is the Firmware version what phone has.
		The Y is the Firmware version what TSP has. */

	//sprintf(buf, "%d\n", phone_ver + buf_tmp[1]+(buf_tmp[0]*100) );
	sprintf(buf, "%x\n", 0x1000000 + (touch_hw_ver*0x10000) + (touch_sw_ver*0x100) +  phone_ver);
	//sprintf(buf, "%d\n", 1000000 + (buf_tmp[8]*10000) + (buf_tmp[9]*100) +  phone_ver);

	return sprintf(buf, "%s", buf );
}

/* firmware - update */
static ssize_t firmware_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	char *after;

	unsigned long value = simple_strtoul(buf, &after, 10);	
	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
	firmware_ret_val = -1;
	printk("[TSP] firmware_store  valuie : %d\n",value);
	if ( value == 0 )
	{
		printk("[TSP] Firmware update start!!\n" );

		//firm_update( );
#if FIRM_TEST
		printk("[TSP] start update cypress touch firmware !!\n");
		g_FirmwareImageSize = CYPRESS_FIRMWARE_IMAGE_SIZE;

		if(g_pTouchFirmware == NULL)
		{
			printk("[TSP][ERROR] %s() kmalloc fail !! \n", __FUNCTION__);
			return -1;
		}


		/* ready for firmware code */
		size = issp_request_firmware("touch.hex");

		/* firmware update */
		//	issp_upgrade();

		g_FirmwareImageSize = 0;

		// step.1 power off/on

		// step.2 enable irq


#endif
		return size;
	}

	return size;
}

static ssize_t firmware_ret_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	printk("[TSP] %s!\n", __func__);

	printk("[TSP] %s, firmware_ret_val = %d\n", __func__, firmware_ret_val);
	now_tma140_update_luisa = 1;
	firm_update();
	now_tma140_update_luisa = 0;

	return sprintf(buf, "%d", firmware_ret_val );
}

static ssize_t firmware_ret_store(
		struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	printk("[TSP] %s, operate nothing!\n", __func__);

	return size;
}


#define TMA140_RET_SUCCESS 0x00
int sv_tch_firmware_update = 0;
int firm_update( void )
{
	uint8_t update_num;

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
	printk("[TSP] disable_irq : %d\n", __LINE__ );
	disable_irq(tsp_irq);
	local_irq_disable();
	
	for(update_num = 1; update_num <= 5 ; update_num++)
	{
		sv_tch_firmware_update = cypress_update(HW_ver);

		if(sv_tch_firmware_update == TMA140_RET_SUCCESS)
		{
			firmware_ret_val = 1; //SUCCESS
			printk( "[TSP] %s, %d : TST200 firmware update SUCCESS !!\n", __func__, __LINE__);
			break;
		}
		else
		{
			printk( "[TSP] %s, %d : TST200 firmware update RETRY !!\n", __func__, __LINE__);
			if(update_num == 5)
			{
				firmware_ret_val = 0; //FAIL
			printk( "[TSP] %s, %d : TST200 firmware update FAIL !!\n", __func__, __LINE__);
			}
		}
	}

	printk("[TSP] enable_irq : %d\n", __LINE__ );
	local_irq_enable();
	enable_irq(tsp_irq);

	return 0;
} 

#if FIRM_TEST
static void issp_request_firmware(char* update_file_name)
{
	int idx_src = 0;
	int idx_dst = 0;
	int line_no = 0;
	int dummy_no = 0;
	char buf[2];
	int ret = 0;

	struct device *dev = &ts_global->input_dev->dev;	
	const struct firmware * fw_entry;

	printk(KERN_INFO "[TSP] %s, %d\n", __func__, __LINE__);
	printk("[TSP] firmware file name : %s\n", update_file_name);

	ret = request_firmware(&fw_entry, update_file_name, dev);
	if ( ret )
	{
		printk("[TSP] request_firmware fail, ln=%d\n", ret );
		return ;
	}
	else
	{
		printk("[TSP] request_firmware success, ln=%d\n", ret );
		printk("[TSP][DEBUG] ret=%d, firmware size=%d\n", ret, fw_entry->size);
		printk("[TSP] %c %c %c %c %c\n", fw_entry->data[0], fw_entry->data[1], fw_entry->data[2], fw_entry->data[3], fw_entry->data[4]);
	}

	do {
		if(fw_entry->data[idx_src] == ':') // remove prefix
		{
			idx_src+=9;
			dummy_no++;

			if(dummy_no != line_no+1)
			{
				printk("[ERROR] Can not skip semicolon !! line_no(%d), dummy_no(%d)\n", line_no, dummy_no);
			}
		}
		else if(fw_entry->data[idx_src] == '\r') // return code
		{
			idx_src+=2; idx_dst--; line_no++;

			if( idx_dst > TSP_LINE_LENGTH*line_no)
			{
				printk("[ERROR] length buffer over error !! line_no(%d), idx_dst(%d)\n", line_no, idx_dst);
			}
		}
		else if(fw_entry->data[idx_src] == 0x0a) // return code
		{
			idx_src+=1; idx_dst--; line_no++;

			if( idx_dst > TSP_LINE_LENGTH*line_no)
			{
				printk("[ERROR] length buffer over error !! line_no(%d), idx_dst(%d)\n", line_no, idx_dst);
			}
		}
		else
		{
			sprintf(buf, "%c%c", fw_entry->data[idx_src], fw_entry->data[idx_src+1]);
			if(idx_dst > TSP_TOTAL_LINES*TSP_LINE_LENGTH)
			{
				printk("[ERROR] buffer over error !!  line_no(%d), idx_dst(%d)\n", line_no, idx_dst);
			}
			g_pTouchFirmware[idx_dst] = simple_strtol(buf, NULL, 16);
			idx_src+=2; idx_dst++;
		}
	} while ( line_no < TSP_TOTAL_LINES );

	release_firmware(fw_entry);
}
#endif

static ssize_t raw_show_tma140(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);

	if(!tsp_testmode)
		return 0;
	
	int tma140_col_num = 8;	//0 ~ 7
	int tma140_row_num = 10;//0 ~ 9

	int  written_bytes = 0 ;	/* & error check */

	uint8_t buf1[1]={0,};
	uint8_t buf2[80]={0,};

	uint16_t ref1[80]={0,};
	uint16_t ref2[80]={0,};

	int i,j,k;
	int ret;

	uint8_t i2c_addr;
	

	/////* Raw Value */////
	/////* Enter Raw Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x40;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xC0;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);
	
	/////* Read Raw Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	printk("[TSP] Raw Value : ");
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		ref1[i] = buf2[i];
		printk(" [%d]%3d", i, buf2[i]);
	}
	printk("\n");


	/////* Difference Value */////
	/////* Enter Difference Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x50;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xD0;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}	
	msleep(50);

	/////* Read Difference Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));


	printk("[TSP] Diff Value : ");
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		ref2[i] = buf2[i];
		printk(" %d", ref2[i]);
	}
	printk("\n");
	

	/////* Send Value */////
	for (i = 0; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		written_bytes += sprintf(buf+written_bytes, "%d %d\n", ref1[i], ref2[i]);
	}

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1") ;
}


static ssize_t raw_enable_tma140(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, ret;
	uint8_t buf1[2] = {0,};

	/////* Enter Inspection Mode */////
/*	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x40;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//enter Inspection Mode
	
		if (ret >= 0)
			break; // i2c success
	}*/
	
	tsp_testmode = 1;
	printk("[TSP] %s start. line : %d, \n", __func__,__LINE__);

	mdelay(100); 

	return 1;
}


static ssize_t raw_disable_tma140(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i, ret;
	uint8_t buf1[2] = {0,};

	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{	
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode
	
		if (ret >= 0)
			break; // i2c success
	}
	
	tsp_testmode = 0;
	printk("[TSP] %s stop. line : %d, \n", __func__,__LINE__);

	return 1;
}


static ssize_t read_rawdata_tma140(struct device *dev, struct device_attribute *attr, char *buf)//AT+TSPPTEST=1,1,1
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);
	
	int tma140_col_num = 8;	//0 ~ 7
	int tma140_row_num = 10;//0 ~ 9

	int  written_bytes = 0 ;	/* & error check */

	uint8_t buf1[1]={0,};
	uint8_t buf2[80]={0,};

	uint16_t ref1[80]={0,};
	uint16_t ref2[80]={0,};

	int i,j,k;
	int ret;

	uint8_t i2c_addr;
	
	tsp_testmode = 1;

	/////* Raw Value */////
	/////* Enter Raw Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x40;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xC0;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);
	
	/////* Read Raw Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

//	printk("[TSP] Raw Value : ");
//	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
//	{
//		ref1[i] = buf2[i];
//		printk(" [%d]%3d", i, buf2[i]);
//	}
//	printk("\n");


	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{	
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode
	
		if (ret >= 0)
			break; // i2c success
	}


	/////* Send Value */////
	for (i = 0; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		written_bytes += sprintf(buf+written_bytes, ",%3d", ref1[i]);
	}

//	printk("[TSP] %s\n", buf);
	
	mdelay(100);

	tsp_testmode = 0;

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1") ;
}


static ssize_t read_difference_tma140(struct device *dev, struct device_attribute *attr, char *buf)//AT+TSPPTEST=1,2,1
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);
	
	int tma140_col_num = 8; //0 ~ 7
	int tma140_row_num = 10;//0 ~ 9

	int  written_bytes = 0 ;	/* & error check */

	uint8_t buf1[1]={0,};
	uint8_t buf2[80]={0,};

	uint16_t ref1[80]={0,};
	uint16_t ref2[80]={0,};

	int i,j,k;
	int ret;

	uint8_t i2c_addr;
	
	tsp_testmode = 1;

	/////* Difference Value */////
	/////* Enter Difference Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x50;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xD0;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);
	
	/////* Read Difference Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

//	printk("[TSP] Difference Value : ");
//	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
//	{
//		ref1[i] = buf2[i];
//		printk(" [%d]%3d", i, buf2[i]);
//	}
//	printk("\n");


	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{	
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode
	
		if (ret >= 0)
			break; // i2c success
	}


	/////* Send Value */////
	for (i = 0; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		written_bytes += sprintf(buf+written_bytes, ",%3d", ref1[i]);
	}

//	printk("[TSP] %s\n", buf);
	
	mdelay(100);

	tsp_testmode = 0;

	if (written_bytes > 0)
		return written_bytes ;

	return sprintf(buf, "-1") ;
}



static ssize_t rawdata_pass_fail_tma140(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[TSP] %s entered. line : %d, \n", __func__,__LINE__);
	
	int tma140_col_num = 8; //0 ~ 7
	int tma140_row_num = 10;//0 ~ 9

	int  written_bytes = 0 ;	/* & error check */

	uint8_t buf1[1]={0,};
	uint8_t buf2[80]={0,};

	uint16_t ref1[80]={0,};
	uint16_t ref2[80]={0,};

	int i,j,k;
	int ret;

	uint8_t i2c_addr;

	uint16_t RAWDATA_MIN = 70;
	uint16_t RAWDATA_MAX = 130;
	uint16_t LIDAC_MIN = 1;
	uint16_t LIDAC_MAX = 30;

	uint8_t test_result = 1;
	
	tsp_testmode = 1;

	/////* Raw Value */////
	/////* Enter Raw Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x40;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xC0;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(50);
	
	/////* Read Raw Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));

	printk("[TSP] Raw Value : ");
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		ref1[i] = buf2[i];
		printk(" [%d]%3d", i, ref1[i]);
	}
	printk("\n");


	/////* Local IDAC Value */////
	/////* Enter Local IDAC Data Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0x60;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}
	msleep(10);
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{
		buf1[0] = 0x00;//address
		buf1[1] = 0xE0;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);
	
		if (ret >= 0)
			break; // i2c success
	}	
	msleep(50);

	/////* Read Local IDAC Data */////
	i2c_addr = 0x07;
	tsp_i2c_read( i2c_addr, buf2, sizeof(buf2));


	printk("[TSP] Local IDAC Value : ");
	for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
	{
		ref2[i] = buf2[i];
		printk(" %d", ref2[i]);
	}
	printk("\n");



	/////* Exit Inspection Mode */////
	for (i = 0; i < I2C_RETRY_CNT; i++)
	{	
		buf1[0] = 0x00;//address
		buf1[1] = 0x00;//value
		ret = i2c_master_send(ts_global->client, buf1, 2);	//exit Inspection Mode
	
		if (ret >= 0)
			break; // i2c success
	}

	mdelay(100);

	tsp_testmode = 0;


	/////* Check Result */////
#if 0	
	if(touch_hw_ver == TSP_HW_VER1)
	{
		if(touch_sw_ver == TSP_SW_VER1)
			test_result = 1;
		else
			test_result = 0;
	}
	else if(touch_hw_ver == TSP_HW_VER2)
	{
		if(touch_sw_ver == TSP_SW_VER2)
			test_result = 1;
		else
			test_result = 0;	
	}
	else if(touch_hw_ver == TSP_HW_VER3)
	{
		if(touch_sw_ver == TSP_SW_VER3)
			test_result = 1;
		else
			test_result = 0;	
	}	
	else
	{
		test_result = 0;
	}
#endif
	
	if(test_result ==1)
	{
		for(i = 0 ; i < (tma140_col_num * tma140_row_num) ; i++)
		{
			if(ref1[i] < RAWDATA_MIN && ref1[i] > RAWDATA_MAX)
			{
				test_result = 0;
				break;
			}

			if(ref2[i] < LIDAC_MIN && ref2[i] > LIDAC_MAX)
			{
				test_result = 0;
				break;
			}
		}
	}


	/////* Send Result */////
	if(test_result == 1)
		return sprintf(buf, "1"); // success
	else
		return sprintf(buf, "0"); // fail

}



module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");
