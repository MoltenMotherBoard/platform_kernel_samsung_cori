/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*	@file	drivers/power/linear-power.c
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
/**
 *
 *   @file   linear-power.c
 *
 *   @brief  Power Driver for SS6000
 *
 ****************************************************************************/
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/ioctl.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#if defined(CONFIG_HAS_WAKELOCK)
#include <linux/wakelock.h>
#endif /*CONFIG_HAS_WAKELOCK*/
#include <linux/mfd/max8986/max8986.h>
#include <linux/mfd/max8986/max8986-private.h>
#include <linux/broadcom/types.h>
#include <linux/broadcom/bcm_kril_Interface.h>
#include <linux/broadcom/bcm_fuse_sysparm.h>

#include <plat/bcm_auxadc.h>
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif /*CONFIG_HAS_EARLYSUSPEND*/
#include <linux/gpio.h>

#if defined(CONFIG_CPU_FREQ_GOV_BCM21553)  
#include <mach/bcm21553_cpufreq_gov.h>
#endif

#include <linux/d2041/core.h>

#define TRUE	1
#define FALSE	0

#define FEAT_EN_TEST_MODE 1

/* # of shifts used to divide the sum to get the average. */
#define ADC_RUNNING_AVG_SHIFT 3
/* # of samples to perform voltage running sum */
#define ADC_RUNNING_AVG_SIZE (1 << ADC_RUNNING_AVG_SHIFT)

/*Macros to control schedule frequency of charging monitor work queue - with
 * and without charger present */
#define BATTERY_LVL_MON_INTERVAL_WHILE_CHARGING		5000 /* 5 sec */
#define	BATTERY_LVL_MON_INTERVAL			30000 /* 30 sec */
#define BAT_TEMP_EXCEED_LIMIT_COUNT_MAX			3
#define BATTERY_CHARGING_HYSTERESIS			10
#define SUCCESS 0		/* request is successfull */
#define USB_PREENUM_CURR				90

#define USB_PREENUM_CURR_REQ_VAL MAX8986_CHARGING_CURR_450MA

#define BATT_RECHARGE_VOLT	4130

#define BATT_LOW_VOLT		3400

#define BAT_PERCENT_INIT_VALUE 1

#define HIGH_SUSPEND_TEMP	600
#define LOW_SUSPEND_TEMP	-50
#define HIGH_RECOVER_TEMP	400
#define LOW_RECOVER_TEMP	0


#define BAT_30SEC_INTERVAL_RECHARGE (10*HZ) /* 10 sec*/
#define BAT_90MIN_INTERVAL_RECHARGE (90*60*HZ) /* 90 min*/
#define BAT_300MIN_FAST_CHARGE_TIMER (300*60*HZ) /* 300 min*/
#define WAKE_LOCK_TIME		(HZ * 3)	/* 3 sec */
#define MAX8986_LOG_CHARGING_TIME 1
#define AUXADC_BATVOLT_CHANNEL		3
#define AUXADC_BATTEMP_CHANNEL		0
#define AUXADC_BATVF_CHANNEL		1

#define CHG_EN 22
#define CHG_DET 		10
#define CHG_STATE 	23
#define SH_ON  0
#define SH_OFF 1

int prev_scaled_level=0;
int init_batt_lvl_interval=0;

static int is_ovp_suspended=false;
static int check_func_called_by_cp=false;
static int cnt_func_called=0;
static struct platform_device *power_device;

extern void dwc_otg_cil_get_BC11_Charger_Type(pmu_muic_chgtyp bc11_chargertype);
extern int fsa880_muic_register_event_handler(int event,void (*handler) (int, u32, void *), void* data);
extern void fsa880_muic_unregister_event_handler(int event);
extern pmu_muic_chgtyp fsa880_muic_get_charger_type(void);
extern void fsa880_muic_force_charger_detection(void);
extern int fsa880_muic_get_jig_status(void);
int pmu_is_charger_inserted();

struct max8986_power {
	struct max8986 *max8986;

	/* power supplies */
	struct power_supply wall;
	struct power_supply usb;
	struct power_supply battery;
	/* current power source */
	enum power_supply_type power_src;

	/*charger type*/
	u8 charger_type;
	u8 dcd_timout;
	u8 batt_percentage;
	u8 batt_health;
	u32 batt_voltage;
	u16 batt_adc_avg;
	int temp_running_sum;
	int tem_read_inx;
	int temp_reading[ADC_RUNNING_AVG_SIZE];
	int batt_temp_adc_avg;
	int batt_temp_celsius;
	int is_charger_inserted;
	int is_jig_inserted;
	int is_timer_expired;
	int is_recharge;
	int is_volt_recharge;
	int level;
	int tmp;
	u8 volt[10];
	int button;
	int ear_adc;
	u32 temp_exceed_limit_count;

	/* battery status */
	int charging_status;
	int suspended;
	int isFullcharged;

  //#if defined(CONFIG_BATT_LVL_FROM_ADC)
	int level_running_sum;
	int level_read_inx;
	int level_reading[ADC_RUNNING_AVG_SIZE];
	int batt_level_adc_avg;
  //#endif
	struct delayed_work batt_lvl_mon_wq;
	struct timer_list	fast_charge_timer;

#if defined(CONFIG_HAS_WAKELOCK)
	struct wake_lock usb_charger_wl;
	struct wake_lock temp_adc_wl;
	struct wake_lock detect_accessory_wl;
#endif /*CONFIG_HAS_WAKELOCK*/

#if defined(CONFIG_HAS_WAKELOCK) 
	struct wake_lock jig_on_wl;
#endif /*CONFIG_HAS_WAKELOCK*/

#if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend_desc;
#endif /*CONFIG_HAS_EARLYSUSPEND*/
#ifdef MAX8986_LOG_CHARGING_TIME
	ktime_t charging_start_time;
	ktime_t charging_end_time;
	ktime_t charging_time;
	unsigned long current_time;	
	unsigned long next_expire_time;
#endif
	struct mutex power_mtx;
};

#if defined(CONFIG_CPU_FREQ_GOV_BCM21553)  
static struct cpufreq_client_desc *jig_client;
#endif

/* these typedefs should ideally be exported by KRIL header files */
typedef enum {
	/* Charger plug in event for both USB and Wall */
	EM_BATTMGR_CHARGER_PLUG_IN_EVENT,
	/* Charger plug out event */
	EM_BATTMGR_CHARGER_PLUG_OUT_EVENT,
	/* End of Charge event. Battery is full - charging is done. */
	EM_BATTMGR_ENDOFCHARGE_EVENT,
	/* BATT temp is outside window (safety) or extreme temperature */
	EM_BATTMGR_BATT_EXTREME_TEMP_EVENT,
	/* BATT low is detected */
	EM_BATTMGR_LOW_BATT_EVENT,
	/* BATT empty is detected */
	EM_BATTMGR_EMPTY_BATT_EVENT,
	/* BATT level change is detected */
	EM_BATTMGR_BATTLEVEL_CHANGE_EVENT
} HAL_EM_BATTMGR_Event_en_t;

typedef struct {
	/* The event type */
	HAL_EM_BATTMGR_Event_en_t eventType;
	/* The battery level, 0~N, depend the sysparm */
	u8 inLevel;
	/* Adc value in mV. Ex, 4000 is 4.0V, 3800 is 3.8V */
	u16 inAdc_avg;
	/* Total levels */
	u8 inTotal_levels;

} HAL_EM_BatteryLevel_t;



/*********************************************************************
 *                             DEBUG CODE                            *
 *********************************************************************/

/* Enable/disable debug logs */
enum {
	/* Disable all logging */
	DEBUG_DISABLE = 0U,
	DEBUG_FLOW    = (1U << 0),
};

#define DEFAULT_LOG_LVL    (DEBUG_DISABLE)

struct debug {
	int log_lvl;
};

#define __param_check_debug(name, p, type) \
	static inline struct type *__check_##name(void) { return (p); }

#define param_check_debug(name, p) \
	__param_check_debug(name, p, debug)

static int param_set_debug(const char *val, struct kernel_param *kp);
static int param_get_debug(char *buffer, struct kernel_param *kp);

static struct debug debug = {
	.log_lvl = DEFAULT_LOG_LVL,
};
module_param_named(debug, debug, debug, S_IRUGO | S_IWUSR | S_IWGRP);

/* helpers to test the log_lvl bitmap */
#define IS_FLOW_DBG_ENABLED	(debug.log_lvl & DEBUG_FLOW)

/* List of commands supported */
enum {
	CMD_SET_LOG_LVL = 'l',
	CMD_SHOW_BAT_STAT = 'b',
	CMD_CHARGING_CTRL = 'c',
};

static void cmd_show_usage(void)
{
	const char usage[] = "Usage:\n"
	  "echo 'cmd string' > /sys/module/max8986_power/parameters/debug\n"
	  "'cmd string' must be constructed as follows:\n"
	  "Update log level: l 0x01\n"
	  "Show battery voltage: b volt\n"
	  "Show battery temperature: b temp\n"
	  "Start battery charging: c start current (MBCCTRL4[3:0] value\n"
	  "Stop battery charging: c stop\n"
	  "Test start/stop battery charging repeatedly: c test\n";

	pr_info("%s", usage);
}

/*
 * Command handlers
 */
static void cmd_set_log_lvl(const char *p)
{
	sscanf(p, "%x", &debug.log_lvl);
}

#define AUXADC_BATVOLT_CHANNEL		3
#define AUXADC_BATTEMP_CHANNEL		0

static void cmd_show_bat_stat(const char *p)
{
	int val;

	/* Skip white spaces */
	while (*p == ' ' || *p == '\t')
		p++;

	if (strncmp("volt", p, strlen("volt")) == 0) {
		val = auxadc_access(AUXADC_BATVOLT_CHANNEL);
		pr_info("adc value for battery voltage: 0x%x\n", val);
	} else if (strncmp("temp", p, strlen("temp")) == 0) {
		val = auxadc_access(AUXADC_BATTEMP_CHANNEL);
		pr_info("adc value for battery temperature: 0x%x\n", val);
	} else {
		pr_info("invalid command\n");
	}
}

static void cmd_charging_ctrl(const char *p)
{
	int val;

	/* Skip white spaces */
	while (*p == ' ' || *p == '\t')
		p++;

	if (strncmp("start", p, strlen("start")) == 0) {
		/* Skip 'start' */
		p += strlen("start");

		/* Skip white spaces */
		while (*p == ' ' || *p == '\t')
			p++;

		/* Get current to be configured */
		sscanf(p, "%d", &val);
		pr_info("Charging current: %d\n", val);

		pmu_start_charging();
		pmu_set_charging_current(val);

	} else if (strncmp("stop", p, strlen("stop")) == 0) {
		pmu_stop_charging();
	} else if (strncmp("test", p, strlen("test")) == 0) {
		int i;

		pmu_start_charging();
		for (i = 0; i < 150; i++) {
			pmu_set_charging_current(0);
			pmu_set_charging_current(22);
		}
	} else {
		pr_info("invalid command\n");
	}
}

static int param_set_debug(const char *val, struct kernel_param *kp)
{
	const char *p;

	if (!val)
		return -EINVAL;

	/* Command is only one character followed by a space. Arguments,
	 * if any, starts from offset 2 in val.
	 */
	p = &val[2];

	switch (val[0]) {
	case CMD_SET_LOG_LVL:
		cmd_set_log_lvl(p);
		break;
	case CMD_SHOW_BAT_STAT:
		cmd_show_bat_stat(p);
		break;
	case CMD_CHARGING_CTRL:
		cmd_charging_ctrl(p);
		break;
	default:
		cmd_show_usage();
		break;
	}
	return 0;
}

static int param_get_debug(char *buffer, struct kernel_param *kp)
{
	cmd_show_usage();
	return 0;
}

/*****************************************************************************
 * power supply interface
 *****************************************************************************/
static enum power_supply_property battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TEMP, /* Temp prop is register only if a valid temp
						adc channel is specified */
};

static enum power_supply_property wall_props[] = {
	POWER_SUPPLY_PROP_ONLINE
};

static enum power_supply_property usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE
};

static int max8986_usb_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct max8986_power *max8986_power =
		dev_get_drvdata(psy->dev->parent);

	if (unlikely(!max8986_power)) {
		pr_err("%s: invalid driver data\n", __func__);
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval =
			(max8986_power->power_src == POWER_SUPPLY_TYPE_USB)
			? 1 : 0;
		break;

	default:
		pr_info("usb: property %d is not implemented\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int max8986_wall_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	int ret = 0;
	struct max8986_power *max8986_power =
		dev_get_drvdata(psy->dev->parent);

	if (unlikely(!max8986_power)) {
		pr_err("%s: invalid driver data\n", __func__);
		return -EINVAL;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval =
			(max8986_power->power_src == POWER_SUPPLY_TYPE_MAINS)
			? 1 : 0;
		break;

	default:
		pr_info("wall: property %d is not implemented\n", psp);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int max8986_battery_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	struct max8986_power *max8986_power =
		dev_get_drvdata(psy->dev->parent);
	struct linear_power_pdata *pdata;
	if (unlikely(!max8986_power || !max8986_power->max8986)) {
		pr_err("%s: invalid driver data\n", __func__);
		return -EINVAL;
	}
	pdata = max8986_power->max8986->pdata->power;
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = max8986_power->charging_status;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->batt_technology;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max8986_power->batt_percentage;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (pdata->batt_adc_tbl_t.num_entries)
			val->intval =
				pdata->batt_adc_tbl_t.bat_vol[pdata->batt_adc_tbl_t.num_entries-1]
				* 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		if (pdata->batt_adc_tbl_t.num_entries)
			val->intval = pdata->batt_adc_tbl_t.bat_vol[0] * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max8986_power->batt_voltage;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval =
			(max8986_power->power_src ==
			 POWER_SUPPLY_TYPE_BATTERY) ? 1 : 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = max8986_power->batt_health;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (max8986_power->max8986->pdata->pmu_event_cb)
			val->intval =
				max8986_power->max8986->pdata->pmu_event_cb(
						PMU_EVENT_BATT_TEMP_TO_DEGREE_C,
					max8986_power->batt_temp_adc_avg);
		else
			pr_info("%s: pmu_event_cb is not working\n", __func__);

			max8986_power->batt_temp_celsius = val->intval;
		break;
	default:
		pr_info("bat: property %d is not implemented\n", psp);
		ret = -EINVAL;
		break;
	}
	return ret;
}

/*****************************************************************************
 * pmu query functions
 *****************************************************************************/

static int max8986_is_jig_inserted(struct max8986_power *max8986_power)
{
	int ret;

	ret =  fsa880_muic_get_jig_status();
	pr_info("%s=%d", __func__,ret);
	
	return ret;

}

static int max8986_get_fc_current(struct max8986_power *max8986_power)
{
	pr_info("%s", __func__);
	return MAX8986_CHARGING_CURR_450MA;
}



static u8 max8986_get_charging_current(struct max8986_power *max8986_power,u8 charger_type, u8* supply_type)
{
	u8 cc;

	switch(charger_type)
	{
	case PMU_MUIC_CHGTYP_USB:
	case PMU_MUIC_CHGTYP_DOWNSTREAM_PORT:
		*supply_type = POWER_SUPPLY_TYPE_USB;
		cc =  USB_PREENUM_CURR_REQ_VAL;
		break;

	case PMU_MUIC_CHGTYP_DEDICATED_CHGR:
		*supply_type = POWER_SUPPLY_TYPE_MAINS;
		cc =  MAX8986_CHARGING_CURR_450MA;
		break;

	case PMU_MUIC_CHGTYP_SPL_500MA:
		*supply_type = POWER_SUPPLY_TYPE_MAINS;
		cc =  MAX8986_CHARGING_CURR_500MA;
		break;

	case PMU_MUIC_CHGTYP_SPL_1A:
		*supply_type = POWER_SUPPLY_TYPE_MAINS;
		cc =  MAX8986_CHARGING_CURR_950MA;
		break;

	case PMU_MUIC_CHGTYP_DEAD_BATT_CHG:
		*supply_type = POWER_SUPPLY_TYPE_MAINS; //??
		cc = MAX8986_CHARGING_CURR_90MA;
		break;

	default:
		*supply_type = POWER_SUPPLY_TYPE_BATTERY;
		cc = MAX8986_CHARGING_CURR_90MA;
		break;
	}
   
	return cc;
}

static int get_batt_bar_level(struct max8986_power *max8986_power)
{
	int level = 0;
	u8 percentage = 	max8986_power->batt_percentage;

	if(percentage >= 80) {
		level = 6; // 80% ~ 100%
	}
	else if(percentage >= 65) { 
		level = 5; //65% ~ 79%
	}
	else if(percentage >= 50) {	
		level = 4; //50% ~ 64%
	}
	else if(percentage >= 35) {
		level = 3; //35% ~ 49%
	}
	else if(percentage >= 20) {	
		level = 2; //20% ~ 34%
	}
	else if(percentage >= 5) {
		level = 1; // 5% ~ 19%
	}
	else {
		level = 0;  // 0% ~ 5%
	}

	return level;
}



/****************************************************************************
*
* max8986_get_batt_temperature
*
* returns: temperature measurement in ADC units or -1 on error
*
***************************************************************************/
static int max8986_get_batt_temperature(struct max8986_power *max8986_power)
{
	int temp;
	int i;
	struct linear_power_pdata *pdata =
      		  max8986_power->max8986->pdata->power;

	//max8986_power = g_ptr_data;

	if (pdata->temp_adc_channel < 0)
		return -EINVAL;

	/* get 10 bit ADC output */
	temp = auxadc_access(pdata->temp_adc_channel);
	//temp >>= 2; /* making it 8 bit */
	if (temp <= 0) {
		pr_err("%s:Error reading ADC\n", __func__);
		return -1;
	}
	/* If it is the very first measurement taken,
	 * initialize the buffer elements to the same value
	 * */
	if (max8986_power->batt_temp_adc_avg == 0) {
		max8986_power->temp_running_sum = 0;
		for (i = 0; i < ADC_RUNNING_AVG_SIZE; i++) {
			temp = auxadc_access(pdata->temp_adc_channel);
			//temp >>= 2; /* making it 8 bit */
			max8986_power->temp_reading[i] = temp;
			max8986_power->temp_running_sum += temp;
		}
		max8986_power->tem_read_inx = 0;
	}
	/* Keep the sum running forwards */
	max8986_power->temp_running_sum -=
		max8986_power->temp_reading[max8986_power->tem_read_inx];
	max8986_power->temp_reading[max8986_power->tem_read_inx] = temp;
	max8986_power->temp_running_sum += temp;
	max8986_power->tem_read_inx =
		(max8986_power->tem_read_inx + 1) % ADC_RUNNING_AVG_SIZE;

	/* Divide the running sum by number
	 * of measurements taken to get the average
	 * */
	max8986_power->batt_temp_adc_avg =
		max8986_power->temp_running_sum >> ADC_RUNNING_AVG_SHIFT;

	return max8986_power->batt_temp_adc_avg;
}

static void max8986_ctrl_charging(struct max8986_power *max8986_power, int ctrl)
{
	u8 regVal;
	struct max8986 *max8986 = max8986_power->max8986;

	pr_info("%s:%s\n", __func__,(ctrl?"START CHARGING":"STOP CHARGING"));

	if( ctrl==TRUE)
	{
#if defined(CONFIG_LINEAR_POWER)
         // start_charge
         if(max8986_power->charger_type == PMU_MUIC_CHGTYP_DEDICATED_CHGR)
         {
            gpio_direction_output(CHG_EN,0);
            udelay(100);
            gpio_direction_output(CHG_EN,1);
            udelay(300);      
            gpio_direction_output(CHG_EN,0);            
         }  
		else
            gpio_direction_output(CHG_EN,0);
#endif	
	}
	else
	{
#if defined(CONFIG_LINEAR_POWER)	
      	// stop_charge
      	gpio_direction_output(CHG_EN,1);
#endif      
	}
}

static void max8986_start_charging(struct max8986_power *max8986_power,
		int charger_type)
{
	u8 regVal;
	u8 supply_type;
	struct max8986 *max8986 = max8986_power->max8986;
	u8 charging_cc = max8986_get_charging_current(max8986_power,charger_type,&supply_type);

	if(supply_type == POWER_SUPPLY_TYPE_BATTERY)
	{
		pr_info("%s: NO charger connected !!!\n", __func__);
		return;
	}
	dwc_otg_cil_get_BC11_Charger_Type(charger_type);
	mutex_lock(&max8986_power->power_mtx);
#if defined(CONFIG_LINEAR_POWER)
		// start_charge
	if(charger_type==PMU_MUIC_CHGTYP_DEDICATED_CHGR)
	{
		gpio_direction_output(CHG_EN,0);
		udelay(100);
		gpio_direction_output(CHG_EN,1);
		udelay(300);
		gpio_direction_output(CHG_EN,0);
	}	
	else
		gpio_direction_output(CHG_EN,0);
#endif

	if((max8986_power->is_timer_expired  == FALSE) && ( !timer_pending(&max8986_power->fast_charge_timer)))
		mod_timer(&max8986_power->fast_charge_timer, jiffies + BAT_300MIN_FAST_CHARGE_TIMER);

	max8986_power->suspended = false;

	if(max8986_power->isFullcharged != TRUE)
		max8986_power->charging_status = POWER_SUPPLY_STATUS_CHARGING;

	if (max8986_power->power_src != supply_type)
	{
		max8986_power->power_src = supply_type;
		power_supply_changed((supply_type == POWER_SUPPLY_TYPE_USB)
				? &max8986_power->usb : &max8986_power->wall);
	}
	power_supply_changed(&max8986_power->battery);
#ifdef MAX8986_LOG_CHARGING_TIME
	max8986_power->charging_start_time = ktime_get();
#endif
	mutex_unlock(&max8986_power->power_mtx);
}

static void max8986_stop_charging(struct max8986_power *max8986_power,
		bool updatePwrSrc)
{
	u8 regVal;
	enum power_supply_type old_pwr_src;
	struct max8986 *max8986 = max8986_power->max8986;

	mutex_lock(&max8986_power->power_mtx);

#if defined(CONFIG_LINEAR_POWER)
		// stop_charge
		gpio_direction_output(CHG_EN,1);
#endif

	if(timer_pending(&max8986_power->fast_charge_timer))
		del_timer(&max8986_power->fast_charge_timer);

	if(max8986_power->isFullcharged != TRUE)
		max8986_power->charging_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	
	if (updatePwrSrc) {
		old_pwr_src = max8986_power->power_src;
		max8986_power->power_src = POWER_SUPPLY_TYPE_BATTERY;
		max8986_power->charging_status =
			POWER_SUPPLY_STATUS_DISCHARGING;
		if (old_pwr_src == POWER_SUPPLY_TYPE_USB)
			power_supply_changed(&max8986_power->usb);
		else if (old_pwr_src == POWER_SUPPLY_TYPE_MAINS) {
			power_supply_changed(&max8986_power->wall);
		}
	}
	power_supply_changed(&max8986_power->battery);
	mutex_unlock(&max8986_power->power_mtx);
}

static void max8986_check_batt_vf(struct max8986_power *max8986_power)
{
	int batt_vf;

	batt_vf = auxadc_access(AUXADC_BATVF_CHANNEL);
	batt_vf >>= 2;
	//power_supply_changed(&max8986_power->battery);

	if(max8986_power->is_charger_inserted){
		if( batt_vf==0xFF )
		{
			pr_info("%s: vf=%d: battery removed\n", __func__,batt_vf);
			max8986_power->batt_percentage=0;
			max8986_power->isFullcharged = FALSE;			
			max8986_power->batt_health = POWER_SUPPLY_HEALTH_DEAD;
			max8986_stop_charging(max8986_power,TRUE);			
			power_supply_changed(&max8986_power->battery);
		}
	}
		
}

static s8 max8986_get_batt_voltage(u16 batt_adc,struct batt_adc_tbl_t *batt_adc_tbl, u32 *voltage)
{
	int inx;
	u32 per = 100;
	int ratio = 0;
	*voltage = batt_adc_tbl->bat_vol[batt_adc_tbl->num_entries - 1];

	if (batt_adc < batt_adc_tbl->bat_adc[0]) {
		*voltage = batt_adc_tbl->bat_vol[0];
		pr_info("%s: Batt voltage too low\n", __func__);
		return -1; /*batt too low*/
	} else if (batt_adc >
			batt_adc_tbl->bat_adc[batt_adc_tbl->num_entries - 1]) {
		pr_info("%s: Batt voltage too high\n", __func__);
				return 1; /*beyond max limit*/
	}
	for (inx = 0; inx < (batt_adc_tbl->num_entries - 1); inx++) {
		if (batt_adc >= batt_adc_tbl->bat_adc[inx] &&
				batt_adc < batt_adc_tbl->bat_adc[inx+1]) {
			per = inx*100 +
				((batt_adc - batt_adc_tbl->bat_adc[inx])*100)/
				(batt_adc_tbl->bat_adc[inx+1]-batt_adc_tbl->bat_adc[inx]);
			per /= batt_adc_tbl->num_entries-1;
			/* using liner interpolation (lerp) */
			ratio = ((batt_adc - batt_adc_tbl->bat_adc[inx]) * 1000)/(batt_adc_tbl->bat_adc[inx+1]-batt_adc_tbl->bat_adc[inx]);
			*voltage = batt_adc_tbl->bat_vol[inx] + ((batt_adc_tbl->bat_vol[inx + 1] - batt_adc_tbl->bat_vol[inx]) * ratio)/1000;
			break;
		}
	}

	return 0;
}

/****************************************************************************
*
* bcm59035_get_batt_level
*
* returns: voltage measurement in ADC units or -1 on error
*
***************************************************************************/
//#if defined(CONFIG_BATT_LVL_FROM_ADC)
static int max8986_get_batt_level(struct max8986_power *max8986_power)
{
	int level;
	int i;
	/* Hard coding ADC channel number for battery level. This is only
	 * temporary. Once CP callbacks are supported this will be removed.
	 * */
	u8 level_adc_channel = AUXADC_BATVOLT_CHANNEL;

	/* get 10 bit ADC output */
	level = auxadc_access(level_adc_channel);
	//level = level >> 2; /* making it a 8-bit value */
	if (level <= 0) {
		pr_err("%s:Error reading ADC\n", __func__);
		return -1;
	}
	/* If it is the very first measurement taken, initialize the buffer
	*  elements to the same value
	*  */
	if (max8986_power->batt_level_adc_avg == 0) {
		max8986_power->level_running_sum = 0;
		for (i = 0; i < ADC_RUNNING_AVG_SIZE; i++) {
			level = auxadc_access(level_adc_channel);
			//level = level >> 2;
			max8986_power->level_reading[i] = level;
			max8986_power->level_running_sum += level;
		}
		max8986_power->level_read_inx = 0;
	}
	/* Keep the sum running forwards */
	max8986_power->level_running_sum -=
	max8986_power->level_reading[max8986_power->level_read_inx];
	max8986_power->level_reading[max8986_power->level_read_inx] = level;
	max8986_power->level_running_sum += level;
	max8986_power->level_read_inx =
		(max8986_power->level_read_inx + 1) % ADC_RUNNING_AVG_SIZE;

	/* Divide the running sum by number of measurements taken to get the
	* average */
	max8986_power->batt_level_adc_avg =
		max8986_power->level_running_sum >> ADC_RUNNING_AVG_SHIFT;

	return max8986_power->batt_level_adc_avg;
}

static void max8986_get_batt_level_adc(struct max8986_power *max8986_power)
{
	int batt_level;
	s8 bat_state;
	u8 bat_per = 0,old_bat_per=0;
	struct linear_power_pdata *pdata =
		max8986_power->max8986->pdata->power;
#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock(&max8986_power->temp_adc_wl);
#endif
	batt_level = max8986_get_batt_level(max8986_power);
#if defined(CONFIG_HAS_WAKELOCK)
	wake_unlock(&max8986_power->temp_adc_wl);
#endif

	max8986_power->batt_adc_avg = batt_level-BATTERY_CHARGING_HYSTERESIS;

	bat_state = max8986_get_batt_voltage(max8986_power->batt_adc_avg,&pdata->batt_adc_tbl_t, &max8986_power->batt_voltage);
	old_bat_per = max8986_power->batt_percentage;
	if (max8986_power->max8986->pdata->pmu_event_cb)
		bat_per=max8986_power->max8986->pdata->pmu_event_cb(PMU_EVENT_BATT_ADC_TO_VOLTAGE,max8986_power->batt_voltage);	
	else
		pr_info("%s: pmu_event_cb is not working\n", __func__);

	if (bat_state > 0) {
		max8986_power->batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
	} else if (max8986_power->batt_health ==POWER_SUPPLY_HEALTH_OVERVOLTAGE && is_ovp_suspended !=true) {
		max8986_power->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	}

	if (max8986_power->charging_status == POWER_SUPPLY_STATUS_FULL){
		bat_per = 100;
		prev_scaled_level=100;
	}

	if (bat_per != old_bat_per) {
		if( bat_per > old_bat_per)
		{
			//if( max8986_power->is_charger_inserted == TRUE )
			//{
				max8986_power->batt_percentage  = bat_per;
				max8986_power->level = get_batt_bar_level(max8986_power);
			//}	
		}
		else
		{
			if( max8986_power->is_charger_inserted != TRUE  || max8986_power->batt_percentage==BAT_PERCENT_INIT_VALUE )
			{
				max8986_power->batt_percentage  = bat_per;
				max8986_power->level = get_batt_bar_level(max8986_power);
			}
		}		
#if 0
		/*"N mV" = (("8-Bit_ADC_code" - 30) * 4) + 3400*/
		max8986_power->batt_voltage =
			(max8986_power->batt_adc_avg - 30)*4 + 3400 - 12;
#endif
		if (bat_state <= 0 &&
				max8986_power->batt_voltage > pdata->batt_adc_tbl_t.bat_vol[pdata->batt_adc_tbl_t.num_entries-1])
			max8986_power->batt_voltage =
				pdata->batt_adc_tbl_t.bat_vol[pdata->batt_adc_tbl_t.num_entries-1];

		pr_info("%s: Battery percentage : %d, volt = %d\n", __func__,
				max8986_power->batt_percentage,
				max8986_power->batt_voltage);
		power_supply_changed(&max8986_power->battery);
	}
}
//#endif /* CONFIG_BATT_LVL_FROM_ADC */
/*****************************************************************************
 * cp callbacks
 *****************************************************************************/
#if defined(CONFIG_BRCM_FUSE_RIL_CIB) && !defined(CONFIG_BATT_LVL_FROM_ADC)

static void inner_function(struct max8986_power *max8986_power)
{
	// 5hr_Timer
	if(max8986_power->is_timer_expired)
	{
		max8986_power->current_time = jiffies;
		pr_info("%s:%ld\n", __func__,max8986_power->current_time);

		if( time_after(max8986_power->current_time,max8986_power->next_expire_time))
		{
			pr_info("%s:recharge=%s\n", __func__,((max8986_power->is_recharge)?"recharge":"not yet"));
			if( max8986_power->is_recharge ) // Charging enable
			{
				max8986_ctrl_charging(max8986_power,TRUE);
				max8986_power->is_recharge = FALSE;
				max8986_power->next_expire_time =max8986_power->current_time+BAT_90MIN_INTERVAL_RECHARGE;
				pr_info("%s:90_min_next=%ld\n", __func__,max8986_power->next_expire_time);
			}
			else // Charging disable
			{
				max8986_ctrl_charging(max8986_power,FALSE);
				max8986_power->is_recharge = TRUE;
				max8986_power->next_expire_time = max8986_power->current_time+BAT_30SEC_INTERVAL_RECHARGE;
				pr_info("%s:30_sec_next=%ld\n", __func__,max8986_power->next_expire_time);
			}
		}		
	}
	
	// Recharge
	if( (max8986_power->isFullcharged==TRUE)&&(max8986_power->batt_voltage <= BATT_RECHARGE_VOLT))
	{
		max8986_power->is_volt_recharge=TRUE;
		max8986_start_charging(max8986_power,max8986_power->charger_type);
	}

	// Low Battery
	if(max8986_power->batt_voltage <=BATT_LOW_VOLT)
	{
		if (max8986_power->is_charger_inserted != TRUE)
		{
			max8986_power->batt_percentage=0;
			max8986_power->isFullcharged = FALSE;
			max8986_power->batt_health = POWER_SUPPLY_HEALTH_DEAD;
			max8986_stop_charging(max8986_power,TRUE);
			power_supply_changed(&max8986_power->battery);

		}
		else
		{
			max8986_power->batt_percentage=1;
			max8986_power->batt_health = POWER_SUPPLY_HEALTH_GOOD;
			max8986_start_charging(max8986_power,max8986_power->charger_type);
		}
	}
}
static void max8986_ril_adc_notify_cb(unsigned long msg_type, int result,
		void *dataBuf, unsigned long dataLength)
{
	HAL_EM_BatteryLevel_t *batt_lvl = (HAL_EM_BatteryLevel_t *) dataBuf;
	struct max8986_power *max8986_power;
	struct linear_power_pdata *pdata;
	
	u8 bat_per = 0,old_bat_per=0;
	s8 bat_state;
	u32 voltage;
	int celsius,temp;
#if 1
	if (batt_lvl == NULL || power_device == NULL) {
		pr_err("%s:Invalid params ...\n", __func__);
		return;
	}
	max8986_power = platform_get_drvdata(power_device);
	if (max8986_power == NULL) {
		pr_err("%s:Device not init\n", __func__);
		return;
	}
#endif

	if (max8986_power == NULL) {
		pr_err("%s:max8986_power is NULL\n", __func__);
		return;
	}
	pdata = max8986_power->max8986->pdata->power;
	if (pdata == NULL) {
		pr_err("%s:pdata is NULL\n", __func__);
		return;
	}	

	if(batt_lvl->inAdc_avg>=0xfff0)
	        batt_lvl->inAdc_avg=pdata->batt_adc_tbl_t.bat_adc[0]+1;
	
	switch (batt_lvl->eventType) {
	case EM_BATTMGR_BATTLEVEL_CHANGE_EVENT:
			max8986_power->batt_adc_avg = batt_lvl->inAdc_avg;
		
		bat_state = max8986_get_batt_voltage(max8986_power->batt_adc_avg,&pdata->batt_adc_tbl_t, &max8986_power->batt_voltage);
		old_bat_per = max8986_power->batt_percentage;
		//bat_per = calculate_batt_level(max8986_power->batt_voltage);
		if (max8986_power->max8986->pdata->pmu_event_cb)
			bat_per=max8986_power->max8986->pdata->pmu_event_cb(PMU_EVENT_BATT_ADC_TO_VOLTAGE,max8986_power->batt_voltage);	
		else
			pr_info("%s: pmu_event_cb is not working\n", __func__);
		
		check_func_called_by_cp=false;
		if (bat_state > 0) {
			max8986_power->batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		} else if (max8986_power->batt_health ==POWER_SUPPLY_HEALTH_OVERVOLTAGE && is_ovp_suspended !=true) {
			max8986_power->batt_health = POWER_SUPPLY_HEALTH_GOOD;
		}
		if (max8986_power->charging_status == POWER_SUPPLY_STATUS_FULL){
			bat_per = 100;
			prev_scaled_level=100;
		}

		if (bat_per != old_bat_per) {
			if( bat_per > old_bat_per)
			{
				//if( max8986_power->is_charger_inserted == TRUE )
				//{
					max8986_power->batt_percentage  = bat_per;
					max8986_power->level = get_batt_bar_level(max8986_power);
				//}	
			}
			else
			{
				if( max8986_power->is_charger_inserted != TRUE  || max8986_power->batt_percentage==BAT_PERCENT_INIT_VALUE )
				{
					max8986_power->batt_percentage  = bat_per;
					max8986_power->level = get_batt_bar_level(max8986_power);
				}
			}

			if (bat_state <= 0 &&
					max8986_power->batt_voltage > pdata->batt_adc_tbl_t.bat_vol[pdata->batt_adc_tbl_t.num_entries-1])
				max8986_power->batt_voltage =
					pdata->batt_adc_tbl_t.bat_vol[pdata->batt_adc_tbl_t.num_entries-1];

         		pr_info("%s: Battery percentage : %d, volt = %d\n", __func__,
				max8986_power->batt_percentage,
				max8986_power->batt_voltage);
			power_supply_changed(&max8986_power->battery);
		}
		inner_function(max8986_power);				

		if(pmu_is_charger_inserted() && max8986_power->batt_voltage > 4100)
		{
			if(  gpio_get_value(CHG_STATE)==SH_OFF && gpio_get_value(CHG_DET)==SH_ON 
            && max8986_power->is_timer_expired != TRUE )
			{
				max8986_power->charging_status = POWER_SUPPLY_STATUS_FULL;
				max8986_power->isFullcharged = TRUE;
				max8986_power->is_timer_expired=FALSE;		
				max8986_power->batt_percentage=100;
				prev_scaled_level=100;
				power_supply_changed(&max8986_power->battery);
				max8986_stop_charging(max8986_power,FALSE);
			}
		}
		
		break;
	case EM_BATTMGR_EMPTY_BATT_EVENT:
		pr_info("%s: low batt  event\n", __func__);
		if(!pmu_is_charger_inserted())
			max8986_power->batt_percentage = 0;
		else
			max8986_power->batt_percentage = 1;

		max8986_power->batt_voltage =BATT_LOW_VOLT;

		pr_info("Battery percentage : %d, volt = %d\n",
				max8986_power->batt_percentage,
				max8986_power->batt_voltage);
		power_supply_changed(&max8986_power->battery);
		break;
	
	default:
		break;
	}
}
#endif
/*****************************************************************************
 * charger monitoring
 *****************************************************************************/
static void max8986_batt_lvl_mon_wq(struct work_struct *work)
{
	int temp,charger_type,celsius;
	int batt_level;
	struct max8986_power *max8986_power =
		container_of(work, struct max8986_power, batt_lvl_mon_wq.work);
	struct linear_power_pdata *pdata =
		max8986_power->max8986->pdata->power;
	/*init to charging case */
	int mon_interval = BATTERY_LVL_MON_INTERVAL_WHILE_CHARGING;

	if(!pmu_is_charger_inserted())
		mon_interval = BATTERY_LVL_MON_INTERVAL;

	if( init_batt_lvl_interval<=5){
		++init_batt_lvl_interval;
		mon_interval = BATTERY_LVL_MON_INTERVAL_WHILE_CHARGING/2;
		pr_info("%s: init_batt_lvl_interval\%d\n",__func__, init_batt_lvl_interval);
	}	

#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock(&max8986_power->temp_adc_wl);
#endif
	batt_level = max8986_get_batt_level(max8986_power);
#if defined(CONFIG_HAS_WAKELOCK)
	wake_unlock(&max8986_power->temp_adc_wl);
#endif
	
	/*Do temperature monitoring if a valid temp adc channel is specified */
	if (pdata->temp_adc_channel >= 0) {
#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock(&max8986_power->temp_adc_wl);
#endif
		temp = max8986_get_batt_temperature(max8986_power);

#if defined(CONFIG_HAS_WAKELOCK)
		wake_unlock(&max8986_power->temp_adc_wl);
#endif
		if (temp < 0) {
			pr_err("%s: Error reading temp\n", __func__);
		} 
                else 
                {	
			if (max8986_power->max8986->pdata->pmu_event_cb)
					max8986_power->batt_temp_celsius=max8986_power->max8986->pdata->pmu_event_cb(PMU_EVENT_BATT_TEMP_TO_DEGREE_C,max8986_power->batt_temp_adc_avg);
			else
				pr_info("%s: pmu_event_cb is not working\n", __func__);

			celsius = max8986_power->batt_temp_celsius;
                	max8986_power->tmp = celsius/10;
			
			if ((celsius >= HIGH_SUSPEND_TEMP) ||(celsius <= LOW_SUSPEND_TEMP)) 
			{
				pr_info("%s: Batt temp(%d) beyond limit : ADC =%d\n",__func__, celsius, max8986_power->batt_temp_adc_avg);

				if (max8986_power->suspended == false) 
				{

					if (max8986_power->is_charger_inserted) 
					{
						max8986_power->batt_health =(celsius >= HIGH_SUSPEND_TEMP)
						? POWER_SUPPLY_HEALTH_OVERHEAT : POWER_SUPPLY_HEALTH_COLD;

						pr_info("%s: Temp out of range - Charging stopped\n",__func__);

						max8986_stop_charging(max8986_power,FALSE);
						max8986_power->suspended = true;	
						power_supply_changed(&max8986_power->battery);
					}
				}
			}
			else if ((celsius >= LOW_RECOVER_TEMP) && (celsius <= HIGH_RECOVER_TEMP)) 
			{
				if (max8986_power->suspended==true) 
				{
				pr_info("%s: Batt temp(%d) recovery : ADC =%d\n",__func__, celsius, max8986_power->batt_temp_adc_avg);
					max8986_power->suspended = false;
					max8986_power->batt_health =POWER_SUPPLY_HEALTH_GOOD;

					if (max8986_power->is_charger_inserted)
					{
						max8986_start_charging(max8986_power,max8986_power->power_src);
						power_supply_changed(&max8986_power->battery);
					}
				}
			}				
		}
		
	}
	/*Reading voltage level from ADC*/

	if(pmu_is_charger_inserted())
	{
		cnt_func_called++;

		if( ((check_func_called_by_cp==true) && (cnt_func_called>=20)) ||( cnt_func_called  >= 100) ||( init_batt_lvl_interval<=5) || ((check_func_called_by_cp==true) && (max8986_power->charging_status ==  POWER_SUPPLY_STATUS_FULL)))
		{
			cnt_func_called=0;
			max8986_get_batt_level_adc(max8986_power);		
		
			if( mon_interval == BATTERY_LVL_MON_INTERVAL_WHILE_CHARGING)
				check_func_called_by_cp=true;
		}
		inner_function(max8986_power);

		// Temporary EOC
		if(  gpio_get_value(CHG_STATE)==SH_OFF && gpio_get_value(CHG_DET)==SH_ON && max8986_power->batt_voltage > 4100
         && max8986_power->is_timer_expired != TRUE)
		{
			max8986_power->charging_status = POWER_SUPPLY_STATUS_FULL;
			max8986_power->isFullcharged = TRUE;
			max8986_power->is_timer_expired=FALSE;		
			max8986_power->batt_percentage=100;
			prev_scaled_level=100;
			power_supply_changed(&max8986_power->battery);
			max8986_stop_charging(max8986_power,FALSE);
		}
		else if(  gpio_get_value(CHG_STATE)==SH_OFF && gpio_get_value(CHG_DET)==SH_OFF )
		{
			/* stop charging on over-voltage*/
			max8986_power->batt_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			max8986_stop_charging(max8986_power, true);
			is_ovp_suspended = true;
		}
		else if(is_ovp_suspended == true)
		{
			/* re-start charging as charger voltage has come down to valid limit*/
			max8986_power->charger_type = fsa880_muic_get_charger_type();
			if (max8986_power->charger_type != PMU_MUIC_CHGTYP_NONE)
			{
				/* Notify through event callback */
				if(max8986_power->max8986->pdata->pmu_event_cb)
				{
					max8986_power->max8986->pdata->pmu_event_cb(PMU_EVENT_CHARGER_INSERT,
						max8986_power->charger_type);
				}
				max8986_power->batt_health = POWER_SUPPLY_HEALTH_GOOD;
				max8986_start_charging(max8986_power, max8986_power->charger_type);
			}
			is_ovp_suspended = false;
		}		
	}
	else
	{
		if (KRIL_DevSpecific_Cmd(BCM_POWER_CLIENT,RIL_DEVSPECIFICPARAM_BCM_PMU_GET_BATT_ADC,NULL, 0) == FALSE)
			pr_err("%s: KRIL_DevSpecific_Cmd failed\n", __func__);		
	}

#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock(&max8986_power->temp_adc_wl);
#endif
	max8986_check_batt_vf(max8986_power);
#if defined(CONFIG_HAS_WAKELOCK)
		wake_unlock(&max8986_power->temp_adc_wl);
#endif


	schedule_delayed_work(&max8986_power->batt_lvl_mon_wq, msecs_to_jiffies(mon_interval));

	power_supply_changed(&max8986_power->battery);

}
/*****************************************************************************
 * MUIC event handler
 *****************************************************************************/

static void max8986_muic_event(int event, u32 param,  void *data)
{
	struct max8986_power *max8986_power = data;
	struct max8986 *max8986 = max8986_power->max8986;
	pr_info("%s:event = %d param = %d\n", __func__, event,param);
	switch(event)
	{
	case MAX8986_MUIC_EVENT_CHARGER_TYPE:

		if(param == PMU_MUIC_CHGTYP_NONE)
		{
			max8986_power->dcd_timout = false;
			pr_info("%s: Charger Removed\n", __func__);

			if (max8986->pdata->pmu_event_cb)
				max8986->pdata->pmu_event_cb(PMU_EVENT_CHARGER_REMOVE,
					max8986_power->charger_type);
			else
				pr_info("%s: pmu_event_cb is not working\n", __func__);
			
			wake_lock_timeout(&max8986_power->detect_accessory_wl,WAKE_LOCK_TIME);
			max8986_power->is_charger_inserted=FALSE;
			max8986_power->isFullcharged = FALSE;
			max8986_power->is_timer_expired=FALSE;
			max8986_power->is_recharge = FALSE;
			is_ovp_suspended = false;

			max8986_stop_charging(max8986_power, true);
			max8986_power->charger_type = param;
			wake_unlock(&max8986_power->usb_charger_wl);	/* wake lock is needed only for usb */

		}
		else
		{
			if( is_ovp_suspended == true){
				pr_info("%s: ovp suspended\n", __func__);
				break;
			}
			if(max8986_power->dcd_timout)
			{
				pr_info("%s: Invalid Charger\n", __func__);
				break;
			}
			max8986_power->charger_type = param;
#if defined(CONFIG_HAS_WAKELOCK)
			//if(max8986_power->charger_type == PMU_MUIC_CHGTYP_USB ||max8986_power->charger_type == PMU_MUIC_CHGTYP_DOWNSTREAM_PORT)
				wake_lock(&max8986_power->usb_charger_wl);	/* wake lock is needed only for usb */
#endif
			max8986_power->is_charger_inserted= TRUE;
			max8986_start_charging(max8986_power, max8986_power->charger_type);

				/* Notify through event callback */
			if(max8986->pdata->pmu_event_cb)
			{
				max8986->pdata->pmu_event_cb(PMU_EVENT_CHARGER_INSERT,
						max8986_power->charger_type);
			}
			else
				pr_info("%s: pmu_event_cb is not working\n", __func__);
			
			cancel_delayed_work_sync(&max8986_power->batt_lvl_mon_wq);
			schedule_delayed_work(&max8986_power->batt_lvl_mon_wq, msecs_to_jiffies(500));		
		}
		break;

	default:
		break;
	}
}
/*****************************************************************************
 * power driver interrupt handling
 *****************************************************************************/
/* Charger insertion Check For TSP */
#if defined(CONFIG_TOUCHSCREEN_F760)
extern void set_tsp_for_ta_detect(int state);
int tsp_charger_type_status=0;
EXPORT_SYMBOL(tsp_charger_type_status);
/* Charger insertion Check For TSP */
#endif

#if 0
static void max8986_power_isr(int irq, void *data)
{
	u8 hostact;
	struct max8986_power *max8986_power = data;
	struct max8986 *max8986 = max8986_power->max8986;

	pr_info("%s:interrupt id = %u\n", __func__, irq);
	switch (irq) {
	case MAX8986_IRQID_INT2_CHGINS:
		/* Charger insertion is taken care in CHGTYP interrupt */
            #if defined(CONFIG_TOUCHSCREEN_F760)
             tsp_charger_type_status = TRUE;//TSP Charging[JG]
             set_tsp_for_ta_detect(1);//TSP Charging[JG]
            #endif
		pr_info("%s: Charger inserted\n", __func__);
		break;

	case MAX8986_IRQID_INT2_CHGRM:
		pr_info("%s: Charger Removed\n", __func__);
            #if defined(CONFIG_TOUCHSCREEN_F760)
             tsp_charger_type_status = FALSE;//TSP Charging[JG]
             set_tsp_for_ta_detect(0);//TSP Charging[JG]
             #endif
		if( max8986_power->batt_health == POWER_SUPPLY_HEALTH_DEAD){		
			max8986->read_dev(max8986, MAX8986_PM_REG_HOSTACT, &hostact);						
			hostact |= MAX8986_HOSTACT_HOSTDICOFF;
			max8986->write_dev(max8986, MAX8986_PM_REG_HOSTACT, hostact);			
		}			 
		break;

	case MAX8986_IRQID_INT2_CHGEOC:
		pr_info("%s: End of Charging\n", __func__);
		max8986_disable_irq(max8986, MAX8986_IRQID_INT2_CHGEOC);
		max8986_power->charging_status = POWER_SUPPLY_STATUS_FULL;
		max8986_power->isFullcharged = TRUE;
		max8986_power->is_timer_expired=FALSE;		
		max8986_power->is_volt_recharge=FALSE;
		max8986_power->batt_percentage=100;
		prev_scaled_level=100;
		power_supply_changed(&max8986_power->battery);
		max8986_stop_charging(max8986_power,FALSE);
#ifdef MAX8986_LOG_CHARGING_TIME
		max8986_power->charging_end_time = ktime_get();
		max8986_power->charging_time =
			ktime_sub(max8986_power->charging_end_time,
					max8986_power->charging_start_time);
		pr_info("%s:Total Charging Time	%lld us\n", __func__,
				ktime_to_us(max8986_power->charging_time));
#endif
		break;
	case MAX8986_IRQID_INT2_MBCCHGERR:
		pr_info("%s:MAX8986_IRQID_INT2_MBCCHGERR\n", __func__);
		max8986_disable_irq(max8986, MAX8986_IRQID_INT2_MBCCHGERR);
		max8986_power->charging_status = POWER_SUPPLY_STATUS_FULL;
		max8986_power->isFullcharged = FALSE;
		power_supply_changed(&max8986_power->battery);	
		max8986_power->is_timer_expired=TRUE;
		max8986_power->next_expire_time = jiffies;
		break;
	case MAX8986_IRQID_INT2_CHGERR:
		pr_info("%s:MAX8986_IRQID_INT2_CHGERR\n", __func__);
		break;

	case MAX8986_IRQID_INT3_JIGONBINS:  
		pr_info("%s:MAX8986_IRQID_INT3_JIGONBINS\n", __func__);
#if defined(CONFIG_HAS_WAKELOCK)
        	wake_lock(&max8986_power->jig_on_wl);
#endif

#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
		cpufreq_bcm_dvfs_disable(jig_client);
#endif
		break;

	case MAX8986_IRQID_INT3_JIGONBRM:
		pr_info("%s:MAX8986_IRQID_INT3_JIGONBRM\n", __func__);
#if defined(CONFIG_HAS_WAKELOCK)
       	wake_unlock(&max8986_power->jig_on_wl);
#endif

#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
		cpufreq_bcm_dvfs_enable(jig_client);
#endif
		break;

    default:
		pr_info("%s:Not supporting irq\n", __func__);
		break;
		
	} /* switch (irq) */
}
#endif

static void on_fast_charge_timer_expired(unsigned long x)
{
	struct max8986_power *max8986_power = (struct max8986_power *)x;
	
	pr_info("%s:fast_charge_timer_expired\n", __func__);
	max8986_power->batt_percentage  = 100;
	prev_scaled_level=100;
	max8986_power->charging_status = POWER_SUPPLY_STATUS_FULL;
	max8986_power->isFullcharged = FALSE;
	power_supply_changed(&max8986_power->battery);
	max8986_power->is_timer_expired=TRUE;
	max8986_power->next_expire_time = jiffies;
}

int pmu_is_charger_inserted()
{
	struct max8986_power *max8986_power;

	if(!power_device)
		return FALSE;

	max8986_power = platform_get_drvdata(power_device);	

	return max8986_power->is_charger_inserted;

}
int current_charger_status(struct max8986_power *max8986_power)
{
	switch(fsa880_muic_get_charger_type())
	{
	case PMU_MUIC_CHGTYP_USB:
	case PMU_MUIC_CHGTYP_DOWNSTREAM_PORT:
		max8986_power->charger_type = PMU_MUIC_CHGTYP_USB;
		return TRUE;
	case PMU_MUIC_CHGTYP_DEDICATED_CHGR:
		max8986_power->charger_type = PMU_MUIC_CHGTYP_DEDICATED_CHGR;
		return TRUE;
	default:		
		return FALSE;
	}
	return FALSE;
}
EXPORT_SYMBOL(pmu_is_charger_inserted);
void set_button(int value)
{
	struct max8986_power *max8986_power;

	if(!power_device)
		return;

	max8986_power = platform_get_drvdata(power_device);	

	max8986_power->button = value;
}
EXPORT_SYMBOL(set_button);
int max8986_ear_adc()
{
	int ear_adc = 0;
	struct max8986_power *max8986_power;

	if(!power_device)
		return ear_adc;

	max8986_power = platform_get_drvdata(power_device);	

	ear_adc = auxadc_access(2);
	max8986_power->ear_adc = ear_adc;
	ear_adc >>= 2;
	//power_supply_changed(&max8986_power->battery);
	pr_info("%s: ear=%d---------------\n", __func__,ear_adc);

	return ear_adc;
		
}
EXPORT_SYMBOL(max8986_ear_adc);

/*****************************************************************************
 * usb driver callbacks
 *****************************************************************************/
int pmu_get_usb_enum_current(void)
{
	int cc = MAX8986_CHARGING_CURR_UNKNOWN;
	struct max8986_power *max8986_power;
	pr_info("%s\n", __func__);
	if(!power_device)
	{
		pr_info("%s:Power driver not initialized \n", __func__);
		return cc;
	}

	max8986_power = platform_get_drvdata(power_device);
	if(max8986_power)
	{
		struct linear_power_pdata *pdata = max8986_power->max8986->pdata->power;
		switch(max8986_power->charger_type)
		{
		case PMU_MUIC_CHGTYP_USB:
		case PMU_MUIC_CHGTYP_DOWNSTREAM_PORT:
			cc = pdata->usb_charging_cc;
		break;

		case PMU_MUIC_CHGTYP_DEDICATED_CHGR:
			cc =  pdata->wac_charging_cc;
			break;

		case PMU_MUIC_CHGTYP_SPL_500MA:
			cc =  MAX8986_CHARGING_CURR_500MA;
			break;

		case PMU_MUIC_CHGTYP_SPL_1A:
			cc =  MAX8986_CHARGING_CURR_950MA;
			break;

		case PMU_MUIC_CHGTYP_DEAD_BATT_CHG:
			cc = MAX8986_CHARGING_CURR_90MA;
			break;

		default:
			break;
		}
	}
	pr_info("%s : cc = %x\n",__func__,cc);
	return cc;
}

EXPORT_SYMBOL(pmu_get_usb_enum_current);

extern void pmu_start_charging(void)
{
	struct max8986_power *max8986_power;
	if(!power_device)
	{
		pr_info("%s:Power driver not initialized \n", __func__);
		return;
	}

	max8986_power = platform_get_drvdata(power_device);
	if(max8986_power)
	{
		max8986_start_charging(max8986_power,max8986_power->charger_type);
	}
}
EXPORT_SYMBOL(pmu_start_charging);

extern void pmu_stop_charging(void)
{
	struct max8986_power *max8986_power;
	if(!power_device)
	{
		pr_info("%s:Power driver not initialized \n", __func__);
		return;
	}

	max8986_power = platform_get_drvdata(power_device);
	if(max8986_power)
	{
		max8986_stop_charging(max8986_power, false);
	}
}
EXPORT_SYMBOL(pmu_stop_charging);

pmu_charging_current pmu_get_charging_current()
{
	pmu_charging_current cc = MAX8986_CHARGING_CURR_UNKNOWN;
	struct max8986_power *max8986_power;
	if(!power_device)
	{
		pr_info("%s:Power driver not initialized \n", __func__);
		return cc;
	}

	max8986_power = platform_get_drvdata(power_device);
	if(max8986_power)
		cc = max8986_get_fc_current(max8986_power);
	pr_info("%s: cc: %d\n", __func__, cc);
	return cc;
}
EXPORT_SYMBOL(pmu_get_charging_current);

void pmu_set_usb_enum_current(bool pre_enum)
{

}
EXPORT_SYMBOL(pmu_set_usb_enum_current);

void pmu_set_charging_current(pmu_charging_current charging_cur)
{

}

EXPORT_SYMBOL(pmu_set_charging_current);

u8 max8986_read_accessory_type()
{
	pr_info("%s\n", __func__);

	return 0;

}
EXPORT_SYMBOL(max8986_read_accessory_type);


static int max8986_power_ioctl_handler(u32 cmd, u32 arg, void *pri_data)
{
	struct max8986_power *max8986_power = pri_data;
	int ret = -EINVAL;
	pr_info("Inside %s, IOCTL command %d\n", __func__, cmd);
	switch (cmd) {
	case BCM_PMU_IOCTL_START_CHARGING:
	{
		int pwr_spply_type;
		if (copy_from_user(&pwr_spply_type, (int *)arg, sizeof(int))
				!= 0) {
			pr_info("Error copying data from user\n");
			return -EFAULT;
		}
		pr_info("max8986_power_ioctl_handler: max8986_power->power_src \
				%d, pwr_spply_type %d\n",
				max8986_power->power_src, pwr_spply_type);
		if (max8986_power->power_src != pwr_spply_type)
			return -EINVAL;
		if (max8986_power->charging_status !=
				POWER_SUPPLY_STATUS_CHARGING) {
			max8986_start_charging(max8986_power,
					fsa880_muic_get_charger_type());
			ret = SUCCESS;
		} else {
			pr_info("max8986_power: already in charging mode or \
					charger is not connected\n");
			ret = -EPERM;
		}
		break;
	}
	case BCM_PMU_IOCTL_STOP_CHARGING:
	{
		if ((max8986_power->charging_status !=
					POWER_SUPPLY_STATUS_DISCHARGING) &&
				(max8986_power->charging_status !=
				 POWER_SUPPLY_STATUS_NOT_CHARGING)) {
			max8986_stop_charging(max8986_power, FALSE);
			ret = SUCCESS;
		} else {
			pr_info("max8986_power: already not in charging mode\
					\n");
			ret = -EPERM;
		}
		break;
	}
	case BCM_PMU_IOCTL_SET_CHARGING_CURRENT:
	{
		/* Not required for now */
		break;
	}
	case BCM_PMU_IOCTL_GET_CHARGING_CURRENT:
	{
		/* Not required for now */
		break;
	}
	} /* End of switch */

	return ret;
}

static int spa_Create_Attrs(struct device * dev, struct device_attribute *power_attr,  int array_size)
{
	int i; 
	int rc = EINVAL;

	for (i = 0; i < array_size; i++)
	{
		rc = device_create_file(dev, power_attr+i);
		if (rc)
		{
			device_remove_file(dev, power_attr+i);
		}
	}
	return rc;
}

static ssize_t spa_Test_Show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t spa_Test_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

#ifdef FEAT_EN_TEST_MODE
#define SPA_TEST_ATTR(_name)													\
{																				\
        .attr = { .name = #_name, .mode = S_IRUGO | S_IWUG, .owner = THIS_MODULE },	\
        .show = spa_Test_Show,														\
        .store = spa_Test_Store,														\
}

enum {
	BATT_HEALTH = 0,
	BATT_SET_CHG_TIMER,
	BATT_START_RECHARGE,
	BATT_VOLT_RECHARGE,
	BATT_CUR_VOLT,
	BATT_CUR_TEMP,
	BATT_VOLT_ADC,
	BATT_TEMP_ADC,
	BATT_CAPACITY,	
	BATT_STATUS,
	BATT_IS_FULL,
	BATT_CHARGER_TYPE,
	BATT_IS_CHG,
	BATT_IS_JIG,
	BATT_LEVEL,
	BUTTON,
	BATT_FACTORY_TEMP,
	BATT_FACTORY_VOLT,
};

static struct device_attribute spa_Test_Attrs[] = {
	SPA_TEST_ATTR(health),
	SPA_TEST_ATTR(timerExpire),
	SPA_TEST_ATTR(TimerRecharge),
	SPA_TEST_ATTR(VoltRecharge),
	SPA_TEST_ATTR(batt_vol),
	SPA_TEST_ATTR(batt_temp),
	SPA_TEST_ATTR(batt_vol_adc),	
	SPA_TEST_ATTR(batt_temp_adc),
	SPA_TEST_ATTR(capacity),	
	SPA_TEST_ATTR(status),
	SPA_TEST_ATTR(isFullcharged),
	SPA_TEST_ATTR(charger_type),
	SPA_TEST_ATTR(is_charger_inserted),
	SPA_TEST_ATTR(is_jig_inserted),	
	SPA_TEST_ATTR(level),	
	SPA_TEST_ATTR(button),	
	SPA_TEST_ATTR(tmp),	
	SPA_TEST_ATTR(volt),	
};

static ssize_t spa_Test_Show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t i = 0;
	const ptrdiff_t off = attr - spa_Test_Attrs;

	struct max8986_power *max8986_power =
		dev_get_drvdata(dev->parent);
	if (unlikely(!max8986_power || !max8986_power->max8986)) {
		pr_err("%s: invalid driver data\n", __func__);
		return -EINVAL;
	}

	switch (off)
	{
		case BATT_HEALTH:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->batt_health);
			break;
		case BATT_SET_CHG_TIMER:
			i += scnprintf(buf + i, PAGE_SIZE -i, "%d\n", max8986_power->is_timer_expired);
			break;
		case BATT_START_RECHARGE:
			i += scnprintf(buf + i, PAGE_SIZE -i, "%d\n", max8986_power->is_recharge);
			break;
		case BATT_VOLT_RECHARGE:
			i += scnprintf(buf + i, PAGE_SIZE -i, "%d\n", max8986_power->is_volt_recharge);
			break;
		case BATT_CUR_VOLT:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->batt_voltage);
			break;
		case BATT_CUR_TEMP:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->batt_temp_celsius);
			break;
		case BATT_VOLT_ADC :
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->batt_adc_avg);
			break;
		case BATT_TEMP_ADC :
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->batt_temp_adc_avg);
			break;
		case BATT_CAPACITY:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->batt_percentage);
			break;
		case BATT_STATUS:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->charging_status);
			break;
		case BATT_IS_FULL:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->isFullcharged);
			break;
		case BATT_CHARGER_TYPE:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->charger_type);
			break;
		case BATT_IS_CHG:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->is_charger_inserted);
			break;
		case BATT_IS_JIG:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->is_jig_inserted);
			break;
		case BATT_LEVEL:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->level);
			break;	
		case BUTTON:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n", ((max8986_power->button)?"ON":"OFF"));
			break;
		case BATT_FACTORY_TEMP:
			i += scnprintf(buf + i, PAGE_SIZE - i, "%d\n", max8986_power->tmp);			
			break;
		case BATT_FACTORY_VOLT:
			{
			int tmp, tmp_value;
			tmp = max8986_power->batt_voltage/10;
			tmp_value = max8986_power->batt_voltage/1000;
			scnprintf(max8986_power->volt,10,"%d.%d", tmp_value,tmp-100*tmp_value);
			i += scnprintf(buf + i, PAGE_SIZE - i, "%s\n", max8986_power->volt);
			break;
			}
		default :
			break;
	}
	
	return i;
}

static ssize_t spa_Test_Store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	ssize_t ret = 0;
	const ptrdiff_t off = attr - spa_Test_Attrs;

	switch (off)
	{
		default :
			break;
	}

	return ret;
}
#endif

static void max8986_power_early_suspend(struct early_suspend *h)
{
	struct max8986_power *max8986_power =
		container_of(h, struct max8986_power, early_suspend_desc);
	pr_info("[%s]+++\n", __func__);
	if(!max8986_power->is_charger_inserted )
	{
		pr_info("[%s] cancel delayed work \n", __func__);	
		cancel_delayed_work_sync(&max8986_power->batt_lvl_mon_wq);
		wake_unlock(&max8986_power->usb_charger_wl);	/* wake lock is needed only for usb */

	}	
	else
	{
		cancel_delayed_work_sync(&max8986_power->batt_lvl_mon_wq);
		pr_info("[%s] reschedule delayed work \n", __func__);	
		schedule_delayed_work(&max8986_power->batt_lvl_mon_wq,msecs_to_jiffies(500));	
	}
	pr_info("[%s] chg=%d, jig=%d -------------\n", __func__,max8986_power->is_charger_inserted,max8986_power->is_jig_inserted);		
	pr_info("[%s]+++\n", __func__);
}
static void max8986_power_late_resume(struct early_suspend *h)
{
	struct max8986_power *max8986_power =
		container_of(h, struct max8986_power, early_suspend_desc);
	pr_info("[%s]+++\n", __func__);
	schedule_delayed_work(&max8986_power->batt_lvl_mon_wq,
			msecs_to_jiffies(500));
	max8986_power->batt_temp_adc_avg=0;
	pr_info("[%s]+++\n", __func__);
}

/*****************************************************************************
 * init code
 *****************************************************************************/
static void max8986_init_charger(struct max8986_power *max8986_power)
{
	u8 reg_val;
	struct max8986 *max8986 = max8986_power->max8986;

	/*Register for PM interrupts */
#if 0
	max8986_request_irq(max8986, MAX8986_IRQID_INT2_CHGEOC, FALSE,
		max8986_power_isr, max8986_power);
	max8986_request_irq(max8986, MAX8986_IRQID_INT2_CHGINS, TRUE,
		max8986_power_isr, max8986_power);
	max8986_request_irq(max8986, MAX8986_IRQID_INT2_CHGERR, FALSE,
		max8986_power_isr, max8986_power);
	max8986_request_irq(max8986, MAX8986_IRQID_INT2_CHGRM, TRUE,
		max8986_power_isr, max8986_power);
	max8986_request_irq(max8986, MAX8986_IRQID_INT2_MBCCHGERR, FALSE,
		max8986_power_isr, max8986_power);
	max8986_request_irq(max8986, MAX8986_IRQID_INT3_VERYLOWBAT, TRUE,
		max8986_power_isr, max8986_power);
	max8986_request_irq(max8986, MAX8986_IRQID_INT3_JIGONBINS, TRUE,
		max8986_power_isr, max8986_power);
	max8986_request_irq(max8986, MAX8986_IRQID_INT3_JIGONBRM, TRUE,
		max8986_power_isr, max8986_power);
#endif
	//fsa880_muic_register_event_handler(MAX8986_MUIC_EVENT_CHARGER_OVP, max8986_muic_event,max8986_power);
	//fsa880_muic_register_event_handler(MAX8986_MUIC_EVENT_DCDTMR, max8986_muic_event,max8986_power);
	fsa880_muic_register_event_handler(MAX8986_MUIC_EVENT_CHARGER_TYPE, max8986_muic_event,max8986_power);

}

static int __devinit linear_power_probe(struct platform_device *pdev)
{
	struct d2041 *d2041 = platform_get_drvdata(pdev);
	u8 reg_val;
	struct max8986_power *max8986_power;
	struct max8986 *max8986;
	int ret = 0;
	struct linear_power_pdata *power_pdata;
	
#if defined(CONFIG_BRCM_FUSE_RIL_CIB) && !defined(CONFIG_BATT_LVL_FROM_ADC)
	unsigned long notify_id_list[] = { RIL_NOTIFY_DEVSPECIFIC_BATT_LEVEL };
#endif

	pr_info("%s\n", __func__);

	max8986 = kzalloc(sizeof(struct max8986), GFP_KERNEL);
	if (unlikely(!max8986)) {
		pr_err("%s: max8986 memory alloc failed\n", __func__);
		return -ENOMEM;
	}	

	max8986_power = kzalloc(sizeof(struct max8986_power), GFP_KERNEL);
	if (unlikely(!max8986_power)) {
		pr_err("%s: max8986_power memory alloc failed\n", __func__);
		kfree(max8986);
		return -ENOMEM;
	}

	max8986->pdata = d2041->pdata;
	max8986->pdata->power = d2041->pdata->power;
	max8986->pdata->pmu_event_cb =d2041->pdata->pmu_event_cb;
	max8986_power->max8986 = max8986;

	if (unlikely(!max8986->pdata || !max8986->pdata->power)) {
		pr_err("%s: invalid platform data\n", __func__);
		kfree(max8986_power);
		kfree(max8986);
		return -EINVAL;
	}
	
	platform_set_drvdata(pdev, max8986_power);
	power_device = pdev;
	power_pdata = max8986->pdata->power;

	//g_ptr_data = max8986_power;
	
#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
	jig_client = cpufreq_bcm_client_get("jig_client");
	if (!jig_client) {
		pr_err("%s: cpufreq_bcm_client_get failed\n", __func__);
		return -EIO;
	}
#endif
#if defined(CONFIG_HAS_WAKELOCK)
    // Prevent the full system suspend.
	wake_lock_init(&max8986_power->jig_on_wl, WAKE_LOCK_SUSPEND,
						__stringify(jig_on_wl));
#endif	

#if defined(CONFIG_HAS_WAKELOCK)
	wake_lock_init(&max8986_power->usb_charger_wl, WAKE_LOCK_SUSPEND,
						__stringify(usb_charger_wl));
	wake_lock_init(&max8986_power->temp_adc_wl, WAKE_LOCK_IDLE,
		 __stringify(temp_adc_wl));
	wake_lock_init(&max8986_power->detect_accessory_wl, WAKE_LOCK_SUSPEND,
						__stringify(detect_accessory_wl));
	
#endif

	mutex_init(&max8986_power->power_mtx);

	/*set voltage to 20% by default */
	max8986_power->batt_percentage = BAT_PERCENT_INIT_VALUE;
	//max8986_power->batt_voltage = power_pdata->batt_adc_tbl.bat_vol[2]*1000;
	max8986_power->batt_voltage = 3500;	
	max8986_power->batt_health = POWER_SUPPLY_HEALTH_GOOD;
	max8986_power->level = 0;	
	max8986_power->isFullcharged = FALSE;
	max8986_power->is_charger_inserted= FALSE;
	max8986_power->is_jig_inserted= FALSE;
	max8986_power->is_timer_expired=FALSE;
	max8986_power->is_recharge=FALSE;
	max8986_power->is_volt_recharge=FALSE;
	max8986_power->suspended = FALSE;
	
	INIT_DELAYED_WORK(&max8986_power->batt_lvl_mon_wq,
		max8986_batt_lvl_mon_wq);

	init_timer(&max8986_power->fast_charge_timer);
	max8986_power->fast_charge_timer.function = on_fast_charge_timer_expired;
	max8986_power->fast_charge_timer.data = (unsigned long)(unsigned long*)(max8986_power);

	max8986_power->power_src = POWER_SUPPLY_TYPE_BATTERY;
	max8986_power->charging_status = POWER_SUPPLY_STATUS_DISCHARGING;

	/*register power supplies */
	//max8986_power->wall.name = "max8986-wall";
	max8986_power->wall.name = "battery";
	max8986_power->wall.type = POWER_SUPPLY_TYPE_MAINS;
	max8986_power->wall.properties = wall_props;
	max8986_power->wall.num_properties = ARRAY_SIZE(wall_props);
	max8986_power->wall.get_property = max8986_wall_get_property;
	ret = power_supply_register(&pdev->dev, &max8986_power->wall);
	if (ret) {
		pr_err("%s: wall charger registration failed\n", __func__);
		goto wall_err;
	}

	max8986_power->battery.name = "max8986-battery";
	max8986_power->battery.properties = battery_props;
	max8986_power->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	/* Temp property is kept as the last entry in battery_props array.
	 * Temp prop is registered only if a valid
	 * temp adc channel is specified in platform data
	 * */
	max8986_power->battery.num_properties =ARRAY_SIZE(battery_props);
	max8986_power->battery.get_property = max8986_battery_get_property;
	ret = power_supply_register(&pdev->dev, &max8986_power->battery);
	if (ret) {
		pr_err("%s: battery registration failed\n", __func__);
		goto batt_err;
	}

	max8986_power->usb.name = "max8986-usb",
	    max8986_power->usb.type = POWER_SUPPLY_TYPE_USB;
	max8986_power->usb.properties = usb_props;
	max8986_power->usb.num_properties = ARRAY_SIZE(usb_props);
	max8986_power->usb.get_property = max8986_usb_get_property;
	ret = power_supply_register(&pdev->dev, &max8986_power->usb);
	if (ret) {
		pr_err("%s: usb power supply registration failed\n", __func__);
		goto usb_err;
	}
	/* init pmu */
	max8986_init_charger(max8986_power);

#if defined(CONFIG_BRCM_FUSE_RIL_CIB) && !defined(CONFIG_BATT_LVL_FROM_ADC)
	if (KRIL_Register(BCM_POWER_CLIENT, NULL,
		max8986_ril_adc_notify_cb, notify_id_list,
		ARRAY_SIZE(notify_id_list)) == FALSE) {
		pr_err("%s: KRIL_Register failed\n", __func__);
	}
#else
	pr_err("%s:KRIL_Register not defined\n", __func__);
#endif
	/* start the workqueue */
#if 0
	queue_delayed_work(max8986_power->max8986->pmu_workqueue,
		&max8986_power->batt_lvl_mon_wq, msecs_to_jiffies(500));
#else
	schedule_delayed_work(&max8986_power->batt_lvl_mon_wq, msecs_to_jiffies(500));

#endif
	//max8986_register_ioctl_handler(max8986, MAX8986_SUBDEV_POWER,max8986_power_ioctl_handler, max8986_power);
#if defined(CONFIG_HAS_EARLYSUSPEND)
	max8986_power->early_suspend_desc.level =
		EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	max8986_power->early_suspend_desc.suspend = max8986_power_early_suspend;
	max8986_power->early_suspend_desc.resume = max8986_power_late_resume;
	register_early_suspend(&max8986_power->early_suspend_desc);
#endif /*CONFIG_HAS_EARLYSUSPEND*/

#ifdef FEAT_EN_TEST_MODE
	spa_Create_Attrs(max8986_power->wall.dev, spa_Test_Attrs, ARRAY_SIZE(spa_Test_Attrs));
#endif
	max8986_power->is_timer_expired=FALSE;
	max8986_power->is_recharge=FALSE;

	if( max8986_is_jig_inserted(max8986_power))
	{
		max8986_power->is_jig_inserted = TRUE;
#if defined(CONFIG_HAS_WAKELOCK)
        	wake_lock(&max8986_power->jig_on_wl);
#endif

#ifdef CONFIG_CPU_FREQ_GOV_BCM21553
		cpufreq_bcm_dvfs_disable(jig_client);
#endif
	pr_info("%s jig inserted at init time\n", __func__);
	}
    	else
		max8986_power->is_jig_inserted = FALSE;

	if(current_charger_status(max8986_power)){
		max8986_power->is_charger_inserted = TRUE;
      if(max8986->pdata->pmu_event_cb)
      {
         max8986->pdata->pmu_event_cb(PMU_EVENT_CHARGER_INSERT, max8986_power->charger_type);
      }
      else
         pr_info("%s: pmu_event_cb is not working\n", __func__);
      
		max8986_start_charging(max8986_power, max8986_power->charger_type);
		wake_lock(&max8986_power->usb_charger_wl);	
	}
	else{
		max8986_power->is_charger_inserted = FALSE;		
      if(max8986->pdata->pmu_event_cb)
      {
         max8986->pdata->pmu_event_cb(PMU_EVENT_CHARGER_REMOVE, PMU_MUIC_CHGTYP_NONE);	
      }
      else
         pr_info("%s: pmu_event_cb is not working\n", __func__);
		max8986_ctrl_charging(max8986_power,FALSE);
	}

	/* if usb/wall charger is already connected, then start the charging
	 * process by simulating charger insertion interrupt.
	 */
	pr_info("%s: success\n", __func__);
	return 0;

usb_err:
	power_supply_unregister(&max8986_power->battery);
batt_err:
	power_supply_unregister(&max8986_power->wall);
wall_err:
	kfree(max8986_power);
	return ret;
}

static int __devexit linear_power_remove(struct platform_device *pdev)
{
	struct max8986_power *max8986_power = platform_get_drvdata(pdev);

	if (max8986_power) {
		if(timer_pending(&max8986_power->fast_charge_timer))
			del_timer(&max8986_power->fast_charge_timer);		
		cancel_delayed_work_sync(&max8986_power->batt_lvl_mon_wq);
		power_supply_unregister(&max8986_power->wall);
		power_supply_unregister(&max8986_power->usb);
		power_supply_unregister(&max8986_power->battery);
#if defined(CONFIG_HAS_EARLYSUSPEND)
		unregister_early_suspend(&max8986_power->early_suspend_desc);
#endif
#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock_destroy(&max8986_power->temp_adc_wl);
		wake_lock_destroy(&max8986_power->usb_charger_wl);
		wake_lock_destroy(&max8986_power->detect_accessory_wl);		
#endif
#if defined(CONFIG_HAS_WAKELOCK)
		wake_lock_destroy(&max8986_power->jig_on_wl);
#endif
	}
	kfree(max8986_power);
	return 0;
}

static struct platform_driver linear_power_driver = {
	.driver = {
		.name = "linear-power",
		.owner = THIS_MODULE,
	},
	.remove = __devexit_p(linear_power_remove),
	.probe = linear_power_probe,
};

static int __init linear_power_init(void)
{
	return platform_driver_register(&linear_power_driver);
}

late_initcall(linear_power_init);

static void __exit linear_power_exit(void)
{
	platform_driver_unregister(&linear_power_driver);
}

module_exit(linear_power_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Power Driver for Maxim MAX8986 PMU");
MODULE_ALIAS("platform:max8986-power");
