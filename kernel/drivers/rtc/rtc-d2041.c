/*
 *	Real Time Clock driver for Dialog D2041
 *
 *	Copyright (C) 2011 Dialog Semiconductor Ltd.
 *
 *  	Author: D. Chen, D. Patel
 *
 *  	This program is free software; you can redistribute  it and/or modify it
 *  	under  the terms of  the GNU General  Public License as published by the
 *  	Free Software Foundation;  either version 2 of the  License, or (at your
 *  	option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/completion.h>
#include <linux/d2041/rtc.h>
#include <linux/d2041/d2041_reg.h> 
#include <linux/d2041/core.h>
#include <linux/delay.h>
#include <linux/platform_device.h>

#define DRIVER_NAME "d2041-rtc"

#define to_d2041_from_rtc_dev(d) container_of(d, struct d2041, rtc.pdev.dev)

#define D2041_YEAR_BASE		100	/*2000 - 1900 */
#define SEC_YEAR_BASE 			12  /* 2012 */

static struct rtc_time init_rtc_time =
{
.tm_sec = 0xca,
.tm_min = 0xfe,
.tm_hour = 0xca,
.tm_mday = 0xfe,
.tm_mon = 0xca,
.tm_year = 0xfe,
.tm_wday = 0xca,
.tm_yday = 0xfe,
.tm_isdst =0xca
};

static struct rtc_time history_rtc_time[1024];
static unsigned int history_rtc_time_index = 0;

static int d2041_rtc_check_param(struct rtc_time *rtc_tm)
{
	if ((rtc_tm->tm_sec > D2041_RTC_SECONDS_LIMIT) || (rtc_tm->tm_sec < 0))
		return -D2041_RTC_INVALID_SECONDS;

	if ((rtc_tm->tm_min > D2041_RTC_MINUTES_LIMIT) || (rtc_tm->tm_min < 0))
		return -D2041_RTC_INVALID_MINUTES;

	if ((rtc_tm->tm_hour > D2041_RTC_HOURS_LIMIT) || (rtc_tm->tm_hour < 0))
		return -D2041_RTC_INVALID_HOURS;

	if (rtc_tm->tm_mday == 0)
		return -D2041_RTC_INVALID_DAYS;

	if ((rtc_tm->tm_mon > D2041_RTC_MONTHS_LIMIT) || (rtc_tm->tm_mon <= 0))
		return -D2041_RTC_INVALID_MONTHS;

	if ((rtc_tm->tm_year > D2041_RTC_YEARS_LIMIT) || (rtc_tm->tm_year < 0))
		return -D2041_RTC_INVALID_YEARS;

	return 0;
}

/*
 * Read current time and date in RTC
 */
static int d2041_rtc_readtime(struct device *dev, struct rtc_time *tm)
{
	struct d2041 *d2041 = dev_get_drvdata(dev);
	u8 rtc_time1[6];
	int ret = 0;

	ret = d2041_block_read(d2041, D2041_COUNTS_REG, 6, rtc_time1);
	if (ret < 0)
	{
		dlg_err("%s: read error %d\n", __func__, ret);
		return ret;
	}

	tm->tm_sec  = rtc_time1[0] & D2041_RTC_SECS_MASK;
	tm->tm_min  = rtc_time1[1] & D2041_RTC_MINS_MASK;
	tm->tm_hour = rtc_time1[2] & D2041_RTC_HRS_MASK;
	tm->tm_mday = rtc_time1[3] & D2041_RTC_DAY_MASK;
	tm->tm_mon  = rtc_time1[4] & D2041_RTC_MTH_MASK;
	tm->tm_year = rtc_time1[5] & D2041_RTC_YRS_MASK;

	dlg_info("%s : RTC register %02x-%02x-%02x %02x:%02x:%02x\n", __func__, 
				rtc_time1[5], rtc_time1[4], rtc_time1[3], rtc_time1[2], rtc_time1[1], rtc_time1[0]);
	
	 /* sanity checking */
	ret = d2041_rtc_check_param(tm);
	if (ret)
	{
		dlg_err("%s : check error %d\n", __func__, ret);
		return ret;
	}

	tm->tm_year += D2041_YEAR_BASE;
	tm->tm_mon -= 1;
	tm->tm_yday = rtc_year_days(tm->tm_mday, tm->tm_mon, tm->tm_year);

	return ret;
}

/*
 * Set current time and date in RTC
 */
static int d2041_rtc_settime(struct device *dev, struct rtc_time *tm)
{
	struct d2041 *d2041 = dev_get_drvdata(dev);
	u8 rtc_time[6];
	int ret = 0;
	
	rtc_time[0] = tm->tm_sec;
	rtc_time[1] = tm->tm_min;
	rtc_time[2] = tm->tm_hour;
	rtc_time[3] = tm->tm_mday;
	rtc_time[4] = tm->tm_mon + 1;
	rtc_time[5] = tm->tm_year - D2041_YEAR_BASE;
	rtc_time[5] |= D2041_COUNTY_MONITOR;

	dlg_info("%s : RTC register %02x-%02x-%02x %02x:%02x:%02x\n", __func__, 
		rtc_time[0], rtc_time[1], rtc_time[2], rtc_time[3], rtc_time[4], rtc_time[5]);

	/* Write time to RTC */
	ret = d2041_block_write(d2041, D2041_COUNTS_REG, 6, rtc_time);
	if (ret < 0)
		dlg_err("%s: write error %d\n", __func__, ret);

   memcpy(&history_rtc_time[history_rtc_time_index++], tm, sizeof(struct rtc_time));
   if(history_rtc_time_index >= 1024)
      history_rtc_time_index = 0;      

	return 0;
}

static int d2041_rtc_stop_alarm(struct d2041 *d2041)
{
	int ret;
	u8 time[6];

	dlg_info("%s \n", __func__);

	/* For latching the data before RMW */
	ret = d2041_block_read(d2041, D2041_ALARMS_REG, 6, time);
	if (ret < 0)
		return ret;

	/* Set RTC_SET to stop the clock */
	ret = d2041_clear_bits(d2041, D2041_ALARMY_REG, D2041_ALARMY_ALARMON);
	if (ret < 0)
		return ret;

	return 0;      
}


static int d2041_rtc_start_alarm(struct d2041 *d2041)
{
	int ret;
	u8 time[6];

	dlg_info("%s \n", __func__);

	/* For latching the data before RMW */
	ret = d2041_block_read(d2041, D2041_ALARMS_REG, 6, time);
	if (ret < 0)
		return ret;

	ret = d2041_set_bits(d2041, D2041_ALARMY_REG, D2041_ALARMY_ALARMON);
	if (ret < 0)
		return ret;

	return 0;
}

/*
 * Read alarm time and date in RTC
 */
static int d2041_rtc_readalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct d2041 *d2041 = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	u8 time[6];
	int ret;

	ret = d2041_block_read(d2041, D2041_ALARMS_REG, 6, time);
	if (ret < 0)
	{
		dlg_err("%s: read error %d\n", __func__, ret);
		return ret;
	}

	tm->tm_sec = time[0] & D2041_RTC_ALMSECS_MASK;	
	tm->tm_min = time[1] & D2041_RTC_ALMMINS_MASK;
	tm->tm_hour = time[2] & D2041_RTC_ALMHRS_MASK;	
	tm->tm_mday = time[3] & D2041_RTC_ALMDAY_MASK;
	tm->tm_mon = time[4] & D2041_RTC_ALMMTH_MASK;	
	tm->tm_year = time[5] & D2041_RTC_ALMYRS_MASK;

	ret = d2041_rtc_check_param(tm);
	if (ret < 0)
	{
		dlg_err("%s : check error %d\n", __func__, ret);
		return ret;
	}
    
	tm->tm_year += D2041_YEAR_BASE;
	tm->tm_mon -= 1;

	dlg_info("%s : RTC register %02x-%02x-%02x %02x:%02x:%02x\n", __func__, 
                        time[5], time[4], time[3], time[2], time[1], time[0]);
	
	return 0;
}

static int d2041_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct d2041 *d2041 = dev_get_drvdata(dev);
	struct rtc_time *tm = &alrm->time;
	struct rtc_time alarm_tm;
	u8 time[6], rtc_ctrl, reg_val;
	int ret;

	/* Set RTC_SET to stop the clock */
	ret = d2041_clear_bits(d2041, D2041_ALARMY_REG, D2041_ALARMY_ALARMON);
	if (ret < 0)
	{
		dlg_err("%s: clear bits error %d\n", __func__, ret);
		return ret;
	}

	memcpy(&alarm_tm, &alrm->time, sizeof(struct rtc_time));

	alarm_tm.tm_year -= D2041_YEAR_BASE;
	alarm_tm.tm_mon += 1;

	ret = d2041_rtc_check_param(&alarm_tm);
	if (ret < 0)
	{
		dlg_err("%s : check error %d\n", __func__, ret);
		return ret;
	}
	
	memset(time, 0, sizeof(time));

	time[0] = alarm_tm.tm_sec;
	d2041_reg_read(d2041, D2041_ALARMMI_REG, &reg_val);
	rtc_ctrl = reg_val & (~D2041_RTC_ALMMINS_MASK);
	time[1] = rtc_ctrl | alarm_tm.tm_min;
	time[2] |= alarm_tm.tm_hour;
	time[3] |= alarm_tm.tm_mday;
	time[4] |= alarm_tm.tm_mon;
	d2041_reg_read(d2041, D2041_ALARMY_REG, &reg_val);
	rtc_ctrl = reg_val & (~D2041_RTC_ALMYRS_MASK);
	time[5] = rtc_ctrl | alarm_tm.tm_year;

	/* Write time to RTC */
	ret = d2041_block_write(d2041, D2041_ALARMS_REG, 6, time);
	if (ret < 0)
	{
		dlg_err("%s: write error %d\n", __func__, ret);
	    	return ret;
	}

	if (alrm->enabled)
		ret = d2041_rtc_start_alarm(d2041);

	dlg_info("%s : RTC register %02x-%02x-%02x %02x:%02x:%02x\n", __func__, 
                        time[5], time[4], time[3], time[2], time[1], time[0]);

    return ret;
}

static int d2041_rtc_alarm_irq_enable(struct device *dev,
				  unsigned int enabled)
{
	struct d2041 *d2041 = dev_get_drvdata(dev);

	dlg_info("%s \n", __func__);

	if (enabled)
		return d2041_rtc_start_alarm(d2041);
	else
		return d2041_rtc_stop_alarm(d2041);
}

static int d2041_rtc_update_irq_enable(struct device *dev,
					unsigned int enabled)
{
	int ret;
	struct d2041 *d2041 = dev_get_drvdata(dev);
	u8 time[6];

	dlg_info("%s \n", __func__);
	
	/* For latching the data before RMW */
	ret = d2041_block_read(d2041, D2041_ALARMS_REG, 6, time);
	if (ret < 0)
		return ret;

	if (enabled) {
		ret = d2041_set_bits(d2041, D2041_ALARMY_REG, D2041_ALARMY_TICKON);
		if (ret < 0)
			return ret;
	} else {
		ret = d2041_clear_bits(d2041, D2041_ALARMY_REG, D2041_ALARMY_TICKON);
		if (ret < 0)
			return ret;
        }
	return 0;
}

static irqreturn_t d2041_rtc_timer_alarm_handler(int irq, void *data)
{
	struct d2041 *d2041 = data;
	struct rtc_device *rtc = d2041->rtc.rtc;

	rtc_update_irq(rtc, 1, RTC_AF | RTC_IRQF);
    	dlg_info("\nRTC: TIMER ALARM\n");
	return IRQ_HANDLED;
}

static irqreturn_t d2041_rtc_tick_alarm_handler(int irq, void *data)
{
	struct d2041 *d2041 = data;
	struct rtc_device *rtc = d2041->rtc.rtc;
	
	rtc_update_irq(rtc, 1, RTC_PF | RTC_IRQF);
	dlg_info("RTC: TICK ALARM\n");
	return IRQ_HANDLED;
}

static const struct rtc_class_ops d2041_rtc_ops = {
	.read_time = d2041_rtc_readtime,
	.set_time = d2041_rtc_settime,
	.read_alarm = d2041_rtc_readalarm,
	.set_alarm = d2041_rtc_setalarm,
	.alarm_irq_enable = d2041_rtc_alarm_irq_enable,
	.update_irq_enable = d2041_rtc_update_irq_enable,
};

#ifdef CONFIG_PM
static int d2041_rtc_suspend(struct device *dev)
{
	return 0;
}
static int d2041_rtc_resume(struct device *dev)
{
#if defined(CONFIG_RTC_ANDROID_ALARM_WORKAROUND)
	/* This option selects temporary fix for alarm handling in 'Android'
	 * environment. This option enables code to disable alarm in the
	 * 'resume' handler of RTC driver. In the normal mode,
	 * android handles all alarms in software without using the RTC chip.
	 * Android sets the alarm in the rtc only in the suspend path (by
	 * calling .set_alarm with struct rtc_wkalrm->enabled set to 1).
	 * In the resume path, android tries to disable alarm by calling
	 * .set_alarm with struct rtc_wkalrm->enabled' field set to 0.
	 * But unfortunately, it memsets the rtc_wkalrm struct to 0, which
	 * causes the rtc lib to flag error and control does not reach this
	 * driver. Hence this workaround.
	 */
	d2041_rtc_alarm_irq_enable(dev, 0);
#endif
	return 0; 
}
#else
#define d2041_rtc_suspend NULL
#define d2041_rtc_resume NULL
#endif

static void d2041_rtc_time_fixup(struct device *dev)
{
	struct rtc_time current_rtc_time;
   memset(&current_rtc_time, 0 , sizeof(struct rtc_time));

	d2041_rtc_readtime(dev, &current_rtc_time);
	current_rtc_time.tm_year += SEC_YEAR_BASE;
   memcpy(&init_rtc_time, &current_rtc_time, sizeof(struct rtc_time));
	d2041_rtc_settime(dev, &current_rtc_time);
}

static int d2041_rtc_probe(struct platform_device *pdev)
{
	struct d2041 *d2041 = platform_get_drvdata(pdev);
	struct d2041_rtc *dlg_rtc = &d2041->rtc;
	int ret = 0;
	u8 reg_val;

	dlg_info("Starting RTC\n");
	device_init_wakeup(&pdev->dev, 1);

	dlg_rtc->rtc = rtc_device_register("d2041", &pdev->dev,
					  &d2041_rtc_ops, THIS_MODULE);
	if (IS_ERR(dlg_rtc->rtc)) {
		ret = PTR_ERR(dlg_rtc->rtc);
		dlg_err("failed to register RTC: %d\n", ret);
		return ret;
	}

	ret = d2041_reg_read(d2041, D2041_COUNTY_REG,&reg_val);
	if(ret < 0)
		reg_val = 0;	

	if (!(reg_val & D2041_COUNTY_MONITOR))
	{
		dlg_info("Fixup RTC\n");
		d2041_set_bits(d2041, D2041_COUNTY_REG, D2041_COUNTY_MONITOR);
		d2041_rtc_time_fixup(&pdev->dev);
	}
        
	d2041_register_irq(d2041, D2041_IRQ_EALRAM, d2041_rtc_timer_alarm_handler, 
                            0, "RTC Timer Alarm", d2041);
#if 0
	d2041_register_irq(d2041, D2041_IRQ_ETICK, d2041_rtc_tick_alarm_handler, 
                            0, "RTC Tick Alarm", d2041);
#endif
	dev_info(d2041->dev, "\nRTC registered\n");

	return 0;
}

static int __devexit d2041_rtc_remove(struct platform_device *pdev)
{
	struct d2041 *d2041 = platform_get_drvdata(pdev);
	struct d2041_rtc *dlg_rtc = &d2041->rtc;
	d2041_free_irq(d2041, D2041_IRQ_EALRAM);
	//d2041_free_irq(d2041, D2041_IRQ_ETICK);
	rtc_device_unregister(dlg_rtc->rtc);

	return 0;
}

static struct dev_pm_ops d2041_rtc_pm_ops = {
	.suspend = d2041_rtc_suspend,
	.resume = d2041_rtc_resume,
	.thaw = d2041_rtc_resume,
	.restore = d2041_rtc_resume,
	.poweroff = d2041_rtc_suspend,
};

static struct platform_driver d2041_rtc_driver = {
	.probe = d2041_rtc_probe,
	.remove = __devexit_p(d2041_rtc_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &d2041_rtc_pm_ops,
	},
};

static int __init d2041_rtc_init(void)
{
	return platform_driver_register(&d2041_rtc_driver);
}
module_init(d2041_rtc_init);

static void __exit d2041_rtc_exit(void)
{
	platform_driver_unregister(&d2041_rtc_driver);
}
module_exit(d2041_rtc_exit);

MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("RTC driver for the Dialog D2041 PMIC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
