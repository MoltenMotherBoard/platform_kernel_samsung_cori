/*
 * core.h  --  Core Driver for Dialog Semiconductor D2041 PMIC
 *
 * Copyright 2011 Dialog Semiconductor Ltd
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __LINUX_D2041_CORE_H_
#define __LINUX_D2041_CORE_H_

#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/d2041/pmic.h>
#include <linux/d2041/rtc.h>
#include <linux/d2041/audio.h>
#include <plat/bcm_i2c.h>

/*
 * Register values.
 */  

#define I2C						            1

#define D2041_I2C					        "d2041"

//#define D2041_IRQ					        S3C_EINT(9)

/* Module specific error codes */
#define INVALID_REGISTER				    2
#define INVALID_READ					    3
#define INVALID_PAGE					    4

/* Total number of registers in D2041 */
#define D2041_MAX_REGISTER_CNT				(D2041_PAGE1_REG_END+1)


#define D2041_I2C_DEVICE_NAME				"d2041_i2c"
#define D2041_I2C_ADDR					    (0x90 >> 1)


/*
 * DA1980 Number of Interrupts
 */

//enum d2041_irq_num { //TODO MW: change to enum
#define D2041_IRQ_EVDDMON				    0	 
#define D2041_IRQ_EALRAM				    1	
#define D2041_IRQ_ESEQRDY				    2	
#define D2041_IRQ_ETICK					    3	

#define D2041_IRQ_ENONKEY_LO			    4	
#define D2041_IRQ_ENONKEY_HI			    5	
#define D2041_IRQ_ENONKEY_HOLDON		    6	
#define D2041_IRQ_ENONKEY_HOLDOFF		    7

#define D2041_IRQ_ETA					    8	
#define D2041_IRQ_ENJIGON				    9	

#define D2041_IRQ_EGPI0					    10	

#define D2041_NUM_IRQ					    11

//#ifdef CONFIG_BOARD_CORI
#define D2041_IOCTL_REGL_MAPPING_NUM		23	
//#endif /* CONFIG_BOARD_CORI */

#define D2041_MCTL_MODE_INIT(_reg_id, _dsm_mode, _default_pm_mode) \
    [_reg_id] = { \
        .reg_id = _reg_id, \
        .dsm_opmode = _dsm_mode, \
        .default_pm_mode = _default_pm_mode, \
    }

// for DEBUGGING and Troubleshooting
#if 1	//defined(DEBUG)
#define dlg_crit(fmt, ...) \
			printk(KERN_CRIT fmt, ##__VA_ARGS__)
#define dlg_err(fmt, ...) \
			printk(KERN_ERR fmt, ##__VA_ARGS__)
#define dlg_warn(fmt, ...) \
			printk(KERN_WARNING fmt, ##__VA_ARGS__)
#define dlg_info(fmt, ...) \
			printk(KERN_INFO fmt, ##__VA_ARGS__)
#else
#define dlg_crit(fmt, ...) 	do { } while(0);
#define dlg_err(fmt, ...)	do { } while(0);
#define dlg_warn(fmt, ...)	do { } while(0);
#define dlg_info(fmt, ...)	do { } while(0);
#endif

typedef u32 (*pmu_platform_callback)(int event, int param);

struct d2041_irq {
	irq_handler_t handler;
	void *data;
};

struct d2041_onkey {
	struct platform_device *pdev;
	struct input_dev *input;
};
struct d2041_power {
	struct platform_device *pdev;
	struct input_dev *input;
};
struct d2041_regl_map {
    u8 reg_id;
    u8 dsm_opmode;
    u8 default_pm_mode;
};

/**
 * Data to be supplied by the platform to initialise the D2041.
 *
 * @init: Function called during driver initialisation.  Should be
 *        used by the platform to configure GPIO functions and similar.
 * @irq_high: Set if D2041 IRQ is active high.
 * @irq_base: Base IRQ for genirq (not currently used).
 */
 struct batt_adc_tbl_t {
	u16 *bat_adc;
	u16 *bat_vol;
	u32 num_entries;
};
struct linear_power_pdata
{
	u8 usb_charging_cc;
	u8 wac_charging_cc;
	u8 eoc_current;

	int temp_adc_channel;
	u16 temp_low_limit;
	u16 temp_high_limit;

	struct batt_adc_tbl_t batt_adc_tbl_t;
	u8 batt_technology;
};
struct d2041_platform_data {
	struct i2c_slave_platform_data i2c_pdata;

	int 	(*init)(struct d2041 *d2041);
	int 	(*irq_init)(struct d2041 *d2041);
	int		irq_mode;	/* Clear interrupt by read/write(0/1) */
	int		irq_base;	/* IRQ base number of D2041 */
	struct	d2041_audio_platform_data *audio_pdata;
	struct  regulator_consumer_supply *regulator_data;
        // temporary code
	struct  linear_power_pdata *power;
	pmu_platform_callback pmu_event_cb;
	
	//unsigned char regl_mapping[D2041_IOCTL_REGL_MAPPING_NUM];	/* Regulator mapping for IOCTL */
	struct d2041_regl_map regl_map[D2041_NUMBER_OF_REGULATORS];
};

struct d2041 {
	struct device *dev;

	struct i2c_client *i2c_client;

	int (*read_dev)(struct d2041 * const d2041, char const reg, int const size, void * const dest);
	int (*write_dev)(struct d2041 * const d2041, char const reg, int const size, const u8 *src/*void * const src*/);
	u8 *reg_cache;

	/* Interrupt handling */
    	struct work_struct irq_work;
    	struct task_struct *irq_task;
	struct mutex irq_mutex; /* IRQ table mutex */
	struct d2041_irq irq[D2041_NUM_IRQ];
	int chip_irq;

	struct delayed_work monitor_work;
	struct task_struct  *monitor_task;

	struct d2041_pmic pmic;
	struct d2041_rtc rtc;
	struct d2041_onkey onkey;
	struct d2041_audio audio;
	struct d2041_power power;

    struct d2041_platform_data *pdata;
};

/*
 * d2041 Core device initialisation and exit.
 */
int 	d2041_device_init(struct d2041 *d2041, int irq, struct d2041_platform_data *pdata);
void 	d2041_device_exit(struct d2041 *d2041);

/*
 * d2041 device IO
 */
int 	d2041_clear_bits(struct d2041 * const d2041, u8 const reg, u8 const mask);
int 	d2041_set_bits(struct d2041* const d2041, u8 const reg, u8 const mask);
int     d2041_reg_read(struct d2041 * const d2041, u8 const reg, u8 *dest);
int 	d2041_reg_write(struct d2041 * const d2041, u8 const reg, u8 const val);
int 	d2041_block_read(struct d2041 * const d2041, u8 const start_reg, u8 const regs, u8 * const dest);
int 	d2041_block_write(struct d2041 * const d2041, u8 const start_reg, u8 const regs, u8 * const src);

/*
 * d2041 internal interrupts
 */
int 	d2041_register_irq(struct d2041 *d2041, int irq, irq_handler_t handler, 
				unsigned long flags, const char *name, void *data);
int 	d2041_free_irq(struct d2041 *d2041, int irq);
int 	d2041_mask_irq(struct d2041 *d2041, int irq);
int 	d2041_unmask_irq(struct d2041 *d2041, int irq);
int 	d2041_irq_init(struct d2041 *d2041, int irq, struct d2041_platform_data *pdata);
int 	d2041_irq_exit(struct d2041 *d2041);

#ifdef CONFIG_BOARD_CORI
/* DLG IOCTL interface */
extern int d2041_ioctl_regulator(struct d2041 *d2041, unsigned int cmd, unsigned long arg);
#endif /* CONFIG_BOARD_CORI */

/* DLG new prototype */
int d2041_shutdown(struct d2041 *d2041);
void d2041_system_poweroff(void);
extern int d2041_core_reg2volt(int reg);
extern void set_MCTL_enabled(void);

#endif
