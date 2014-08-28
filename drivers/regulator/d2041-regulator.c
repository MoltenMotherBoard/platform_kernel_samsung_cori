/*
* d2041-regulator.c: Regulator driver for Dialog D2041
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
//TODO MW: format file (change to tabs, fix indents, ect...)
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/d2041/core.h>
#include <linux/d2041/pmic.h>
#include <linux/d2041/d2041_reg.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#ifdef CONFIG_BOARD_CORI
#include <linux/broadcom/pmu_chip.h>
#include <asm/uaccess.h>
#endif /* CONFIG_BOARD_CORI */

/****************************************************************************
 * TODO MW: REMOVE this when Cougar chip available               REMOVE BEGIN
 ****************************************************************************
 */


#define PANTHER 0

#if PANTHER

#warning ===== Building PANTHER version of regulators for test purposes !!! =====
// Map logical registers of LDOs 11-20 and AUD to physical registers od LDOs 1-10 - just for tests purposes
#define D2041_SUPPLY_REG                        (70) // Panther SUPPLY_REG
/* BUCK_1 to LDO_10 are the same on both chips - remap just those: */
#define D2041_LDO11_REG                         D2041_LDO1_REG
#define D2041_LDO12_REG                         D2041_LDO2_REG
#define D2041_LDO13_REG                         D2041_LDO3_REG
#define D2041_LDO14_REG                         D2041_LDO4_REG
#define D2041_LDO15_REG                         D2041_LDO5_REG
#define D2041_LDO16_REG                         D2041_LDO6_REG
#define D2041_LDO17_REG                         D2041_LDO7_REG
#define D2041_LDO18_REG                         D2041_LDO8_REG
#define D2041_LDO19_REG                         D2041_LDO9_REG
#define D2041_LDO20_REG                         D2041_LDO10_REG
#define D2041_LDO_AUD_REG                       D2041_LDO1_REG

#define MCTL_ENABLED                (0) // TODO MW: change this to test both conditions
#define MCTL_PRECONFIGURED_FLAG     (0) // TODO MW: change this to test both conditions

#else // ! PANTHER

//unnessary comment at official version //James #warning ===== Building COUGAR version of regulators. =====

#endif // PANTHER

/****************************************************************************
 * TODO MW: REMOVE this when Cougar chip available                 REMOVE END
 ****************************************************************************
 */


struct regl_register_map {
    u8 pm_reg;
    u8 mctl_reg;
    u8 dsm_opmode;
};


#define DRIVER_NAME                 "d2041-regulator"

#define REGULATOR_REGISTER_ADJUST   (D2041_LDO13_REG - D2041_LDO12_REG - 1)             /* = 4 */
#define MCTL_REGISTER_ADJUST        (D2041_LDO1_MCTL_REG - D2041_BUCK4_MCTL_REG - 1)    /* = -25 */

#define mV_to_uV(v)                 ((v) * 1000)
#define uV_to_mV(v)                 ((v) / 1000)
#define MAX_MILLI_VOLT              (3300)
#define get_regulator_reg(n)        (regulator_register_map[n].pm_reg)
#define get_regulator_mctl_reg(n)   (regulator_register_map[n].mctl_reg)
#define get_regulator_dsm_opmode(n) (regulator_register_map[n].dsm_opmode)

#define D2041_DEFINE_REGL(_name, _pm_reg, _mctl_reg) \
    [D2041_##_name] = \
        { \
            .pm_reg = _pm_reg, \
            .mctl_reg = _mctl_reg, \
        }

extern struct d2041 *d2041_regl_info;

static struct regl_register_map regulator_register_map[] = {
    D2041_DEFINE_REGL(BUCK_1,  D2041_BUCK1_TURBO_REG,   D2041_BUCK1_MCTL_REG  ), /* 0 */
    D2041_DEFINE_REGL(BUCK_2,  D2041_BUCK2_REG,   D2041_BUCK2_MCTL_REG  ), /* 1 */
    D2041_DEFINE_REGL(BUCK_3,  D2041_BUCK3_REG,   D2041_BUCK3_MCTL_REG  ), /* 2 */
    D2041_DEFINE_REGL(BUCK_4,  D2041_BUCK4_REG,   D2041_BUCK4_MCTL_REG  ), /* 3 */
    D2041_DEFINE_REGL(LDO_1,   D2041_LDO1_REG,    D2041_LDO1_MCTL_REG   ), /* 4 */
    D2041_DEFINE_REGL(LDO_2,   D2041_LDO2_REG,    D2041_LDO2_MCTL_REG   ), /* 5 */
    D2041_DEFINE_REGL(LDO_3,   D2041_LDO3_REG,    D2041_LDO3_MCTL_REG   ), /* 6 */
    D2041_DEFINE_REGL(LDO_4,   D2041_LDO4_REG,    D2041_LDO4_MCTL_REG   ), /* 7 */
    D2041_DEFINE_REGL(LDO_5,   D2041_LDO5_REG,    D2041_LDO5_MCTL_REG   ), /* 8 */
    D2041_DEFINE_REGL(LDO_6,   D2041_LDO6_REG,    D2041_LDO6_MCTL_REG   ), /* 9 */
    D2041_DEFINE_REGL(LDO_7,   D2041_LDO7_REG,    D2041_LDO7_MCTL_REG   ), /* 10 */
    D2041_DEFINE_REGL(LDO_8,   D2041_LDO8_REG,    D2041_LDO8_MCTL_REG   ), /* 11 */
    D2041_DEFINE_REGL(LDO_9,   D2041_LDO9_REG,    D2041_LDO9_MCTL_REG   ), /* 12 */
    D2041_DEFINE_REGL(LDO_10,  D2041_LDO10_REG,   D2041_LDO10_MCTL_REG  ), /* 13 */
    D2041_DEFINE_REGL(LDO_11,  D2041_LDO11_REG,   D2041_LDO11_MCTL_REG  ), /* 14 */
    D2041_DEFINE_REGL(LDO_12,  D2041_LDO12_REG,   D2041_LDO12_MCTL_REG  ), /* 15 */
    D2041_DEFINE_REGL(LDO_13,  D2041_LDO13_REG,   D2041_LDO13_MCTL_REG  ), /* 16 */
    D2041_DEFINE_REGL(LDO_14,  D2041_LDO14_REG,   D2041_LDO14_MCTL_REG  ), /* 17 */
    D2041_DEFINE_REGL(LDO_15,  D2041_LDO15_REG,   D2041_LDO15_MCTL_REG  ), /* 18 */
    D2041_DEFINE_REGL(LDO_16,  D2041_LDO16_REG,   D2041_LDO16_MCTL_REG  ), /* 19 */
    D2041_DEFINE_REGL(LDO_17,  D2041_LDO17_REG,   D2041_LDO17_MCTL_REG  ), /* 20 */
    D2041_DEFINE_REGL(LDO_18,  D2041_LDO18_REG,   D2041_LDO18_MCTL_REG  ), /* 21 */
    D2041_DEFINE_REGL(LDO_19,  D2041_LDO19_REG,   D2041_LDO19_MCTL_REG  ), /* 22 */
    D2041_DEFINE_REGL(LDO_20,  D2041_LDO20_REG,   D2041_LDO20_MCTL_REG  ), /* 23 */
    D2041_DEFINE_REGL(LDO_AUD, D2041_LDO_AUD_REG, D2041_LDO_AUD_MCTL_REG)  /* 24 */
};

#define MICRO_VOLT(x)	((x)*1000L*1000L)
#define _DEFINE_REGL_VOLT(vol, regVal)	{ MICRO_VOLT(vol), regVal }

static const int core_dvs_vol[][2] = {
	_DEFINE_REGL_VOLT(0.500, 0),
	_DEFINE_REGL_VOLT(0.525, 1),
	_DEFINE_REGL_VOLT(0.550, 2),
	_DEFINE_REGL_VOLT(0.575, 3),
	_DEFINE_REGL_VOLT(0.600, 4),
	_DEFINE_REGL_VOLT(0.625, 5),
	_DEFINE_REGL_VOLT(0.650, 6),
	_DEFINE_REGL_VOLT(0.675, 7),
	_DEFINE_REGL_VOLT(0.700, 8),
	_DEFINE_REGL_VOLT(0.725, 9),
	_DEFINE_REGL_VOLT(0.750, 0xA),
	_DEFINE_REGL_VOLT(0.775, 0xB),
	_DEFINE_REGL_VOLT(0.800, 0xC),
	_DEFINE_REGL_VOLT(0.825, 0xD),
	_DEFINE_REGL_VOLT(0.850, 0xE),
	_DEFINE_REGL_VOLT(0.875, 0xF),
	_DEFINE_REGL_VOLT(0.900, 0x10),
	_DEFINE_REGL_VOLT(0.925, 0x11),
	_DEFINE_REGL_VOLT(0.950, 0x12),
	_DEFINE_REGL_VOLT(0.975, 0x13),
	_DEFINE_REGL_VOLT(1.000, 0x14),
	_DEFINE_REGL_VOLT(1.025, 0x15),
	_DEFINE_REGL_VOLT(1.050, 0x16),
	_DEFINE_REGL_VOLT(1.075, 0x17),
	_DEFINE_REGL_VOLT(1.100, 0x18),
	_DEFINE_REGL_VOLT(1.125, 0x19),
	_DEFINE_REGL_VOLT(1.150, 0x1A),
	_DEFINE_REGL_VOLT(1.175, 0x1B),
	_DEFINE_REGL_VOLT(1.200, 0x1C),
	_DEFINE_REGL_VOLT(1.225, 0x1D),
	_DEFINE_REGL_VOLT(1.250, 0x1E),
	_DEFINE_REGL_VOLT(1.275, 0x1F),
	_DEFINE_REGL_VOLT(1.300, 0x20),
	_DEFINE_REGL_VOLT(1.325, 0x21),
	_DEFINE_REGL_VOLT(1.350, 0x22),
	_DEFINE_REGL_VOLT(1.375, 0x23),
	_DEFINE_REGL_VOLT(1.400, 0x24),
	_DEFINE_REGL_VOLT(1.425, 0x25),
	_DEFINE_REGL_VOLT(1.450, 0x26),
	_DEFINE_REGL_VOLT(1.475, 0x27),
	_DEFINE_REGL_VOLT(1.500, 0x28),
	_DEFINE_REGL_VOLT(1.525, 0x29),
	_DEFINE_REGL_VOLT(1.550, 0x2A),
	_DEFINE_REGL_VOLT(1.575, 0x2B),
	_DEFINE_REGL_VOLT(1.600, 0x2C),
	_DEFINE_REGL_VOLT(1.625, 0x2D),
	_DEFINE_REGL_VOLT(1.650, 0x2E),
	_DEFINE_REGL_VOLT(1.675, 0x2F),
	_DEFINE_REGL_VOLT(1.700, 0x30),
	_DEFINE_REGL_VOLT(1.725, 0x31),
	_DEFINE_REGL_VOLT(1.750, 0x32),
	_DEFINE_REGL_VOLT(1.775, 0x33),
	_DEFINE_REGL_VOLT(1.800, 0x34),
	_DEFINE_REGL_VOLT(1.825, 0x35),
	_DEFINE_REGL_VOLT(1.850, 0x36),
	_DEFINE_REGL_VOLT(1.875, 0x37),
	_DEFINE_REGL_VOLT(1.900, 0x38),
	_DEFINE_REGL_VOLT(1.925, 0x39),
	_DEFINE_REGL_VOLT(1.950, 0x3A),
	_DEFINE_REGL_VOLT(1.975, 0x3B),
	_DEFINE_REGL_VOLT(2.000, 0x3C),
	_DEFINE_REGL_VOLT(2.025, 0x3D),
	_DEFINE_REGL_VOLT(2.050, 0x3E),
	_DEFINE_REGL_VOLT(2.075, 0x3F),
};

#if 1   // regulator function speed-up
static int mctl_status=0;   // default is disable

static int is_mode_control_enabled(void)
{
    // 0 : mctl disable, 1 : mctl enable
    return mctl_status;
}
#else
static inline bool is_mode_control_enabled(struct d2041 *d2041)
{
    u8 reg_val;
	bool ret;

    d2041_reg_read(d2041, D2041_POWERCONT_REG, &reg_val);
	ret = (reg_val & D2041_POWERCONT_MCTRLEN);
    
    return ret;
}
#endif

//Needed for sysparms
static struct regulator_dev *d2041_rdev[D2041_NUMBER_OF_REGULATORS];

static inline int get_global_mctl_mode(struct d2041 *d2041)
{
    u8 reg_val;

    d2041_reg_read(d2041, D2041_STATUSA_REG, &reg_val);

    // Remove "NOT" operation
    return ((reg_val & D2041_STATUSA_MCTL) >> D2041_STATUSA_MCTL_SHIFT);
}


static unsigned int get_regulator_mctl_mode(struct d2041 *d2041, int regulator_id)
{
    //TODO: Make sure we can assume we have always current value in the buffer and do this:
    u8 reg_val, regulator_mctl_reg = get_regulator_mctl_reg(regulator_id);
    int ret = 0;

    ret = d2041_reg_read(d2041, regulator_mctl_reg, &reg_val);
    reg_val &= D2041_REGULATOR_MCTL0;
    reg_val >>= MCTL0_SHIFT;
    
    return reg_val;
 }


static int set_regulator_mctl_mode(struct d2041 *d2041, int regulator_id, u8 mode)
{
    u8 reg_val, mctl_reg;
    int ret = 0;

    if(regulator_id < 0 || regulator_id >= D2041_NUMBER_OF_REGULATORS)
        return -EINVAL;
    if(mode > REGULATOR_MCTL_TURBO)
        return -EINVAL;

    mctl_reg = get_regulator_mctl_reg(regulator_id);
    ret = d2041_reg_read(d2041, mctl_reg, &reg_val);
    if(ret < 0)
        return ret;

    reg_val &= ~(D2041_REGULATOR_MCTL0 | D2041_REGULATOR_MCTL2);
    reg_val |= ((mode << MCTL0_SHIFT) | ( mode << MCTL2_SHIFT));
    ret = d2041_reg_write(d2041, mctl_reg, reg_val);
    dlg_info("[REGULATOR] %s. reg_val = 0x%X\n", __func__, reg_val);

    return ret;
}


static int d2041_register_regulator(struct d2041 *d2041, int reg,
                                    struct regulator_init_data *initdata);


static struct regulator_init_data d2041_regulators_init_data[D2041_NUMBER_OF_REGULATORS] = {

    [D2041_BUCK_1] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_BUCK12_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_BUCK12_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .always_on = 1,
        },
    },


    [D2041_BUCK_2] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_BUCK12_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_BUCK12_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .always_on = 1,
        },
    },


    [D2041_BUCK_3] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_BUCK3_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_BUCK3_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },

    [D2041_BUCK_4] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_BUCK4_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_BUCK4_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_MODE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
        },
    },

    [D2041_LDO_1] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },

    [D2041_LDO_2] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },

    [D2041_LDO_3] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },

    [D2041_LDO_4] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },
    [D2041_LDO_5] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },

    [D2041_LDO_6] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },

    [D2041_LDO_7] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },

    [D2041_LDO_8] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },
    [D2041_LDO_9] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
        },
    },
    [D2041_LDO_10] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
         },
    },
    [D2041_LDO_11] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_12] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
            .boot_on = 1,
         },
    },
    [D2041_LDO_13] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_14] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_15] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_16] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_17] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_18] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_19] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_20] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    },
    [D2041_LDO_AUD] =  {
        .constraints = {
            .min_uV = mV_to_uV(D2041_LDO_VOLT_LOWER),
            .max_uV = mV_to_uV(D2041_LDO_VOLT_UPPER),
            .valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_STATUS,
            .valid_modes_mask = REGULATOR_MODE_NORMAL,
         },
    }
};


/*
void dump_MCTL_REG(void)
{
	u8 reg_val;
	int i;

	dlg_info("BUCK / LDO reg\n");
	for(i=D2041_BUCK1_REG; i<=D2041_LDO_AUD_REG; i++){
		d2041_reg_read(d2041_regl_info,i,&reg_val);
		dlg_info("addr[0x%x], val[0x%x]\n", i, reg_val);
	}

	dlg_info("MCTL reg\n");
	for(i=D2041_LDO1_MCTL_REG; i<=D2041_BUCK1_TURBO_REG; i++){
		d2041_reg_read(d2041_regl_info,i,&reg_val);
		dlg_info("addr[0x%x], val[0x%x]\n", i, reg_val);
	}
}
EXPORT_SYMBOL(dump_MCTL_REG);
*/

void set_MCTL_enabled(void)
{
    u8 reg_val;

	if(d2041_regl_info==NULL){
		dlg_err("MCTRL_EN bit is not set\n");
		return;
	}
		
    d2041_reg_read(d2041_regl_info, D2041_POWERCONT_REG, &reg_val);
    reg_val |= D2041_POWERCONT_MCTRLEN;
    d2041_reg_write(d2041_regl_info, D2041_POWERCONT_REG, reg_val);

    mctl_status=1;
}
EXPORT_SYMBOL(set_MCTL_enabled);


int __init d2041_platform_regulator_init(struct d2041 *d2041)
{
    int i;
    u8 reg_val=0;

    for(i = D2041_BUCK_1; i < D2041_NUMBER_OF_REGULATORS; i++) {
        if (d2041->pdata != NULL &&
	    d2041->pdata->regulator_data != NULL &&
  	    d2041->pdata->regulator_data[i].supply != NULL) {
  	    
		d2041_regulators_init_data[i].consumer_supplies = &d2041->pdata->regulator_data[i];
		d2041_regulators_init_data[i].num_consumer_supplies = 1;

		dev_info(d2041->dev, "No consumers for regulator %s\n",
			d2041_regulators_init_data[i].constraints.name);
		}
		
        d2041_register_regulator(d2041, i, &(d2041_regulators_init_data[i]));
        regulator_register_map[i].dsm_opmode = d2041->pdata->regl_map[i].default_pm_mode;
		
        if(i >= D2041_BUCK_1) {
            d2041_reg_write(d2041, get_regulator_mctl_reg(i),
							d2041->pdata->regl_map[i].dsm_opmode);
        }
    }

	// set MISC_MCTL
	/*
	reg_val = (D2041_MISC_MCTL3_DIGICLK | D2041_MISC_MCTL2_DIGICLK |
			   D2041_MISC_MCTL1_DIGICLK | D2041_MISC_MCTL0_DIGICLK |
		       D2041_MISC_MCTL3_BBAT | D2041_MISC_MCTL2_BBAT |
			   D2041_MISC_MCTL1_BBAT | D2041_MISC_MCTL0_BBAT);
		*/
	reg_val = 0x0F;
	d2041_reg_write(d2041_regl_info, D2041_MISC_MCTL_REG, reg_val);

	d2041_reg_write(d2041_regl_info, D2041_BUCK1_TURBO_REG, D2041_BUCK1_NM_VOLT);
	d2041_reg_write(d2041_regl_info, D2041_BUCK1_RETENTION_REG, D2041_BUCK1_RETENTION_VOLT);

	// set AUD_LDO to 2.5v
	d2041_reg_write(d2041_regl_info, D2041_LDO_AUD_REG, 0x5A);

	// sim_vcc(LDO11) -> default 1.8v
	d2041_reg_write(d2041_regl_info, D2041_LDO11_REG, 0xC);

	// LDO_AUD pull-down disable
	d2041_reg_write(d2041_regl_info, D2041_PULLDOWN_REG_D, (1<<0));

    return 0;
}



static int d2041_regulator_val_to_mvolts(unsigned int val, int regulator_id, struct regulator_dev *rdev)
{
    struct regulation_constraints *constraints;
    int min_mV, mvolts;

    constraints =  rdev->constraints;
    min_mV = uV_to_mV(constraints->min_uV);

    if ( (regulator_id >= D2041_BUCK_1) && (regulator_id <= D2041_BUCK_4) )
        return ((val * D2041_BUCK_VOLT_STEP) + min_mV );

    mvolts = (val * D2041_LDO_VOLT_STEP) + min_mV;
    if(mvolts > MAX_MILLI_VOLT)
        mvolts = MAX_MILLI_VOLT;

    return mvolts;
}


static unsigned int d2041_regulator_mvolts_to_val(int mV, int regulator_id, struct regulator_dev *rdev)
{
    struct regulation_constraints *constraints =  rdev->constraints;
    int min_mV = uV_to_mV(constraints->min_uV);

    if ( (regulator_id >= D2041_BUCK_1) && (regulator_id <= D2041_BUCK_4) )
        return ((mV - min_mV) / D2041_BUCK_VOLT_STEP);

    return ((mV - min_mV) / D2041_LDO_VOLT_STEP);
}

static int d2041_regulator_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV)
{
    struct d2041 *d2041 = rdev_get_drvdata(rdev);
    int mV_val;
    int min_mV = uV_to_mV(min_uV);
    int max_mV = uV_to_mV(max_uV);
    unsigned int reg_num, regulator_id = rdev_get_id(rdev);
    u8 val;
    int ret = 0;

    /* before we do anything check the lock bit */
    ret = d2041_reg_read(d2041, D2041_SUPPLY_REG, &val);
    if(val & D2041_SUPPLY_VLOCK)
        d2041_clear_bits(d2041, D2041_SUPPLY_REG, D2041_SUPPLY_VLOCK);

    mV_val =  d2041_regulator_mvolts_to_val(min_mV, regulator_id, rdev);

    /* Sanity check for maximum value */
    if (d2041_regulator_val_to_mvolts(mV_val, regulator_id, rdev) > max_mV)
        return -EINVAL;

    reg_num = get_regulator_reg(regulator_id);

    ret = d2041_reg_read(d2041, reg_num, &val);
    val &= ~D2041_MAX_VSEL;
    d2041_reg_write(d2041, reg_num, (val | mV_val));

    /* For BUCKs enable the ramp */
    if (regulator_id <= D2041_BUCK_4)
        d2041_set_bits(d2041, D2041_SUPPLY_REG, (D2041_SUPPLY_VBUCK1GO << regulator_id));

    return ret;
}
//EXPORT_SYMBOL_GPL(d2041_regulator_set_voltage);


static int d2041_regulator_get_voltage(struct regulator_dev *rdev)
{
    struct d2041 *d2041 = rdev_get_drvdata(rdev);
    unsigned int reg_num, regulator_id = rdev_get_id(rdev);
    int ret;
    u8 val;

	dlg_info("[[%s]], regulator_id[%d]\n", __func__, regulator_id);
	

    reg_num = get_regulator_reg(regulator_id);
    ret = d2041_reg_read(d2041, reg_num, &val);
    val &= D2041_MAX_VSEL;
    ret = mV_to_uV(d2041_regulator_val_to_mvolts(val, regulator_id, rdev));

    return ret;
}
//EXPORT_SYMBOL_GPL(d2041_regulator_get_voltage);


static int d2041_regulator_enable(struct regulator_dev *rdev)
{
    struct d2041 *d2041 = rdev_get_drvdata(rdev);
    u8 reg_val;
    int ret = 0;
    unsigned int regulator_id = rdev_get_id(rdev);
    unsigned int reg_num;

    if (regulator_id >= D2041_NUMBER_OF_REGULATORS)
        return -EINVAL;

	if(!is_mode_control_enabled()){
		reg_num = get_regulator_reg(regulator_id);
		
		d2041_reg_read(d2041, reg_num, &reg_val);
		reg_val |= (1<<6);
		ret = d2041_reg_write(d2041, reg_num,reg_val);

	}else {
    reg_num = get_regulator_mctl_reg(regulator_id);
	    
    ret = d2041_reg_read(d2041, reg_num, &reg_val);
    if(ret < 0) {
        dlg_err("I2C read error\n");
        return ret;
    }

    reg_val &= ~(D2041_REGULATOR_MCTL1 | D2041_REGULATOR_MCTL3);   // Clear MCTL11 and MCTL01
    reg_val |= (D2041_REGULATOR_MCTL1_ON | D2041_REGULATOR_MCTL3_ON);

    switch(get_regulator_dsm_opmode(regulator_id)) {
        case D2041_REGULATOR_LPM_IN_DSM :
            reg_val &= ~(D2041_REGULATOR_MCTL0 | D2041_REGULATOR_MCTL2);
            reg_val |= (D2041_REGULATOR_MCTL0_SLEEP | D2041_REGULATOR_MCTL2_SLEEP);
            break;
        case D2041_REGULATOR_OFF_IN_DSM :
            reg_val &= ~(D2041_REGULATOR_MCTL0 | D2041_REGULATOR_MCTL2);
            break;
        case D2041_REGULATOR_ON_IN_DSM :
            reg_val &= ~(D2041_REGULATOR_MCTL0 | D2041_REGULATOR_MCTL2);
            reg_val |= (D2041_REGULATOR_MCTL0_ON | D2041_REGULATOR_MCTL2_ON);
            break;
    }

    ret |= d2041_reg_write(d2041, reg_num, reg_val);
	}

    return ret;
}
//EXPORT_SYMBOL_GPL(d2041_regulator_enable);


static int d2041_regulator_disable(struct regulator_dev *rdev)
{
    struct d2041 *d2041 = rdev_get_drvdata(rdev);
    unsigned int regulator_id = rdev_get_id(rdev);
    unsigned int reg_num = 0;
    int ret = 0;
    u8 reg_val;

    if (regulator_id >= D2041_NUMBER_OF_REGULATORS)
        return -EINVAL;

	if(!is_mode_control_enabled()){
	
		reg_num = get_regulator_reg(regulator_id);
		
		d2041_reg_read(d2041, reg_num, &reg_val);
		reg_val &= ~(1<<6);
		d2041_reg_write(d2041, reg_num, reg_val);
	}else{

    reg_num = get_regulator_mctl_reg(regulator_id);
    /* 0x00 ==  D2041_REGULATOR_MCTL0_OFF | D2041_REGULATOR_MCTL1_OFF 
     *        | D2041_REGULATOR_MCTL2_OFF | D2041_REGULATOR_MCTL3_OFF 
     */

    ret = d2041_reg_write(d2041, reg_num, 0x00);

	}

    return ret;
}
//EXPORT_SYMBOL_GPL(d2041_regulator_disable);


static unsigned int d2041_regulator_get_mode(struct regulator_dev *rdev)
{
    struct d2041 *d2041 = rdev_get_drvdata(rdev);
    unsigned int regulator_id = rdev_get_id(rdev);
    unsigned int mode = 0;

    if (regulator_id >= D2041_NUMBER_OF_REGULATORS)
            return -EINVAL;

    mode = get_regulator_mctl_mode(d2041, regulator_id);

    /* Map d2041 regulator mode to Linux framework mode */
    switch(mode) {
        case REGULATOR_MCTL_TURBO:
            mode = REGULATOR_MODE_FAST;
            break;
        case REGULATOR_MCTL_ON:
            mode = REGULATOR_MODE_NORMAL;
            break;
        case REGULATOR_MCTL_SLEEP:
            mode = REGULATOR_MODE_IDLE;
            break;
        case REGULATOR_MCTL_OFF:
            mode = REGULATOR_MODE_STANDBY;
            break;
        default:
            /* unsupported or unknown mode */
            break;
    }

    dlg_info("[REGULATOR] : [%s] >> MODE(%d)\n", __func__, mode);

    return mode;
}
//EXPORT_SYMBOL_GPL(d2041_regulator_get_mode);



/* TODO MW: remove set function ? - mode is HW controled in mctl, we cannot set, what about normal mode ?  */
/*
TODO
Set mode is it software controlled?
if yes then following implementation is useful ??
if no then we dont need following implementation ??
*/
static int d2041_regulator_set_mode(struct regulator_dev *rdev, unsigned int mode)
{
    struct d2041 *d2041 = rdev_get_drvdata(rdev);
    unsigned int regulator_id = rdev_get_id(rdev);
    int ret;
    u8 mctl_mode;

    dlg_info("[REGULATOR] : regulator_set_mode. mode is %d\n", mode);

    switch(mode) {
        case REGULATOR_MODE_FAST :
            mctl_mode = REGULATOR_MCTL_TURBO;
            break;
        case REGULATOR_MODE_NORMAL :
            mctl_mode = REGULATOR_MCTL_ON;
            break;
        case REGULATOR_MODE_IDLE :
            mctl_mode = REGULATOR_MCTL_SLEEP;
            break;
        case REGULATOR_MODE_STANDBY:
            mctl_mode = REGULATOR_MCTL_OFF;
            break;
        default:
            return -EINVAL;
    }
    
    ret = set_regulator_mctl_mode(d2041, regulator_id, mode);

    return ret;
}
//EXPORT_SYMBOL_GPL(d2041_regulator_set_mode);


static int d2041_regulator_is_enabled(struct regulator_dev *rdev)
{
    struct d2041 *d2041 = rdev_get_drvdata(rdev);
    unsigned int reg_num, regulator_id = rdev_get_id(rdev);
    int ret = -EINVAL;
    u8 reg_val = 0;

	if(!is_mode_control_enabled()){

		reg_num = get_regulator_reg(regulator_id);
		
	    ret = d2041_reg_read(d2041, reg_num, &reg_val);
	    if(ret < 0) {
	        dlg_err("I2C read error. \n");
	        return ret;
	    }

		/* 0x0 : off, 0x1 : on */
		ret = reg_val & (1<<6);
	}else{
    if (regulator_id >= D2041_NUMBER_OF_REGULATORS)
        return -EINVAL;

    reg_num = get_regulator_mctl_reg(regulator_id);
    ret = d2041_reg_read(d2041, reg_num, &reg_val);
    if(ret < 0) {
        dlg_err("I2C read error. \n");
        return ret;
    }

    /* 0x0 : Off    * 0x1 : On    * 0x2 : Sleep    * 0x3 : n/a */
		ret = ((reg_val & (D2041_REGULATOR_MCTL1|D2041_REGULATOR_MCTL3)) >= 1) ? 1 : 0;
	}

    return ret;
}
//EXPORT_SYMBOL_GPL(d2041_regulator_is_enabled);


static struct regulator_ops d2041_ldo_ops = {
    .set_voltage = d2041_regulator_set_voltage,
    .get_voltage = d2041_regulator_get_voltage,
    .enable = d2041_regulator_enable,
    .disable = d2041_regulator_disable,
    .is_enabled = d2041_regulator_is_enabled,
    .get_mode = d2041_regulator_get_mode,
    .set_mode = d2041_regulator_set_mode,
};


static struct regulator_desc d2041_reg[D2041_NUMBER_OF_REGULATORS] = {
    {
        .name = "D2041_BUCK_1",
        .id = D2041_BUCK_1,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_BUCK_2",
        .id = D2041_BUCK_2,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_BUCK_3",
        .id = D2041_BUCK_3,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_BUCK_4",
        .id = D2041_BUCK_4,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_1",
        .id = D2041_LDO_1,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_2",
        .id = D2041_LDO_2,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_3",
        .id = D2041_LDO_3,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_4",
        .id = D2041_LDO_4,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_5",
        .id = D2041_LDO_5,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_6",
        .id = D2041_LDO_6,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_7",
        .id = D2041_LDO_7,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_8",
        .id = D2041_LDO_8,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_9",
        .id = D2041_LDO_9,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_10",
        .id = D2041_LDO_10,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_11",
        .id = D2041_LDO_11,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_12",
        .id = D2041_LDO_12,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_13",
        .id = D2041_LDO_13,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_14",
        .id = D2041_LDO_14,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_15",
        .id = D2041_LDO_15,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_16",
        .id = D2041_LDO_16,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_17",
        .id = D2041_LDO_17,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_18",
        .id = D2041_LDO_18,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_19",
        .id = D2041_LDO_19,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_20",
        .id = D2041_LDO_20,
        .n_voltages = D2041_MAX_VSEL + 1,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    },
    {
        .name = "D2041_LDO_AUD",
        .id = D2041_LDO_AUD,
        .ops = &d2041_ldo_ops,
        .type = REGULATOR_VOLTAGE,
        .owner = THIS_MODULE,
    }
};

#ifdef CONFIG_BOARD_CORI 
/****************************************************************/
/*		Regulators IOCTL debugging			*/
/****************************************************************/


int d2041_ioctl_regulator(struct d2041 *d2041, unsigned int cmd, unsigned long arg)
{
	//struct max8986_regl_priv *regl_priv = (struct max8986_regl_priv *) pri_data;
	int ret = -EINVAL;
	int regid;

	dlg_info("Inside %s, IOCTL command is %d\n", __func__, cmd);
	switch (cmd) {
    	case BCM_PMU_IOCTL_SET_VOLTAGE:
    	{
    		pmu_regl_volt regulator;
    		if (copy_from_user(&regulator, (pmu_regl_volt *)arg,
    				   sizeof(pmu_regl_volt)) != 0) 
    		{
    			return -EFAULT;
    		}

    		if (regulator.regl_id < 0 
    		    || regulator.regl_id >= ARRAY_SIZE(d2041->pdata->regl_map))
    			return -EINVAL;
    		regid = d2041->pdata->regl_map[regulator.regl_id].reg_id;
    		if (regid < 0) 
    			return -EINVAL;

    		ret = d2041_regulator_set_voltage(d2041_rdev[regid],
    			regulator.voltage, regulator.voltage);
    		break;
    	}
    	case BCM_PMU_IOCTL_GET_VOLTAGE:
    	{
    		pmu_regl_volt regulator;
    		int volt;
    		if (copy_from_user(&regulator, (pmu_regl_volt *)arg,
    				   sizeof(pmu_regl_volt)) != 0) 
    		{
    			return -EFAULT;
    		}
    		if (regulator.regl_id < 0 
    		    || regulator.regl_id >= ARRAY_SIZE(d2041->pdata->regl_map))
    			return -EINVAL;
    		regid = d2041->pdata->regl_map[regulator.regl_id].reg_id;
    		if (regid < 0) 
    			return -EINVAL;

    		volt = d2041_regulator_get_voltage(d2041_rdev[regid]);
    		if (volt > 0) {
    			regulator.voltage = volt;
    			ret = copy_to_user((pmu_regl_volt *)arg, &regulator,
    					   sizeof(regulator));
    		} else {
    			ret = volt;
    		}
    		break;
    	}
    	case BCM_PMU_IOCTL_GET_REGULATOR_STATE:
    	{
    		pmu_regl rmode;
    		unsigned int mode;
    		if (copy_from_user(&rmode, (pmu_regl *)arg, 
    				   sizeof(pmu_regl)) != 0)
    			return -EFAULT;
    		
    		if (rmode.regl_id < 0 
    		    || rmode.regl_id >= ARRAY_SIZE(d2041->pdata->regl_map))
    			return -EINVAL;
    		regid = d2041->pdata->regl_map[rmode.regl_id].reg_id;
    		if (regid < 0) 
    			return -EINVAL;

    		mode = d2041_regulator_get_mode(d2041_rdev[regid]);
    		switch (mode) {
    		    case REGULATOR_MODE_FAST:
    			    rmode.state = PMU_REGL_TURBO;
    			    break;
    		    case REGULATOR_MODE_NORMAL:
    			    rmode.state = PMU_REGL_ON;
    			    break;
    		    case REGULATOR_MODE_STANDBY:
    			    rmode.state = PMU_REGL_ECO;
    			    break;
    		    default:
    			    rmode.state = PMU_REGL_OFF;
    			    break;
    		};
    		ret = copy_to_user((pmu_regl *)arg, &rmode, sizeof(rmode));
    		break;
    	}
    	case BCM_PMU_IOCTL_SET_REGULATOR_STATE:
    	{
    		pmu_regl rmode;
    		unsigned int mode;
    		if (copy_from_user(&rmode, (pmu_regl *)arg,
    					sizeof(pmu_regl)) != 0)
    			return -EFAULT;
    		if (rmode.regl_id < 0 
    		    || rmode.regl_id >= ARRAY_SIZE(d2041->pdata->regl_map))
    			return -EINVAL;
    		regid = d2041->pdata->regl_map[rmode.regl_id].reg_id;
    		if (regid < 0) 
    			return -EINVAL;
    		
    		switch (rmode.state) {
    		    case PMU_REGL_TURBO:
    			    mode = REGULATOR_MODE_FAST;
    			    break;
    		    case PMU_REGL_ON:
    			    mode = REGULATOR_MODE_NORMAL;
    			    break;
    		    case PMU_REGL_ECO:
    			    mode = REGULATOR_MODE_STANDBY;
    			    break;
    		    default:
    			    mode = REGULATOR_MCTL_OFF;
    			    break;
    		};
    		ret = d2041_regulator_set_mode(d2041_rdev[regid], mode);
    		break;
    	}
    	case BCM_PMU_IOCTL_ACTIVATESIM:
    	{
    		int id = 16;	/*check the status of SIMLDO*/
    		pmu_sim_volt sim_volt;
    		int value;
    		if (copy_from_user(&sim_volt, (int *)arg, sizeof(int)) != 0)
    			return -EFAULT;
    		regid = d2041->pdata->regl_map[id].reg_id;
    		if (regid < 0) 
    			return -EINVAL;
    		
    		/*check the status of SIMLDO*/
    		ret = d2041_regulator_is_enabled(d2041_rdev[regid]);
    		if (ret) {
    			dlg_info("SIMLDO is activated already\n");
    			return -EPERM;
    		}
    		/* Put SIMLDO in ON State */
    		ret = d2041_regulator_enable(d2041_rdev[regid]);
    		if (ret)
    			return ret;
    		/* Set SIMLDO voltage */
    		value = 2500000;			// 2.5V
    		ret = d2041_regulator_set_voltage(d2041_rdev[regid], value, value);
    		break;
    	}
    	case BCM_PMU_IOCTL_DEACTIVATESIM:
    	{
    		int id = 16;	/*check the status of SIMLDO*/
    		ret = d2041_regulator_is_enabled(d2041_rdev[id]);
    		if (!ret) {
    			dlg_info("SIMLDFO is already disabled\n");
    			return -EPERM;
    		}
    		regid = d2041->pdata->regl_map[id].reg_id;
    		if (regid < 0) 
    			return -EINVAL;
    		
    		ret = d2041_regulator_disable(d2041_rdev[regid]);
    		if (ret)
    			return ret;
    		break;
    	}
	}	/*end of switch*/
	return ret;
}
#endif /* CONFIG_BOARD_CORI */
/* IOCTL end */

#if 0   // Need to update.
// TODO MW: consider use of kernel parameters to override defaults/boot loader settings
//TBD
static void d2041_configure_mctl(struct d2041 *d2041, struct regulator_dev *rdev)
{
//TODO MW check and test this code ! Find out this is the bahaviour we want.
    int mV_val = 0;
    u8 misc_mctlv = 0;
    int i;

    /* WARNING: we assume here that once configured, MCTL registers content doesn't change !
     * If there is any change to MCTL registers during kernel runtime,
     * the regulator_mctl[] table MUST also be updated !
     */

    if (is_mctl_preconfigured() == true)  { /* MCTL configured before (preasumably by BL code) */
        for (i = D2041_BUCK_1; i < D2041_NUMBER_OF_REGULATORS; i++) {
            /* Just copy all mctl registers to the buffer */
            d2041_reg_read((d2041, get_regulator_mctl_reg(i), &regulator_mctl[i]);
         }
    } else { /* MCTL NOT preconfigured */
        /* Use predefined defaults
         * TODO MW: Those settings should  be customized for specific platform
         */
        for (i = D2041_BUCK_1; i < D2041_NUMBER_OF_REGULATORS; i++) {
            d2041_reg_write(d2041, get_regulator_mctl_reg(i), regulator_mctl[i]);
         }

        mV_val =  d2041_regulator_mvolts_to_val(D2041_BUCK1_RETENTION_VOLT, D2041_BUCK_1, rdev);
        d2041_reg_write(d2041, D2041_BUCK1_RETENTION_REG, mV_val);
        mV_val =  d2041_regulator_mvolts_to_val(D2041_BUCK1_TURBO_VOLT, D2041_BUCK_1, rdev);
        d2041_reg_write(d2041, D2041_BUCK1_TURBO_REG, mV_val);

        misc_mctlv = D2041_MISC_MCTL3_DIGICLK | D2041_MISC_MCTL2_DIGICLK | D2041_MISC_MCTL1_DIGICLK;
        d2041_reg_write(d2041, D2041_MISC_MCTL_REG, misc_mctlv);
    }

}
#endif


int d2041_core_reg2volt(int reg)
{
	int volt = -EINVAL;
	int i;

	#if 1	// will be removed
	switch(reg){
		case 0xe:
			reg = 0x1D;	//1.22
			break;
		case 0xa:
			reg = 0x20;	//1.30
			break;
		case 0x10:
			reg = 0x1C;	//1.18
			break;
		case 0xd:
			reg = 0x1E;	//1.24
			break;
		case 0x7:
			reg = 0x23;	//1.36
			break;
			
		default:
			dlg_info("[DLG] invalid reg value\n");
			break;
	}
	#endif

	for (i = 0; i < ARRAY_SIZE(core_dvs_vol); i++) {
		if (core_dvs_vol[i][1] == reg) {
			volt = core_dvs_vol[i][0];
			break;
		}
	}

	dlg_info("[DLG] [%s]-volt[%d]\n", __func__, volt);

	return volt;
}
EXPORT_SYMBOL(d2041_core_reg2volt);


static int d2041_regulator_probe(struct platform_device *pdev)
{
    struct regulator_dev *rdev;
    //struct d2041 *d2041 = platform_get_drvdata(pdev);

    if (pdev->id < D2041_BUCK_1 || pdev->id >= D2041_NUMBER_OF_REGULATORS)
        return -ENODEV;

    /* register regulator */
    rdev = regulator_register(&d2041_reg[pdev->id], &pdev->dev,
                  pdev->dev.platform_data,
                  dev_get_drvdata(&pdev->dev));
    if (IS_ERR(rdev)) {
        dev_err(&pdev->dev, "failed to register %s\n",
            d2041_reg[pdev->id].name);
        return PTR_ERR(rdev);
    }

    d2041_rdev[pdev->id] = rdev;		/* rdev required for IOCTL support */


    /* DLG TODO configure of MCTL_1 and MCTL_2 for Sleep/others modes ? */
#if 0 //ndef PANTHER. Need to update
    d2041_configure_mctl(d2041, rdev);
#endif //  PANTHER

    /* DLG TODO is to enable the mControl Enable bit as a part of system feature
       so that the different modes can be used.
       Question is do we have to disable mControl Enable bit before we shutdown or
       is it handled by digital??
     */
     /* TODO MW: find out if we want to have mctl functionality enabled/disabled in runtime
      * (some interface function need to be omplemented) or we want to have it configured
      * during boot time (configurable by kernel parameter) ?
      */

    return 0;
}

static int d2041_regulator_remove(struct platform_device *pdev)
{
    struct regulator_dev *rdev = platform_get_drvdata(pdev);
    struct d2041 *d2041 = rdev_get_drvdata(rdev);
    int i;

    for (i = 0; i < ARRAY_SIZE(d2041->pmic.pdev); i++)
        platform_device_unregister(d2041->pmic.pdev[i]);

    regulator_unregister(rdev);

    return 0;
}

static int d2041_register_regulator(struct d2041 *d2041, int reg,
              struct regulator_init_data *initdata)
{
    struct platform_device *pdev;
    int ret;
    if (reg < D2041_BUCK_1 || reg >= D2041_NUMBER_OF_REGULATORS)
        return -EINVAL;

    if (d2041->pmic.pdev[reg])
        return -EBUSY;

    pdev = platform_device_alloc(DRIVER_NAME, reg);
    if (!pdev)
        return -ENOMEM;

    d2041->pmic.pdev[reg] = pdev;

    initdata->driver_data = d2041;

    pdev->dev.platform_data = initdata;
    pdev->dev.parent = d2041->dev;
    platform_set_drvdata(pdev, d2041);

    ret = platform_device_add(pdev);

    if (ret != 0) {
        dev_err(d2041->dev, "Failed to register regulator %d: %d\n",
            reg, ret);
        platform_device_del(pdev);
        d2041->pmic.pdev[reg] = NULL;
    }
    return ret;
}


static struct platform_driver d2041_regulator_driver = {
    .probe = d2041_regulator_probe,
    .remove = d2041_regulator_remove,
    .driver        = {
        .name    = DRIVER_NAME,
    },
};

static int __init d2041_regulator_init(void)
{
    return platform_driver_register(&d2041_regulator_driver);
}
subsys_initcall(d2041_regulator_init);

static void __exit d2041_regulator_exit(void)
{
    platform_driver_unregister(&d2041_regulator_driver);
}
module_exit(d2041_regulator_exit);

/* Module information */
MODULE_AUTHOR("Dialog Semiconductor Ltd <divyang.patel@diasemi.com>");
MODULE_DESCRIPTION("D2041 voltage and current regulator driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);

