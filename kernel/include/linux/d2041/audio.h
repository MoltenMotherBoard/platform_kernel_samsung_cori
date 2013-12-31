/*
 * audio.h: Audio amplifier driver for D2041
 *
 * Copyright(c) 2011 Dialog Semiconductor Ltd.
 * Author: Mariusz Wojtasik <mariusz.wojtasik@diasemi.com>
 *
 * This program is free software; you can redistribute  it and/or modify
 * it under  the terms of  the GNU General  Public License as published by
 * the Free Software Foundation;  either version 2 of the  License, or
 * (at your option) any later version.
 *
 */

#ifndef __LINUX_D2041_AUDIO_H
#define __LINUX_D2041_AUDIO_H

#include <linux/platform_device.h>

#define D2041_MAX_GAIN_TABLE 24

enum d2041_audio_input_val {
    D2041_IN_NONE   = 0x00,
    D2041_IN_A1     = 0x01,
    D2041_IN_A2     = 0x02,
    D2041_IN_B1     = 0x04,
    D2041_IN_B2     = 0x08
};

enum d2041_input_mode_val {
    D2041_IM_TWO_SINGLE_ENDED   = 0x00,
    D2041_IM_ONE_SINGLE_ENDED_1 = 0x01,
    D2041_IM_ONE_SINGLE_ENDED_2 = 0x02,
    D2041_IM_FULLY_DIFFERENTIAL = 0x03
};
enum d2041_input_path_sel {
    D2041_INPUTA,
    D2041_INPUTB
};

enum d2041_preamp_gain_val {
    D2041_PREAMP_GAIN_NEG_6DB   = 0x07,
    D2041_PREAMP_GAIN_NEG_5DB   = 0x08,
    D2041_PREAMP_GAIN_NEG_4DB   = 0x09,
    D2041_PREAMP_GAIN_NEG_3DB   = 0x0a,
    D2041_PREAMP_GAIN_NEG_2DB   = 0x0b,
    D2041_PREAMP_GAIN_NEG_1DB   = 0x0c,
    D2041_PREAMP_GAIN_0DB       = 0x0d,
    D2041_PREAMP_GAIN_1DB       = 0x0e,
    D2041_PREAMP_GAIN_2DB       = 0x0f,
    D2041_PREAMP_GAIN_3DB       = 0x10,
    D2041_PREAMP_GAIN_4DB       = 0x11,
    D2041_PREAMP_GAIN_5DB       = 0x12,
    D2041_PREAMP_GAIN_6DB       = 0x13,
    D2041_PREAMP_GAIN_7DB       = 0x14,
    D2041_PREAMP_GAIN_8DB       = 0x15,
    D2041_PREAMP_GAIN_9DB       = 0x16,
    D2041_PREAMP_GAIN_10DB      = 0x17,
    D2041_PREAMP_GAIN_11DB      = 0x18,
    D2041_PREAMP_GAIN_12DB      = 0x19,
    D2041_PREAMP_GAIN_13DB      = 0x1a,
    D2041_PREAMP_GAIN_14DB      = 0x1b,
    D2041_PREAMP_GAIN_15DB      = 0x1c,
    D2041_PREAMP_GAIN_16DB      = 0x1d,
    D2041_PREAMP_GAIN_17DB      = 0x1e,
    D2041_PREAMP_GAIN_18DB      = 0x1f,
    D2041_PREAMP_GAIN_MUTE
};

static inline int d2041_pagain_to_reg(enum d2041_preamp_gain_val gain_val)
{
    return (int)gain_val;
}

static inline enum d2041_preamp_gain_val d2041_reg_to_pagain(int reg)
{
    return ((reg < D2041_PREAMP_GAIN_NEG_6DB) ? D2041_PREAMP_GAIN_NEG_6DB :
            (enum d2041_preamp_gain_val)reg);
}

static inline int d2041_pagain_to_db(enum d2041_preamp_gain_val gain_val)
{
    return (int)gain_val - (int)D2041_PREAMP_GAIN_0DB;
}

static inline enum d2041_preamp_gain_val d2041_db_to_pagain(int db)
{
    return (enum d2041_preamp_gain_val)(db + (int)D2041_PREAMP_GAIN_0DB);
}

enum d2041_mixer_selector_val {
    D2041_MSEL_A1   = 0x01, /* if A is fully diff. then selects A2 as the inverting input */
    D2041_MSEL_B1   = 0x02, /* if B is fully diff. then selects B2 as the inverting input */
    D2041_MSEL_A2   = 0x04, /* if A is fully diff. then selects A1 as the non-inverting input */
    D2041_MSEL_B2   = 0x08  /* if B is fully diff. then selects B1 as the non-inverting input */
};

enum d2041_mixer_attenuation_val {
    D2041_MIX_ATT_0DB       = 0x00, /* attenuation = 0dB */
    D2041_MIX_ATT_NEG_6DB   = 0x01, /* attenuation = -6dB */
    D2041_MIX_ATT_NEG_9DB5  = 0x02, /* attenuation = -9.5dB */
    D2041_MIX_ATT_NEG_12DB  = 0x03  /* attenuation = -12dB */
};

enum d2041_hp_vol_val {
    D2041_HPVOL_NEG_57DB,   D2041_HPVOL_NEG_56DB,   D2041_HPVOL_NEG_55DB,   D2041_HPVOL_NEG_54DB,
    D2041_HPVOL_NEG_53DB,   D2041_HPVOL_NEG_52DB,   D2041_HPVOL_NEG_51DB,   D2041_HPVOL_NEG_50DB,
    D2041_HPVOL_NEG_49DB,   D2041_HPVOL_NEG_48DB,   D2041_HPVOL_NEG_47DB,   D2041_HPVOL_NEG_46DB,
    D2041_HPVOL_NEG_45DB,   D2041_HPVOL_NEG_44DB,   D2041_HPVOL_NEG_43DB,   D2041_HPVOL_NEG_42DB,
    D2041_HPVOL_NEG_41DB,   D2041_HPVOL_NEG_40DB,   D2041_HPVOL_NEG_39DB,   D2041_HPVOL_NEG_38DB,
    D2041_HPVOL_NEG_37DB,   D2041_HPVOL_NEG_36DB,   D2041_HPVOL_NEG_35DB,   D2041_HPVOL_NEG_34DB,
    D2041_HPVOL_NEG_33DB,   D2041_HPVOL_NEG_32DB,   D2041_HPVOL_NEG_31DB,   D2041_HPVOL_NEG_30DB,
    D2041_HPVOL_NEG_29DB,   D2041_HPVOL_NEG_28DB,   D2041_HPVOL_NEG_27DB,   D2041_HPVOL_NEG_26DB,
    D2041_HPVOL_NEG_25DB,   D2041_HPVOL_NEG_24DB,   D2041_HPVOL_NEG_23DB,   D2041_HPVOL_NEG_22DB,
    D2041_HPVOL_NEG_21DB,   D2041_HPVOL_NEG_20DB,   D2041_HPVOL_NEG_19DB,   D2041_HPVOL_NEG_18DB,
    D2041_HPVOL_NEG_17DB,   D2041_HPVOL_NEG_16DB,   D2041_HPVOL_NEG_15DB,   D2041_HPVOL_NEG_14DB,
    D2041_HPVOL_NEG_13DB,   D2041_HPVOL_NEG_12DB,   D2041_HPVOL_NEG_11DB,   D2041_HPVOL_NEG_10DB,
    D2041_HPVOL_NEG_9DB,    D2041_HPVOL_NEG_8DB,    D2041_HPVOL_NEG_7DB,    D2041_HPVOL_NEG_6DB,
    D2041_HPVOL_NEG_5DB,    D2041_HPVOL_NEG_4DB,    D2041_HPVOL_NEG_3DB,    D2041_HPVOL_NEG_2DB,
    D2041_HPVOL_NEG_1DB,    D2041_HPVOL_0DB,        D2041_HPVOL_1DB,        D2041_HPVOL_2DB,
    D2041_HPVOL_3DB,        D2041_HPVOL_4DB,        D2041_HPVOL_5DB,        D2041_HPVOL_6DB,
    D2041_HPVOL_MUTE
};

static inline int d2041_hpvol_to_reg(enum d2041_hp_vol_val vol_val)
{
    return (int)vol_val;
}

static inline enum d2041_hp_vol_val d2041_reg_to_hpvol(int reg)
{
    return (enum d2041_hp_vol_val)reg;
}

static inline int d2041_hpvol_to_db(enum d2041_hp_vol_val vol_val)
{
    return (int)vol_val - (int)D2041_HPVOL_0DB;
}

static inline enum d2041_hp_vol_val d2041_db_to_hpvol(int db)
{
    return (enum d2041_hp_vol_val)(db + (int)D2041_HPVOL_0DB);
}

enum d2041_sp_vol_val {
    D2041_SPVOL_NEG_24DB    = 0x1b, D2041_SPVOL_NEG_23DB = 0x1c,
    D2041_SPVOL_NEG_22DB    = 0x1d, D2041_SPVOL_NEG_21DB = 0x1e,
    D2041_SPVOL_NEG_20DB    = 0x1f, D2041_SPVOL_NEG_19DB = 0x20,
    D2041_SPVOL_NEG_18DB    = 0x21, D2041_SPVOL_NEG_17DB = 0x22,
    D2041_SPVOL_NEG_16DB    = 0x23, D2041_SPVOL_NEG_15DB = 0x24,
    D2041_SPVOL_NEG_14DB    = 0x25, D2041_SPVOL_NEG_13DB = 0x26,
    D2041_SPVOL_NEG_12DB    = 0x27, D2041_SPVOL_NEG_11DB = 0x28,
    D2041_SPVOL_NEG_10DB    = 0x29, D2041_SPVOL_NEG_9DB = 0x2a,
    D2041_SPVOL_NEG_8DB     = 0x2b, D2041_SPVOL_NEG_7DB = 0x2c,
    D2041_SPVOL_NEG_6DB     = 0x2d, D2041_SPVOL_NEG_5DB = 0x2e,
    D2041_SPVOL_NEG_4DB     = 0x2f, D2041_SPVOL_NEG_3DB = 0x30,
    D2041_SPVOL_NEG_2DB     = 0x31, D2041_SPVOL_NEG_1DB = 0x32,
    D2041_SPVOL_0DB         = 0x33,
    D2041_SPVOL_1DB         = 0x34, D2041_SPVOL_2DB = 0x35,
    D2041_SPVOL_3DB         = 0x36, D2041_SPVOL_4DB = 0x37,
    D2041_SPVOL_5DB         = 0x38, D2041_SPVOL_6DB = 0x39,
    D2041_SPVOL_7DB         = 0x3a, D2041_SPVOL_8DB = 0x3b,
    D2041_SPVOL_9DB         = 0x3c, D2041_SPVOL_10DB = 0x3d,
    D2041_SPVOL_11DB        = 0x3e, D2041_SPVOL_12DB = 0x3f,
    D2041_SPVOL_MUTE
};

static inline int d2041_spvol_to_reg(enum d2041_sp_vol_val vol_val)
{
    return (int)vol_val;
}

static inline enum d2041_sp_vol_val d2041_reg_to_spvol(int reg)
{
    return ((reg < D2041_SPVOL_NEG_24DB) ? D2041_SPVOL_NEG_24DB :
            (enum d2041_sp_vol_val)reg);
}

static inline int d2041_spvol_to_db(enum d2041_sp_vol_val vol_val)
{
    return (int)vol_val - (int)D2041_SPVOL_0DB;
}

static inline enum d2041_sp_vol_val d2041_db_to_spvol(int db)
{
    return (enum d2041_sp_vol_val)(db + (int)D2041_SPVOL_0DB);
}


enum d2041_audio_output_sel {
    D2041_OUT_NONE  = 0x00,
    D2041_OUT_HPL   = 0x01,
    D2041_OUT_HPR   = 0x02,
    D2041_OUT_HPLR  = D2041_OUT_HPL | D2041_OUT_HPR,
    D2041_OUT_SPKR  = 0x04,
};

struct d2041_audio_platform_data
{
    u8 ina_def_mode;
    u8 inb_def_mode;
    u8 ina_def_preampgain;
    u8 inb_def_preampgain;

    u8 lhs_def_mixer_in;
    u8 rhs_def_mixer_in;
    u8 ihf_def_mixer_in;

    int hs_input_path;
    int ihf_input_path;
};

struct d2041_audio {
	struct platform_device  *pdev;
    bool IHFenabled;
    bool HSenabled;
    u8 hs_pga_gain;
    u8 hs_pre_gain;
    struct timer_list timer;
	struct work_struct work;
    u8 AudioStart; //0=not started, 1=hs starting, 2=ihf starting, 3=started
};

int d2041_audio_hs_poweron(bool on);
int d2041_audio_hs_shortcircuit_enable(bool en);
int d2041_audio_hs_set_gain(enum d2041_audio_output_sel hs_path_sel, enum d2041_hp_vol_val hs_gain_val);
int d2041_audio_hs_ihf_poweron(void);
int d2041_audio_hs_ihf_poweroff(void);
int d2041_audio_hs_ihf_enable_bypass(bool en);
int d2041_audio_hs_ihf_set_gain(enum d2041_sp_vol_val ihfgain_val);
int d2041_audio_set_mixer_input(enum d2041_audio_output_sel path_sel, enum d2041_audio_input_val input_val);
int d2041_audio_set_input_mode(enum d2041_input_path_sel inpath_sel, enum d2041_input_mode_val mode_val);
int d2041_audio_set_input_preamp_gain(enum d2041_input_path_sel inpath_sel, enum d2041_preamp_gain_val pagain_val);
int d2041_audio_hs_preamp_gain(enum d2041_preamp_gain_val hsgain_val);
int d2041_audio_ihf_preamp_gain(enum d2041_preamp_gain_val ihfgain_val);
int d2041_audio_enable_zcd(int enable);
int d2041_audio_enable_vol_slew(int enable);
int d2041_set_hs_noise_gate(u16 regval);		
int d2041_set_ihf_noise_gate(u16 regval);
int d2041_set_ihf_none_clip(u16 regval);
int d2041_set_ihf_pwr(u8 regval);

#endif /* __LINUX_D2041_AUDIO_H */
