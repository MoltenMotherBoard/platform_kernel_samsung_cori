/*
 * d2041-audio.c: Audio amplifier driver for D2041
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

#include <linux/module.h>
#include <linux/d2041/d2041_reg.h>
#include <linux/d2041/core.h>
#include <linux/d2041/audio.h>
#include <linux/delay.h>
#define DRIVER_NAME "d2041-audio"

static struct d2041 *d2041;
struct d2041_audio *dlg_audio = NULL;
static u8 dlg_audio_sleep = true;
static inline u8 audio_read(int reg)
{
    u8 val;

    d2041_reg_read(d2041, reg, &val);

    return val;
}

static inline int audio_write(int reg, u8 const val)
{
    return d2041_reg_write(d2041, reg, val);
}

static inline int audio_read_block(int start_reg, int len, u8 *dest)
{
    return d2041_block_read(d2041, start_reg, len, dest);
}

static inline int audio_write_block(int start_reg, int len, u8 *src)
{
    return d2041_block_write(d2041, start_reg, len, src);
}

static inline u8 audio_set_bits(int reg, u8 mask)
{
    return d2041_set_bits(d2041, reg, mask);
}

static inline int audio_clear_bits(int reg, u8 mask)
{
    return d2041_clear_bits(d2041, reg, mask);
}


/*
 * API functions
 */

static inline int d2041_audio_poweron(bool on)
{
    int ret = 0;
    int (*set_bits)(struct d2041 * const d2041, u8 const reg, u8 const mask);

	if(on==true)		
		audio_write(D2041_LDO_AUD_MCTL_REG, 0x44);
		
    set_bits = (on) ? d2041_set_bits : d2041_clear_bits;

    ret |= set_bits(d2041, D2041_PREAMP_A_CTRL1_REG, D2041_PREAMP_EN);
    ret |= set_bits(d2041, D2041_PREAMP_B_CTRL1_REG, D2041_PREAMP_EN);
    ret |= set_bits(d2041, D2041_MXHPR_CTRL_REG, D2041_MX_EN);
    ret |= set_bits(d2041, D2041_MXHPL_CTRL_REG, D2041_MX_EN);
    ret |= set_bits(d2041, D2041_CP_CTRL_REG, D2041_CP_EN);

	if(on==false){
		audio_write(D2041_LDO_AUD_MCTL_REG, 0x00);
	}

    return ret;
}

int d2041_audio_hs_poweron1(bool on)
{
    int ret = 0;
    u8 regval;

    dlg_info("d2041_audio_hs_poweron on=%d speaker_power=%d \n", on,dlg_audio->IHFenabled);     

    if(on) 
    {
        //if(dlg_audio->IHFenabled==false)
        //{
        //    audio_write(D2041_LDO_AUD_MCTL_REG, 0x44); //AUD_LDO on
        //}
        

        if (!dlg_audio_sleep) {

                regval = audio_read(D2041_PREAMP_B_CTRL1_REG) & ~D2041_PREAMP_VOL;
                regval |= (D2041_PREAMP_EN | D2041_PREAMP_ZC_EN);
                regval &= ~D2041_PREAMP_MUTE;

                ret |= audio_write(D2041_PREAMP_B_CTRL1_REG,regval);
                ret |= audio_write(D2041_PREAMP_B_CTRL2_REG,0x00); 

                mdelay(50);
                
                ret |= audio_write(D2041_HP_L_GAIN_REG, 0x1C);
                ret |= audio_write(D2041_HP_R_GAIN_REG, 0x1C);

                ret |= audio_write(D2041_HP_L_CTRL_REG,0xa0); 
                ret |= audio_write(D2041_HP_R_CTRL_REG,0xa0);

                for(regval = 0x1d; regval < (dlg_audio->hs_pga_gain+1); regval++){
                        ret |= audio_write(D2041_HP_L_GAIN_REG, regval);
                        ret |= audio_write(D2041_HP_R_GAIN_REG, regval);
                } 
                
                regval = audio_read(D2041_PREAMP_B_CTRL1_REG);
                regval |= ((dlg_audio->hs_pre_gain << D2041_PREAMP_VOL_SHIFT) & D2041_PREAMP_VOL);
                regval &= ~D2041_PREAMP_ZC_EN;       
                ret |= audio_write(D2041_PREAMP_B_CTRL1_REG,regval);

                ret |= audio_write(D2041_CP_CTRL_REG,0xCD);//charge Pump enable, CPVDD/1
                ret |= audio_write(D2041_CP_DELAY_REG,0x85); //check
                mdelay(50);
		
        } else {
                regval = audio_read(D2041_PREAMP_B_CTRL1_REG) & ~D2041_PREAMP_VOL;
                regval |= ((dlg_audio->hs_pre_gain << D2041_PREAMP_VOL_SHIFT) & D2041_PREAMP_VOL);
                regval |= (D2041_PREAMP_EN | D2041_PREAMP_ZC_EN);
                regval &= ~D2041_PREAMP_MUTE;

                ret |= audio_write(D2041_PREAMP_B_CTRL1_REG,regval);
                ret |= audio_write(D2041_PREAMP_B_CTRL2_REG,0x00); 

                ret |= audio_write(D2041_MXHPR_CTRL_REG,0x11);
                ret |= audio_write(D2041_MXHPL_CTRL_REG,0x09);   

                regval = audio_read(D2041_HP_L_GAIN_REG) & ~D2042_HP_AMP_GAIN & ~D2041_HP_AMP_MUTE_EN; 
                ret |= audio_write(D2041_HP_L_GAIN_REG, regval | (dlg_audio->hs_pga_gain & D2042_HP_AMP_GAIN));

                regval = audio_read(D2041_HP_R_GAIN_REG) & ~D2042_HP_AMP_GAIN & ~D2041_HP_AMP_MUTE_EN;
                ret |= audio_write(D2041_HP_R_GAIN_REG, regval | (dlg_audio->hs_pga_gain & D2042_HP_AMP_GAIN));

                ret |= audio_write(D2041_CP_CTRL_REG,0xC9);
                ret |= audio_write(D2041_CP_DELAY_REG,0x84);

                ret |= audio_write(D2041_CP_DETECTOR_REG,0x00);
                ret |= audio_write(D2041_CP_VOL_THRESHOLD_REG,0x32);
                ret |= audio_write(D2041_BIAS_CTRL_REG,0x3F);

                mdelay(65);

                regval = audio_read(D2041_PREAMP_B_CTRL1_REG);
                regval &= ~D2041_PREAMP_ZC_EN;       
                ret |= audio_write(D2041_PREAMP_B_CTRL1_REG,regval);
                
                ret |= audio_write(D2041_HP_L_CTRL_REG,0xa0);
                ret |= audio_write(D2041_HP_R_CTRL_REG,0xa0);

                mdelay(25);
                ret |= audio_write(D2041_CP_CTRL_REG,0xCD);
                ret |= audio_write(D2041_CP_DELAY_REG,0x85);
                mdelay(40);
        }
        dlg_audio->HSenabled = true;
        
     }
     else
     {

        ret |= audio_write(D2041_CP_CTRL_REG,0xC9);//charge Pump VDD/2 enable
        ret |= audio_write(D2041_CP_DELAY_REG,0x84);
        mdelay(20); 
        audio_write(D2041_HP_L_CTRL_REG,0x90); 
        audio_write(D2041_HP_R_CTRL_REG,0x90); 

        regval = audio_read(D2041_PREAMP_B_CTRL1_REG) & ~D2041_PREAMP_VOL;
        regval |= (D2041_PREAMP_MUTE);
        ret |= audio_write(D2041_PREAMP_B_CTRL1_REG,regval);

        regval = audio_read(D2041_HP_L_GAIN_REG);
        while(regval--){
                audio_write(D2041_HP_L_GAIN_REG,regval);
                audio_write(D2041_HP_R_GAIN_REG,regval); 
        }
        
        mdelay(30); 
        audio_write(D2041_HP_L_CTRL_REG,0xD0); 
        audio_write(D2041_HP_R_CTRL_REG,0xD0); 

        dlg_audio_sleep = false;
        dlg_audio->HSenabled = false;

       // if(dlg_audio->IHFenabled==false)
       //     audio_write(D2041_LDO_AUD_MCTL_REG, 0x00); //AUD_LDO off
     }
     
    return ret;
}

int d2041_audio_hs_poweron(bool on)
{
    if(dlg_audio->AudioStart==0)
    {
        del_timer(&dlg_audio->timer); 
        mod_timer(&dlg_audio->timer, jiffies + msecs_to_jiffies(10));
        dlg_audio->AudioStart=1;
    }
    else
    {
        d2041_audio_hs_poweron1(on);
    }
    return 0;
}
EXPORT_SYMBOL(d2041_audio_hs_poweron);


int d2041_audio_hs_shortcircuit_enable(bool en)
{
    dlg_err("%s(%s): NOT SUPPORTED.\n", __func__, en ? "on" : "off");
    return -EINVAL;
}
EXPORT_SYMBOL(d2041_audio_hs_shortcircuit_enable);

int d2041_audio_hs_set_gain(enum d2041_audio_output_sel hs_path_sel, enum d2041_hp_vol_val hs_gain_val)
{
    int ret = 0;
    u8 regval;

    dlg_info("d2041_audio_hs_set_gain path=%d gain=%d \n", hs_path_sel, hs_gain_val);

    if(dlg_audio->HSenabled==true)
    {
            if (hs_path_sel & D2041_OUT_HPL) 
            {
                   regval = audio_read(D2041_HP_L_GAIN_REG) & ~D2042_HP_AMP_GAIN;
                   ret |= audio_write(D2041_HP_L_GAIN_REG, regval | (hs_gain_val & D2042_HP_AMP_GAIN));
                   ret |= audio_clear_bits(D2041_HP_L_CTRL_REG, D2041_HP_AMP_MUTE_EN);
            }

            if (hs_path_sel & D2041_OUT_HPR) 
            {
                   regval = audio_read(D2041_HP_R_GAIN_REG) & ~D2042_HP_AMP_GAIN;
                   ret |= audio_write(D2041_HP_R_GAIN_REG, regval | (hs_gain_val & D2042_HP_AMP_GAIN));
                   ret |= audio_clear_bits(D2041_HP_R_CTRL_REG, D2041_HP_AMP_MUTE_EN);
            }            
            
    }
    dlg_audio->hs_pga_gain=hs_gain_val;
    
    return ret;
}
EXPORT_SYMBOL(d2041_audio_hs_set_gain);

int d2041_audio_hs_ihf_poweron1(void) //speaker on/off
{
    int ret = 0;
     
    dlg_info("d2041_audio_hs_ihf_poweron  headset_power=%d \n", dlg_audio->HSenabled);
    
   // if(dlg_audio->HSenabled==false)
   //     audio_write(D2041_LDO_AUD_MCTL_REG, 0x44); //AUD_LDO on

    ret |= audio_write(D2041_PREAMP_A_CTRL2_REG,0x03); //pre amp A fully differential

    ret |= audio_set_bits(D2041_PREAMP_A_CTRL1_REG, D2041_PREAMP_EN);
    ret |= audio_clear_bits(D2041_PREAMP_A_CTRL1_REG, D2041_PREAMP_MUTE);
    
    ret |= audio_write(D2041_MXSP_CTRL_REG,0x07);//Mixer set to A1+A2
    ret |= audio_set_bits(D2041_SP_CTRL_REG, D2041_SP_EN);
    ret |= audio_clear_bits(D2041_SP_CTRL_REG, D2041_SP_MUTE);

    ret |= audio_write(D2041_SP_CFG1_REG,0x00); //no setting
    ret |= audio_write(D2041_SP_CFG2_REG,0x00); //no setting
    //ret |= audio_write(D2041_SP_NG1_REG,0x00); //Noise Gate Diable
    //ret |= audio_write(D2041_SP_NG2_REG,0x00); //Noise Gate Diable
    ret |= audio_write(D2041_SP_NON_CLIP_ZC_REG,0x00); //Zero cross and Non clip disable
    ret |= audio_write(D2041_SP_NON_CLIP_REG,0x00);  //no define
    ret |= audio_write(D2041_SP_PWR_REG,0x00); //power limiter disable
    ret |= audio_write(D2041_BIAS_CTRL_REG,0x06); //bias control    
	
    dlg_audio->IHFenabled = true;

    return ret;
}

int d2041_audio_hs_ihf_poweron(void)
{
    if(dlg_audio->AudioStart==0)
    {
        del_timer(&dlg_audio->timer); 
        mod_timer(&dlg_audio->timer, jiffies + msecs_to_jiffies(20));
        dlg_audio->AudioStart=2;
    }
    else
    {
        d2041_audio_hs_ihf_poweron1();
    }
    return 0;
}

EXPORT_SYMBOL(d2041_audio_hs_ihf_poweron);

int d2041_audio_hs_ihf_poweroff(void)
{
    int ret = 0;
    
    dlg_info("d2041_audio_hs_ihf_poweroff  headset_power=%d \n", dlg_audio->HSenabled);

    audio_write(D2041_SP_CTRL_REG,0x32);
    audio_write(D2041_MXSP_CTRL_REG,0x00);
    audio_write(D2041_PREAMP_A_CTRL1_REG,0x32);    

    dlg_audio->IHFenabled = false;

    //if(dlg_audio->HSenabled==false)
    //    audio_write(D2041_LDO_AUD_MCTL_REG, 0x00); //AUD_LDO off
        
    dlg_audio->IHFenabled = false;

    return ret;
}
EXPORT_SYMBOL(d2041_audio_hs_ihf_poweroff);

int d2041_audio_hs_ihf_enable_bypass(bool en)
{
    dlg_err("%s(%s): NOT SUPPORTED.\n", __func__, en ? "on" : "off");
    return -EINVAL;
}
EXPORT_SYMBOL(d2041_audio_hs_ihf_enable_bypass);

int d2041_audio_hs_ihf_set_gain(enum d2041_sp_vol_val ihfgain_val)
{
    u8 regval;
     
    dlg_info("[%s]-ihfgain_val[0x%x]\n", __func__, ihfgain_val);
    //ihfgain_val = 0x33;
    
    regval = audio_read(D2041_SP_CTRL_REG);
    if (ihfgain_val == D2041_SPVOL_MUTE) 
    {
        regval |= D2041_SP_MUTE;
    } 
    else 
    {
        regval &= ~(D2041_SP_VOL | D2041_SP_MUTE);
        regval |= (ihfgain_val << D2041_SP_VOL_SHIFT) & D2041_SP_VOL;			
   }
    return  audio_write(D2041_SP_CTRL_REG,regval);
}
EXPORT_SYMBOL(d2041_audio_hs_ihf_set_gain);

int d2041_audio_set_mixer_input(enum d2041_audio_output_sel path_sel, enum d2041_audio_input_val input_val)
{
    int reg;
    u8 regval;

    if(path_sel == D2041_OUT_HPL)
        reg = D2041_MXHPL_CTRL_REG;
    else if(path_sel == D2041_OUT_HPR)
        reg = D2041_MXHPR_CTRL_REG;
    else if(path_sel == D2041_OUT_SPKR)
        reg = D2041_MXSP_CTRL_REG;
    else
        return -EINVAL;

    regval = audio_read(reg) & ~D2041_MX_SEL;
    regval |= (input_val << D2041_MX_SEL_SHIFT) & D2041_MX_SEL;
    
    return audio_write(reg,regval);
}
EXPORT_SYMBOL(d2041_audio_set_mixer_input);

int d2041_audio_set_input_mode(enum d2041_input_path_sel inpath_sel, enum d2041_input_mode_val mode_val)
{
    int reg;
    u8 regval;

    if (inpath_sel == D2041_INPUTA)
        reg = D2041_PREAMP_A_CTRL2_REG;
    else if (inpath_sel == D2041_INPUTB)
        reg = D2041_PREAMP_B_CTRL2_REG;
    else
        return -EINVAL;

    regval = audio_read(reg) & ~D2041_PREAMP_CFG;
    regval |= mode_val & D2041_PREAMP_CFG;
    
    return audio_write(reg,regval);
}
EXPORT_SYMBOL(d2041_audio_set_input_mode);

int d2041_audio_set_input_preamp_gain(enum d2041_input_path_sel inpath_sel, enum d2041_preamp_gain_val pagain_val)
{
    int reg;
    u8 regval;

    if (inpath_sel == D2041_INPUTA)
        reg = D2041_PREAMP_A_CTRL1_REG;
    else if (inpath_sel == D2041_INPUTB)
        reg = D2041_PREAMP_B_CTRL1_REG;
    else
        return -EINVAL;

    regval = audio_read(reg) & ~D2041_PREAMP_VOL;
    regval |= (pagain_val << D2041_PREAMP_VOL_SHIFT) & D2041_PREAMP_VOL;

    dlg_info("[%s]-addr[0x%x] ihfgain_val[0x%x]\n", __func__, reg,regval);

    return audio_write(reg,regval);
}
EXPORT_SYMBOL(d2041_audio_set_input_preamp_gain);

int d2041_audio_hs_preamp_gain(enum d2041_preamp_gain_val hsgain_val)
{
    u8 regval = 0;
    int ret = 0;

    dlg_info("d2041_audio_hs_preamp_gain hsgain_val=%d \n",hsgain_val);

    if(dlg_audio->HSenabled==true)
    {
            regval = audio_read(D2041_PREAMP_B_CTRL1_REG) & ~D2041_PREAMP_VOL;
            regval |= (hsgain_val << D2041_PREAMP_VOL_SHIFT) & D2041_PREAMP_VOL;
            ret = audio_write(D2041_PREAMP_B_CTRL1_REG,regval);
    }
    dlg_audio->hs_pre_gain = hsgain_val;
    
    return ret;
}
EXPORT_SYMBOL(d2041_audio_hs_preamp_gain);

int d2041_audio_ihf_preamp_gain(enum d2041_preamp_gain_val ihfgain_val)
{
    u8 regval;
	
    dlg_info("d2041_audio_ihf_preamp_gain gain=%d \n", ihfgain_val);
    
    regval = audio_read(D2041_PREAMP_A_CTRL1_REG) & ~D2041_PREAMP_VOL;
    regval |= (ihfgain_val << D2041_PREAMP_VOL_SHIFT) & D2041_PREAMP_VOL;

    return audio_write(D2041_PREAMP_A_CTRL1_REG,regval);
}
EXPORT_SYMBOL(d2041_audio_ihf_preamp_gain);

int d2041_audio_enable_zcd(int enable)
{
    int ret = 0;

    dlg_info("d2041_audio_enable_zcd =%d \n",enable);
    
    ret |= audio_set_bits(D2041_PREAMP_A_CTRL1_REG, D2041_PREAMP_ZC_EN);
    ret |= audio_set_bits(D2041_PREAMP_B_CTRL1_REG, D2041_PREAMP_ZC_EN);
    ret |= audio_set_bits(D2041_HP_L_CTRL_REG, D2041_HP_AMP_ZC_EN);
    ret |= audio_set_bits(D2041_HP_R_CTRL_REG, D2041_HP_AMP_ZC_EN);
    ret |= audio_set_bits(D2041_SP_NON_CLIP_ZC_REG, D2041_SP_ZC_EN);

    return ret;

}
EXPORT_SYMBOL(d2041_audio_enable_zcd);

int d2041_audio_enable_vol_slew(int enable)
{
    dlg_err("%s(%s): NOT SUPPORTED.\n", __func__, enable ? "on" : "off");
    return -EINVAL;
}
EXPORT_SYMBOL(d2041_audio_enable_vol_slew);

int d2041_set_hs_noise_gate(u16 regval)		
{
    int ret = 0;
    
    dlg_info("d2041_set_hs_noise_gate regval=%d \n",regval);
    
    ret |= audio_write(D2041_HP_NG1_REG, (u8)(regval & 0x00FF));
    ret |= audio_write(D2041_HP_NG2_REG, (u8)((regval >> 8) & 0x00FF));

    return ret;
}
EXPORT_SYMBOL(d2041_set_hs_noise_gate);

int d2041_set_ihf_noise_gate(u16 regval)
{
    int ret = 0;

    dlg_info("d2041_set_ihf_noise_gate = %d \n",regval);
    
    ret |= audio_write(D2041_SP_NG1_REG, (u8)(regval & 0x00FF));
    ret |= audio_write(D2041_SP_NG2_REG, (u8)((regval >> 8) & 0x00FF));

    return ret;
}
EXPORT_SYMBOL(d2041_set_ihf_noise_gate);

int d2041_set_ihf_none_clip(u16 regval)
{
    int ret = 0;

    dlg_info("d2041_set_ihf_none_clip = %d \n",regval);
    
    ret |= audio_write(D2041_SP_NON_CLIP_ZC_REG, (u8)(regval & 0x00FF));
    ret |= audio_write(D2041_SP_NON_CLIP_REG, (u8)((regval >> 8) & 0x00FF));

    return ret;
}
EXPORT_SYMBOL(d2041_set_ihf_none_clip);

int d2041_set_ihf_pwr(u8 regval)
{
    return audio_write(D2041_SP_PWR_REG, regval);
}
EXPORT_SYMBOL(d2041_set_ihf_pwr);

static void d2041_audio_timer(unsigned long data)
{
    schedule_work(&dlg_audio->work);
}

static void d2041_audio_worker(struct work_struct *work)
{
    if(dlg_audio->AudioStart==1) //hs
    {
        d2041_audio_hs_poweron1(1);
    }
    else if(dlg_audio->AudioStart==2) //ihf
    {
        d2041_audio_hs_ihf_poweron1();
    }

    dlg_audio->AudioStart=3;
    return; 
}
static int d2041_audio_probe(struct platform_device *pdev)
{
    int ret = 0;

    d2041 = platform_get_drvdata(pdev);
    if(!d2041)
        return -EINVAL;
        
    dev_info(d2041->dev, "Starting audio\n");
    dlg_audio = &d2041->audio;

    dlg_audio->HSenabled = false;
    dlg_audio->IHFenabled = false;
    dlg_audio->AudioStart= 0;

    // Audio mcontrol
    audio_write(D2041_LDO_AUD_MCTL_REG, 0x44); 

    INIT_WORK(&dlg_audio->work,d2041_audio_worker);
    init_timer(&dlg_audio->timer);
	dlg_audio->timer.function = d2041_audio_timer;
	dlg_audio->timer.data = (unsigned long) d2041;
    
    dev_info(d2041->dev, "\nAudio started.\n");
    return ret;
}

static int __devexit d2041_audio_remove(struct platform_device *pdev)
{
    struct d2041_audio *dlg_audio = NULL;

    d2041 = platform_get_drvdata(pdev);
    if(!d2041)
        return -EINVAL;
    dlg_audio = &d2041->audio;

    d2041 = NULL;

    return 0;
}

#ifdef CONFIG_PM
static int d2041_audio_suspend(struct device *dev)
{
    //u8 regval = 0;

    if( dlg_audio->HSenabled ==true || dlg_audio->IHFenabled == true)
        return 0;
    
    dev_info(d2041->dev,"d2041_audio_suspend \n");

    audio_write(D2041_LDO_AUD_MCTL_REG, 0x00); 
    audio_write(D2041_HP_L_CTRL_REG,0x20); 
    audio_write(D2041_HP_R_CTRL_REG,0x20); 
    dlg_audio_sleep = true;
    return 0;
}

static int d2041_audio_resume(struct device *dev)
{
        dev_info(d2041->dev,"d2041_audio_resume \n");
        audio_write(D2041_LDO_AUD_MCTL_REG, 0x44); 
        mdelay(30);
        return 0;
}
#else
#define d2041_audio_suspend NULL
#define d2041_audio_resume NULL
#endif

static struct dev_pm_ops d2041_audio_pm_ops = {
	.suspend = d2041_audio_suspend,
	.resume = d2041_audio_resume,
};


static struct platform_driver d2041_audio_driver = {
        .probe = d2041_audio_probe,
        .remove = __devexit_p(d2041_audio_remove),
        .driver = {
                .name = DRIVER_NAME,
                .owner = THIS_MODULE,
                .pm = &d2041_audio_pm_ops,
        },
};


static int __init d2041_audio_init(void)
{
        return platform_driver_register(&d2041_audio_driver);
}
late_initcall(d2041_audio_init);

static void __exit d2041_audio_exit(void)
{
        platform_driver_unregister(&d2041_audio_driver);
}
module_exit(d2041_audio_exit);

MODULE_DESCRIPTION("D2041 Audio amplifier driver");
MODULE_AUTHOR("Mariusz Wojtasik <mariusz.wojtasik@diasemi.com>");
MODULE_LICENSE("GPL v2");

