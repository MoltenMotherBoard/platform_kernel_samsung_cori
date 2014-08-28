/*******************************************************************************
* Copyright 2010 Broadcom Corporation.  All rights reserved.
*
*       @file   include/linux/broadcom/dialog/dialog-audio.h
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

/*
*
*****************************************************************************
*
* dialog-audio.h
*
* PURPOSE:
*
* This file sould be used by user of Audio in Dialog PMU.
*
* NOTES:
*
* ****************************************************************************/

#ifndef __DIALOG_AUDIO_H__
#define __DIALOG_AUDIO_H__

/* Dialog HS path */
typedef enum {
	AUDIO_HS_LEFT,
	AUDIO_HS_RIGHT,
	AUDIO_HS_BOTH
} dialog_audio_hs_path;

typedef struct {
	int cfg :2;
	int atk :3;
	int deb :2;
	int en  :1;
	int rms :2;
	int rel :3;
}dialog_noise_gate_t;

typedef struct {
	int zc_en :1;
	int rel :3;
	int atk :3;
	int en  :1;
	int hld :2;
	int thd :6;
}dialog_spk_nonclip_t;

typedef struct {
	int pwr :6;
	int limt_en :1;
}dialog_spk_pwr_t;



/* Function declarations */
extern int dialog_audio_hs_poweron(void);
extern int dialog_audio_hs_poweroff(void);
extern int dialog_audio_hs_set_gain(int hspath, int hsgain);
extern int dialog_audio_ihf_poweron(void);
extern int dialog_audio_ihf_poweroff(void);
extern int dialog_audio_ihf_set_gain(int ihfgain);
extern int dialog_set_hs_preamp_gain(int gain);
extern int dialog_set_ihf_preamp_gain(int gain);

extern int dialog_set_hs_noise_gate(dialog_noise_gate_t noisegate);
extern int dialog_set_ihf_noise_gate(dialog_noise_gate_t noisegate);
extern int dialog_set_ihf_none_clip(dialog_spk_nonclip_t noneclip);
extern int dialog_set_ihf_pwr(dialog_spk_pwr_t power);


#endif /* __DIALOG_AUDIO_H__ */
