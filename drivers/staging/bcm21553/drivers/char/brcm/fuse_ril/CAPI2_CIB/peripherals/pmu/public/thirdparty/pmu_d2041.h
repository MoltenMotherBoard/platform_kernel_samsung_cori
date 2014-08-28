/*
 * pmic.h  --  Power Managment Driver for Dialog D2041 PMIC
 *
 * Copyright 2011 Dialog Semiconductor Ltd.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#ifndef __PMU_D2041_H
#define __PMU_D2041_H

// DLG TODO: Need to be checked !!!
typedef enum {
	PMU_IRQID_INT1_PONKEYBR,      // R&C  on key rising
	PMU_IRQID_INT1_PONKEYBF,      // R&C  on key falling
	PMU_IRQID_INT1_PONKEYBH,      // R&C  on key pressed > hold debounce time
	PMU_IRQID_INT1_RTC60S,        // R&C  real-time clock needs adjustment
	PMU_IRQID_INT1_RTCA,          // R&C  alarm
	PMU_IRQID_INT1_SMPL,          // R&C  one minute elapsed
	PMU_IRQID_INT1_RTC1S,         // R&C  SMPL

	PMU_IRQID_INT2_CHGINS,        // R&C  wall charger inserted (high clears CHGRM)
	PMU_IRQID_INT2_CHGRM,         // R&C  wall charger removed (high clears CHGINS)
	PMU_IRQID_INT2_CHGERR,        // R&C  wall charger error (V too high)
	PMU_IRQID_INT2_EOC,           // R&C  wall/usb charging done
	PMU_IRQID_INT2_MBCCHGERR,     // R&C  main battery charge error (over time)

	PMU_IRQID_INT3_JIGONBINS,     // R&C  JIGONB insertion : accessory inserted (high clears ACDRM)
	PMU_IRQID_INT3_JIGONBRM,      // R&C  JIGONB removal : accessory removed (high clears ACDINS)
	PMU_IRQID_INT3_MUIC,          // R&C  MUIC interrupt
	PMU_IRQID_INT3_VERYLOWBAT,    // R&C  The main battery voltage falling below 3.1V

	PMU_MUICID_INT1_ADC,           // R&C ADC Change Interrupt
	PMU_MUICID_INT1_ADCLOW,   	   // R&C ADC Low bit change Interrupt
	PMU_MUICID_INT1_ADCERROR,	   // R&C ADC Error Interrupt

	PMU_MUICID_INT2_CHGTYP,        // R&C Charge Type Interrupt
	PMU_MUICID_INT2_CHGDETRUN,     // R&C Charger Detection Running Status Interrupt
	PMU_MUICID_INT2_DCDTMR,        // R&C DCD Timer Interrupt
	PMU_MUICID_INT2_DBCHG,         // R&C Dead Battery Charging Interrupt
	PMU_MUICID_INT2_VBVOLT,        // R&C VB Voltage Interrupt

	PMU_MUICID_INT3_OVP,           // R&C VB Over Voltage Protection Interrupt

	PMU_TOTAL_IRQ
} PMU_InterruptId_t;


#endif  /* __PMU_D2041_H */
