/* linux/tsu8111.h
 *
 * header for TSU8111 USB switch device.
 *
 * Copyright (c) by Wonguk Jeong <wonguk.jeong@samsung.com>
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

#ifndef _TSU8111_H_
#define _TSU8111_H_

#include <linux/types.h>
#include <linux/mfd/88pm860x.h>
//#include <linux/d1982/core.h>


struct tsu8111_platform_data {
       int intb_gpio;
       void (*usb_cb) (u8 attached);
       void (*uart_cb) (u8 attached);
       void (*charger_cb) (u8 attached);
       void (*jig_cb) (u8 attached);
       void (*reset_cb) (void);
	void (*mhl_cb)(u8 attached);
	void (*cardock_cb)(bool attached);
	void (*deskdock_cb)(bool attached);
	void (*id_open_cb)(void);
	//struct d1980_vbus_pdata	*vbus;	
#define TSU8111_ATTACHED (1)
#define TSU8111_DETACHED (0)
};

void tsu8111_set_switch(const char* buf);
ssize_t tsu8111_get_switch(char* buf);

#ifdef CONFIG_VIDEO_MHL_V1
void TSU8111_CheckAndHookAudioDock(void);
#endif

#endif /* _TSU8111_H_ */

