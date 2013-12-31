  /*
 *
 * model option for each sensor table
 *
. COPYRIGHT (C)  SAMSUNG Electronics CO., LTD (Suwon, Korea). 2010   
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
*/

#if !defined(_CAMACQ_MODEL_H_)
#define _CAMACQ_MODEL_H_

/* Define Model */
// #define BENI
// #define GFORCE
// #define VOLGA
#define TOTORO


/* Include */
#if defined(WIN32)

#elif defined(_LINUX_)

#if defined(GFORCE)
#include "camacq_s5k4ecgx_mipi.h"
#include "camacq_pxa950.h"
#elif defined(BENI)
#include "camacq_isx006.h"
#include "camacq_pxa950.h"
#elif defined(VOLGA)
#include "camacq_isx006.h"
#include "camacq_s5k6aafx13.h"
#elif defined(CONFIG_BCM_CAM_SR200PC10)
#include "camacq_sr200pc10.h"
#elif defined(CONFIG_BCM_CAM_SR200PC20)
#include "camacq_sr200pc20.h"
#elif defined(CONFIG_BCM_CAM_ISX005)
//#include "camacq_sr200pc10.h"
#include "camacq_isx005.h"
#elif defined(CONFIG_BCM_CAM_S5K5CCGX)
#include "camacq_s5k5ccgx.h"#elif defined(CONFIG_BCM_CAM_S5K4ECGX)
#include "camacq_s5k4ecgx.h"
#else
#include "camacq_isx006.h"
#include "camacq_s5k6aafx13.h"
#include "camacq_pxa950.h"
#endif

#endif /* WIN32 || _RTKE_ */

/* Global */
#undef GLOBAL

#if !defined(_CAMACQ_API_C_)
#define GLOBAL extern
#else
#define GLOBAL
#endif

/* Definition */

/* Enumeration */

/* Type Definition */

/* Global Value */

/* Function */

#undef GLOBAL

#endif /* _CAMACQ_MODEL_H_ */
