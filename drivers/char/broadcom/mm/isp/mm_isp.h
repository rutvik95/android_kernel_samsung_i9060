/*******************************************************************************
Copyright 2010 Broadcom Corporation.  All rights reserved.

Unless you and Broadcom execute a separate written software license agreement
governing use of this software, this software is licensed to you under the
terms of the GNU General Public License version 2, available at
http://www.gnu.org/copyleft/gpl.html (the "GPL").

Notwithstanding the above, under no circumstances may you combine this software
in any way with any other Broadcom software provided under a license other than
the GPL, without Broadcom's express prior written consent.
*******************************************************************************/
#ifndef _ISP_H_
#define _ISP_H_
#include <linux/ioctl.h>

#define ISP_VERSION 460
#define HERA_ISP    1
#define HERA_A0_ISP 0


#include <linux/broadcom/mm_fw_usr_ifc.h>

#define BCM_ISP_MAGIC	'I'

#define SINT4P12(x) ((signed short)(((x) > 0) ? ((x)*4096+0.5) : \
						((x)*4096-0.5)))
#define UINT4P12(x) ((uint16_t)((x)*4096+0.5))

#define TRUNC12(x) (((int)(x*4096))&0xFFFF)

#define MAX_NUM_ISP_REGS 70

struct regs_t {
	unsigned long offset;
	unsigned long value;
};

struct isp_job_post_t {
	struct regs_t isp_regs[MAX_NUM_ISP_REGS];
	unsigned int num_regs;
};

#endif /*_ISP_H_*/

