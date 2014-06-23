#ifndef __MACH_QL_VX5B3D_H
#define __MACH_QL_VX5B3D_H

#define QLVX_DRIVER_VER "ql_vx5b3d ver 1.0"
/* Quicklogic VX5B3D MIPI4 to RGB on Broadcom platform
   20130320 - rev 1.  
*/

//defined to enable QL code
#ifndef CONFIG_QUICKLOGIC_VX5B3D
#define CONFIG_QUICKLOGIC_VX5B3D
#endif

//Porting: enable setup VX chip and panel from Diolan board.
//#define QL_VX_INIT_EXTERNAL

//enable debug message.
#define QL_DEBUG
#ifndef QL_DEBUG
#define QL_DBG(x...)
#define QL_DBGL(x...)
#define QL_ERR(f, x...)
#else
#define QL_DBG(f, x...) printk("[QLVX] %s: " f, __func__,## x)
#define QL_DBGL(lvl, f, x...) do {if (lvl) printk("[QLVX] %s: " f, __func__,## x); } while(0)
#define QL_ERR(f, x...) printk("[QLVX] ERROR %s: " f, __func__,## x)
#endif
#define QL_DBG_FUNC_ENTER QL_DBG("+++\n");
#define QL_DBG_FUNC_EXIT QL_DBG("---\n");

int quickvx_init(void);
int quickvx_standby(void);

#endif
