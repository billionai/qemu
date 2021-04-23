/*
 *  PowerPC CPU initialization for qemu.
 *
 *  Copyright (c) 2003-2007 Jocelyn Mayer
 *  Copyright 2011 Freescale Semiconductor, Inc.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 */

#include "disas/dis-asm.h"
#include "exec/gdbstub.h"
#include "kvm_ppc.h"
#include "sysemu/arch_init.h"
#include "sysemu/cpus.h"
#include "sysemu/hw_accel.h"
#include "sysemu/tcg.h"
#include "cpu-models.h"
#include "mmu-hash32.h"
#include "mmu-hash64.h"
#include "qemu/error-report.h"
#include "qemu/module.h"
#include "qemu/qemu-print.h"
#include "qapi/error.h"
#include "qapi/qmp/qnull.h"
#include "qapi/visitor.h"
#include "hw/qdev-properties.h"
#include "hw/ppc/ppc.h"
#include "mmu-book3s-v3.h"
#include "qemu/cutils.h"
#include "disas/capstone.h"
#include "fpu/softfloat.h"
#include "qapi/qapi-commands-machine-target.h"

/* #define PPC_DUMP_CPU */
/* #define PPC_DEBUG_SPR */
/* #define PPC_DUMP_SPR_ACCESSES */
/* #define USE_APPLE_GDB */

static inline void vscr_init(CPUPPCState *env, uint32_t val)
{
    /* Altivec always uses round-to-nearest */
    set_float_rounding_mode(float_round_nearest_even, &env->vec_status);
    helper_mtvscr(env, val);
}

/*
 * AMR     => SPR 29 (Power 2.04)
 * CTRL    => SPR 136 (Power 2.04)
 * CTRL    => SPR 152 (Power 2.04)
 * SCOMC   => SPR 276 (64 bits ?)
 * SCOMD   => SPR 277 (64 bits ?)
 * TBU40   => SPR 286 (Power 2.04 hypv)
 * HSPRG0  => SPR 304 (Power 2.04 hypv)
 * HSPRG1  => SPR 305 (Power 2.04 hypv)
 * HDSISR  => SPR 306 (Power 2.04 hypv)
 * HDAR    => SPR 307 (Power 2.04 hypv)
 * PURR    => SPR 309 (Power 2.04 hypv)
 * HDEC    => SPR 310 (Power 2.04 hypv)
 * HIOR    => SPR 311 (hypv)
 * RMOR    => SPR 312 (970)
 * HRMOR   => SPR 313 (Power 2.04 hypv)
 * HSRR0   => SPR 314 (Power 2.04 hypv)
 * HSRR1   => SPR 315 (Power 2.04 hypv)
 * LPIDR   => SPR 317 (970)
 * EPR     => SPR 702 (Power 2.04 emb)
 * perf    => 768-783 (Power 2.04)
 * perf    => 784-799 (Power 2.04)
 * PPR     => SPR 896 (Power 2.04)
 * DABRX   => 1015    (Power 2.04 hypv)
 * FPECR   => SPR 1022 (?)
 * ... and more (thermal management, performance counters, ...)
 */

/*****************************************************************************/
/* Exception vectors models                                                  */
static void init_excp_4xx_real(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_PIT]      = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_FIT]      = 0x00001010;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00001020;
    env->excp_vectors[POWERPC_EXCP_DEBUG]    = 0x00002000;
    env->ivor_mask = 0x0000FFF0UL;
    env->ivpr_mask = 0xFFFF0000UL;
    /* Hardware reset vector */
    env->hreset_vector = 0xFFFFFFFCUL;
#endif
}

static void init_excp_4xx_softmmu(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_PIT]      = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_FIT]      = 0x00001010;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00001020;
    env->excp_vectors[POWERPC_EXCP_DTLB]     = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_ITLB]     = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_DEBUG]    = 0x00002000;
    env->ivor_mask = 0x0000FFF0UL;
    env->ivpr_mask = 0xFFFF0000UL;
    /* Hardware reset vector */
    env->hreset_vector = 0xFFFFFFFCUL;
#endif
}

static void init_excp_MPC5xx(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_FPA]      = 0x00000E00;
    env->excp_vectors[POWERPC_EXCP_EMUL]     = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DABR]     = 0x00001C00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001C00;
    env->excp_vectors[POWERPC_EXCP_MEXTBR]   = 0x00001E00;
    env->excp_vectors[POWERPC_EXCP_NMEXTBR]  = 0x00001F00;
    env->ivor_mask = 0x0000FFF0UL;
    env->ivpr_mask = 0xFFFF0000UL;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_MPC8xx(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_FPA]      = 0x00000E00;
    env->excp_vectors[POWERPC_EXCP_EMUL]     = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_ITLB]     = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DTLB]     = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_ITLBE]    = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_DTLBE]    = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_DABR]     = 0x00001C00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001C00;
    env->excp_vectors[POWERPC_EXCP_MEXTBR]   = 0x00001E00;
    env->excp_vectors[POWERPC_EXCP_NMEXTBR]  = 0x00001F00;
    env->ivor_mask = 0x0000FFF0UL;
    env->ivpr_mask = 0xFFFF0000UL;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_G2(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000A00;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_e200(CPUPPCState *env, target_ulong ivpr_mask)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000FFC;
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_APU]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_FIT]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DTLB]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ITLB]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DEBUG]    = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_SPEU]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_EFPDI]    = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_EFPRI]    = 0x00000000;
    env->ivor_mask = 0x0000FFF7UL;
    env->ivpr_mask = ivpr_mask;
    /* Hardware reset vector */
    env->hreset_vector = 0xFFFFFFFCUL;
#endif
}

static void init_excp_BookE(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_CRITICAL] = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_APU]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_FIT]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DTLB]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_ITLB]     = 0x00000000;
    env->excp_vectors[POWERPC_EXCP_DEBUG]    = 0x00000000;
    env->ivor_mask = 0x0000FFF0UL;
    env->ivpr_mask = 0xFFFF0000UL;
    /* Hardware reset vector */
    env->hreset_vector = 0xFFFFFFFCUL;
#endif
}

static void init_excp_601(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_IO]       = 0x00000A00;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_RUNM]     = 0x00002000;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_602(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    /* XXX: exception prefix has a special behavior on 602 */
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_WDT]      = 0x00001500;
    env->excp_vectors[POWERPC_EXCP_EMUL]     = 0x00001600;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_603(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_604(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_7x0(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001700;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_750cl(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_750cx(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001700;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

/* XXX: Check if this is correct */
static void init_excp_7x5(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001700;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_7400(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_VPU]      = 0x00000F20;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_VPUA]     = 0x00001600;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001700;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

static void init_excp_7450(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_VPU]      = 0x00000F20;
    env->excp_vectors[POWERPC_EXCP_IFTLB]    = 0x00001000;
    env->excp_vectors[POWERPC_EXCP_DLTLB]    = 0x00001100;
    env->excp_vectors[POWERPC_EXCP_DSTLB]    = 0x00001200;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_SMI]      = 0x00001400;
    env->excp_vectors[POWERPC_EXCP_VPUA]     = 0x00001600;
    /* Hardware reset vector */
    env->hreset_vector = 0x00000100UL;
#endif
}

#if defined(TARGET_PPC64)
static void init_excp_970(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_DSEG]     = 0x00000380;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_ISEG]     = 0x00000480;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_HDECR]    = 0x00000980;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_VPU]      = 0x00000F20;
    env->excp_vectors[POWERPC_EXCP_IABR]     = 0x00001300;
    env->excp_vectors[POWERPC_EXCP_MAINT]    = 0x00001600;
    env->excp_vectors[POWERPC_EXCP_VPUA]     = 0x00001700;
    env->excp_vectors[POWERPC_EXCP_THERM]    = 0x00001800;
    /* Hardware reset vector */
    env->hreset_vector = 0x0000000000000100ULL;
#endif
}

static void init_excp_POWER7(CPUPPCState *env)
{
#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_RESET]    = 0x00000100;
    env->excp_vectors[POWERPC_EXCP_MCHECK]   = 0x00000200;
    env->excp_vectors[POWERPC_EXCP_DSI]      = 0x00000300;
    env->excp_vectors[POWERPC_EXCP_DSEG]     = 0x00000380;
    env->excp_vectors[POWERPC_EXCP_ISI]      = 0x00000400;
    env->excp_vectors[POWERPC_EXCP_ISEG]     = 0x00000480;
    env->excp_vectors[POWERPC_EXCP_EXTERNAL] = 0x00000500;
    env->excp_vectors[POWERPC_EXCP_ALIGN]    = 0x00000600;
    env->excp_vectors[POWERPC_EXCP_PROGRAM]  = 0x00000700;
    env->excp_vectors[POWERPC_EXCP_FPU]      = 0x00000800;
    env->excp_vectors[POWERPC_EXCP_DECR]     = 0x00000900;
    env->excp_vectors[POWERPC_EXCP_HDECR]    = 0x00000980;
    env->excp_vectors[POWERPC_EXCP_SYSCALL]  = 0x00000C00;
    env->excp_vectors[POWERPC_EXCP_TRACE]    = 0x00000D00;
    env->excp_vectors[POWERPC_EXCP_HDSI]     = 0x00000E00;
    env->excp_vectors[POWERPC_EXCP_HISI]     = 0x00000E20;
    env->excp_vectors[POWERPC_EXCP_HV_EMU]   = 0x00000E40;
    env->excp_vectors[POWERPC_EXCP_HV_MAINT] = 0x00000E60;
    env->excp_vectors[POWERPC_EXCP_PERFM]    = 0x00000F00;
    env->excp_vectors[POWERPC_EXCP_VPU]      = 0x00000F20;
    env->excp_vectors[POWERPC_EXCP_VSXU]     = 0x00000F40;
    /* Hardware reset vector */
    env->hreset_vector = 0x0000000000000100ULL;
#endif
}

static void init_excp_POWER8(CPUPPCState *env)
{
    init_excp_POWER7(env);

#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_SDOOR]    = 0x00000A00;
    env->excp_vectors[POWERPC_EXCP_FU]       = 0x00000F60;
    env->excp_vectors[POWERPC_EXCP_HV_FU]    = 0x00000F80;
    env->excp_vectors[POWERPC_EXCP_SDOOR_HV] = 0x00000E80;
#endif
}

static void init_excp_POWER9(CPUPPCState *env)
{
    init_excp_POWER8(env);

#if !defined(CONFIG_USER_ONLY)
    env->excp_vectors[POWERPC_EXCP_HVIRT]    = 0x00000EA0;
    env->excp_vectors[POWERPC_EXCP_SYSCALL_VECTORED] = 0x00000000;
#endif
}

static void init_excp_POWER10(CPUPPCState *env)
{
    init_excp_POWER9(env);
}

#endif

/*****************************************************************************/
/* Power management enable checks                                            */
static int check_pow_none(CPUPPCState *env)
{
    return 0;
}

static int check_pow_nocheck(CPUPPCState *env)
{
    return 1;
}

static int check_pow_hid0(CPUPPCState *env)
{
    if (env->spr[SPR_HID0] & 0x00E00000) {
        return 1;
    }

    return 0;
}

static int check_pow_hid0_74xx(CPUPPCState *env)
{
    if (env->spr[SPR_HID0] & 0x00600000) {
        return 1;
    }

    return 0;
}

static bool ppc_cpu_interrupts_big_endian_always(PowerPCCPU *cpu)
{
    return true;
}

#ifdef TARGET_PPC64
static bool ppc_cpu_interrupts_big_endian_lpcr(PowerPCCPU *cpu)
{
    return !(cpu->env.spr[SPR_LPCR] & LPCR_ILE);
}
#endif

/*****************************************************************************/
/* PowerPC implementations definitions                                       */

#define POWERPC_FAMILY(_name)                                               \
    static void                                                             \
    glue(glue(ppc_, _name), _cpu_family_class_init)(ObjectClass *, void *); \
                                                                            \
    static const TypeInfo                                                   \
    glue(glue(ppc_, _name), _cpu_family_type_info) = {                      \
        .name = stringify(_name) "-family-" TYPE_POWERPC_CPU,               \
        .parent = TYPE_POWERPC_CPU,                                         \
        .abstract = true,                                                   \
        .class_init = glue(glue(ppc_, _name), _cpu_family_class_init),      \
    };                                                                      \
                                                                            \
    static void glue(glue(ppc_, _name), _cpu_family_register_types)(void)   \
    {                                                                       \
        type_register_static(                                               \
            &glue(glue(ppc_, _name), _cpu_family_type_info));               \
    }                                                                       \
                                                                            \
    type_init(glue(glue(ppc_, _name), _cpu_family_register_types))          \
                                                                            \
    static void glue(glue(ppc_, _name), _cpu_family_class_init)

static void init_proc_401(CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_401(env);
    init_excp_4xx_real(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(12, 16, 20, 24);
    SET_WDT_PERIOD(16, 20, 24, 28);
}

POWERPC_FAMILY(401)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 401";
    pcc->init_proc = init_proc_401;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_WRTEE | PPC_DCR |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_40x_ICBT |
                       PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_4xx_COMMON | PPC_40x_EXCP;
    pcc->msr_mask = (1ull << MSR_KEY) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_REAL;
    pcc->excp_model = POWERPC_EXCP_40x;
    pcc->bus_model = PPC_FLAGS_INPUT_401;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_401x2(CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_401x2(env);
    gen_spr_compress(env);
    /* Memory management */
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_4xx_softmmu(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(12, 16, 20, 24);
    SET_WDT_PERIOD(16, 20, 24, 28);
}

POWERPC_FAMILY(401x2)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 401x2";
    pcc->init_proc = init_proc_401x2;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_DCR | PPC_WRTEE |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_40x_ICBT |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC |
                       PPC_4xx_COMMON | PPC_40x_EXCP;
    pcc->msr_mask = (1ull << 20) |
                    (1ull << MSR_KEY) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_4xx_Z;
    pcc->excp_model = POWERPC_EXCP_40x;
    pcc->bus_model = PPC_FLAGS_INPUT_401;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_401x3(CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_401(env);
    gen_spr_401x2(env);
    gen_spr_compress(env);
    init_excp_4xx_softmmu(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(12, 16, 20, 24);
    SET_WDT_PERIOD(16, 20, 24, 28);
}

POWERPC_FAMILY(401x3)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 401x3";
    pcc->init_proc = init_proc_401x3;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_DCR | PPC_WRTEE |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_40x_ICBT |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC |
                       PPC_4xx_COMMON | PPC_40x_EXCP;
    pcc->msr_mask = (1ull << 20) |
                    (1ull << MSR_KEY) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_4xx_Z;
    pcc->excp_model = POWERPC_EXCP_40x;
    pcc->bus_model = PPC_FLAGS_INPUT_401;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_IOP480(CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_401x2(env);
    gen_spr_compress(env);
    /* Memory management */
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_4xx_softmmu(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(8, 12, 16, 20);
    SET_WDT_PERIOD(16, 20, 24, 28);
}

POWERPC_FAMILY(IOP480)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "IOP480";
    pcc->init_proc = init_proc_IOP480;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_DCR | PPC_WRTEE |
                       PPC_CACHE | PPC_CACHE_ICBI |  PPC_40x_ICBT |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC |
                       PPC_4xx_COMMON | PPC_40x_EXCP;
    pcc->msr_mask = (1ull << 20) |
                    (1ull << MSR_KEY) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_4xx_Z;
    pcc->excp_model = POWERPC_EXCP_40x;
    pcc->bus_model = PPC_FLAGS_INPUT_401;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_403(CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_403(env);
    gen_spr_403_real(env);
    init_excp_4xx_real(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(8, 12, 16, 20);
    SET_WDT_PERIOD(16, 20, 24, 28);
}

POWERPC_FAMILY(403)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 403";
    pcc->init_proc = init_proc_403;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_DCR | PPC_WRTEE |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_40x_ICBT |
                       PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_4xx_COMMON | PPC_40x_EXCP;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_PE) |
                    (1ull << MSR_PX) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_REAL;
    pcc->excp_model = POWERPC_EXCP_40x;
    pcc->bus_model = PPC_FLAGS_INPUT_401;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_PX |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_403GCX(CPUPPCState *env)
{
    gen_spr_40x(env);
    gen_spr_401_403(env);
    gen_spr_403(env);
    gen_spr_403_real(env);
    gen_spr_403_mmu(env);
    gen_spr_40x_bus_control(env);
    /* Memory management */
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_4xx_softmmu(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(8, 12, 16, 20);
    SET_WDT_PERIOD(16, 20, 24, 28);
}

POWERPC_FAMILY(403GCX)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 403 GCX";
    pcc->init_proc = init_proc_403GCX;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_DCR | PPC_WRTEE |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_40x_ICBT |
                       PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC |
                       PPC_4xx_COMMON | PPC_40x_EXCP;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_PE) |
                    (1ull << MSR_PX) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_4xx_Z;
    pcc->excp_model = POWERPC_EXCP_40x;
    pcc->bus_model = PPC_FLAGS_INPUT_401;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_PX |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_405(CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_40x(env);
    gen_spr_405(env);
    gen_spr_40x_bus_control(env);
    /* Memory management */
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_4xx_softmmu(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(8, 12, 16, 20);
    SET_WDT_PERIOD(16, 20, 24, 28);
}

POWERPC_FAMILY(405)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 405";
    pcc->init_proc = init_proc_405;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_DCR | PPC_WRTEE |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_40x_ICBT |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_40x_TLB | PPC_MEM_TLBIA | PPC_MEM_TLBSYNC |
                       PPC_4xx_COMMON | PPC_405_MAC | PPC_40x_EXCP;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_SOFT_4xx;
    pcc->excp_model = POWERPC_EXCP_40x;
    pcc->bus_model = PPC_FLAGS_INPUT_405;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DWE |
                 POWERPC_FLAG_DE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_440EP(CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env, 0x000000000000FFFFULL);
    gen_spr_440(env);
    gen_spr_usprgh(env);
    gen_spr_440_misc(env);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_BOOKE_MCSR, "MCSR");
    gen_spr_not_implemented(env, SPR_BOOKE_MCSRR0, "MCSRR0");
    gen_spr_not_implemented(env, SPR_BOOKE_MCSRR1, "MCSRR1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_440_CCR1, "CCR1");
    /* Memory management */
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_BookE(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(12, 16, 20, 24);
    SET_WDT_PERIOD(20, 24, 28, 32);
}

POWERPC_FAMILY(440EP)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 440 EP";
    pcc->init_proc = init_proc_440EP;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_FLOAT | PPC_FLOAT_FRES | PPC_FLOAT_FSEL |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_DCR | PPC_WRTEE | PPC_RFMCI |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_MFTB |
                       PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |
                       PPC_440_SPEC;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DWE |
                 POWERPC_FLAG_DE | POWERPC_FLAG_BUS_CLK;
}

POWERPC_FAMILY(460EX)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 460 EX";
    pcc->init_proc = init_proc_440EP;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_FLOAT | PPC_FLOAT_FRES | PPC_FLOAT_FSEL |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_DCR | PPC_DCRX | PPC_WRTEE | PPC_RFMCI |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_MFTB |
                       PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |
                       PPC_440_SPEC;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DWE |
                 POWERPC_FLAG_DE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_440GP(CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env, 0x000000000000FFFFULL);
    gen_spr_440(env);
    gen_spr_usprgh(env);
    gen_spr_440_misc(env);
    /* Memory management */
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_BookE(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* XXX: TODO: allocate internal IRQ controller */

    SET_FIT_PERIOD(12, 16, 20, 24);
    SET_WDT_PERIOD(20, 24, 28, 32);
}

POWERPC_FAMILY(440GP)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 440 GP";
    pcc->init_proc = init_proc_440GP;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_DCR | PPC_DCRX | PPC_WRTEE | PPC_MFAPIDI |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_TLBIVA | PPC_MFTB |
                       PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |
                       PPC_440_SPEC;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DWE |
                 POWERPC_FLAG_DE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_440x4(CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env, 0x000000000000FFFFULL);
    gen_spr_440(env);
    gen_spr_usprgh(env);
    gen_spr_440_misc(env);
    /* Memory management */
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_BookE(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* XXX: TODO: allocate internal IRQ controller */

    SET_FIT_PERIOD(12, 16, 20, 24);
    SET_WDT_PERIOD(20, 24, 28, 32);
}

POWERPC_FAMILY(440x4)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 440x4";
    pcc->init_proc = init_proc_440x4;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_DCR | PPC_WRTEE |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_MFTB |
                       PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |
                       PPC_440_SPEC;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DWE |
                 POWERPC_FLAG_DE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_440x5(CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env, 0x000000000000FFFFULL);
    gen_spr_440(env);
    gen_spr_usprgh(env);
    gen_spr_440_misc(env);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_BOOKE_MCSR, "MCSR");
    gen_spr_not_implemented(env, SPR_BOOKE_MCSRR0, "MCSRR0");
    gen_spr_not_implemented(env, SPR_BOOKE_MCSRR1, "MCSRR1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_440_CCR1, "CCR1");
    /* Memory management */
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_BookE(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    ppc40x_irq_init(env_archcpu(env));

    SET_FIT_PERIOD(12, 16, 20, 24);
    SET_WDT_PERIOD(20, 24, 28, 32);
}

POWERPC_FAMILY(440x5)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 440x5";
    pcc->init_proc = init_proc_440x5;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_DCR | PPC_WRTEE | PPC_RFMCI |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_MFTB |
                       PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |
                       PPC_440_SPEC;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DWE |
                 POWERPC_FLAG_DE | POWERPC_FLAG_BUS_CLK;
}

POWERPC_FAMILY(440x5wDFPU)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 440x5 with double precision FPU";
    pcc->init_proc = init_proc_440x5;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_FLOAT | PPC_FLOAT_FSQRT |
                       PPC_FLOAT_STFIWX |
                       PPC_DCR | PPC_WRTEE | PPC_RFMCI |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_MFTB |
                       PPC_BOOKE | PPC_4xx_COMMON | PPC_405_MAC |
                       PPC_440_SPEC;
    pcc->insns_flags2 = PPC2_FP_CVT_S64;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_403;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DWE |
                 POWERPC_FLAG_DE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_MPC5xx(CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_5xx_8xx(env);
    gen_spr_5xx(env);
    init_excp_MPC5xx(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* XXX: TODO: allocate internal IRQ controller */
}

POWERPC_FAMILY(MPC5xx)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "Freescale 5xx cores (aka RCPU)";
    pcc->init_proc = init_proc_MPC5xx;
    pcc->check_pow = check_pow_none;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING |
                       PPC_MEM_EIEIO | PPC_MEM_SYNC |
                       PPC_CACHE_ICBI | PPC_FLOAT | PPC_FLOAT_STFIWX |
                       PPC_MFTB;
    pcc->msr_mask = (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_REAL;
    pcc->excp_model = POWERPC_EXCP_603;
    pcc->bus_model = PPC_FLAGS_INPUT_RCPU;
    pcc->bfd_mach = bfd_mach_ppc_505;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_MPC8xx(CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_5xx_8xx(env);
    gen_spr_8xx(env);
    init_excp_MPC8xx(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* XXX: TODO: allocate internal IRQ controller */
}

POWERPC_FAMILY(MPC8xx)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "Freescale 8xx cores (aka PowerQUICC)";
    pcc->init_proc = init_proc_MPC8xx;
    pcc->check_pow = check_pow_none;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING  |
                       PPC_MEM_EIEIO | PPC_MEM_SYNC |
                       PPC_CACHE_ICBI | PPC_MFTB;
    pcc->msr_mask = (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_MPC8xx;
    pcc->excp_model = POWERPC_EXCP_603;
    pcc->bus_model = PPC_FLAGS_INPUT_RCPU;
    pcc->bfd_mach = bfd_mach_ppc_860;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_BUS_CLK;
}

/* Freescale 82xx cores (aka PowerQUICC-II)                                  */

static void init_proc_G2(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_G2_755(env);
    gen_spr_G2(env);
    /* Time base */
    gen_tbl(env);
    /* External access control */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_EAR, "EAR");
    /* Hardware implementation register */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID2, "HID2");
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_G2(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(G2)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC G2";
    pcc->init_proc = init_proc_G2;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC | PPC_6xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_TGPR) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_AL) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_RI);
    pcc->mmu_model = POWERPC_MMU_SOFT_6xx;
    pcc->excp_model = POWERPC_EXCP_G2;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_ec603e;
    pcc->flags = POWERPC_FLAG_TGPR | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_G2LE(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_G2_755(env);
    gen_spr_G2(env);
    /* Time base */
    gen_tbl(env);
    /* External access control */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_EAR, "EAR");
    /* Hardware implementation register */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID2, "HID2");

    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_G2(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(G2LE)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC G2LE";
    pcc->init_proc = init_proc_G2LE;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC | PPC_6xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_TGPR) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_AL) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_6xx;
    pcc->excp_model = POWERPC_EXCP_G2;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_ec603e;
    pcc->flags = POWERPC_FLAG_TGPR | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_e200(CPUPPCState *env)
{
    /* Time base */
    gen_tbl(env);
    gen_spr_BookE(env, 0x000000070000FFFFULL);
    /* XXX : not implemented */
    gen_spr_spefscr(env);
    /* Memory management */
    gen_spr_BookE206(env, 0x0000005D, NULL, 0);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_ALTCTXCR, "ALTCTXCR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_BUCSR, "BUCSR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_CTXCR, "CTXCR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_DBCNT, "DBCNT");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_DBCR3, "DBCR3");
    /* XXX : not implemented */
    gen_spr_l1fgc(env, SPR_Exxx_L1CFG0, 0x00000000);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_L1CSR0, "L1CSR0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_L1FINV0, "L1FINV0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_BOOKE_TLB0CFG, "TLB0CFG");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_BOOKE_TLB1CFG, "TLB1CFG");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_BOOKE_IAC3, "IAC3");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_BOOKE_IAC4, "IAC4");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_MMUCSR0, "MMUCSR0"); /* TOFIX */
    gen_spr_not_implemented(env, SPR_BOOKE_DSRR0, "DSRR0");
    gen_spr_not_implemented(env, SPR_BOOKE_DSRR1, "DSRR1");
#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 64;
    env->nb_ways = 1;
    env->id_tlbs = 0;
    env->tlb_type = TLB_EMB;
#endif
    init_excp_e200(env, 0xFFFF0000UL);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* XXX: TODO: allocate internal IRQ controller */
}

POWERPC_FAMILY(e200)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "e200 core";
    pcc->init_proc = init_proc_e200;
    pcc->check_pow = check_pow_hid0;
    /*
     * XXX: unimplemented instructions:
     * dcblc
     * dcbtlst
     * dcbtstls
     * icblc
     * icbtls
     * tlbivax
     * all SPE multiply-accumulate instructions
     */
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL |
                       PPC_SPE | PPC_SPE_SINGLE |
                       PPC_WRTEE | PPC_RFDI |
                       PPC_CACHE | PPC_CACHE_LOCK | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_TLBIVAX |
                       PPC_BOOKE;
    pcc->msr_mask = (1ull << MSR_UCLE) |
                    (1ull << MSR_SPE) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE206;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_860;
    pcc->flags = POWERPC_FLAG_SPE | POWERPC_FLAG_CE |
                 POWERPC_FLAG_UBLE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_e300(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_603(env);
    /* Time base */
    gen_tbl(env);
    /* hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID2, "HID2");
    /* Breakpoints */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_DABR, "DABR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_DABR2, "DABR2");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_IABR2, "IABR2");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_IBCR, "IBCR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_DBCR, "DBCR");
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_603(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(e300)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "e300 core";
    pcc->init_proc = init_proc_e300;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC | PPC_6xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_TGPR) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_AL) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_6xx;
    pcc->excp_model = POWERPC_EXCP_603;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_603;
    pcc->flags = POWERPC_FLAG_TGPR | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_BUS_CLK;
}

enum fsl_e500_version {
    fsl_e500v1,
    fsl_e500v2,
    fsl_e500mc,
    fsl_e5500,
    fsl_e6500,
};

static void init_proc_e500(CPUPPCState *env, int version)
{
    uint32_t tlbncfg[2];
    uint64_t ivor_mask;
    uint64_t ivpr_mask = 0xFFFF0000ULL;
    uint32_t l1cfg0 = 0x3800  /* 8 ways */
                    | 0x0020; /* 32 kb */
    uint32_t l1cfg1 = 0x3800  /* 8 ways */
                    | 0x0020; /* 32 kb */
    uint32_t mmucfg = 0;
#if !defined(CONFIG_USER_ONLY)
    int i;
#endif

    /* Time base */
    gen_tbl(env);
    /*
     * XXX The e500 doesn't implement IVOR7 and IVOR9, but doesn't
     *     complain when accessing them.
     * gen_spr_BookE(env, 0x0000000F0000FD7FULL);
     */
    switch (version) {
    case fsl_e500v1:
    case fsl_e500v2:
    default:
        ivor_mask = 0x0000000F0000FFFFULL;
        break;
    case fsl_e500mc:
    case fsl_e5500:
        ivor_mask = 0x000003FE0000FFFFULL;
        break;
    case fsl_e6500:
        ivor_mask = 0x000003FF0000FFFFULL;
        break;
    }
    gen_spr_BookE(env, ivor_mask);
    gen_spr_usprg3(env);
    /* Processor identification */
    gen_spr_pir(env);
    /* XXX : not implemented */
    gen_spr_spefscr(env);
#if !defined(CONFIG_USER_ONLY)
    /* Memory management */
    env->nb_pids = 3;
    env->nb_ways = 2;
    env->id_tlbs = 0;
    switch (version) {
    case fsl_e500v1:
        tlbncfg[0] = gen_tlbncfg(2, 1, 1, 0, 256);
        tlbncfg[1] = gen_tlbncfg(16, 1, 9, TLBnCFG_AVAIL | TLBnCFG_IPROT, 16);
        break;
    case fsl_e500v2:
        tlbncfg[0] = gen_tlbncfg(4, 1, 1, 0, 512);
        tlbncfg[1] = gen_tlbncfg(16, 1, 12, TLBnCFG_AVAIL | TLBnCFG_IPROT, 16);
        break;
    case fsl_e500mc:
    case fsl_e5500:
        tlbncfg[0] = gen_tlbncfg(4, 1, 1, 0, 512);
        tlbncfg[1] = gen_tlbncfg(64, 1, 12, TLBnCFG_AVAIL | TLBnCFG_IPROT, 64);
        break;
    case fsl_e6500:
        mmucfg = 0x6510B45;
        env->nb_pids = 1;
        tlbncfg[0] = 0x08052400;
        tlbncfg[1] = 0x40028040;
        break;
    default:
        cpu_abort(env_cpu(env), "Unknown CPU: " TARGET_FMT_lx "\n",
                  env->spr[SPR_PVR]);
    }
#endif
    /* Cache sizes */
    switch (version) {
    case fsl_e500v1:
    case fsl_e500v2:
        env->dcache_line_size = 32;
        env->icache_line_size = 32;
        break;
    case fsl_e500mc:
    case fsl_e5500:
        env->dcache_line_size = 64;
        env->icache_line_size = 64;
        l1cfg0 |= 0x1000000; /* 64 byte cache block size */
        l1cfg1 |= 0x1000000; /* 64 byte cache block size */
        break;
    case fsl_e6500:
        env->dcache_line_size = 32;
        env->icache_line_size = 32;
        l1cfg0 |= 0x0F83820;
        l1cfg1 |= 0x0B83820;
        break;
    default:
        cpu_abort(env_cpu(env), "Unknown CPU: " TARGET_FMT_lx "\n",
                  env->spr[SPR_PVR]);
    }
    gen_spr_BookE206(env, 0x000000DF, tlbncfg, mmucfg);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_BBEAR, "BBEAR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_BBTAR, "BBTAR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_MCAR, "MCAR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_BOOKE_MCSR, "MCSR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_NPIDR, "NPIDR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_Exxx_BUCSR, "BUCSR");
    /* XXX : not implemented */
    gen_spr_l1fgc(env, SPR_Exxx_L1CFG0, l1cfg0);
    gen_spr_l1fgc(env, SPR_Exxx_L1CFG1, l1cfg1);
    gen_spr_l1csr0(env);
    gen_spr_l1csr1(env);
    if (version != fsl_e500v1 && version != fsl_e500v2) {
        gen_spr_l2csr0(env);
    }
    gen_spr_not_implemented(env, SPR_BOOKE_MCSRR0, "MCSRR0");
    gen_spr_not_implemented(env, SPR_BOOKE_MCSRR1, "MCSRR1");
    gen_spr_mmucsr0(env);
    gen_spr_not_implemented_no_write(env, SPR_BOOKE_EPR, "EPR");
    /* XXX better abstract into Emb.xxx features */
    if ((version == fsl_e5500) || (version == fsl_e6500)) {
        gen_spr_not_implemented(env, SPR_BOOKE_EPCR, "EPCR");
        gen_spr_mas73(env);
        ivpr_mask = (target_ulong)~0xFFFFULL;
    }

    if (version == fsl_e6500) {
        /* Thread identification */
        gen_spr_not_implemented_no_write(env, SPR_TIR, "TIR");
        gen_spr_not_implemented_no_write(env, SPR_BOOKE_TLB0PS, "TLB0PS");
        gen_spr_not_implemented_no_write(env, SPR_BOOKE_TLB1PS, "TLB1PS");
    }

#if !defined(CONFIG_USER_ONLY)
    env->nb_tlb = 0;
    env->tlb_type = TLB_MAS;
    for (i = 0; i < BOOKE206_MAX_TLBN; i++) {
        env->nb_tlb += booke206_tlb_size(env, i);
    }
#endif

    init_excp_e200(env, ivpr_mask);
    /* Allocate hardware IRQ controller */
    ppce500_irq_init(env_archcpu(env));
}

static void init_proc_e500v1(CPUPPCState *env)
{
    init_proc_e500(env, fsl_e500v1);
}

POWERPC_FAMILY(e500v1)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "e500v1 core";
    pcc->init_proc = init_proc_e500v1;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL |
                       PPC_SPE | PPC_SPE_SINGLE |
                       PPC_WRTEE | PPC_RFDI |
                       PPC_CACHE | PPC_CACHE_LOCK | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_TLBIVAX | PPC_MEM_SYNC;
    pcc->insns_flags2 = PPC2_BOOKE206;
    pcc->msr_mask = (1ull << MSR_UCLE) |
                    (1ull << MSR_SPE) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE206;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_860;
    pcc->flags = POWERPC_FLAG_SPE | POWERPC_FLAG_CE |
                 POWERPC_FLAG_UBLE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_e500v2(CPUPPCState *env)
{
    init_proc_e500(env, fsl_e500v2);
}

POWERPC_FAMILY(e500v2)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "e500v2 core";
    pcc->init_proc = init_proc_e500v2;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL |
                       PPC_SPE | PPC_SPE_SINGLE | PPC_SPE_DOUBLE |
                       PPC_WRTEE | PPC_RFDI |
                       PPC_CACHE | PPC_CACHE_LOCK | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_MEM_TLBSYNC | PPC_TLBIVAX | PPC_MEM_SYNC;
    pcc->insns_flags2 = PPC2_BOOKE206;
    pcc->msr_mask = (1ull << MSR_UCLE) |
                    (1ull << MSR_SPE) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DWE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_BOOKE206;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_860;
    pcc->flags = POWERPC_FLAG_SPE | POWERPC_FLAG_CE |
                 POWERPC_FLAG_UBLE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_e500mc(CPUPPCState *env)
{
    init_proc_e500(env, fsl_e500mc);
}

POWERPC_FAMILY(e500mc)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "e500mc core";
    pcc->init_proc = init_proc_e500mc;
    pcc->check_pow = check_pow_none;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL | PPC_MFTB |
                       PPC_WRTEE | PPC_RFDI | PPC_RFMCI |
                       PPC_CACHE | PPC_CACHE_LOCK | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_FLOAT | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_FSEL |
                       PPC_FLOAT_STFIWX | PPC_WAIT |
                       PPC_MEM_TLBSYNC | PPC_TLBIVAX | PPC_MEM_SYNC;
    pcc->insns_flags2 = PPC2_BOOKE206 | PPC2_PRCNTL;
    pcc->msr_mask = (1ull << MSR_GS) |
                    (1ull << MSR_UCLE) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PX) |
                    (1ull << MSR_RI);
    pcc->mmu_model = POWERPC_MMU_BOOKE206;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    /* FIXME: figure out the correct flag for e500mc */
    pcc->bfd_mach = bfd_mach_ppc_e500;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

#ifdef TARGET_PPC64
static void init_proc_e5500(CPUPPCState *env)
{
    init_proc_e500(env, fsl_e5500);
}

POWERPC_FAMILY(e5500)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "e5500 core";
    pcc->init_proc = init_proc_e5500;
    pcc->check_pow = check_pow_none;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL | PPC_MFTB |
                       PPC_WRTEE | PPC_RFDI | PPC_RFMCI |
                       PPC_CACHE | PPC_CACHE_LOCK | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_FLOAT | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_FSEL |
                       PPC_FLOAT_STFIWX | PPC_WAIT |
                       PPC_MEM_TLBSYNC | PPC_TLBIVAX | PPC_MEM_SYNC |
                       PPC_64B | PPC_POPCNTB | PPC_POPCNTWD;
    pcc->insns_flags2 = PPC2_BOOKE206 | PPC2_PRCNTL | PPC2_PERM_ISA206 |
                        PPC2_FP_CVT_S64;
    pcc->msr_mask = (1ull << MSR_CM) |
                    (1ull << MSR_GS) |
                    (1ull << MSR_UCLE) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PX) |
                    (1ull << MSR_RI);
    pcc->mmu_model = POWERPC_MMU_BOOKE206;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    /* FIXME: figure out the correct flag for e5500 */
    pcc->bfd_mach = bfd_mach_ppc_e500;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_e6500(CPUPPCState *env)
{
    init_proc_e500(env, fsl_e6500);
}

POWERPC_FAMILY(e6500)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "e6500 core";
    pcc->init_proc = init_proc_e6500;
    pcc->check_pow = check_pow_none;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL | PPC_MFTB |
                       PPC_WRTEE | PPC_RFDI | PPC_RFMCI |
                       PPC_CACHE | PPC_CACHE_LOCK | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBZ | PPC_CACHE_DCBA |
                       PPC_FLOAT | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_FSEL |
                       PPC_FLOAT_STFIWX | PPC_WAIT |
                       PPC_MEM_TLBSYNC | PPC_TLBIVAX | PPC_MEM_SYNC |
                       PPC_64B | PPC_POPCNTB | PPC_POPCNTWD | PPC_ALTIVEC;
    pcc->insns_flags2 = PPC2_BOOKE206 | PPC2_PRCNTL | PPC2_PERM_ISA206 |
                        PPC2_FP_CVT_S64 | PPC2_ATOMIC_ISA206;
    pcc->msr_mask = (1ull << MSR_CM) |
                    (1ull << MSR_GS) |
                    (1ull << MSR_UCLE) |
                    (1ull << MSR_CE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IS) |
                    (1ull << MSR_DS) |
                    (1ull << MSR_PX) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_VR);
    pcc->mmu_model = POWERPC_MMU_BOOKE206;
    pcc->excp_model = POWERPC_EXCP_BOOKE;
    pcc->bus_model = PPC_FLAGS_INPUT_BookE;
    pcc->bfd_mach = bfd_mach_ppc_e500;
    pcc->flags = POWERPC_FLAG_CE | POWERPC_FLAG_DE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK | POWERPC_FLAG_VRE;
}

#endif

/* Non-embedded PowerPC                                                      */

#define POWERPC_MSRR_601     (0x0000000000001040ULL)

static void init_proc_601(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_601(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_hid0(env);
    /* XXX : not implemented */
        gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
        gen_spr_not_implemented(env, SPR_601_HID2, "HID2");
    /* XXX : not implemented */
        gen_spr_not_implemented(env, SPR_601_HID5, "HID5");
    /* Memory management */
    init_excp_601(env);
    /*
     * XXX: beware that dcache line size is 64
     *      but dcbz uses 32 bytes "sectors"
     * XXX: this breaks clcs instruction !
     */
    env->dcache_line_size = 32;
    env->icache_line_size = 64;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(601)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 601";
    pcc->init_proc = init_proc_601;
    pcc->check_pow = check_pow_none;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_POWER_BR |
                       PPC_FLOAT |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO | PPC_MEM_TLBIE |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_601;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_601;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_601;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_RTC_CLK;
}

#define POWERPC_MSRR_601v    (0x0000000000001040ULL)

static void init_proc_601v(CPUPPCState *env)
{
    init_proc_601(env);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_601_HID15, "HID15");
}

POWERPC_FAMILY(601v)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 601v";
    pcc->init_proc = init_proc_601v;
    pcc->check_pow = check_pow_none;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_POWER_BR |
                       PPC_FLOAT |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO | PPC_MEM_TLBIE |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR);
    pcc->mmu_model = POWERPC_MMU_601;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_601;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_RTC_CLK;
}

static void init_proc_602(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_602(env);
    /* Time base */
    gen_tbl(env);
    /* hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* Memory management */
    gen_low_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_602(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(602)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 602";
    pcc->init_proc = init_proc_602;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_6xx_TLB | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_602_SPEC;
    pcc->msr_mask = (1ull << MSR_VSX) |
                    (1ull << MSR_SA) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_TGPR) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    /* XXX: 602 MMU is quite specific. Should add a special case */
    pcc->mmu_model = POWERPC_MMU_SOFT_6xx;
    pcc->excp_model = POWERPC_EXCP_602;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_602;
    pcc->flags = POWERPC_FLAG_TGPR | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_603(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_603(env);
    /* Time base */
    gen_tbl(env);
    /* hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* Memory management */
    gen_low_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_603(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(603)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 603";
    pcc->init_proc = init_proc_603;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC | PPC_6xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_TGPR) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_6xx;
    pcc->excp_model = POWERPC_EXCP_603;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_603;
    pcc->flags = POWERPC_FLAG_TGPR | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_603E(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_603(env);
    /* Time base */
    gen_tbl(env);
    /* hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* Memory management */
    gen_low_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_603(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(603E)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 603e";
    pcc->init_proc = init_proc_603E;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC | PPC_6xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_TGPR) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_6xx;
    pcc->excp_model = POWERPC_EXCP_603E;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_ec603e;
    pcc->flags = POWERPC_FLAG_TGPR | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_604(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_604(env);
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* Memory management */
    gen_low_BATs(env);
    init_excp_604(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(604)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 604";
    pcc->init_proc = init_proc_604;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_604;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_604;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_604E(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_604(env);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_MMCR1, "MMCR1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC3, "PMC3");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC4, "PMC4");
    /* Time base */
    gen_tbl(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* Memory management */
    gen_low_BATs(env);
    init_excp_604(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(604E)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 604E";
    pcc->init_proc = init_proc_604E;
    pcc->check_pow = check_pow_nocheck;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_604;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_604;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_740(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* Memory management */
    gen_low_BATs(env);
    init_excp_7x0(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(740)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 740";
    pcc->init_proc = init_proc_740;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_7x0;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_750;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_750(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* XXX : not implemented */
    gen_spr_not_implemented_write_nop(env, SPR_L2CR, "L2CR");
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* Memory management */
    gen_low_BATs(env);
    /*
     * XXX: high BATs are also present but are known to be bugged on
     *      die version 1.x
     */
    init_excp_7x0(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(750)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 750";
    pcc->init_proc = init_proc_750;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_7x0;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_750;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_750cl(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* XXX : not implemented */
    gen_spr_not_implemented_write_nop(env, SPR_L2CR, "L2CR");
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    /* Those registers are fake on 750CL */
    gen_spr_not_implemented(env, SPR_THRM1, "THRM1");
    gen_spr_not_implemented(env, SPR_THRM2, "THRM2");
    gen_spr_not_implemented(env, SPR_THRM3, "THRM3");
    /* XXX: not implemented */
    gen_spr_not_implemented(env, SPR_750_TDCL, "TDCL");
    gen_spr_not_implemented(env, SPR_750_TDCH, "TDCH");
    /* DMA */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_WPAR, "WPAR");
    gen_spr_not_implemented(env, SPR_750_DMAL, "DMAL");
    gen_spr_not_implemented(env, SPR_750_DMAU, "DMAU");
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750CL_HID2, "HID2");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750CL_HID4, "HID4");
    /* Quantization registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_GQR0, "GQR0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_GQR1, "GQR1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_GQR2, "GQR2");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_GQR3, "GQR3");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_GQR4, "GQR4");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_GQR5, "GQR5");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_GQR6, "GQR6");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_GQR7, "GQR7");
    /* Memory management */
    gen_low_BATs(env);
    /* PowerPC 750cl has 8 DBATs and 8 IBATs */
    gen_high_BATs(env);
    init_excp_750cl(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(750cl)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 750 CL";
    pcc->init_proc = init_proc_750cl;
    pcc->check_pow = check_pow_hid0;
    /*
     * XXX: not implemented:
     * cache lock instructions:
     * dcbz_l
     * floating point paired instructions
     * psq_lux
     * psq_lx
     * psq_stux
     * psq_stx
     * ps_abs
     * ps_add
     * ps_cmpo0
     * ps_cmpo1
     * ps_cmpu0
     * ps_cmpu1
     * ps_div
     * ps_madd
     * ps_madds0
     * ps_madds1
     * ps_merge00
     * ps_merge01
     * ps_merge10
     * ps_merge11
     * ps_mr
     * ps_msub
     * ps_mul
     * ps_muls0
     * ps_muls1
     * ps_nabs
     * ps_neg
     * ps_nmadd
     * ps_nmsub
     * ps_res
     * ps_rsqrte
     * ps_sel
     * ps_sub
     * ps_sum0
     * ps_sum1
     */
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_7x0;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_750;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_750cx(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* XXX : not implemented */
    gen_spr_not_implemented_write_nop(env, SPR_L2CR, "L2CR");
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* This register is not implemented but is present for compatibility */
    gen_spr_not_implemented(env, SPR_SDA, "SDA");
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* Memory management */
    gen_low_BATs(env);
    /* PowerPC 750cx has 8 DBATs and 8 IBATs */
    gen_high_BATs(env);
    init_excp_750cx(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(750cx)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 750CX";
    pcc->init_proc = init_proc_750cx;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_7x0;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_750;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_750fx(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* XXX : not implemented */
    gen_spr_not_implemented_write_nop(env, SPR_L2CR, "L2CR");
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_THRM4, "THRM4");
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750FX_HID2, "HID2");
    /* Memory management */
    gen_low_BATs(env);
    /* PowerPC 750fx & 750gx has 8 DBATs and 8 IBATs */
    gen_high_BATs(env);
    init_excp_7x0(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(750fx)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 750FX";
    pcc->init_proc = init_proc_750fx;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_7x0;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_750;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_750gx(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* XXX : not implemented (XXX: different from 750fx) */
    gen_spr_not_implemented_write_nop(env, SPR_L2CR, "L2CR");
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_750_THRM4, "THRM4");
    /* Hardware implementation registers */
    /* XXX : not implemented (XXX: different from 750fx) */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented (XXX: different from 750fx) */
    gen_spr_not_implemented(env, SPR_750FX_HID2, "HID2");
    /* Memory management */
    gen_low_BATs(env);
    /* PowerPC 750fx & 750gx has 8 DBATs and 8 IBATs */
    gen_high_BATs(env);
    init_excp_7x0(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(750gx)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 750GX";
    pcc->init_proc = init_proc_750gx;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_7x0;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_750;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_745(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    gen_spr_G2_755(env);
    /* Time base */
    gen_tbl(env);
    /* Thermal management */
    gen_spr_thrm(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID2, "HID2");
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_7x5(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(745)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 745";
    pcc->init_proc = init_proc_745;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC | PPC_6xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_6xx;
    pcc->excp_model = POWERPC_EXCP_7x5;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_750;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_755(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    gen_spr_G2_755(env);
    /* Time base */
    gen_tbl(env);
    /* L2 cache control */
    /* XXX : not implemented */
    gen_spr_not_implemented_write_nop(env, SPR_L2CR, "L2CR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L2PMCR, "L2PMCR");
    /* Thermal management */
    gen_spr_thrm(env);
    /* Hardware implementation registers */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID0, "HID0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID1, "HID1");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_HID2, "HID2");
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_6xx_7xx_soft_tlb(env, 64, 2);
    init_excp_7x5(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(755)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 755";
    pcc->init_proc = init_proc_755;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FRSQRTE | PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC | PPC_6xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN;
    pcc->msr_mask = (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_6xx;
    pcc->excp_model = POWERPC_EXCP_7x5;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_750;
    pcc->flags = POWERPC_FLAG_SE | POWERPC_FLAG_BE |
                 POWERPC_FLAG_PMM | POWERPC_FLAG_BUS_CLK;
}

static void init_proc_7400(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    vscr_init(env, 0x00010000);
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_UBAMR, "UBAMR");
    /* XXX: this seems not implemented on all revisions. */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_MSSCR1, "MSSCR1");
    /* Thermal management */
    gen_spr_thrm(env);
    /* Memory management */
    gen_low_BATs(env);
    init_excp_7400(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(7400)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 7400 (aka G4)";
    pcc->init_proc = init_proc_7400;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBA | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_MEM_TLBIA |
                       PPC_SEGMENT | PPC_EXTERN |
                       PPC_ALTIVEC;
    pcc->msr_mask = (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_74xx;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_7400;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_7410(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    vscr_init(env, 0x00010000);
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_UBAMR, "UBAMR");
    /* Thermal management */
    gen_spr_thrm(env);
    /* L2PMCR */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L2PMCR, "L2PMCR");
    /* LDSTDB */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_LDSTDB, "LDSTDB");
    /* Memory management */
    gen_low_BATs(env);
    init_excp_7400(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(7410)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 7410 (aka G4)";
    pcc->init_proc = init_proc_7410;
    pcc->check_pow = check_pow_hid0;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBA | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_MEM_TLBIA |
                       PPC_SEGMENT | PPC_EXTERN |
                       PPC_ALTIVEC;
    pcc->msr_mask = (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_74xx;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_7400;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_7440(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    vscr_init(env, 0x00010000);
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_UBAMR, "UBAMR");
    /* LDSTCR */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_LDSTCR, "LDSTCR");
    /* ICTRL */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_ICTRL, "ICTRL");
    /* MSSSR0 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_MSSSR0, "MSSSR0");
    /* PMC */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC5, "PMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC5, "UPMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC6, "PMC6");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC6, "UPMC6");
    /* Memory management */
    gen_low_BATs(env);
    gen_74xx_soft_tlb(env, 128, 2);
    init_excp_7450(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(7440)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 7440 (aka G4)";
    pcc->init_proc = init_proc_7440;
    pcc->check_pow = check_pow_hid0_74xx;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBA | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_MEM_TLBIA | PPC_74xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN |
                       PPC_ALTIVEC;
    pcc->msr_mask = (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_74xx;
    pcc->excp_model = POWERPC_EXCP_74xx;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_7400;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_7450(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    vscr_init(env, 0x00010000);
    /* Level 3 cache control */
    gen_l3_ctrl(env);
    /* L3ITCR1 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L3ITCR1, "L3ITCR1");
    /* L3ITCR2 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L3ITCR2, "L3ITCR2");
    /* L3ITCR3 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L3ITCR3, "L3ITCR3");
    /* L3OHCR */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L3OHCR, "L3OHCR");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_UBAMR, "UBAMR");
    /* LDSTCR */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_LDSTCR, "LDSTCR");
    /* ICTRL */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_ICTRL, "ICTRL");
    /* MSSSR0 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_MSSSR0, "MSSSR0");
    /* PMC */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC5, "PMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC5, "UPMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC6, "PMC6");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC6, "UPMC6");
    /* Memory management */
    gen_low_BATs(env);
    gen_74xx_soft_tlb(env, 128, 2);
    init_excp_7450(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(7450)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 7450 (aka G4)";
    pcc->init_proc = init_proc_7450;
    pcc->check_pow = check_pow_hid0_74xx;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBA | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_MEM_TLBIA | PPC_74xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN |
                       PPC_ALTIVEC;
    pcc->msr_mask = (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_74xx;
    pcc->excp_model = POWERPC_EXCP_74xx;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_7400;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_7445(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    vscr_init(env, 0x00010000);
    /* LDSTCR */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_LDSTCR, "LDSTCR");
    /* ICTRL */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_ICTRL, "ICTRL");
    /* MSSSR0 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_MSSSR0, "MSSSR0");
    /* PMC */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC5, "PMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC5, "UPMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC6, "PMC6");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC6, "UPMC6");
    /* SPRGs */
    gen_spr_not_implemented(env, SPR_SPRG4, "SPRG4");
    gen_spr_not_implemented_ureg(env, SPR_USPRG4, "USPRG4");
    gen_spr_not_implemented(env, SPR_SPRG5, "SPRG5");
    gen_spr_not_implemented_ureg(env, SPR_USPRG5, "USPRG5");
    gen_spr_not_implemented(env, SPR_SPRG6, "SPRG6");
    gen_spr_not_implemented_ureg(env, SPR_USPRG6, "USPRG6");
    gen_spr_not_implemented(env, SPR_SPRG7, "SPRG7");
    gen_spr_not_implemented_ureg(env, SPR_USPRG7, "USPRG7");
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_74xx_soft_tlb(env, 128, 2);
    init_excp_7450(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(7445)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 7445 (aka G4)";
    pcc->init_proc = init_proc_7445;
    pcc->check_pow = check_pow_hid0_74xx;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBA | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_MEM_TLBIA | PPC_74xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN |
                       PPC_ALTIVEC;
    pcc->msr_mask = (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_74xx;
    pcc->excp_model = POWERPC_EXCP_74xx;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_7400;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_7455(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    vscr_init(env, 0x00010000);
    /* Level 3 cache control */
    gen_l3_ctrl(env);
    /* LDSTCR */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_LDSTCR, "LDSTCR");
    /* ICTRL */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_ICTRL, "ICTRL");
    /* MSSSR0 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_MSSSR0, "MSSSR0");
    /* PMC */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC5, "PMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC5, "UPMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC6, "PMC6");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC6, "UPMC6");
    /* SPRGs */
    gen_spr_not_implemented(env, SPR_SPRG4, "SPRG4");
    gen_spr_not_implemented_ureg(env, SPR_USPRG4, "USPRG4");
    gen_spr_not_implemented(env, SPR_SPRG5, "SPRG5");
    gen_spr_not_implemented_ureg(env, SPR_USPRG5, "USPRG5");
    gen_spr_not_implemented(env, SPR_SPRG6, "SPRG6");
    gen_spr_not_implemented_ureg(env, SPR_USPRG6, "USPRG6");
    gen_spr_not_implemented(env, SPR_SPRG7, "SPRG7");
    gen_spr_not_implemented_ureg(env, SPR_USPRG7, "USPRG7");
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_74xx_soft_tlb(env, 128, 2);
    init_excp_7450(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(7455)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 7455 (aka G4)";
    pcc->init_proc = init_proc_7455;
    pcc->check_pow = check_pow_hid0_74xx;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBA | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_MEM_TLBIA | PPC_74xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN |
                       PPC_ALTIVEC;
    pcc->msr_mask = (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_74xx;
    pcc->excp_model = POWERPC_EXCP_74xx;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_7400;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_7457(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    vscr_init(env, 0x00010000);
    /* Level 3 cache control */
    gen_l3_ctrl(env);
    /* L3ITCR1 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L3ITCR1, "L3ITCR1");
    /* L3ITCR2 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L3ITCR2, "L3ITCR2");
    /* L3ITCR3 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L3ITCR3, "L3ITCR3");
    /* L3OHCR */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_L3OHCR, "L3OHCR");
    /* LDSTCR */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_LDSTCR, "LDSTCR");
    /* ICTRL */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_ICTRL, "ICTRL");
    /* MSSSR0 */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_MSSSR0, "MSSSR0");
    /* PMC */
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC5, "PMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC5, "UPMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC6, "PMC6");
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_7XX_UPMC6, "UPMC6");
    /* SPRGs */
    gen_spr_not_implemented(env, SPR_SPRG4, "SPRG4");
    gen_spr_not_implemented_ureg(env, SPR_USPRG4, "USPRG4");
    gen_spr_not_implemented(env, SPR_SPRG5, "SPRG5");
    gen_spr_not_implemented_ureg(env, SPR_USPRG5, "USPRG5");
    gen_spr_not_implemented(env, SPR_SPRG6, "SPRG6");
    gen_spr_not_implemented_ureg(env, SPR_USPRG6, "USPRG6");
    gen_spr_not_implemented(env, SPR_SPRG7, "SPRG7");
    gen_spr_not_implemented_ureg(env, SPR_USPRG7, "USPRG7");
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_74xx_soft_tlb(env, 128, 2);
    init_excp_7450(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(7457)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 7457 (aka G4)";
    pcc->init_proc = init_proc_7457;
    pcc->check_pow = check_pow_hid0_74xx;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBA | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_MEM_TLBIA | PPC_74xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN |
                       PPC_ALTIVEC;
    pcc->msr_mask = (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_SOFT_74xx;
    pcc->excp_model = POWERPC_EXCP_74xx;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_7400;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
}

static void init_proc_e600(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_spr_sdr1(env);
    gen_spr_7xx(env);
    /* Time base */
    gen_tbl(env);
    /* 74xx specific SPR */
    gen_spr_74xx(env);
    vscr_init(env, 0x00010000);
    /* XXX : not implemented */
    gen_spr_not_implemented_ureg(env, SPR_LDSTCR, "LDSTCR");
    gen_spr_not_implemented_ureg(env, SPR_UBAMR, "UBAMR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_LDSTCR, "LDSTCR");
    gen_spr_not_implemented(env, SPR_LDSTCR, "LDSTCR");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_ICTRL, "ICTRL");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_MSSSR0, "MSSSR0");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC5, "PMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_UPMC5, "UPMC5");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_PMC6, "PMC6");
    /* XXX : not implemented */
    gen_spr_not_implemented(env, SPR_7XX_UPMC6, "UPMC6");
    /* SPRGs */
    gen_spr_not_implemented(env, SPR_SPRG4, "SPRG4");
    gen_spr_not_implemented_ureg(env, SPR_USPRG4, "USPRG4");
    gen_spr_not_implemented(env, SPR_SPRG5, "SPRG5");
    gen_spr_not_implemented_ureg(env, SPR_USPRG5, "USPRG5");
    gen_spr_not_implemented(env, SPR_SPRG6, "SPRG6");
    gen_spr_not_implemented_ureg(env, SPR_USPRG6, "USPRG6");
    gen_spr_not_implemented(env, SPR_SPRG7, "SPRG7");
    gen_spr_not_implemented_ureg(env, SPR_USPRG7, "USPRG7");
    /* Memory management */
    gen_low_BATs(env);
    gen_high_BATs(env);
    gen_74xx_soft_tlb(env, 128, 2);
    init_excp_7450(env);
    env->dcache_line_size = 32;
    env->icache_line_size = 32;
    /* Allocate hardware IRQ controller */
    ppc6xx_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(e600)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC e600";
    pcc->init_proc = init_proc_e600;
    pcc->check_pow = check_pow_hid0_74xx;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI |
                       PPC_CACHE_DCBA | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_MEM_TLBIA | PPC_74xx_TLB |
                       PPC_SEGMENT | PPC_EXTERN |
                       PPC_ALTIVEC;
    pcc->insns_flags2 = PPC_NONE;
    pcc->msr_mask = (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_ILE) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_EP) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->mmu_model = POWERPC_MMU_32B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash32_handle_mmu_fault;
#endif
    pcc->excp_model = POWERPC_EXCP_74xx;
    pcc->bus_model = PPC_FLAGS_INPUT_6xx;
    pcc->bfd_mach = bfd_mach_ppc_7400;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
}
#if defined(TARGET_PPC64)

static int check_pow_970(CPUPPCState *env)
{
    if (env->spr[SPR_HID0] & (HID0_DEEPNAP | HID0_DOZE | HID0_NAP)) {
        return 1;
    }

    return 0;
}

static void init_proc_book3s_common(CPUPPCState *env)
{
    gen_spr_ne_601(env);
    gen_tbl(env);
    gen_spr_usprg3(env);
    gen_spr_book3s_altivec(env);
    /*
     * Can't find information on what this should be on reset.  This
     * value is the one used by 74xx processors.
     */
    vscr_init(env, 0x00010000);
    gen_spr_book3s_pmu_sup(env);
    gen_spr_book3s_pmu_user(env);
    gen_spr_book3s_ctrl(env);
}

static void init_proc_970(CPUPPCState *env)
{
    /* Common Registers */
    init_proc_book3s_common(env);
    gen_spr_sdr1(env);
    gen_spr_book3s_dbg(env);

    /* 970 Specific Registers */
    gen_spr_970_hid(env);
    gen_spr_970_hior(env);
    gen_low_BATs(env);
    gen_spr_970_pmu_sup(env);
    gen_spr_970_pmu_user(env);
    gen_spr_970_lpar(env);
    gen_spr_970_dbg(env);

    /* env variables */
    env->dcache_line_size = 128;
    env->icache_line_size = 128;

    /* Allocate hardware IRQ controller */
    init_excp_970(env);
    ppc970_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(970)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->desc = "PowerPC 970";
    pcc->init_proc = init_proc_970;
    pcc->check_pow = check_pow_970;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_64B | PPC_ALTIVEC |
                       PPC_SEGMENT_64B | PPC_SLBI;
    pcc->insns_flags2 = PPC2_FP_CVT_S64;
    pcc->msr_mask = (1ull << MSR_SF) |
                    (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI);
    pcc->mmu_model = POWERPC_MMU_64B;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash64_handle_mmu_fault;
    pcc->hash64_opts = &ppc_hash64_opts_basic;
#endif
    pcc->excp_model = POWERPC_EXCP_970;
    pcc->bus_model = PPC_FLAGS_INPUT_970;
    pcc->bfd_mach = bfd_mach_ppc64;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
    pcc->l1_dcache_size = 0x8000;
    pcc->l1_icache_size = 0x10000;
}

static void init_proc_power5plus(CPUPPCState *env)
{
    /* Common Registers */
    init_proc_book3s_common(env);
    gen_spr_sdr1(env);
    gen_spr_book3s_dbg(env);

    /* POWER5+ Specific Registers */
    gen_spr_970_hid(env);
    gen_spr_970_hior(env);
    gen_low_BATs(env);
    gen_spr_970_pmu_sup(env);
    gen_spr_970_pmu_user(env);
    gen_spr_power5p_common(env);
    gen_spr_power5p_lpar(env);
    gen_spr_power5p_ear(env);
    gen_spr_power5p_tb(env);

    /* env variables */
    env->dcache_line_size = 128;
    env->icache_line_size = 128;

    /* Allocate hardware IRQ controller */
    init_excp_970(env);
    ppc970_irq_init(env_archcpu(env));
}

POWERPC_FAMILY(POWER5P)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);

    dc->fw_name = "PowerPC,POWER5";
    dc->desc = "POWER5+";
    pcc->init_proc = init_proc_power5plus;
    pcc->check_pow = check_pow_970;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_STFIWX |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_64B |
                       PPC_SEGMENT_64B | PPC_SLBI;
    pcc->insns_flags2 = PPC2_FP_CVT_S64;
    pcc->msr_mask = (1ull << MSR_SF) |
                    (1ull << MSR_VR) |
                    (1ull << MSR_POW) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI);
    pcc->lpcr_mask = LPCR_RMLS | LPCR_ILE | LPCR_LPES0 | LPCR_LPES1 |
        LPCR_RMI | LPCR_HDICE;
    pcc->mmu_model = POWERPC_MMU_2_03;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash64_handle_mmu_fault;
    pcc->hash64_opts = &ppc_hash64_opts_basic;
    pcc->lrg_decr_bits = 32;
#endif
    pcc->excp_model = POWERPC_EXCP_970;
    pcc->bus_model = PPC_FLAGS_INPUT_970;
    pcc->bfd_mach = bfd_mach_ppc64;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK;
    pcc->l1_dcache_size = 0x8000;
    pcc->l1_icache_size = 0x10000;
}

static void init_proc_POWER7(CPUPPCState *env)
{
    /* Common Registers */
    init_proc_book3s_common(env);
    gen_spr_sdr1(env);
    gen_spr_book3s_dbg(env);

    /* POWER7 Specific Registers */
    gen_spr_book3s_ids(env);
    gen_spr_rmor(env);
    gen_spr_amr(env);
    gen_spr_book3s_purr(env);
    gen_spr_power5p_common(env);
    gen_spr_power5p_lpar(env);
    gen_spr_power5p_ear(env);
    gen_spr_power5p_tb(env);
    gen_spr_power6_common(env);
    gen_spr_power6_dbg(env);
    gen_spr_power7_book4(env);

    /* env variables */
    env->dcache_line_size = 128;
    env->icache_line_size = 128;

    /* Allocate hardware IRQ controller */
    init_excp_POWER7(env);
    ppcPOWER7_irq_init(env_archcpu(env));
}

static bool ppc_pvr_match_power7(PowerPCCPUClass *pcc, uint32_t pvr)
{
    if ((pvr & CPU_POWERPC_POWER_SERVER_MASK) == CPU_POWERPC_POWER7P_BASE) {
        return true;
    }
    if ((pvr & CPU_POWERPC_POWER_SERVER_MASK) == CPU_POWERPC_POWER7_BASE) {
        return true;
    }
    return false;
}

static bool cpu_has_work_POWER7(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;

    if (cs->halted) {
        if (!(cs->interrupt_request & CPU_INTERRUPT_HARD)) {
            return false;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_EXT)) &&
            (env->spr[SPR_LPCR] & LPCR_P7_PECE0)) {
            return true;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_DECR)) &&
            (env->spr[SPR_LPCR] & LPCR_P7_PECE1)) {
            return true;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_MCK)) &&
            (env->spr[SPR_LPCR] & LPCR_P7_PECE2)) {
            return true;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_HMI)) &&
            (env->spr[SPR_LPCR] & LPCR_P7_PECE2)) {
            return true;
        }
        if (env->pending_interrupts & (1u << PPC_INTERRUPT_RESET)) {
            return true;
        }
        return false;
    } else {
        return msr_ee && (cs->interrupt_request & CPU_INTERRUPT_HARD);
    }
}

POWERPC_FAMILY(POWER7)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);

    dc->fw_name = "PowerPC,POWER7";
    dc->desc = "POWER7";
    pcc->pvr_match = ppc_pvr_match_power7;
    pcc->pcr_mask = PCR_VEC_DIS | PCR_VSX_DIS | PCR_COMPAT_2_05;
    pcc->pcr_supported = PCR_COMPAT_2_06 | PCR_COMPAT_2_05;
    pcc->init_proc = init_proc_POWER7;
    pcc->check_pow = check_pow_nocheck;
    cc->has_work = cpu_has_work_POWER7;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_FRSQRTES |
                       PPC_FLOAT_STFIWX |
                       PPC_FLOAT_EXT |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_64B | PPC_64H | PPC_64BX | PPC_ALTIVEC |
                       PPC_SEGMENT_64B | PPC_SLBI |
                       PPC_POPCNTB | PPC_POPCNTWD |
                       PPC_CILDST;
    pcc->insns_flags2 = PPC2_VSX | PPC2_DFP | PPC2_DBRX | PPC2_ISA205 |
                        PPC2_PERM_ISA206 | PPC2_DIVE_ISA206 |
                        PPC2_ATOMIC_ISA206 | PPC2_FP_CVT_ISA206 |
                        PPC2_FP_TST_ISA206 | PPC2_FP_CVT_S64 |
                        PPC2_PM_ISA206;
    pcc->msr_mask = (1ull << MSR_SF) |
                    (1ull << MSR_VR) |
                    (1ull << MSR_VSX) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->lpcr_mask = LPCR_VPM0 | LPCR_VPM1 | LPCR_ISL | LPCR_DPFD |
        LPCR_VRMASD | LPCR_RMLS | LPCR_ILE |
        LPCR_P7_PECE0 | LPCR_P7_PECE1 | LPCR_P7_PECE2 |
        LPCR_MER | LPCR_TC |
        LPCR_LPES0 | LPCR_LPES1 | LPCR_HDICE;
    pcc->lpcr_pm = LPCR_P7_PECE0 | LPCR_P7_PECE1 | LPCR_P7_PECE2;
    pcc->mmu_model = POWERPC_MMU_2_06;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash64_handle_mmu_fault;
    pcc->hash64_opts = &ppc_hash64_opts_POWER7;
    pcc->lrg_decr_bits = 32;
#endif
    pcc->excp_model = POWERPC_EXCP_POWER7;
    pcc->bus_model = PPC_FLAGS_INPUT_POWER7;
    pcc->bfd_mach = bfd_mach_ppc64;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK | POWERPC_FLAG_CFAR |
                 POWERPC_FLAG_VSX;
    pcc->l1_dcache_size = 0x8000;
    pcc->l1_icache_size = 0x8000;
    pcc->interrupts_big_endian = ppc_cpu_interrupts_big_endian_lpcr;
}

static void init_proc_POWER8(CPUPPCState *env)
{
    /* Common Registers */
    init_proc_book3s_common(env);
    gen_spr_sdr1(env);
    gen_spr_book3s_207_dbg(env);

    /* POWER8 Specific Registers */
    gen_spr_book3s_ids(env);
    gen_spr_rmor(env);
    gen_spr_amr(env);
    gen_spr_iamr(env);
    gen_spr_book3s_purr(env);
    gen_spr_power5p_common(env);
    gen_spr_power5p_lpar(env);
    gen_spr_power5p_ear(env);
    gen_spr_power5p_tb(env);
    gen_spr_power6_common(env);
    gen_spr_power6_dbg(env);
    gen_spr_power8_tce_address_control(env);
    gen_spr_power8_ids(env);
    gen_spr_power8_ebb(env);
    gen_spr_power8_fscr(env);
    gen_spr_power8_pmu_sup(env);
    gen_spr_power8_pmu_user(env);
    gen_spr_power8_tm(env);
    gen_spr_power8_pspb(env);
    gen_spr_power8_dpdes(env);
    gen_spr_vtb(env);
    gen_spr_power8_ic(env);
    gen_spr_power8_book4(env);
    gen_spr_power8_rpr(env);

    /* env variables */
    env->dcache_line_size = 128;
    env->icache_line_size = 128;

    /* Allocate hardware IRQ controller */
    init_excp_POWER8(env);
    ppcPOWER7_irq_init(env_archcpu(env));
}

static bool ppc_pvr_match_power8(PowerPCCPUClass *pcc, uint32_t pvr)
{
    if ((pvr & CPU_POWERPC_POWER_SERVER_MASK) == CPU_POWERPC_POWER8NVL_BASE) {
        return true;
    }
    if ((pvr & CPU_POWERPC_POWER_SERVER_MASK) == CPU_POWERPC_POWER8E_BASE) {
        return true;
    }
    if ((pvr & CPU_POWERPC_POWER_SERVER_MASK) == CPU_POWERPC_POWER8_BASE) {
        return true;
    }
    return false;
}

static bool cpu_has_work_POWER8(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;

    if (cs->halted) {
        if (!(cs->interrupt_request & CPU_INTERRUPT_HARD)) {
            return false;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_EXT)) &&
            (env->spr[SPR_LPCR] & LPCR_P8_PECE2)) {
            return true;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_DECR)) &&
            (env->spr[SPR_LPCR] & LPCR_P8_PECE3)) {
            return true;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_MCK)) &&
            (env->spr[SPR_LPCR] & LPCR_P8_PECE4)) {
            return true;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_HMI)) &&
            (env->spr[SPR_LPCR] & LPCR_P8_PECE4)) {
            return true;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_DOORBELL)) &&
            (env->spr[SPR_LPCR] & LPCR_P8_PECE0)) {
            return true;
        }
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_HDOORBELL)) &&
            (env->spr[SPR_LPCR] & LPCR_P8_PECE1)) {
            return true;
        }
        if (env->pending_interrupts & (1u << PPC_INTERRUPT_RESET)) {
            return true;
        }
        return false;
    } else {
        return msr_ee && (cs->interrupt_request & CPU_INTERRUPT_HARD);
    }
}

POWERPC_FAMILY(POWER8)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);

    dc->fw_name = "PowerPC,POWER8";
    dc->desc = "POWER8";
    pcc->pvr_match = ppc_pvr_match_power8;
    pcc->pcr_mask = PCR_TM_DIS | PCR_COMPAT_2_06 | PCR_COMPAT_2_05;
    pcc->pcr_supported = PCR_COMPAT_2_07 | PCR_COMPAT_2_06 | PCR_COMPAT_2_05;
    pcc->init_proc = init_proc_POWER8;
    pcc->check_pow = check_pow_nocheck;
    cc->has_work = cpu_has_work_POWER8;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_FRSQRTES |
                       PPC_FLOAT_STFIWX |
                       PPC_FLOAT_EXT |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBIE | PPC_MEM_TLBSYNC |
                       PPC_64B | PPC_64H | PPC_64BX | PPC_ALTIVEC |
                       PPC_SEGMENT_64B | PPC_SLBI |
                       PPC_POPCNTB | PPC_POPCNTWD |
                       PPC_CILDST;
    pcc->insns_flags2 = PPC2_VSX | PPC2_VSX207 | PPC2_DFP | PPC2_DBRX |
                        PPC2_PERM_ISA206 | PPC2_DIVE_ISA206 |
                        PPC2_ATOMIC_ISA206 | PPC2_FP_CVT_ISA206 |
                        PPC2_FP_TST_ISA206 | PPC2_BCTAR_ISA207 |
                        PPC2_LSQ_ISA207 | PPC2_ALTIVEC_207 |
                        PPC2_ISA205 | PPC2_ISA207S | PPC2_FP_CVT_S64 |
                        PPC2_TM | PPC2_PM_ISA206;
    pcc->msr_mask = (1ull << MSR_SF) |
                    (1ull << MSR_HV) |
                    (1ull << MSR_TM) |
                    (1ull << MSR_VR) |
                    (1ull << MSR_VSX) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_TS0) |
                    (1ull << MSR_TS1) |
                    (1ull << MSR_LE);
    pcc->lpcr_mask = LPCR_VPM0 | LPCR_VPM1 | LPCR_ISL | LPCR_KBV |
        LPCR_DPFD | LPCR_VRMASD | LPCR_RMLS | LPCR_ILE |
        LPCR_AIL | LPCR_ONL | LPCR_P8_PECE0 | LPCR_P8_PECE1 |
        LPCR_P8_PECE2 | LPCR_P8_PECE3 | LPCR_P8_PECE4 |
        LPCR_MER | LPCR_TC | LPCR_LPES0 | LPCR_HDICE;
    pcc->lpcr_pm = LPCR_P8_PECE0 | LPCR_P8_PECE1 | LPCR_P8_PECE2 |
                   LPCR_P8_PECE3 | LPCR_P8_PECE4;
    pcc->mmu_model = POWERPC_MMU_2_07;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc_hash64_handle_mmu_fault;
    pcc->hash64_opts = &ppc_hash64_opts_POWER7;
    pcc->lrg_decr_bits = 32;
    pcc->n_host_threads = 8;
#endif
    pcc->excp_model = POWERPC_EXCP_POWER8;
    pcc->bus_model = PPC_FLAGS_INPUT_POWER7;
    pcc->bfd_mach = bfd_mach_ppc64;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK | POWERPC_FLAG_CFAR |
                 POWERPC_FLAG_VSX | POWERPC_FLAG_TM;
    pcc->l1_dcache_size = 0x8000;
    pcc->l1_icache_size = 0x8000;
    pcc->interrupts_big_endian = ppc_cpu_interrupts_big_endian_lpcr;
}

#ifdef CONFIG_SOFTMMU
/*
 * Radix pg sizes and AP encodings for dt node ibm,processor-radix-AP-encodings
 * Encoded as array of int_32s in the form:
 *  0bxxxyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
 *  x -> AP encoding
 *  y -> radix mode supported page size (encoded as a shift)
 */
static struct ppc_radix_page_info POWER9_radix_page_info = {
    .count = 4,
    .entries = {
        0x0000000c, /*  4K - enc: 0x0 */
        0xa0000010, /* 64K - enc: 0x5 */
        0x20000015, /*  2M - enc: 0x1 */
        0x4000001e  /*  1G - enc: 0x2 */
    }
};
#endif /* CONFIG_SOFTMMU */

static void init_proc_POWER9(CPUPPCState *env)
{
    /* Common Registers */
    init_proc_book3s_common(env);
    gen_spr_book3s_207_dbg(env);

    /* POWER8 Specific Registers */
    gen_spr_book3s_ids(env);
    gen_spr_amr(env);
    gen_spr_iamr(env);
    gen_spr_book3s_purr(env);
    gen_spr_power5p_common(env);
    gen_spr_power5p_lpar(env);
    gen_spr_power5p_ear(env);
    gen_spr_power5p_tb(env);
    gen_spr_power6_common(env);
    gen_spr_power6_dbg(env);
    gen_spr_power8_tce_address_control(env);
    gen_spr_power8_ids(env);
    gen_spr_power8_ebb(env);
    gen_spr_power8_fscr(env);
    gen_spr_power8_pmu_sup(env);
    gen_spr_power8_pmu_user(env);
    gen_spr_power8_tm(env);
    gen_spr_power8_pspb(env);
    gen_spr_power8_dpdes(env);
    gen_spr_vtb(env);
    gen_spr_power8_ic(env);
    gen_spr_power8_book4(env);
    gen_spr_power8_rpr(env);
    gen_spr_power9_mmu(env);

    /* POWER9 Specific registers */
    gen_spr_TIDR(env);

    gen_spr_PSSCR(env);

    /* env variables */
    env->dcache_line_size = 128;
    env->icache_line_size = 128;

    /* Allocate hardware IRQ controller */
    init_excp_POWER9(env);
    ppcPOWER9_irq_init(env_archcpu(env));
}

static bool ppc_pvr_match_power9(PowerPCCPUClass *pcc, uint32_t pvr)
{
    if ((pvr & CPU_POWERPC_POWER_SERVER_MASK) == CPU_POWERPC_POWER9_BASE) {
        return true;
    }
    return false;
}

static bool cpu_has_work_POWER9(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;

    if (cs->halted) {
        uint64_t psscr = env->spr[SPR_PSSCR];

        if (!(cs->interrupt_request & CPU_INTERRUPT_HARD)) {
            return false;
        }

        /* If EC is clear, just return true on any pending interrupt */
        if (!(psscr & PSSCR_EC)) {
            return true;
        }
        /* External Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_EXT)) &&
            (env->spr[SPR_LPCR] & LPCR_EEE)) {
            bool heic = !!(env->spr[SPR_LPCR] & LPCR_HEIC);
            if (heic == 0 || !msr_hv || msr_pr) {
                return true;
            }
        }
        /* Decrementer Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_DECR)) &&
            (env->spr[SPR_LPCR] & LPCR_DEE)) {
            return true;
        }
        /* Machine Check or Hypervisor Maintenance Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_MCK |
            1u << PPC_INTERRUPT_HMI)) && (env->spr[SPR_LPCR] & LPCR_OEE)) {
            return true;
        }
        /* Privileged Doorbell Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_DOORBELL)) &&
            (env->spr[SPR_LPCR] & LPCR_PDEE)) {
            return true;
        }
        /* Hypervisor Doorbell Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_HDOORBELL)) &&
            (env->spr[SPR_LPCR] & LPCR_HDEE)) {
            return true;
        }
        /* Hypervisor virtualization exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_HVIRT)) &&
            (env->spr[SPR_LPCR] & LPCR_HVEE)) {
            return true;
        }
        if (env->pending_interrupts & (1u << PPC_INTERRUPT_RESET)) {
            return true;
        }
        return false;
    } else {
        return msr_ee && (cs->interrupt_request & CPU_INTERRUPT_HARD);
    }
}

POWERPC_FAMILY(POWER9)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);

    dc->fw_name = "PowerPC,POWER9";
    dc->desc = "POWER9";
    pcc->pvr_match = ppc_pvr_match_power9;
    pcc->pcr_mask = PCR_COMPAT_2_05 | PCR_COMPAT_2_06 | PCR_COMPAT_2_07;
    pcc->pcr_supported = PCR_COMPAT_3_00 | PCR_COMPAT_2_07 | PCR_COMPAT_2_06 |
                         PCR_COMPAT_2_05;
    pcc->init_proc = init_proc_POWER9;
    pcc->check_pow = check_pow_nocheck;
    cc->has_work = cpu_has_work_POWER9;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_FRSQRTES |
                       PPC_FLOAT_STFIWX |
                       PPC_FLOAT_EXT |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBSYNC |
                       PPC_64B | PPC_64H | PPC_64BX | PPC_ALTIVEC |
                       PPC_SEGMENT_64B | PPC_SLBI |
                       PPC_POPCNTB | PPC_POPCNTWD |
                       PPC_CILDST;
    pcc->insns_flags2 = PPC2_VSX | PPC2_VSX207 | PPC2_DFP | PPC2_DBRX |
                        PPC2_PERM_ISA206 | PPC2_DIVE_ISA206 |
                        PPC2_ATOMIC_ISA206 | PPC2_FP_CVT_ISA206 |
                        PPC2_FP_TST_ISA206 | PPC2_BCTAR_ISA207 |
                        PPC2_LSQ_ISA207 | PPC2_ALTIVEC_207 |
                        PPC2_ISA205 | PPC2_ISA207S | PPC2_FP_CVT_S64 |
                        PPC2_TM | PPC2_ISA300 | PPC2_PRCNTL;
    pcc->msr_mask = (1ull << MSR_SF) |
                    (1ull << MSR_HV) |
                    (1ull << MSR_TM) |
                    (1ull << MSR_VR) |
                    (1ull << MSR_VSX) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->lpcr_mask = LPCR_VPM1 | LPCR_ISL | LPCR_KBV | LPCR_DPFD |
        (LPCR_PECE_U_MASK & LPCR_HVEE) | LPCR_ILE | LPCR_AIL |
        LPCR_UPRT | LPCR_EVIRT | LPCR_ONL | LPCR_HR | LPCR_LD |
        (LPCR_PECE_L_MASK & (LPCR_PDEE | LPCR_HDEE | LPCR_EEE |
                             LPCR_DEE | LPCR_OEE))
        | LPCR_MER | LPCR_GTSE | LPCR_TC |
        LPCR_HEIC | LPCR_LPES0 | LPCR_HVICE | LPCR_HDICE;
    pcc->lpcr_pm = LPCR_PDEE | LPCR_HDEE | LPCR_EEE | LPCR_DEE | LPCR_OEE;
    pcc->mmu_model = POWERPC_MMU_3_00;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc64_v3_handle_mmu_fault;
    /* segment page size remain the same */
    pcc->hash64_opts = &ppc_hash64_opts_POWER7;
    pcc->radix_page_info = &POWER9_radix_page_info;
    pcc->lrg_decr_bits = 56;
    pcc->n_host_threads = 4;
#endif
    pcc->excp_model = POWERPC_EXCP_POWER9;
    pcc->bus_model = PPC_FLAGS_INPUT_POWER9;
    pcc->bfd_mach = bfd_mach_ppc64;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK | POWERPC_FLAG_CFAR |
                 POWERPC_FLAG_VSX | POWERPC_FLAG_TM | POWERPC_FLAG_SCV;
    pcc->l1_dcache_size = 0x8000;
    pcc->l1_icache_size = 0x8000;
    pcc->interrupts_big_endian = ppc_cpu_interrupts_big_endian_lpcr;
}

#ifdef CONFIG_SOFTMMU
/*
 * Radix pg sizes and AP encodings for dt node ibm,processor-radix-AP-encodings
 * Encoded as array of int_32s in the form:
 *  0bxxxyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
 *  x -> AP encoding
 *  y -> radix mode supported page size (encoded as a shift)
 */
static struct ppc_radix_page_info POWER10_radix_page_info = {
    .count = 4,
    .entries = {
        0x0000000c, /*  4K - enc: 0x0 */
        0xa0000010, /* 64K - enc: 0x5 */
        0x20000015, /*  2M - enc: 0x1 */
        0x4000001e  /*  1G - enc: 0x2 */
    }
};
#endif /* CONFIG_SOFTMMU */

static void init_proc_POWER10(CPUPPCState *env)
{
    /* Common Registers */
    init_proc_book3s_common(env);
    gen_spr_book3s_207_dbg(env);

    /* POWER8 Specific Registers */
    gen_spr_book3s_ids(env);
    gen_spr_amr(env);
    gen_spr_iamr(env);
    gen_spr_book3s_purr(env);
    gen_spr_power5p_common(env);
    gen_spr_power5p_lpar(env);
    gen_spr_power5p_ear(env);
    gen_spr_power6_common(env);
    gen_spr_power6_dbg(env);
    gen_spr_power8_tce_address_control(env);
    gen_spr_power8_ids(env);
    gen_spr_power8_ebb(env);
    gen_spr_power8_fscr(env);
    gen_spr_power8_pmu_sup(env);
    gen_spr_power8_pmu_user(env);
    gen_spr_power8_tm(env);
    gen_spr_power8_pspb(env);
    gen_spr_vtb(env);
    gen_spr_power8_ic(env);
    gen_spr_power8_book4(env);
    gen_spr_power8_rpr(env);
    gen_spr_power9_mmu(env);

    gen_spr_PSSCR(env);

    /* env variables */
    env->dcache_line_size = 128;
    env->icache_line_size = 128;

    /* Allocate hardware IRQ controller */
    init_excp_POWER10(env);
    ppcPOWER9_irq_init(env_archcpu(env));
}

static bool ppc_pvr_match_power10(PowerPCCPUClass *pcc, uint32_t pvr)
{
    if ((pvr & CPU_POWERPC_POWER_SERVER_MASK) == CPU_POWERPC_POWER10_BASE) {
        return true;
    }
    return false;
}

static bool cpu_has_work_POWER10(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;

    if (cs->halted) {
        uint64_t psscr = env->spr[SPR_PSSCR];

        if (!(cs->interrupt_request & CPU_INTERRUPT_HARD)) {
            return false;
        }

        /* If EC is clear, just return true on any pending interrupt */
        if (!(psscr & PSSCR_EC)) {
            return true;
        }
        /* External Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_EXT)) &&
            (env->spr[SPR_LPCR] & LPCR_EEE)) {
            bool heic = !!(env->spr[SPR_LPCR] & LPCR_HEIC);
            if (heic == 0 || !msr_hv || msr_pr) {
                return true;
            }
        }
        /* Decrementer Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_DECR)) &&
            (env->spr[SPR_LPCR] & LPCR_DEE)) {
            return true;
        }
        /* Machine Check or Hypervisor Maintenance Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_MCK |
            1u << PPC_INTERRUPT_HMI)) && (env->spr[SPR_LPCR] & LPCR_OEE)) {
            return true;
        }
        /* Privileged Doorbell Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_DOORBELL)) &&
            (env->spr[SPR_LPCR] & LPCR_PDEE)) {
            return true;
        }
        /* Hypervisor Doorbell Exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_HDOORBELL)) &&
            (env->spr[SPR_LPCR] & LPCR_HDEE)) {
            return true;
        }
        /* Hypervisor virtualization exception */
        if ((env->pending_interrupts & (1u << PPC_INTERRUPT_HVIRT)) &&
            (env->spr[SPR_LPCR] & LPCR_HVEE)) {
            return true;
        }
        if (env->pending_interrupts & (1u << PPC_INTERRUPT_RESET)) {
            return true;
        }
        return false;
    } else {
        return msr_ee && (cs->interrupt_request & CPU_INTERRUPT_HARD);
    }
}

POWERPC_FAMILY(POWER10)(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);

    dc->fw_name = "PowerPC,POWER10";
    dc->desc = "POWER10";
    pcc->pvr_match = ppc_pvr_match_power10;
    pcc->pcr_mask = PCR_COMPAT_2_05 | PCR_COMPAT_2_06 | PCR_COMPAT_2_07 |
                    PCR_COMPAT_3_00;
    pcc->pcr_supported = PCR_COMPAT_3_10 | PCR_COMPAT_3_00 | PCR_COMPAT_2_07 |
                         PCR_COMPAT_2_06 | PCR_COMPAT_2_05;
    pcc->init_proc = init_proc_POWER10;
    pcc->check_pow = check_pow_nocheck;
    cc->has_work = cpu_has_work_POWER10;
    pcc->insns_flags = PPC_INSNS_BASE | PPC_ISEL | PPC_STRING | PPC_MFTB |
                       PPC_FLOAT | PPC_FLOAT_FSEL | PPC_FLOAT_FRES |
                       PPC_FLOAT_FSQRT | PPC_FLOAT_FRSQRTE |
                       PPC_FLOAT_FRSQRTES |
                       PPC_FLOAT_STFIWX |
                       PPC_FLOAT_EXT |
                       PPC_CACHE | PPC_CACHE_ICBI | PPC_CACHE_DCBZ |
                       PPC_MEM_SYNC | PPC_MEM_EIEIO |
                       PPC_MEM_TLBSYNC |
                       PPC_64B | PPC_64H | PPC_64BX | PPC_ALTIVEC |
                       PPC_SEGMENT_64B | PPC_SLBI |
                       PPC_POPCNTB | PPC_POPCNTWD |
                       PPC_CILDST;
    pcc->insns_flags2 = PPC2_VSX | PPC2_VSX207 | PPC2_DFP | PPC2_DBRX |
                        PPC2_PERM_ISA206 | PPC2_DIVE_ISA206 |
                        PPC2_ATOMIC_ISA206 | PPC2_FP_CVT_ISA206 |
                        PPC2_FP_TST_ISA206 | PPC2_BCTAR_ISA207 |
                        PPC2_LSQ_ISA207 | PPC2_ALTIVEC_207 |
                        PPC2_ISA205 | PPC2_ISA207S | PPC2_FP_CVT_S64 |
                        PPC2_TM | PPC2_ISA300 | PPC2_PRCNTL | PPC2_ISA310;
    pcc->msr_mask = (1ull << MSR_SF) |
                    (1ull << MSR_HV) |
                    (1ull << MSR_TM) |
                    (1ull << MSR_VR) |
                    (1ull << MSR_VSX) |
                    (1ull << MSR_EE) |
                    (1ull << MSR_PR) |
                    (1ull << MSR_FP) |
                    (1ull << MSR_ME) |
                    (1ull << MSR_FE0) |
                    (1ull << MSR_SE) |
                    (1ull << MSR_DE) |
                    (1ull << MSR_FE1) |
                    (1ull << MSR_IR) |
                    (1ull << MSR_DR) |
                    (1ull << MSR_PMM) |
                    (1ull << MSR_RI) |
                    (1ull << MSR_LE);
    pcc->lpcr_mask = LPCR_VPM1 | LPCR_ISL | LPCR_KBV | LPCR_DPFD |
        (LPCR_PECE_U_MASK & LPCR_HVEE) | LPCR_ILE | LPCR_AIL |
        LPCR_UPRT | LPCR_EVIRT | LPCR_ONL | LPCR_HR | LPCR_LD |
        (LPCR_PECE_L_MASK & (LPCR_PDEE | LPCR_HDEE | LPCR_EEE |
                             LPCR_DEE | LPCR_OEE))
        | LPCR_MER | LPCR_GTSE | LPCR_TC |
        LPCR_HEIC | LPCR_LPES0 | LPCR_HVICE | LPCR_HDICE;
    pcc->lpcr_pm = LPCR_PDEE | LPCR_HDEE | LPCR_EEE | LPCR_DEE | LPCR_OEE;
    pcc->mmu_model = POWERPC_MMU_3_00;
#if defined(CONFIG_SOFTMMU)
    pcc->handle_mmu_fault = ppc64_v3_handle_mmu_fault;
    /* segment page size remain the same */
    pcc->hash64_opts = &ppc_hash64_opts_POWER7;
    pcc->radix_page_info = &POWER10_radix_page_info;
    pcc->lrg_decr_bits = 56;
#endif
    pcc->excp_model = POWERPC_EXCP_POWER9;
    pcc->bus_model = PPC_FLAGS_INPUT_POWER9;
    pcc->bfd_mach = bfd_mach_ppc64;
    pcc->flags = POWERPC_FLAG_VRE | POWERPC_FLAG_SE |
                 POWERPC_FLAG_BE | POWERPC_FLAG_PMM |
                 POWERPC_FLAG_BUS_CLK | POWERPC_FLAG_CFAR |
                 POWERPC_FLAG_VSX | POWERPC_FLAG_TM;
    pcc->l1_dcache_size = 0x8000;
    pcc->l1_icache_size = 0x8000;
    pcc->interrupts_big_endian = ppc_cpu_interrupts_big_endian_lpcr;
}

#if !defined(CONFIG_USER_ONLY)
void cpu_ppc_set_vhyp(PowerPCCPU *cpu, PPCVirtualHypervisor *vhyp)
{
    CPUPPCState *env = &cpu->env;

    cpu->vhyp = vhyp;

    /*
     * With a virtual hypervisor mode we never allow the CPU to go
     * hypervisor mode itself
     */
    env->msr_mask &= ~MSR_HVB;
}

#endif /* !defined(CONFIG_USER_ONLY) */

#endif /* defined(TARGET_PPC64) */

/*****************************************************************************/
/* Generic CPU instantiation routine                                         */
static void init_ppc_proc(PowerPCCPU *cpu)
{
    PowerPCCPUClass *pcc = POWERPC_CPU_GET_CLASS(cpu);
    CPUPPCState *env = &cpu->env;
#if !defined(CONFIG_USER_ONLY)
    int i;

    env->irq_inputs = NULL;
    /* Set all exception vectors to an invalid address */
    for (i = 0; i < POWERPC_EXCP_NB; i++) {
        env->excp_vectors[i] = (target_ulong)(-1ULL);
    }
    env->ivor_mask = 0x00000000;
    env->ivpr_mask = 0x00000000;
    /* Default MMU definitions */
    env->nb_BATs = 0;
    env->nb_tlb = 0;
    env->nb_ways = 0;
    env->tlb_type = TLB_NONE;
#endif
    /* Register SPR common to all PowerPC implementations */
    gen_spr_generic(env);
    gen_spr_pvr(env, pcc);
    /* Register SVR if it's defined to anything else than POWERPC_SVR_NONE */
    if (pcc->svr != POWERPC_SVR_NONE) {
        gen_spr_svr(env, pcc);
    }
    /* PowerPC implementation specific initialisations (SPRs, timers, ...) */
    (*pcc->init_proc)(env);

#if !defined(CONFIG_USER_ONLY)
    ppc_gdb_gen_spr_xml(cpu);
#endif

    /* MSR bits & flags consistency checks */
    if (env->msr_mask & (1 << 25)) {
        switch (env->flags & (POWERPC_FLAG_SPE | POWERPC_FLAG_VRE)) {
        case POWERPC_FLAG_SPE:
        case POWERPC_FLAG_VRE:
            break;
        default:
            fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                    "Should define POWERPC_FLAG_SPE or POWERPC_FLAG_VRE\n");
            exit(1);
        }
    } else if (env->flags & (POWERPC_FLAG_SPE | POWERPC_FLAG_VRE)) {
        fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                "Should not define POWERPC_FLAG_SPE nor POWERPC_FLAG_VRE\n");
        exit(1);
    }
    if (env->msr_mask & (1 << 17)) {
        switch (env->flags & (POWERPC_FLAG_TGPR | POWERPC_FLAG_CE)) {
        case POWERPC_FLAG_TGPR:
        case POWERPC_FLAG_CE:
            break;
        default:
            fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                    "Should define POWERPC_FLAG_TGPR or POWERPC_FLAG_CE\n");
            exit(1);
        }
    } else if (env->flags & (POWERPC_FLAG_TGPR | POWERPC_FLAG_CE)) {
        fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                "Should not define POWERPC_FLAG_TGPR nor POWERPC_FLAG_CE\n");
        exit(1);
    }
    if (env->msr_mask & (1 << 10)) {
        switch (env->flags & (POWERPC_FLAG_SE | POWERPC_FLAG_DWE |
                              POWERPC_FLAG_UBLE)) {
        case POWERPC_FLAG_SE:
        case POWERPC_FLAG_DWE:
        case POWERPC_FLAG_UBLE:
            break;
        default:
            fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                    "Should define POWERPC_FLAG_SE or POWERPC_FLAG_DWE or "
                    "POWERPC_FLAG_UBLE\n");
            exit(1);
        }
    } else if (env->flags & (POWERPC_FLAG_SE | POWERPC_FLAG_DWE |
                             POWERPC_FLAG_UBLE)) {
        fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                "Should not define POWERPC_FLAG_SE nor POWERPC_FLAG_DWE nor "
                "POWERPC_FLAG_UBLE\n");
            exit(1);
    }
    if (env->msr_mask & (1 << 9)) {
        switch (env->flags & (POWERPC_FLAG_BE | POWERPC_FLAG_DE)) {
        case POWERPC_FLAG_BE:
        case POWERPC_FLAG_DE:
            break;
        default:
            fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                    "Should define POWERPC_FLAG_BE or POWERPC_FLAG_DE\n");
            exit(1);
        }
    } else if (env->flags & (POWERPC_FLAG_BE | POWERPC_FLAG_DE)) {
        fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                "Should not define POWERPC_FLAG_BE nor POWERPC_FLAG_DE\n");
        exit(1);
    }
    if (env->msr_mask & (1 << 2)) {
        switch (env->flags & (POWERPC_FLAG_PX | POWERPC_FLAG_PMM)) {
        case POWERPC_FLAG_PX:
        case POWERPC_FLAG_PMM:
            break;
        default:
            fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                    "Should define POWERPC_FLAG_PX or POWERPC_FLAG_PMM\n");
            exit(1);
        }
    } else if (env->flags & (POWERPC_FLAG_PX | POWERPC_FLAG_PMM)) {
        fprintf(stderr, "PowerPC MSR definition inconsistency\n"
                "Should not define POWERPC_FLAG_PX nor POWERPC_FLAG_PMM\n");
        exit(1);
    }
    if ((env->flags & (POWERPC_FLAG_RTC_CLK | POWERPC_FLAG_BUS_CLK)) == 0) {
        fprintf(stderr, "PowerPC flags inconsistency\n"
                "Should define the time-base and decrementer clock source\n");
        exit(1);
    }
    /* Allocate TLBs buffer when needed */
#if !defined(CONFIG_USER_ONLY)
    if (env->nb_tlb != 0) {
        int nb_tlb = env->nb_tlb;
        if (env->id_tlbs != 0) {
            nb_tlb *= 2;
        }
        switch (env->tlb_type) {
        case TLB_6XX:
            env->tlb.tlb6 = g_new0(ppc6xx_tlb_t, nb_tlb);
            break;
        case TLB_EMB:
            env->tlb.tlbe = g_new0(ppcemb_tlb_t, nb_tlb);
            break;
        case TLB_MAS:
            env->tlb.tlbm = g_new0(ppcmas_tlb_t, nb_tlb);
            break;
        }
        /* Pre-compute some useful values */
        env->tlb_per_way = env->nb_tlb / env->nb_ways;
    }
    if (env->irq_inputs == NULL) {
        warn_report("no internal IRQ controller registered."
                    " Attempt QEMU to crash very soon !");
    }
#endif
    if (env->check_pow == NULL) {
        warn_report("no power management check handler registered."
                    " Attempt QEMU to crash very soon !");
    }
}

#if defined(PPC_DUMP_CPU)
static void dump_ppc_sprs(CPUPPCState *env)
{
    ppc_spr_t *spr;
#if !defined(CONFIG_USER_ONLY)
    uint32_t sr, sw;
#endif
    uint32_t ur, uw;
    int i, j, n;

    printf("Special purpose registers:\n");
    for (i = 0; i < 32; i++) {
        for (j = 0; j < 32; j++) {
            n = (i << 5) | j;
            spr = &env->spr_cb[n];
            uw = spr->uea_write != NULL && spr->uea_write != SPR_NOACCESS;
            ur = spr->uea_read != NULL && spr->uea_read != SPR_NOACCESS;
#if !defined(CONFIG_USER_ONLY)
            sw = spr->oea_write != NULL && spr->oea_write != SPR_NOACCESS;
            sr = spr->oea_read != NULL && spr->oea_read != SPR_NOACCESS;
            if (sw || sr || uw || ur) {
                printf("SPR: %4d (%03x) %-8s s%c%c u%c%c\n",
                       (i << 5) | j, (i << 5) | j, spr->name,
                       sw ? 'w' : '-', sr ? 'r' : '-',
                       uw ? 'w' : '-', ur ? 'r' : '-');
            }
#else
            if (uw || ur) {
                printf("SPR: %4d (%03x) %-8s u%c%c\n",
                       (i << 5) | j, (i << 5) | j, spr->name,
                       uw ? 'w' : '-', ur ? 'r' : '-');
            }
#endif
        }
    }
    fflush(stdout);
    fflush(stderr);
}
#endif

static void ppc_cpu_realize(DeviceState *dev, Error **errp)
{
    CPUState *cs = CPU(dev);
    PowerPCCPU *cpu = POWERPC_CPU(dev);
    PowerPCCPUClass *pcc = POWERPC_CPU_GET_CLASS(cpu);
    Error *local_err = NULL;

    cpu_exec_realizefn(cs, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        return;
    }
    if (cpu->vcpu_id == UNASSIGNED_CPU_INDEX) {
        cpu->vcpu_id = cs->cpu_index;
    }

    if (tcg_enabled()) {
        if (ppc_fixup_cpu(cpu) != 0) {
            error_setg(errp, "Unable to emulate selected CPU with TCG");
            goto unrealize;
        }
    }

    create_ppc_opcodes(cpu, &local_err);
    if (local_err != NULL) {
        error_propagate(errp, local_err);
        goto unrealize;
    }
    init_ppc_proc(cpu);

    ppc_gdb_init(cs, pcc);
    qemu_init_vcpu(cs);

    pcc->parent_realize(dev, errp);

#if defined(PPC_DUMP_CPU)
    {
        CPUPPCState *env = &cpu->env;
        const char *mmu_model, *excp_model, *bus_model;
        switch (env->mmu_model) {
        case POWERPC_MMU_32B:
            mmu_model = "PowerPC 32";
            break;
        case POWERPC_MMU_SOFT_6xx:
            mmu_model = "PowerPC 6xx/7xx with software driven TLBs";
            break;
        case POWERPC_MMU_SOFT_74xx:
            mmu_model = "PowerPC 74xx with software driven TLBs";
            break;
        case POWERPC_MMU_SOFT_4xx:
            mmu_model = "PowerPC 4xx with software driven TLBs";
            break;
        case POWERPC_MMU_SOFT_4xx_Z:
            mmu_model = "PowerPC 4xx with software driven TLBs "
                "and zones protections";
            break;
        case POWERPC_MMU_REAL:
            mmu_model = "PowerPC real mode only";
            break;
        case POWERPC_MMU_MPC8xx:
            mmu_model = "PowerPC MPC8xx";
            break;
        case POWERPC_MMU_BOOKE:
            mmu_model = "PowerPC BookE";
            break;
        case POWERPC_MMU_BOOKE206:
            mmu_model = "PowerPC BookE 2.06";
            break;
        case POWERPC_MMU_601:
            mmu_model = "PowerPC 601";
            break;
#if defined(TARGET_PPC64)
        case POWERPC_MMU_64B:
            mmu_model = "PowerPC 64";
            break;
#endif
        default:
            mmu_model = "Unknown or invalid";
            break;
        }
        switch (env->excp_model) {
        case POWERPC_EXCP_STD:
            excp_model = "PowerPC";
            break;
        case POWERPC_EXCP_40x:
            excp_model = "PowerPC 40x";
            break;
        case POWERPC_EXCP_601:
            excp_model = "PowerPC 601";
            break;
        case POWERPC_EXCP_602:
            excp_model = "PowerPC 602";
            break;
        case POWERPC_EXCP_603:
            excp_model = "PowerPC 603";
            break;
        case POWERPC_EXCP_603E:
            excp_model = "PowerPC 603e";
            break;
        case POWERPC_EXCP_604:
            excp_model = "PowerPC 604";
            break;
        case POWERPC_EXCP_7x0:
            excp_model = "PowerPC 740/750";
            break;
        case POWERPC_EXCP_7x5:
            excp_model = "PowerPC 745/755";
            break;
        case POWERPC_EXCP_74xx:
            excp_model = "PowerPC 74xx";
            break;
        case POWERPC_EXCP_BOOKE:
            excp_model = "PowerPC BookE";
            break;
#if defined(TARGET_PPC64)
        case POWERPC_EXCP_970:
            excp_model = "PowerPC 970";
            break;
#endif
        default:
            excp_model = "Unknown or invalid";
            break;
        }
        switch (env->bus_model) {
        case PPC_FLAGS_INPUT_6xx:
            bus_model = "PowerPC 6xx";
            break;
        case PPC_FLAGS_INPUT_BookE:
            bus_model = "PowerPC BookE";
            break;
        case PPC_FLAGS_INPUT_405:
            bus_model = "PowerPC 405";
            break;
        case PPC_FLAGS_INPUT_401:
            bus_model = "PowerPC 401/403";
            break;
        case PPC_FLAGS_INPUT_RCPU:
            bus_model = "RCPU / MPC8xx";
            break;
#if defined(TARGET_PPC64)
        case PPC_FLAGS_INPUT_970:
            bus_model = "PowerPC 970";
            break;
#endif
        default:
            bus_model = "Unknown or invalid";
            break;
        }
        printf("PowerPC %-12s : PVR %08x MSR %016" PRIx64 "\n"
               "    MMU model        : %s\n",
               object_class_get_name(OBJECT_CLASS(pcc)),
               pcc->pvr, pcc->msr_mask, mmu_model);
#if !defined(CONFIG_USER_ONLY)
        if (env->tlb.tlb6) {
            printf("                       %d %s TLB in %d ways\n",
                   env->nb_tlb, env->id_tlbs ? "splitted" : "merged",
                   env->nb_ways);
        }
#endif
        printf("    Exceptions model : %s\n"
               "    Bus model        : %s\n",
               excp_model, bus_model);
        printf("    MSR features     :\n");
        if (env->flags & POWERPC_FLAG_SPE) {
            printf("                        signal processing engine enable"
                   "\n");
        } else if (env->flags & POWERPC_FLAG_VRE) {
            printf("                        vector processor enable\n");
        }
        if (env->flags & POWERPC_FLAG_TGPR) {
            printf("                        temporary GPRs\n");
        } else if (env->flags & POWERPC_FLAG_CE) {
            printf("                        critical input enable\n");
        }
        if (env->flags & POWERPC_FLAG_SE) {
            printf("                        single-step trace mode\n");
        } else if (env->flags & POWERPC_FLAG_DWE) {
            printf("                        debug wait enable\n");
        } else if (env->flags & POWERPC_FLAG_UBLE) {
            printf("                        user BTB lock enable\n");
        }
        if (env->flags & POWERPC_FLAG_BE) {
            printf("                        branch-step trace mode\n");
        } else if (env->flags & POWERPC_FLAG_DE) {
            printf("                        debug interrupt enable\n");
        }
        if (env->flags & POWERPC_FLAG_PX) {
            printf("                        inclusive protection\n");
        } else if (env->flags & POWERPC_FLAG_PMM) {
            printf("                        performance monitor mark\n");
        }
        if (env->flags == POWERPC_FLAG_NONE) {
            printf("                        none\n");
        }
        printf("    Time-base/decrementer clock source: %s\n",
               env->flags & POWERPC_FLAG_RTC_CLK ? "RTC clock" : "bus clock");
        dump_ppc_insns(env);
        dump_ppc_sprs(env);
        fflush(stdout);
    }
#endif
    return;

unrealize:
    cpu_exec_unrealizefn(cs);
}

static void ppc_cpu_unrealize(DeviceState *dev)
{
    PowerPCCPU *cpu = POWERPC_CPU(dev);
    PowerPCCPUClass *pcc = POWERPC_CPU_GET_CLASS(cpu);

    pcc->parent_unrealize(dev);

    cpu_remove_sync(CPU(cpu));

    destroy_ppc_opcodes(cpu);
}

static gint ppc_cpu_compare_class_pvr(gconstpointer a, gconstpointer b)
{
    ObjectClass *oc = (ObjectClass *)a;
    uint32_t pvr = *(uint32_t *)b;
    PowerPCCPUClass *pcc = (PowerPCCPUClass *)a;

    /* -cpu host does a PVR lookup during construction */
    if (unlikely(strcmp(object_class_get_name(oc),
                        TYPE_HOST_POWERPC_CPU) == 0)) {
        return -1;
    }

    return pcc->pvr == pvr ? 0 : -1;
}

PowerPCCPUClass *ppc_cpu_class_by_pvr(uint32_t pvr)
{
    GSList *list, *item;
    PowerPCCPUClass *pcc = NULL;

    list = object_class_get_list(TYPE_POWERPC_CPU, false);
    item = g_slist_find_custom(list, &pvr, ppc_cpu_compare_class_pvr);
    if (item != NULL) {
        pcc = POWERPC_CPU_CLASS(item->data);
    }
    g_slist_free(list);

    return pcc;
}

static gint ppc_cpu_compare_class_pvr_mask(gconstpointer a, gconstpointer b)
{
    ObjectClass *oc = (ObjectClass *)a;
    uint32_t pvr = *(uint32_t *)b;
    PowerPCCPUClass *pcc = (PowerPCCPUClass *)a;

    /* -cpu host does a PVR lookup during construction */
    if (unlikely(strcmp(object_class_get_name(oc),
                        TYPE_HOST_POWERPC_CPU) == 0)) {
        return -1;
    }

    if (pcc->pvr_match(pcc, pvr)) {
        return 0;
    }

    return -1;
}

PowerPCCPUClass *ppc_cpu_class_by_pvr_mask(uint32_t pvr)
{
    GSList *list, *item;
    PowerPCCPUClass *pcc = NULL;

    list = object_class_get_list(TYPE_POWERPC_CPU, true);
    item = g_slist_find_custom(list, &pvr, ppc_cpu_compare_class_pvr_mask);
    if (item != NULL) {
        pcc = POWERPC_CPU_CLASS(item->data);
    }
    g_slist_free(list);

    return pcc;
}

static const char *ppc_cpu_lookup_alias(const char *alias)
{
    int ai;

    for (ai = 0; ppc_cpu_aliases[ai].alias != NULL; ai++) {
        if (strcmp(ppc_cpu_aliases[ai].alias, alias) == 0) {
            return ppc_cpu_aliases[ai].model;
        }
    }

    return NULL;
}

static ObjectClass *ppc_cpu_class_by_name(const char *name)
{
    char *cpu_model, *typename;
    ObjectClass *oc;
    const char *p;
    unsigned long pvr;

    /*
     * Lookup by PVR if cpu_model is valid 8 digit hex number (excl:
     * 0x prefix if present)
     */
    if (!qemu_strtoul(name, &p, 16, &pvr)) {
        int len = p - name;
        len = (len == 10) && (name[1] == 'x') ? len - 2 : len;
        if ((len == 8) && (*p == '\0')) {
            return OBJECT_CLASS(ppc_cpu_class_by_pvr(pvr));
        }
    }

    cpu_model = g_ascii_strdown(name, -1);
    p = ppc_cpu_lookup_alias(cpu_model);
    if (p) {
        g_free(cpu_model);
        cpu_model = g_strdup(p);
    }

    typename = g_strdup_printf("%s" POWERPC_CPU_TYPE_SUFFIX, cpu_model);
    oc = object_class_by_name(typename);
    g_free(typename);
    g_free(cpu_model);

    return oc;
}

PowerPCCPUClass *ppc_cpu_get_family_class(PowerPCCPUClass *pcc)
{
    ObjectClass *oc = OBJECT_CLASS(pcc);

    while (oc && !object_class_is_abstract(oc)) {
        oc = object_class_get_parent(oc);
    }
    assert(oc);

    return POWERPC_CPU_CLASS(oc);
}

/* Sort by PVR, ordering special case "host" last. */
static gint ppc_cpu_list_compare(gconstpointer a, gconstpointer b)
{
    ObjectClass *oc_a = (ObjectClass *)a;
    ObjectClass *oc_b = (ObjectClass *)b;
    PowerPCCPUClass *pcc_a = POWERPC_CPU_CLASS(oc_a);
    PowerPCCPUClass *pcc_b = POWERPC_CPU_CLASS(oc_b);
    const char *name_a = object_class_get_name(oc_a);
    const char *name_b = object_class_get_name(oc_b);

    if (strcmp(name_a, TYPE_HOST_POWERPC_CPU) == 0) {
        return 1;
    } else if (strcmp(name_b, TYPE_HOST_POWERPC_CPU) == 0) {
        return -1;
    } else {
        /* Avoid an integer overflow during subtraction */
        if (pcc_a->pvr < pcc_b->pvr) {
            return -1;
        } else if (pcc_a->pvr > pcc_b->pvr) {
            return 1;
        } else {
            return 0;
        }
    }
}

static void ppc_cpu_list_entry(gpointer data, gpointer user_data)
{
    ObjectClass *oc = data;
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);
    DeviceClass *family = DEVICE_CLASS(ppc_cpu_get_family_class(pcc));
    const char *typename = object_class_get_name(oc);
    char *name;
    int i;

    if (unlikely(strcmp(typename, TYPE_HOST_POWERPC_CPU) == 0)) {
        return;
    }

    name = g_strndup(typename,
                     strlen(typename) - strlen(POWERPC_CPU_TYPE_SUFFIX));
    qemu_printf("PowerPC %-16s PVR %08x\n", name, pcc->pvr);
    for (i = 0; ppc_cpu_aliases[i].alias != NULL; i++) {
        PowerPCCPUAlias *alias = &ppc_cpu_aliases[i];
        ObjectClass *alias_oc = ppc_cpu_class_by_name(alias->model);

        if (alias_oc != oc) {
            continue;
        }
        /*
         * If running with KVM, we might update the family alias later, so
         * avoid printing the wrong alias here and use "preferred" instead
         */
        if (strcmp(alias->alias, family->desc) == 0) {
            qemu_printf("PowerPC %-16s (alias for preferred %s CPU)\n",
                        alias->alias, family->desc);
        } else {
            qemu_printf("PowerPC %-16s (alias for %s)\n",
                        alias->alias, name);
        }
    }
    g_free(name);
}

void ppc_cpu_list(void)
{
    GSList *list;

    list = object_class_get_list(TYPE_POWERPC_CPU, false);
    list = g_slist_sort(list, ppc_cpu_list_compare);
    g_slist_foreach(list, ppc_cpu_list_entry, NULL);
    g_slist_free(list);

#ifdef CONFIG_KVM
    qemu_printf("\n");
    qemu_printf("PowerPC %-16s\n", "host");
#endif
}

static void ppc_cpu_defs_entry(gpointer data, gpointer user_data)
{
    ObjectClass *oc = data;
    CpuDefinitionInfoList **first = user_data;
    const char *typename;
    CpuDefinitionInfo *info;

    typename = object_class_get_name(oc);
    info = g_malloc0(sizeof(*info));
    info->name = g_strndup(typename,
                           strlen(typename) - strlen(POWERPC_CPU_TYPE_SUFFIX));

    QAPI_LIST_PREPEND(*first, info);
}

CpuDefinitionInfoList *qmp_query_cpu_definitions(Error **errp)
{
    CpuDefinitionInfoList *cpu_list = NULL;
    GSList *list;
    int i;

    list = object_class_get_list(TYPE_POWERPC_CPU, false);
    g_slist_foreach(list, ppc_cpu_defs_entry, &cpu_list);
    g_slist_free(list);

    for (i = 0; ppc_cpu_aliases[i].alias != NULL; i++) {
        PowerPCCPUAlias *alias = &ppc_cpu_aliases[i];
        ObjectClass *oc;
        CpuDefinitionInfo *info;

        oc = ppc_cpu_class_by_name(alias->model);
        if (oc == NULL) {
            continue;
        }

        info = g_malloc0(sizeof(*info));
        info->name = g_strdup(alias->alias);
        info->q_typename = g_strdup(object_class_get_name(oc));

        QAPI_LIST_PREPEND(cpu_list, info);
    }

    return cpu_list;
}

static void ppc_cpu_set_pc(CPUState *cs, vaddr value)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);

    cpu->env.nip = value;
}

static bool ppc_cpu_has_work(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;

    return msr_ee && (cs->interrupt_request & CPU_INTERRUPT_HARD);
}

static void ppc_cpu_reset(DeviceState *dev)
{
    CPUState *s = CPU(dev);
    PowerPCCPU *cpu = POWERPC_CPU(s);
    PowerPCCPUClass *pcc = POWERPC_CPU_GET_CLASS(cpu);
    CPUPPCState *env = &cpu->env;
    target_ulong msr;
    int i;

    pcc->parent_reset(dev);

    msr = (target_ulong)0;
    msr |= (target_ulong)MSR_HVB;
    msr |= (target_ulong)0 << MSR_AP; /* TO BE CHECKED */
    msr |= (target_ulong)0 << MSR_SA; /* TO BE CHECKED */
    msr |= (target_ulong)1 << MSR_EP;
#if defined(DO_SINGLE_STEP) && 0
    /* Single step trace mode */
    msr |= (target_ulong)1 << MSR_SE;
    msr |= (target_ulong)1 << MSR_BE;
#endif
#if defined(CONFIG_USER_ONLY)
    msr |= (target_ulong)1 << MSR_FP; /* Allow floating point usage */
    msr |= (target_ulong)1 << MSR_FE0; /* Allow floating point exceptions */
    msr |= (target_ulong)1 << MSR_FE1;
    msr |= (target_ulong)1 << MSR_VR; /* Allow altivec usage */
    msr |= (target_ulong)1 << MSR_VSX; /* Allow VSX usage */
    msr |= (target_ulong)1 << MSR_SPE; /* Allow SPE usage */
    msr |= (target_ulong)1 << MSR_PR;
#if defined(TARGET_PPC64)
    msr |= (target_ulong)1 << MSR_TM; /* Transactional memory */
#endif
#if !defined(TARGET_WORDS_BIGENDIAN)
    msr |= (target_ulong)1 << MSR_LE; /* Little-endian user mode */
    if (!((env->msr_mask >> MSR_LE) & 1)) {
        fprintf(stderr, "Selected CPU does not support little-endian.\n");
        exit(1);
    }
#endif
#endif

#if defined(TARGET_PPC64)
    if (mmu_is_64bit(env->mmu_model)) {
        msr |= (1ULL << MSR_SF);
    }
#endif

    hreg_store_msr(env, msr, 1);

#if !defined(CONFIG_USER_ONLY)
    env->nip = env->hreset_vector | env->excp_prefix;
    if (env->mmu_model != POWERPC_MMU_REAL) {
        ppc_tlb_invalidate_all(env);
    }
#endif

    hreg_compute_hflags(env);
    env->reserve_addr = (target_ulong)-1ULL;
    /* Be sure no exception or interrupt is pending */
    env->pending_interrupts = 0;
    s->exception_index = POWERPC_EXCP_NONE;
    env->error_code = 0;
    ppc_irq_reset(cpu);

    /* tininess for underflow is detected before rounding */
    set_float_detect_tininess(float_tininess_before_rounding,
                              &env->fp_status);

    for (i = 0; i < ARRAY_SIZE(env->spr_cb); i++) {
        ppc_spr_t *spr = &env->spr_cb[i];

        if (!spr->name) {
            continue;
        }
        env->spr[i] = spr->default_value;
    }
}

#ifndef CONFIG_USER_ONLY

static bool ppc_cpu_is_big_endian(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;

    cpu_synchronize_state(cs);

    return !msr_le;
}

#ifdef CONFIG_TCG
static void ppc_cpu_exec_enter(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);

    if (cpu->vhyp) {
        PPCVirtualHypervisorClass *vhc =
            PPC_VIRTUAL_HYPERVISOR_GET_CLASS(cpu->vhyp);
        vhc->cpu_exec_enter(cpu->vhyp, cpu);
    }
}

static void ppc_cpu_exec_exit(CPUState *cs)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);

    if (cpu->vhyp) {
        PPCVirtualHypervisorClass *vhc =
            PPC_VIRTUAL_HYPERVISOR_GET_CLASS(cpu->vhyp);
        vhc->cpu_exec_exit(cpu->vhyp, cpu);
    }
}
#endif /* CONFIG_TCG */

#endif /* !CONFIG_USER_ONLY */

static void ppc_cpu_instance_init(Object *obj)
{
    PowerPCCPU *cpu = POWERPC_CPU(obj);
    PowerPCCPUClass *pcc = POWERPC_CPU_GET_CLASS(cpu);
    CPUPPCState *env = &cpu->env;

    cpu_set_cpustate_pointers(cpu);
    cpu->vcpu_id = UNASSIGNED_CPU_INDEX;

    env->msr_mask = pcc->msr_mask;
    env->mmu_model = pcc->mmu_model;
    env->excp_model = pcc->excp_model;
    env->bus_model = pcc->bus_model;
    env->insns_flags = pcc->insns_flags;
    env->insns_flags2 = pcc->insns_flags2;
    env->flags = pcc->flags;
    env->bfd_mach = pcc->bfd_mach;
    env->check_pow = pcc->check_pow;

    /*
     * Mark HV mode as supported if the CPU has an MSR_HV bit in the
     * msr_mask. The mask can later be cleared by PAPR mode but the hv
     * mode support will remain, thus enforcing that we cannot use
     * priv. instructions in guest in PAPR mode. For 970 we currently
     * simply don't set HV in msr_mask thus simulating an "Apple mode"
     * 970. If we ever want to support 970 HV mode, we'll have to add
     * a processor attribute of some sort.
     */
#if !defined(CONFIG_USER_ONLY)
    env->has_hv_mode = !!(env->msr_mask & MSR_HVB);
#endif

    ppc_hash64_init(cpu);
}

static void ppc_cpu_instance_finalize(Object *obj)
{
    PowerPCCPU *cpu = POWERPC_CPU(obj);

    ppc_hash64_finalize(cpu);
}

static bool ppc_pvr_match_default(PowerPCCPUClass *pcc, uint32_t pvr)
{
    return pcc->pvr == pvr;
}

static void ppc_disas_set_info(CPUState *cs, disassemble_info *info)
{
    PowerPCCPU *cpu = POWERPC_CPU(cs);
    CPUPPCState *env = &cpu->env;

    if ((env->hflags >> MSR_LE) & 1) {
        info->endian = BFD_ENDIAN_LITTLE;
    }
    info->mach = env->bfd_mach;
    if (!env->bfd_mach) {
#ifdef TARGET_PPC64
        info->mach = bfd_mach_ppc64;
#else
        info->mach = bfd_mach_ppc;
#endif
    }
    info->disassembler_options = (char *)"any";
    info->print_insn = print_insn_ppc;

    info->cap_arch = CS_ARCH_PPC;
#ifdef TARGET_PPC64
    info->cap_mode = CS_MODE_64;
#endif
}

static Property ppc_cpu_properties[] = {
    DEFINE_PROP_BOOL("pre-2.8-migration", PowerPCCPU, pre_2_8_migration, false),
    DEFINE_PROP_BOOL("pre-2.10-migration", PowerPCCPU, pre_2_10_migration,
                     false),
    DEFINE_PROP_BOOL("pre-3.0-migration", PowerPCCPU, pre_3_0_migration,
                     false),
    DEFINE_PROP_END_OF_LIST(),
};

#ifdef CONFIG_TCG
#include "hw/core/tcg-cpu-ops.h"

static struct TCGCPUOps ppc_tcg_ops = {
  .initialize = ppc_translate_init,
  .cpu_exec_interrupt = ppc_cpu_exec_interrupt,
  .tlb_fill = ppc_cpu_tlb_fill,

#ifndef CONFIG_USER_ONLY
  .do_interrupt = ppc_cpu_do_interrupt,
  .cpu_exec_enter = ppc_cpu_exec_enter,
  .cpu_exec_exit = ppc_cpu_exec_exit,
  .do_unaligned_access = ppc_cpu_do_unaligned_access,
#endif /* !CONFIG_USER_ONLY */
};
#endif /* CONFIG_TCG */

static void ppc_cpu_class_init(ObjectClass *oc, void *data)
{
    PowerPCCPUClass *pcc = POWERPC_CPU_CLASS(oc);
    CPUClass *cc = CPU_CLASS(oc);
    DeviceClass *dc = DEVICE_CLASS(oc);

    device_class_set_parent_realize(dc, ppc_cpu_realize,
                                    &pcc->parent_realize);
    device_class_set_parent_unrealize(dc, ppc_cpu_unrealize,
                                      &pcc->parent_unrealize);
    pcc->pvr_match = ppc_pvr_match_default;
    pcc->interrupts_big_endian = ppc_cpu_interrupts_big_endian_always;
    device_class_set_props(dc, ppc_cpu_properties);

    device_class_set_parent_reset(dc, ppc_cpu_reset, &pcc->parent_reset);

    cc->class_by_name = ppc_cpu_class_by_name;
    cc->has_work = ppc_cpu_has_work;
    cc->dump_state = ppc_cpu_dump_state;
    cc->dump_statistics = ppc_cpu_dump_statistics;
    cc->set_pc = ppc_cpu_set_pc;
    cc->gdb_read_register = ppc_cpu_gdb_read_register;
    cc->gdb_write_register = ppc_cpu_gdb_write_register;
#ifndef CONFIG_USER_ONLY
    cc->get_phys_page_debug = ppc_cpu_get_phys_page_debug;
    cc->vmsd = &vmstate_ppc_cpu;
#endif
#if defined(CONFIG_SOFTMMU)
    cc->write_elf64_note = ppc64_cpu_write_elf64_note;
    cc->write_elf32_note = ppc32_cpu_write_elf32_note;
#endif

    cc->gdb_num_core_regs = 71;
#ifndef CONFIG_USER_ONLY
    cc->gdb_get_dynamic_xml = ppc_gdb_get_dynamic_xml;
#endif
#ifdef USE_APPLE_GDB
    cc->gdb_read_register = ppc_cpu_gdb_read_register_apple;
    cc->gdb_write_register = ppc_cpu_gdb_write_register_apple;
    cc->gdb_num_core_regs = 71 + 32;
#endif

    cc->gdb_arch_name = ppc_gdb_arch_name;
#if defined(TARGET_PPC64)
    cc->gdb_core_xml_file = "power64-core.xml";
#else
    cc->gdb_core_xml_file = "power-core.xml";
#endif
#ifndef CONFIG_USER_ONLY
    cc->virtio_is_big_endian = ppc_cpu_is_big_endian;
#endif
    cc->disas_set_info = ppc_disas_set_info;

    dc->fw_name = "PowerPC,UNKNOWN";

#ifdef CONFIG_TCG
    cc->tcg_ops = &ppc_tcg_ops;
#endif /* CONFIG_TCG */
}

static const TypeInfo ppc_cpu_type_info = {
    .name = TYPE_POWERPC_CPU,
    .parent = TYPE_CPU,
    .instance_size = sizeof(PowerPCCPU),
    .instance_align = __alignof__(PowerPCCPU),
    .instance_init = ppc_cpu_instance_init,
    .instance_finalize = ppc_cpu_instance_finalize,
    .abstract = true,
    .class_size = sizeof(PowerPCCPUClass),
    .class_init = ppc_cpu_class_init,
};

#ifndef CONFIG_USER_ONLY
static const TypeInfo ppc_vhyp_type_info = {
    .name = TYPE_PPC_VIRTUAL_HYPERVISOR,
    .parent = TYPE_INTERFACE,
    .class_size = sizeof(PPCVirtualHypervisorClass),
};
#endif

static void ppc_cpu_register_types(void)
{
    type_register_static(&ppc_cpu_type_info);
#ifndef CONFIG_USER_ONLY
    type_register_static(&ppc_vhyp_type_info);
#endif
}

type_init(ppc_cpu_register_types)
