/*
 *  PowerPC internal definitions for qemu.
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

#ifndef PPC_INTERNAL_H
#define PPC_INTERNAL_H

#define FUNC_MASK(name, ret_type, size, max_val)                  \
static inline ret_type name(uint##size##_t start,                 \
                              uint##size##_t end)                 \
{                                                                 \
    ret_type ret, max_bit = size - 1;                             \
                                                                  \
    if (likely(start == 0)) {                                     \
        ret = max_val << (max_bit - end);                         \
    } else if (likely(end == max_bit)) {                          \
        ret = max_val >> start;                                   \
    } else {                                                      \
        ret = (((uint##size##_t)(-1ULL)) >> (start)) ^            \
            (((uint##size##_t)(-1ULL) >> (end)) >> 1);            \
        if (unlikely(start > end)) {                              \
            return ~ret;                                          \
        }                                                         \
    }                                                             \
                                                                  \
    return ret;                                                   \
}

#if defined(TARGET_PPC64)
FUNC_MASK(MASK, target_ulong, 64, UINT64_MAX);
#else
FUNC_MASK(MASK, target_ulong, 32, UINT32_MAX);
#endif
FUNC_MASK(mask_u32, uint32_t, 32, UINT32_MAX);
FUNC_MASK(mask_u64, uint64_t, 64, UINT64_MAX);

/*****************************************************************************/
/***                           Instruction decoding                        ***/
#define EXTRACT_HELPER(name, shift, nb)                                       \
static inline uint32_t name(uint32_t opcode)                                  \
{                                                                             \
    return extract32(opcode, shift, nb);                                      \
}

#define EXTRACT_SHELPER(name, shift, nb)                                      \
static inline int32_t name(uint32_t opcode)                                   \
{                                                                             \
    return sextract32(opcode, shift, nb);                                     \
}

#define EXTRACT_HELPER_SPLIT(name, shift1, nb1, shift2, nb2)                  \
static inline uint32_t name(uint32_t opcode)                                  \
{                                                                             \
    return extract32(opcode, shift1, nb1) << nb2 |                            \
               extract32(opcode, shift2, nb2);                                \
}

#define EXTRACT_HELPER_SPLIT_3(name,                                          \
                              d0_bits, shift_op_d0, shift_d0,                 \
                              d1_bits, shift_op_d1, shift_d1,                 \
                              d2_bits, shift_op_d2, shift_d2)                 \
static inline int16_t name(uint32_t opcode)                                   \
{                                                                             \
    return                                                                    \
        (((opcode >> (shift_op_d0)) & ((1 << (d0_bits)) - 1)) << (shift_d0)) | \
        (((opcode >> (shift_op_d1)) & ((1 << (d1_bits)) - 1)) << (shift_d1)) | \
        (((opcode >> (shift_op_d2)) & ((1 << (d2_bits)) - 1)) << (shift_d2));  \
}


/* Opcode part 1 */
EXTRACT_HELPER(opc1, 26, 6);
/* Opcode part 2 */
EXTRACT_HELPER(opc2, 1, 5);
/* Opcode part 3 */
EXTRACT_HELPER(opc3, 6, 5);
/* Opcode part 4 */
EXTRACT_HELPER(opc4, 16, 5);
/* Update Cr0 flags */
EXTRACT_HELPER(Rc, 0, 1);
/* Update Cr6 flags (Altivec) */
EXTRACT_HELPER(Rc21, 10, 1);
/* Destination */
EXTRACT_HELPER(rD, 21, 5);
/* Source */
EXTRACT_HELPER(rS, 21, 5);
/* First operand */
EXTRACT_HELPER(rA, 16, 5);
/* Second operand */
EXTRACT_HELPER(rB, 11, 5);
/* Third operand */
EXTRACT_HELPER(rC, 6, 5);
/***                               Get CRn                                 ***/
EXTRACT_HELPER(crfD, 23, 3);
EXTRACT_HELPER(BF, 23, 3);
EXTRACT_HELPER(crfS, 18, 3);
EXTRACT_HELPER(crbD, 21, 5);
EXTRACT_HELPER(crbA, 16, 5);
EXTRACT_HELPER(crbB, 11, 5);
/* SPR / TBL */
EXTRACT_HELPER(_SPR, 11, 10);
static inline uint32_t SPR(uint32_t opcode)
{
    uint32_t sprn = _SPR(opcode);

    return ((sprn >> 5) & 0x1F) | ((sprn & 0x1F) << 5);
}
/***                              Get constants                            ***/
/* 16 bits signed immediate value */
EXTRACT_SHELPER(SIMM, 0, 16);
/* 16 bits unsigned immediate value */
EXTRACT_HELPER(UIMM, 0, 16);
/* 5 bits signed immediate value */
EXTRACT_SHELPER(SIMM5, 16, 5);
/* 5 bits signed immediate value */
EXTRACT_HELPER(UIMM5, 16, 5);
/* 4 bits unsigned immediate value */
EXTRACT_HELPER(UIMM4, 16, 4);
/* Bit count */
EXTRACT_HELPER(NB, 11, 5);
/* Shift count */
EXTRACT_HELPER(SH, 11, 5);
/* lwat/stwat/ldat/lwat */
EXTRACT_HELPER(FC, 11, 5);
/* Vector shift count */
EXTRACT_HELPER(VSH, 6, 4);
/* Mask start */
EXTRACT_HELPER(MB, 6, 5);
/* Mask end */
EXTRACT_HELPER(ME, 1, 5);
/* Trap operand */
EXTRACT_HELPER(TO, 21, 5);

EXTRACT_HELPER(CRM, 12, 8);

#ifndef CONFIG_USER_ONLY
EXTRACT_HELPER(SR, 16, 4);
#endif

/* mtfsf/mtfsfi */
EXTRACT_HELPER(FPBF, 23, 3);
EXTRACT_HELPER(FPIMM, 12, 4);
EXTRACT_HELPER(FPL, 25, 1);
EXTRACT_HELPER(FPFLM, 17, 8);
EXTRACT_HELPER(FPW, 16, 1);

/* mffscrni */
EXTRACT_HELPER(RM, 11, 2);

/* addpcis */
EXTRACT_HELPER_SPLIT_3(DX, 10, 6, 6, 5, 16, 1, 1, 0, 0)
#if defined(TARGET_PPC64)
/* darn */
EXTRACT_HELPER(L, 16, 2);
#endif

/***                            Jump target decoding                       ***/
/* Immediate address */
static inline target_ulong LI(uint32_t opcode)
{
    return (opcode >> 0) & 0x03FFFFFC;
}

static inline uint32_t BD(uint32_t opcode)
{
    return (opcode >> 0) & 0xFFFC;
}

EXTRACT_HELPER(BO, 21, 5);
EXTRACT_HELPER(BI, 16, 5);
/* Absolute/relative address */
EXTRACT_HELPER(AA, 1, 1);
/* Link */
EXTRACT_HELPER(LK, 0, 1);

/* DFP Z22-form */
EXTRACT_HELPER(DCM, 10, 6)

/* DFP Z23-form */
EXTRACT_HELPER(RMC, 9, 2)
EXTRACT_HELPER(Rrm, 16, 1)

EXTRACT_HELPER_SPLIT(DQxT, 3, 1, 21, 5);
EXTRACT_HELPER_SPLIT(xT, 0, 1, 21, 5);
EXTRACT_HELPER_SPLIT(xS, 0, 1, 21, 5);
EXTRACT_HELPER_SPLIT(xA, 2, 1, 16, 5);
EXTRACT_HELPER_SPLIT(xB, 1, 1, 11, 5);
EXTRACT_HELPER_SPLIT(xC, 3, 1,  6, 5);
EXTRACT_HELPER(DM, 8, 2);
EXTRACT_HELPER(UIM, 16, 2);
EXTRACT_HELPER(SHW, 8, 2);
EXTRACT_HELPER(SP, 19, 2);
EXTRACT_HELPER(IMM8, 11, 8);
EXTRACT_HELPER(DCMX, 16, 7);
EXTRACT_HELPER_SPLIT_3(DCMX_XV, 5, 16, 0, 1, 2, 5, 1, 6, 6);

void helper_compute_fprf_float16(CPUPPCState *env, float16 arg);
void helper_compute_fprf_float32(CPUPPCState *env, float32 arg);
void helper_compute_fprf_float128(CPUPPCState *env, float128 arg);

/* Raise a data fault alignment exception for the specified virtual address */
void ppc_cpu_do_unaligned_access(CPUState *cs, vaddr addr,
                                 MMUAccessType access_type,
                                 int mmu_idx, uintptr_t retaddr);

/* translate.c */

int ppc_fixup_cpu(PowerPCCPU *cpu);
void create_ppc_opcodes(PowerPCCPU *cpu, Error **errp);
void destroy_ppc_opcodes(PowerPCCPU *cpu);

/* gdbstub.c */
void ppc_gdb_init(CPUState *cs, PowerPCCPUClass *ppc);
gchar *ppc_gdb_arch_name(CPUState *cs);

/* spr-common.c */
#include "cpu.h"
void gen_spr_generic(CPUPPCState *env);
void gen_spr_ne_601(CPUPPCState *env);
void gen_spr_sdr1(CPUPPCState *env);
void gen_low_BATs(CPUPPCState *env);
void gen_high_BATs(CPUPPCState *env);
void gen_tbl(CPUPPCState *env);
void gen_6xx_7xx_soft_tlb(CPUPPCState *env, int nb_tlbs, int nb_ways);
void gen_spr_G2_755(CPUPPCState *env);
void gen_spr_7xx(CPUPPCState *env);
#ifdef TARGET_PPC64
void gen_spr_amr(CPUPPCState *env);
void gen_spr_iamr(CPUPPCState *env);
#endif /* TARGET_PPC64 */
void gen_spr_thrm(CPUPPCState *env);
void gen_spr_604(CPUPPCState *env);
void gen_spr_603(CPUPPCState *env);
void gen_spr_G2(CPUPPCState *env);
void gen_spr_602(CPUPPCState *env);
void gen_spr_601(CPUPPCState *env);
void gen_spr_74xx(CPUPPCState *env);
void gen_l3_ctrl(CPUPPCState *env);
void gen_74xx_soft_tlb(CPUPPCState *env, int nb_tlbs, int nb_ways);
void gen_spr_not_implemented(CPUPPCState *env,
                             int num, const char *name);
void gen_spr_not_implemented_ureg(CPUPPCState *env,
                                  int num, const char *name);
void gen_spr_not_implemented_no_write(CPUPPCState *env,
                                      int num, const char *name);
void gen_spr_not_implemented_write_nop(CPUPPCState *env,
                                       int num, const char *name);
void gen_spr_PSSCR(CPUPPCState *env);
void gen_spr_TIDR(CPUPPCState *env);
void gen_spr_pvr(CPUPPCState *env, PowerPCCPUClass *pcc);
void gen_spr_svr(CPUPPCState *env, PowerPCCPUClass *pcc);
void gen_spr_pir(CPUPPCState *env);
void gen_spr_spefscr(CPUPPCState *env);
void gen_spr_l1fgc(CPUPPCState *env, int num, int initial_value);
void gen_spr_hid0(CPUPPCState *env);
void gen_spr_mas73(CPUPPCState *env);
void gen_spr_mmucsr0(CPUPPCState *env);
void gen_spr_l1csr0(CPUPPCState *env);
void gen_spr_l1csr1(CPUPPCState *env);
void gen_spr_l2csr0(CPUPPCState *env);
void gen_spr_usprg3(CPUPPCState *env);
void gen_spr_usprgh(CPUPPCState *env);
void gen_spr_BookE(CPUPPCState *env, uint64_t ivor_mask);
uint32_t gen_tlbncfg(uint32_t assoc, uint32_t minsize,
                     uint32_t maxsize, uint32_t flags,
                     uint32_t nentries);
void gen_spr_BookE206(CPUPPCState *env, uint32_t mas_mask,
                             uint32_t *tlbncfg, uint32_t mmucfg);
void gen_spr_440(CPUPPCState *env);
void gen_spr_440_misc(CPUPPCState *env);
void gen_spr_40x(CPUPPCState *env);
void gen_spr_405(CPUPPCState *env);
void gen_spr_401_403(CPUPPCState *env);
void gen_spr_401(CPUPPCState *env);
void gen_spr_401x2(CPUPPCState *env);
void gen_spr_403(CPUPPCState *env);
void gen_spr_403_real(CPUPPCState *env);
void gen_spr_403_mmu(CPUPPCState *env);
void gen_spr_40x_bus_control(CPUPPCState *env);
void gen_spr_compress(CPUPPCState *env);
void gen_spr_5xx_8xx(CPUPPCState *env);
void gen_spr_5xx(CPUPPCState *env);
void gen_spr_8xx(CPUPPCState *env);
void gen_spr_970_hid(CPUPPCState *env);
void gen_spr_970_hior(CPUPPCState *env);
void gen_spr_book3s_ctrl(CPUPPCState *env);
void gen_spr_book3s_altivec(CPUPPCState *env);
void gen_spr_book3s_dbg(CPUPPCState *env);
void gen_spr_book3s_207_dbg(CPUPPCState *env);
void gen_spr_970_dbg(CPUPPCState *env);
void gen_spr_book3s_pmu_sup(CPUPPCState *env);
void gen_spr_book3s_pmu_user(CPUPPCState *env);
void gen_spr_970_pmu_sup(CPUPPCState *env);
void gen_spr_970_pmu_user(CPUPPCState *env);
void gen_spr_power8_pmu_sup(CPUPPCState *env);
void gen_spr_power8_pmu_user(CPUPPCState *env);
void gen_spr_power5p_ear(CPUPPCState *env);
void gen_spr_power5p_tb(CPUPPCState *env);
void gen_spr_970_lpar(CPUPPCState *env);
void gen_spr_power5p_lpar(CPUPPCState *env);
void gen_spr_book3s_ids(CPUPPCState *env);
void gen_spr_rmor(CPUPPCState *env);
void gen_spr_power8_ids(CPUPPCState *env);
void gen_spr_book3s_purr(CPUPPCState *env);
void gen_spr_power6_dbg(CPUPPCState *env);
void gen_spr_power5p_common(CPUPPCState *env);
void gen_spr_power6_common(CPUPPCState *env);
void gen_spr_power8_tce_address_control(CPUPPCState *env);
void gen_spr_power8_tm(CPUPPCState *env);
void gen_spr_power8_ebb(CPUPPCState *env);
void gen_spr_vtb(CPUPPCState *env);
void gen_spr_power8_fscr(CPUPPCState *env);
void gen_spr_power8_pspb(CPUPPCState *env);
void gen_spr_power8_dpdes(CPUPPCState *env);
void gen_spr_power8_ic(CPUPPCState *env);
void gen_spr_power8_book4(CPUPPCState *env);
void gen_spr_power7_book4(CPUPPCState *env);
void gen_spr_power8_rpr(CPUPPCState *env);
void gen_spr_power9_mmu(CPUPPCState *env);
/* TODO: find better solution for gen_op_mfspr and gen_op_mtspr */
void spr_noaccess(DisasContext *ctx, int gprn, int sprn);
#define SPR_NOACCESS (&spr_noaccess)

#endif /* PPC_INTERNAL_H */
