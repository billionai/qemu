#include "exec/translator.h"
#include "spr_tcg.h"

#ifdef TARGET_PPC64
void gen_fscr_facility_check(DisasContext *ctx, int facility_sprn,
                                    int bit, int sprn, int cause)
{
    TCGv_i32 t1 = tcg_const_i32(bit);
    TCGv_i32 t2 = tcg_const_i32(sprn);
    TCGv_i32 t3 = tcg_const_i32(cause);

    gen_helper_fscr_facility_check(cpu_env, t1, t2, t3);

    tcg_temp_free_i32(t3);
    tcg_temp_free_i32(t2);
    tcg_temp_free_i32(t1);
}

void gen_msr_facility_check(DisasContext *ctx, int facility_sprn,
                                   int bit, int sprn, int cause)
{
    TCGv_i32 t1 = tcg_const_i32(bit);
    TCGv_i32 t2 = tcg_const_i32(sprn);
    TCGv_i32 t3 = tcg_const_i32(cause);

    gen_helper_msr_facility_check(cpu_env, t1, t2, t3);

    tcg_temp_free_i32(t3);
    tcg_temp_free_i32(t2);
    tcg_temp_free_i32(t1);
}
#endif
/*****************************************************************************/
/* Reader and writer functions for SPRs */

void spr_noaccess(DisasContext *ctx, int gprn, int sprn)
{
#if 0
    sprn = ((sprn >> 5) & 0x1F) | ((sprn & 0x1F) << 5);
    printf("ERROR: try to access SPR %d !\n", sprn);
#endif
}

/*
 * Generic callbacks:
 * do nothing but store/retrieve spr value
 */
void spr_load_dump_spr(int sprn)
{
#ifdef PPC_DUMP_SPR_ACCESSES
    TCGv_i32 t0 = tcg_const_i32(sprn);
    gen_helper_load_dump_spr(cpu_env, t0);
    tcg_temp_free_i32(t0);
#endif
}

void spr_read_generic(DisasContext *ctx, int gprn, int sprn)
{
    gen_load_spr(cpu_gpr[gprn], sprn);
    spr_load_dump_spr(sprn);
}

void spr_store_dump_spr(int sprn)
{
#ifdef PPC_DUMP_SPR_ACCESSES
    TCGv_i32 t0 = tcg_const_i32(sprn);
    gen_helper_store_dump_spr(cpu_env, t0);
    tcg_temp_free_i32(t0);
#endif
}

void spr_write_generic(DisasContext *ctx, int sprn, int gprn)
{
    gen_store_spr(sprn, cpu_gpr[gprn]);
    spr_store_dump_spr(sprn);
}

/* SPR common to all PowerPC */
/* XER */

/* I really see no reason to keep these gen_*_xer */
/* instead of just leaving the code in the spr_*_xer */
void gen_read_xer(DisasContext *ctx, TCGv dst)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();
    tcg_gen_mov_tl(dst, cpu_xer);
    tcg_gen_shli_tl(t0, cpu_so, XER_SO);
    tcg_gen_shli_tl(t1, cpu_ov, XER_OV);
    tcg_gen_shli_tl(t2, cpu_ca, XER_CA);
    tcg_gen_or_tl(t0, t0, t1);
    tcg_gen_or_tl(dst, dst, t2);
    tcg_gen_or_tl(dst, dst, t0);
    if (is_isa300(ctx)) {
        tcg_gen_shli_tl(t0, cpu_ov32, XER_OV32);
        tcg_gen_or_tl(dst, dst, t0);
        tcg_gen_shli_tl(t0, cpu_ca32, XER_CA32);
        tcg_gen_or_tl(dst, dst, t0);
    }
    tcg_temp_free(t0);
    tcg_temp_free(t1);
    tcg_temp_free(t2);
}

void gen_write_xer(TCGv src)
{
    /* Write all flags, while reading back check for isa300 */
    tcg_gen_andi_tl(cpu_xer, src,
                    ~((1u << XER_SO) |
                      (1u << XER_OV) | (1u << XER_OV32) |
                      (1u << XER_CA) | (1u << XER_CA32)));
    tcg_gen_extract_tl(cpu_ov32, src, XER_OV32, 1);
    tcg_gen_extract_tl(cpu_ca32, src, XER_CA32, 1);
    tcg_gen_extract_tl(cpu_so, src, XER_SO, 1);
    tcg_gen_extract_tl(cpu_ov, src, XER_OV, 1);
    tcg_gen_extract_tl(cpu_ca, src, XER_CA, 1);
}

void spr_read_xer(DisasContext *ctx, int gprn, int sprn)
{
    gen_read_xer(ctx, cpu_gpr[gprn]);
}

void spr_write_xer(DisasContext *ctx, int sprn, int gprn)
{
    gen_write_xer(cpu_gpr[gprn]);
}

/* LR */
void spr_read_lr(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_mov_tl(cpu_gpr[gprn], cpu_lr);
}

void spr_write_lr(DisasContext *ctx, int sprn, int gprn)
{
    tcg_gen_mov_tl(cpu_lr, cpu_gpr[gprn]);
}

/* CTR */
void spr_read_ctr(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_mov_tl(cpu_gpr[gprn], cpu_ctr);
}

void spr_write_ctr(DisasContext *ctx, int sprn, int gprn)
{
    tcg_gen_mov_tl(cpu_ctr, cpu_gpr[gprn]);
}

/* User read access to SPR */
/* USPRx */
/* UMMCRx */
/* UPMCx */
/* USIA */
/* UDECR */
void spr_read_ureg(DisasContext *ctx, int gprn, int sprn)
{
    gen_load_spr(cpu_gpr[gprn], sprn + 0x10);
}

/* SPR common to all non-embedded PowerPC, except 601 */
/* Time base */
void spr_read_tbl(DisasContext *ctx, int gprn, int sprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_load_tbl(cpu_gpr[gprn], cpu_env);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_end();
        gen_stop_exception(ctx);
    }
}

void spr_read_tbu(DisasContext *ctx, int gprn, int sprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_load_tbu(cpu_gpr[gprn], cpu_env);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_end();
        gen_stop_exception(ctx);
    }
}

ATTRIBUTE_UNUSED
void spr_read_atbl(DisasContext *ctx, int gprn, int sprn)
{
    gen_helper_load_atbl(cpu_gpr[gprn], cpu_env);
}

ATTRIBUTE_UNUSED
void spr_read_atbu(DisasContext *ctx, int gprn, int sprn)
{
    gen_helper_load_atbu(cpu_gpr[gprn], cpu_env);
}

/* PowerPC 601 specific registers */
/* RTC */
void spr_read_601_rtcl(DisasContext *ctx, int gprn, int sprn)
{
    gen_helper_load_601_rtcl(cpu_gpr[gprn], cpu_env);
}

void spr_read_601_rtcu(DisasContext *ctx, int gprn, int sprn)
{
    gen_helper_load_601_rtcu(cpu_gpr[gprn], cpu_env);
}

/* SPE specific registers */
void spr_read_spefscr(DisasContext *ctx, int gprn, int sprn)
{
    TCGv_i32 t0 = tcg_temp_new_i32();
    tcg_gen_ld_i32(t0, cpu_env, offsetof(CPUPPCState, spe_fscr));
    tcg_gen_extu_i32_tl(cpu_gpr[gprn], t0);
    tcg_temp_free_i32(t0);
}

void spr_write_spefscr(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_temp_new_i32();
    tcg_gen_trunc_tl_i32(t0, cpu_gpr[gprn]);
    tcg_gen_st_i32(t0, cpu_env, offsetof(CPUPPCState, spe_fscr));
    tcg_temp_free_i32(t0);
}

#ifndef CONFIG_USER_ONLY
void spr_write_generic32(DisasContext *ctx, int sprn, int gprn)
{
#ifdef TARGET_PPC64
    TCGv t0 = tcg_temp_new();
    tcg_gen_ext32u_tl(t0, cpu_gpr[gprn]);
    gen_store_spr(sprn, t0);
    tcg_temp_free(t0);
    spr_store_dump_spr(sprn);
#else
    spr_write_generic(ctx, sprn, gprn);
#endif
}

void spr_write_clear(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    gen_load_spr(t0, sprn);
    tcg_gen_neg_tl(t1, cpu_gpr[gprn]);
    tcg_gen_and_tl(t0, t0, t1);
    gen_store_spr(sprn, t0);
    tcg_temp_free(t0);
    tcg_temp_free(t1);
}

void spr_access_nop(DisasContext *ctx, int sprn, int gprn)
{
}

void spr_read_decr(DisasContext *ctx, int gprn, int sprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_load_decr(cpu_gpr[gprn], cpu_env);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_decr(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_decr(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_tbl(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_tbl(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_end();
        gen_stop_exception(ctx);
    }
}

void spr_write_tbu(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_tbu(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_end();
        gen_stop_exception(ctx);
    }
}

ATTRIBUTE_UNUSED
void spr_write_atbl(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_atbl(cpu_env, cpu_gpr[gprn]);
}

ATTRIBUTE_UNUSED
void spr_write_atbu(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_atbu(cpu_env, cpu_gpr[gprn]);
}

/* IBAT0U...IBAT0U */
/* IBAT0L...IBAT7L */
void spr_read_ibat(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_ld_tl(cpu_gpr[gprn], cpu_env,
                  offsetof(CPUPPCState,
                           IBAT[sprn & 1][(sprn - SPR_IBAT0U) / 2]));
}

void spr_read_ibat_h(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_ld_tl(cpu_gpr[gprn], cpu_env,
                  offsetof(CPUPPCState,
                           IBAT[sprn & 1][((sprn - SPR_IBAT4U) / 2) + 4]));
}

void spr_write_ibatu(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32((sprn - SPR_IBAT0U) / 2);
    gen_helper_store_ibatu(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_ibatu_h(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32(((sprn - SPR_IBAT4U) / 2) + 4);
    gen_helper_store_ibatu(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_ibatl(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32((sprn - SPR_IBAT0L) / 2);
    gen_helper_store_ibatl(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_ibatl_h(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32(((sprn - SPR_IBAT4L) / 2) + 4);
    gen_helper_store_ibatl(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

/* DBAT0U...DBAT7U */
/* DBAT0L...DBAT7L */
void spr_read_dbat(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_ld_tl(cpu_gpr[gprn], cpu_env,
                  offsetof(CPUPPCState,
                           DBAT[sprn & 1][(sprn - SPR_DBAT0U) / 2]));
}

void spr_read_dbat_h(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_ld_tl(cpu_gpr[gprn], cpu_env,
                  offsetof(CPUPPCState,
                           DBAT[sprn & 1][((sprn - SPR_DBAT4U) / 2) + 4]));
}

void spr_write_dbatu(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32((sprn - SPR_DBAT0U) / 2);
    gen_helper_store_dbatu(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_dbatu_h(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32(((sprn - SPR_DBAT4U) / 2) + 4);
    gen_helper_store_dbatu(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_dbatl(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32((sprn - SPR_DBAT0L) / 2);
    gen_helper_store_dbatl(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_dbatl_h(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32(((sprn - SPR_DBAT4L) / 2) + 4);
    gen_helper_store_dbatl(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

/* SDR1 */
void spr_write_sdr1(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_sdr1(cpu_env, cpu_gpr[gprn]);
}

void spr_write_601_rtcu(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_601_rtcu(cpu_env, cpu_gpr[gprn]);
}

void spr_write_601_rtcl(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_601_rtcl(cpu_env, cpu_gpr[gprn]);
}

void spr_write_hid0_601(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_hid0_601(cpu_env, cpu_gpr[gprn]);
    /* Must stop the translation as endianness may have changed */
    gen_stop_exception(ctx);
}

void spr_read_601_ubat(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_ld_tl(cpu_gpr[gprn], cpu_env,
                  offsetof(CPUPPCState,
                           IBAT[sprn & 1][(sprn - SPR_IBAT0U) / 2]));
}

void spr_write_601_ubatu(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32((sprn - SPR_IBAT0U) / 2);
    gen_helper_store_601_batl(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_601_ubatl(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32((sprn - SPR_IBAT0U) / 2);
    gen_helper_store_601_batu(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_read_40x_pit(DisasContext *ctx, int gprn, int sprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_load_40x_pit(cpu_gpr[gprn], cpu_env);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_40x_pit(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_40x_pit(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_40x_dbcr0(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_store_spr(sprn, cpu_gpr[gprn]);
    gen_helper_store_40x_dbcr0(cpu_env, cpu_gpr[gprn]);
    /* We must stop translation as we may have rebooted */
    gen_stop_exception(ctx);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_40x_sler(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_40x_sler(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_booke_tcr(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_booke_tcr(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_booke_tsr(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_booke_tsr(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

/* PowerPC 403 specific registers */
/* PBL1 / PBU1 / PBL2 / PBU2 */
void spr_read_403_pbr(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_ld_tl(cpu_gpr[gprn], cpu_env,
                  offsetof(CPUPPCState, pb[sprn - SPR_403_PBL1]));
}

void spr_write_403_pbr(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32(sprn - SPR_403_PBL1);
    gen_helper_store_403_pbr(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_pir(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();
    tcg_gen_andi_tl(t0, cpu_gpr[gprn], 0xF);
    gen_store_spr(SPR_PIR, t0);
    tcg_temp_free(t0);
}
/* end PPC 403 specific */

/* Callback used to write the exception vector base */
void spr_write_excp_prefix(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();
    tcg_gen_ld_tl(t0, cpu_env, offsetof(CPUPPCState, ivpr_mask));
    tcg_gen_and_tl(t0, t0, cpu_gpr[gprn]);
    tcg_gen_st_tl(t0, cpu_env, offsetof(CPUPPCState, excp_prefix));
    gen_store_spr(sprn, t0);
    tcg_temp_free(t0);
}

void spr_write_excp_vector(DisasContext *ctx, int sprn, int gprn)
{
    int sprn_offs;

    if (sprn >= SPR_BOOKE_IVOR0 && sprn <= SPR_BOOKE_IVOR15) {
        sprn_offs = sprn - SPR_BOOKE_IVOR0;
    } else if (sprn >= SPR_BOOKE_IVOR32 && sprn <= SPR_BOOKE_IVOR37) {
        sprn_offs = sprn - SPR_BOOKE_IVOR32 + 32;
    } else if (sprn >= SPR_BOOKE_IVOR38 && sprn <= SPR_BOOKE_IVOR42) {
        sprn_offs = sprn - SPR_BOOKE_IVOR38 + 38;
    } else {
        printf("Trying to write an unknown exception vector %d %03x\n",
               sprn, sprn);
        gen_inval_exception(ctx, POWERPC_EXCP_PRIV_REG);
        return;
    }

    TCGv t0 = tcg_temp_new();
    tcg_gen_ld_tl(t0, cpu_env, offsetof(CPUPPCState, ivor_mask));
    tcg_gen_and_tl(t0, t0, cpu_gpr[gprn]);
    tcg_gen_st_tl(t0, cpu_env, offsetof(CPUPPCState, excp_vectors[sprn_offs]));
    gen_store_spr(sprn, t0);
    tcg_temp_free(t0);
}
/* end exception vector callbacks */

void spr_read_thrm(DisasContext *ctx, int gprn, int sprn)
{
    gen_helper_fixup_thrm(cpu_env);
    gen_load_spr(cpu_gpr[gprn], sprn);
    spr_load_dump_spr(sprn);
}

void spr_write_e500_l1csr0(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();

    tcg_gen_andi_tl(t0, cpu_gpr[gprn], L1CSR0_DCE | L1CSR0_CPE);
    gen_store_spr(sprn, t0);
    tcg_temp_free(t0);
}

void spr_write_e500_l1csr1(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();

    tcg_gen_andi_tl(t0, cpu_gpr[gprn], L1CSR1_ICE | L1CSR1_CPE);
    gen_store_spr(sprn, t0);
    tcg_temp_free(t0);
}

void spr_write_e500_l2csr0(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();

    tcg_gen_andi_tl(t0, cpu_gpr[gprn],
                    ~(E500_L2CSR0_L2FI | E500_L2CSR0_L2FL | E500_L2CSR0_L2LFC));
    gen_store_spr(sprn, t0);
    tcg_temp_free(t0);
}

void spr_write_booke206_mmucsr0(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_booke206_tlbflush(cpu_env, cpu_gpr[gprn]);
}

void spr_write_booke_pid(DisasContext *ctx, int sprn, int gprn)
{
    TCGv_i32 t0 = tcg_const_i32(sprn);
    gen_helper_booke_setpid(cpu_env, t0, cpu_gpr[gprn]);
    tcg_temp_free_i32(t0);
}

void spr_write_eplc(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_booke_set_eplc(cpu_env, cpu_gpr[gprn]);
}

void spr_write_epsc(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_booke_set_epsc(cpu_env, cpu_gpr[gprn]);
}

void spr_write_mas73(DisasContext *ctx, int sprn, int gprn)
{
    TCGv val = tcg_temp_new();
    tcg_gen_ext32u_tl(val, cpu_gpr[gprn]);
    gen_store_spr(SPR_BOOKE_MAS3, val);
    tcg_gen_shri_tl(val, cpu_gpr[gprn], 32);
    gen_store_spr(SPR_BOOKE_MAS7, val);
    tcg_temp_free(val);
}

void spr_read_mas73(DisasContext *ctx, int gprn, int sprn)
{
    TCGv mas7 = tcg_temp_new();
    TCGv mas3 = tcg_temp_new();
    gen_load_spr(mas7, SPR_BOOKE_MAS7);
    tcg_gen_shli_tl(mas7, mas7, 32);
    gen_load_spr(mas3, SPR_BOOKE_MAS3);
    tcg_gen_or_tl(cpu_gpr[gprn], mas3, mas7);
    tcg_temp_free(mas3);
    tcg_temp_free(mas7);
}

#ifdef TARGET_PPC64
void spr_read_cfar(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_mov_tl(cpu_gpr[gprn], cpu_cfar);
}

void spr_write_cfar(DisasContext *ctx, int sprn, int gprn)
{
    tcg_gen_mov_tl(cpu_cfar, cpu_gpr[gprn]);
}

void spr_write_ureg(DisasContext *ctx, int sprn, int gprn)
{
    gen_store_spr(sprn + 0x10, cpu_gpr[gprn]);
}

ATTRIBUTE_UNUSED
void spr_read_purr(DisasContext *ctx, int gprn, int sprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_load_purr(cpu_gpr[gprn], cpu_env);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_purr(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_purr(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

/* HDECR */
void spr_read_hdecr(DisasContext *ctx, int gprn, int sprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_load_hdecr(cpu_gpr[gprn], cpu_env);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_end();
        gen_stop_exception(ctx);
    }
}

void spr_write_hdecr(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_hdecr(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_end();
        gen_stop_exception(ctx);
    }
}

void spr_read_vtb(DisasContext *ctx, int gprn, int sprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_load_vtb(cpu_gpr[gprn], cpu_env);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_vtb(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_vtb(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

void spr_write_tbu40(DisasContext *ctx, int sprn, int gprn)
{
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_io_start();
    }
    gen_helper_store_tbu40(cpu_env, cpu_gpr[gprn]);
    if (tb_cflags(ctx->base.tb) & CF_USE_ICOUNT) {
        gen_stop_exception(ctx);
    }
}

/* 64 bits PowerPC specific SPRs */
/* PIDR */
void spr_write_pidr(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_pidr(cpu_env, cpu_gpr[gprn]);
}

void spr_write_lpidr(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_lpidr(cpu_env, cpu_gpr[gprn]);
}

void spr_read_hior(DisasContext *ctx, int gprn, int sprn)
{
    tcg_gen_ld_tl(cpu_gpr[gprn], cpu_env, offsetof(CPUPPCState, excp_prefix));
}

void spr_write_hior(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();
    tcg_gen_andi_tl(t0, cpu_gpr[gprn], 0x3FFFFF00000ULL);
    tcg_gen_st_tl(t0, cpu_env, offsetof(CPUPPCState, excp_prefix));
    tcg_temp_free(t0);
}

void spr_write_ptcr(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_ptcr(cpu_env, cpu_gpr[gprn]);
}

void spr_write_pcr(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_pcr(cpu_env, cpu_gpr[gprn]);
}

/* DPDES */
void spr_read_dpdes(DisasContext *ctx, int gprn, int sprn)
{
    gen_helper_load_dpdes(cpu_gpr[gprn], cpu_env);
}

void spr_write_dpdes(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_dpdes(cpu_env, cpu_gpr[gprn]);
}

void spr_write_amr(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();

    /*
     * Note, the HV=1 PR=0 case is handled earlier by simply using
     * spr_write_generic for HV mode in the SPR table
     */

    /* Build insertion mask into t1 based on context */
    if (ctx->pr) {
        gen_load_spr(t1, SPR_UAMOR);
    } else {
        gen_load_spr(t1, SPR_AMOR);
    }

    /* Mask new bits into t2 */
    tcg_gen_and_tl(t2, t1, cpu_gpr[gprn]);

    /* Load AMR and clear new bits in t0 */
    gen_load_spr(t0, SPR_AMR);
    tcg_gen_andc_tl(t0, t0, t1);

    /* Or'in new bits and write it out */
    tcg_gen_or_tl(t0, t0, t2);
    gen_store_spr(SPR_AMR, t0);
    spr_store_dump_spr(SPR_AMR);

    tcg_temp_free(t0);
    tcg_temp_free(t1);
    tcg_temp_free(t2);
}

void spr_write_uamor(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();

    /*
     * Note, the HV=1 case is handled earlier by simply using
     * spr_write_generic for HV mode in the SPR table
     */

    /* Build insertion mask into t1 based on context */
    gen_load_spr(t1, SPR_AMOR);

    /* Mask new bits into t2 */
    tcg_gen_and_tl(t2, t1, cpu_gpr[gprn]);

    /* Load AMR and clear new bits in t0 */
    gen_load_spr(t0, SPR_UAMOR);
    tcg_gen_andc_tl(t0, t0, t1);

    /* Or'in new bits and write it out */
    tcg_gen_or_tl(t0, t0, t2);
    gen_store_spr(SPR_UAMOR, t0);
    spr_store_dump_spr(SPR_UAMOR);

    tcg_temp_free(t0);
    tcg_temp_free(t1);
    tcg_temp_free(t2);
}

void spr_write_iamr(DisasContext *ctx, int sprn, int gprn)
{
    TCGv t0 = tcg_temp_new();
    TCGv t1 = tcg_temp_new();
    TCGv t2 = tcg_temp_new();

    /*
     * Note, the HV=1 case is handled earlier by simply using
     * spr_write_generic for HV mode in the SPR table
     */

    /* Build insertion mask into t1 based on context */
    gen_load_spr(t1, SPR_AMOR);

    /* Mask new bits into t2 */
    tcg_gen_and_tl(t2, t1, cpu_gpr[gprn]);

    /* Load AMR and clear new bits in t0 */
    gen_load_spr(t0, SPR_IAMR);
    tcg_gen_andc_tl(t0, t0, t1);

    /* Or'in new bits and write it out */
    tcg_gen_or_tl(t0, t0, t2);
    gen_store_spr(SPR_IAMR, t0);
    spr_store_dump_spr(SPR_IAMR);

    tcg_temp_free(t0);
    tcg_temp_free(t1);
    tcg_temp_free(t2);
}

#endif
#endif /* !defined(CONFIG_USER_ONLY) */

#ifdef TARGET_PPC64
void spr_read_prev_upper32(DisasContext *ctx, int gprn, int sprn)
{
    TCGv spr_up = tcg_temp_new();
    TCGv spr = tcg_temp_new();

    gen_load_spr(spr, sprn - 1);
    tcg_gen_shri_tl(spr_up, spr, 32);
    tcg_gen_ext32u_tl(cpu_gpr[gprn], spr_up);

    tcg_temp_free(spr);
    tcg_temp_free(spr_up);
}

void spr_write_prev_upper32(DisasContext *ctx, int sprn, int gprn)
{
    TCGv spr = tcg_temp_new();

    gen_load_spr(spr, sprn - 1);
    tcg_gen_deposit_tl(spr, spr, cpu_gpr[gprn], 32, 32);
    gen_store_spr(sprn - 1, spr);

    tcg_temp_free(spr);
}

void spr_read_tar(DisasContext *ctx, int gprn, int sprn)
{
    gen_fscr_facility_check(ctx, SPR_FSCR, FSCR_TAR, sprn, FSCR_IC_TAR);
    spr_read_generic(ctx, gprn, sprn);
}

void spr_write_tar(DisasContext *ctx, int sprn, int gprn)
{
    gen_fscr_facility_check(ctx, SPR_FSCR, FSCR_TAR, sprn, FSCR_IC_TAR);
    spr_write_generic(ctx, sprn, gprn);
}

void spr_read_tm(DisasContext *ctx, int gprn, int sprn)
{
    gen_msr_facility_check(ctx, SPR_FSCR, MSR_TM, sprn, FSCR_IC_TM);
    spr_read_generic(ctx, gprn, sprn);
}

void spr_write_tm(DisasContext *ctx, int sprn, int gprn)
{
    gen_msr_facility_check(ctx, SPR_FSCR, MSR_TM, sprn, FSCR_IC_TM);
    spr_write_generic(ctx, sprn, gprn);
}

void spr_read_tm_upper32(DisasContext *ctx, int gprn, int sprn)
{
    gen_msr_facility_check(ctx, SPR_FSCR, MSR_TM, sprn, FSCR_IC_TM);
    spr_read_prev_upper32(ctx, gprn, sprn);
}

void spr_write_tm_upper32(DisasContext *ctx, int sprn, int gprn)
{
    gen_msr_facility_check(ctx, SPR_FSCR, MSR_TM, sprn, FSCR_IC_TM);
    spr_write_prev_upper32(ctx, sprn, gprn);
}

void spr_read_ebb(DisasContext *ctx, int gprn, int sprn)
{
    gen_fscr_facility_check(ctx, SPR_FSCR, FSCR_EBB, sprn, FSCR_IC_EBB);
    spr_read_generic(ctx, gprn, sprn);
}

void spr_write_ebb(DisasContext *ctx, int sprn, int gprn)
{
    gen_fscr_facility_check(ctx, SPR_FSCR, FSCR_EBB, sprn, FSCR_IC_EBB);
    spr_write_generic(ctx, sprn, gprn);
}

void spr_read_ebb_upper32(DisasContext *ctx, int gprn, int sprn)
{
    gen_fscr_facility_check(ctx, SPR_FSCR, FSCR_EBB, sprn, FSCR_IC_EBB);
    spr_read_prev_upper32(ctx, gprn, sprn);
}

void spr_write_ebb_upper32(DisasContext *ctx, int sprn, int gprn)
{
    gen_fscr_facility_check(ctx, SPR_FSCR, FSCR_EBB, sprn, FSCR_IC_EBB);
    spr_write_prev_upper32(ctx, sprn, gprn);
}

void spr_write_hmer(DisasContext *ctx, int sprn, int gprn)
{
    TCGv hmer = tcg_temp_new();

    gen_load_spr(hmer, sprn);
    tcg_gen_and_tl(hmer, cpu_gpr[gprn], hmer);
    gen_store_spr(sprn, hmer);
    spr_store_dump_spr(sprn);
    tcg_temp_free(hmer);
}

void spr_write_lpcr(DisasContext *ctx, int sprn, int gprn)
{
    gen_helper_store_lpcr(cpu_env, cpu_gpr[gprn]);
}
#endif
