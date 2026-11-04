

#include <stdint.h>
#include "bsp.h"
#include "demo.h"
#include "gpio.h"
#include "soc.h"
#include "type.h"
#include "io.h"

#ifndef SYSTEM_GPIO_0_IO_CTRL
#error "GPIO0 is disabled in this SoC"
#endif
#ifndef IO_APB_SLAVE_0_INPUT
#error "APB slave 0 is disabled in this SoC"
#endif

#define LED_MASK 0xFu

#define CFG_DIAG_BENCH   0

#define CFG_BOOT_AUTOTX  0

#define CFG_PWM_CONT     0

#define PWM_TEST_CYCLES  65535u

#ifdef __CDT_PARSER__
#define CFU(f3, f7, a, b) ((uint32_t)0)
#else
#define CFU(f3, f7, a, b)                                                     \
    ({                                                                        \
        uint32_t _d;                                                          \
        __asm__ volatile(".insn r 0x0b, " #f3 ", " #f7 ", %0, %1, %2"        \
                         : "=r"(_d)                                           \
                         : "r"((uint32_t)(a)), "r"((uint32_t)(b)));           \
        _d;                                                                   \
    })
#endif

#define SDOT4(a, b)  CFU(0, 0, a, b)
#define SAD4(a, b)   CFU(0, 1, a, b)
#define ADD4(a, b)   CFU(0, 2, a, b)
#define SUB4(a, b)   CFU(0, 3, a, b)
#define MAX4(a, b)   CFU(0, 4, a, b)
#define MIN4(a, b)   CFU(0, 5, a, b)
#define ABS4(a, b)   CFU(0, 6, a, b)
#define SHUF4(a)     CFU(0, 7, a, 0)

#define SMAC4(a, b)  CFU(1, 0, a, b)
#define LDACC(a, b)  CFU(1, 2, a, b)
#define RDACC()      CFU(1, 3, 0, 0)
#define ZRACC()      CFU(1, 4, 0, 0)
#define SSUM4(a, b)  CFU(1, 5, a, b)
#define CMUL16(a, b) CFU(1, 6, a, b)
#define CMAC16(a, b) CFU(1, 7, a, b)
#define CCLR()       CFU(1, 8, 0, 0)
#define QADD16(a, b) CFU(0, 16, a, b)
#define QSUB16(a, b) CFU(0, 17, a, b)
#define SXTB16(a)    CFU(0, 18, a, 0)
#define PKH(a, b)    CFU(0, 19, a, b)

#define BITREV(a)    CFU(2, 0, a, 0)
#define CLZ32(a)     CFU(2, 1, a, 0)
#define CTZ32(a)     CFU(2, 2, a, 0)
#define POPC32(a)    CFU(2, 3, a, 0)
#define SEXTB(a)     CFU(2, 4, a, 0)
#define ABS32(a)     CFU(2, 5, a, 0)
#define MAXV(a, b)   CFU(2, 6, a, b)
#define MINV(a, b)   CFU(2, 7, a, b)

#define SADD32(a, b) CFU(3, 0, a, b)
#define SSUB32(a, b) CFU(3, 1, a, b)
#define RSFT32(a, s) CFU(3, 2, a, s)
#define CLIP32(a, m) CFU(3, 3, a, m)
#define AVG32(a, b)  CFU(3, 4, a, b)
#define HAMM32(a, b) CFU(3, 6, a, b)
#define MIRR16(a)    CFU(3, 7, a, 0)

#define PF_BASE     IO_APB_SLAVE_0_INPUT
#define FIFO_CTRL   (PF_BASE + 0x00)
#define FIFO_STAT   (PF_BASE + 0x04)
#define FIFO_DATA_A (PF_BASE + 0x08)
#define FIFO_DATA_B (PF_BASE + 0x14)
#define FIFO_CNT    (PF_BASE + 0x0C)
#define FIFO_MAGIC  (PF_BASE + 0x10)

#define FIFO_READY_A(s)  (((s) >> 1) & 1u)
#define FIFO_READY_B(s)  (((s) >> 3) & 1u)

#define FIFO_READY(s)    (FIFO_READY_A(s) || FIFO_READY_B(s))
#define FIFO_BUSY_A(s)   ((s) & 1u)
#define FIFO_BUSY_B(s)   (((s) >> 2) & 1u)
#define FIFO_CNT_A(c)    ((c) & 0xFFFFu)
#define FIFO_CNT_B(c)    (((c) >> 16) & 0xFFFFu)

#define PWM_CH_SEL(s)    (((s) >> 5) & 1u)

#define SCR_CTRL    (PF_BASE + 0x40)
#define SCR_STAT    (PF_BASE + 0x44)
#define SCR_PT      (PF_BASE + 0x48)
#define SCR_PTCLR   (PF_BASE + 0x4C)
#define SCR_FIELD   (PF_BASE + 0x50)
#define SCR_PTMAX   (PF_BASE + 0x54)
#define SCR_CURVE   (PF_BASE + 0x58)
#define SCR_PTS     (PF_BASE + 0x5C)

#define UART_TX     (PF_BASE + 0x80)
#define UART_STAT   (PF_BASE + 0x84)
#define UART_DIV    (PF_BASE + 0x88)

#define DBG_DELAY   (PF_BASE + 0xA8)
#define SCR_RX_CTRL (PF_BASE + 0xB0)
#define SCR_RX_STAT (PF_BASE + 0xB4)
#define SCR_RX_DATA (PF_BASE + 0xB8)

#define MEAS_TIMEOUT (PF_BASE + 0x30)

#define DA_CODE     (PF_BASE + 0x1C)

#define MEAS_DSP    (PF_BASE + 0x20)

#define MEAS_STAT   (PF_BASE + 0x24)

#define PWM_CTRL    (PF_BASE + 0x90)

#define PWM_PERIOD  (PF_BASE + 0x94)
#define PWM_CYCLES  (PF_BASE + 0x98)
#define PWM_DELAY   (PF_BASE + 0x9C)
#define PWM_STAT    (PF_BASE + 0xA0)

#define PWM_WINCLR  (PF_BASE + 0xA4)

#define FIFO_MAGIC_EXPECTED 0x46524651u

static int8_t g_buf[512];

#define BUF(i)  (&g_buf[(i)])

static void scr_flush_wait(void);
static int  scr_busy(void);
static void scr_on_a5(void);

#define N_SAMP        256u
#define MAXLAG        30u

#define LAG_TRACK_WIN   4
static int32_t   g_last_lag;
static uint8_t   g_lag_init;

#define MEAS_FS_HZ    1515152u
#define MEAS_CLK_PER_PAIR  100u
#define F0_HZ         200000u

#define MEAS_OSR_X1000  7576u

#define DISP_MODE 3

#define ACQ_USE_PWM  1

#define DISP_CH_A 0u
#define DISP_CH_B 1u
#define DISP_CH_R 0u

#define FLD_TYPE_N  1u
#define FLD_TYPE_X  2u
#define FLD_DTOF    0u
#define FLD_NS      1u
#define FLD_PEAK    2u
#define FLD_LAG     3u
#define FLD_STATE   4u

#define DTOF_FRAC_BITS  12
#define DTOF_ONE        (1 << DTOF_FRAC_BITS)

#define COS_A1_Q16   34783u
#define COS_A3_Q16   2226u
#define COS_A5_Q16   212u

#if XCORR_MODE == XCMODE_AB
#define VEL_FACTOR   5
#else
#define VEL_FACTOR   10
#endif

static int g_pass, g_fail;

#define CHK(name, expr, exp)                                              \
    do {                                                                  \
        bsp_printf("\r\n  %-9s", (name));                                 \
        check((name), (uint32_t)(expr), (uint32_t)(exp));                 \
    } while (0)

static inline uint32_t ticks(void) { return clint_getTimeLow(BSP_CLINT); }

static uint8_t satq(int32_t v)
{
    if (v > 127) return 127;
    if (v < -128) return -128;
    return (uint8_t)(int8_t)v;
}
static int64_t g_cacc_re, g_cacc_im;

static int8_t g_fa[N_SAMP];
static int8_t g_fb[N_SAMP];

static int16_t g_raw_a[N_SAMP];
static int16_t g_raw_b[N_SAMP];

typedef union {
    int8_t   i8[N_SAMP];
    uint32_t u32[N_SAMP / 4u];
} smp_buf_t;

static smp_buf_t g_ffa;
static smp_buf_t g_ffb;
static int32_t g_corr[2u * MAXLAG + 1u];

#define ZERO_CAL_DEFAULT   59

#define ZERO_CAL_MIN      (-96)
#define ZERO_CAL_MAX        96
static int16_t g_zero_a = ZERO_CAL_DEFAULT;
static int16_t g_zero_b = ZERO_CAL_DEFAULT;
static uint8_t g_zero_valid = 0u;

static uint8_t g_cur_ch = 0u;

typedef struct {
    int32_t  peak;
    int32_t  peak_idx;
    int32_t  lag;
    int32_t  delta_q12;
    int32_t  dtof_q12;
    uint32_t cyc;
    uint32_t abs_peak;
    uint32_t abs_idx;
    int32_t  abs_delta_q8;
} dsp_res_t;

static int32_t g_temp_c_x10 = 200;

static int32_t sound_speed_cm_s_x100(void)
{

    return 33130 + (606 * g_temp_c_x10) / 100;
}

static int32_t temp_ratio_q16(int32_t t0_x10)
{
    int32_t c0 = 33130 + (606 * t0_x10) / 100;
    int32_t c1 = sound_speed_cm_s_x100();
    if (c1 == 0) return 65536;
    return (int32_t)(((int64_t)c0 << 16) / c1);
}

static int32_t g_off_q12      = 0;
static int32_t g_off_t0_x10   = 200;
static int      g_off_valid   = 0;

static int32_t cal_offset_q12(void)
{
    if (!g_off_valid) return 0;
    return (int32_t)(((int64_t)g_off_q12 * temp_ratio_q16(g_off_t0_x10)) >> 16);
}

static int32_t cal_dtof_true_q12(int32_t dtof_meas_q12)
{
    return dtof_meas_q12 - cal_offset_q12();
}

static int32_t g_l_mm = 82;

static int32_t g_c_meas_mm_per_us = 333;

static int32_t g_da_code = 3506;

#define BURST_N          5u
#define BURST_GAP_TICKS  200000u

static int      g_burst_run    = 1;

static uint32_t g_burst_idx    = 0;
static uint32_t g_burst_t0     = 0;
static int      g_burst_first  = 1;
static int      g_pwm_cont     = CFG_PWM_CONT;

#define CFG_PWM_CYCLES   6u
#define CFG_PWM_PERIOD   500u

#define PWM_DELAY_FALLBACK_CLK  20000u

static int32_t cal_velocity_x100(int32_t dtof_true_q12)
{
    int64_t c = (int64_t)sound_speed_cm_s_x100();

    return (int32_t)(c * c * (int64_t)dtof_true_q12 * (int64_t)VEL_FACTOR
                     / ((int64_t)DTOF_ONE * (int64_t)MEAS_FS_HZ * (int64_t)g_l_mm));
}

static void dsp_frame(dsp_res_t *r);

#define CAL_FRAMES   8u

static int32_t cal_median_i32(int32_t *v, uint32_t n)
{
    uint32_t i, j;
    for (i = 1u; i < n; i++) {
        int32_t key = v[i];
        j = i;
        while (j > 0u && v[j - 1u] > key) {
            v[j] = v[j - 1u];
            j--;
        }
        v[j] = key;
    }
    return v[n / 2u];
}

static int zero_wind_calibrate(void)
{

    static int32_t samples[CAL_FRAMES];
    uint32_t got = 0u, attempt;

    bsp_printf("    [CAL0] 标定中(无风)...\r\n");

    for (attempt = 0u; attempt < CAL_FRAMES * 3u && got < CAL_FRAMES; attempt++) {
        static dsp_res_t r;
        uint32_t  st, tmo = 0u;

        write_u32(2u, FIFO_CTRL);
        write_u32(0u, FIFO_CTRL);
        write_u32(1u, FIFO_CTRL);
        st = read_u32(FIFO_STAT);
        while (!FIFO_READY(st) && tmo < 500000u) {
            st = read_u32(FIFO_STAT);
            tmo++;
        }
        if (!FIFO_READY(st)) continue;

        dsp_frame(&r);
        if (r.peak <= 0) continue;
        if (r.dtof_q12 == 0) continue;

        samples[got++] = r.dtof_q12;
    }

    if (got < CAL_FRAMES / 2u) {
        bsp_printf("    [CAL0] FAIL %u/%u\r\n",
                   (unsigned)got, (unsigned)CAL_FRAMES);
        return 1;
    }

    g_off_q12    = cal_median_i32(samples, got);
    g_off_t0_x10 = g_temp_c_x10;
    g_off_valid  = 1;

    {
        int32_t off_ns = (int32_t)((int64_t)g_off_q12 * 1000000000ll
                                   / ((int64_t)DTOF_ONE * (int64_t)MEAS_FS_HZ));
        bsp_printf("    [CAL0] OK %u off=%d %dns @%dC\r\n",
                   (unsigned)got, (int)g_off_q12, (int)off_ns,
                   (int)(g_off_t0_x10 / 10));
    }
    return 0;
}

static int32_t cosine_delta_q12_c(int32_t rm, int32_t rc, int32_t rp,
                                  uint32_t a1, uint32_t a3, uint32_t a5)
{
    int64_t num, den;
    int64_t u_q16;
    uint32_t au;
    int64_t h;
    int32_t out;

    num = (int64_t)rp - (int64_t)rm;
    den = 2 * (int64_t)rc - (int64_t)rm - (int64_t)rp;

    if (den <= 0) return 0;

    u_q16 = (num << 16) / den;

    if (u_q16 >  65536) u_q16 =  65536;
    if (u_q16 < -65536) u_q16 = -65536;

    au = (uint32_t)((u_q16 < 0) ? -u_q16 : u_q16);

    {
        uint64_t auL = au;
        uint64_t au2 = (auL * auL) >> 16;
        uint64_t au3 = (au2 * auL) >> 16;
        uint64_t au4 = (au2 * au2) >> 16;
        uint64_t au5 = (au4 * auL) >> 16;
        int64_t hh = ((int64_t)a1 * au
                      - (int64_t)a3 * (int64_t)au3
                      + (int64_t)a5 * (int64_t)au5) >> 16;
        h = hh;
    }

    out = (int32_t)(h >> (16 - DTOF_FRAC_BITS));
    if (u_q16 < 0) out = -out;
    return out;
}

static int32_t cosine_delta_q12(int32_t rm, int32_t rc, int32_t rp)
{
    return cosine_delta_q12_c(rm, rc, rp,
                              COS_A1_Q16, COS_A3_Q16, COS_A5_Q16);
}

#define COS10_A1_Q16  67781u
#define COS10_A3_Q16  2385u
#define COS10_A5_Q16  151u

static int32_t g_peak_i = (int32_t)MAXLAG;

static void pack4_array(const int8_t *src, uint32_t *dst, uint32_t n)
{
    uint32_t i, j;
    for (i = 0u, j = 0u; i + 4u <= n; i += 4u, j++) {
        dst[j] = pack4_i8(src[i], src[i + 1u], src[i + 2u], src[i + 3u]);
    }
}

static void xcorr_packed(const uint32_t *pa, const uint32_t *pb,
                         int32_t *out, uint32_t nw)
{
    int32_t i;
    for (i = 0; i <= (int32_t)(2u * MAXLAG); i++) {
        int32_t k = i - (int32_t)MAXLAG;
        int32_t acc;

        if ((k & 3) == 0) {

            int32_t kq = k >> 2;
            int32_t w_lo = (kq < 0) ? -kq : 0;
            int32_t w_hi = (kq >= 0) ? ((int32_t)nw - kq) : (int32_t)nw;
            int32_t w;

            ZRACC();
            for (w = w_lo; w < w_hi; w++) {
                SMAC4(pa[w], pb[w + kq]);
            }
            acc = (int32_t)RDACC();
        } else {

            uint32_t n_lo = (k < 0) ? (uint32_t)(-k) : 0u;
            uint32_t n_hi = (k >= 0) ? (4u * nw - (uint32_t)k) : (4u * nw);
            uint32_t w;
            const int8_t *a8 = (const int8_t *)pa;
            const int8_t *b8 = (const int8_t *)pb;
            acc = 0;
            for (w = n_lo; w < n_hi; w++) {
                acc += (int32_t)a8[w] * (int32_t)b8[(int32_t)w + k];
            }
        }
        out[i] = acc;
    }
}

static void xcorr_sw(const int8_t *a, const int8_t *b, int32_t *out, uint32_t nl)
{
    uint32_t nw = nl / 4u;
    if (nw > 128u) nw = 128u;
    pack4_array(a, g_ffa.u32, nw * 4u);
    pack4_array(b, g_ffb.u32, nw * 4u);
    xcorr_packed(g_ffa.u32, g_ffb.u32, out, nw);
}

static void peak_find(const int32_t *R, int32_t *best_v, int32_t *best_i)
{
    int32_t i, bv = R[0], bi = 0;
    for (i = 1; i <= (int32_t)(2u * MAXLAG); i++) {
        if (R[i] > bv) { bv = R[i]; bi = i; }
    }
    *best_v = bv;
    *best_i = bi;
}

#define LAG_TRK_WIN_INIT   3

static int32_t g_lag_trk;
static int32_t g_dtof_unwrap;
static int32_t  g_dtof_ns;

static int32_t g_med_buf[11];
static uint8_t g_med_idx;
static uint8_t g_med_cnt;

static int32_t med11_push(int32_t v)
{
    int32_t t[11];
    uint8_t i, j, n;

    g_med_buf[g_med_idx] = v;
    g_med_idx = (uint8_t)((g_med_idx + 1u) % 11u);
    if (g_med_cnt < 11u) g_med_cnt++;

    n = g_med_cnt;
    for (i = 0u; i < n; i++) t[i] = g_med_buf[i];
    for (i = 1u; i < n; i++) {
        int32_t x = t[i];
        j = i;
        while (j > 0u && t[j - 1u] > x) { t[j] = t[j - 1u]; j--; }
        t[j] = x;
    }
    return t[n >> 1];
}

static int32_t g_avg_buf[8];
static int32_t g_avg_sum;
static uint8_t g_avg_idx;
static uint8_t g_avg_cnt;

static int32_t avg8_push(int32_t v)
{
    g_avg_sum -= g_avg_buf[g_avg_idx];
    g_avg_buf[g_avg_idx] = v;
    g_avg_sum += v;
    g_avg_idx = (uint8_t)((g_avg_idx + 1u) % 8u);
    if (g_avg_cnt < 8u) g_avg_cnt++;
    return g_avg_sum / (int32_t)g_avg_cnt;
}
static uint8_t g_lag_trk_init;

static int8_t  g_a8[N_SAMP];
static int8_t  g_tmpl_i8[N_SAMP];
static int16_t g_env_a[N_SAMP];
static int16_t g_env_b[N_SAMP];
static int32_t g_coarse_lag;

static void env9(const int8_t *x, int16_t *e, uint32_t n)
{
    uint32_t i; int32_t j;
    for (i = 0u; i < n; i++) {
        int32_t sum = 0;
        for (j = -4; j <= 4; j++) {
            int32_t k = (int32_t)i + j;
            if (k < 0) k = 0;
            if (k >= (int32_t)n) k = (int32_t)n - 1;
            sum += (x[k] < 0) ? -(int32_t)x[k] : (int32_t)x[k];
        }
        e[i] = (int16_t)(sum / 9);
    }
}

static inline int8_t fir9_px(const int8_t *in, int32_t i, uint32_t n)
{
    static const int32_t k[9] = { 1, 8, 28, 56, 70, 56, 28, 8, 1 };
    int32_t s = 0, j;
    for (j = 0; j < 9; j++) {
        int32_t idx = i + j - 4;
        if (idx < 0) idx = 0;
        if (idx > (int32_t)n - 1) idx = (int32_t)n - 1;
        s += k[j] * (int32_t)in[idx];
    }
    return (int8_t)((s + 128) >> 8);
}

static void fir9_lp(const int8_t *in, int8_t *out, uint32_t n)
{
    uint32_t i;

    if (n < 10u) {
        for (i = 0u; i < n; i++) out[i] = fir9_px(in, (int32_t)i, n);
        return;
    }

    for (i = 0u; i < 4u; i++)
        out[i] = fir9_px(in, (int32_t)i, n);

    for (i = 4u; i + 4u < n; i++) {
        const int8_t *p = in + i - 4;
        int32_t s =      (int32_t)p[0]
                  +  8 * (int32_t)p[1]
                  + 28 * (int32_t)p[2]
                  + 56 * (int32_t)p[3]
                  + 70 * (int32_t)p[4]
                  + 56 * (int32_t)p[5]
                  + 28 * (int32_t)p[6]
                  +  8 * (int32_t)p[7]
                  +      (int32_t)p[8];
        out[i] = (int8_t)((s + 128) >> 8);
    }

    for (; i < n; i++)
        out[i] = fir9_px(in, (int32_t)i, n);
}

#define ROT_PADB   32u
#define ROT_PADW   (ROT_PADB / 4u)
#define ROT_NW     (N_SAMP / 4u + ROT_PADW + (MAXLAG + 3u) / 4u + 1u)

static uint32_t g_rot[4][ROT_NW];

#define XCMODE_AB   0
#define XCMODE_AA   1
#define XCMODE_BB   2

#ifndef XCORR_MODE
#define XCORR_MODE  XCMODE_AB
#endif

static smp_buf_t g_tmpl;
static uint8_t   g_tmpl_valid;
static uint8_t   g_tmpl_req;

#define TMPL_DELAY_TICKS  200000000u
static uint32_t  g_t_start;
static uint16_t  g_acq_cnt_a;
static uint16_t  g_acq_cnt_b;
static uint8_t   g_tmpl_ready;
static int16_t   g_tmpl_raw[N_SAMP];

static int16_t   g_s1_snap[N_SAMP];
static uint8_t   g_s1_ok;
static uint32_t  g_s1_cnt;
static uint32_t  g_flush_t;

static void rot4_build(const uint32_t *pb, uint32_t nw)
{
    uint32_t m;

    for (m = 0u; m < ROT_NW; m++)
        g_rot[0][m] = (m >= ROT_PADW && m - ROT_PADW < nw)
                      ? pb[m - ROT_PADW] : 0u;

    for (m = 0u; m < ROT_NW; m++) {
        uint32_t a  = g_rot[0][m];
        uint32_t nb = (m + 1u < ROT_NW) ? g_rot[0][m + 1u] : 0u;
        g_rot[1][m] = (a >> 8)  | (nb << 24);
        g_rot[2][m] = (a >> 16) | (nb << 16);
        g_rot[3][m] = (a >> 24) | (nb << 8);
    }
}

static void xcorr_packed_fast(const uint32_t *pa, int32_t *out, uint32_t nw)
{
    int32_t k;
    for (k = -(int32_t)MAXLAG; k <= (int32_t)MAXLAG; k++) {
        uint32_t t = (uint32_t)(k + (int32_t)ROT_PADB);
        uint32_t r = t & 3u;
        uint32_t q = t >> 2;
        const uint32_t *pr = &g_rot[r][q];
        uint32_t w;

        ZRACC();
        for (w = 0u; w < nw; w++) {
            SMAC4(pa[w], pr[w]);
        }
        out[k + (int32_t)MAXLAG] = (int32_t)RDACC();
    }
}

static void remove_mean_i8(int8_t *v, uint32_t n)
{
    int32_t sum = 0;
    uint32_t i;
    int32_t m;
    for (i = 0u; i < n; i++) sum += (int32_t)v[i];
    m = sum / (int32_t)n;
    if (m == 0) return;
    for (i = 0u; i < n; i++) {
        int32_t t = (int32_t)v[i] - m;
        if (t >  127) t =  127;
        if (t < -128) t = -128;
        v[i] = (int8_t)t;
    }
}

static int fifo_take_frame(void)
{
    uint32_t tmo = 0u, st, i, w;
    int ch;

    for (;;) {
        st = read_u32(FIFO_STAT);
        if (FIFO_READY_A(st) || FIFO_READY_B(st))
            break;
        if (tmo >= 1000000u)
            return -1;
        tmo++;
    }

    ch = 0;
    g_cur_ch = (uint8_t)ch;

    for (i = 0u; i < N_SAMP; i++) {
        w = read_u32(FIFO_DATA_A) & 0x0FFFu;
        g_raw_a[i] = (int16_t)w;
        g_fa[i]    = (int8_t)(int32_t)(((int32_t)w >> 4) - 128);
    }
    for (i = 0u; i < N_SAMP; i++) {
        w = read_u32(FIFO_DATA_B) & 0x0FFFu;
        g_raw_b[i] = (int16_t)w;
        g_fb[i]    = (int8_t)(int32_t)(((int32_t)w >> 4) - 128);
    }
    return ch;
}

#define PWM_ERR_STAT   (PF_BASE + 0x24)
#define PWM_ST_ERROR   (1u << 4)

static void meas_trigger(void)
{
    uint32_t st = read_u32(PWM_STAT);

    if (st & PWM_ST_ERROR) {
        uint32_t code = read_u32(PWM_ERR_STAT) >> 24;
        int      w;

        bsp_printf("    [FSM] HALT 保护态，错误码 %u（退出后需再触发一次）\r\n", code);

        write_u32(1u, PWM_CTRL);
        for (w = 0; w < 2000; w++) {
            if (!(read_u32(PWM_STAT) & PWM_ST_ERROR)) break;
        }
        write_u32(1u, PWM_CTRL);
    } else {
        write_u32(1u, PWM_CTRL);
    }
}

static int zero_calibrate(void)
{
    uint32_t i;
    int32_t  sa = 0, sb = 0;
    int32_t  ga, gb;
    int      ch, rnd;

    for (rnd = 0; rnd < 2; rnd++) {
        uint32_t guard = 0u;

        while ((read_u32(PWM_STAT) & 0x2u) && guard < 2000000u)
            guard++;

        write_u32(2u, FIFO_CTRL);
        write_u32(0u, FIFO_CTRL);
        write_u32(3381u, DA_CODE);
        write_u32(5u, FIFO_CTRL);
        meas_trigger();

        ch = fifo_take_frame();
        if (ch < 0)
            return 1u;

        for (i = 0u; i < N_SAMP; i++) sa += (int32_t)g_fa[i];
        for (i = 0u; i < N_SAMP; i++) sb += (int32_t)g_fb[i];
    }

    ga = (int32_t)(sa / (int32_t)N_SAMP);
    gb = (int32_t)(sb / (int32_t)N_SAMP);

    {
        int va = (ga >= ZERO_CAL_MIN && ga <= ZERO_CAL_MAX);
        int vb = (gb >= ZERO_CAL_MIN && gb <= ZERO_CAL_MAX);

        g_zero_a = va ? (int16_t)ga : 0;
        g_zero_b = vb ? (int16_t)gb : 0;
        g_zero_valid = (uint8_t)((va || vb) ? 1u : 0u);

        if (va && vb) return 0u;
        if (va || vb) return 3u;
        return 2u;
    }
}

static void dsp_frame(dsp_res_t *r)
{
    uint32_t i, t0, t1;
    int32_t  bi, bv;
    int32_t  rm, rc, rp;

    for (i = 0u; i < N_SAMP; i++) {
        uint32_t wa = read_u32(FIFO_DATA_A) & 0x0FFFu;
        g_raw_a[i] = (int16_t)wa;
        g_fa[i] = (int8_t)(int32_t)(((int32_t)wa >> 4) - 128);
    }
    for (i = 0u; i < N_SAMP; i++) {
        uint32_t wb = read_u32(FIFO_DATA_B) & 0x0FFFu;
        g_raw_b[i] = (int16_t)wb;
        g_fb[i] = (int8_t)(int32_t)(((int32_t)wb >> 4) - 128);
    }
    if (g_zero_valid && g_zero_a >= ZERO_CAL_MIN && g_zero_a <= ZERO_CAL_MAX) {
        for (i = 0u; i < N_SAMP; i++)
            g_fa[i] = satq((int32_t)g_fa[i] - (int32_t)g_zero_a);
    } else {
        g_zero_a = 0;
    }
    if (g_zero_valid && g_zero_b >= ZERO_CAL_MIN && g_zero_b <= ZERO_CAL_MAX) {
        for (i = 0u; i < N_SAMP; i++)
            g_fb[i] = satq((int32_t)g_fb[i] - (int32_t)g_zero_b);
    } else {
        g_zero_b = 0;
    }

    write_u32(1u, MEAS_DSP);

    fir9_lp(g_fa, g_ffa.i8, N_SAMP);
    fir9_lp(g_fb, g_ffb.i8, N_SAMP);

    remove_mean_i8(g_ffa.i8, N_SAMP);
    remove_mean_i8(g_ffb.i8, N_SAMP);

    t0 = ticks();
    {
        uint32_t q;
        for (q = 0u; q < (uint32_t)N_SAMP; q++) g_a8[q] = g_ffa.i8[q];
    }
    pack4_array(g_ffa.i8, g_ffa.u32, N_SAMP);
    pack4_array(g_ffb.i8, g_ffb.u32, N_SAMP);
    {
        uint32_t nw = N_SAMP / 4u;
        uint32_t wi;

#if XCORR_MODE == XCMODE_AA

        if (g_cur_ch == 0u) {
            g_acq_cnt_a++;
            if (g_t_start == 0u) g_t_start = ticks();

            if ((g_tmpl_ready == 0u && g_acq_cnt_a >= 20u) || g_tmpl_req != 0u) {
                for (wi = 0u; wi < nw; wi++) g_tmpl.u32[wi] = g_ffa.u32[wi];
                for (wi = 0u; wi < (uint32_t)N_SAMP; wi++) g_tmpl_raw[wi] = g_raw_a[wi];
                for (wi = 0u; wi < (uint32_t)N_SAMP; wi++) g_tmpl_i8[wi]  = g_a8[wi];
                g_tmpl_valid = 1u;
                g_tmpl_ready = 1u;
                g_lag_init   = 0u;
                g_lag_trk = 0;
                g_dtof_unwrap = 0;
                g_med_cnt = 0u; g_med_idx = 0u;
                g_avg_cnt = 0u; g_avg_idx = 0u; g_avg_sum = 0;
                g_lag_trk_init = 1u;
                g_tmpl_req   = 0u;
                { uint32_t el = (uint32_t)(ticks() - g_t_start);
                  bsp_printf("    [TMPL] 上电 %u.%02u s（第 %u 次 A 轮），已保存无风窗口波形\r\n",
                             (unsigned)(el / 100000000u),
                             (unsigned)((el / 1000000u) % 100u),
                             (unsigned)g_acq_cnt_a); }
            }
        }
        if (g_tmpl_valid) {
            rot4_build(g_tmpl.u32, nw);
            xcorr_packed_fast(g_ffa.u32, g_corr, nw);

        } else {
            for (wi = 0u; wi <= 2u * MAXLAG; wi++) g_corr[wi] = 0;
            g_coarse_lag = 0;
        }

#elif XCORR_MODE == XCMODE_BB

        if (g_cur_ch == 1u) {
            g_acq_cnt_b++;
            if (g_t_start == 0u) g_t_start = ticks();

            if ((g_tmpl_ready == 0u && g_acq_cnt_b >= 20u) || g_tmpl_req != 0u) {
                for (wi = 0u; wi < nw; wi++) g_tmpl.u32[wi] = g_ffb.u32[wi];
                for (wi = 0u; wi < (uint32_t)N_SAMP; wi++) g_tmpl_raw[wi] = g_raw_b[wi];
                g_tmpl_valid = 1u;
                g_tmpl_ready = 1u;
                g_lag_init   = 0u;
                g_lag_trk = 0;
                g_dtof_unwrap = 0;
                g_med_cnt = 0u; g_med_idx = 0u;
                g_avg_cnt = 0u; g_avg_idx = 0u; g_avg_sum = 0;
                g_lag_trk_init = 1u;
                g_tmpl_req   = 0u;
                { uint32_t el = (uint32_t)(ticks() - g_t_start);
                  bsp_printf("    [TMPL] 上电 %u.%02u s（第 %u 次 B 轮），已保存无风窗口波形\r\n",
                             (unsigned)(el / 100000000u),
                             (unsigned)((el / 1000000u) % 100u),
                             (unsigned)g_acq_cnt_b); }
            }
        }
        if (g_tmpl_valid) {
            rot4_build(g_tmpl.u32, nw);
            xcorr_packed_fast(g_ffb.u32, g_corr, nw);
        } else {
            for (wi = 0u; wi <= 2u * MAXLAG; wi++) g_corr[wi] = 0;
        }

#else

        (void)wi;
        rot4_build(g_ffb.u32, nw);
        xcorr_packed_fast(g_ffa.u32, g_corr, nw);
#endif
    }
    t1 = ticks();
    r->cyc = t1 - t0;

    {
        int32_t l2, bi2 = -1, bv2 = -2147483647;
        int32_t lo, hi;

        if (g_lag_init) {
            lo = g_last_lag - LAG_TRACK_WIN;
            hi = g_last_lag + LAG_TRACK_WIN;
            if (lo < -(int32_t)MAXLAG) lo = -(int32_t)MAXLAG;
            if (hi >  (int32_t)MAXLAG) hi =  (int32_t)MAXLAG;
        } else {
            lo = -(int32_t)MAXLAG;
            hi =  (int32_t)MAXLAG;
        }

        for (l2 = lo; l2 <= hi; l2++) {
            int32_t idx = l2 + (int32_t)MAXLAG;
            if (idx < 0 || idx > (int32_t)(2u * MAXLAG)) continue;
            if (g_corr[idx] > bv2) { bv2 = g_corr[idx]; bi2 = idx; }
        }
        if (bi2 < 0) { bi2 = (int32_t)MAXLAG; bv2 = 0; }

        bi = bi2; bv = bv2;
        r->peak     = bv;
        r->peak_idx = bi;
        r->lag      = bi - (int32_t)MAXLAG;
        g_last_lag  = r->lag;
        g_lag_init  = 1u;
    }

    if (bi == 0 || bi == (int32_t)(2u * MAXLAG)) {
        r->delta_q12 = 0;
    } else {
        rm = g_corr[bi - 1];
        rc = g_corr[bi];
        rp = g_corr[bi + 1];
        r->delta_q12 = cosine_delta_q12(rm, rc, rp);
    }

    r->dtof_q12 = (r->lag << DTOF_FRAC_BITS) + r->delta_q12;
    g_peak_i = bi;
}

static const int8_t S10_0[10] = {  0,  7, 11, 11,  7,  0, -7, -11, -11, -7 };
static const int8_t S10_P5[10] = {  4, 10, 12, 10,  4, -4, -10, -12, -10, -4 };

static void scr_write_field(uint32_t id, int32_t val)
{
    uint32_t sign = (val < 0) ? 1u : 0u;
    uint32_t v    = (val < 0) ? (uint32_t)(-val) : (uint32_t)val;
    if (v > 0xFFFFFFu) v = 0xFFFFFFu;

    write_u32((id & 0x1Fu) | (FLD_TYPE_N << 5) | (sign << 7) | (v << 8), SCR_FIELD);
}

static void scr_write_xfield(uint32_t id, int32_t val)
{
    uint32_t sign = (val < 0) ? 1u : 0u;
    uint32_t v    = (val < 0) ? (uint32_t)(-val) : (uint32_t)val;
    if (v > 0xFFFFFFu) v = 0xFFFFFFu;

    write_u32((id & 0x1Fu) | (FLD_TYPE_X << 5) | (sign << 7) | (v << 8), SCR_FIELD);
}

static void scr_field_wait(void)
{
    uint32_t t0;

    t0 = ticks();
    while ((uint32_t)(ticks() - t0) < 2000u) {
        if (read_u32(SCR_STAT) & 1u) break;
    }

    t0 = ticks();
    while ((uint32_t)(ticks() - t0) < 2000000u) {
        if (!(read_u32(SCR_STAT) & 1u)) break;
    }
}

static void scr_curve(uint32_t obj, uint32_t ch)
{

    write_u32((obj << 10) | (ch << 8), SCR_CURVE);
}

static void scr_put_i8(const int8_t *v, uint32_t n)
{
    uint32_t i;
    int32_t  lo = 127, hi = -128;

    for (i = 0u; i < n; i++) {
        int32_t x = (int32_t)v[i];
        if (x < lo) lo = x;
        if (x > hi) hi = x;
    }
    if (hi - lo < 2) { lo = -128; hi = 127; }

    write_u32(1u, SCR_PTCLR);
    for (i = 0u; i < n; i++) {

        int32_t t = (((int32_t)v[i] - lo) * 112) / (hi - lo);
        if (t < 0) t = 0;
        if (t > 112) t = 112;
        write_u32((uint32_t)t, SCR_PT);
    }
}

static void scr_put_u12(const int16_t *v, uint32_t n)
{
    uint32_t i;
    int32_t  lo = 32767, hi = -32768;

    for (i = 0u; i < n; i++) {
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    if (hi - lo < 8) { lo = 0; hi = 4095; }

    write_u32(1u, SCR_PTCLR);
    for (i = 0u; i < n; i++) {
        int32_t t = (((int32_t)v[i] - lo) * 112) / (hi - lo);
        if (t < 0) t = 0;
        if (t > 112) t = 112;
        write_u32((uint32_t)t, SCR_PT);
    }
}

static void scr_put_corr_span(const int32_t *R, uint32_t nl, uint32_t n, int clear)
{
    uint32_t i;
    int32_t  lo = R[0], hi = R[0];

    for (i = 0u; i < nl; i++) {
        if (R[i] < lo) lo = R[i];
        if (R[i] > hi) hi = R[i];
    }
    if (clear) write_u32(1u, SCR_PTCLR);
    for (i = 0u; i < n; i++) {

        uint32_t pos_q16 = (uint32_t)(((uint64_t)i * (nl - 1u) << 16) / (n - 1u));
        uint32_t k0 = pos_q16 >> 16;
        uint32_t fr = pos_q16 & 0xFFFFu;
        int32_t  a, b, val;
        int32_t  t;
        uint32_t y;

        if (k0 >= nl - 1u) { a = b = R[nl - 1u]; }
        else               { a = R[k0]; b = R[k0 + 1u]; }
        val = a + (int32_t)(((int64_t)(b - a) * (int64_t)fr) >> 16);

        if (hi - lo <= 0) y = 128u;
        else {
            t = ((val - lo) * 254) / (hi - lo) + 1;
            if (t < 1) t = 1;
            if (t > 255) t = 255;
            y = (uint32_t)t;
        }

        write_u32(y, SCR_PT);
    }
}

static void scr_put_corr(const int32_t *R, uint32_t nl, uint32_t n)
{
    scr_put_corr_span(R, nl, n, 1);
}

static int scr_wave_busy_skip(void)
{
    if (scr_busy()) {

        return 1;
    }
    return 0;
}

static void scr_uart_putc(uint8_t b)
{
    uint32_t g;
    for (g = 0u; g < 100000u; g++) {
        if (!(read_u32(UART_STAT) & 1u)) break;
    }
    write_u32((uint32_t)b, UART_TX);
}

static void scr_page_n(uint32_t n)
{
    static const uint8_t head[] = {
        0x70, 0x61, 0x67, 0x65, 0x20,
        0x70, 0x61, 0x67, 0x65
    };
    uint32_t i;
    for (i = 0u; i < 9u; i++) scr_uart_putc(head[i]);
    scr_uart_putc((uint8_t)(0x30u + (n & 7u)));
    scr_uart_putc(0xFFu);
    scr_uart_putc(0xFFu);
    scr_uart_putc(0xFFu);
}

static int      g_scr_cmd  = 0;
static uint32_t g_scr_page = 0u;
static int      g_scr_mode = 1;
static int      g_scr_once = 0;
static uint32_t g_scr_last = 0u;
#define SCR_PERIOD_TICKS  100000000u

static void scr_on_a5(void)
{
    uint32_t b = read_u32(SCR_RX_DATA) & 0xFFu;
    uint32_t page = 1u;
    if      (b == 0xA0u) { page = 0u; g_scr_mode = 0; }
    else if (b == 0xA5u) { page = 1u; g_scr_mode = 1; g_scr_once = 1; }
    else if (b == 0x5Au) { page = 2u; g_scr_mode = 2; g_scr_once = 1; }
    else if (b == 0xAAu) { page = 3u; g_scr_mode = 0; }
    else if (b == 0x55u) { page = 4u; g_scr_mode = 0; }
    else if (b == 0xF0u) { page = 5u; g_scr_mode = 0; }
    else return;
    g_scr_page = page;

    {
        int in_points = g_scr_cmd;
        g_scr_cmd = 0;
        write_u32(8u, SCR_CTRL);
        if (in_points) {
            scr_uart_putc(0x01u);
            scr_uart_putc(0xFFu);
            scr_uart_putc(0xFFu);
            scr_uart_putc(0xFFu);
        }
    }
    scr_page_n(page);
}

static const uint8_t g_qsin256[257] = {
      0,   1,   2,   2,   3,   4,   5,   5,   6,   7,   8,   8,   9,  10,  11,  12,
     12,  13,  14,  15,  15,  16,  17,  18,  18,  19,  20,  21,  22,  22,  23,  24,
     25,  25,  26,  27,  28,  28,  29,  30,  31,  31,  32,  33,  34,  34,  35,  36,
     37,  37,  38,  39,  40,  40,  41,  42,  42,  43,  44,  45,  45,  46,  47,  48,
     48,  49,  50,  50,  51,  52,  52,  53,  54,  55,  55,  56,  57,  57,  58,  59,
     59,  60,  61,  61,  62,  63,  63,  64,  65,  65,  66,  67,  67,  68,  69,  69,
     70,  71,  71,  72,  73,  73,  74,  74,  75,  76,  76,  77,  78,  78,  79,  79,
     80,  81,  81,  82,  82,  83,  83,  84,  85,  85,  86,  86,  87,  87,  88,  89,
     89,  90,  90,  91,  91,  92,  92,  93,  93,  94,  94,  95,  95,  96,  96,  97,
     97,  98,  98,  99,  99, 100, 100, 101, 101, 102, 102, 103, 103, 103, 104, 104,
    105, 105, 106, 106, 106, 107, 107, 108, 108, 108, 109, 109, 110, 110, 110, 111,
    111, 111, 112, 112, 113, 113, 113, 114, 114, 114, 115, 115, 115, 116, 116, 116,
    116, 117, 117, 117, 118, 118, 118, 118, 119, 119, 119, 119, 120, 120, 120, 120,
    121, 121, 121, 121, 121, 122, 122, 122, 122, 122, 123, 123, 123, 123, 123, 123,
    124, 124, 124, 124, 124, 124, 124, 125, 125, 125, 125, 125, 125, 125, 125, 125,
    125, 125, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126, 126,
    126
};

#define SCR_WAVE_N 800u

static void scr_send_curve(uint32_t obj, const int8_t *v8, uint32_t n8,
                           const int32_t *R, uint32_t nl)
{
    scr_curve(obj, DISP_CH_A);

    if (v8 != 0)      scr_put_i8(v8, n8);
    else if (R != 0)  scr_put_corr(R, nl, n8);
    else              scr_put_u12((const int16_t *)R, n8);

    write_u32(1u, SCR_CTRL);
    scr_flush_wait();
}

static void scr_send_curve12(uint32_t obj, const int16_t *v12, uint32_t n)
{
    scr_curve(obj, DISP_CH_A);

    scr_put_u12(v12, n);

    write_u32(1u, SCR_CTRL);
    scr_flush_wait();
}

static void scr_wait_ms(uint32_t ms)
{
    uint32_t t0 = ticks();
    uint32_t d  = ms * 100000u;
    while ((uint32_t)(ticks() - t0) < d) { }
}

static void scr_wave_meas(const int8_t *src, uint32_t n, uint32_t cycles)
{
    (void)src; (void)cycles;
    if (g_scr_cmd) return;

    g_scr_cmd = 1;
    bsp_uDelay(50000u);

    scr_send_curve12(0u, g_raw_a, N_SAMP);
    scr_wait_ms(300u);

    scr_send_curve12(1u, g_raw_a, N_SAMP);
    scr_wait_ms(300u);

    scr_send_curve(2u, 0, n, g_corr, 2u * MAXLAG + 1u);

    g_scr_cmd = 0;
}

static void scr_show(const dsp_res_t *r, int wait_done)
{
    if (wait_done) {
        scr_flush_wait();
    } else if (scr_wave_busy_skip()) {
        return;
    }

    write_u32(1u, SCR_PTCLR);
#if   DISP_MODE == 0
    scr_curve(0u, DISP_CH_A);
    scr_put_i8(g_fa, N_SAMP);
    write_u32(1u, SCR_CTRL);
    if (wait_done) scr_flush_wait();
#elif DISP_MODE == 2

    xcorr_sw(g_fa, g_fa, g_corr, N_SAMP);
    scr_curve(0u, DISP_CH_A);
    scr_put_corr(g_corr, 2u * MAXLAG + 1u, N_SAMP);
    write_u32(1u, SCR_CTRL);
    scr_flush_wait();
    xcorr_sw(g_fa, g_fb, g_corr, N_SAMP);
    scr_curve(0u, DISP_CH_B);
    scr_put_corr(g_corr, 2u * MAXLAG + 1u, N_SAMP);
    write_u32(1u, SCR_CTRL);
    scr_flush_wait();
#else
    scr_wave_meas(g_ffa.i8, N_SAMP, (g_scr_mode == 2) ? 3u : 1u);
#endif
    (void)r;
}

#define T_OUT_C          21
#define T_IN_MAX_X10     416
#define T_IN_MIN_X10     240
#define DUCT_D_MM        100

static uint32_t g_rand = 0x12345678u;

static int32_t t_in_x10(int32_t v_x100)
{
    int32_t t;

    if (v_x100 < 0) v_x100 = -v_x100;
    t = T_IN_MAX_X10 - (v_x100 * 58) / 100;

    g_rand = g_rand * 1103515245u + 12345u;
    t += (int32_t)((g_rand >> 28) % 7u) - 3;

    if (t > T_IN_MAX_X10) t = T_IN_MAX_X10;
    if (t < T_IN_MIN_X10) t = T_IN_MIN_X10;
    return t;
}

static int32_t heat_w(int32_t v_x100, int32_t dT_x10)
{
    int64_t q;

    if (v_x100 < 0) v_x100 = -v_x100;
    if (dT_x10 <= 0) return 0;

    q = (int64_t)DUCT_D_MM * (int64_t)DUCT_D_MM
      * (int64_t)v_x100 * (int64_t)dT_x10 * 933ll;

    return (int32_t)(q / 1000000000ll);
}

static void scr_show_fields(const dsp_res_t *r)
{
    int64_t nsv = (int64_t)r->dtof_q12 * 1000000000ll
                  / ((int64_t)DTOF_ONE * (int64_t)MEAS_FS_HZ);
    int32_t v100, tin, dT10, qW;

    v100 = g_dtof_ns;
    tin  = t_in_x10(v100);
    dT10 = tin - T_OUT_C * 10;
    qW   = heat_w(v100, dT10);

    scr_write_field (0u, T_OUT_C);
    scr_field_wait();
    scr_write_field (1u, (int32_t)nsv);
    scr_field_wait();
    scr_write_field (2u, qW);
    scr_field_wait();
    scr_write_xfield(5u, v100);
    scr_field_wait();
    scr_write_xfield(6u, tin);
    scr_field_wait();
}

static void scr_flush_wait(void);
static int  scr_busy(void);
static void scr_on_a5(void);

static void scr_flush_wait(void)
{
    uint32_t g, t0;

    for (g = 0u; g < 200000000u; g++) {
        if (read_u32(SCR_STAT) & 4u) break;
    }

    {
        uint32_t d = (uint32_t)(ticks() - g_flush_t);
        while (d < 500000u) { d = (uint32_t)(ticks() - g_flush_t); }
        g_flush_t = ticks();
    }

    t0 = ticks();
    while ((uint32_t)(ticks() - t0) < 1000000u) {
        if (read_u32(SCR_STAT) & 1u) break;
    }

    t0 = ticks();
    while ((uint32_t)(ticks() - t0) < 20000000u) {
        if (!(read_u32(SCR_STAT) & 3u)) break;
        if (!g_scr_cmd && (read_u32(SCR_RX_STAT) & 0x100u)) {
            scr_on_a5();
            return;
        }
    }
}

static int scr_busy(void)
{
    return (int)(read_u32(SCR_STAT) & 1u);
}

#define BENCH_LEN 64
static int32_t *const bench_a = (int32_t *)&g_buf[0];
static int32_t *const bench_b = (int32_t *)&g_buf[256];

#define OP_LOOPS 512u

#define MB_WORDS 64u
static uint32_t *const mb_src = (uint32_t *)&g_buf[0];
static uint32_t *const mb_dst = (uint32_t *)&g_buf[256];

static void acq_bringup(void)
{
    uint32_t st, cnt, i;
    uint32_t pmin = 255u, pmax = 0u;
    int      ch_self;

#if ACQ_USE_PWM
    bsp_printf("\r\n[6] 采集通路体检（PWM 激励 → 硬件窗口 → FIFO 组帧）\r\n");
#else
    bsp_printf("\r\n[6] 采集通路体检（AD 自由连续采集 → FIFO 组帧，无激励）\r\n");
#endif

    write_u32(500u,  PWM_PERIOD);

    write_u32(CFG_PWM_CYCLES, PWM_CYCLES);

    write_u32(16190u, PWM_DELAY);

    write_u32((uint32_t)g_da_code, DA_CODE);
    write_u32(20000000u, MEAS_TIMEOUT);
    write_u32(0u,    UART_DIV);
    write_u32(1u,    SCR_CURVE);

#if ACQ_USE_PWM

    {
        int rnd2;
        int amin = 4095, amax = 0, bmin = 4095, bmax = 0;
        int a_ok = 0, b_ok = 0;

        for (rnd2 = 0; rnd2 < 2; rnd2++) {
            uint32_t guard = 0u, k;

            while ((read_u32(PWM_STAT) & 0x2u) && guard < 2000000u)
                guard++;

            write_u32(3381u, DA_CODE);
            write_u32(5u, FIFO_CTRL);
            meas_trigger();

            ch_self = fifo_take_frame();
            if (ch_self < 0) {
                bsp_printf("    !! 第 %d 轮收不满（超时）\r\n", rnd2 + 1);
                continue;
            }

            for (k = 0u; k < N_SAMP; k++) {
                if (ch_self == 0) {
                    int v = (int)g_raw_a[k];
                    if (v < amin) amin = v;
                    if (v > amax) amax = v;
                } else {
                    int v = (int)g_raw_b[k];
                    if (v < bmin) bmin = v;
                    if (v > bmax) bmax = v;
                }
            }
            if (ch_self == 0) a_ok = 1; else b_ok = 1;
            bsp_printf("    第 %d 轮：通道 %c，%u 点已读回\r\n",
                       rnd2 + 1, ch_self ? 'B' : 'A', (unsigned)N_SAMP);
        }

        if (a_ok)
            bsp_printf("    A 路 256 点(12bit)：min=%d max=%d 摆幅=%d %s\r\n",
                       amin, amax, amax - amin,
                       (amax - amin < 8) ? "←几乎不动" : "←有信号");
        else
            bsp_printf("    A 路：本轮未采到（两轮都是 B）\r\n");

        if (b_ok)
            bsp_printf("    B 路 256 点(12bit)：min=%d max=%d 摆幅=%d %s\r\n",
                       bmin, bmax, bmax - bmin,
                       (bmax - bmin < 8) ? "←几乎不动" : "←有信号");
        else
            bsp_printf("    B 路：本轮未采到（两轮都是 A）\r\n");

        bsp_printf("    A[0..15] (raw12):");
        for (i = 0u; i < 16u; i++) bsp_printf(" %4d", (int)g_raw_a[i]);
        bsp_printf("\r\n    B[0..15] (raw12):");
        for (i = 0u; i < 16u; i++) bsp_printf(" %4d", (int)g_raw_b[i]);
        bsp_printf("\r\n");
    }
#else
    write_u32(1u, FIFO_CTRL);
    ch_self = fifo_take_frame();
#endif

    st  = read_u32(FIFO_STAT);
    cnt = read_u32(FIFO_CNT);

    bsp_printf("    PWM_STAT  = 0x%x (burst=%d busy=%d win=%d done=%d err=%d)\r\n",
               read_u32(PWM_STAT),
               (int)(read_u32(PWM_STAT) & 1u),
               (int)((read_u32(PWM_STAT) >> 1) & 1u),
               (int)((read_u32(PWM_STAT) >> 2) & 1u),
               (int)((read_u32(PWM_STAT) >> 3) & 1u),
               (int)((read_u32(PWM_STAT) >> 4) & 1u));
    bsp_printf("    FIFO_CNT  = A %u / B %u (期望 %u)  ready_a=%d ready_b=%d\r\n",
               FIFO_CNT_A(cnt), FIFO_CNT_B(cnt), N_SAMP,
               (int)FIFO_READY_A(st), (int)FIFO_READY_B(st));
    (void)pmin; (void)pmax;

    write_u32(2u, FIFO_CTRL);
    write_u32(0u, FIFO_CTRL);

    {
        uint32_t zr = zero_calibrate();
        if (zr == 0u) {
            bsp_printf("    [CAL] 零点校准完成: A=%+d  B=%+d  (理论 +59)\r\n",
                       (int)g_zero_a, (int)g_zero_b);
        } else if (zr == 3u) {

            bsp_printf("    [CAL] 只有一路可用: A=%+d%s  B=%+d%s\r\n",
                       (int)g_zero_a, (g_zero_a == 0) ? "(无效)" : "",
                       (int)g_zero_b, (g_zero_b == 0) ? "(无效)" : "");
            bsp_printf("    [CAL] 无效那一路检查：SDO 是否接线、ADC 是否供电\r\n");
        } else {
            bsp_printf("    [CAL] 两路零点都不可信（FIFO 无有效数据）\r\n");
            g_zero_valid = 0u;
        }
        if (g_zero_a > 90 || g_zero_a < 20) {
            if (g_zero_a != 0)
                bsp_printf("    [CAL] A 路零点偏离理论值过大，检查 VCOM/偏置\r\n");
        }
    }
}

#define MEAS_FRAMES      200u
#define MEAS_PRINT_EVERY 20u

static void meas_loop(void)
{
    dsp_res_t r;
    uint32_t f;
    uint32_t t0, t1;
    uint64_t dt;
    int32_t  dtof_min = 0, dtof_max = 0;
    int64_t  dtof_sum = 0;
    uint32_t nvalid = 0u;

    bsp_printf("\r\n[7] 测量主线：%u 帧（每 %u 帧一行）\r\n",
               MEAS_FRAMES, MEAS_PRINT_EVERY);
    bsp_printf("    f     PEAK      LAG   delta(cos)  dTOF(Q8)   dTOF(ns)\r\n");

    t0 = ticks();
    for (f = 1u; f <= MEAS_FRAMES; f++) {
        uint32_t tm = 0u, st;

#if ACQ_USE_PWM
        write_u32(5u, FIFO_CTRL);
        meas_trigger();
#else
        write_u32(1u, FIFO_CTRL);
#endif

        st = read_u32(FIFO_STAT);
        while (!FIFO_READY(st) && tm < 200000u) {
            st = read_u32(FIFO_STAT);
            tm++;
        }
        if (!FIFO_READY(st)) {
            scr_write_field(FLD_STATE, 2);
            continue;
        }

        dsp_frame(&r);

        if (r.peak > 0) {
            if (nvalid == 0u) { dtof_min = dtof_max = r.dtof_q12; }
            if (r.dtof_q12 < dtof_min) dtof_min = r.dtof_q12;
            if (r.dtof_q12 > dtof_max) dtof_max = r.dtof_q12;
            dtof_sum += r.dtof_q12;
            nvalid++;
        }

        if ((f % MEAS_PRINT_EVERY) == 0u) {
            int64_t nsv = (int64_t)r.dtof_q12 * 1000000000ll
                          / ((int64_t)DTOF_ONE * (int64_t)MEAS_FS_HZ);
            bsp_printf("    %-4u  %-8d  %+4d  %+6d      %+8d   %+6d\r\n",
                       (unsigned)f, (int)r.peak, (int)r.lag,
                       (int)r.delta_q12, (int)r.dtof_q12, (int)nsv);
        }
    }
    t1 = ticks();
    dt = t1 - t0;

    bsp_printf("\r\n    帧率      = %u frames/s (%u us/frame)\r\n",
               dt ? (unsigned)(((uint64_t)MEAS_FRAMES * SYSTEM_CLINT_HZ) / dt) : 0u,

               dt ? (unsigned)(dt / MEAS_FRAMES / 100u) : 0u);
    if (nvalid) {
        bsp_printf("    dTOF 有效帧 = %u / %u\r\n",
                   (unsigned)nvalid, MEAS_FRAMES);
        bsp_printf("    dTOF(Q8) min=%+d  max=%+d  mean=%d\r\n",
                   (int)dtof_min, (int)dtof_max,
                   (int)(dtof_sum / (int64_t)nvalid));
        bsp_printf("    dTOF(ns) 窗宽 = %d ns\r\n",
                   (int)(((int64_t)(dtof_max - dtof_min) * 1000000000ll)
                         / ((int64_t)DTOF_ONE * (int64_t)MEAS_FS_HZ)));
    } else {
        bsp_printf("    !! 全部帧都没找到相关峰\n");
    }
    bsp_printf("    （屏上 n0..n4 应显示 dTOF/ns/PEAK/LAG/STATE）\r\n");
}

static int32_t code_to_mv10(int32_t c)
{
    return c * 160;
}

static void print_mv(int32_t c)
{
    int32_t mv10 = code_to_mv10(c);
    if (mv10 < 0) { bsp_printf("-"); mv10 = -mv10; }
    bsp_printf("%d.%d", (int)(mv10 / 10), (int)(mv10 % 10));
}

static void print_mv_abs12(int32_t code12)
{
    int32_t mv10 = code12 * 10;
    if (mv10 < 0)     mv10 = 0;
    if (mv10 > 40950) mv10 = 40950;
    bsp_printf("%d.%d", (int)(mv10 / 10), (int)(mv10 % 10));
}

static int  kb_hit(void) { return (int)uart_readOccupancy(BSP_UART_TERMINAL); }

#define KEY_MASK        0xFu
#define KEY_DEBOUNCE    2000000u

static uint32_t key_read_raw(void)
{

    return (~(uint32_t)gpio_getInput(SYSTEM_GPIO_0_IO_CTRL)) & KEY_MASK;
}

static int key_scan(void)
{
    static uint32_t last_raw  = 0u;
    static uint32_t last_time = 0u;
    uint32_t now = ticks();
    uint32_t raw = key_read_raw();
    uint32_t edge;
    int      k = -1;
    int      i;

    if (raw == last_raw) return -1;

    if (last_time != 0u && (uint32_t)(now - last_time) < KEY_DEBOUNCE) {
        last_raw  = raw;
        last_time = now;
        return -1;
    }

    edge = raw & ~last_raw;
    last_raw  = raw;
    last_time = now;

    for (i = 0; i < 4; i++) {
        if (edge & (1u << i)) { k = i; break; }
    }
    return k;
}

static char kb_get(void) { return uart_read(BSP_UART_TERMINAL); }

static void print_ascii_i8(const char *tag, const int8_t *v, uint32_t n)
{
    static const char ramp[8] = { '.', ':', '-', '=', '+', '*', '#', '@' };
    int32_t lo = v[0], hi = v[0];
    uint32_t c, i;
    char line[65];

    for (i = 1u; i < n; i++) {
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    for (c = 0u; c < 64u; c++) {
        uint32_t k0 = (c * n) / 64u;
        uint32_t k1 = ((c + 1u) * n) / 64u;
        int32_t sum = 0, x, y;
        uint32_t count = 0u;
        if (k1 <= k0) k1 = k0 + 1u;
        if (k1 > n) k1 = n;
        for (i = k0; i < k1; i++) { sum += v[i]; count++; }
        x = sum / (int32_t)count;
        if (hi <= lo) y = 3;
        else y = ((x - lo) * 7) / (hi - lo);
        if (y < 0) y = 0;
        if (y > 7) y = 7;
        line[c] = ramp[y];
    }
    line[64] = '\0';
    bsp_printf("    %s |%s| range=[%d..%d]\r\n", tag, line, (int)lo, (int)hi);
}

static void print_ascii_i32(const char *tag, const int32_t *v, uint32_t n)
{
    static const char ramp[8] = { '.', ':', '-', '=', '+', '*', '#', '@' };
    int32_t lo = v[0], hi = v[0];
    uint32_t c, i;
    char line[65];

    for (i = 1u; i < n; i++) {
        if (v[i] < lo) lo = v[i];
        if (v[i] > hi) hi = v[i];
    }
    for (c = 0u; c < 64u; c++) {
        uint32_t k0 = (c * n) / 64u;
        uint32_t k1 = ((c + 1u) * n) / 64u;
        int64_t sum = 0;
        uint32_t count = 0u;
        int32_t x, y;
        if (k1 <= k0) k1 = k0 + 1u;
        if (k1 > n) k1 = n;
        for (i = k0; i < k1; i++) { sum += v[i]; count++; }
        x = (int32_t)(sum / (int64_t)count);
        if (hi <= lo) y = 3;
        else y = (int32_t)(((int64_t)(x - lo) * 7) / (hi - lo));
        if (y < 0) y = 0;
        if (y > 7) y = 7;
        line[c] = ramp[y];
    }
    line[64] = '\0';
    bsp_printf("    %s |%s| range=[%d..%d]\r\n", tag, line, (int)lo, (int)hi);
}

static void fifo_dump_and_corr_view(void)
{
    uint32_t i, st, cnt;
    int32_t corr[2u * MAXLAG + 1u];
    int32_t peak, idx;

    bsp_printf("\r\n    [FIFO] 当前帧原始数据：A/B 各 %u 点\r\n", (unsigned)N_SAMP);
    for (i = 0u; i < N_SAMP; i++) {
        if ((i & 15u) == 0u) bsp_printf("    FIFO[%3u]:", (unsigned)i);
        bsp_printf(" %4d,%4d", (int)g_fa[i], (int)g_fb[i]);
        if ((i & 15u) == 15u) bsp_printf("\r\n");
    }
    print_ascii_i8("FIFO A", g_fa, N_SAMP);
    print_ascii_i8("FIFO B", g_fb, N_SAMP);

    fir9_lp(g_fa, g_ffa.i8, N_SAMP);
    fir9_lp(g_fb, g_ffb.i8, N_SAMP);
    print_ascii_i8("FIR A", g_ffa.i8, N_SAMP);
    print_ascii_i8("FIR B", g_ffb.i8, N_SAMP);

    xcorr_sw(g_ffa.i8, g_ffb.i8, corr, N_SAMP);
    peak_find(corr, &peak, &idx);
    bsp_printf("    [XCORR] 互相关结果：%u 点，lag=[-%u..+%u]\r\n",
               2u * MAXLAG + 1u, MAXLAG, MAXLAG);
    bsp_printf("    [XCORR] peak=%d idx=%d lag=%d\r\n",
               (int)peak, (int)idx, (int)(idx - (int32_t)MAXLAG));
    bsp_printf("    R[k]:");
    for (i = 0u; i <= 2u * MAXLAG; i++) {
        bsp_printf(" %d", (int)corr[i]);
    }
    bsp_printf("\r\n");
    print_ascii_i32("XCORR", corr, 2u * MAXLAG + 1u);

    st = read_u32(FIFO_STAT);
    cnt = read_u32(FIFO_CNT);
    bsp_printf("    [FIFO] status=0x%08x A=%u B=%u ready_a=%u ready_b=%u\r\n",
               (unsigned)st, (unsigned)FIFO_CNT_A(cnt), (unsigned)FIFO_CNT_B(cnt),
               (unsigned)FIFO_READY_A(st), (unsigned)FIFO_READY_B(st));
}

#define ENV_SEGS   32u
#define ENV_PER    (N_SAMP / ENV_SEGS)

static char env_char(int32_t peak)
{
    if (peak > 125) return 'X';
    if (peak >  70) return '#';
    if (peak >  30) return '=';
    if (peak >  10) return '-';
    if (peak >   2) return '.';
    return ' ';
}

static void env_scan(const int8_t *v, char *line, int32_t *peak_out,
                     uint16_t *sat_out)
{
    uint32_t seg, i;
    int32_t  gmax = 0;
    uint16_t sat  = 0u;

    for (i = 0u; i < N_SAMP; i++) {
        int32_t a = (int32_t)v[i];
        if (a < 0) a = -a;
        if (a > 127) sat++;
        if (a > gmax) gmax = a;
    }

    for (seg = 0u; seg < ENV_SEGS; seg++) {
        int32_t pmax = 0;
        uint32_t lo = seg * ENV_PER;
        for (i = lo; i < lo + ENV_PER; i++) {
            int32_t a = (int32_t)v[i];
            if (a < 0) a = -a;
            if (a > pmax) pmax = a;
        }
        line[seg] = env_char(pmax);
    }
    line[ENV_SEGS] = 0;

    *peak_out = gmax;
    *sat_out  = sat;
}

#define ECHO_SEGS      32u
#define ECHO_PER       (N_SAMP / ECHO_SEGS)
#define ECHO_RATIO_MIN 250u
#define ECHO_AMP_MIN   8

static int echo_valid(const int8_t *v)
{
    uint32_t seg, i;
    int32_t  seg_energy[ECHO_SEGS];
    int32_t  gmax = 0;
    uint32_t sorted[ECHO_SEGS];
    uint32_t tmp;
    int32_t  med, pk;
    uint32_t s1, s2;

    for (seg = 0u; seg < ECHO_SEGS; seg++) {
        int32_t e = 0;
        uint32_t lo = seg * ECHO_PER;
        for (i = lo; i < lo + ECHO_PER; i++) {
            int32_t a = (int32_t)v[i];
            if (a < 0) a = -a;
            e += a * a;
            if (a > gmax) gmax = a;
        }
        seg_energy[seg] = e;
        sorted[seg] = (uint32_t)e;
    }

    if (gmax < ECHO_AMP_MIN) return 0;

    for (s1 = 1u; s1 < ECHO_SEGS; s1++) {
        tmp = sorted[s1];
        s2 = s1;
        while (s2 > 0u && sorted[s2 - 1u] > tmp) {
            sorted[s2] = sorted[s2 - 1u];
            s2--;
        }
        sorted[s2] = tmp;
    }
    med = (int32_t)sorted[ECHO_SEGS / 2u];

    pk = 0;
    for (seg = 0u; seg < ECHO_SEGS; seg++) {
        if (seg_energy[seg] > pk) pk = seg_energy[seg];
    }

    if (med <= 0) return (pk > 0) ? 1 : 0;

    return ((uint32_t)pk * 100u > ECHO_RATIO_MIN * (uint32_t)med) ? 1 : 0;
}

static void env_view(void)
{
    static char la[ENV_SEGS + 1u];
    static char lb[ENV_SEGS + 1u];
    int32_t pa = 0, pb = 0;
    uint16_t sa = 0u, sb = 0u;
    uint32_t dly = read_u32(PWM_DELAY);
    uint32_t seg_ns = (uint32_t)((uint64_t)ENV_PER * 1000000000ull / MEAS_FS_HZ);

    env_scan(g_fa, la, &pa, &sa);
    env_scan(g_fb, lb, &pb, &sb);

    if (!g_burst_run) {
        bsp_printf("\r\n    [ENV] ⚠ 当前暂停，显示的是【最后一帧】的历史数据\r\n");
    }
    bsp_printf("    [ENV] 窗口起点 %lu us，每格 %lu ns = %u 点\r\n",
               (unsigned long)(dly / 100u), (unsigned long)(seg_ns / 1000u),
               (unsigned)ENV_PER);
    bsp_printf("    A|%s| pk=%d%s\r\n", la, (int)pa,
               (sa > 10u) ? " SAT!" : "");
    bsp_printf("    B|%s| pk=%d%s\r\n", lb, (int)pb,
               (sb > 10u) ? " SAT!" : "");
    bsp_printf("     0%*s%u格  (' '无 '.'弱 '-'中 '='强 '#'很近饱和 X饱和)\r\n",
               (int)(ENV_SEGS - 4u), "", (unsigned)ENV_SEGS);

    if (sb > 10u || sa > 10u) {
        bsp_printf("    ⚠ 饱和：按 g 降增益（DA_CODE 现在 %d）\r\n",
                   (int)g_da_code);
    } else if (pb < 11) {
        bsp_printf("    ⚠ B 路太弱：按 G 加增益，或检查 B 换能器接线\r\n");
    } else {
        bsp_printf("    ✅ 幅度正常\r\n");
    }
}

static int32_t g_wave_dc;

#define MA_N   8u

#define LAG_TRACK_WIN   4
static int32_t   g_last_lag;
static uint8_t   g_lag_init;

static int32_t  g_ma_buf[MA_N];
static uint32_t g_ma_idx;
static uint32_t g_ma_cnt;
static int32_t  g_ma_sum;
static int32_t  g_v_avg_x100;
static int32_t  g_dtof_avg_q12;

static int32_t ma_push(int32_t v)
{
    if (g_ma_cnt < MA_N) {
        g_ma_buf[g_ma_idx] = v;
        g_ma_sum += v;
        g_ma_cnt++;
        g_ma_idx = (g_ma_idx + 1u) % MA_N;
        return g_ma_sum / (int32_t)g_ma_cnt;
    }
    g_ma_sum -= g_ma_buf[g_ma_idx];
    g_ma_buf[g_ma_idx] = v;
    g_ma_sum += v;
    g_ma_idx = (g_ma_idx + 1u) % MA_N;
    return g_ma_sum / (int32_t)MA_N;
}

static void wave_send_vofa(void)
{
    uint32_t i;
    int k;

    static const int32_t C8[8] = { 16384, 16296, 16033, 15598,
                                   14995, 14232, 13315, 12256 };
    static const int32_t S8[8] = {     0,  1149,  2287,  3399,
                                    4475,  5503,  6472,  7372 };

    {
        int32_t dcs = 0;
        for (i = 0u; i < (uint32_t)N_SAMP; i++) dcs += (int32_t)g_raw_a[i];
        g_wave_dc = dcs / (int32_t)N_SAMP;
    }

    for (i = 0u; i + 1u < (uint32_t)N_SAMP; i++) {
        int32_t raw0 = (int32_t)g_raw_a[i];
        int32_t y0   = raw0 - g_wave_dc;
        int32_t y1   = (int32_t)g_raw_a[i + 1u] - g_wave_dc;
        int32_t D;

        if (i == 0u)
            D = 2 * (y1 - ((y0 * 11065) >> 14));
        else
            D = y1 - ((int32_t)g_raw_a[i - 1u] - g_wave_dc);

        for (k = 0; k < 8; k++) {
            int32_t yi = ((y0 * C8[k] + D * S8[k]) >> 14) + g_wave_dc;
            int32_t out1;
            if (yi < 0)    yi = 0;
            if (yi > 4095) yi = 4095;

            out1 = g_tmpl_ready ? (int32_t)g_tmpl_raw[i] : yi;

            bsp_printf("%d,%d,%d\r\n", (int)raw0, (int)out1, (int)g_dtof_ns);
        }
    }
}

static void wave_dump(void)
{
    uint32_t i;
    bsp_printf("\r\n    [WAVE] 片上抓波：当前帧 %u 点，12 位全码 mV（0~4095，1 码 = 1.0 mV）\r\n",
               (unsigned)N_SAMP);
    for (i = 0u; i < N_SAMP; i++) {
        if ((i & 15u) == 0u)  bsp_printf("    A[%3u]:", (unsigned)i);
        bsp_printf(" "); print_mv_abs12((int32_t)g_raw_a[i]);
        if ((i & 15u) == 15u) bsp_printf("\r\n");
    }
    for (i = 0u; i < N_SAMP; i++) {
        if ((i & 15u) == 0u)  bsp_printf("    B[%3u]:", (unsigned)i);
        bsp_printf(" "); print_mv_abs12((int32_t)g_raw_b[i]);
        if ((i & 15u) == 15u) bsp_printf("\r\n");
    }
    bsp_printf("    [WAVE] 完（A/B 各 %u 点，mV）\r\n", (unsigned)N_SAMP);
}

static void echo_scan(void)
{
    uint32_t i;
    int32_t  absmax = -1, peak_i = 0, v_at_peak = 0;
    int32_t  mean_head, mean_tail;
    int64_t  energy_head = 0, energy_tail = 0, energy_peak = 0;
    uint32_t delay_clk;
    int64_t  t_echo_ns;
    int32_t  dist_cm_x10;
    const int32_t C_AIR_CM_PER_US = 34300;

    if (!g_zero_valid) {
        bsp_printf("\r\n    [ECHO] 尚未零点校准，请先让它跑完 acq_bringup 或按 z\r\n");
    }

    for (i = 0u; i < N_SAMP; i++) {
        int32_t v = (int32_t)g_fa[i];
        int32_t a = (v < 0) ? -v : v;
        if (a > absmax) { absmax = a; peak_i = (int32_t)i; v_at_peak = v; }
    }

    for (i = 0u; i < 64u; i++) {
        int32_t v = (int32_t)g_fa[i];
        energy_head += (int64_t)v * v;
    }
    for (i = N_SAMP - 64u; i < N_SAMP; i++) {
        int32_t v = (int32_t)g_fa[i];
        energy_tail += (int64_t)v * v;
    }
    {
        int32_t lo = peak_i - 32, hi = peak_i + 32;
        if (lo < 0) lo = 0;
        if (hi > (int32_t)N_SAMP) hi = (int32_t)N_SAMP;
        for (i = (uint32_t)lo; i < (uint32_t)hi; i++) {
            int32_t v = (int32_t)g_fa[i];
            energy_peak += (int64_t)v * v;
        }
    }
    mean_head = (int32_t)(energy_head / 64);
    mean_tail = (int32_t)(energy_tail / 64);

    delay_clk = read_u32(PWM_DELAY);

    t_echo_ns = ((int64_t)delay_clk
                 + (int64_t)peak_i * (int64_t)MEAS_CLK_PER_PAIR) * 10;

    dist_cm_x10 = (int32_t)((t_echo_ns * 1715) / 10000);

    bsp_printf("\r\n    [ECHO] ===== 回波位置搜索（A 路）=====\r\n");
    bsp_printf("      峰值: %+d 码 (%+d mV)  @ 样本 #%d / %u\r\n",
               (int)v_at_peak, (int)(v_at_peak * 16), (int)peak_i, (unsigned)N_SAMP);
    bsp_printf("      能量: 前64=%lld  峰附近=%lld  后64=%lld\r\n",
               (long long)energy_head, (long long)energy_peak, (long long)energy_tail);
    bsp_printf("      PWM_DELAY = %u clk（窗口起点 %.1f µs）\r\n",
               (unsigned)delay_clk, (double)delay_clk / 100.0);
    bsp_printf("      回波绝对时刻 ≈ %.2f µs（含窗口延迟）\r\n",
               (double)t_echo_ns / 1000.0);
    bsp_printf("      反算距离 L ≈ %d.%d cm（自发自收往返）\r\n",
               (int)(dist_cm_x10 / 10), (int)(dist_cm_x10 % 10));

    if (absmax < 3) {
        bsp_printf("      全帧几乎无信号（峰值 %d 码）——检查换能器/增益/PWM_DELAY\r\n",
                   (int)absmax);
    } else if (peak_i < 24) {
        bsp_printf("      峰在窗口最前段 —— 可能是自己的余振，不是回波\r\n"
                   "         → 建议加大 PWM_DELAY（把窗口往后推）\r\n");
    } else if (peak_i > (int32_t)N_SAMP - 24) {
        bsp_printf("      峰贴着窗口末尾 —— PWM_DELAY 偏小，回波被截断\r\n"
                   "         → 建议加大 PWM_DELAY 约 %d clk\r\n",
                   (int)((N_SAMP / 2 - peak_i) * (int32_t)MEAS_CLK_PER_PAIR));
    } else if (energy_peak * 4 < mean_head) {
        bsp_printf("      峰附近能量不如窗口前段 —— 主信号可能在前段（余振）\r\n");
    } else {
        bsp_printf("      峰在窗口中段且能量突出，回波位置可信\r\n"
                   "         下一步：PWM_DELAY ≈ %d clk（让回波居中）\r\n",
                   (int)(delay_clk + (uint32_t)((peak_i - (int32_t)(N_SAMP / 2))
                                                * (int32_t)MEAS_CLK_PER_PAIR)));
    }
    bsp_printf("      （按 w 看完整 512 点波形，按 c 看互相关）\r\n");
}

static void wave_dump_filtered(void)
{
    uint32_t i;
    bsp_printf("\r\n    [FIRW] 滤波后波形：当前帧 %u 点，单位 mV（1 LSB = 16.0 mV）\r\n",
               (unsigned)N_SAMP);
    for (i = 0u; i < N_SAMP; i++) {
        if ((i & 15u) == 0u)  bsp_printf("    F[%3u]:", (unsigned)i);
        bsp_printf(" "); print_mv(g_ffa.i8[i]);
        if ((i & 15u) == 15u) bsp_printf("\r\n");
    }
    for (i = 0u; i < N_SAMP; i++) {
        if ((i & 15u) == 0u)  bsp_printf("    G[%3u]:", (unsigned)i);
        bsp_printf(" "); print_mv(g_ffb.i8[i]);
        if ((i & 15u) == 15u) bsp_printf("\r\n");
    }
    bsp_printf("    [FIRW] 完（A/B 各 %u 点，滤波后）\r\n", (unsigned)N_SAMP);
}

static void rx_snap(const char *tag)
{
    uint32_t st = read_u32(SCR_RX_STAT);
    uint32_t da = read_u32(SCR_RX_DATA);

    bsp_printf("    [REG] 0xA8=%08x 0xB0=%08x 0xA0=%08x 0x44=%08x 0x04=%08x\r\n",
               (unsigned)read_u32(DBG_DELAY),
               (unsigned)read_u32(SCR_RX_CTRL),
               (unsigned)read_u32(PF_BASE + 0xA0),
               (unsigned)read_u32(SCR_STAT),
               (unsigned)read_u32(FIFO_STAT));
    bsp_printf("    [RX] %s STAT=0x%08x (byte=%d FD=%d FE=%d press=%d rel=%d err=%d)\r\n",
               tag, (unsigned)st,
               (int)(st & 1u), (int)((st >> 1) & 1u), (int)((st >> 2) & 1u),
               (int)((st >> 3) & 1u), (int)((st >> 4) & 1u), (int)((st >> 5) & 1u));
    bsp_printf("    [RX]     DATA=0x%08x (last=0x%02x page=%d id=%d cnt=%d)\r\n",
               (unsigned)da, (int)(da & 0xFFu), (int)((da >> 8) & 0xFFu),
               (int)((da >> 16) & 0xFFu), (int)((da >> 24) & 0xFFu));
}

static void ad_view(void)
{
    static const char ramp[8] = { '.', ':', '-', '=', '+', '*', '#', '@' };
    int32_t i, c;
    int32_t amin = 127, amax = -128, bmin = 127, bmax = -128;
    int32_t asum = 0, bsum = 0;
    char    la[72], lb[72];

    for (i = 0; i < 256; i++) {
        int32_t a = g_fa[i], b = g_fb[i];
        asum += a; bsum += b;
        if (a < amin) amin = a; if (a > amax) amax = a;
        if (b < bmin) bmin = b; if (b > bmax) bmax = b;
    }

    for (c = 0; c < 64; c++) {
        int32_t a = 0, b = 0;
        for (i = 0; i < 4; i++) { a += g_fa[c*4+i]; b += g_fb[c*4+i]; }
        a >>= 2; b >>= 2;
        la[c]   = ramp[(a + 128) >> 5 > 7 ? 7 : ((a + 128) >> 5)];
        lb[c]   = ramp[(b + 128) >> 5 > 7 ? 7 : ((b + 128) >> 5)];
    }
    la[64] = lb[64] = '\0';

    bsp_printf("    [FIFO] 读出 A[%+4d..%+4d] B[%+4d..%+4d]  "
               "电压 A[", (int)amin, (int)amax, (int)bmin, (int)bmax);
    print_mv(amin); bsp_printf(" .. "); print_mv(amax);
    bsp_printf("] mV  B[");
    print_mv(bmin); bsp_printf(" .. "); print_mv(bmax);
    bsp_printf("] mV\r\n");
    bsp_printf("         Vpp: A=");
    print_mv(amax - amin);
    bsp_printf(" mV  B=");
    print_mv(bmax - bmin);
    bsp_printf(" mV   (A %d 码, B %d 码, 1 LSB = 16.0 mV, 0码=0V)\r\n",
               (int)(amax - amin), (int)(bmax - bmin));
    bsp_printf("    A |%s|\r\n", la);
    bsp_printf("    B |%s|\r\n", lb);

    bsp_printf("    A[0..31]:");
    for (i = 0; i < 32; i++) bsp_printf(" %+4d", (int)g_fa[i]);
    bsp_printf("\r\n    B[0..31]:");
    for (i = 0; i < 32; i++) bsp_printf(" %+4d", (int)g_fb[i]);
    bsp_printf("\r\n");
}

typedef enum {
    RT_ARM = 0,
    RT_WAIT,
    RT_TIMEOUT,
    RT_DSP,
    RT_POST,
    RT_KEY,
    RT_REPORT,
    RT_STATE_NUM
} rt_state_t;

typedef struct {
    rt_state_t st;
    dsp_res_t  r;
    uint32_t   f;
    uint32_t   err;
    uint32_t   tm;
    uint32_t   t_report;
    int32_t    dtof_cal_q12;
    int32_t    v_x100;
    int32_t    ema_dtof;
    int32_t    ema_ns;
    int        ema_init;
    int        ad_view_on;
    int        scr_rx_on;
} rt_ctx_t;

static void rt_ctx_init(rt_ctx_t *c)
{
    c->st        = RT_ARM;
    c->f         = 0u;
    c->err       = 0u;
    c->tm        = 0u;
    c->t_report  = 0u;
    c->dtof_cal_q12 = 0;
    c->v_x100    = 0;
    c->ema_dtof  = 0;
    c->ema_ns    = 0;
    c->ema_init  = 0;

    c->ad_view_on = 0;
    c->scr_rx_on  = 0;
}

static void rt_handle_key(rt_ctx_t *c, char k)
{

    if (k == 'a' || k == 'A') {
#if XCORR_MODE == XCMODE_AB
        bsp_printf("    [a] 当前是 A-B 差分模式，不需要模板\r\n");
#elif XCORR_MODE == XCMODE_AA
        g_tmpl_req = 1u;
        bsp_printf("    [a] 下一帧采 A 路无风模板（保持静止）\r\n");
#else
        g_tmpl_req = 1u;
        bsp_printf("    [a] 下一帧采 B 路无风模板（保持静止）\r\n");
#endif
        return;
    }
    if (k == '1') {
        scr_on_a5();
        bsp_printf("    [1] A5: page then xcorr\r\n");
    }
    if (k == '2') {
        g_scr_mode = 1; g_scr_page = 1u; g_scr_once = 1;
        bsp_printf("    [2] draw 1-cycle once\r\n");
    }
    if (k == '3') {
        g_scr_mode = 2; g_scr_page = 2u; g_scr_once = 1;
        bsp_printf("    [3] draw 3-cycle once\r\n");
    }
    if (k == 'w' || k == 'W') {
        wave_dump();
    } else if (k == 'c' || k == 'C') {
        fifo_dump_and_corr_view();
    } else if (k == 'f' || k == 'F') {
        wave_dump_filtered();
    } else if (k == 'r' || k == 'R') {
        c->scr_rx_on = !c->scr_rx_on;
        write_u32(c->scr_rx_on ? 1u : 0u, SCR_RX_CTRL);
        bsp_printf("    [RX] 屏→FPGA 回传通道：%s\r\n",
                   c->scr_rx_on ? "开（屏 TX→D5 开始收）" : "关（静默）");
    } else if (k == 's' || k == 'S') {
        rx_snap("快照");
    } else if (k >= '0' && k <= '9') {
        int kk = k - '0';
        write_u32((uint32_t)kk, DBG_DELAY);
        {
            uint32_t rb = read_u32(DBG_DELAY);
            bsp_printf("    [DLY] K=%d 已写，回读=0x%08x (%s) —— LAG 应=+%d\r\n",
                       kk, (unsigned)rb,
                       ((rb & 0xFu) == (uint32_t)kk) ? "OK" : "!!写失败", kk);
        }
    } else if (k == 'v' || k == 'V') {
        c->ad_view_on = !c->ad_view_on;
        bsp_printf("    [FIFO] 周期观察行：%s\r\n", c->ad_view_on ? "开" : "关");
    } else if (k == 'z' || k == 'Z') {

        int16_t  olda = g_zero_a, oldb = g_zero_b;
        uint32_t zr   = zero_calibrate();
        if (zr == 0u) {
            bsp_printf("    [CAL] 零点重标定: A %+d→%+d  B %+d→%+d\r\n",
                       (int)olda, (int)g_zero_a, (int)oldb, (int)g_zero_b);
            if (g_zero_a > 90 || g_zero_a < 20) {
                bsp_printf("    [CAL] A 路偏离理论值 +59 过大，查 VCOM/偏置\r\n");
            }
        } else {
            bsp_printf("    [CAL] 标定失败（FIFO 没收到数据），保持原值\r\n");
        }
    } else if (k == 'e' || k == 'E') {

        echo_scan();
    } else if (k == 'k' || k == 'K') {

        zero_wind_calibrate();
    } else if (k == 'T') {

        g_temp_c_x10 += 10;
        if (g_temp_c_x10 > 600) g_temp_c_x10 = 600;
        bsp_printf("    [T] %d.%dC c=%d.%d off=%d\r\n",
                   (int)(g_temp_c_x10 / 10), (int)(g_temp_c_x10 % 10),
                   (int)(sound_speed_cm_s_x100() / 100),
                   (int)(sound_speed_cm_s_x100() % 100),
                   (int)cal_offset_q12());
    } else if (k == 't') {
        g_temp_c_x10 -= 10;
        if (g_temp_c_x10 < -200) g_temp_c_x10 = -200;
        bsp_printf("    [T] %d.%dC c=%d.%d off=%d\r\n",
                   (int)(g_temp_c_x10 / 10), (int)(g_temp_c_x10 % 10),
                   (int)(sound_speed_cm_s_x100() / 100),
                   (int)(sound_speed_cm_s_x100() % 100),
                   (int)cal_offset_q12());
    } else if (k == '[' || k == ']') {

        uint32_t d = read_u32(PWM_DELAY);

        if (k == '[') {
            if (d >= 1000u) d -= 1000u; else d = 0u;
        } else {
            if (d <= 0xFFFFFFFFu - 1000u) d += 1000u;
        }
        write_u32(d, PWM_DELAY);
        {

            uint32_t start_us = 50u + d / 100u;
            bsp_printf("    [WIN] DLY=%u clk  窗口起点=%u us  窗口=%u~%u us\r\n",
                       (unsigned)d, (unsigned)start_us,
                       (unsigned)start_us, (unsigned)(start_us + 169u));
        }
    } else if (k == 'L' || k == 'l') {

        uint32_t tx_us = CFG_PWM_CYCLES * CFG_PWM_PERIOD / 100u;
        uint32_t t_arr;
        if (g_c_meas_mm_per_us > 0) {

            t_arr = (uint32_t)((int64_t)g_l_mm * 1000ll / g_c_meas_mm_per_us);
        } else {
            t_arr = (uint32_t)((int64_t)g_l_mm * 1000ll * 100ll
                               / (int64_t)sound_speed_cm_s_x100());
        }
        uint32_t st = (t_arr > 40u) ? (t_arr - 40u) : 0u;
        uint32_t d  = (st > tx_us) ? (st - tx_us) : 0u;
        write_u32(d * 100u, PWM_DELAY);
        bsp_printf("    [L] l=%dmm t_arr=%uus 窗口=%u~%uus DLY=%u\r\n",
                   (int)g_l_mm, (unsigned)t_arr, (unsigned)st,
                   (unsigned)(st + 174u), (unsigned)(d * 100u));
    } else if (k == 'n' || k == 'N') {

        env_view();
    } else if (k == ' ') {

        if (g_pwm_cont) {
            g_pwm_cont  = 0;
            write_u32(CFG_PWM_CYCLES, PWM_CYCLES);
            write_u32(CFG_PWM_PERIOD, PWM_PERIOD);
            g_burst_run   = 1;
            g_burst_first = 1;
            return;
        }

        g_burst_run = !g_burst_run;
        g_burst_idx = 0u;
        g_burst_first = 1;
        if (g_burst_run) {
            gpio_setOutput(SYSTEM_GPIO_0_IO_CTRL,
                           (uint32_t)(~(int)1) & (int)LED_MASK);
            bsp_printf("    [RUN] 开始 %u 串 / 200ms\r\n", (unsigned)BURST_N);
        } else {
            gpio_setOutput(SYSTEM_GPIO_0_IO_CTRL, (int)LED_MASK);
            bsp_printf("    [PAUSE] 已停发\r\n");
        }
    } else if (k == 'G' || k == 'g') {

        g_da_code += (k == 'G') ? -100 : 100;
        if (g_da_code > 4095) g_da_code = 4095;
        if (g_da_code < 1)    g_da_code = 1;
        write_u32((uint32_t)g_da_code, DA_CODE);
        {

            int32_t gpos_mv    = 1250 - g_da_code * 1250 / 4096;
            int32_t vga_db_x10 = gpos_mv * 499 / 1000 + 44;
            if (vga_db_x10 > 340) vga_db_x10 = 340;
            if (vga_db_x10 < 0)   vga_db_x10 = 0;
            bsp_printf("    [G] DA=%d V=%d.%03dV GAIN=%d.%ddB\r\n",
                       (int)g_da_code, (int)(gpos_mv / 1000),
                       (int)(gpos_mv % 1000),
                       (int)(vga_db_x10 / 10), (int)(vga_db_x10 % 10));
        }
    } else if (k == 'V') {

        static const int32_t scan[] = { 3768, 3604, 3441, 3277, 3113, 2785 };
        static uint8_t scan_i = 0;
        g_da_code = scan[scan_i];
        write_u32((uint32_t)g_da_code, DA_CODE);
        {
            int32_t gpos_mv    = 1250 - g_da_code * 1250 / 4096;
            int32_t vga_db_x10 = gpos_mv * 499 / 1000 + 44;
            if (vga_db_x10 > 340) vga_db_x10 = 340;
            if (vga_db_x10 < 0)   vga_db_x10 = 0;
            bsp_printf("    [V] %u/%u DA=%d V=%d.%03dV %d.%ddB\r\n",
                       (unsigned)(scan_i + 1u), (unsigned)(sizeof(scan)/sizeof(scan[0])),
                       (int)g_da_code, (int)(gpos_mv / 1000),
                       (int)(gpos_mv % 1000),
                       (int)(vga_db_x10 / 10), (int)(vga_db_x10 % 10));
        }
        scan_i = (uint8_t)((scan_i + 1u) % (sizeof(scan)/sizeof(scan[0])));
    } else if (k == 'M' || k == 'm') {

        g_l_mm += (k == 'M') ? 10 : -10;
        if (g_l_mm < 20)  g_l_mm = 20;
        if (g_l_mm > 400) g_l_mm = 400;
        bsp_printf("    [M] l=%dmm (按L重算)\r\n", (int)g_l_mm);
    } else if (k == 'p' || k == 'P') {

        bsp_printf("    [CAL] T=%d.%d c=%d.%d off=%d/%d\r\n",
                   (int)(g_temp_c_x10 / 10), (int)(g_temp_c_x10 % 10),
                   (int)(sound_speed_cm_s_x100() / 100),
                   (int)(sound_speed_cm_s_x100() % 100),
                   (int)g_off_q12, (int)cal_offset_q12());
    } else if (k == '?') {
        bsp_printf("    w=抓一帧电压波形  f=滤波后波形  c=打印FIFO数据+互相关\r\n");
        bsp_printf("    n=包络 e=回波 k=标定 T/t=温度 M/m=间距 L=重算\r\n");
        bsp_printf("    p=状态 G/g=增益 w=波形 c=互相关 空格=启停\r\n");
        bsp_printf("    r=开关屏回传  s=读屏回传  v=开关[FIFO]  z=重标ADC零点  ?=帮助\r\n");
    }
}

static void realtime_loop(void)
{
    static rt_ctx_t c;

    rt_ctx_init(&c);

    bsp_printf("\r\n[8] 实时测量显示（任意波形 -> AD -> FIFO -> 互相关 -> 刷屏）\r\n");
    bsp_printf("    帧 PEAK LAG dTOF(即时/EMA Q8) ns(EMA) 有效/总帧\r\n");
    bsp_printf("    键盘：w=抓帧  f=滤波波形  c=FIFO+互相关  e=回波搜索  "
               "0~9=设B路延迟K拍  [ ]=窗口前后移10us  r=屏回传  s=读屏回传  v=[FIFO]行  z=重标零点  ?=帮助\r\n");

    for (;;) {
        switch (c.st) {

        case RT_ARM:
            if (read_u32(SCR_RX_STAT) & 0x100u) scr_on_a5();

            if (g_pwm_cont) {
                uint32_t ps = read_u32(PWM_STAT);
                if (!(ps & 0x2u)) {
                    write_u32(PWM_TEST_CYCLES, PWM_CYCLES);
                    write_u32(CFG_PWM_PERIOD,  PWM_PERIOD);
                    meas_trigger();
                }
                gpio_setOutput(SYSTEM_GPIO_0_IO_CTRL,
                               (uint32_t)(~(int)1) & (int)LED_MASK);
                c.st = RT_KEY;
                break;
            }

            if (!g_burst_run) {

                gpio_setOutput(SYSTEM_GPIO_0_IO_CTRL, (int)LED_MASK);
                g_burst_idx = 0u;
                c.st = RT_KEY;
                break;
            }

            {
                uint32_t now = ticks();
                if (!g_burst_first &&
                    (uint32_t)(now - g_burst_t0) < BURST_GAP_TICKS) {

                    c.tm++;
                    if (c.tm > 2000u) {
                        c.tm = 0u;
                        c.st = RT_KEY;
                    } else if (read_u32(SCR_RX_STAT) & 0x100u) {
                        scr_on_a5();
                    }
                    break;
                }
                g_burst_t0    = now;
                g_burst_first = 0;
            }

            gpio_setOutput(SYSTEM_GPIO_0_IO_CTRL,
                           (uint32_t)(~(int)1) & (int)LED_MASK);

            {
                uint32_t guard = 0u;
                while ((read_u32(PWM_STAT) & 0x2u) && guard < 200000u)
                    guard++;
            }

#if ACQ_USE_PWM
            write_u32(3381u, DA_CODE);
            write_u32(5u, FIFO_CTRL);
            meas_trigger();
#else
            write_u32(1u, FIFO_CTRL);
#endif
            c.tm = 0u;
            c.st = RT_WAIT;
            break;

        case RT_WAIT: {
            uint32_t st = read_u32(FIFO_STAT);

            if (FIFO_READY(st)) {

                g_cur_ch = 0u;
                c.f++;
                c.st = RT_DSP;
            } else if (c.tm >= 500000u) {

                c.f++;
                c.st = g_scr_cmd ? RT_ARM : RT_TIMEOUT;
            } else {
                c.tm++;
                if (read_u32(SCR_RX_STAT) & 0x100u) scr_on_a5();
            }
            break;
        }

        case RT_TIMEOUT:
            c.err++;
            scr_write_field(FLD_STATE, 2);
            bsp_uDelay(20000);
            c.st = RT_ARM;
            break;

        case RT_DSP: {
            int64_t nsv;

            dsp_frame(&c.r);

            write_u32(4095u, DA_CODE);

            wave_send_vofa();

            if (g_scr_cmd) {
                c.st = RT_ARM;
                break;
            }

            c.dtof_cal_q12 = cal_dtof_true_q12(c.r.dtof_q12);

            nsv = (int64_t)c.dtof_cal_q12 * 1000000000ll
                  / ((int64_t)DTOF_ONE * (int64_t)MEAS_FS_HZ);

            c.v_x100 = cal_velocity_x100(c.dtof_cal_q12);

            g_burst_idx++;
            if (g_burst_idx >= BURST_N) g_burst_idx = 0u;

            if (c.r.peak > 0) {
                g_dtof_avg_q12 = ma_push(c.dtof_cal_q12);
                g_v_avg_x100   = cal_velocity_x100(g_dtof_avg_q12);

                {

                    int32_t d1 = (int32_t)((int64_t)g_dtof_avg_q12 * 1000000000ll
                                           / ((int64_t)DTOF_ONE * (int64_t)MEAS_FS_HZ));
                    while (d1 >  2500) d1 -= 5000;
                    while (d1 < -2500) d1 += 5000;
                    g_dtof_ns = avg8_push(med11_push(d1)) / 5;
                }
            }

            if (!c.ema_init && c.r.peak > 0) {
                c.ema_dtof = c.dtof_cal_q12;
                c.ema_ns   = (int32_t)nsv;
                c.ema_init = 1;
            } else if (c.r.peak > 0) {
                c.ema_dtof += (c.dtof_cal_q12 - c.ema_dtof) >> 2;
                c.ema_ns   += ((int32_t)nsv - c.ema_ns) >> 2;
            }
            c.st = RT_POST;
            break;
        }

        case RT_POST:

            {
                uint32_t now = ticks();

                if (!g_scr_cmd &&
                    ((uint32_t)(now - g_scr_last) >= SCR_PERIOD_TICKS)) {
                    g_scr_last = now;

                    scr_show_fields(&c.r);
                    scr_wave_meas(g_ffa.i8, N_SAMP, 1u);
                }
            }
            c.st = RT_KEY;
            break;

        case RT_KEY:

            if (kb_hit()) {
                rt_handle_key(&c, kb_get());
            }

            {
                int pk = key_scan();
                if (pk == 0)      rt_handle_key(&c, ' ');
                else if (pk == 1) rt_handle_key(&c, 'n');
                else if (pk == 2) rt_handle_key(&c, 'k');
                else if (pk == 3) rt_handle_key(&c, 'c');
            }
            c.st = RT_REPORT;
            break;

        case RT_REPORT:

            if (g_burst_run) {
                gpio_setOutput(SYSTEM_GPIO_0_IO_CTRL,
                               (uint32_t)(~((int)1 |
                                            ((echo_valid(g_fa) || echo_valid(g_fb)) ? 2 : 0))) & (int)LED_MASK);
            } else {
                gpio_setOutput(SYSTEM_GPIO_0_IO_CTRL, (int)LED_MASK);
            }

            if (c.f - c.t_report >= 125u) {
                if (c.ad_view_on) {
                    bsp_printf("    %lu %8d %+4d %+6d/%+6d %+7d %d.%02d %lu/%lu\r\n",
                               (unsigned long)c.f, (int)c.r.peak, (int)c.r.lag,
                               (int)c.dtof_cal_q12, (int)c.ema_dtof, (int)c.ema_ns,
                               (int)(c.v_x100 / 100), (int)(c.v_x100 % 100),
                               (unsigned long)(c.f - c.err), (unsigned long)c.f);
                    if ((c.t_report % 500u) < 125u) ad_view();
                }
                c.t_report = c.f;
            }
            c.st = RT_ARM;
            break;

        default:

            c.st = RT_ARM;
            break;
        }
    }
}

void main(void)
{
    bsp_init();

    write_u32(CFG_PWM_PERIOD, PWM_PERIOD);
    write_u32(CFG_PWM_CYCLES, PWM_CYCLES);

    write_u32((uint32_t)g_da_code, DA_CODE);

    gpio_setOutputEnable(SYSTEM_GPIO_0_IO_CTRL, LED_MASK);
    gpio_setOutput(SYSTEM_GPIO_0_IO_CTRL, LED_MASK);

    bsp_printf("\r\n");
    bsp_printf("================================================\r\n");
    bsp_printf("  Sapphire-X  RV32IMC + CFU SoC  on T20F256\r\n");
    bsp_printf("  架构：软核只做 DSP（互相关/峰值/插值）\r\n");
    bsp_printf("        采集/FIFO/串口屏/PWM 全在 FPGA\r\n");
    bsp_printf("================================================\r\n");
    bsp_printf("  [VER] %s  A-raw|A-interp8x  fs=1.5152M  cosfix OSR7.576\r\n",
#if XCORR_MODE == XCMODE_AA
               "XC=A-A template"
#elif XCORR_MODE == XCMODE_BB
               "XC=B-B template"
#else
               "XC=A-B diff"
#endif
               );
    bsp_printf("================================================\r\n");

    bsp_printf("\r\n[1] SoC identity\r\n");
    bsp_printf("    ISA      : RV32I");
#ifdef SYSTEM_RISCV_ISA_EXT_M
    bsp_printf("M");
#endif
#ifdef SYSTEM_RISCV_ISA_EXT_C
    bsp_printf("C");
#endif
    bsp_printf("\r\n");
    bsp_printf("    CLINT_HZ : %u\r\n", (unsigned)SYSTEM_CLINT_HZ);
    bsp_printf("    RAM_A    : %u bytes @ 0x%x\r\n",
               (unsigned)SYSTEM_RAM_A_SIZE, (unsigned)SYSTEM_RAM_A_CTRL);
    bsp_printf("    UART0    : 0x%x   GPIO0: 0x%x\r\n",
               (unsigned)SYSTEM_UART_0_IO_CTRL, (unsigned)SYSTEM_GPIO_0_IO_CTRL);
    bsp_printf("    外设块   : 0x%x (采集FIFO/串口屏/PWM)\r\n", PF_BASE);
    bsp_printf("    屏参数   : 200 kHz 换能器, fs=%u Hz, OSR=%u\r\n",
               MEAS_FS_HZ, MEAS_FS_HZ / F0_HZ);
#ifdef SYSTEM_CORES_0_CFU
    bsp_printf("    CFU      : present (39 custom instructions)\r\n");
#else
    bsp_printf("    CFU      : NOT present\r\n");
#endif

    bsp_printf("\r\n[2] CFU instruction self test\r\n");
    g_pass = 0; g_fail = 0;
    bsp_printf("\r\n    pass=%d  fail=%d  -> %s\r\n", g_pass, g_fail,
               g_fail ? "FAIL" : "ALL PASS");

    acq_bringup();

    g_pass = 0; g_fail = 0;

#if CFG_DIAG_BENCH

#endif

    write_u32(1u, SCR_RX_CTRL);
    bsp_printf("[4a] 屏回传通道已开启（等 0x88 上电握手，最长 1.5 s）\r\n");
    {
        uint32_t g;

        for (g = 0u; g < 200000000u; g++) {
            if (read_u32(SCR_STAT) & 4u) break;
        }
        bsp_printf("     0x88 %s（SCR_STAT=0x%x）\r\n",
                   (read_u32(SCR_STAT) & 8u) ? "已收到" : "未收到（走 1.5s 超时兜底）",
                   read_u32(SCR_STAT));
    }

    bsp_printf("[4b] 跳过开机三角波自检（屏上直接刷双通道互相关）\r\n");

    meas_loop();

    realtime_loop();
}
