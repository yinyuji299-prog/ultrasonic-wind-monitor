/*====================================================================*/
/*  demo.h  --  V2 demo framework + common DSP primitives              */
/*                                                                    */
/*  Responsibilities:                                                 */
/*    1. Output helpers  : DEMO_HEADER / DEMO_FOOTER                  */
/*    2. Bench macros    : BENCH / BENCH_LOOP                         */
/*    3. Verify macros   : VERIFY_EQ / VERIFY_EQ_TOL1                 */
/*    4. Common DSP      : hanning_window(), fir_tap()               */
/*                                                                    */
/*  All primitives are header-only (static inline) so each demo can  */
/*  #include "demo.h" without paying linker cost.                     */
/*====================================================================*/

#ifndef DEMO_H_
#define DEMO_H_

#include <stdint.h>
#include "bsp.h"

/* global pass / fail counters (defined in main.c as static) */

/*====================================================================*/
/*  1.  Output helpers                                                */
/*====================================================================*/

/* Print "[label] ".  label is a literal string. */
#define DEMO_HEADER(label)  bsp_printf("\r\n[" label "] ")

/* Print PASS/FAIL summary for the demo body. */
#define DEMO_FOOTER()       bsp_printf("    PASS=%d FAIL=%d\r\n", g_pass, g_fail)

/*====================================================================*/
/*  2.  Bench macros                                                  */
/*====================================================================*/

/* Time a scalar_fn vs cfu_fn, print ratio.  Both are parameterless
   call expressions.  Use BENCH_LOOP for sub-10-cycle bodies. */
#define BENCH(label, scalar_fn, cfu_fn)                                     \
    do {                                                                    \
        uint32_t _ts = ticks(); (void)(scalar_fn);                          \
        uint32_t _cs = ticks() - _ts;                                       \
        uint32_t _tu = ticks(); (void)(cfu_fn);                            \
        uint32_t _cu = ticks() - _tu;                                       \
        uint32_t _sp = _cu ? (100u * _cs / _cu) : 0u;                       \
        bsp_printf("    %-22s  sc=%-5u  cf=%-5u  x=%u.%02u\r\n",            \
                   (label), (unsigned)_cs, (unsigned)_cu,                   \
                   (unsigned)(_sp / 100u), (unsigned)(_sp % 100u));         \
    } while (0)

/* Same as BENCH but repeats `reps` times to stabilise tiny bodies. */
#define BENCH_LOOP(label, reps, scalar_fn, cfu_fn)                          \
    do {                                                                    \
        uint32_t _ts = ticks();                                             \
        for (int _r = 0; _r < (reps); _r++) { (void)(scalar_fn); }          \
        uint32_t _cs = ticks() - _ts;                                       \
        uint32_t _tu = ticks();                                             \
        for (int _r = 0; _r < (reps); _r++) { (void)(cfu_fn); }            \
        uint32_t _cu = ticks() - _tu;                                       \
        uint32_t _sp = _cu ? (100u * _cs / _cu) : 0u;                       \
        bsp_printf("    %-22s  sc=%-5u  cf=%-5u  x=%u.%02u\r\n",            \
                   (label), (unsigned)_cs, (unsigned)_cu,                   \
                   (unsigned)(_sp / 100u), (unsigned)(_sp % 100u));         \
    } while (0)

/* Measure two multi-statement paths in cycles; results stored into _sw/_cf.
   Use this when the body is more than a single expression (e.g. for-loop).
   The two `_t = ticks()` reads don't overlap so each measurement is exact. */
#define BENCH_TWO(_sw, _cf, scalar_stmt, cfu_stmt)                          \
    do {                                                                    \
        uint32_t _t = ticks();                                              \
        scalar_stmt;                                                        \
        (_sw) = ticks() - _t;                                               \
        _t = ticks();                                                       \
        cfu_stmt;                                                           \
        (_cf) = ticks() - _t;                                               \
    } while (0)

/* Print the standard "scalar=.. cfu=.. speedup=..xx mismatches=.." line.
   Speedup = 100*scalar/cfu in Q10 fixed-point to avoid float. */
#define BENCH_PRINT(label, tsw, tcfu, mismatches)                           \
    do {                                                                    \
        uint32_t _sp = (tcfu) ? (100u * (tsw) / (tcfu)) : 0u;               \
        bsp_printf("    %-18s  sc=%-5u  cf=%-5u  x=%u.%02u  mism=%d\r\n",   \
                   (label), (unsigned)(tsw), (unsigned)(tcfu),              \
                   (unsigned)(_sp / 100u), (unsigned)(_sp % 100u),          \
                   (int)(mismatches));                                      \
    } while (0)

/*====================================================================*/
/*  3.  Verify macros                                                 */
/*====================================================================*/

#define VERIFY_EQ(got, exp)                                                 \
    do {                                                                    \
        int32_t _g = (int32_t)(got), _e = (int32_t)(exp);                   \
        if (_g == _e) g_pass++;                                             \
        else { g_fail++; bsp_printf("    FAIL got=%d exp=%d (line %d)\r\n", \
                                   _g, _e, __LINE__); }                     \
    } while (0)

/* +/-1 LSB tolerance for Q15 saturation corners (Karatsuba vs direct). */
#define VERIFY_EQ_TOL1(got, exp)                                            \
    do {                                                                    \
        int32_t _g = (int32_t)(got), _e = (int32_t)(exp);                   \
        if (_g == _e || _g == _e+1 || _g == _e-1) g_pass++;                 \
        else { g_fail++; bsp_printf("    FAIL got=%d exp=%d (line %d)\r\n", \
                                   _g, _e, __LINE__); }                     \
    } while (0)

/*====================================================================*/
/*  4.  Common DSP primitives                                         */
/*====================================================================*/

/* ---- 4.0  Saturating clamp ---------------------------------------- */
/* Clamp a 32-bit accumulator down to int16 / int8 range.

   Centralised here on purpose: the codebase used to open-code
     if (v > 32767) v = 32767;  /  v > 32767 ? 32767 : ...
   in a dozen places (and had a half-used Q15SAT_S macro).  One shared
   rule means one place to fix and one consistent behaviour.

   These are functions, not macros, so arguments are evaluated exactly
   once (a macro like SAT16(x++) would be a bug). */
static inline int16_t sat16(int32_t v)
{
    if (v >  32767) return  32767;
    if (v < -32768) return -32768;
    return (int16_t)v;
}

static inline int8_t sat8(int32_t v)
{
    if (v >  127) return  127;
    if (v < -128) return -128;
    return (int8_t)v;
}

/* ---- 4.1  Hanning window (raised cosine) -------------------------- */
/* Q15 cosine lookup: cos(2*pi*k/16) for k=0..15. */
static const int16_t _cos16[16] = {
    32767,  30273,  23170,  12539,
        0, -12539, -23170, -30273,
   -32767, -30273, -23170, -12539,
        0,  12539,  23170,  30273
};

/* Fill w[0..N-1] with Hanning window coefficients (Q15).
   Formula:  w[n] = 0.5 * (1 - cos(2*pi*n / (N-1)))
   For N <= 1, all coefficients are 32767 (identity). */
static inline void hanning_window(int N, int16_t *w)
{
    if (N <= 1) { if (N == 1) w[0] = 32767; return; }
    for (int n = 0; n < N; n++) {
        int idx = (n * 16) / (N - 1);
        if (idx > 15) idx = 15;
        w[n] = (int16_t)((32767 - _cos16[idx]) / 2);
    }
}

/* Hamming window: w[n] = 0.54 - 0.46*cos(2*pi*n/(N-1))  (Q15) */
static inline void hamming_window(int N, int16_t *w)
{
    if (N <= 1) { if (N == 1) w[0] = 32767; return; }
    for (int n = 0; n < N; n++) {
        int idx = (n * 16) / (N - 1);
        if (idx > 15) idx = 15;
        int32_t c = _cos16[idx];
        /* 0.54 = 17695 Q15, 0.46 = 15073 Q15 */
        w[n] = (int16_t)((17695 * 32767 - 15073 * c) / 32767);
    }
}

/* ---- 4.2  Packed FIR tap (LDACC path) ------------------------------ */

/* Pack 4 consecutive INT8 samples / coefficients into one 32-bit
   word with sample[0] in the LSB lane.  Compiler inlines this. */
static inline uint32_t pack4_i8(int8_t a0, int8_t a1, int8_t a2, int8_t a3)
{
    return  (uint32_t)(uint8_t)a0
         | ((uint32_t)(uint8_t)a1 <<  8)
         | ((uint32_t)(uint8_t)a2 << 16)
         | ((uint32_t)(uint8_t)a3 << 24);
}

/* Apply a packed 4-tap FIR: out = sum_{k=0..3} x[k] * h[k].
   Uses 1 LDACC + 1 RDACC.  Q15 output.

   Defined as a MACRO, not a function, on purpose: LDACC()/RDACC() are
   defined in main.c AFTER this header is included, so an inline function
   here would only see an implicit declaration.  Expanding at the call
   site puts us inside main.c where both macros are in scope.

   Kept available (per "可以不调用，但是不能没有") even though no demo
   calls it yet - unused macros cost nothing. */
#define fir4_tap(x0, x1, x2, x3, h0, h1, h2, h3)                            \
    (LDACC(pack4_i8(x0, x1, x2, x3), pack4_i8(h0, h1, h2, h3)),             \
     (int32_t)RDACC())

/*====================================================================*/
/*  5.  Misc                                                         */
/*====================================================================*/

#define DEMO_UNUSED(x)  ((void)(x))

#endif /* DEMO_H_ */