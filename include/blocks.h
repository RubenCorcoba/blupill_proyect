#ifndef BLOCKS_H
#define BLOCKS_H
#include <math.h>
#include <stdint.h>

#ifdef TRACE_EXEC
#include <stdio.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define DELAY(delay, phase, mem, mem_size) \
    (mem)[(((unsigned)(phase) - ((unsigned)(delay) - 1U)) % (unsigned)(mem_size))]

#define SHIFT_IN(elem, phase, mem, mem_size) \
    do { \
        ++(phase); \
        (mem)[(unsigned)(phase) % (unsigned)(mem_size)] = (elem); \
    } while (0)

typedef struct OrderTwoState_s{int32_t mem[2]; uint32_t phase;}*OrderTwoState;
inline static int32_t pole_24r25_3f49(int32_t input, OrderTwoState state)
{
    enum{SZ_MEM=sizeof(state->mem)/sizeof(*state->mem)};
    // 30 bit fraccion
    int32_t a1     = -1910917036L;
    int32_t a2     = 989560464L;
    int32_t t1     = (-(int64_t)a1 * (int64_t)DELAY(1, state->phase, state->mem, SZ_MEM)) >> 30;
    int32_t t2     = (-(int64_t)a2 * (int64_t)DELAY(2, state->phase, state->mem, SZ_MEM)) >> 30;
    int32_t output = t1 + t2 + input;

#ifdef TRACE_EXEC
    printf("pd[%p]: input %d, output %d, input-1: %d, input-2: %d\n", (void *)state, input, output, DELAY(1,state->phase,state->mem,SZ_MEM),
           DELAY(2,state->phase,state->mem,SZ_MEM));
#endif
    SHIFT_IN(output, state->phase, state->mem, SZ_MEM);
    return output;
}

typedef struct OrderOneState_s{int32_t mem;} *OrderOneState;
inline static int32_t pole_1r_0f(int32_t input, OrderOneState state)
{
    int32_t output = input + state->mem;
#ifdef TRACE_EXEC
    printf("pz[%p]: input %d, output %d, output-1: %d\n", (void *)state, input, output, state->mem);
#endif
    state->mem = output;
    return output;
}

inline static int32_t comb_2d(int32_t input, OrderTwoState state)
{
    enum{CICLOS_DELAY=2,SZ_MEM=sizeof(state->mem)/(sizeof(*state->mem))};

    int32_t output = input - DELAY(CICLOS_DELAY, state->phase, state->mem, SZ_MEM);
#ifdef TRACE_EXEC
    printf("cmb[%p]: input: %d, output: %d\n", (void *)state, input, output);
    for (int i = 0; i < SZ_MEM; ++i) {
        printf("input-%d: %d ", i + 1, DELAY(i + 1, state->phase, state->mem, SZ_MEM));
        printf("\n");
    }
#endif
    SHIFT_IN(input, state->phase, state->mem, SZ_MEM);
    return output;
}

#define NCO_PI 3.14159265358979323846
#ifndef NCO_AMPLITUDE
#define NCO_AMPLITUDE INT16_MAX
#endif
#ifndef NCO_GUARD_BITS
#define NCO_GUARD_BITS 3
#endif

typedef struct NcoState_s{int32_t coef_real,coef_imag,state_real,state_imag; }*NcoState;

inline static void nco_init(NcoState state, double outFreq, double sampFreq)
{
    state->coef_real  = (int32_t)((1 << 30) * cos(-2 * NCO_PI * outFreq / sampFreq));
    state->coef_imag  = (int32_t)((1 << 30) * sin(-2 * NCO_PI * outFreq / sampFreq));
    state->state_real = (int32_t)INT16_MAX << NCO_GUARD_BITS;
    state->state_imag = 0;
}
#define MUL_I64(a, b) ((int64_t)(a) * (int64_t)(b))

typedef struct ComplexInt16_s{int16_t real, imag;} *ComplexInt16;

inline static void nco_sample(NcoState state, ComplexInt16 cplx)
{
    int32_t sta_r = state->state_real >> NCO_GUARD_BITS;
    int32_t sta_i = state->state_imag >> NCO_GUARD_BITS;
    if (!sta_i || sta_r > NCO_AMPLITUDE || sta_r < -NCO_AMPLITUDE) {
        sta_r              = sta_r >= 0 ? NCO_AMPLITUDE : -NCO_AMPLITUDE;
        sta_i              = 0;
        state->state_real = sta_r << NCO_GUARD_BITS;
        state->state_imag = 0;
    } else if (!sta_r || sta_i > NCO_AMPLITUDE || sta_r < -NCO_AMPLITUDE) {
        sta_r              = 0;
        sta_i              = sta_i >= 0 ? NCO_AMPLITUDE : -NCO_AMPLITUDE;
        state->state_real = 0;
        state->state_imag = sta_i << NCO_GUARD_BITS;
    }
    cplx->real = sta_r;
    cplx->imag = sta_i;
    state->state_real =
        (int32_t)((MUL_I64(state->coef_real, state->state_real) - MUL_I64(state->coef_imag, state->state_imag)) >> 30);
    state->state_imag =
        (int32_t)((MUL_I64(state->coef_imag, state->state_real) + MUL_I64(state->coef_real, state->state_imag)) >> 30);
}
#undef MUL_I64


#ifdef __cplusplus
}
#endif

#endif // BLOCKS_H

