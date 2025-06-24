#ifndef BLOCKS_H
#define BLOCKS_H
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#ifdef TRACE_EXEC
#include <stdio.h>
#endif

#define BLOCKS_H_PI 3.14159265358979323846

#define MEM_SZ_(state) (sizeof((state)->mem) / sizeof(*(state)->mem))
#define DELAY_(delay, phase, mem, mem_size) \
    (mem)[(((unsigned)(phase) - ((unsigned)(delay) - 1U)) % (unsigned)(mem_size))]

#define DELAY(delay, state) DELAY_(delay, (state)->phase, (state)->mem, MEM_SZ_(state))

#define SHIFT_IN_(elem, phase, mem, mem_size) \
    do { \
        ++(phase); \
        (mem)[(unsigned)(phase) % (unsigned)(mem_size)] = (elem); \
    } while (0)

#define SHIFT_IN(elem, state) SHIFT_IN_(elem, (state)->phase, (state)->mem, MEM_SZ_(state))

typedef struct ConjugatePolePair_s {
    int32_t a1;
    int32_t a2;
    int32_t mem[2];
    unsigned phase;
} *ConjugatePolePair;

#ifndef POINT_POSITION
#define POINT_POSITION 30
#endif

inline static void ConjugatePolePair_initCoef(ConjugatePolePair state, double a1, double a2)
{
    assert(state);
    a1 = a1 * (1 << POINT_POSITION);
    a2 = a2 * (1 << POINT_POSITION);
    assert(a1 <= INT32_MAX && a1 > INT32_MIN);
    assert(a2 <= INT32_MAX && a2 > INT32_MIN);
    *state = (struct ConjugatePolePair_s){.a1 = (int32_t)a1, .a2 = (int32_t)a2, .mem={}, .phase=0};
}

inline static void ConjugatePolePair_initRadFrec(ConjugatePolePair state, double r, double frec, double fsamp)
{
    assert(state);
    ConjugatePolePair_initCoef(state, -2 * r * cos(2 * BLOCKS_H_PI * frec / fsamp), pow(r, 2));
}

inline static int32_t ConjugatePolePair_step(ConjugatePolePair state, int32_t input)
{
    int32_t t1     = (-(int64_t)state->a1 * (int64_t)DELAY(1, state)) >> POINT_POSITION;
    int32_t t2     = (-(int64_t)state->a2 * (int64_t)DELAY(2, state)) >> POINT_POSITION;
    int32_t output = t1 + t2 + input;
#ifdef TRACE_EXEC
    printf("pd[%p]: input %d, output %d, input-1: %d, input-2: %d\n", (void *)state, input, output, DELAY(1, state),
           DELAY(2, state));
#endif
    SHIFT_IN(output, state);
    return output;
}

inline static int32_t poleAtFrecZero_step(int32_t mem[1], int32_t input)
{
    int32_t output = input + *mem;
#ifdef TRACE_EXEC
    printf("pz[%p]: input %d, output %d, output-1: %d\n", (void *)mem, input, output, *mem);
#endif
    *mem = output;
    return output;
}

typedef struct Comb_s {
    int32_t *memPtr;
    unsigned delay;
    unsigned phase;
    unsigned memSz;
} *Comb;

inline static void Comb_init(Comb state, unsigned delay, unsigned memSz, int32_t *mem)
{
    state->memPtr = mem;
    state->delay  = delay;
    state->memSz  = memSz;
    state->phase  = 0;
}

inline static int32_t Comb_step(Comb state, int32_t input)
{
    int32_t output = input - DELAY_(14, state->phase, state->memPtr, state->memSz);
#ifdef TRACE_EXEC
    printf("cmb[%p]: input: %d, output: %d\n", state, input, output);
    for (int i = 0; i < 14; ++i) {
        printf("input-%d: %d ", i + 1, DELAY(i + 1, state->phase, state->memPtr, state->memSz));
        printf("\n");
    }
#endif
    SHIFT_IN_(input, state->phase, state->memPtr, state->memSz);
    return output;
}

#ifndef NCO_AMPLITUDE
#define NCO_AMPLITUDE INT16_MAX
#endif
#ifndef NCO_GUARD_BITS
#define NCO_GUARD_BITS 3
#endif

typedef struct Nco_s {
    int32_t coefReal, coefImag, stateReal, stateImag;
} *Nco;

typedef struct CplxI16_s {
    int16_t real;
    int16_t imag;
} CplxI16;

inline static void Nco_init(Nco state, double outFreq, double sampFreq)
{
    state->coefReal  = (int32_t)((1 << POINT_POSITION) * cos(2 * BLOCKS_H_PI * outFreq / sampFreq));
    state->coefImag  = (int32_t)((1 << POINT_POSITION) * sin(2 * BLOCKS_H_PI * outFreq / sampFreq));
    state->stateReal = (int32_t)NCO_AMPLITUDE << NCO_GUARD_BITS;
    state->stateImag = 0;
}
#define MUL_I64(a, b) ((int64_t)(a) * (int64_t)(b))

inline static CplxI16 Nco_sample(Nco state)
{
    CplxI16 output;
    int32_t sta_r = state->stateReal;
    int32_t sta_i = state->stateImag;
    if (!sta_i || sta_r > (NCO_AMPLITUDE << NCO_GUARD_BITS) || sta_r < -(NCO_AMPLITUDE << NCO_GUARD_BITS)) {
        sta_r = (sta_r >= 0 ? NCO_AMPLITUDE : -NCO_AMPLITUDE) << NCO_GUARD_BITS;
        sta_i = 0;
    } else if (!sta_r || sta_i > (NCO_AMPLITUDE << NCO_GUARD_BITS) || sta_r < -(NCO_AMPLITUDE << NCO_GUARD_BITS)) {
        sta_r = 0;
        sta_i = (sta_i >= 0 ? NCO_AMPLITUDE : -NCO_AMPLITUDE) << NCO_GUARD_BITS;
    }
    output.real      = sta_r >> NCO_GUARD_BITS;
    output.imag      = sta_i >> NCO_GUARD_BITS;
    state->stateReal = (int32_t)((MUL_I64(state->coefReal, sta_r) - MUL_I64(state->coefImag, sta_i)) >> POINT_POSITION);
    state->stateImag = (int32_t)((MUL_I64(state->coefImag, sta_r) + MUL_I64(state->coefReal, sta_i)) >> POINT_POSITION);
    return output;
}
#undef MUL_I64

#endif // BLOCKS_H

