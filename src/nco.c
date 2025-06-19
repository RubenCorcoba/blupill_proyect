/**********************************************************************************************************************
Copyright (c) 2025, Fernando Alberto Miranda Bonomi <fmirandabonomi@herrera.unt.edu.ar>
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the “Software”), to deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

SPDX-License-Identifier: MIT
**********************************************************************************************************************/

/// @addtogroup Nco
/// @{

#include "nco.h"
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdbool.h>

/// @brief Constante pi, relación del perímetro de una circunferencia a su diámetro
#define PI 3.14159265358979323846

/// @breif Amplitud máxima
#define AMPLITUDE INT16_MAX

/// @brief Estado del oscilador numerico
struct Nco_s {
    int32_t bReal; ///< Parte real de exp(-2*j*pi*fs/fc)
    int32_t bImag; ///< Parte imag de exp(-2*j*pi*fs/fc)
    int16_t real;  ///< Parte real de la muestra actual
    int16_t imag;  ///< Parte imaginaria de la muestra actual
};

static bool init = false;
Nco Nco_create(double outFreq, double sampFreq)
{
    if (!sampFreq) return NULL;
    static struct Nco_s inst[1];
    if(init) return NULL;

    inst->real = AMPLITUDE;
    inst->imag = 0;
    inst->bReal = (int32_t)((1 << 15) * cos(-2 * PI * outFreq / sampFreq));
    inst->bImag = (int32_t)((1 << 15) * sin(-2 * PI * outFreq / sampFreq));
    return inst;
}
void Nco_delete(Nco self)
{
    if (self) init=false;
}
void Nco_tick(Nco self)
{
    int32_t auxReal, auxImag;
    auxReal = ((self->bReal * (int32_t)self->real) >> 15) - ((self->bImag * (int32_t)self->imag) >> 15);
    auxImag = ((self->bReal * (int32_t)self->imag) >> 15) + ((self->bImag * (int32_t)self->real) >> 15);
    if ((auxImag > -8 && auxImag < 8) || auxReal >= AMPLITUDE || auxReal <= -AMPLITUDE) {
        auxReal = auxReal >= 0 ? AMPLITUDE : -AMPLITUDE;
        auxImag = 0;
    }
    if ((auxReal > -8 && auxReal < 8) || auxImag >= AMPLITUDE || auxImag <= -AMPLITUDE) {
        auxImag = auxImag >= 0 ? AMPLITUDE : -AMPLITUDE;
        auxReal = 0;
    }
    self->real = (int16_t)auxReal;
    self->imag = (int16_t)auxImag;
}
int16_t Nco_getReal(Nco self)
{
    return self->real;
}
int16_t Nco_getImag(Nco self)
{
    return self->imag;
}

void Nco_getSamples(Nco self, ComplexSample *dest, size_t numSamples)
{
    if (!dest) return;
    for (size_t i = 0; i < numSamples; ++i) {
        dest[i].real = self->real;
        dest[i].imag = self->imag;
        Nco_tick(self);
    }
}
/// @}
