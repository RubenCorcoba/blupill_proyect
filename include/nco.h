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

#ifndef NCO_H
#define NCO_H

/**
 * @addtogroup Nco Oscilador Numérico
 *
 * @brief Oscilador numérico complejo, genera muestras de seno y -coseno de 16 bit con signo, de amplitud INT16_MAX (que
 * puede interpretarse como punto fijo 0.15 con máxima amplitud 0x0.FFFFp0
 *
 * @{
 */

#include <stdint.h>

/**
 * @brief Oscilador de control numérico
 * @startuml
 * class Nco{
 * + create(fc,fs)
 * + delete()
 * + tick()
 * + getReal()
 * + getImag()
 * }
 * @enduml
 */
typedef struct Nco_s *Nco;

typedef struct ComplexSample_s ComplexSample;

struct ComplexSample_s {
    int16_t real;
    int16_t imag;
};

/**
 * @brief Crea un oscilador de control numérico
 * @param outFreq frecuencia a generar
 * @param sampFreq frecuencia de muestreo, fc >= 2*fs para evitar alias
 * @retval Nco El oscilador
 * @retval NULL Si no pudo crearlo
 */
Nco Nco_create(double outFreq, double sampFreq);

/**
 * @brief Destruye un oscilador de control numérico, liberando los recursos asociados
 * @warning Luego de este llamado el oscilador deja de ser válido
 * @param self El oscilador
 */
void Nco_delete(Nco self);

/**
 * @brief Ejecuta un paso de un oscilador de control numérico
 * @param self El oscilador
 */
void Nco_tick(Nco self);

/**
 * @brief Obtiene la parte real de la muestra actual
 * @param self El oscilador
 * @return int16_t Parte real actual
 */
int16_t Nco_getReal(Nco self);

/**
 * @brief Obtiene la parte imaginaria de la muestra actual
 * @param self El oscilador
 * @return int16_t Parte imaginaria
 */
int16_t Nco_getImag(Nco self);

/**
 * @brief Llena buffers con muestras reales e imaginarias
 * @param self El Nco
 * @param dest Destino
 * @param numSamples Cantidad de muestras a generar
 */
void Nco_getSamples(Nco self, ComplexSample *dest, size_t numSamples);

/// @}

#endif // NCO_H
