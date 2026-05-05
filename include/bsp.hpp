#pragma once

#include <stdint.h>

// Definiciones de tamaño de buffer

// Cantidad de bytes por muestra (cada muestra son dos canales y cada canal 16
// bit)
constexpr int NCANALES_ADC = 2;
constexpr int NBYTES_CANAL = 2;
constexpr int NBYTES_MUESTRA = NCANALES_ADC * NBYTES_CANAL;

// Número de muestras por cada mitad del
// buffer (cada muestra = NBYTES_MUESTRA bytes)
constexpr int NMUESTRAS_BUFFER = 512;

// Buffers y variables de control
extern uint8_t
    buffer_ADC[2][NMUESTRAS_BUFFER * NBYTES_MUESTRA];  // Doble buffer circular
extern volatile uint32_t
    cuenta_buffers_cargados;  // Contador de buffers llenados por el DMA
extern volatile uint32_t
    cuenta_buffers_vistos;  // Contador de buffers procesados en el bucle
                            // principal

// Declaración de funciones
void bsp_init();
void transmite(uint8_t* datos,
               int nbytes);  // Función para transmitir datos al servidor

// Prototipo de la interrupción DMA (declaración externa)
extern "C" void DMA1_Channel1_IRQHandler(void);
