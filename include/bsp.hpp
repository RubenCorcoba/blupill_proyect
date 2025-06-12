#pragma once

#include <stdint.h>

// Definiciones de tamaño de buffer
constexpr int NMUESTRAS_BUFFER = 512; // Número de muestras por cada mitad del buffer (cada muestra = 2 bytes)

// Buffers y variables de control
extern uint8_t buffer_ADC[2][NMUESTRAS_BUFFER * 2]; // Doble buffer circular
extern volatile uint32_t cuenta_buffers_cargados; // Contador de buffers llenados por el DMA
extern volatile uint32_t cuenta_buffers_vistos;   // Contador de buffers procesados en el bucle principal

// Declaración de funciones
void bsp_init();
void transmite(uint8_t* datos, int nbytes); // Función para transmitir datos al servidor

// Prototipo de la interrupción DMA (declaración externa)
extern "C" void DMA1_Channel1_IRQHandler(void);
