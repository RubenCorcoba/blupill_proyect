/// Versión 13 bluepill_w5100_ethernet RDC - Remapeo SPI1 a pines alternativos (PA15, PB3, PB4, PB5)
#include <Arduino.h>
#include "bsp.hpp"
////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////
// Función setup: Configuración inicial
void setup(void) {
    bsp_init();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);

    pinMode(PB9, OUTPUT); 
    digitalWrite(PB9, LOW);
}



////////////////////////////////////////////////////////////////
// Función loop: Ciclo principal del programa
void loop(void) {
    // Verificar si hay un buffer listo para transmitir
    if (cuenta_buffers_cargados == cuenta_buffers_vistos) {
        return; // No hay nuevos datos, salir
    }

    // Si hay un nuevo buffer disponible
    if (cuenta_buffers_cargados == cuenta_buffers_vistos + 1) {
        digitalWrite(LED_BUILTIN, 0);
        transmite(buffer_ADC[cuenta_buffers_vistos % 2], NMUESTRAS_BUFFER * 2); // Enviar datos
        cuenta_buffers_vistos++; // Registrar que este buffer ya fue procesado
        digitalWrite(LED_BUILTIN, 1);
    } else {
        // sincronizar para evitar desbordes
        cuenta_buffers_vistos = cuenta_buffers_cargados;
    }
}