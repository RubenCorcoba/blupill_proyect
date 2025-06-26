#include "bsp.hpp"

#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>

static EthernetClient client;  // Objeto cliente para comunicación Ethernet

static void ADC_DMA_Init();                // Inicialización del ADC con DMA
static void Ethernet_Init(uint8_t pinSS);  // Inicialización del módulo Ethernet

// Buffers y variables de control
alignas(int16_t) uint8_t
    buffer_ADC[2][NMUESTRAS_BUFFER * 2];  // Doble buffer circular
volatile uint32_t cuenta_buffers_cargados =
    0;  // Contador de buffers llenados por el DMA
volatile uint32_t cuenta_buffers_vistos =
    0;  // Contador de buffers procesados en el bucle principal

static void DownConverter_init(volatile uint32_t *cuentaMediosBuffer,
                               int szBuffer, volatile uint8_t *buffer);

////////////////////////////////////////////////////////////////
// Función para transmitir los datos al servidor
void transmite(uint8_t *datos, int nbytes) {
    static byte ip_servidor[4] = {192, 168, 1, 120};  // IP del servidor

    // Intentar conectarse si no hay conexión activa
    if (!client.connected()) {
        client.connect(ip_servidor, 4000);
        return;  // Salir si aún no se conecta
    }

    // Transmitir datos si ya está conectado
    client.write(datos, nbytes);
}

////////////////////////////////////////////////////////////////
// Configuración del ADC con DMA
static void ADC_DMA_Init(void) {
    // Activar relojes necesarios
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Configurar PA0 como entrada analógica
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);

    // Configurar ADC:
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;      // Reloj ADC = PCLK2 / 6 = 12MHz
    ADC1->SQR3 = 0;                         // Canal 0 (PA0)
    ADC1->SMPR2 = 4 << ADC_SMPR2_SMP0_Pos;  // Tiempo de muestreo (41.5 ciclos)
    ADC1->CR1 = 0;                          // Sin configuración especial

    ADC1->CR2 = ADC_CR2_ADON;         // Encender ADC
    delay(1);                         // Esperar estabilización
    ADC1->CR2 |= ADC_CR2_CAL;         // Calibrar ADC
    while (ADC1->CR2 & ADC_CR2_CAL);  // Esperar fin calibración

    // Modo continuo
    ADC1->CR1 = ADC_CR1_EOCIE;  // Habilita interrupcion?
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_ADON;  // Re-encender ADC

    NVIC_EnableIRQ(ADC1_2_IRQn);  // Habilitar interrupción de ADC
}

////////////////////////////////////////////////////////////////
// Configuración del módulo Ethernet (W5100)
static void Ethernet_Init(uint8_t pinSS) {
    static byte mac[6] = {0xDE, 0xAD, 0xBE,
                          0xEF, 0xFE, 0xED};  // MAC arbitraria
    static byte ip[4] = {192, 168, 1, 33};    // IP fija del dispositivo
    Ethernet.init(pinSS);
    Ethernet.begin(mac, ip);  // Inicia el módulo con esos parámetros
    delay(100);               // Esperar a que se configure correctamente
}

////////////////////////////////////////////////////////////////
// Manejador de interrupciones del DMA1 Canal 1
// extern "C" void DMA1_Channel1_IRQHandler(void) {
//     // Interrupción por mitad del buffer lleno
//     if (DMA1->ISR & DMA_ISR_HTIF1) {
//         DMA1->IFCR |= DMA_IFCR_CHTIF1; // Limpiar bandera
//         cuenta_buffers_cargados++;    // Registrar nuevo buffer disponible
//         digitalWrite((PB9), !digitalRead(PB9));  // Toggle
//     }

//     // Interrupción por buffer completo lleno
//     if (DMA1->ISR & DMA_ISR_TCIF1) {
//         DMA1->IFCR |= DMA_IFCR_CTCIF1; // Limpiar bandera
//         cuenta_buffers_cargados++;    // Registrar nuevo buffer disponible
//         digitalWrite(PB9, !digitalRead(PB9));  // Toggle
//     }
// }

// DownConverter
//  m-|*exp(-2j*pi*30kHz/fsamp*k)|-|(int32_t)|-|<<4|-|polo(r=24/25,fp/fsamp=3/49)|...
//   -|polo(r=24/25,fp/fsamp=3/49)|-|polo(r=1,fp/fsamp=0)|-|polo(r=1,fp/fsamp=0)|...
//   -|downsample(7)|-|comb(d=2)      |-|comb(d=2) |-|>>(4+13)|-|(int16_t)|-msal
static void DownConverter_tick(void) {
    constexpr int PUNTO_OL = 14;
    static int32_t m, oli = 1 << PUNTO_OL, olq, mi, mq;
    const int32_t adc_in = ADC1->DR;

    // Conversión a entero con signo y extensión de signo
    m = adc_in & (1 << 11) ? adc_in & (int16_t)0x07ff
                           : adc_in | (int16_t)0xfffff800;

    // Paso de oscilador local 30 kHz y mezcla
    {
        // exp. punto fijo 2**-15
        constexpr int PUNTO = 15;
        constexpr int32_t pasoi = 21670;
        constexpr int32_t pasoq = -24580;
        int i = (oli * pasoi - olq * pasoq) >> PUNTO;
        int q = (oli * pasoq + olq * pasoi) >> PUNTO;
        oli = i;
        olq = q;
        mi = m * i;
        mq = m * q;
    }
    // Polo doble complejo
    {
        constexpr int PUNTO = 15;
        constexpr int32_t a1 = -58317;
        constexpr int32_t a2 = 30199;
        static int32_t d1i[2], d1q[2], d2i[2], d2q[2], fase;
        mi = (mi - a1 * d1i[fase % 2] - a2 * d1i[(1 + fase) % 2]) >> PUNTO;
        mq = (mq - a1 * d1q[fase % 2] - a2 * d1q[(1 + fase) % 2]) >> PUNTO;
        d1i[(fase + 1) % 2] = mi;
        d1q[(fase + 1) % 2] = mq;
        mi = (mi - a1 * d2i[fase % 2] - a2 * d2i[(1 + fase) % 2]) >> PUNTO;
        mq = (mq - a1 * d2q[fase % 2] - a2 * d2q[(1 + fase) % 2]) >> PUNTO;
        d2i[(fase + 1) % 2] = mi;
        d2q[(fase + 1) % 2] = mq;
        ++fase;
    }
    // integrador doble
    {
        static int32_t d1i, d1q, d2i, d2q;
        mi += d1i;
        mq += d1q;
        d1i = mi;
        d1q = mq;
        mi += d2i;
        mq += d2q;
        d2i = mi;
        d2q = mq;
    }
    // submuestreo
    {
        static int muestra = 0;
        if (muestra == 6) {
            muestra = 0;
            // 2xcomb delay 2
            {
                static int32_t d1i[2], d1q[2], d2i[2], d2q[2], fase;
                int32_t ai, aq;
                ai = mi - d1i[(fase + 1) % 2];
                aq = mq - d1q[(fase + 1) % 2];
                d1i[(fase + 1) % 2] = mi;
                d1q[(fase + 1) % 2] = mq;
                mi = ai;
                mq = aq;
                ai = mi - d2i[(fase + 1) % 2];
                aq = mq - d2q[(fase + 1) % 2];
                d2i[(fase + 1) % 2] = mi;
                d2q[(fase + 1) % 2] = mq;
                mi = ai;
                mq = aq;
                ++fase;
            }
            // Ajuste de ganancia
            {
                mi >>= 7;
                mq >>= 7;
            }
            // salida
            {
                constexpr int LIMITE_2 = sizeof(buffer_ADC) / sizeof(int16_t),
                              LIMITE_1 = LIMITE_2 / 2;
                static_assert(LIMITE_2 % 4 == 0);
                static int cursor;
                volatile int16_t *const buffer =
                    (volatile int16_t *)(buffer_ADC);
                buffer[cursor] = (int16_t)mi;
                buffer[cursor + 1] = (int16_t)mq;
                cursor += 2;
                if (cursor == LIMITE_1) ++cuenta_buffers_cargados;
                if (cursor == LIMITE_2) {
                    ++cuenta_buffers_cargados;
                    cursor = 0;
                }
            }
        } else {
            ++muestra;
        }
    }
}

extern "C" void ADC1_2_IRQHandler(void) {
    // resetear bandera de irq
    ADC1->SR = ~ADC_SR_EOC;
    // Procesa muestras
    DownConverter_tick();
}

void bsp_init() {
    DownConverter_init(&cuenta_buffers_cargados, sizeof(buffer_ADC),
                       (volatile uint8_t *)buffer_ADC);
    SPI.setMOSI(PB15);
    SPI.setMISO(PB14);
    SPI.setSCLK(PB13);
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    Ethernet_Init(PB12);  // Configura el módulo Ethernet, inicializa SPI
    ADC_DMA_Init();       // Configura el ADC con DMA para adquisición de datos
}
