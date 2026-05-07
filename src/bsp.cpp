#include "bsp.hpp"

#include <Arduino.h>
#include <Ethernet.h>
#include <SPI.h>

static EthernetClient client;  // Objeto cliente para comunicación Ethernet

static void ADC_DMA_Init();                // Inicialización del ADC con DMA
static void Ethernet_Init(uint8_t pinSS);  // Inicialización del módulo Ethernet

// Buffers y variables de control
uint8_t buffer_ADC[2]
                  [NMUESTRAS_BUFFER * NBYTES_MUESTRA];  // Doble buffer circular
volatile uint32_t cuenta_buffers_cargados =
    0;  // Contador de buffers llenados por el DMA
volatile uint32_t cuenta_buffers_vistos =
    0;  // Contador de buffers procesados en el bucle principal

////////////////////////////////////////////////////////////////
// Función para transmitir los datos al servidor
void transmite(uint8_t* datos, int nbytes) {
    static byte ip_servidor[4] = {192, 168, 1, 120};  // IP del servidor

    // Intentar conectarse si no hay conexión activa
    if (!client.connected()) {
        client.connect(ip_servidor, 4000);
        return;  // Salir si aún no se conecta
    }

    // Transmitir datos si ya está conectado
    GPIOC->BRR = 1 << 13;
    client.write(datos, nbytes);
    GPIOC->BSRR = 1 << 13;
}

////////////////////////////////////////////////////////////////
// Configuración del ADC con DMA
static void ADC_DMA_Init(void) {
    // Activar relojes necesarios
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC2EN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Configurar PA0 como entrada analógica
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
    GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);

    // Configurar ADC:

    const uint32_t SMPR2_VAL = // Tiempo de muestreo (55.5 ciclos)
        (0b101U << ADC_SMPR2_SMP0_Pos) +  // Canal 0 55.5 ciclos
        (0b101U << ADC_SMPR2_SMP1_Pos);   // Canal 1 55.5 ciclos
                                        
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6;  // Reloj ADC = PCLK2 / 6 = 12MHz
    ADC1->CR1 = 0;
    ADC1->SQR1 = 0;
    ADC1->SQR2 = 0;
    ADC1->SQR3 = 0;  // Canal 0 (PA0)
    ADC1->SMPR2 = SMPR2_VAL;

    ADC2->CR1 = 0;
    ADC2->SQR1 = 0;
    ADC2->SQR2 = 0;
    ADC2->SQR3 = 1;  // Canal 1 (PA1)
    ADC2->SMPR2 = SMPR2_VAL;

    ADC1->CR2 = ADC_CR2_ADON;  // Encender ADC
    ADC2->CR2 = ADC_CR2_ADON;
    delay(1);                  // Esperar estabilización
    ADC1->CR2 |= ADC_CR2_CAL;  // Calibrar ADC 1
    ADC2->CR2 |= ADC_CR2_CAL;  // Calibrar ADC 2
    while ((ADC1->CR2 & ADC_CR2_CAL) &&
           (ADC2->CR2 & ADC_CR2_CAL));  // Esperar fin calibración

    // Conversión simultánea regular
    ADC1->CR1 |= 0b0110U << ADC_CR1_DUALMOD_Pos;

    // Modo continuo + DMA habilitado
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA;
    ADC2->CR2 |= ADC_CR2_CONT;

    ADC2->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_ADON;  // Re-encender ADC

    // Configurar DMA para leer del ADC y guardar en buffer
    DMA1_Channel1->CPAR =
        (uint32_t)&ADC1->DR;  // Dirección del registro del ADC
    DMA1_Channel1->CMAR =
        (uint32_t)buffer_ADC;  // Dirección base del buffer doble
    DMA1_Channel1->CNDTR = NMUESTRAS_BUFFER * 2;

    // Configurar DMA1 Canal 1
    DMA1_Channel1->CCR = DMA_CCR_MINC |  // Incrementar dirección en RAM
                         DMA_CCR_CIRC |  // Modo circular (doble buffer)
                         DMA_CCR_HTIE |  // Interrupción mitad del buffer
                         DMA_CCR_TCIE |  // Interrupción fin del buffer
                         DMA_CCR_EN |    // Activar DMA
                         (0b10U << DMA_CCR_MSIZE_Pos) |  // Memoria de 32 bits
                         (0b10U << DMA_CCR_PSIZE_Pos);  // Periférico de 32 bits

    NVIC_EnableIRQ(
        DMA1_Channel1_IRQn);  // Habilitar interrupción del canal 1 del DMA
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
void DMA1_Channel1_IRQHandler(void) {
    // Interrupción por mitad del buffer lleno
    if (DMA1->ISR & DMA_ISR_HTIF1) {
        DMA1->IFCR |= DMA_IFCR_CHTIF1;  // Limpiar bandera
        cuenta_buffers_cargados++;      // Registrar nuevo buffer disponible
        GPIOB->BSRR = 1 << 9;
    }

    // Interrupción por buffer completo lleno
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR |= DMA_IFCR_CTCIF1;  // Limpiar bandera
        cuenta_buffers_cargados++;      // Registrar nuevo buffer disponible
        GPIOB->BRR = 1 << 9;
    }
}

void bsp_init() {
    SPI.setMOSI(PB15);
    SPI.setMISO(PB14);
    SPI.setSCLK(PB13);
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    Ethernet_Init(PB12);  // Configura el módulo Ethernet, inicializa SPI
    ADC_DMA_Init();       // Configura el ADC con DMA para adquisición de datos
}
