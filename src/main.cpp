// Versión 10 blupill_w5100_ethernet RDC
#include <Arduino.h>
#include <Ethernet.h>  // Librería para el W5100
#include <SPI.h>
////////////////////////////////////////////////////////////////
// Definiciones de tamaño de buffer
constexpr int LEN_BUFFER_ADC = 16; // Número de muestras por mitad del buffer (total = 32)
// Buffers y variables de control
uint8_t buffer_ADC[2][LEN_BUFFER_ADC * 2]; // Dos Buffer con capacidad para LEN_BUFFER_ADC muestras (cada muestra son 2 bytes)
volatile uint32_t cuenta_buffers_cargados = 0; // Contador de buffers llenados por DMA
volatile uint32_t cuenta_buffers_vistos = 0;   // Contador de buffers procesados en el bucle principal

// Declaración de funciones
void ADC_DMA_Init(void);  // Inicialización del ADC con DMA
void Ethernet_Init(void); // Inicialización del módulo Ethernet
void transmite(uint8_t* datos,int nbytes); // Función para transmitir datos

static EthernetClient client; // Objeto cliente para comunicación Ethernet

// Prototipo de la interrupción DMA (declaración externa)
extern "C" void DMA1_Channel1_IRQHandler(void);

void setup(void) {
    ADC_DMA_Init(); // Inicializa el ADC con DMA
    Ethernet_Init(); // Inicializa el módulo Ethernet
    SPI.begin(); // Inicializa el módulo SPI
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0));  // Configura el SPI a 14 MHz
}

void loop(void) {
    // Verificar si hay un buffer listo para transmitir
    if (cuenta_buffers_cargados == cuenta_buffers_vistos) {
        // No hay nuevos buffers listos, esperar
        return;
    }

    if (cuenta_buffers_cargados == cuenta_buffers_vistos + 1) { // Hay un buffer listo para transmitir
        transmite(buffer_ADC[cuenta_buffers_vistos % 2],LEN_BUFFER_ADC*2); // Transmitir los datos del medio buffer actual
        cuenta_buffers_vistos++; // Incrementar el contador de buffers procesados
    } else {
        // La diferencia es distinta a 1, desbordamiento detectado
        cuenta_buffers_vistos = cuenta_buffers_cargados; // Sincronizar contadores para recuperar el estado y evitar inconsistencias.
    }
}

void transmite(uint8_t* datos,int nbytes) {
    static byte ip_servidor[4] = {192, 168, 1, 120}; // IP del servidor al que se conecta

    // Intentamos conectarnos si no estamos conectados
    if (!client.connected()) {
        client.connect(ip_servidor, 4000);
    }

    if (client.connected()) {
        client.write(datos, nbytes); // Transmitir buffer al servidor
    }
    
}

void ADC_DMA_Init(void) {
    // Habilita los relojes para ADC1, GPIOA y DMA1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;  // Relojes para ADC1 y GPIOA
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;  // Reloj para DMA1

    // Configura el pin PA0 como entrada analógica
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);  // PA0 como entrada analógica

    // Ajusta el prescaler del ADC para obtener 9 MHz
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;  // Prescaler del ADC = PCLK2/8

    // Configura el ADC
    ADC1->SQR3 = 0;  // Canal 0 (PA0) en la secuencia de conversión
    ADC1->SMPR2 = ADC_SMPR2_SMP0_1;  // Tiempo de muestreo: 7.5 ciclos de reloj
    ADC1->CR1 = 0;  // Configuración inicial del registro CR1 del ADC
    ADC1->CR2 = ADC_CR2_ADON;  // Enciende el ADC
    delay(1);  // Demora para estabilización
    ADC1->CR2 |= ADC_CR2_CAL;  // Calibrar el ADC
    while (ADC1->CR2 & ADC_CR2_CAL);  // Espera a que termine la calibración

    // Habilita el modo continuo y DMA
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_ADON;  // Enciende el ADC nuevamente

    // Configura el DMA
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;  // Dirección periférica del ADC1
    DMA1_Channel1->CMAR = (uint32_t)buffer_ADC;  // Dirección de memoria (buffer ADC)
    DMA1_Channel1->CNDTR = LEN_BUFFER_ADC * 2;  // Número de transferencias (32 muestras)
    DMA1_Channel1->CCR = DMA_CCR_MINC |  // Incremento automático de memoria
                         DMA_CCR_CIRC |  // Modo circular
                         DMA_CCR_HTIE |  // Interrupción de mitad de transferencia
                         DMA_CCR_TCIE |  // Interrupción de transferencia completa
                         DMA_CCR_EN |    // Habilitar canal
                         DMA_CCR_MSIZE_0 |  // Tamaño de memoria: 16 bits
                         DMA_CCR_PSIZE_0;   // Tamaño periférico: 16 bits

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);  // Habilitar interrupción del canal DMA1
}

void Ethernet_Init(void) {
    // Inicializa el módulo Ethernet W5100
    static byte mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // Dirección MAC
    static byte ip[4] = {192, 168, 1, 100}; // Dirección IP local
    Ethernet.begin(mac, ip);  // Configura la dirección MAC e IP
}

// Interrupción del DMA1 (canal 1)
void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_HTIF1) {  // Verifica bandera de mitad de transferencia
        DMA1->IFCR |= DMA_IFCR_CHTIF1; // Limpia bandera
        cuenta_buffers_cargados++;    // Incrementa buffers cargados
    }

    if (DMA1->ISR & DMA_ISR_TCIF1) {  // Verifica bandera de transferencia completa
        DMA1->IFCR |= DMA_IFCR_CTCIF1; // Limpia bandera
        cuenta_buffers_cargados++;    // Incrementa buffers cargados
    }
}
