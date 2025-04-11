// Versión 11 blupill_w5100_ethernet RDC
#include <Arduino.h>
#include <Ethernet.h>  // Librería para el módulo W5100 Ethernet
#include <SPI.h>

////////////////////////////////////////////////////////////////
// Definiciones de tamaño de buffer
constexpr int NMUESTRAS_BUFFER = 8; // Número de muestras por cada mitad del buffer 

// Buffers y variables de control
uint8_t buffer_ADC[2][NMUESTRAS_BUFFER * 2]; // Doble buffer: cada buffer tiene capacidad para NMUESTRAS_BUFFER muestras (cada muestra = 2 bytes)
volatile uint32_t cuenta_buffers_cargados = 0; // Contador de buffers llenados por el DMA
volatile uint32_t cuenta_buffers_vistos = 0;   // Contador de buffers procesados en el bucle principal

// Declaración de funciones
void ADC_DMA_Init(void);  // Inicialización del ADC con DMA
void Ethernet_Init(void); // Inicialización del módulo Ethernet
void transmite(uint8_t* datos, int nbytes); // Función para transmitir datos al servidor

static EthernetClient client; // Objeto cliente para comunicación Ethernet

// Prototipo de la interrupción DMA (declaración externa)
extern "C" void DMA1_Channel1_IRQHandler(void);

////////////////////////////////////////////////////////////////
// Función setup: Configuración inicial
void setup(void) {
    pinMode(LED_BUILTIN,OUTPUT);
    digitalWrite(LED_BUILTIN,1);
    ADC_DMA_Init();     // Configura el ADC con DMA para la adquisición de datos
    Ethernet_Init();    // Configura el módulo Ethernet
    SPI.begin();        // Inicializa el módulo SPI
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); // Configura SPI a 14 MHz
}

////////////////////////////////////////////////////////////////
// Función loop: Ciclo principal del programa
void loop(void) {
    // Verificar si hay un buffer listo para transmitir
    if (cuenta_buffers_cargados == cuenta_buffers_vistos) {
        // No hay nuevos buffers listos, esperar
        return;
    }

    // Si hay un buffer listo para transmisión
    if (cuenta_buffers_cargados == cuenta_buffers_vistos + 1) {
        digitalWrite(LED_BUILTIN,0);
        // Seleccionar el buffer correcto en base a los contadores
        transmite(buffer_ADC[cuenta_buffers_vistos % 2], NMUESTRAS_BUFFER * 2); // Transmitir los datos del buffer actual
        cuenta_buffers_vistos++; // Incrementar el contador de buffers procesados
        digitalWrite(LED_BUILTIN,1);
    } else {
        // Detectar desbordamiento en el procesamiento de buffers
        cuenta_buffers_vistos = cuenta_buffers_cargados; // Sincronizar contadores para evitar inconsistencias
    }
}

////////////////////////////////////////////////////////////////
// Función para transmitir los datos al servidor
void transmite(uint8_t* datos, int nbytes) {
    static byte ip_servidor[4] = { 192,168,1,120 }; // IP del servidor al que se conecta

    // Intentar conectarse si no hay conexión establecida
    if (!client.connected()) {
        client.connect(ip_servidor, 4000);
        return;
    }

    // Si la conexión está activa, transmitir los datos
    client.write(datos, nbytes); // Enviar buffer al servidor
}

////////////////////////////////////////////////////////////////
// Configuración del ADC con DMA
void ADC_DMA_Init(void) {
    // Habilitar relojes para ADC1, GPIOA y DMA1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN; // Reloj para ADC1 y GPIOA
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;                       // Reloj para DMA1

    // Configurar pin PA0 como entrada analógica
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0); // PA0 configurado como entrada analógica

    // Configurar el ADC (prescaler y secuencia de conversión)
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8; // Prescaler del ADC (PCLK2/8 = 9 MHz)
    ADC1->SQR3 = 0;                    // Canal 0 (PA0) en la secuencia de conversión
    ADC1->SMPR2 = ADC_SMPR2_SMP0_1;    // Tiempo de muestreo: 7.5 ciclos
    ADC1->CR1 = 0;                     // Configuración inicial del ADC
    ADC1->CR2 = ADC_CR2_ADON;          // Encender ADC
    delay(1);                          // Esperar estabilización
    ADC1->CR2 |= ADC_CR2_CAL;          // Calibración del ADC
    while (ADC1->CR2 & ADC_CR2_CAL);   // Esperar a que termine la calibración

    // Configurar modo continuo y DMA
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_ADON;         // Encender el ADC nuevamente

    // Configuración del DMA para leer datos del ADC
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;       // Dirección periférica (registro de datos del ADC)
    DMA1_Channel1->CMAR = (uint32_t)buffer_ADC;      // Dirección de memoria (buffer ADC)
    DMA1_Channel1->CNDTR = NMUESTRAS_BUFFER * 2; // Número de transferencias (32 muestras)
    DMA1_Channel1->CCR = DMA_CCR_MINC |  // Incremento automático de memoria
                         DMA_CCR_CIRC |  // Modo circular
                         DMA_CCR_HTIE |  // Interrupción en mitad de transferencia
                         DMA_CCR_TCIE |  // Interrupción en transferencia completa
                         DMA_CCR_EN |    // Habilitar canal
                         DMA_CCR_MSIZE_0 | // Tamaño de memoria: 16 bits
                         DMA_CCR_PSIZE_0;  // Tamaño periférico: 16 bits

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // Habilitar interrupción del canal DMA1
}

////////////////////////////////////////////////////////////////
// Configuración del módulo Ethernet
void Ethernet_Init(void) {
    static byte mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // Dirección MAC
    static byte ip[4] = {192, 168, 1, 33};                   // Dirección IP local
    Ethernet.begin(mac, ip);                                  // Configurar MAC e IP
}

////////////////////////////////////////////////////////////////
// Interrupción DMA (canal 1)
void DMA1_Channel1_IRQHandler(void) {
    if (DMA1->ISR & DMA_ISR_HTIF1) {  // Verificar bandera de mitad de transferencia
        DMA1->IFCR |= DMA_IFCR_CHTIF1; // Limpiar bandera de interrupción
        cuenta_buffers_cargados++;    // Incrementar contador de buffers cargados
    }

    if (DMA1->ISR & DMA_ISR_TCIF1) {  // Verificar bandera de transferencia completa
        DMA1->IFCR |= DMA_IFCR_CTCIF1; // Limpiar bandera de interrupción
        cuenta_buffers_cargados++;    // Incrementar contador de buffers cargados
    }
}
