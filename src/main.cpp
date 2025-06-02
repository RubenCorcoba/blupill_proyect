/// Versión 13 bluepill_w5100_ethernet RDC - Remapeo SPI1 a pines alternativos (PA15, PB3, PB4, PB5)
#include <Arduino.h>
#include <Ethernet.h>  // Librería para el módulo W5100 Ethernet
#include <SPI.h>

////////////////////////////////////////////////////////////////
// Definiciones de tamaño de buffer
constexpr int NMUESTRAS_BUFFER = 256; // Número de muestras por cada mitad del buffer (cada muestra = 2 bytes)

// Buffers y variables de control
uint8_t buffer_ADC[2][NMUESTRAS_BUFFER * 2]; // Doble buffer circular
volatile uint32_t cuenta_buffers_cargados = 0; // Contador de buffers llenados por el DMA
volatile uint32_t cuenta_buffers_vistos = 0;   // Contador de buffers procesados en el bucle principal

// Declaración de funciones
void ADC_DMA_Init(void);      // Inicialización del ADC con DMA
void Ethernet_Init(void);     // Inicialización del módulo Ethernet
void transmite(uint8_t* datos, int nbytes); // Función para transmitir datos al servidor

static EthernetClient client; // Objeto cliente para comunicación Ethernet

// Prototipo de la interrupción DMA (declaración externa)
extern "C" void DMA1_Channel1_IRQHandler(void);

////////////////////////////////////////////////////////////////
// Función setup: Configuración inicial
void setup(void) {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, 1);
    
    ADC_DMA_Init();     // Configura el ADC con DMA para adquisición de datos
    Ethernet_Init();    // Configura el módulo Ethernet
    SPI.setMOSI(PB5);
    SPI.setMISO(PB4);
    SPI.setSCLK(PB3);
    SPI.setSSEL(PA15);
    SPI.begin();        // Inicializa el módulo SPI
    SPI.beginTransaction(SPISettings(14000000, MSBFIRST, SPI_MODE0)); // Configura SPI a 14 MHz
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
////////////////////////////////////////////////////////////////
// Función para transmitir los datos al servidor
void transmite(uint8_t* datos, int nbytes) {
    static byte ip_servidor[4] = {192, 168, 1, 120}; // IP del servidor

    // Intentar conectarse si no hay conexión activa
    if (!client.connected()) {
        client.connect(ip_servidor, 4000);
        return; // Salir si aún no se conecta
    }

    // Transmitir datos si ya está conectado
    client.write(datos, nbytes);
}

////////////////////////////////////////////////////////////////
// Configuración del ADC con DMA
void ADC_DMA_Init(void) {
    // Activar relojes necesarios
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Configurar PA0 como entrada analógica
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);

    // Configurar ADC:
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV2; // Reloj ADC = PCLK2 / 2
    ADC1->SQR3 = 0; // Canal 0 (PA0)
    ADC1->SMPR2 = 0; // Tiempo de muestreo mínimo (1.5 ciclos)
    ADC1->CR1 = 0; // Sin configuración especial

    ADC1->CR2 = ADC_CR2_ADON; // Encender ADC
    delay(1); // Esperar estabilización
    ADC1->CR2 |= ADC_CR2_CAL; // Calibrar ADC
    while (ADC1->CR2 & ADC_CR2_CAL); // Esperar fin calibración

    // Modo continuo + DMA habilitado
    ADC1->CR2 |= ADC_CR2_CONT | ADC_CR2_DMA;
    ADC1->CR2 |= ADC_CR2_ADON; // Re-encender ADC

    // Configurar DMA para leer del ADC y guardar en buffer
    DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR; // Dirección del registro del ADC
    DMA1_Channel1->CMAR = (uint32_t)buffer_ADC; // Dirección base del buffer doble
    DMA1_Channel1->CNDTR = NMUESTRAS_BUFFER * 2; 

    // Configurar DMA1 Canal 1
    DMA1_Channel1->CCR = DMA_CCR_MINC |  // Incrementar dirección en RAM
                         DMA_CCR_CIRC | // Modo circular (doble buffer)
                         DMA_CCR_HTIE | // Interrupción mitad del buffer
                         DMA_CCR_TCIE | // Interrupción fin del buffer
                         DMA_CCR_EN |   // Activar DMA
                         DMA_CCR_MSIZE_0 | // Memoria de 16 bits
                         DMA_CCR_PSIZE_0;  // Periférico de 16 bits

    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // Habilitar interrupción del canal 1 del DMA
}

////////////////////////////////////////////////////////////////
// Configuración del módulo Ethernet (W5100)
void Ethernet_Init(void) {
    static byte mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // MAC arbitraria
    static byte ip[4] = {192, 168, 1, 33}; // IP fija del dispositivo

    Ethernet.begin(mac, ip); // Inicia el módulo con esos parámetros
    delay(100); // Esperar a que se configure correctamente
}

////////////////////////////////////////////////////////////////
// Manejador de interrupciones del DMA1 Canal 1
extern "C" void DMA1_Channel1_IRQHandler(void) {
    // Interrupción por mitad del buffer lleno
    if (DMA1->ISR & DMA_ISR_HTIF1) {
        DMA1->IFCR |= DMA_IFCR_CHTIF1; // Limpiar bandera
        cuenta_buffers_cargados++;    // Registrar nuevo buffer disponible
    }

    // Interrupción por buffer completo lleno
    if (DMA1->ISR & DMA_ISR_TCIF1) {
        DMA1->IFCR |= DMA_IFCR_CTCIF1; // Limpiar bandera
        cuenta_buffers_cargados++;    // Registrar nuevo buffer disponible
    }
}
