#include "bsp.hpp"
#include "nco.h"
#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>


static EthernetClient client; // Objeto cliente para comunicación Ethernet


static void ADC_DMA_Init();      // Inicialización del ADC con DMA
static void Ethernet_Init(uint8_t pinSS);     // Inicialización del módulo Ethernet

// Buffers y variables de control
uint8_t buffer_ADC[2][NMUESTRAS_BUFFER * 2]; // Doble buffer circular
volatile uint32_t cuenta_buffers_cargados = 0; // Contador de buffers llenados por el DMA
volatile uint32_t cuenta_buffers_vistos = 0;   // Contador de buffers procesados en el bucle principal

static void DownConverter_init(void);

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
static void ADC_DMA_Init(void) {
    // Activar relojes necesarios
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN;
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Configurar PA0 como entrada analógica
    GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);

    // Configurar ADC:
    RCC->CFGR |= RCC_CFGR_ADCPRE_DIV6; // Reloj ADC = PCLK2 / 6 = 12MHz
    ADC1->SQR3 = 0; // Canal 0 (PA0)
    ADC1->SMPR2 = 4 << ADC_SMPR2_SMP0_Pos; // Tiempo de muestreo (41.5 ciclos)
    ADC1->CR1 = 0; // Sin configuración especial

    ADC1->CR2 = ADC_CR2_ADON; // Encender ADC
    delay(1); // Esperar estabilización
    ADC1->CR2 |= ADC_CR2_CAL; // Calibrar ADC
    while (ADC1->CR2 & ADC_CR2_CAL); // Esperar fin calibración

    // Modo continuo
    ADC1->CR1 = ADC_CR1_EOCIE; // Habilita interrupcion?
    ADC1->CR2 |= ADC_CR2_CONT;
    ADC1->CR2 |= ADC_CR2_ADON; // Re-encender ADC

    NVIC_EnableIRQ(ADC1_2_IRQn); // Habilitar interrupción de ADC
}

////////////////////////////////////////////////////////////////
// Configuración del módulo Ethernet (W5100)
static void Ethernet_Init(uint8_t pinSS) {
    static byte mac[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED}; // MAC arbitraria
    static byte ip[4] = {192, 168, 1, 33}; // IP fija del dispositivo
    Ethernet.init(pinSS);
    Ethernet.begin(mac, ip); // Inicia el módulo con esos parámetros
    delay(100); // Esperar a que se configure correctamente
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

static struct DownConverter_s{
    int32_t Ai;
    int32_t Aq;
    int32_t Bi;
    int32_t Bq;
    int32_t Ci;
    int32_t Cq;
    int32_t f;
    int32_t k;
    Nco nco;
}downConverter;

static void DownConverter_init(void)
{
    downConverter.Ai=0;
    downConverter.Aq=0;
    downConverter.Bi=0;
    downConverter.Bq=0;
    downConverter.Ci=0;
    downConverter.Cq=0;
    downConverter.f=0;
    downConverter.k=0;
    downConverter.nco = Nco_create(27,200);
}

static void DownConverter_output(int32_t real,int32_t imag)
{

}

static void DownConverter_tick(void)
{
    int32_t m,xi,xq,mi,mq;

    xi = Nco_getReal(downConverter.nco);
    xq = Nco_getImag(downConverter.nco);
    mi = (m*xi)>> 15;
    mq = (m*xq)>> 15;
    Nco_tick(downConverter.nco);

    downConverter.Ai += mi;
    downConverter.Aq += mq;

    if (downConverter.f==6){
        DownConverter_output(downConverter.Ai-downConverter.Ci, downConverter.Aq-downConverter.Cq);
        downConverter.Ci = downConverter.Bi;
        downConverter.Cq = downConverter.Bq;
        downConverter.Bi = downConverter.Ai;
        downConverter.Bq = downConverter.Aq;
        downConverter.f = 0;
    } else {
        downConverter.f++;
    }
}

extern "C" void ADC1_2_IRQHandler(void)
{
    DownConverter_tick();
    ADC1->SR = 0;
    // resetear bandera de irq
}

void bsp_init()
{
    SPI.setMOSI(PB15);
    SPI.setMISO(PB14);
    SPI.setSCLK(PB13);
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    Ethernet_Init(PB12);    // Configura el módulo Ethernet, inicializa SPI
    ADC_DMA_Init();     // Configura el ADC con DMA para adquisición de datos
}
