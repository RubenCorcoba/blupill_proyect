#include "bsp.hpp"
#include "blocks.h"
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

// DownConverter
//            nco                                         cpol1i,cpol1q
//  m-|*exp(-2j*pi*30kHz/fsamp*k)|-|(int32_t)|-|<<4|-|polo(r=24/25,fp/fsamp=3/49)|...
//        cpol2i,cpol2q               dcpol1i,dcpol1q        dcpol2i,dcpol2q
//   -|polo(r=24/25,fp/fsamp=3/49)|-|polo(r=1,fp/fsamp=0)|-|polo(r=1,fp/fsamp=0)|...
//                      comb1i,comb1q     comb2i,comb2q
//   -|downsample(7)|-|comb(d=2)      |-|comb(d=2)      |-|>>(4+13)|-|(int16_t)|-msal
//
static struct DownConverter_s{
    struct NcoState_s nco;
    struct OrderTwoState_s cpol1i,cpol1q,cpol2i,cpol2q,comb1i,comb1q,comb2i,comb2q;
    struct OrderOneState_s dcpol1i,dcpol1q,dcpol2i,dcpol2q;
    unsigned downsample_counter;
}downConverter;

static void DownConverter_init(void)
{
    downConverter = (struct DownConverter_s){};
    nco_init(&downConverter.nco,30*6*(12.5+41.5),72000); //< 30 kHz
}

static void DownConverter_output(int32_t i,int32_t q)
{
    (void)i;
    (void)q;
}

static void DownConverter_tick(void)
{
    constexpr int GUARD_BITS = 4;
    struct ComplexInt16_s x;
    int32_t m,mi,mq;
    // DR Zero<15..12>#Unsigned<11..0>
    // Convert to signed and extend to 32 bit
    uint16_t adc_in = ADC1->DR;
    constexpr uint16_t bits_signo = 0x1f<<11;
    m = (int16_t)(adc_in&(1<<11) ? adc_in&(~bits_signo) : adc_in | bits_signo);

    nco_sample(&downConverter.nco,&x);
    mi = (m*x.real) >> (15-GUARD_BITS);
    mq = (m*x.imag) >> (15-GUARD_BITS);

    mi = pole_24r25_3f49(mi,&downConverter.cpol1i);
    mq = pole_24r25_3f49(mq,&downConverter.cpol1q);
    mi = pole_24r25_3f49(mi,&downConverter.cpol2i);
    mq = pole_24r25_3f49(mq,&downConverter.cpol2q);
    mi = pole_1r_0f(mi,&downConverter.dcpol1i);
    mq = pole_1r_0f(mq,&downConverter.dcpol1q);
    mi = pole_1r_0f(mi,&downConverter.dcpol2i);
    mq = pole_1r_0f(mq,&downConverter.dcpol2q);

    if (downConverter.downsample_counter==6){
        mi = comb_2d(mi,&downConverter.comb1i);
        mq = comb_2d(mq,&downConverter.comb1q);
        mi = comb_2d(mi,&downConverter.comb2i);
        mq = comb_2d(mq,&downConverter.comb2q);
        mi = mi >> (13+GUARD_BITS);
        mq = mq >> (13+GUARD_BITS);
        DownConverter_output(mi,mq);
        downConverter.downsample_counter = 0;
    } else {
        ++downConverter.downsample_counter;
    }
}

extern "C" void ADC1_2_IRQHandler(void)
{
    ADC1->SR = 0;
    // resetear bandera de irq
    DownConverter_tick();
}

void bsp_init()
{
    DownConverter_init();
    SPI.setMOSI(PB15);
    SPI.setMISO(PB14);
    SPI.setSCLK(PB13);
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;
    Ethernet_Init(PB12);    // Configura el módulo Ethernet, inicializa SPI
    ADC_DMA_Init();     // Configura el ADC con DMA para adquisición de datos
}
