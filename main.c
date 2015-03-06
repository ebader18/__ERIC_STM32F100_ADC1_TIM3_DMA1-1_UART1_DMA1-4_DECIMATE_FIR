#include "main.h"
#include "stm32f10x.h"
//#include <string.h>
#include <stdio.h>

#define ADC1_DR_Address                 ((uint32_t)0x4001244C)
#define USART1_DR_Base                  ((uint32_t)0x40013804)
#define SIZE_FIR_TAP                    128
#define SIZE_DECIMATION                 64
#define SIZE_TX_DATA                    1024

volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004;
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000;
volatile unsigned int *SCB_DEMCR    = (volatile unsigned int *)0xE000EDFC;

void EnableTiming(void);
void Configure_Data();
void Configure_RCC();
void Configure_GPIOs();
void Configure_NVIC();
void Configure_DMA1_1();
void Configure_DMA1_4();
void Configure_ADC1();
void Configure_TIM3();
void Configure_UART1();

volatile short adcData[2 * SIZE_DECIMATION];
voltaile short firBuffer[SIZE_FIR_TAP];
volatile short txData[SIZE_TX_DATA];

int main(void)
{
  Configure_Data();
  Configure_RCC();
  Configure_GPIOs();
  Configure_NVIC();
  Configure_DMA1_1();
  Configure_ADC1();
  Configure_TIM3();
  Configure_UART1();
  
  __CLEAR_BIT(GPIOC->ODR, 8);
  __CLEAR_BIT(GPIOC->ODR, 9);
  
  DMA_Cmd(DMA1_Channel1, ENABLE);
  ADC_DMACmd(ADC1, ENABLE);
  ADC_ExternalTrigConvCmd(ADC1, ENABLE);
  ADC_Cmd(ADC1, ENABLE);
  TIM_Cmd(TIM3,ENABLE);
  
  USART_Cmd(USART1, ENABLE);
  USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
  
  while(1)
  {
  }
}

void EnableTiming(void)
{
    static int enabled = 0;
 
    if (!enabled)
    {
        *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
        *DWT_CYCCNT = 0; // reset the counter
        *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter
 
        enabled = 1;
    }
}

void Configure_Data()
{
// http://t-filter.appspot.com/fir/index.html //
  /*sampling frequency: 16000 Hz, fixed point precision: 16 bits
* 0 Hz - 450 Hz, gain = 0, desired attenuation = -40 dB
* 500 Hz - 2500 Hz, gain = 1, desired ripple = 5 dB
* 2600 Hz - 8000 Hz, gain = 0, desired attenuation = -40 dB*/
  //short tap[32] = {2240,2319,2702,2536,1811,556,-1253,-3626,-6318,-8603,-9356,-7520,-2812,3814,10313,14339,14339,10313,3814,-2812,-7520,-9356,-8603,-6318,-3626,-1253,556,1811,2536,2702,2319,2240};
  
  /*sampling frequency: 16000 Hz, fixed point precision: 16 bits
* 0 Hz - 1000 Hz, gain = 0, desired attenuation = -40 dB
* 1100 Hz - 1800 Hz, gain = 1, desired ripple = 1 dB
* 2000 Hz - 8000 Hz, gain = 0, desired attenuation = -40 dB*/
  //short tap[32] = {-5197,-881,412,2283,4000,4783,4082,1907,-1131,-3979,-5542,-5122,-2738,838,4338,6485,6485,4338,838,-2738,-5122,-5542,-3979,-1131,1907,4082,4783,4000,2283,412,-881,-5197};
  
  /*sampling frequency: 16000 Hz, fixed point precision: 16 bits
  * 0 Hz - 900 Hz, gain = 0, desired attenuation = -40 dB
  * 1000 Hz - 2500 Hz, gain = 1, desired ripple = 1 dB
  * 2600 Hz - 8000 Hz, gain = 0, desired attenuation = -40 dB*/
  //short tap[64] = {-3196,-69,890,2336,3484,3282,1410,-1129,-2657,-2288,-745,341,4,-1182,-1743,-773,1214,2775,2820,1511,71,-233,783,1964,1540,-1438,-5930,-9071,-7939,-1877,6450,12399,12399,6450,-1877,-7939,-9071,-5930,-1438,1540,1964,783,-233,71,1511,2820,2775,1214,-773,-1743,-1182,4,341,-745,-2288,-2657,-1129,1410,3282,3484,2336,890,-69,-3196};

  /*sampling frequency: 16000 Hz, fixed point precision: 16 bits
  * 0 Hz - 900 Hz, gain = 0, desired attenuation = -40 dB
  * 1000 Hz - 2500 Hz, gain = 1, desired ripple = 1 dB
  * 2600 Hz - 8000 Hz, gain = 0, desired attenuation = -40 dB*/
  //short tap[128] = {-427,365,-521,1006,1834,1281,254,-661,-1239,-1243,-674,33,369,217,-97,-140,222,717,918,623,41,-388,-389,-90,94,-130,-649,-1011,-834,-166,526,758,463,56,46,544,1110,1130,384,-691,-1345,-1162,-451,10,-298,-1101,-1497,-766,891,2394,2642,1513,75,-295,784,2108,1715,-1400,-6024,-9116,-7822,-1717,6432,12171,12171,6432,-1717,-7822,-9116,-6024,-1400,1715,2108,784,-295,75,1513,2642,2394,891,-766,-1497,-1101,-298,10,-451,-1162,-1345,-691,384,1130,1110,544,46,56,463,758,526,-166,-834,-1011,-649,-130,94,-90,-389,-388,41,623,918,717,222,-140,-97,217,369,33,-674,-1243,-1239,-661,254,1281,1834,1006,-521,365,-427};

  /*sampling frequency: 16000 Hz, fixed point precision: 16 bits
  * 0 Hz - 1000 Hz, gain = 0, desired attenuation = -40 dB
  * 1100 Hz - 1900 Hz, gain = 1, desired ripple = 1 dB
  * 2000 Hz - 8000 Hz, gain = 0, desired attenuation = -40 dB*/
  //short tap[128] = {-1351,272,194,79,-57,-162,-165,-12,288,640,889,882,552,-33,-675,-1119,-1165,-766,-59,681,1170,1232,883,315,-204,-464,-414,-179,25,18,-233,-583,-787,-639,-102,647,1288,1513,1193,451,-387,-958,-1048,-711,-234,26,-138,-635,-1091,-1042,-205,1298,2904,3818,3377,1422,-1547,-4473,-6153,-5752,-3203,695,4555,6938,6938,4555,695,-3203,-5752,-6153,-4473,-1547,1422,3377,3818,2904,1298,-205,-1042,-1091,-635,-138,26,-234,-711,-1048,-958,-387,451,1193,1513,1288,647,-102,-639,-787,-583,-233,18,25,-179,-414,-464,-204,315,883,1232,1170,681,-59,-766,-1165,-1119,-675,-33,552,882,889,640,288,-12,-165,-162,-57,79,194,272,-1351};

  /*for(uint16_t idxSample = 0; idxSample < SIZE_FIR_TAP - 1; idxSample++)
    lastADCValues[idxSample] = 0;*/
}

void ADC_ready(uint16_t idxDMAcurrentPos)
{
  /*uint16_t idxStart, idxSample, idxDecimation, idxTap;
  int32_t tSample = 0;
  short DecimatedData[SIZE_SAMPLE];
  short tap[128] = {-1351,272,194,79,-57,-162,-165,-12,288,640,889,882,552,-33,-675,-1119,-1165,-766,-59,681,1170,1232,883,315,-204,-464,-414,-179,25,18,-233,-583,-787,-639,-102,647,1288,1513,1193,451,-387,-958,-1048,-711,-234,26,-138,-635,-1091,-1042,-205,1298,2904,3818,3377,1422,-1547,-4473,-6153,-5752,-3203,695,4555,6938,6938,4555,695,-3203,-5752,-6153,-4473,-1547,1422,3377,3818,2904,1298,-205,-1042,-1091,-635,-138,26,-234,-711,-1048,-958,-387,451,1193,1513,1288,647,-102,-639,-787,-583,-233,18,25,-179,-414,-464,-204,315,883,1232,1170,681,-59,-766,-1165,-1119,-675,-33,552,882,889,640,288,-12,-165,-162,-57,79,194,272,-1351};
  
  EnableTiming();
  if (idxDMAcurrentPos == SIZE_SAMPLE * SIZE_DECIMATION)
    idxStart = 0;
  else if (idxDMAcurrentPos == 2 * SIZE_SAMPLE * SIZE_DECIMATION)
    idxStart = SIZE_SAMPLE * SIZE_DECIMATION;
  
  // Decimate
  for(idxSample = 0; idxSample < SIZE_SAMPLE; idxSample++)
  {
    DecimatedData[idxSample] = ADCValues[idxStart + (idxSample * SIZE_DECIMATION)];
    for(idxDecimation = 1; idxDecimation < SIZE_DECIMATION; idxDecimation++)
      DecimatedData[idxSample] += ADCValues[idxStart + (idxSample * SIZE_DECIMATION) + idxDecimation];
  }*/
  // Apply FIR
  /*for(idxSample = 0; idxSample < SIZE_FIR_TAP - 1; idxSample++)
  {
    tSample = tap[0] * DecimatedData[idxSample];
    for(idxTap = 0; idxTap < idxSample; idxTap++)
      tSample += tap[idxTap + 1] * DecimatedData[idxSample - idxTap - 1];
    for(idxTap = 1; idxTap < SIZE_FIR_TAP - idxSample; idxTap++)
      tSample += tap[idxSample + idxTap] * lastADCValues[idxTap - 1];
    TXData[idxSample] = (short)(tSample>>16);
  }
  for(idxSample = SIZE_FIR_TAP - 1; idxSample < SIZE_SAMPLE; idxSample++)
  {
    tSample = tap[0] * DecimatedData[idxSample];
    for(idxTap = 1; idxTap < SIZE_FIR_TAP; idxTap++)
      tSample += tap[idxTap] * DecimatedData[idxSample - idxTap];
    TXData[idxSample] = (short)(tSample>>16);
  }
  // Save the most recent data for the next round
  for(idxSample = 0; idxSample < SIZE_FIR_TAP - 1; idxSample++)
    lastADCValues[idxSample] = DecimatedData[SIZE_SAMPLE - 1 - idxSample];*/
  
  //Configure_DMA1_4((uint32_t)&ADCValues[idxStart]);
  /*for(idxSample = 0; idxSample < SIZE_SAMPLE; idxSample++)
    TXData[idxSample] = DecimatedData[idxSample];
  
  Configure_DMA1_4((uint32_t)&TXData[0]);
  DMA_Cmd(DMA1_Channel4, ENABLE);*/
}

void Configure_RCC()
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;                                           // Enable GPIOC Clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;                                           // Enable ADC1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                                           // Enable GPIOA Clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                                           // Enable Alternate Fuctions Clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;                                           // Enable TIM3 Clock
  RCC->AHBENR |= RCC_AHBPeriph_DMA1;                                            // Enable DMA1 Clock
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;                                         // Enable UART1 Clock
}

void Configure_GPIOs()
{
  GPIOC->CRH = 0x44444422;                                                      // PC8 and PC9 as General Purpose Push Pull Output Max Speed = 2MHz
  GPIOA->CRL = 0x44444404;                                                      // PA1 as input analog mode
  GPIOA->CRH = 0x22224444;                                                      // PA12, PA13, PA14 & PA15 as General Purpose Push Pull Output Max Speed = 2MHz
  
  __CLEAR_BIT(GPIOC->ODR, 8);
  __CLEAR_BIT(GPIOC->ODR, 9);
  __CLEAR_BIT(GPIOA->ODR, 12);
  __CLEAR_BIT(GPIOA->ODR, 13);
  __CLEAR_BIT(GPIOA->ODR, 14);
  __CLEAR_BIT(GPIOA->ODR, 15);
}

void Configure_NVIC()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;                      // Enable DMA1 channel1 IRQ Channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;                      // Enable DMA1 channel4 IRQ Channel
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void Configure_DMA1_1()
{
  DMA_InitTypeDef DMA_InitStructure;
  
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&adcData[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2 * SIZE_DECIMATION;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
}

void Configure_DMA1_4()
{
  DMA_InitTypeDef DMA_InitStructure;
  
  DMA_DeInit(DMA1_Channel4);  
  DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&txData[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
  DMA_InitStructure.DMA_BufferSize = SIZE_TX_DATA;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  
  DMA_Init(DMA1_Channel4, &DMA_InitStructure);
  
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
}

void Configure_ADC1()
{
  ADC_InitTypeDef ADC_InitStructure;
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);
}

void Configure_TIM3()
{
  TIM_TimeBaseInitTypeDef TimeBaseInitStructure; 
  
  TimeBaseInitStructure.TIM_Prescaler = 24 - 1;
  TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TimeBaseInitStructure.TIM_Period = 1000/16/SIZE_DECIMATION;                                        // in microseconds
  TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TimeBaseInitStructure.TIM_RepetitionCounter = 0;
  
  TIM_TimeBaseInit(TIM3,&TimeBaseInitStructure);
  TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
}

void Configure_UART1()
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  USART_InitStructure.USART_BaudRate = 921600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  
  USART_Init(USART1, &USART_InitStructure);
}