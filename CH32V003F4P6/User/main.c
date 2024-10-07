/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : Kaushal
 * Version            : V1.0.0
 * Date               : 2024/08/25
 * Description        : Main program body.
 *********************************************************************************
 *******************************************************************************/
#include "debug.h"
/* Global define */
#define SHIFT_LAT_PORT          (GPIOC)
#define SHIFT_LAT_PIN           (GPIO_Pin_4)

#define INSCAN_GPIO_PORT        (GPIOC)
#define INSCAN_GPIO_PINS        (GPIO_Pin_7)

#define LED_GPIO_PORT           (GPIOC)
#define LED_GPIO_PINS           (GPIO_Pin_3)

#define TIC_IN_MS           40  //Derived from TMR2

#define SEGMENT_SREG    1
#define SEG_COM_IN_SREG 0
//Dig4,Dig3,Dig2,Sig1
#define DIGIT1_POS      7
#define DIGIT2_POS      6
#define DIGIT3_POS      5
#define DIGIT4_POS      4
#define IP1             3
#define IP2             2
#define IP3             1
#define IP4             0

/* Global Variable */
vu8 val;
const uint8_t digitCode[10]= {0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F};
//volatile uint16_t tm2_cnt=0;

uint8_t digit_turn=0;
uint8_t ms1Sec=19;
uint8_t ms40cnt=0;

typedef struct
{
    uint8_t a:1;
    uint8_t b:1;
    uint8_t c:1;
    uint8_t d:1;
    uint8_t e:1;
    uint8_t f:1;
    uint8_t g:1;
    uint8_t dp:1;
}shift_bits;
typedef union
{
    uint8_t sbyte;
    shift_bits sbits;
}shift_pins;

shift_pins sreg[2];
uint8_t i;

volatile shift_pins segBuf[4];

uint16_t test=0;

volatile uint16_t readingAdc[2]={0};
/*********************************************************************
 * Private function prototypes
 *
 */
void SPI_1Lines_HalfDuplex_Init(void);
void Timer2_Init(void);
void GPIO_Pins_Init(void);
void GPIO_TogglePin(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);

void loadSegBuf(uint16_t data,uint8_t dot_pos);
void dispBuf(void);
uint8_t scanKeys(void);
void sendToShiftReg(void);
void ADC_Function_Init(void);
void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize);
/*********************************************************************/
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/*********************************************************************
 * @fn TIM2_UP_IRQHandler
 *
 * @brief TIM2_UP_IRQHandler function handles the interrupt for TIM2 and auto-reload values of TIM2.
 */
void TIM2_IRQHandler()
{
    //volatile uint16_t tempcnt = TIM2->CNT, temparr = TIM2->ATRLR;
    if (TIM_GetITStatus(TIM2, TIM_IT_Update))
    {
        //tm2_cnt++;
        ms1Sec++;
        ms40cnt++;
    }
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}

int main(void)
{
  //  uint8_t i=0;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif
    printf("SystemClk:%d\r\n",SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    GPIO_Pins_Init();
    SPI_1Lines_HalfDuplex_Init();
    Timer2_Init();

    ADC_Function_Init();
    DMA_Tx_Init(DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)readingAdc, 2);
    DMA_Cmd(DMA1_Channel1, ENABLE);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_241Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_241Cycles);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    GPIO_WriteBit(SHIFT_LAT_PORT, SHIFT_LAT_PIN, RESET);
    while(1)
    {
        dispBuf();
        if(ms40cnt)
        {
            ms40cnt = 0;
            //test = readingAdc[1];//scanKeys();
            loadSegBuf(test,0);
        }
        if(ms1Sec > (1000/TIC_IN_MS))
        {
            ms1Sec = 0;
            GPIO_TogglePin(LED_GPIO_PORT,LED_GPIO_PINS);
            //printf("ADC CH3:%d\n\r",readingAdc[0]);
            printf("ADC CH4:%d\n\r",readingAdc[1]);
            test = readingAdc[1];
        }
    }
}

void GPIO_Pins_Init(void)
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1, ENABLE );
    GPIO_InitTypeDef GPIO_InitStructure={0};
    //For LAT pin of shift Register All three pins are on PORTC an output so configured together
    GPIO_InitStructure.GPIO_Pin = SHIFT_LAT_PIN | LED_GPIO_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init( SHIFT_LAT_PORT, &GPIO_InitStructure ); //

    GPIO_InitStructure.GPIO_Pin = INSCAN_GPIO_PINS;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init( INSCAN_GPIO_PORT, &GPIO_InitStructure );
}
void SPI_1Lines_HalfDuplex_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    SPI_InitTypeDef SPI_InitStructure={0};
//    NVIC_InitTypeDef NVIC_InitStructure={0};

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC | RCC_APB2Periph_SPI1, ENABLE );

#if (SPI_MODE == HOST_MODE)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

//    GPIO_InitStructure.GPIO_Pin = SHIFT_LAT_PIN;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_30MHz;
//    GPIO_Init( SHIFT_LAT_PORT, &GPIO_InitStructure ); //

#elif (SPI_MODE == SLAVE_MODE)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
#endif


#if (SPI_MODE == HOST_MODE)
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Tx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;

#elif (SPI_MODE == SLAVE_MODE)
    SPI_InitStructure.SPI_Direction = SPI_Direction_1Line_Rx;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Slave;

#endif

    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//48/16=3Mhz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init( SPI1, &SPI_InitStructure );

//    NVIC_InitStructure.NVIC_IRQChannel = SPI1_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

#if (SPI_MODE == SLAVE_MODE)
//    SPI_I2S_ITConfig( SPI1, SPI_I2S_IT_RXNE , ENABLE );

#endif

    SPI_Cmd( SPI1, ENABLE );
}
void Timer2_Init(void)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    NVIC_InitTypeDef NVIC_InitStructure;

    //TIM_InternalClockConfig(TIM2);//default value internal clock So, no need to write this line
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_SetAutoreload(TIM2, TIC_IN_MS);
    TIM_PrescalerConfig(TIM2, (48000-1), TIM_PSCReloadMode_Immediate);
    TIM_CounterModeConfig(TIM2, TIM_CounterMode_Up);
   // TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

    NVIC_Init(&NVIC_InitStructure);

    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_SetCounter(TIM2, 0);
    TIM_Cmd(TIM2, ENABLE);
}
void GPIO_TogglePin(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
    GPIOx->OUTDR ^= (uint32_t)GPIO_Pin;
}

void loadSegBuf(uint16_t data,uint8_t dot_pos)
{
  uint8_t i,temp;
  for(i=0;i<4;i++)
  {
    temp = data%10;
    data = data/10;
    segBuf[i].sbyte = digitCode[temp];
  }
  if((dot_pos>0) && (dot_pos<4))
  {
    segBuf[dot_pos].sbits.dp = 1;
  }
}
void dispBuf(void)
{
  if(++digit_turn > 3)
  {
    digit_turn = 0;
  }
  sreg[SEG_COM_IN_SREG].sbyte = ((sreg[SEG_COM_IN_SREG].sbyte & 0x0F) | (1<<(7-digit_turn)));
  sreg[SEGMENT_SREG].sbyte = segBuf[digit_turn].sbyte;

  sendToShiftReg();
}
uint8_t scanKeys(void)
{
  uint8_t keyStatus=0;
  uint8_t i;
  FlagStatus ki=RESET;

  ki = GPIO_ReadInputDataBit(INSCAN_GPIO_PORT, INSCAN_GPIO_PINS);
  if(ki)
  {
    for(i=0;i<4;i++)
    {
        sreg[SEG_COM_IN_SREG].sbyte &= 0xF0;
        sreg[SEG_COM_IN_SREG].sbyte |= (1<<i);
        sendToShiftReg();
        ki = GPIO_ReadInputDataBit(INSCAN_GPIO_PORT, INSCAN_GPIO_PINS);
        if(ki)
        {
          keyStatus |= (1<<i);
        }
    }
    sreg[SEG_COM_IN_SREG].sbyte |= 0x0F;
    sendToShiftReg();
  }
  return keyStatus;
}
void sendToShiftReg(void)
{
  if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
  SPI_I2S_SendData(SPI1, sreg[SEG_COM_IN_SREG].sbyte);

  if(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE))
  SPI_I2S_SendData(SPI1, sreg[SEGMENT_SREG].sbyte);

  Delay_Us(25);
  GPIO_WriteBit(SHIFT_LAT_PORT, SHIFT_LAT_PIN, SET);
  Delay_Us(25);
  GPIO_WriteBit(SHIFT_LAT_PORT, SHIFT_LAT_PIN, RESET);
}

void ADC_Function_Init(void)
{
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 2;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_Calibration_Vol(ADC1, ADC_CALVOL_50PERCENT);
    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
}
void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, u32 ppadr, u32 memadr, u16 bufsize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = ppadr;
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufsize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}
