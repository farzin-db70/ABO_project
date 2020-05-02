#define STM8S003

#include "stm8s.h"
#include "globalVar.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Evalboard I/Os configuration */
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t text[100];
uint16_t* readADC;
uint16_t readedTemp,readedWaterLvl;
uint8_t setTemp,tempTelorance,setWaterLvl,waterLvlTelorance;

volatile uint8_t rxBuffer[100],rxCounter , recivePacketFlag;
/* Private function prototypes -----------------------------------------------*/
void Delay (uint16_t nCount);
static void ADC_Config(void);
static void GPIO_Config(void);
static void TIM2_Config(void);
static void ClOCK_Config(void);
static void UART1_setup(void);
uint16_t* readAllADC(void);
void SendData(uint8_t *data, unsigned int len);
void LoopFunction(void);

/* Private functions ---------------------------------------------------------*/
/* Public functions ----------------------------------------------------------*/

/**
* @brief  Main program.
* @param  None
* @retval None
*/
void main(void)
{
  ClOCK_Config();
  GPIO_Config();
  ADC_Config();
  UART1_setup();
  readADC = calloc(6,sizeof(unsigned char));
  enableInterrupts();
  while (1)
  {
    LoopFunction();
  }
}





void LoopFunction(void){
  //raed temp and water level sensor 
  readADC = readAllADC();
  //calculate temp from sensors value 
  readedTemp = readADC[SENSOR_TEMP_CH]/10;
  //temp control runtime 
  if(readedTemp < setTemp-tempTelorance){
    ON(HEATER);
  }else if (readedTemp > setTemp + tempTelorance){
    OFF(HEATER);
  }
  
  
  //calculate water level from sensors value 
  readedWaterLvl = readADC[SENSOR_WATER_LVL_CH]/102;
  //water level control runtime 
  if(readedWaterLvl < setWaterLvl-waterLvlTelorance){
    ON(WATER_VALVE);
  }else if (readedWaterLvl > setWaterLvl + waterLvlTelorance){
    OFF(WATER_VALVE);
  }
  if(recivePacketFlag==1){
    SendData((uint8_t *)rxBuffer,(unsigned int)rxCounter);
    rxCounter=0;
    recivePacketFlag = 0;
  }
  //send water level and temp to esp12
  //sprintf((char*)text,"pot:%u,%u,%u\r\n",readADC[0],readADC[1]);
  //SendData(text,strlen((const char*)text));
}


void SendData(uint8_t *data, unsigned int len) {
  unsigned int l;
  for (l = 0; l < len; l++) {
    while (!((UART1->SR) & UART1_SR_TC));
    UART1->DR = data[l];
  }
}

uint16_t* readAllADC(void){
  static uint8_t adcBuffer[2][2];
  ADC1->CSR |= (uint8_t)(0x0F); //select all channel 
  ADC1->CR3 |= ADC1_CR3_DBUF; //enable adc buffer
  ADC1->CR2 |= ADC1_CR2_SCAN; //enable adc scan mode
  ADC1->CR1 |= ADC1_CR1_CONT; //enable adc continus mode
  ADC1->CR1 |= ADC1_CR1_ADON; //start converion
  while(ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET);
  adcBuffer[0][0] = (uint8_t)(ADC1->DB2RH)&0x03;
  adcBuffer[0][1] = (uint8_t)(ADC1->DB2RL);
  adcBuffer[1][0] = (uint8_t)(ADC1->DB3RH)&0x03;
  adcBuffer[1][1] = (uint8_t)(ADC1->DB3RL);
  ADC1->CSR &= (uint8_t)(~ADC1_FLAG_EOC);
  ADC1->CR1 &= (uint8_t)(~ADC1_CR1_ADON);
  ADC1->CR1 &= (uint8_t)(~ADC1_CR1_CONT);
  return (uint16_t*)adcBuffer;
}


static void ClOCK_Config(void){
  CLK_HSECmd(DISABLE);
  CLK_LSICmd(DISABLE);
  CLK_HSICmd(ENABLE);
                                
  CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
                                

  
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_TIMER4, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_ADC, ENABLE);
  CLK_PeripheralClockConfig(CLK_PERIPHERAL_UART1, ENABLE);
}


static void GPIO_Config(void)
{
  /* Initialize I/Os in Output Mode */
  GPIO_Init(WATER_POMP, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(AIR_POMP, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(HEATER, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(WATER_VALVE, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(FEADER, GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_Init(LIGHT, GPIO_MODE_OUT_PP_LOW_FAST);
}

static void UART1_setup(void)
{
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_FAST);
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);
  UART1_DeInit();
  
  UART1_Init(9600,
             UART1_WORDLENGTH_8D,
             UART1_STOPBITS_1,
             UART1_PARITY_NO,
             UART1_SYNCMODE_CLOCK_DISABLE,
             UART1_MODE_TXRX_ENABLE);
  UART1_ITConfig(UART1_IT_RXNE,ENABLE);
  UART1_Cmd(ENABLE);
  
}


static void ADC_Config()
{
  GPIO_Init(SENSOR_WATER_LVL, GPIO_MODE_IN_FL_NO_IT);
  GPIO_Init(SENSOR_TEMP, GPIO_MODE_IN_FL_NO_IT);
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
            ADC1_CHANNEL_2,
            ADC1_PRESSEL_FCPU_D18,
            ADC1_EXTTRIG_GPIO,
            DISABLE,
            ADC1_ALIGN_RIGHT,
            ADC1_SCHMITTTRIG_CHANNEL2,
            DISABLE);
  ADC1_Init(ADC1_CONVERSIONMODE_CONTINUOUS,
            ADC1_CHANNEL_3,
            ADC1_PRESSEL_FCPU_D18,
            ADC1_EXTTRIG_GPIO,
            DISABLE,
            ADC1_ALIGN_RIGHT,
            ADC1_SCHMITTTRIG_CHANNEL3,
            DISABLE);
  ADC1_Cmd(ENABLE);
}

static void TIM2_Config(void)
{
  /* Time base configuration */
  TIM2_TimeBaseInit(TIM2_PRESCALER_32, 31200);
  
  /* Prescaler configuration */
  TIM2_PrescalerConfig(TIM2_PRESCALER_32, TIM2_PSCRELOADMODE_IMMEDIATE);
  
  /* Output Compare Active Mode configuration: Channel1 */
  /*
  TIM2_OCMode = TIM2_OCMODE_INACTIVE
  TIM2_OCPolarity = TIM2_OCPOLARITY_HIGH
  TIM2_Pulse = CCR1_Val
  */
  TIM2_OC1Init(TIM2_OCMODE_INACTIVE, TIM2_OUTPUTSTATE_ENABLE,CCR1_Val, TIM2_OCPOLARITY_HIGH);
  TIM2_OC1PreloadConfig(DISABLE);
  
  
  TIM2_ARRPreloadConfig(ENABLE);
  
  /* TIM IT enable */
  TIM2_ITConfig(TIM2_IT_CC1, ENABLE);
  
  
  /* TIM2 enable counter */
  TIM2_Cmd(ENABLE);
}
/**
* @brief Delay
* @param nCount
* @retval None
*/
void Delay(uint16_t nCount)
{
  /* Decrement nCount value */
  while (nCount != 0)
  {
    nCount--;
  }
}

#ifdef USE_FULL_ASSERT

/**
* @brief  Reports the name of the source file and the source line number
*   where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
  ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
* @}
*/


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
