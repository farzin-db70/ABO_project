#define STM8S003
#define F_CPU 8000000UL
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
uint16_t readADC[2];
uint16_t readedTemp,readedWaterLvl;
uint8_t setTemp,tempTelorance,setWaterLvl,waterLvlTelorance;
uint16_t mainCounter;
uint8_t rxBuffer[100],rxCounter , recivePacketFlag;

/* Private function prototypes -----------------------------------------------*/
void Delay (uint16_t nCount);
static void ADC_Config(void);
static void GPIO_Config(void);
static void TIM2_Config(void);
static void ClOCK_Config(void);
static void UART1_setup(void);
void readAllADC(uint16_t *saveBuffer);
void SendData(uint8_t *data, unsigned int len);
void CleanRecivedBuffer(void);
void Repoet2Esp(void);
void ReadCommandFromEsp(void);
void ControlParamiters(void);
unsigned char Parse_Do(uint8_t * inputBuffer);
void ReadKey_do(void);

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
  enableInterrupts();
  while (1)
  {
    mainCounter++;
    ReadKey_do();
    if(mainCounter%90)ControlParamiters();
    if(mainCounter%100)ReadCommandFromEsp();
    if(mainCounter==1)Repoet2Esp();
    if(mainCounter==500)mainCounter=0;
  }
}


void ReadKey_do(void){
  static char keyDebounce=0;
  
  /*runtime for force feed switch and debouncce*/
  for(unsigned char i=0;i<10;i++){
    if(Read(FORCE_FEED))keyDebounce++;
    Delay(10);
  }
  if(keyDebounce>6){
    ON(FEADER);
    Delay(3000);
    OFF(FEADER);
    return;
  }else{
    keyDebounce=0;
  }
  
  /*runtime for force clean switch and debouncce*/
  for(unsigned char i=0;i<10;i++){
    if(Read(FORCE_CLEAN))keyDebounce++;
    Delay(10);
  }
  if(keyDebounce>6){
    Tgl(WATER_POMP);
    return;
  }else{
    keyDebounce=0;
  }
}


void ReadCommandFromEsp(void){
  unsigned char commandState;
  if(recivePacketFlag==1){
    SendData(rxBuffer,(unsigned int)rxCounter);
    commandState = Parse_Do(rxBuffer);
    if(commandState == ACK){
      Report_ACK();
    }
    CleanRecivedBuffer();
  }
}

unsigned char Parse_Do(uint8_t * inputBuffer){
  unsigned char command = inputBuffer[2];
  switch(command){
  case 0x01:    //run the feeder command
    ON(FEADER);
    Delay(inputBuffer[2]*1000);
    OFF(FEADER);
    return ACK;
  case 0x02:    //lighting command = ON 
    ON(LIGHT);
    return ACK;
  case 0x03:    //lighting command = OFF
    OFF(LIGHT);
    return ACK;
  case 0x04:    //Cleaner motor command = ON 
    ON(WATER_POMP);
    SendData("cl->ON",5);
    return ACK;
  case 0x05:    //Cleaner motor command = OFF
    OFF(WATER_POMP);
    SendData("cl->OF",5);
    return ACK;
  case 0x06:    //air Pomp command = ON
    ON(AIR_POMP);
    SendData("ar->ON",5);
    return ACK;
  case 0x07:    //air Pomp command =OFF
    OFF(AIR_POMP);
    SendData("ar->OF",5);
    return ACK;
  case 0x08:    //change water level and temp limit switch
    OFF(AIR_POMP);
    SendData("ar->OF",5);
    return ACK;
  } 
  return NACK;
}



void Repoet2Esp(void){
  //send water level and temp to esp12
  sprintf((char*)text,"pot:%u,%u\r\n",readADC[0],readADC[1]);
  SendData(text,strlen((const char*)text));
}

/*raed the situation paramiters and control temp and water flow
recomanded -> on version 2: added water acideite */
void ControlParamiters(void){
  static unsigned char tErrRunOnce=1, wErrRunOnce=1;
  //raed temp and water level sensor 
  readAllADC(readADC);
  //calculate temp from sensors value 
  readedTemp = readADC[SENSOR_TEMP_CH]/10;
  //temp control runtime 
  if(readedTemp < setTemp-tempTelorance){
    ON(HEATER);
  }else if (readedTemp > setTemp + tempTelorance){
    OFF(HEATER);
  }
  
  if(readedTemp > setTemp + tempTelorance + 3){
    if(tErrRunOnce)SendData("ERR:overtemp\n",12);
    tErrRunOnce=0;
  }else{
    tErrRunOnce=1;
  }
  
  //calculate water level from sensors value 
  readedWaterLvl = readADC[SENSOR_WATER_LVL_CH]/102;
  //water level control runtime 
  if(readedWaterLvl < setWaterLvl-waterLvlTelorance){
    ON(WATER_VALVE);
  }else if (readedWaterLvl > setWaterLvl + waterLvlTelorance){
    OFF(WATER_VALVE);
  }
  
  if(readedWaterLvl > setWaterLvl + waterLvlTelorance+1){
    if(wErrRunOnce)SendData("ERR:overFlow\n",12);
    wErrRunOnce=0;
  }else{
    wErrRunOnce=1;
  }
  
  //if error flag is on LED error on and green led is of
  if((wErrRunOnce==0) || (tErrRunOnce==0)){
    ON(LED_ERR);
    OFF(LED_NORAML_STATE);
  }else{
    OFF(LED_ERR);
    ON(LED_NORAML_STATE);
  }
}



void CleanRecivedBuffer(void){
  for(unsigned char i=0;i<100;i++)rxBuffer[i]=0; //clean the buffer 
  rxCounter=0;
  recivePacketFlag = 0;
}

void SendData(uint8_t *data, unsigned int len) {
  disableInterrupts();
  unsigned int l;
  for (l = 0; l < len; l++) {
    while (!((UART1->SR) & UART1_SR_TC));
    UART1->DR = data[l];
  }
  enableInterrupts();
}

void readAllADC(uint16_t *saveBuffer){
  uint8_t adcBuffer[2][2];
  ADC1->CR1 |= ADC1_CR1_ADON; //start converion
  ADC1->CSR |= (uint8_t)(0x0F); //select all channel 
  ADC1->CR3 |= ADC1_CR3_DBUF; //enable adc buffer
  ADC1->CR2 |= ADC1_CR2_SCAN; //enable adc scan mode
  ADC1->CR1 |= ADC1_CR1_CONT; //enable adc continus mode
  while(ADC1_GetFlagStatus(ADC1_FLAG_EOC) == RESET);
  adcBuffer[0][0] = (uint8_t)(ADC1->DB2RH)&0x03;
  adcBuffer[0][1] = (uint8_t)(ADC1->DB2RL);
  adcBuffer[1][0] = (uint8_t)(ADC1->DB3RH)&0x03;
  adcBuffer[1][1] = (uint8_t)(ADC1->DB3RL);
  ADC1->CSR &= (uint8_t)(~ADC1_FLAG_EOC);
  ADC1->CR1 &= (uint8_t)(~ADC1_CR1_ADON);
  ADC1->CR1 &= (uint8_t)(~ADC1_CR1_CONT);
  saveBuffer[0] = ((uint16_t*)adcBuffer)[0];
  saveBuffer[1] = ((uint16_t*)adcBuffer)[1];
  Delay(0xFFFF);
}


static void ClOCK_Config(void){
  CLK_DeInit();
  CLK_HSECmd(ENABLE);
  CLK_LSICmd(DISABLE);
  CLK_HSICmd(DISABLE);
  CLK_ClockSwitchConfig(CLK_SWITCHMODE_AUTO,CLK_SOURCE_HSE,DISABLE,CLK_CURRENTCLOCKSTATE_DISABLE);
  CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
  CLK_ClockSecuritySystemEnable();
  
  
  
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
  
  GPIO_Init(FORCE_FEED, GPIO_MODE_IN_PU_IT);
  GPIO_Init(FORCE_CLEAN, GPIO_MODE_IN_PU_IT);
  
  GPIO_Init(LED_ERR, GPIO_MODE_OUT_OD_HIZ_FAST);
  GPIO_Init(LED_NORAML_STATE, GPIO_MODE_OUT_OD_HIZ_FAST);
}

static void UART1_setup(void)
{
  GPIO_Init(GPIOD, GPIO_PIN_5, GPIO_MODE_OUT_PP_HIGH_SLOW);
  GPIO_Init(GPIOD, GPIO_PIN_6, GPIO_MODE_IN_FL_NO_IT);
  UART1_DeInit();
  
  UART1_Init(4800,
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
