#ifndef __GLBALVAR_H_
#define __GLBALVAR_H_

#define MODE_ADDRESS    0x40A5
#define SYS_STATE_ADD   0x40B5

#define CCR1_Val ((uint16_t)55)
#define CCR2_Val ((uint16_t)488)
#define CCR3_Val  ((uint16_t)244)

#define WATER_POMP              GPIOD, GPIO_PIN_3    
#define AIR_POMP                GPIOC, GPIO_PIN_7 
#define HEATER                  GPIOD, GPIO_PIN_1    
#define WATER_VALVE             GPIOC, GPIO_PIN_4

#define SENSOR_WATER_LVL        GPIOC, GPIO_PIN_4
#define SENSOR_TEMP             GPIOD, GPIO_PIN_2

#define FEADER                  GPIOA, GPIO_PIN_3
#define LIGHT                   GPIOC, GPIO_PIN_3

#define SENSOR_WATER_LVL_CH     (uint8_t)0
#define SENSOR_TEMP_CH          (uint8_t)1


#define ON(RELAY)               GPIO_WriteHigh(RELAY)
#define OFF(RELAY)              GPIO_WriteLow(RELAY)

#endif
