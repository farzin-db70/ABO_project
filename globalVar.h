#ifndef __GLBALVAR_H_
#define __GLBALVAR_H_

#define MODE_ADDRESS    0x40A5
#define SYS_STATE_ADD   0x40B5

#define CCR1_Val ((uint16_t)55)
#define CCR2_Val ((uint16_t)488)
#define CCR3_Val  ((uint16_t)244)

#define RELAY1                  GPIOD, GPIO_PIN_3    
#define RELAY2                  GPIOC, GPIO_PIN_7 
#define RELAY3                  GPIOD, GPIO_PIN_1    
#define RELAY4                  GPIOC, GPIO_PIN_4

#define SENSOR_1                GPIOC, GPIO_PIN_4
#define SENSOR_2                GPIOD, GPIO_PIN_2

#define MOTOR_1                 GPIOA, GPIO_PIN_3
#define MOTOR_2                 GPIOC, GPIO_PIN_3

#define SENSOR_1_CH             (uint8_t)2
#define SENSOR_2_CH             (uint8_t)3


#define MotorON()               GPIO_WriteHigh(RELAY1)
#define MotorOFF()              GPIO_WriteLow(RELAY1)

#endif
