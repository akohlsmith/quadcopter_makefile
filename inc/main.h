/*
 * main.h
 *
 *  Created on: 10 jul 2012
 *      Author: BenjaminVe
 */

#ifndef MAIN_H_
#define MAIN_H_

//Bluetooth cstes
#define BT_BAUD 9600
#define MAX_STRLEN 20 // this is the maximum string length of our string in characters

// Function prototypes
void timing_handler();
/*- Initialisation methods ---------------------------------------------------*/
void initPB0();
void initPA15();
void TIM3_Config();
void TIM1_Config(void);
void PWM1_Config(int period);
void PWM3_Config(int period);
void init_LED(void);
void init_BT_serial();
void init_USART1(uint32_t baudrate);
void USART1_IRQHandler(void);


#endif /* MAIN_H_ */
