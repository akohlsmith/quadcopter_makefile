/*
 * motorControl.h
 *
 *  Created on: Nov 16, 2013
 *      Author: wetzel
 */

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_

#include <stdint.h>

void PWM_SetDC(uint16_t channel, uint16_t dutycycle);
void TIM3_Config();
void TIM1_Config(void);
void PWM1_Config(int period);
void PWM3_Config(int period);



#endif /* MOTORCONTROL_H_ */
