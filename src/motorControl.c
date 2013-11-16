/*
 * motorControl.c
 *
 *  Created on: Nov 16, 2013
 *      Author: wetzel
 */

#include "motorControl.h"
#include "stm32f4xx_conf.h"

//PWM
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0,
                Channel4Pulse = 0;

void PWM_SetDC(uint16_t channel, uint16_t dutycycle) {
        if (channel == 1) {
                TIM3->CCR1 = dutycycle;
                TIM1->CCR1 = dutycycle;

        } else if (channel == 2) {
                TIM3->CCR2 = dutycycle;
                TIM1->CCR2 = dutycycle;
        } else if (channel == 3) {
                TIM3->CCR3 = dutycycle;
                TIM1->CCR3 = dutycycle;
        } else if (channel == 4) {
                TIM3->CCR4 = dutycycle;
                TIM1->CCR4 = dutycycle;
        }
}

void TIM3_Config() {
        GPIO_InitTypeDef GPIO_InitStructure;

        /* TIM3 clock enable */
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

        /* GPIOC, GPIOB and GPIOA clock enable */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOB, ENABLE);

        /* GPIOC Configuration: TIM3 CH1 (PC6) and TIM3 CH2 (PC7) */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOC, &GPIO_InitStructure);

        /* GPIOB Configuration:  TIM3 CH3 (PB0) and TIM3 CH4 (PB1) */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        /* Connect TIM3 pins to AF2 */
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
}

/**
 * @brief  Configure the TIM1 Pins.
 * @param  None
 * @retval None
 */
void TIM1_Config(void) {
        GPIO_InitTypeDef GPIO_InitStructure;

        /* GPIOA and GPIOB clocks enable */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

        /* TIM1 clock enable */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

        /* GPIOA Configuration: Channel 1 to 4 as alternate function push-pull */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13
                        | GPIO_Pin_14;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOE, &GPIO_InitStructure);

        //  /* Connect TIM pins to AF1 */
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
        GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
}

void PWM1_Config(int period) {

        /* -----------------------------------------------------------------------
         1/ Generate 3 complementary PWM signals with 3 different duty cycles:

         In this example TIM1 input clock (TIM1CLK) is set to 2 * APB2 clock (PCLK2),
         since APB2 prescaler is different from 1 (APB2 Prescaler = 2, see system_stm32f4xx.c file).
         TIM1CLK = 2 * PCLK2
         PCLK2 = HCLK / 2
         => TIM1CLK = 2*(HCLK / 2) = HCLK = SystemCoreClock

         To get TIM1 counter clock at 168 MHz, the prescaler is computed as follows:
         Prescaler = (TIM1CLK / TIM1 counter clock) - 1
         Prescaler = (SystemCoreClock / 168 MHz) - 1 = 0

         The objective is to generate PWM signal at 17.57 KHz:
         - TIM1_Period = (SystemCoreClock / 17570) - 1

         To get TIM1 output clock at 17.57 KHz, the period (ARR) is computed as follows:
         ARR = (TIM1 counter clock / TIM1 output clock) - 1
         = 9561

         The Three Duty cycles are computed as the following description:

         TIM1 Channel1 duty cycle = (TIM1_CCR1/ TIM1_ARR)* 100 = 50%
         TIM1 Channel2 duty cycle = (TIM1_CCR2/ TIM1_ARR)* 100 = 25%
         TIM1 Channel3 duty cycle = (TIM1_CCR3/ TIM1_ARR)* 100 = 12.5%

         The Timer pulse is calculated as follows:
         - TIM1_CCRx = (DutyCycle * TIM1_ARR)/ 100

         2/ Insert a dead time equal to (11/SystemCoreClock) ns

         3/ Configure the break feature, active at High level, and using the automatic
         output enable feature

         4/ Use the Locking parameters level1.

         Note:
         SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
         Each time the core clock (HCLK) changes, user had to call SystemCoreClockUpdate()
         function to update SystemCoreClock variable value. Otherwise, any configuration
         based on this variable will be incorrect.
         ----------------------------------------------------------------------- */

        /* Time Base configuration */
        uint16_t PrescalerValue = 0;
        /* Compute the prescaler value */
        PrescalerValue = (uint16_t)((SystemCoreClock / 2) / 1600000) - 1;

        TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseStructure.TIM_Period = period;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

        TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

        /* Channel 1to 4 Configuration in PWM mode */
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
        TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
        TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
        TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

        TIM_OC1Init(TIM1, &TIM_OCInitStructure);

        TIM_OCInitStructure.TIM_Pulse = Channel2Pulse;
        TIM_OC2Init(TIM1, &TIM_OCInitStructure);

        TIM_OCInitStructure.TIM_Pulse = Channel3Pulse;
        TIM_OC3Init(TIM1, &TIM_OCInitStructure);

        TIM_OCInitStructure.TIM_Pulse = Channel4Pulse;
        TIM_OC4Init(TIM1, &TIM_OCInitStructure);

        /* Automatic Output enable, Break, dead time and lock configuration*/
        TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
        TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
        TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
        TIM_BDTRInitStructure.TIM_DeadTime = 11;
        TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
        TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
        TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

        TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

        /* TIM1 counter enable */
        TIM_Cmd(TIM1, ENABLE);

        /* Main Output Enable */
        TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PWM3_Config(int period) {
        uint16_t PrescalerValue = 0;
        /* Compute the prescaler value */
        PrescalerValue = (uint16_t)((SystemCoreClock / 4) / 1600000) - 1;
        /* Time base configuration */
        TIM_TimeBaseStructure.TIM_Period = period;
        TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
        TIM_TimeBaseStructure.TIM_ClockDivision = 0;
        TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
        TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
        /* PWM1 Mode configuration: Channel1 */
        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
        TIM_OC1Init(TIM3, &TIM_OCInitStructure);
        TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
        /* PWM1 Mode configuration: Channel2 */
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OC2Init(TIM3, &TIM_OCInitStructure);
        TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
        /* PWM1 Mode configuration: Channel3 */
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OC3Init(TIM3, &TIM_OCInitStructure);
        TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
        /* PWM1 Mode configuration: Channel4 */
        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
        TIM_OCInitStructure.TIM_Pulse = 0;
        TIM_OC4Init(TIM3, &TIM_OCInitStructure);
        TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
        TIM_ARRPreloadConfig(TIM3, ENABLE);
        /* TIM3 enable counter */
        TIM_Cmd(TIM3, ENABLE);
}



