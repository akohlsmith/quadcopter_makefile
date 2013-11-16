#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <math.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "main.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

#include "MPU6050.h"
#include "communication.h"

#define CONNECTED_LENGTH 20

#define ACC_X   0   /*!< X accelerator value */
#define ACC_Y   1   /*!< Y accelerator value */
#define ACC_Z   2   /*!< Z accelerator value */
#define TEMP    6   /*!< TEMP value */
#define GYRO_X  3   /*!< X gyroscope value */
#define GYRO_Y  4   /*!< Y gyroscope value */
#define GYRO_Z  5   /*!< Z gyroscope value */

#define PI 3.1415
#define RAD_TO_DEG 57.2957795

/* Kalman filter variables */
double Q_angle; // Process noise variance for the accelerometer
double Q_bias; // Process noise variance for the gyro bias
double R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

double angle[3]; // The angle calculated by the Kalman filter - part of the 2x1 state matrix
double bias[3]; // The gyro bias calculated by the Kalman filter - part of the 2x1 state matrix
double rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

double P[3][2][2]; // Error covariance matrix - This is a 2x2 matrix
double K[2]; // Kalman gain - This is a 2x1 matrix
double y; // Angle difference - 1x1 matrix
double S; // Estimate error - 1x1 matrix

double accXangle, accYangle; // Angle calculate using the accelerometer
double temp; // Temperature
double gyroXangle, gyroYangle; // Angle calculate using the gyro
double compAngleX, compAngleY; // Calculate the angle using a complementary filter
double kalAngleX, kalAngleY; // Calculate the angle using a Kalman filter

//UART connection
static __IO uint32_t TimingDelay;
char received_string[MAX_CMD_SIZE]; // this will hold the received string
char rcvd[CMD_HISTORY_SIZE][4];

//Bt connection
uint8_t connected[CONNECTED_LENGTH];
uint8_t idx = 0;
bool isConnected = FALSE;
bool init = TRUE;

//PWM
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef TIM_OCInitStructure;
TIM_BDTRInitTypeDef TIM_BDTRInitStructure;
uint16_t TimerPeriod = 0;
uint16_t Channel1Pulse = 0, Channel2Pulse = 0, Channel3Pulse = 0,
                Channel4Pulse = 0;

uint16_t rc_channel[4] = { 0, 0, 0, 0 }; /*!< Values of the RC channels */

// Private variables
volatile uint32_t time_var1, time_var2;
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

// Private function prototypes
void Delay(volatile uint32_t nCount);
void initUSB();

void calculation_test();

int16_t AccelGyro[7] = { 0 };
double kal[3];

void Kalman() {
	/* We will set the varibles like so, these can also be tuned by the user */
	Q_angle = 0.001;
	Q_bias = 0.003;
	R_measure = 0.03;
	int i = 0;
	for (; i < 3; i++) {
		bias[i] = 0; // Reset bias
		P[i][0][0] = 0; // Since we assume tha the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
		P[i][0][1] = 0;
		P[i][1][0] = 0;
		P[i][1][1] = 0;
		kal[i] = 0;
	}
}
// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double getAngle(double newAngle, double newRate, double dt, int key) {
	// KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
	// Modified by Kristian Lauszus
	// See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	rate = newRate - bias[key];
	angle[key] += dt * rate;

	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P[key][0][0] += dt
			* (dt * P[key][1][1] - P[key][0][1] - P[key][1][0] + Q_angle);
	P[key][0][1] -= dt * P[key][1][1];
	P[key][1][0] -= dt * P[key][1][1];
	P[key][1][1] += Q_bias * dt;

	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	S = P[key][0][0] + R_measure;
	/* Step 5 */
	K[0] = P[key][0][0] / S;
	K[1] = P[key][1][0] / S;

	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	y = newAngle - angle[key];
	/* Step 6 */
	angle[key] += K[0] * y;
	bias[key] += K[1] * y;

	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	P[key][0][0] -= K[0] * P[key][0][0];
	P[key][0][1] -= K[0] * P[key][0][1];
	P[key][1][0] -= K[1] * P[key][0][0];
	P[key][1][1] -= K[1] * P[key][0][1];

	return angle[key];
}

int main(void) {
	uint16_t j = 0;

	initUSB();
	init_LED();

	//PWM config (motor control)
	TIM1_Config();
	TIM3_Config();
	PWM1_Config(10000);
	PWM3_Config(10000);

	//Initialize the bluetooth
	//Init the array to test the bluetooth connexion status
	for (; j < CONNECTED_LENGTH; j++) {
		connected[j] = 0;
	}
	init_BT_serial();

	int a = 0;
	/*
	 * Disable STDOUT buffering. Otherwise nothing will be printed
	 * before a newline character or when the buffer is flushed.
	 */
	setbuf(stdout, NULL);

	//Initialization of the accelerometer
	MPU6050_I2C_Init();
	MPU6050_Initialize();

	if (MPU6050_TestConnection() == TRUE) {
		// connection success
		printf("i2c succes\r\n");
		Kalman();
		//Init angles
		MPU6050_GetRawAccelGyro(AccelGyro);
		// atan2 outputs the value of -p to p (radians) - see http://en.wikipedia.org/wiki/Atan2
		// We then convert it to 0 to 2p and then from radians to degrees
		accYangle = (atan2(AccelGyro[ACC_X], AccelGyro[ACC_Z]) + PI)
				* RAD_TO_DEG;
		accXangle = (atan2(AccelGyro[ACC_Y], AccelGyro[ACC_Z]) + PI)
				* RAD_TO_DEG;

		angle[0] = accXangle; // Set starting angle
		angle[1] = accYangle;
		gyroXangle = accXangle;
		gyroYangle = accYangle;
		compAngleX = accXangle;
		compAngleY = accYangle;

		while (1) {
			//Read MPU6050
			GPIO_SetBits(GPIOD, GPIO_Pin_12 );
			Delay(200);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12 );
			Delay(200);
			MPU6050_GetRawAccelGyro(AccelGyro);
			accYangle = (atan2(AccelGyro[ACC_X], AccelGyro[ACC_Z]) + PI)
					* RAD_TO_DEG;
			accXangle = (atan2(AccelGyro[ACC_Y], AccelGyro[ACC_Z]) + PI)
					* RAD_TO_DEG;
			double gyroXrate = (double) AccelGyro[GYRO_X] / 131.0;
			double gyroYrate = -((double) AccelGyro[GYRO_Y] / 131.0);

			kalAngleX = getAngle(accXangle, gyroXrate, 0.4, 0); // Calculate the angle using a Kalman filter
			kalAngleY = getAngle(accYangle, gyroYrate, 0.4, 1);
			//kal[0]=getAngle((double) AccelGyro[ACC_X], (double) AccelGyro[GYRO_X], 0.4, 0);
			//kal[1]=getAngle((double) AccelGyro[ACC_Y], (double) AccelGyro[GYRO_Y], 0.4, 1);
			//kal[2]=getAngle((double) AccelGyro[ACC_Z], (double) AccelGyro[GYRO_Z], 0.4, 2);
			/*
			 printf("AccX:%s%.7d$, AccY:%s%.7d$ ,AccZ:%s%.7d$\r\nGyrX:%s%.7d$, GyrY:%s%.7d$ ,GyrZ:%s%.7d$\r\nTemp: $%.7d$\r\nKalX:$%f$, KalY:$%f$, KalZ:$%f$\r\n",
			 AccelGyro[ACC_X]>0?"$0":"$",
			 AccelGyro[ACC_X],
			 AccelGyro[ACC_Y]>0?"$0":"$",
			 AccelGyro[ACC_Y],AccelGyro[ACC_Z]>0?"$0":"$",
			 AccelGyro[ACC_Z], AccelGyro[GYRO_X]>0?"$0":"$",
			 AccelGyro[GYRO_X],AccelGyro[GYRO_Y]>0?"$0":"$",
			 AccelGyro[GYRO_Y],AccelGyro[GYRO_Z]>0?"$0":"$",
			 AccelGyro[GYRO_Z],AccelGyro[TEMP],
			 kalAngleX,
			 kalAngleY,
			 kal[2]);
			 */
			printf("AccX:$%f$, AccY:$%f$ KalX:$%f$, KalY:$%f$\r\n",
					accXangle - 180, accYangle - 180, kalAngleX - 180,
					kalAngleY - 180);
		}
	} else {
		// connection faile

		while (1) {

			GPIO_SetBits(GPIOD, GPIO_Pin_12 );
			Delay(500);
			GPIO_ResetBits(GPIOD, GPIO_Pin_12 );
			Delay(500);
			printf("i2c fail\r\n");
			printf("Time:%d\n", a++);
		}
	}

	return 0;
}

void calculation_test() {
	float a = 1.001;
	int iteration = 0;

	for (;;) {
		GPIO_SetBits(GPIOD, GPIO_Pin_12 );
		Delay(500);
		GPIO_ResetBits(GPIOD, GPIO_Pin_12 );
		Delay(500);

		time_var2 = 0;
		for (int i = 0; i < 1000000; i++) {
			a += 0.01 * sqrtf(a);
		}

		printf("Time:      %lu ms\n\r", time_var2);
		printf("Iteration: %i\n\r", iteration);
		printf("Value:     %.5f\n\n\r", a);

		iteration++;
	}
}

/*- Initialisation methods ---------------------------------------------------*/

void initPB0() {
        GPIO_InitTypeDef GPIO_InitStructure;

        /* Enable the GPIO_LED Clock */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

        /* Configure the GPIO_LED pin */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void initPA15() {
        GPIO_InitTypeDef GPIO_InitStructure;

        /* Enable the GPIO_LED Clock */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

        /* Configure the GPIO_LED pin */
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
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

/* This funcion shows how to initialize
 * the GPIO pins on GPIOD and how to configure
 * them as inputs and outputs
 */
void init_LED(void) {

        /* This TypeDef is a structure defined in the
         * ST's library and it contains all the properties
         * the corresponding peripheral has, such as output mode,
         * pullup / pulldown resistors etc.
         *
         * These structures are defined for every peripheral so
         * every peripheral has it's own TypeDef. The good news is
         * they always work the same so once you've got a hang
         * of it you can initialize any peripheral.
         *
         * The properties of the periperals can be found in the corresponding
         * header file e.g. stm32f4xx_gpio.h and the source file stm32f4xx_gpio.c
         */
        GPIO_InitTypeDef GPIO_InitStruct;

        /* This enables the peripheral clock to the GPIOD IO module
         * Every peripheral's clock has to be enabled
         *
         * The STM32F4 Discovery's User Manual and the STM32F407VGT6's
         * datasheet contain the information which peripheral clock has to be used.
         *
         * It is also mentioned at the beginning of the peripheral library's
         * source file, e.g. stm32f4xx_gpio.c
         */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

        /* In this block of instructions all the properties
         * of the peripheral, the GPIO port in this case,
         * are filled with actual information and then
         * given to the Init function which takes care of
         * the low level stuff (setting the correct bits in the
         * peripheral's control register)
         *
         *
         * The LEDs on the STM324F Discovery are connected to the
         * pins PD12 thru PD15
         */
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13
                        | GPIO_Pin_12; // we want to configure all LED GPIO pins
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // we want the pins to be an output
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // this sets the GPIO modules clock speed
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // this sets the pin type to push / pull (as opposed to open drain)
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // this sets the pullup / pulldown resistors to be inactive
        GPIO_Init(GPIOD, &GPIO_InitStruct); // this finally passes all the values to the GPIO_Init function which takes care of setting the corresponding bits.
}

/**
 *Method called to init the gpio to communicate with the bluetooth serial module
 */
void init_BT_serial() {
        //used to monitor the connection
        initPA15();
        init_USART1(BT_BAUD);
}

/* This funcion initializes the USART1 peripheral
 *
 * Arguments: baudrate --> the baudrate at which the USART is
 *                                                    supposed to operate
 */
void init_USART1(uint32_t baudrate) {

        /* This is a concept that has to do with the libraries provided by ST
         * to make development easier the have made up something similar to
         * classes, called TypeDefs, which actually just define the common
         * parameters that every peripheral needs to work correctly
         *
         * They make our life easier because we don't have to mess around with
         * the low level stuff of setting bits in the correct registers
         */
        GPIO_InitTypeDef GPIO_InitStruct; // this is for the GPIO pins used as TX and RX
        USART_InitTypeDef USART_InitStruct; // this is for the USART1 initilization
        NVIC_InitTypeDef NVIC_InitStructure; // this is used to configure the NVIC (nested vector interrupt controller)

        /* enable APB2 peripheral clock for USART1
         * note that only USART1 and USART6 are connected to APB2
         * the other USARTs are connected to APB1
         */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        /* enable the peripheral clock for the pins used by
         * USART1, PB6 for TX and PB7 for RX
         */
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

        /* This sequence sets up the TX and RX pins
         * so they work correctly with the USART1 peripheral
         */
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // Pins 6 (TX) and 7 (RX) are used
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; // the pins are configured as alternate function so the USART peripheral has access to them
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // this defines the IO speed and has nothing to do with the baudrate!
        GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // this defines the output type as push pull mode (as opposed to open drain)
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // this activates the pullup resistors on the IO pins

        GPIO_Init(GPIOB, &GPIO_InitStruct); // now all the values are passed to the GPIO_Init() function which sets the GPIO registers

        /* The RX and TX pins are now connected to their AF
         * so that the USART1 can take over control of the
         * pins
         */
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

        /* Now the USART_InitStruct is used to define the
         * properties of USART1
         */
        USART_InitStruct.USART_BaudRate = baudrate; // the baudrate is set to the value we passed into this init function
        USART_InitStruct.USART_WordLength = USART_WordLength_8b; // we want the data frame size to be 8 bits (standard)
        USART_InitStruct.USART_StopBits = USART_StopBits_1; // we want 1 stop bit (standard)
        USART_InitStruct.USART_Parity = USART_Parity_No; // we don't want a parity bit (standard)
        USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // we don't want flow control (standard)
        USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; // we want to enable the transmitter and the receiver
        USART_Init(USART1, &USART_InitStruct); // again all the properties are passed to the USART_Init function which takes care of all the bit setting

        /* Here the USART1 receive interrupt is enabled
         * and the interrupt controller is configured
         * to jump to the USART1_IRQHandler() function
         * if the USART1 receive interrupt occurs
         */
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); // enable the USART1 receive interrupt

        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; // we want to configure the USART1 interrupts
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; // this sets the priority group of the USART1 interrupts
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; // this sets the subpriority inside the group
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; // the USART1 interrupts are globally enabled
        NVIC_Init(&NVIC_InitStructure); // the properties are passed to the NVIC_Init function which takes care of the low level stuff

        // finally this enables the complete USART1 peripheral
        USART_Cmd(USART1, ENABLE);
}

/*- Interruption handler -----------------------------------------------------*/

// this is the interrupt request handler (IRQ) for ALL USART1 interrupts
void USART1_IRQHandler(void) {

        // check if the USART1 receive interrupt flag was set
        if (USART_GetITStatus(USART1, USART_IT_RXNE)) {

                init = FALSE;
                static uint8_t k = 0;
                static uint8_t cnt = 0; // this counter is used to determine the string length
                char t = USART1->DR; // the character from the USART1 data register is saved in t
                received_string[cnt] = t;
                cnt++;
                if(cnt >= 4){
                  memcpy(&rcvd[k],&received_string,4);
                  analyseString(k);
                  cnt = 0;
                  k++;
                  if(k >= CMD_HISTORY_SIZE)k=0;
                }
                //check if the received character is not the LF character (used to determine end of string)
                // or the if the maximum string length has been been reached

//                if( (t != '\n') && (cnt < MAX_STRLEN) ){
//                        received_string[cnt] = t;
//                        cnt++;
//                        GPIOD->BSRRL = 0x8000; // this sets LED4
//                }
//                else{ // otherwise reset the character counter and print the received string
//                        GPIOD->BSRRH = 0x8000; // this sets LED4
//                        analyseString(cnt);
//                        cnt = 0;
//                }
        }
}

void initUSB() {
	GPIO_InitTypeDef GPIO_InitStructure;

	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1) {
		};
	}

	// ---------- GPIO -------- //
	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14
			| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// ------------- USB -------------- //
	USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb,
			&USR_cb);
}

/*
 * Called from systick handler
 */
void timing_handler() {
	if (time_var1) {
		time_var1--;
	}

	time_var2++;
}

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nCount) {
	time_var1 = nCount;
	while (time_var1) {
	};
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {

}

