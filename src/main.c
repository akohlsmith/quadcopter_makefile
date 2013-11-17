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
#include "stabilisation.h"
#include "motorControl.h"

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

//UART connection
static __IO uint32_t TimingDelay;
char received_string[MAX_CMD_SIZE]; // this will hold the received string
char rcvd[CMD_HISTORY_SIZE][4];

//Bt connection
uint8_t connected[CONNECTED_LENGTH];
uint8_t idx = 0;
bool isConnected = FALSE;
bool init = TRUE;

uint16_t rc_channel[4] = { 0, 0, 0, 0 }; /*!< Values of the RC channels */

// Private variables
volatile uint32_t time_var1, time_var2;
__ALIGN_BEGIN USB_OTG_CORE_HANDLE USB_OTG_dev __ALIGN_END;

// Private function prototypes
void Delay(volatile uint32_t nCount);
void initUSB();

void calculation_test();

int16_t AccelGyro[7] = { 0 };

int main(void) {
	uint16_t j = 0;

	initUSB();
	init_LED();

	//PWM config (motor control)
	TIM1_Config();
	TIM3_Config();
	PWM1_Config(10000);
	PWM3_Config(10000);

	PWM_SetDC(1, 100); //PE9 | PC6//ON 2ms
	PWM_SetDC(2, 100); //PE11 | PC 7
	PWM_SetDC(3, 100); //PE13
	PWM_SetDC(4, 100); //PE14

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
//			printf("AccX:$%f$, AccY:$%f$ KalX:$%f$, KalY:$%f$\r\n",
//					accXangle - 180, accYangle - 180, kalAngleX - 180,
//					kalAngleY - 180);
			//printf("%d, %d, %d, %d\r\n",rc_channel[RC_SPEED],rc_channel[RC_PITCH],rc_channel[RC_ROLL],rc_channel[RC_YAW]);
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
		static uint16_t i = 0;
        // check if the USART1 receive interrupt flag was set
        if (USART_GetITStatus(USART1, USART_IT_RXNE)) {

                init = FALSE;
                static uint8_t k = 0;
                static uint8_t cnt = 0; // this counter is used to determine the string length
                char t = USART1->DR; // the character from the USART1 data register is saved in t
                received_string[cnt] = t;
                cnt++;
                if(cnt >= CMD_SIZE){
                  memcpy(&rcvd[k],&received_string,CMD_SIZE);
                  analyseString(k);
                  printf("%.5d: %d, %d, %d, %d\r\n",i++,received_string[0],received_string[1],received_string[2],received_string[3]);
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

