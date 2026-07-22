// Simple example to show PWM output on TIM2 Channel 4 (PA3)
// Connect an LED to PA3 to see the PWM signal output

//#include "..\ch32fun\ch32fun.h"
#include <ch32x035.h> /* both X033 and X035 */
#include <stdio.h>
#include <debug.h>


// T2C1: PA0
// T2C2: PA1
// T2C3: PA2
// T2C4: PA3

// T1C1: PB9
// T1C2: PB10
// T1C3: PB11
// T1C4: PB12
#define TIM1_PWM_PIN1 GPIO_Pin_9
#define TIM1_PWM_PIN2 GPIO_Pin_10
#define TIM1_PWM_PIN3 GPIO_Pin_11
#define TIM1_PWM_PIN4 GPIO_Pin_12
uint16_t period = 1000; // gives a PWM frequency above 20 kHz

void TIM1_pwm_init() {
	/// TODO refactor code. see https://github.com/CuriousScientist0/CH32_Microcontroller_Tutorials/blob/main/Part%203%20-%20Timers%20and%20PWM/main.c
	// Enable TIM1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM1, ENABLE);

  //Timer pin config
  GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = TIM1_PWM_PIN1 | TIM1_PWM_PIN2 | TIM1_PWM_PIN3 | TIM1_PWM_PIN4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //alternate function, PP: push-pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);    
	
	TIM_Cmd(TIM1, DISABLE); //Disable timer before (re)configuring it
  TIM_OCInitTypeDef TIM_OCInitStructure = {0};
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

	// Timer base configuration
	TIM_TimeBaseInitStructure.TIM_Period = period-1; //Auto-Reload Register (counter's max value before overflowing)
  TIM_TimeBaseInitStructure.TIM_Prescaler = 1; //Prescaler - Main clock's divider
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //No clock division
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //Counting upwards
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //PWM is also directed to a physical pin
  TIM_OCInitStructure.TIM_Pulse = period/4; //Capture-compare register (PWM changes the OCPolarity when counter reaches this value)
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //PWM is high when the counter starts at 0
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //PWM is also directed to a physical pin
  TIM_OCInitStructure.TIM_Pulse = period/4; //Capture-compare register (PWM changes the OCPolarity when counter reaches this value)
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //PWM is high when the counter starts at 0
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
	
	//! REQUIRED: Channel 1/2 PWM configuration
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
  	
	//! REQUIRED: Channel 3/4 PWM configuration
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
  // Main output enable and update
	TIM_ARRPreloadConfig(TIM1, ENABLE);
  	
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_ARRPreloadConfig(TIM1, ENABLE);

	TIM_Cmd(TIM1, ENABLE);
	PRINT("TIM1 PWM initialized\r\n");
}
void Brake(int8_t drive){
	// output 00 to the 2 control pins of the DRV8847
	/// TODO refactor code. can be done with TIM_OCMode_Inactive / TIM_OCMode_Active
	/// disable timer before (re)configuring it
	switch (drive )
	{
	case 1:
		TIM_SetCompare1(TIM1, period + 1);// speed setpoint where period = full speed
		TIM_SetCompare2(TIM1, period + 1);
		break;
	case 2:
		TIM_SetCompare3(TIM1, period + 1);			// speed setpoint where period is full speed
		TIM_SetCompare4(TIM1, period + 1);			
		break;
		case 3:
		TIM_SetCompare1(TIM2, period + 1);			// speed setpoint where period = full speed
		TIM_SetCompare2(TIM2, period + 1);
		break;
	case 4:
		TIM_SetCompare3(TIM2, period + 1);			// speed setpoint where period is full speed
		TIM_SetCompare4(TIM2, period + 1);			
		break;
	}
}

void SetSpeed(int16_t speed, int8_t drive){
		// speed > 0: FORWARD output 10 to the control pins. pin1 is PWM controled for speed adjustment
		// speed < 0: REVERSE output 01 to the control pins. pin2 is PWM controled for speed adjustment
		// speed = 0: COAST output 00 to the control pins.
    // speed specified as a signed integer, where the absolute value is the PWM duty cycle and the sign indicates direction.
		// speed is limited to the range [-period, period], where period is the PWM period.
		// period +1 is used to indicate a coasting condition, where both control pins are set to 0.
	int16_t sp1 = speed >= 0 ?  speed : period + 1 ;
	int16_t sp2 = speed <= 0 ? -speed : period + 1 ;
	switch (drive )
	{
	case 1:
		TIM_SetCompare1(TIM1, sp1);				// speed setpoint where period is full speed
		TIM_SetCompare2(TIM1, sp2);
		break;
	case 2:
		TIM_SetCompare3(TIM1, sp1);				// speed setpoint where period is full speed
		TIM_SetCompare4(TIM1, sp2);
		break;
	case 3:
		TIM_SetCompare1(TIM2, sp1);				// speed setpoint where period is full speed
		TIM_SetCompare2(TIM2, sp2);	
		break;
	case 4:
		TIM_SetCompare3(TIM2, sp1);				// speed setpoint where period is full speed
		TIM_SetCompare4(TIM2, sp2);	
		break;
	}
}

/*void Brake_TIM1(int8_t drive){
	// output 00 to the 2 control pins of the DRV8847
	if (drive == 1){
		TIM_SetCompare1(TIM1, period + 1);
		TIM_SetCompare2(TIM1, period + 1);
	}
	else{
		TIM_SetCompare3(TIM1, period + 1);			// speed setpoint where period is full speed
		TIM_SetCompare4(TIM1, period + 1);			
	}
}

void setSpeed_TIM1(int16_t speed, int8_t drive){
	// limit speed to boundary
	if (speed > period) speed = period;
	if (speed < -period) speed = -period;

	if (drive == 1){
		// FORWARD output 10 to the control pins. pin1 is PWM controled for speed adjustment
		if (speed >0){
			TIM_SetCompare1(TIM1, speed);				// speed setpoint where period is full speed
			TIM_SetCompare2(TIM1, period + 1);	
		}
		// COAST output 00 to the control pins. 
		if (speed == 0) {
			TIM_SetCompare1(TIM1, 0);						// speed setpoint where period is full speed
			TIM_SetCompare2(TIM1, 0);
		}
		// REVERSE output 01 to the control pins. pin2 is PWM controled for speed adjustment
		if (speed < 0){
			TIM_SetCompare1(TIM1, period + 1);	// speed setpoint where period is full speed
			TIM_SetCompare2(TIM1, speed);
		}

	}
	else{
		// FORWARD output 10 to the control pins. pin1 is PWM controled for speed adjustment
		if (speed >0){
			TIM_SetCompare3(TIM1, speed);				// speed setpoint where period is full speed
			TIM_SetCompare4(TIM1, period + 1);	
		}
		// COAST output 00 to the control pins. 
		if (speed == 0) {
			TIM_SetCompare3(TIM1, 0);						// speed setpoint where period is full speed
			TIM_SetCompare4(TIM1, 0);
		}
		// REVERSE output 01 to the control pins. pin2 is PWM controled for speed adjustment
		if (speed < 0){
			TIM_SetCompare3(TIM1, period + 1);	// speed setpoint where period is full speed
			TIM_SetCompare4(TIM1, speed);
		}
	}
}*/

// TODO where is this used ? 

// T2C1: PA0
// T2C2: PA1
// T2C3: PA2
// T2C4: PA3
#define TIM2_PWM_PIN1 GPIO_Pin_0
#define TIM2_PWM_PIN2 GPIO_Pin_1
#define TIM2_PWM_PIN3 GPIO_Pin_2
#define TIM2_PWM_PIN4 GPIO_Pin_3

void TIM2_pwm_init() {
	/// TODO refactor code. see https://github.com/CuriousScientist0/CH32_Microcontroller_Tutorials/blob/main/Part%203%20-%20Timers%20and%20PWM/main.c
	// Enable TIM1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	//Timer pin config
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = TIM2_PWM_PIN1 | TIM2_PWM_PIN2 | TIM2_PWM_PIN3 | TIM2_PWM_PIN4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //alternate function, PP: push-pull
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);    
	
	TIM_Cmd(TIM2, DISABLE); //Disable timer before (re)configuring it
	TIM_OCInitTypeDef TIM_OCInitStructure = {0};
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};

	// Timer base configuration
	TIM_TimeBaseInitStructure.TIM_Period = period-1; //Auto-Reload Register (counter's max value before overflowing)
  	TIM_TimeBaseInitStructure.TIM_Prescaler = 1; //Prescaler - Main clock's divider
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //No clock division
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //Counting upwards
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //PWM is also directed to a physical pin
	TIM_OCInitStructure.TIM_Pulse = period/4; //Capture-compare register (PWM changes the OCPolarity when counter reaches this value)
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //PWM is high when the counter starts at 0
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //PWM is also directed to a physical pin
	TIM_OCInitStructure.TIM_Pulse = period/4; //Capture-compare register (PWM changes the OCPolarity when counter reaches this value)
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //PWM is high when the counter starts at 0
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	
	//! REQUIRED: Channel 1/2 PWM configuration
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  	
	//! REQUIRED: Channel 3/4 PWM configuration
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
  // Main output enable and update
	TIM_ARRPreloadConfig(TIM2, ENABLE);
  	
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
  	TIM_ARRPreloadConfig(TIM2, ENABLE);

	TIM_Cmd(TIM2, ENABLE);
	PRINT("TIM2 PWM initialized\r\n");
}

/*void Brake_TIM2(int8_t drive){
	// output 00 to the 2 control pins of the DRV8847
	if (drive == 1){
		TIM_SetCompare1(TIM2, period + 1);			// speed setpoint where period = full speed
		TIM_SetCompare2(TIM2, period + 1);
	}
	else{
		TIM_SetCompare3(TIM2, period + 1);			// speed setpoint where period is full speed
		TIM_SetCompare4(TIM2, period + 1);			
	}
}

void setSpeed_TIM2(int16_t speed, int8_t drive){
	if (drive == 1){
		// FORWARD output 10 to the control pins. pin1 is PWM controled for speed adjustment
		if (speed >0){
			TIM_SetCompare1(TIM2, speed);				// speed setpoint where period is full speed
			TIM_SetCompare2(TIM2, period + 1);	
		}
		// COAST output 00 to the control pins. 
		if (speed == 0) {
			TIM_SetCompare1(TIM2, 0);						// speed setpoint where period is full speed
			TIM_SetCompare2(TIM2, 0);
		}
		// REVERSE output 01 to the control pins. pin2 is PWM controled for speed adjustment
		if (speed < 0){
			TIM_SetCompare1(TIM2, period + 1);	// speed setpoint where period is full speed
			TIM_SetCompare2(TIM2, speed);
		}

	}
	else{
		// FORWARD output 10 to the control pins. pin1 is PWM controled for speed adjustment
		if (speed >0){
			TIM_SetCompare3(TIM2, speed);				// speed setpoint where period is full speed
			TIM_SetCompare4(TIM2, period + 1);	
		}
		// COAST output 00 to the control pins. 
		if (speed == 0) {
			TIM_SetCompare3(TIM2, 0);						// speed setpoint where period is full speed
			TIM_SetCompare4(TIM2, 0);
		}
		// REVERSE output 01 to the control pins. pin2 is PWM controled for speed adjustment
		if (speed < 0){
			TIM_SetCompare3(TIM2, period + 1);	// speed setpoint where period is full speed
			TIM_SetCompare4(TIM2, speed);
		}
	}
}*/
