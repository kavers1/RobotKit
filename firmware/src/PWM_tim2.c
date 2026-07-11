// Simple example to show PWM output on TIM2 Channel 4 (PA3)
// Connect an LED to PA3 to see the PWM signal output

#include "ch32fun.h"
#include <stdio.h>

// T1C1: PB9
// T1C2: PB10
// T1C3: PB11
// T1C4: PB12

// T2C1: PA0
// T2C2: PA1
// T2C3: PA2
// T2C4: PA3

// TODO where is this used ? 
#define PWM_PIN1 PA0
#define PWM_PIN2 PA1
#define PWM_PIN3 PA2
#define PWM_PIN4 PA3
u16 period = 10000;

void TIM2_pwm_init() {
	// Enable TIM2 clock
	RCC->APB1PCENR |= RCC_APB1Periph_TIM2;
	
	// Timer base configuration
	TIM2->PSC = 4 - 1;					// Prescaler
	TIM2->ATRLR = period - 1;			// PWM period
	
	//forward motor1
	TIM2->CH1CVR = speeddrv1;			// speed setpoint where period is full speed
	TIM2->CH2CVR = period + 1;			// speed setpoint where period is full speed
/* reverse
	TIM2->CH1CVR = period + 1;			// speed setpoint where period is full speed
	TIM2->CH2CVR = speeddrv1;			// speed setpoint where period is full speed
*/
	//! REQUIRED: Channel 1/2 PWM configuration
	TIM2->CHCTLR1 |= (0b110 << 4);		// OC1M = 110 (PWM Mode 1) - bits [14:12]
	TIM2->CHCTLR1 |= TIM_OC1PE;	  		// Channel 1 Preload enable - bit 11
	TIM2->CHCTLR1 |= (0b110 << 12);		// OC2M = 110 (PWM Mode 1) - bits [14:12]
	TIM2->CHCTLR1 |= TIM_OC1PE;		  	// Channel 2 Preload enable - bit 11
	
	//forward motor2
	TIM2->CH3CVR = speeddrv2;			// speed setpoint where period is full speed
	TIM2->CH4CVR = period + 1;			// speed setpoint where period is full speed
/* reverse
	TIM2->CH3CVR = period + 1;			// speed setpoint where period is full speed
	TIM2->CH4CVR = speeddrv2;			// speed setpoint where period is full speed
*/
	//! REQUIRED: Channel 3/4 PWM configuration
	TIM2->CHCTLR2 |= (0b110 << 4);		// OC3M = 110 (PWM Mode 1) - bits [14:12]
	TIM2->CHCTLR2 |= TIM_OC3PE;	  		// Channel 3 Preload enable - bit 11
	TIM2->CHCTLR2 |= (0b110 << 12);		// OC4M = 110 (PWM Mode 1) - bits [14:12]
	TIM2->CHCTLR2 |= TIM_OC4PE;		  	// Channel 4 Preload enable - bit 11
	TIM2->CTLR1 = TIM_ARPE;				    // Auto-reload preload enable

	
	// Enable Channel 1,2,3 and 4 output
	/// TODO if needed set CCxP to invert channel output
	TIM2->CCER = (TIM2->CCER & ~(TIM_CC1E | TIM_CC1P)) | TIM_CC1E;  // enable output 1 clear polarity of output 1
	TIM2->CCER = (TIM2->CCER & ~(TIM_CC2E | TIM_CC2P)) | TIM_CC2E;  // enable output 2 clear polarity of output 2
	TIM2->CCER = (TIM2->CCER & ~(TIM_CC3E | TIM_CC3P)) | TIM_CC3E;  // enable output 3 clear polarity of output 3
	TIM2->CCER = (TIM2->CCER & ~(TIM_CC4E | TIM_CC4P)) | TIM_CC4E;  // enable output 4 clear polarity of output 4
	
	// Main output enable and update
	TIM2->BDTR |= TIM_MOE;				// Main output enable
	TIM2->SWEVGR |= TIM_UG;				// Update registers
	TIM2->CTLR1 |= TIM_CEN;				// Start timer
}

void Brake(int8_t drive){
	// output 00 to the 2 control pins of the DRV8847
	if (drive = 1){
		TIM2->CH1CVR = period + 1;			// speed setpoint where period = full speed
		TIM2->CH2CVR = period + 1;
	}
	else{
		TIM2->CH3CVR = period + 1;			// speed setpoint where period is full speed
		TIM2->CH4CVR = period + 1;			
	}
}
void setSpeed(int16_t speed, int8_t drive){
	if (drive = 1){
		// FORWARD output 10 to the control pins. pin1 is PWM controled for speed adjustment
		if (speed >0){
			TIM2->CH1CVR = speed;				// speed setpoint where period is full speed
			TIM2->CH2CVR = period + 1;	
		}
		// COAST output 00 to the control pins. 
		if (speed == 0) {
			TIM2->CH1CVR = 0;						// speed setpoint where period is full speed
			TIM2->CH2CVR = 0;
		}
		// REVERSE output 01 to the control pins. pin2 is PWM controled for speed adjustment
		if (speed < 0){
			TIM2->CH1CVR = period + 1;	// speed setpoint where period is full speed
			TIM2->CH2CVR = speed;
		}

	}
	else{
		// FORWARD output 10 to the control pins. pin1 is PWM controled for speed adjustment
		if (speed >0){
			TIM2->CH3CVR = speed;				// speed setpoint where period is full speed
			TIM2->CH4CVR = period + 1;	
		}
		// COAST output 00 to the control pins. 
		if (speed == 0) {
			TIM2->CH3CVR = 0;						// speed setpoint where period is full speed
			TIM2->CH4CVR = 0;
		}
		// REVERSE output 01 to the control pins. pin2 is PWM controled for speed adjustment
		if (speed < 0){
			TIM2->CH3CVR = period + 1;	// speed setpoint where period is full speed
			TIM2->CH4CVR = speed;
		}
	}
}

