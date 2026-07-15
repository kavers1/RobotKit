#ifndef _PWM_TIM2_H
#define _PWM_TIM2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch32x035.h> /* both X033 and X035 */

#define FUNCONF_USE_DEBUGPRINTF 1

void TIM1_pwm_init() ;
void Brake_TIM1( uint8_t drive);
void setSpeed_TIM1(int16_t speed, uint8_t drive);

void TIM2_pwm_init() ;
void Brake( uint8_t drive);
void setSpeed(int16_t speed, uint8_t drive);

void Brake( uint8_t drive);
void SetSpeed(int16_t speed, uint8_t drive);

#endif
#ifdef __cplusplus
}
#endif