#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

//#include <ch32x035.h> /* both X033 and X035 */
//#include "system.h"

#define FUNCONF_USE_DEBUGPRINTF 1
void TIM2_pwm_init() ;
void Brake( int8_t drive);
void setSpeed(int16_t speed, int8_t drive);


#endif
#ifdef __cplusplus
}
#endif

