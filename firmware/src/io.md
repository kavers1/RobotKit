# RobotKit Fri3d 2026
## IO assignment CH32X35
| pin | name | fnc | alt | description |
|---|----|---|---|---|
|1|PC15|CC2||USB PD communication channel
|2|VDD|VDD||power supply
|3 |PC0 |AI1| alt TX2| analog input not implemented yet
| 4|PC3 |AI2 || ananlog input not implemented yet
| 5|PA0 |DRV2 IN1 ||PWM output TIM2 hooked up to IN1 of second DRV8847S
| 6|PA1| DRV2 IN2 ||PWM output TIM2 hooked up to IN2 of second DRV8847S
| 7|PA2 |DRV2 IN3 ||PWM output TIM2 hooked up to IN3 of second DRV8847S
| 8|PA3 |DRV2 IN4 ||PWM output TIM2 hooked up to IN4 of second DRV8847S
| 9|PA4 |DRV2 SLEEP ||output to sleep input of second DRV8847S
|10 |PA5 |DRV2 NFAULT| | input of nfault of second DRV8847S
|11 |PA6 |AIO3 || analog input or output not implemented yet
|12 |PA7 |IO4 | alt TX1| Discrete input or output not implemented yet
|13 |PB0 |PWM1 | alt TX4| PWM output not implemented yet
|15 |PB4 | PWR ENABLE|| Discrete output to enable power to the DRV8847s
|16 |PB1 | ADJUST ENABLE|| Discrete input to check power connection (USB <> terminals/barrel)
|17 |PB6 | DRV1 NFAULT|| input of nfault of first DRV8847S
|18 |PB7 | DRV1 SLEEP||output to sleep input of first DRV8847S
|19 |PB8 | NC||
|20 |PB9 | DRV1 IN1||PWM output TIM1 hooked up to IN1 of first DRV8847S
|21 |PB10 | DRV1 IN2||PWM output TIM1 hooked up to IN2 of first DRV8847S
|22 |PB11 | DRV1 IN4||PWM output TIM1 hooked up to IN4 of first DRV8847S
|23 |PB12 | DRV1 IN3||PWM output TIM1 hooked up to IN3 of first DRV8847S
|24 |PC19 | I2C SCL|| I2C clock signal connected to badge
|25 |PC18 | I2C SDA|| I2C data signal connected to badge
|26 |PC16 | USB-N|| USB negative
|27 |PC17 | USB-P|| USB positive
|28 |PC14 | PD CC1|| USB PD communication channel
| 29|GND | || Ground


## badge connector :
- 3V3
- GND
- I2C_SDA
- I2C_SCL
- I2S_DIN : interrupt line from IO expander