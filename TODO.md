- [ ] writing register will set the success flag
- [ ] PD controller list via selected
- [ ] PD active
- [ ] PD PPS
***
- [ ] DRV control
- [ ] DRV PWM
    - TIM2/TIM3 PRSC, ARR bepalen periode
     freq = 48Mhz/((PRSC+1)*(ARR+1))
    - Capture/compare register / ARR+1 -> duty cycle
    - 4 CCR's tied to IO  (First 3 have inverted output too)
      - TIMx : R32_TIMx_CH1CVR, R32_TIMx_CH2CVR, R32_TIMx_CH3CVR, R32_TIMx_CH4CVR
    - OISx in TIMx_CTRL2 to tie to the output channels

- [ ] FWD/REV/BRAKE
- [ ] COAST
      OE_MODE in TIMx_CTRL1 ???
- [ ] RAMP speed
***
- [ ] X035 FET PWM (TIM3)
- [ ] X035 IO
- [ ] AI
- [ ] DI