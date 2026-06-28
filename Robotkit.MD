# RobotKit Fri3d Camp 2026

## Design ideas
    - WCH32x035
    - DRV8847 --> DRV8847S
    - TCA....
    - PCA....

    - WEDO
    - LEGO connector hack
    - Terminal block

## Software Design

### I2C slaves
    - WC32X035
    - DRV8847S 2x
    - TCA
    - PCA

- to allow multiple DRV8847S we have to set their address. This is done by pulling down the nFault line of the DRV8847S drivers
- write ... to the DRV8847S on address 0x60, the default address 
- pull 1 nFault line high 
- write the address register of the selected DRV8847S
- 

this selection of the nfault is done via a register in the CH32X035

#### CH32X035
##### PD functionality 
*registers*
- selected PDO [count]
- Active PDO [count]
- Minimal voltage of selected PDO [mV]
- Maximal voltage of selected PDO [mV]
- Maximal current of selected PDO [mA]
- number of PDO's [count]
- number of PPS's [count]
##### Motor control per DRV8845S 
- control 
    - forward 
    - reverse
    - coast
    - stop
- config
    - mode bits
    - sleep
- status
    - fault flag
- speed 

if we control the speed of the DRV over the IN1-4 signals, the speed will be controlled by a PWM on the IN1-4 signals 
Be aware that this control also has to be set in the DRV8847S

Speed control can be done by using this PWM. The max voltage to the drives can be controlled by the PD functionality.
If going above the 5V DC make sure that your hardware has the 5V regulator to provide the power supply to the CH32X035 and other IC's
Also the PWM driver has needs the 5V


| IC | NR |Register | description | format |
|----|----|---------|-------------|--------|
|CH32X035||```59h```|```default address set in firmware```|byte|
| | 0| STS | status | byte |
| | 1| NRPDO | number of PDO's | byte |
| | 2| NRPPS | number of PPS's | byte |
| | 3| selected | selected PDO | byte |
| | 4| Active | active PDO | byte|
| | 5| MINV | Minimal voltage of selected PDO [mV] |uint16_t
| | 6| MAXV | Maximal voltage of selected PDO [mV] |uint16_t
| | 7| MAXI | Maximal current of selected PDO [mA] |uint16_t
| | 8| Volt | requested voltage of active PPS [mV] |uint16_t
| | 9| Current |  requested max current of active PPS [mA] |uint16_t
| |  |
| |11| DRV1CONFIG | DRV1 configuration | byte |
| |12| DRV1STATUS | DRV1 status | byte |
| |13| DRV1CTRL | DRV1 control | byte |
| |14| DRV1SP | DRV1 speeds setpoint | word |
| |  |
| |16| DRV2CONFIG | DRV1 configuration | byte |
| |17| DRV2STATUS | DRV1 status | byte |
| |18| DRV2CTRL | DRV1 control | byte |
| |19| DRV2SP | DRV1 speeds setpoint | word |
||||||
||21|IOEXP| IO expander interrupt ||
||||||
|DRV8847|0| SLAVE_ADDR | Slave Address RSVD SLAVE_ADDR RW
||1| IC1_CON | IC1 Control 
||2| IC2_CON | IC2 Control 
||3| SLR_STATUS1 | Slew Rate and Fault Status-1 
||4| STATUS2 | Fault Status-2 
||||||
|PCA9685|1| | | byte |
||||||
|TCA9555|1| | | byte |

### I2C addresses
| slave | address| | comment |
|----|----|----|---|
|CH32X035 | 59h |default| set in firmware code |
|DRV8847S   | 60h| default | fixed address |
|DRV8847S-1 |   61h| proposal | configurable via DRV1CNFG register
|DRV8847s-2 |   62h| proposal | configurable via DRV2CNFG register
| ||
|PCA9685 | E0h | base address | settable using solder bridges |
|allcalladr|  E0h|default| 
|subadr1   |  E2h|default| moves with base address|
|subadr2   |  E4h|default| moves with base address|
|subadr3   |  E8h|default|moves with base address|
|||
|TCA9555 |    20h|base address| settable using solder bridges |

if drv8847s is setup as 2 pin interface and configured to use the physical ins IN1-4 then the 2 moters are controlled by those pins. I.e. the CH32X035 DRV1_IN1-4 and DRV2_IN1-4 are controlling the motors. 
The idea is to have PWM control on those pins.
TIM1 and TIM2 of the CH32X035 can be configured to drive those pins.
TIM1 is for DRV1 and TIM2 is for DRV2.
The output of those timers/counters can be directly tied to those pins. The table below gives an overview of how the pin connection should be. If a PIN is GND then it should be configured as an output pulling low. State Z means the pin should be configured as an input with no pull up or down.
see table Table 7-3. 4-Pin Interface (MODE = 0) in the datasheet

| DIR | DRV1 | DRV21
|------|---------|---------|
| FWD  | IN1 PWM | IN3 PWM |
|      | IN2 GND | IN4 GND |
| REV  | IN1 GND | IN3 GND |
|      | IN2 PWM | IN4 PWM |
|BRAKE | IN1 GND | IN3 GND |
|      | IN2 GND | IN4 GND |
|COAST | IN1 Z   | IN3 Z   |
|      | IN2 Z   | IN4 Z   |

the direction is controlled in the DRV-CTRL register of the CH32X035. The PWM is controlled via the SPEED register of the CH32X035.
Make sure that the DRV8847S configuration registers are set correctly to make this happen.



## References
[WCH turorial series](https://www.youtube.com/watch?v=vplR-9WMz4E&list=PLTQB7K9YJgoRY8hFDHP-QRVrtc-c8Fkd6&index=1)
[WCH tutorial: PWM](https://www.youtube.com/watch?v=vU7Jx7lHFGk&list=PLTQB7K9YJgoRY8hFDHP-QRVrtc-c8Fkd6&index=3)
