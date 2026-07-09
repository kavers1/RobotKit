# RobotKit Fri3d Camp 2026

## Design ideas
    - WCH32x035
    - DRV8847 --> DRV8847S
    - TCA9555
    - PCA9685

    - WEDO connector 
    - LEGO connector hack
    - Terminal block

## Software Design

### I2C slaves
    - WC32X035
    - DRV8847S 2x
    - TCA9555
    - PCA9685

- to allow multiple DRV8847S we have to set their address. This is done by pulling down the nFault line of the DRV8847S drivers
- write ... to the DRV8847S on address 0x60, the default address 
- pull 1 nFault line high 
- write the address register of the selected DRV8847S
- 

this selection of the nfault is done via a register in the CH32X035

#### CH32X035
##### PD functionality 
*registers*
- Selected PDO [count]
- Active PDO [count]
- Voltage [mV]
- Minimal voltage [mV]
- Maximal voltage [mV]
- Maximal current [mA]
- Number of PDO's [count]
- Number of PPS's [count]
- Reset PD ???
- PD Status [status bits] 
    -[0] select error
    -[1] active error
    -[2] voltage error
    -[7] PD connection error
in case of an faulty operation the error bit is set and no action is taken
switchin from a fixed PDO to a PPS voltage is copied from PDO, then the voltage can be set.
##### Motor control per DRV8845S 
- control 
    - forward
    - reverse
    - coast
    - stop
- config
    - nfault input / output
    - nfault value when in output (write I2C address)
    - sleep
- status
    - fault flag
- speed
    - speed > 0 is forward
    - speed = 0 is coast
    - speed < 0 is reverse

if we control the speed of the DRV over the IN1-4 signals, the speed will be controlled by a PWM on the IN1-4 signals 
Be aware that this control also has to be set in the DRV8847S mode registers

Speed control can be done by using this PWM. The max voltage to the drives can be controlled by the PD functionality.
If going above the 5V DC make sure that your hardware has the 5V regulator to provide the power supply to the CH32X035 and other IC's
Also the PWM driver has needs the 5V


| IC | NR |Register | description | format |
|----|----|---------|-------------|--------|
|CH32X035| 1| selctd | selected PDO | byte |
| | 2| Active | active PDO | byte|
| | 3| MINV | Minimal voltage [mV]
| | 4| MAXV | Maximal voltage [mV]
| | 5| MAXI | Maximal current [mA]
| | 6| NRPDO | number of PDO's | byte |
| | 7| NRPPS | number of PPS's | byte |
| | 8| Volt | requested voltage [mV]
| | 9| Reset |  reset PD communication
| |10| Status | PD status bits 
| |  |        | [0] select error
| |  |        | [1] active error
| |  |        | [2] voltage error
| |  |        | [7] Comm error
|
| |11| DRV1CONFIG | DRV1 configuration | byte |
| |  |            |    [0] mode bits
| |  |            |    [1] sleep
| |12| DRV1STATUS | DRV1 status | byte |
| |  |            | [0] fault bit
| |13| DRV1CTRL | DRV1 control | byte |
| |  |          | [0] forward
| |  |          | [1] reverse
| |  |          | [2] coast
| |  |          | [3] brake
| |14| DRV1SP | DRV1 speeds setpoint | word |
| |  |
| |16| DRV2CONFIG | DRV1 configuration | byte |
| |17| DRV2STATUS | DRV1 status | byte |
| |18| DRV2CTRL | DRV1 control | byte |
| |19| DRV2SP | DRV1 speeds setpoint | word |
||||||
||21|IOEXP| IO expander interrupt ||
||||[0] interrupt status ||
|DRV8847|0| SLAVE_ADDR | Slave Address RSVD SLAVE_ADDR RW
||1| IC1_CON | IC1 Control 
||2| IC2_CON | IC2 Control 
||3| SLR_STATUS1 | Slew Rate and Fault Status-1 
||4| STATUS2 | Fault Status-2 
||||||
|PCA9685|1| | | byte |
||||||
|TCA|1| | | byte |

### I2C addresses
|CH32X035 | ||
|----|----|----|
|DRV8847S   | 0x60h| default 
|DRV8847S-1 |   0x61h| proposal
|DRV8847s-2 |   0x62h| proposal
|PCA9685 ||
|allcalladr|  0xE0h|default|
|subadr1   |  0xE2h|default|
|subadr2   |  0xE4h|default|
|subadr3   |  0xE8h|default|
|||
|TCA9555 |    0x20h|default|

|DIRECTION|DRV1| DRV2|
|------|---------|---------|
| FWD  | IN1 PWM | IN3 PWM |
|      | IN2 GND | IN4 GND |
| REV  | IN1 GND | IN3 GND |
|      | IN2 PWM | IN4 PWM |
|BRAKE | IN1 VCC | IN3 VCC |
|      | IN2 VCC | IN4 VCC |
|COAST | IN1 GND   | IN3 GND   |
|      | IN2 GND   | IN4 GND   |

DRV1  TIM1 PWM OUT1 en OUIT2

PWM config if counter is less than ccr then out is 1
if CCR > ARR then out is always 0

|DIRECTION|DRV1| DRV2|
|------|---------|---------|
| FWD  | TIM1 CCR1 = speed | TIM1 CCR3 = speed |
|      | TIM1 CCR2 = ATRLR + 1 | TIM1 CCR4 = ATRLR + 1 |
| REV  | TIM1 CCR1 = ATRLR + 1| TIM1 CCR3 = ATRLR + 1 |
|      | TIM1 CCR2 = speed | TIM1 CCR4 = speed |
|BRAKE | TIM1 CCR1 = 0 | TIM1 CCR3 = 0 |
|      | TIM1 CCR2 = 0 | TIM1 CCR4 = 0 |
|COAST | TIM1 CCR1 = ATRLR + 1   | TIM1 CCR3 = ATRLR + 1   |
|      | TIM1 CCR2 = ARATRLRR + 1   | TIM1 CCR4 = ATRLR + 1   |

similar for DRV3/DRV4 but with TIM2
