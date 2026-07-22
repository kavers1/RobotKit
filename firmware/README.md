# RobotKit Fri3d Camp 2026

## Design ideas
The robotkit badge extension is an interface board to drive motors and/or servos. The idea was to reuse the badge as controller for moving parts. This is accomplished by 2 motor drivers, a PWM driver to control servo's and an IO expander to have some inputs for digital signals. All this is connected over the I2C-bus of the badge. The initial idea was to control the motor drives from a CH32X035, the companion mcu also used on the badge. But since the DRV8847 had delivery issues we switched over to the I2C version of the motor drivers (cost impact). The remaining IO pins of the CH32X035 can be used for general IO. This part has not be implemented yet. This will need additional firmware on the CH32X035.

### used hardware :
    - CH32x035
    - DRV8847 --> DRV8847S
    - TCA9555
    - PCA9685

I2C communication to the different IC's is done by selecting the correct slave address. For some of them the address can be set by solder bridges, others are set in software, but the CH32X035 has a fixed address 0x5E and the DRV8847S also has a fixed startup address 0x60. The latter can and must be overwritten during initialization.


During the design it was clear that compatibility with LEGO is a plus. For that reason the PCB has provisions for 2 x 2 WEDO connectors, or 2 x 2 LEGO connectors or a terminal block wire up the motors.

    - WEDO connector 
    - LEGO connector hack
    - Terminal block

Since power supply is crutial but hard to predict what is needed, the PCB has multiple provisions. A 5V DC supply can be used on a terminal or barrel connection or on the USB connector (power pack), then the whole board is 5V DC on the power side. The CH32X035 communication can be on the 3V3 power of the badge. A level shifter is on the board. If the motors require more then the 5V, an USB-PD power supply can be hooked up to the USB connector. In that case an additional K7805 has to be added to the board, to provide the 5V-DC to IO expansion and the PWM. The voltage of the USB-PD can be set using the I2C communication. It shall be set at at least 7V DC and will not excide 18V DC
Keep always in mind the max current the power supply can deliver and the PCB can coop with.
Setting the power supply is done by solder bridges and adding component.

## Software Design

### Note :
this software is based on several other project where code was copied from, (check references) This resulted in 2 different coding styles to approach the CH32X035, one using macro function to set configuration registers and the other addressing the registers directly and do bit manipulations. Both are equally functional. But it is not the best idea for readability to use them both. Due to time limitation, the choice was made to keep them like they are and make the firmware functional before cleaning this up. My appologies for the mix.

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
|CH32X035 |0x5E| firmware defined|
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


##Test code

from machine import I2C, Pin
i2c_bus = I2C(0, sda=Pin(9), scl=Pin(18), freq=100000)
i2c_bus.scan()

returns : 
badge alone
[21,106] dec
[15,6A] hex 
15 = LCD
6A = Accelero
[21, 33, 64, 94, 106, 112] dec
[15, 21, 40, 5E, 6A, 70] hex

21 =
40 = PWM

5E = CH32X035
70