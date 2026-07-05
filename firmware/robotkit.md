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
|CH32X035| 1| selctd | selected PDO | byte |
| | 2| Active | active PDO | byte|
| | 3| MINV | Minimal voltage [mV]
| | 4| MAXV | Maximal voltage [mV]
| | 5| MAXI | Maximal current [mA]
| | 6| NRPDO | number of PDO's | byte |
| | 7| NRPPS | number of PPS's | byte |
| | 8| Volt | requested voltage [mV]
| | 9| Current |  requested max current [mA]
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

| FWD  | IN1 PWM | IN3 PWM |
|------|---------|---------|
|      | IN2 GND | IN4 GND |
| REV  | IN1 GND | IN3 GND |
|      | IN2 PWM | IN4 PWM |
|BRAKE | IN1 GND | IN3 GND |
|      | IN2 GND | IN4 GND |
|COAST | IN1 Z   | IN3 Z   |
|      | IN2 Z   | IN4 Z   |