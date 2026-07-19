/*
 * Copyright (c) 2026 Koen Verstringe <koen.verstringe@gmail.com>
 *
 * Badge Robot Kit firmware for the Fri3d Badge 2026.
 *
 * This firmware runs on a WCH CH32X035 microcontroller and acts as a
 * peripheral to setup the robotkit expansion board for the Fri3d badge. 
 * It allows the configuration of motor voltage and driver settings of motor drivers
 *
 *   PD controller (readable via I2C):
 *     - PD number of fixed voltage sources
 *     - PD number of variable voltage sources
 *     - Minimal voltage of selected PDO
 *     - Maximal voltage of selected PDO
 *     - Maximal current of the selected PDO 
 *   PD controller (writeble via I2C):
 *     - Selected PDO, select the PDO for above readings
 *     - Active PDO, set the PDO to use
 *     - Voltage setpoint of a variable voltage source, for fixed voltage source this is equal to its voltage
 *     - Current setpoint of a variable voltage source, if fixed current settings this is equal to its max current
 *
 *   Motor drivers (readable via I2C)
 *     - hardware status of the of the drive and of the commands (NFAULT)
 *   Motor drivers (writeable via I2C)
 *     - drive hardware sleep signals (SLEEP)
 *     - drive control (FWD,REV,BRAKE,COAST)
 *     - drive speed setpoint (PWM signal)
 * 
 *   IO expander (readable via I2C)
 * 
 *   TODO
 *   TBD hardware signals to X035 IO
 *
 * Hardware:
 *   - I2C1 on PC18 (SDA) / PC19 (SCL) — these are also the SWD pins,
 *     so SWD is disabled once IIC_Init() is called.
 *   - ADC1 reads 6 channels continuously via DMA1_Channel1.
 *   - TIM3 fires at 100 Hz for button debounce scanning.
 *   - TIM1/TIM2 drive PWM outputs; brightness is updated via DMA on TIM_Update.
 */

//#include "system.h"
#include <ch32x035.h> /* both X033 and X035 */
#include <stdlib.h>   /* atoi() */
#include <string.h>   /* memset() */
#include <usbpd_sink.h> 
#include "PWM_tim2.h"

#include "debug.h"

#define USB_MONITOR_PORT        GPIOC // PC3: USB Monitor
/// TODO check
#define USB_MONITOR_PIN         GPIO_Pin_3
//#define USB_MONITOR_CHANNEL     ADC_Channel_13
//#define USB_MONITOR_RANK        (3)


#define DRV1_PORT               GPIOB       // PB
#define ADJ_ENABLE_PIN          GPIO_Pin_1  //PB1
#define PWR_ENABLE_PIN          GPIO_Pin_4  //PB4
#define DRV1_SLEEP_PIN          GPIO_Pin_7  //PB7
#define DRV1_NFAULT_PIN         GPIO_Pin_6  //PB6
#define DRV1_IN1_PIN            GPIO_Pin_9  //PB9
#define DRV1_IN2_PIN            GPIO_Pin_10 //PB10
#define DRV1_IN3_PIN            GPIO_Pin_12 //PB12
#define DRV1_IN4_PIN            GPIO_Pin_11 //PB11

#define DRV2_PORT               GPIOA       // PA
#define DRV2_SLEEP_PIN          GPIO_Pin_4  //PA4
#define DRV2_NFAULT_PIN         GPIO_Pin_5  //PA5
#define DRV2_IN1_PIN            GPIO_Pin_0  //PA0
#define DRV2_IN2_PIN            GPIO_Pin_1  //PA1
#define DRV2_IN3_PIN            GPIO_Pin_2  //PA2
#define DRV2_IN4_PIN            GPIO_Pin_3  //PA3
/// TODO
// define pins of local IO

/* I2C slave interface towards the ESP32S3 on Fri3d badge 2026 */
#define SDA_PORT                GPIOC
#define SDA_PIN                 GPIO_Pin_18
#define SCL_PORT                GPIOC
#define SCL_PIN                 GPIO_Pin_19
#define I2C_ADDRESS             (0x5E) /* 7-bit slave address; write=0x5E, read=0x5F */
#define I2C_TIMEOUT             (-2)
#define I2C_TIMEOUT_TICK        ((SystemCoreClock / 10) - 1) /* 100 ms timeout for I2C reads */
#define I2C_SPEED               (400000)

/*
 * I2C register map layout (also the memory layout of addon_data_t / raw_data[]):
 *
 *   Offset  Size  Description
 *   ------  ----  -------------------------------------------
 *   0x00     3    Firmware version [major, minor, patch]   (READ-ONLY)
 *   0x03     1    Powersupply status                       (READ-Write)
 *   0x04     1    PD controller number of PDO's            (READ-ONLY)
 *   0x05     1    PD controller number of PPS's            (READ-ONLY)
 *   0x06     2    PD controller minimal voltage            (READ-ONLY)
 *   0x08     2    PD controller maximal voltage            (READ-ONLY)
 *   0x0A     2    PD controller maximal current            (READ-ONLY)
 *   0x0C     1    PD controller selected PDO               (READ-WRITE)
 *   0x0D     1    PD controller active PDO                 (READ-WRITE)
 *   0x0E     2    PD controller voltage                    (READ-WRITE)
 *   0x10     2    PD controller current                    (READ-WRITE)
 *   0x12     1    Motor driver 1 status                    (READ-ONLY)
 *   0x13     1    Motor driver 1 config                    (READ-WRITE)
 *   0x14     1    Motor driver 1 control                   (READ-WRITE)
 *   0x15     1    Padding                                  (READ-ONLY)
 *   0x16     2    Motor driver 1 speed                     (READ-WRITE)
 *   0x18     2    padding                                  (READ-ONLY)
 *   0x20     1    Motor driver 2 status                    (READ-ONLY)
 *   0x21     1    Motor driver 2 config                    (READ-WRITE)
 *   0x22     1    Motor driver 2 control                   (READ-WRITE)
 *   0x23     1    padding                                  (READ-ONLY)
 *   0x24     2    Motor driver 2 speed                     (READ-WRITE)
 *   0x26     2    padding                                  (READ-ONLY)
 * 
 *
 * Total: RESULT_BUFFER_SIZE = 39 bytes.
 */
#define PWRSUPPLY          (2)  /* byte offeset to the power supply indictr*/
#define PD_OFFSET          (3)  /* byte offset to PD controller registers*/
#define PD_RW_OFFSET       (9)  /* offset of PD read write registers */
#define PD_SIZE            (15)
#define DRIVE1_OFFSET      (18) /* byte offset to Drive1 registers */
#define DRIVE_RW_OFFSET    (1)  /* offset of Drive1 read write registers*/
#define DRIVE_SIZE         (6)
#define DRIVE2_OFFSET      (32) /* byte offset to Drive2 registers*/
#define RESULT_BUFFER_SIZE (32) 
// writeable registers
#define PD_SELECT          ( PD_OFFSET + 8)
#define PD_ACTIVE          ( PD_OFFSET + 9)
#define PD_VOLTAGE         ( PD_OFFSET + 10)
#define PD_CURRENT         ( PD_OFFSET + 12)
#define DRV1_CONFIG        ( DRIVE1_OFFSET + 1)
#define DRV1_CONTROL       ( DRIVE1_OFFSET + 2)
#define DRV1_SPEED         ( DRIVE1_OFFSET + 4)
#define DRV2_CONFIG        ( DRIVE2_OFFSET + 1)
#define DRV2_CONTROL       ( DRIVE2_OFFSET + 2)
#define DRV2_SPEED         ( DRIVE2_OFFSET + 4)

typedef struct __attribute__((packed))
{
    uint8_t fixed : 1;
    uint8_t usb : 1;
    uint8_t padding : 4;
    uint8_t enable : 1 ;
    uint8_t reboot : 1 ;
}  PWR_status_t;

typedef struct __attribute__((packed))
{
    uint8_t select : 4;
    uint8_t padding :3;
    uint8_t enable :1;
} PDO_select_t ;

typedef struct __attribute__((packed))
{
    uint8_t fault :1;   // value of the fault pin when not in address mode
    uint8_t reserved :7;
} drive_status_t;

typedef struct __attribute__((packed))
{
    uint8_t set_address :1;   // set fault to write address
    uint8_t reserved :6;
    uint8_t address_mode :1; // set fault pin to output to configure the I2C address of the DRV
} drive_config_t;

typedef struct __attribute__((packed))
{
    uint8_t forward :1; 
    uint8_t reverse :1;
    uint8_t brake :1;
    uint8_t coast :1;
    uint8_t reserved :3;
    uint8_t sleep :1;
} drive_control_t;


/*
 * Packed I2C register map struct.
 * The union in addon_state_t lets I2C code treat this as a flat byte array.
 * __attribute__((packed)) prevents padding so sizeof() == RESULT_BUFFER_SIZE.
 *
 * IMPORTANT: adc_channels[] must be 4-byte aligned for DMA to work correctly;
 * the 'unused' padding byte after version[] achieves this.
 */
typedef struct __attribute__((packed))
{
    uint8_t update_power : 1;       /* set when DRV power enable or reboot is written via I2C */
    uint8_t update_pd_select : 1;   /* set when PD select written via I2C */
    uint8_t update_pd_active : 1;   /* set when PD actyive written via I2C */
    uint8_t update_pd_voltage : 1;  /* set when PD voltage written via I2C */
    uint8_t update_pd_current : 1;  /* set when PD current written via I2C */

    uint8_t update_drv1_config : 1; /* set when Drive1 config written via I2C */
    uint8_t update_drv1_control : 1;/* set when Drive1 control written via I2C */
    uint8_t update_drv1_speed : 1;  /* set when Drive1 speed written via I2C */
    
    uint8_t update_drv2_config : 1; /* set when Drive2 config written via I2C */
    uint8_t update_drv2_control : 1;/* set when Drive2 control written via I2C */
    uint8_t update_drv2_speed : 1;   /* set when Drive2 speed written via I2C */

    uint8_t reserved : 4;           /* reserved for future use */
    uint8_t slave_first_write:1;    
    
} update_flags_t;

 typedef struct __attribute__((packed))
{
    uint8_t  version[3];        /* firmware version [major, minor, patch] — READ-ONLY */
    PWR_status_t  powersupply;       /* power supply status. ie fixed 5V suppy or USB supply */             
    uint8_t  numberpdo;         /* number of fixed voltage power delivery */
    uint8_t  numberpps;         /* number of variable power delivey */
    uint16_t minvolt;           /* minimal voltage of selected power delivery */
    uint16_t maxvolt;           /* maximal voltage of selected power delivery */
    uint16_t maxcurrent;        /* maximal current of selected power delivery */
    uint8_t  selected;          /* selector of power delivery*/
    uint8_t  active;        /* the currently active power delivery, most significant bit enables power to DRV's */
    uint16_t voltage;           /* current voltage setpoint */
    uint16_t current;           /* current current setpoint */
    drive_status_t  drv1status; /* fault status of DRV1 */
    drive_config_t  drv1config; /* configuration of DRV1 */
    drive_control_t drv1control;/* control register of DRV1 */
    uint8_t  drv1padding;
    int16_t  drv1speed;         /* speed setpoint of DRV1 */
    uint16_t drv1padding2;
    drive_status_t  drv2status; /* fault status of DRV2 */
    drive_config_t  drv2config; /* configuration of DRV2 */
    drive_control_t drv2control;/* control register of DRV2 */
    uint8_t  drv2padding;
    int16_t  drv2speed;         /* speed setpoint of DRV2*/
//    uint16_t drv2padding2;
//    uint16_t drv2padding3;
} robotkit_data_t;



/* Compile-time check: struct layout must match RESULT_BUFFER_SIZE exactly */
_Static_assert(sizeof(robotkit_data_t) == RESULT_BUFFER_SIZE, "raw data and struct size are not aligned!");

/*
 * Global firmware state.
 * Flags are set by interrupt handlers and consumed by the main loop.
 */

///TODO update flags bekijken
typedef struct
{
    union {
        update_flags_t flags;
        uint16_t any_update;
    };
    
    uint8_t slave_offset;                  /* register offset captured after the most recent ADDR+W. */
    uint8_t slave_position;                /* current read/write cursor, reset to offset on every ADDR (including repeated-START), so write-then-read works without special-casing. */
    
    union
    {
        robotkit_data_t data;                    /* structured access to the register map */
        uint8_t raw_data[RESULT_BUFFER_SIZE]; /* flat byte access for I2C transfers */
    };
} robotkit_state_t;

/* global state variable */
static robotkit_state_t state;
static uint8_t lstActive;

static void init_Power(void)
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );
    

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    /* output : power enable to DRV's */
    GPIO_InitStructure.GPIO_Pin = PWR_ENABLE_PIN;// PWR_ENABLE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* input : do we have fixed power input or USB adjustable */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 ;// ADJ_ENABLE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure); // GPIOB
#ifdef DEBUG
    PRINT("init_Power\r\n");
#endif

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}

static void init_Pd(void)
{
 // initialization done in PD_Connect
    uint8_t rslt = PD_connect();
#ifdef DEBUG
    PRINT("init_Pd %d\r\n", rslt);
#endif
    ;
}

static void init_Drive(void)
{
//TODO check next lines

 /* Enable AFIO, GPIO A, B and C clock */
 /* Enable TIM1, TIM2 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC| RCC_APB2Periph_TIM1, ENABLE );
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.GPIO_Pin = DRV1_SLEEP_PIN | DRV1_IN4_PIN | DRV1_IN3_PIN | DRV1_IN2_PIN | DRV1_IN1_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);              // GPIOB

    //DRV1_NFAULT_PIN moet input zijn at startup
    GPIO_InitStructure.GPIO_Pin = DRV1_NFAULT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DRV1_PORT, &GPIO_InitStructure);          // GPIOB

/// TODO init pwm drv1
    TIM1_pwm_init();
    
    GPIO_InitStructure.GPIO_Pin = DRV2_SLEEP_PIN | DRV2_IN4_PIN | DRV2_IN3_PIN | DRV2_IN2_PIN | DRV2_IN1_PIN ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);              // GPIOA
    
    //DRV2_NFAULT_PIN moet input zijn at startup
    GPIO_InitStructure.GPIO_Pin = DRV2_NFAULT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DRV2_PORT, &GPIO_InitStructure);          // GPIOA

/// TODO init pwm drv2
    TIM2_pwm_init();
#ifdef DEBUG
    PRINT("drive_Init()\r\n");
#endif

}


static void IIC_Init(uint32_t bound, uint16_t address)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStruct = {0};

    /* enable I2C1 and GPIOC clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    /* remap PC18/PC19 to I2C1 SDA/SCL */
    GPIO_PinRemapConfig(GPIO_PartialRemap3_I2C1, ENABLE); // 011: Mapping (SCL/PC19, SDA/PC18)

    /* Disable DIO (SWD) interface on these pins */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE);

    /* configure the GPIO as SDA/SCL pins */
    GPIO_InitStructure.GPIO_Pin = SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // automatic open-drain
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SDA_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SCL_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; // automatic open-drain
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCL_PORT, &GPIO_InitStructure);

    /* configure I2C1 */
    I2C_InitStructure.I2C_ClockSpeed = bound;                                 // bus speed
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;                                // there is only 1 mode
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_16_9;                     // I2C fast mode Tlow/Thigh = 16/9
    I2C_InitStructure.I2C_OwnAddress1 = address << 1;                         // 7 or 10 bit address
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;                               // automatic acknowledge
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // use 7 bit address
    I2C_Init(I2C1, &I2C_InitStructure);

    /* configure I2C interrupts */
    NVIC_InitStruct.NVIC_IRQChannel = I2C1_EV_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = I2C1_ER_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    /* Enable I2C event, error, and buffer interrupts.
     * EVT fires on: address match, byte received, byte transmitted, stop detected.
     * ERR fires on: bus error, arbitration lost, acknowledge failure, etc.
     * BUF fires on: TXE/RXNE (needed so we get an interrupt for each data byte).
     */
    I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, ENABLE);

    /* enable clock stretching */
    I2C_StretchClockCmd(I2C1, ENABLE);

    /* enable I2C1 */
    I2C_Cmd(I2C1, ENABLE);
#ifdef DEBUG
    PRINT("I2C_Init()\r\n");
#endif

}

/*
 * Trigger a reboot into the WCH USB bootloader.
 * SystemReset_StartMode(Start_Mode_BOOT) sets a flag that tells the bootrom
 * to stay in bootloader mode after the reset, allowing firmware reflashing.
 */
static void reset_to_bootloader(void)
{
    SystemReset_StartMode(Start_Mode_BOOT);
    NVIC_SystemReset();
}

static int i2c_pos_is_writable(uint8_t pos)
{

    /// TODO power enable
    if (pos == PWRSUPPLY) return 1;
    if (pos >= PD_OFFSET + PD_RW_OFFSET && 
        pos  < PD_OFFSET + PD_SIZE) return 1;
    if (pos >= DRIVE1_OFFSET + DRIVE_RW_OFFSET && 
        pos  < DRIVE1_OFFSET + DRIVE_SIZE) return 1;
    if (pos >= DRIVE2_OFFSET + DRIVE_RW_OFFSET && 
        pos  < DRIVE2_OFFSET + DRIVE_SIZE) return 1;
     
    return 0;
}

/**
 * Handle one I2C event interrupt for the slave register interface.
 *
 * Protocol (write transaction):
 *   START | ADDR+W | reg_offset | [data bytes...] | STOP
 *
 * Protocol (read transaction — master sets register first, then re-reads):
 *   START | ADDR+W | reg_offset | START | ADDR+R | [data bytes from reg_offset onwards...] | STOP
 *
 * The first byte of every write sets state.slave_position (the register address).
 * Subsequent bytes are interpreted based on that address:
 *   - TODO specify writable register
 *   - 
 *   - 
 *   - any other addr  → write is rejected; remaining RXNE bytes are drained and discarded
 *
 * On TXE (master is reading): bytes are sent sequentially from raw_data[] starting at
 * state.slave_position, advancing the pointer with each byte sent.
 *
 * Reading STAR2 clears the ADDR flag as a hardware side-effect.
 * This releases the clock when clock stretching is enabled
 */
static void i2c_slave_process(void)
{
    uint32_t flag1 = 0, flag2 = 0;
    flag1 = I2C1->STAR1;

    if (flag1 & I2C_STAR1_ADDR)
    {
        state.slave_position = state.slave_offset;
        state.flags.slave_first_write = 1;
    }

    /* Data register not empty (Receiver) flag */
    if (flag1 & I2C_STAR1_RXNE)
    {
        uint8_t byte = I2C_ReceiveData(I2C1);
        /* the first byte received after address+W is the register offset.
         * Subsequent payload bytes are read and written if the memory area
         * is writeable.
         */
        if (state.flags.slave_first_write)
        {
            state.slave_offset = byte;
            state.slave_position = byte;
            state.flags.slave_first_write = 0;
            PRINT("I2C reg: 0x%02x\r\n", byte);
        }
        else
        {
            if (i2c_pos_is_writable(state.slave_position))
            {
                state.raw_data[state.slave_position] = byte;
                // flag any changes 
                switch (state.slave_position) {
                case PWRSUPPLY :
                    state.flags.update_power = 1;
                case PD_SELECT :
                    state.flags.update_pd_select = 1;
                    break;
                case PD_ACTIVE :
                    state.flags.update_pd_active = 1;
                    break;
                case PD_VOLTAGE + 1 : // last byte of voltage written
                    state.flags.update_pd_voltage =1;
                    break;
                case PD_CURRENT + 1 : // last byte of current written
                    state.flags.update_pd_current =1;
                    break;
                case DRV1_CONFIG :
                    state.flags.update_drv1_config = 1;
                    break;
                case DRV1_CONTROL :
                    state.flags.update_drv1_control = 1;
                    break;
                case DRV1_SPEED :
                    state.flags.update_drv1_speed = 1;
                    break;
                case DRV2_CONFIG :
                    state.flags.update_drv2_config = 1;
                    break;
                case DRV2_CONTROL :
                    state.flags.update_drv2_control = 1;
                    break;
                case DRV2_SPEED :
                    state.flags.update_drv2_speed = 1;
                    break;
                default:
                    break;
                }

            }
            state.slave_position++;
        }
    }

    /* Process transmitting data (master is reading from us).
     * Send one byte from raw_data[] at the current pointer position and advance
     * the pointer so consecutive TXE interrupts walk through the register file.
     * If slave_position is out of range, send 0x00 as a safe dummy byte.
     */
    if (flag1 & I2C_STAR1_TXE)
    {
        if (state.slave_position < RESULT_BUFFER_SIZE)
        {
            I2C_SendData(I2C1, state.raw_data[state.slave_position++]);
        }
        else
        {
            /* send dummy data */
            I2C_SendData(I2C1, 0x00);
        }
    }

    /* Detect STOP condition — the master has finished its write transaction.
     * Clear the stop flag so the peripheral is ready for the next transaction.
     */
    if (flag1 & I2C_STAR1_STOPF)
    {
        PRINT("I2C STOP\r\n");
        /* writing CTLR1 after reading STAR1 clears STOPF */
        I2C1->CTLR1 &= ~(I2C_CTLR1_STOP);
    }

    // clock stretching: release the clock
    flag2 = I2C1->STAR2;
    (void)flag2;
}

static void run_Drive1(void){
    // config pin DRV2_SLEEP_PIN  DRV2_NFAULT_PIN
/*    
state.data.drv1status
{
    uint8_t fault :1;   // value of the fault pin when not in address mode
    uint8_t reserved :7;
}

state.data.drv1config
{
    uint8_t set_address :1;   // set fault to write address
    uint8_t reserved :6;
    uint8_t address_mode :1; // set fault pin to output to configure the I2C address of the DRV
} 

state.data.drv1control
{
    uint8_t forward :1; 
    uint8_t reverse :1;
    uint8_t brake :1;
    uint8_t coast :1;
    uint8_t reserved :3;
    uint8_t sleep :1;
}
*/
    if (state.flags.update_drv1_config == 1){
        /// TODO change config
        /// set mode and sleep signals
        /// reset speed and set control to coast
         GPIO_InitTypeDef GPIO_InitStructure = {0};

        if (state.data.drv1control.sleep == 1)
            GPIO_WriteBit(GPIOB, DRV1_SLEEP_PIN , Bit_SET);
        else
            GPIO_WriteBit(GPIOB, DRV1_SLEEP_PIN , Bit_RESET);
        // set fault bit mode to input or output to write I2C address of DRV8847S
        if (state.data.drv1config.set_address == 1){
        /// TODO check init
            GPIO_InitStructure.GPIO_Pin = DRV1_NFAULT_PIN;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        else {
            GPIO_InitStructure.GPIO_Pin = DRV1_NFAULT_PIN;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }

        if (state.data.drv1status.fault == 1)
            GPIO_WriteBit(GPIOB, DRV1_NFAULT_PIN , Bit_SET);
        else
            GPIO_WriteBit(GPIOB, DRV1_NFAULT_PIN , Bit_RESET);
        

        state.flags.update_drv1_config = 0;
    }
    if (state.flags.update_drv1_control == 1){
        /// TODO change control
        /// if going from fwd to rev go over break for xxx ms and reset speed
        /// going from brake or coast to FWD or REV is ok 
        /// clear change flag if not waiting
        if (state.data.drv1control.brake == 1) { // brake
            state.data.drv1speed = 0;
            Brake_TIM1(1);
        }
        else {
            if (state.data.drv1control.coast == 1) { // coast
                state.data.drv1speed = 0;
            }
            setSpeed_TIM1(state.data.drv1speed,1);
        }
        state.flags.update_drv1_control = 0;
    }
    if (state.flags.update_drv1_speed == 1){
        /// TODO adjust speed ie adjust PWM CCR register
        /// if sign change go from REV <--> FWD
        /// if just speed adjustment clear change flag
        setSpeed_TIM1(state.data.drv1speed,1);

        state.flags.update_drv1_speed = 0;
    }
    /// TODO
    /// read fault bit
    uint32_t b = GPIO_ReadInputData(GPIOB);
    if( b  & DRV1_NFAULT_PIN) {
        state.data.drv1status.fault = 1;
    }
    else {
        state.data.drv1status.fault = 0;
    }

}

static void run_Drive2(void){
    if (state.flags.update_drv2_config == 1){
        /// TODO change config
        /// set mode and sleep signals
        /// reset speed and set control to coast
        state.flags.update_drv2_config = 0;
    }
    if (state.flags.update_drv2_control == 1){
        /// TODO change control
        /// if going from fwd to rev go over break for xxx ms and reset speed
        /// going from brake or coast to FWD or REV is ok 
        /// clear change flag if not waiting
        state.flags.update_drv2_control = 0;
    }
    if (state.flags.update_drv2_speed == 1){
        /// TODO adjust speed ie adjust PWM CCR register
        /// if sign change go from REV <--> FWD
        /// if just speed adjustment clear change flag
    }    
}

static void run_PD(void){
/// TODO check if we are connected else return
    PRINT("run_PD\r\n");
    if (state.data.powersupply.fixed) return; // if no PD hardware selected return
    
    if (state.flags.update_pd_select == 1){
        // limit the value range of select and active
        if (state.data.selected > PD_getPDONum() || state.data.selected <= 0) state.data.selected = 1 ;     
        state.data.maxvolt = PD_getPDOMaxVoltage((uint8_t)state.data.selected) ;
        state.data.minvolt = PD_getPDOMinVoltage((uint8_t)state.data.selected) ;
        state.data.maxcurrent = PD_getPDOMaxCurrent((uint8_t)state.data.selected);
        /// TODO  update capabilties and set registers
        //printSourceCap();
        state.flags.update_pd_select = 0;
    }
    if (state.flags.update_pd_active == 1){
        // limit the value range    of select and active
        if (state.data.active> PD_getPDONum() || state.data.active <= 0) state.data.active = lstActive;
        if(state.data.active <= PD_getFixedNum()) {
            if( PD_setPDO((uint8_t)(state.data.active), PD_getPDOVoltage((uint8_t)(state.data.active)))== 0 ){

                PD_setPDO((uint8_t)(lstActive), PD_getPDOVoltage((uint8_t)(lstActive))); // if not succesfull revert to last PDO    
                state.data.active = lstActive ;
            }
        }
        else { // it is a PPS set voltage
            PD_negotiate();
            if(!PD_setPDO(state.data.active, state.data.voltage)) {
                PD_setPDO((uint8_t)(lstActive), state.data.voltage); // if not succesfull revert to last PDO    
                state.data.active = lstActive; // if not succesfull revert to last PDO
            }
        }

        state.flags.update_pd_active = 0;
        state.data.voltage = PD_getVoltage();
    }
    if (state.flags.update_pd_voltage == 1){

        PD_setVoltage( state.data.voltage);
        state.flags.update_pd_voltage = 0;
    }
    PD_negotiate();
/// TODO do we have to do this all the time ? or just at changes and initialize can we combine the 2 states ?
    state.data.voltage    = PD_getPDOVoltage(state.data.active);
    state.data.minvolt = PD_getPDOMinVoltage(state.data.selected);
    state.data.maxvolt = PD_getPDOMaxVoltage(state.data.selected);
    state.data.maxcurrent = PD_getPDOMaxCurrent(state.data.selected);
    PRINT("PD: sel %d act %d volt %d min %d max %d cur %d\r\n", state.data.selected, state.data.active, state.data.voltage, state.data.minvolt, state.data.maxvolt, state.data.maxcurrent); 
}

static void run_Power()
{
    if (state.flags.update_power == 1){
        // limit the value range of select and active
        if (state.data.powersupply.reboot) reset_to_bootloader();
        // if not reboot then set DRV power enable
        GPIO_WriteBit(GPIOB, PWR_ENABLE_PIN, state.data.powersupply.enable ? Bit_SET : Bit_RESET);
        state.flags.update_pd_select = 0;  // reset update flag
    }
 
    uint32_t b = GPIO_ReadInputData(GPIOB);  // read input of ADJ_ENABLE_PIN
    if(( b  & ADJ_ENABLE_PIN) == (uint32_t)Bit_RESET) {
        state.data.powersupply.fixed = 0;
        state.data.powersupply.usb = 1;
    }
    else{
        state.data.powersupply.fixed = 1;
        state.data.powersupply.usb = 0;
    }


}

/* main */
int main(void)
{
    /* Zero-initialise all state data and flags before touching any hardware */
    memset(&state, 0, sizeof(robotkit_state_t));

    /* Embed the firmware version (injected by the build system as preprocessor
     * strings) into the I2C register map so the ESP32 can read it back.
     */


    char version_major[] = VERSION_MAJOR;
    char version_minor[] = VERSION_MINOR;
    char version_patch[] = VERSION_PATCH;
    state.data.version[0] = atoi(version_major) & 0xff;
    state.data.version[1] = atoi(version_minor) & 0xff;
    state.data.version[2] = atoi(version_patch) & 0xff;

    SystemInit();
#ifdef NVIC_PriorityGroup_2
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
#else
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
#endif
    SystemCoreClockUpdate();
    Delay_Init();

    // TODO do we have to remap the serial ????

#if (DEBUG)
    
    USART_Printf_Init(115200);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);
    
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //PC0 = USART2_TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;// alternate function push-pull
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    Delay_Ms(100);

    GPIO_PinRemapConfig( GPIO_PartialRemap3_USART2, ENABLE);// remap USART2 to PC0/PC1
#endif
    
    /* 1-second boot delay so a USB serial monitor can attach and SWD can
     * connect before any peripheral or I2C activity begins.
     */
    Delay_Ms(1000);


    PRINT("\r\nSystemClk: %u\r\n", (unsigned)SystemCoreClock);
    PRINT("ChipID: %08x\r\n", (unsigned)DBGMCU_GetCHIPID());

    /* init power */
    init_Power();
    /* init PD */
    init_Pd();  // setup PD hardware and negotiate initial voltage/current

    PD_reset(); // reset PD state machine to start negotiation
    PRINT("PDONUM: %d\r\n", PD_getPDONum());
    PRINT("PPSNUM: %d\r\n", PD_getPPSNum());
    PRINT("PDOFIX: %d\r\n", PD_getFixedNum( ));
    PRINT("PDVOLT: %d\r\n", PD_getPDOMaxVoltage(1));
    PRINT("PDVOLT: %d\r\n", PD_getPDOMinVoltage(1));
    PRINT("PDCURRENT: %d\r\n", PD_getPDOMaxCurrent(1));
    PRINT("PD CHECK: %d\r\n", PD_checkCC());
    
    /* init drive */
    // init_Drive();
    
    /* configure the I2C pins and interrupts */
    //IIC_Init(I2C_SPEED, I2C_ADDRESS); // maps SWD lines to I2C
    /* init drives */
    //drive_Init();
    /* init local IO */

    PRINT("Robotkit Init done\r\n");

    /* Main event loop — all heavy lifting is done in ISRs and DMA;
     * the loop only reacts to flags set by those background mechanisms.
     */
    while (1)
    {
        /// TODO run logic 
        /*
        since all changes to registers are triggered by receiving I2C register 
        it is all driven by the I2C interrupts and actions.

        check state flags and execute the appropriate action

        keep PD communication alive.
        */
        run_Power();
        run_PD();
/*        run_Drive1();
        run_Drive2();*/
        
        Delay_Ms(1000);
        PD_negotiate();  
        /* I2C master wrote a new value to the outputs register: apply it now.
         * Also handles the reboot-to-bootloader command if that bit is set.
         */
        /*if (state.flags.update_outputs)
        {
            state.flags.update_outputs = 0;
            GPIO_WriteBit(AUX_POWER_PORT, AUX_POWER_PIN, state.data.aux_power ? Bit_SET : Bit_RESET);
            GPIO_WriteBit(LCD_RESET_PORT, LCD_RESET_PIN, state.data.lcd_reset ? Bit_SET : Bit_RESET);
            GPIO_WriteBit(LORA_RESET_PORT, LORA_RESET_PIN, state.data.lora_reset ? Bit_SET : Bit_RESET);

            if (state.data.reboot)
            {
                PRINT("Reboot to bootloader trigger\r\n");
                Delay_Ms(100);
                reset_to_bootloader();
            }
            if (state.data.remap)
            {
                PRINT("Remap SWD trigger\r\n");
                Delay_Ms(100);
                /* disable I2C interrupts */
        /*        I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);

                /* disable I2C1 */
        /*        I2C_Cmd(I2C1, DISABLE);

                /* Re-enable DIO (SWD) interface on these pins */
        /*        GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);
            }
        }*/
    }
}

/* -------------------------------------------------------------------------
 * Interrupt handlers
 * ------------------------------------------------------------------------- */

/* TIM3 Update — fires every 10 ms (configured by Button_Init).
 * Drives the two-sample debounce state machine in Button_Scan().
 */
void TIM3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
    {
        /* Handle button scan */
//        Button_Scan();
    }
    /* Clear interrupt flag */
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

/* Non-Maskable Interrupt — logged for debugging; no recovery attempted. */
void NMI_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void NMI_Handler(void)
{
    PRINT("NMI_Handler\r\n");
}

/* Hard Fault — logs the fault and spins forever (safe stop state). */
void HardFault_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void HardFault_Handler(void)
{
    PRINT("HARDFAULT\r\n");
    while (1)
    {
    }
}

/* Generic I2C1 IRQ (not used; I2C events are routed to I2C1_EV/ER below). */
void I2C1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void I2C1_IRQHandler(void)
{
    PRINT("I2C1_IRQHandler\r\n");
}

/* I2C1 Event — fires on address match, RXNE, TXE, STOP, BTF, etc.
 * All register-map I/O is handled inside i2c_slave_process().
 * Uses plain (interrupt) attribute (full register save) because
 * i2c_slave_process() calls several library functions.
 */
void I2C1_EV_IRQHandler(void) __attribute__((interrupt));
void I2C1_EV_IRQHandler(void)
{
    // see: https://github.com/cnlohr/ch32fun/blob/master/examples_x035/i2c_slave_test/i2c_slave_test.c
    // see: https://github.com/Community-PIO-CH32V/ch32v003fun/blob/master/examples/i2c_slave/i2c_slave.h
    // see: https://github.com/maxint-rd/arduino_core_ch32/blob/main/libraries/Wire/src/utility/twi.c
    i2c_slave_process();
}

/* I2C1 Error — clear whichever error flag fired so the bus is not blocked. */
void I2C1_ER_IRQHandler(void) __attribute__((interrupt));
void I2C1_ER_IRQHandler(void)
{
    uint16_t STAR1 = I2C1->STAR1;
    if (STAR1 & I2C_STAR1_BERR) I2C1->STAR1 &= ~I2C_STAR1_BERR;
    if (STAR1 & I2C_STAR1_ARLO) I2C1->STAR1 &= ~I2C_STAR1_ARLO;
    if (STAR1 & I2C_STAR1_AF) I2C1->STAR1 &= ~I2C_STAR1_AF;
}