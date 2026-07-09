/*
 * Copyright (c) 2026 Bert Outtier <outtierbert@gmail.com> Koen Verstringe 
 *
 * Badge Robot Kit firmware for the Fri3d Badge 2026.
 *
 * This firmware runs on a WCH CH32X035 microcontroller and acts as a
 * peripheral to setup the robotkit expansion board for the Fri3d badge. It allows the configuration of
 * motor voltage and driver settings of motor drivers
 *
 *   PD controller (readable via I2C):
 *     - PD number of fixed voltage sources
 *     - PD number of varialbe voltage sources
 *     - Minimal voltage of selected PDO
 *     - Maximal voltage of selected PDO
 *     - Maximal current of the selected PDO 
 *   PD controller (writeble via I2C):
 *     - Selected PDO, select the PDO for above readings
 *     - Active PDO, set the PDO
 *     - Voltage setpoint of a variable voltage source, for fixed voltage source this is equal to its voltage
 *     - Current setpoint of a variable voltage source, if fixed current settings this is equal to its max current
 *
 *   Motor drivers (readable via I2C)
 *     - hardware status of the of the drive and of the commands
 *   Motor drivers (writeable via I2C)
 *     - drive hardware configuration signals (MODE,SLEEP)
 *     - drive control (FWD,REV,BRAKE,COAST)
 *     - drive speed setpoint (PWM signal)
 * 
 *   IO exopander (readable via I2C)
 *     - IOexpander interrupt bit
 * 
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

#include <ch32x035.h> /* both X033 and X035 */
#include <stdlib.h>   /* atoi() */
#include <string.h>   /* memset() */

#include "debug.h"

#define USB_MONITOR_PORT        GPIOC // PC3: USB Monitor
#define USB_MONITOR_PIN         GPIO_Pin_3
#define USB_MONITOR_CHANNEL     ADC_Channel_13
#define USB_MONITOR_RANK        (3)


#define DRV1_PORT               GPIOB       // PB
#define DRV1_SLEEP_PIN          GPIO_Pin_7  //PB7
#define DRV1_FAULT_PIN          GPIO_Pin_6  //PB6
#define DRV1_IN1_PIN            GPIO_Pin_9  //PB9
#define DRV1_IN2_PIN            GPIO_Pin_10 //PB10
#define DRV1_IN3_PIN            GPIO_Pin_12 //PB12
#define DRV1_IN4_PIN            GPIO_Pin_11 //PB11

#define DRV2_PORT               GPIOA       // PA
#define DRV2_SLEEP_PIN          GPIO_Pin_4  //PA4
#define DRV2_FAULT_PIN          GPIO_Pin_5  //PA5
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

/* TIM3 auto-reload value for a 100 Hz interrupt used for button debounce timing */
#define TIMER_FREQ ((SystemCoreClock / 10000) - 1)

/*
 * I2C register map layout (also the memory layout of addon_data_t / raw_data[]):
 *
 *   Offset  Size  Description
 *   ------  ----  -------------------------------------------
 *   0x00     3    Firmware version [major, minor, patch]   (READ-ONLY)
 *   0x03     1    Padding (required for 4-byte alignment of ADC buffer)
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

#define PD_OFFSET          (3)  /* byte offset to PD controller registers*/
#define PD_RW_OFFSET       (9)  /* offset of PD read write registers */
#define PD_SIZE            (15)
#define DRIVE1_OFFSET      (18) /* byte offset to Drive1 registers */
#define DRIVE_RW_OFFSET    (1)  /* offset of Drive1 read write registers*/
#define DRIVE_SIZE         (6)
#define DRIVE2_OFFSET      (32) /* byte offset to Drive2 registers*/
#define RESULT_BUFFER_SIZE (39) /* byte offset of lcd_brightness */
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
#define DRV_SPEED          ( DRIVE1_OFFSET + 4)

typedef struct{
    uint8_t fault :1;   // value of the fault pin when not in address mode
    uint8_t reserved :7;
} drive_status_t;

typedef struct{
    uint8_t mode :1;    // mode pin of the DRV
    uint8_t sleep :1;   // sleep pin of the DRV
    uint8_t reserved :4;
    uint8_t fault :1;   // set fault to write address
    uint8_t address :1; // set fault pin to output to configure the I2C address of the DRV
} drive_config_t;

typedef struct{
    uint8_t forward :1; 
    uint8_t reverse :1;
    uint8_t brake :1;
    uint8_t coast :1;
    uint8_t reserved :4;
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
    uint8_t version[3];  
    uint8_t  ioexpanderstatus;                /* firmware version [major, minor, patch] — READ-ONLY */
    uint8_t  numberpdo;
    uint8_t  numberpps;
    uint16_t minvolt;
    uint16_t maxvolt;
    uint16_t maxcurrent;
    uint8_t  selected;
    uint8_t  active;
    uint16_t voltage;
    uint16_t current;
    drive_status_t  drv1status;
    drive_config_t  drv1config;
    drive_control_t drv1control;
    uint8_t  drv1padding;
    uint16_t drv1speed;
    uint16_t drv1padding2;
    drive_status_t  drv2status;
    drive_config_t  drv2config;
    drive_control_t drv2control;
    uint8_t  drv2padding;
    uint16_t drv2speed;
    uint16_t drv2padding2;
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
    uint8_t flag_update_pd_select : 1;   /* set when PD select written via I2C */
    uint8_t flag_update_pd_active : 1;   /* set when PD actyive written via I2C */
    uint8_t flag_update_pd_voltage : 1;  /* set when PD voltage written via I2C */
    uint8_t flag_update_pd_current : 1;  /* set when PD current written via I2C */
    uint8_t flag_update_drv1_config : 1; /* set when Drive1 config written via I2C */
    uint8_t flag_update_drv1_control : 1;/* set when Drive1 control written via I2C */
    uint8_t flag_update_drv1_speed : 1;  /* set when Drive1 speed written via I2C */
    uint8_t flag_update_drv2_config : 1; /* set when Drive2 config written via I2C */

    uint8_t flag_update_drv2_control : 1;/* set when Drive2 control written via I2C */
    uint8_t flag_update_drv_speed : 1;   /* set when Drive2 speed written via I2C */
    uint8_t reserved : 6;                  /* reserved for future use */
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


/*
 * Configure AUX_POWER (PB6), LCD_RESET (PB11), and INT_OUTPUT (PC17) as
 * push-pull GPIO outputs.
 * The main loop drives these pins based on the I2C-writable output flags.
 */
static void Outputs_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    /* AUX power and LCD reset are on GPIOB */
    GPIO_InitStructure.GPIO_Pin = AUX_POWER_PIN | LCD_RESET_PIN | LORA_RESET_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = INT_OUTPUT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(INT_OUTPUT_PORT, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      Button_Init
 *
 * @brief   Configure all button/charger GPIO inputs and start TIM3 at 100 Hz
 *          for the Button_Scan() debounce state machine.
 *
 * @param   arr - TIM3 auto-reload value (period)
 *          psc - TIM3 prescaler value
 *
 * @return  none
 */
static void Button_Init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    /* Enable AFIO, TIM3, GPIO A, B and C clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);

    /* configure GPIO as inputs */
    GPIO_InitStructure.GPIO_Pin = CHARGER_CHARGING_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CHARGER_CHARGING_PORT, &GPIO_InitStructure); // GPIOA

    GPIO_InitStructure.GPIO_Pin = CHARGER_STANDBY_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; // TODO: what should these be?
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CHARGER_STANDBY_PORT, &GPIO_InitStructure); // GPIOB

    GPIO_InitStructure.GPIO_Pin = BUTTON_X_PIN | BUTTON_A_PIN | BUTTON_B_PIN | BUTTON_Y_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BUTTON_MENU_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BUTTON_MENU_PORT, &GPIO_InitStructure); // GPIOC

    /* Initialize Timer3 */
    TIM_TimeBaseStructure.TIM_Period = arr;
    TIM_TimeBaseStructure.TIM_Prescaler = psc;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    /* configure timer interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* enable timer interrupts */
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    /* Enable Timer3 */
    TIM_Cmd(TIM3, ENABLE);
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
    if (pos >= PD_OFFSET + PD_RW_OFFSET && pos < PD_OFFSET + PD_SIZE) return 1;
    if (pos >= PD_DRIVE1 + DRIVE_RW_OFFSET && pos < DRIVE1_OFFSET + DRIVE_SIZE) return 1;
    if (pos >= PD_DRIVE2 + DRIVE_RW_OFFSET && pos < DRIVE2_OFFSET + DRIVE_SIZE) return 1;
     1;
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
        state.flag_slave_first_write = 1;
    }

    /* Data register not empty (Receiver) flag */
    if (flag1 & I2C_STAR1_RXNE)
    {
        uint8_t byte = I2C_ReceiveData(I2C1);
        /* the first byte received after address+W is the register offset.
         * Subsequent payload bytes are read and written if the memory area
         * is writeable.
         */
        if (state.flag_slave_first_write)
        {
            state.slave_offset = byte;
            state.slave_position = byte;
            state.flag_slave_first_write = 0;
            PRINT("I2C reg: 0x%02x\r\n", byte);
        }
        else
        {
            if (i2c_pos_is_writable(state.slave_position))
            {
                state.raw_data[state.slave_position] = byte;
                switch (state.slave_postion)
                {
                case PD_SELECT :
                    state.flag_update_pd_select = 1;
                    break;
                case PD_ACTIVE :
                    state.flag_update_pd_active = 1;
                    break;
                case PD_VOLTAGE + 1 : // last byte of voltage written
                    state.flag_update_pd_voltage =1;
                    break;
                case PD_CURRENT + 1 : // last byte of current written
                    state.flag_update_pd_current =1;
                    break;
                case DRV1_CONFIG :
                    state.flag_update_drv1_config = 1;
                    break;
                case DRV1_CONTROL :
                    state.flag_update_drv1_control = 1;
                    break;
                case DRV1_SPEED :
                    state.flag_update_drv1_speed = 1;
                    break;
                case DRV2_CONFIG :
                    state.flag_update_drv2_config = 1;
                    break;
                case DRV2_CONTROL :
                    state.flag_update_drv2_control = 1;
                    break;
                case DRV2_SPEED :
                    state.flag_update_drv_speed = 1;
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

void run_Drive1(void){
    if (state.flag_update_drv1_config == 1){
        /// TODO change config
        /// set mode and sleep signals
        /// reset speed and set control to coast

        // set mode bit
        if (state.data.drv1mode.mode == 1)
            GPIO_WriteBit(GPIOB, DRV1_MODE_PIN , Bit_SET);
        else
            GPIO_WriteBit(GPIOB, DRV1_MODE_PIN , Bit_RESET);
        // set sleep bit
        if (state.data.drv1mode.sleep == 1)
            GPIO_WriteBit(GPIOB, DRV1_SLEEP_PIN , Bit_SET);
        else
            GPIO_WriteBit(GPIOB, DRV1_SLEEP_PIN , Bit_RESET);
        // set fault bit mode to input or output to write I2C address of DRV8847S
        if (state.data.drv1mode.address == 1){
        /// TODO check init
            GPIO_InitStructure.GPIO_Pin = DRV1_FAULT_PIN;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }
        else {
            GPIO_InitStructure.GPIO_Pin = DRV1_FAULT_PIN;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OPD;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
            GPIO_Init(GPIOB, &GPIO_InitStructure);
        }

        if (state.data.drv1mode.fault == 1)
            GPIO_WriteBit(GPIOB, DRV1_FAULT_PIN , Bit_SET);
        else
            GPIO_WriteBit(GPIOB, DRV1_FAULT_PIN , Bit_RESET);
        

        state.flag_update_drv1_config = 0;
    }
    if (state.flag_update_drv1_control == 1){
        /// TODO change control
        /// if going from fwd to rev go over break for xxx ms and reset speed
        /// going from brake or coast to FWD or REV is ok 
        /// clear change flag if not waiting
        if (state.data.drv1_config & 0x01 == 1) { // brake
            state.data.drv1_speed = 0;
            Brake(1);
        }
        else {
            if (state.data.drv1_config & 0x02 == 1) { // coast
                state.data.drv1_speed = 0;
            
            }
            setSpeed(state.data.drv1_speed,1)
        }
        state.flag_update_drv1_control = 0;
    }
    if (state.flag_update_drv1_speed == 1){
        /// TODO adjust speed ie adjust PWM CCR register
        /// if sign change go from REV <--> FWD
        /// if just speed adjustment clear change flag
        setSpeed(state.data.drv1_speed,1)

        state.flag_update_drv1_speed = 0;
    }
    /// read fault bit
}

void run_Drive2(void){
    if (state.flag_update_drv2_config == 1){
        /// TODO change config
        /// set mode and sleep signals
        /// reset speed and set control to coast
        state.flag_update_drv2_config = 0;
    }
    if (state.flag_update_drv2_control == 1){
        /// TODO change control
        /// if going from fwd to rev go over break for xxx ms and reset speed
        /// going from brake or coast to FWD or REV is ok 
        /// clear change flag if not waiting
        state.flag_update_drv2_control = 0;
    }
    if (state.flag_update_drv2_speed == 1){
        /// TODO adjust speed ie adjust PWM CCR register
        /// if sign change go from REV <--> FWD
        /// if just speed adjustment clear change flag
    }    
}

void run_IOexpander (void){
    /// TODO read interupt input and write to register
}

int run_PD(void){
/// TODO check if we are connected else return
    
    if (state.flag_update_pd_select == 1){
        // limit the value range of select and active
        if (state.data.selected > PD_getPDONum() || state.data.selected <= 0) state.data.selected = 1      
        state.data.maxvoltage = PD_getMaxVoltage(state.data.selected) ;
        state.data.minvoltage = PD_getMinVoltage(state.data.selected) ;
        state.data.mincurrent = PD_getMinCurrent(state.data.selected);
        /// TODO  update capabilties and set registers
        printSourceCap();
        state.flag_pd_select = 0;
    }
    if (state.flag_update_pd_active == 1){
        // limit the value range of select and active
        if (state.data.active > PD_getPDONum() || state.data.active <= 0) state.data.active = lstActive;
        if(state.data.active <= PD_getFixedNum()) {
            if(! PD_setPDO(state.data.active, PD_getPDOVoltage(state.data.active)))
                setActive(lstActive); // if not succesfull revert to last PDO
        }
        else { // it is a PPS set voltage
            PD_negotiate();
            if(!PD_setPDO(state.data.active, state.data.voltage)) {
            state.data.active = lstActive; // if not succesfull revert to last PDO
            }
        }
        state.flag_update_pd_active = 0;
        state.data.voltage = PD_getVoltage();
    }
    if (state.flag_update_pd_voltage == 1){

        PD_setVoltage(state.data.active, state.data.voltage);
        state.flag_update_pd_voltage = 0;
    }
    PD_negotiate();
  }
}


/* main */
int main(void)
{
    /* Zero-initialise all state data and flags before touching any hardware */
    memset(&state, 0, sizeof(addon_state_t));

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

    /* configure the output GPIO */
    Outputs_Init();

    /* Perform a hard reset of the LCD controller and Lora module early at boot:
     *   Pull RESET low → wait 120 ms → pull high again.
     *   This satisfies the reset timing requirements of ST7789v SPI LCD modules
     *   and the SX1262
     */
    GPIO_WriteBit(GPIOB, LCD_RESET_PIN | LORA_RESET_PIN, Bit_SET);
    Delay_Ms(10);
    GPIO_WriteBit(GPIOB, LCD_RESET_PIN | LORA_RESET_PIN, Bit_RESET);
    Delay_Ms(120);
    GPIO_WriteBit(GPIOB, LCD_RESET_PIN | LORA_RESET_PIN, Bit_SET);

#if (DEBUG)
    USART_Printf_Init(115200);
#endif

    /* 1-second boot delay so a USB serial monitor can attach and SWD can
     * connect before any peripheral or I2C activity begins.
     */
    Delay_Ms(1000);

    PRINT("SystemClk: %u\r\n", (unsigned)SystemCoreClock);
    PRINT("ChipID: %08x\r\n", (unsigned)DBGMCU_GetCHIPID());

    /* configure the I2C pins and interrupts */
    IIC_Init(I2C_SPEED, I2C_ADDRESS); // maps SWD lines to I2C
    /* configure the GPIO inputs and debounce timer */
    Button_Init(1, TIMER_FREQ);

    /* configure the analog input reading using DMA:
     *   ADC1 scans all ADC_CHANNELS in continuous/circular mode;
     *   DMA writes results directly into state.data.adc_channels[] without CPU.
     */
    ADC_MultiChannel_Init();
    DMA_Tx_Init(ADC_DMA_CHANNEL, (u32)&ADC1->RDATAR, (u32)state.data.adc_channels, ADC_CHANNELS);
    DMA_Cmd(ADC_DMA_CHANNEL, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* configure the LCD backlight PWM output using DMA:
     *   The timer Update event triggers DMA to copy state.data.lcd_brightness
     *   into the compare register (CCR), so the duty cycle tracks the variable
     *   automatically without any extra CPU writes.
     */
    LCD_PWM_Init(100, TIMER_FREQ, state.data.lcd_brightness);
    LCD_PWM_DMA_Init((u32)&state.data.lcd_brightness);
    TIM_DMACmd(LCD_BACKLIGHT_TIM, TIM_DMA_Update, ENABLE);
    TIM_Cmd(LCD_BACKLIGHT_TIM, ENABLE);
    TIM_CtrlPWMOutputs(LCD_BACKLIGHT_TIM, ENABLE);

    /* configure the Debug LED PWM output using DMA (same mechanism as above) */
    LED_PWM_Init(100, TIMER_FREQ, state.data.led_brightness);
    LED_PWM_DMA_Init((u32)&state.data.led_brightness);
    TIM_DMACmd(DEBUG_LED_TIM, TIM_DMA_Update, ENABLE);
    TIM_Cmd(DEBUG_LED_TIM, ENABLE);
    TIM_CtrlPWMOutputs(DEBUG_LED_TIM, ENABLE);

    /* Apply safe defaults: enable AUX power, release LCD reset,
     * release lora reset, set 50% brightness
     */

    PRINT("Expander Init done\r\n");

    /* Main event loop — all heavy lifting is done in ISRs and DMA;
     * the loop only reacts to flags set by those background mechanisms.
     */
    while (1)
    {
        /* Halfway through the debounce window (50 ms): deassert INT_OUTPUT */
        if (state.flag_button_scan_halfway)
        {
            state.flag_button_scan_halfway = 0;
            GPIO_WriteBit(INT_OUTPUT_PORT, INT_OUTPUT_PIN, Bit_RESET);
        }

        /* Debounce complete and state changed: assert INT_OUTPUT to tell the
         * ESP32 to issue an I2C read and fetch the new inputs register.
         */
        if (state.flag_button_state_changed)
        {
            state.flag_button_state_changed = 0;
            GPIO_WriteBit(INT_OUTPUT_PORT, INT_OUTPUT_PIN, Bit_SET);
        }

        /* I2C master wrote a new value to the outputs register: apply it now.
         * Also handles the reboot-to-bootloader command if that bit is set.
         */
        if (state.flag_update_outputs)
        {
            state.flag_update_outputs = 0;
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
                I2C_ITConfig(I2C1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF, DISABLE);

                /* disable I2C1 */
                I2C_Cmd(I2C1, DISABLE);

                /* Re-enable DIO (SWD) interface on these pins */
                GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, DISABLE);
            }
        }
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
        Button_Scan();
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
