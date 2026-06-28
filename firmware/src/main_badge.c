/*
 * Copyright (c) 2026 Bert Outtier <outtierbert@gmail.com>
 *
 * Badge Expander firmware for the Fri3d Badge 2026.
 *
 * This firmware runs on a WCH CH32X035 microcontroller and acts as a
 * peripheral for the main MCU of the Fri3d badge (ESP32S3). It exposes all hardware
 * inputs/outputs through a single I2C slave interface (address 0x50):
 *
 *   Inputs (readable via I2C):
 *     - multimeter analog input (AIN0)
 *     - Joystick (analog X/Y, plus derived digital up/down/left/right)
 *     - 5 buttons: X, A, B, Y, Menu (active LOW with pull-up)
 *     - Battery voltage monitor (ADC)
 *     - USB Voltage monitor (plus derived digital signal "usb_plugged")
 *     - Charger status: charging and standby signals
 *
 *   Outputs (writable via I2C):
 *     - LCD backlight PWM brightness (TIM1 CH4 → DMA → CCR)
 *     - Debug LED PWM brightness     (TIM2 CH3 → DMA → CCR)
 *     - AUX power rail enable/disable
 *     - LCD reset signal
 *     - Reboot-to-bootloader trigger
 *     - Remap I2C to SWD trigger
 *     - Lora reset signal
 *
 *   Interrupt output:
 *     - PC17 is pulsed HIGH when any digital input state changes, then cleared
 *       at the next debounce cycle midpoint, so the badge knows to poll.
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

/*
 * Analog input pin definitions.
 * RANK sets the order in which the ADC scans channels continuously.
 * DMA copies results into state.data.adc_channels[rank-1] in this order:
 *   [0] AIN0        — multimeter analog input : PA0
 *   [1] BATTERY     — battery voltage (via resistor divider) : PC0
 *   [2] USB_MONITOR — 5V USB rail (via resistor divider) : PC3
 *   [3] JOYSTICK_Y  — joystick Y axis : PA6
 *   [4] JOYSTICK_X  — joystick X axis : PA5
 */
#define AIN0_PORT               GPIOA // PA0: Ain0
#define AIN0_PIN                GPIO_Pin_0
#define AIN0_CHANNEL            ADC_Channel_0
#define AIN0_RANK               (1)
#define BATTERY_MONITOR_PORT    GPIOC // PC0: Battery Monitor
#define BATTERY_MONITOR_PIN     GPIO_Pin_0
#define BATTERY_MONITOR_CHANNEL ADC_Channel_10
#define BATTERY_MONITOR_RANK    (2)
#define USB_MONITOR_PORT        GPIOC // PC3: USB Monitor
#define USB_MONITOR_PIN         GPIO_Pin_3
#define USB_MONITOR_CHANNEL     ADC_Channel_13
#define USB_MONITOR_RANK        (3)
#define JOYSTICK_Y_PORT         GPIOA // PA6: JoyY
#define JOYSTICK_Y_PIN          GPIO_Pin_6
#define JOYSTICK_Y_CHANNEL      ADC_Channel_6
#define JOYSTICK_Y_RANK         (4)
#define JOYSTICK_X_PORT         GPIOA // PA5: JoyX
#define JOYSTICK_X_PIN          GPIO_Pin_5
#define JOYSTICK_X_CHANNEL      ADC_Channel_5
#define JOYSTICK_X_RANK         (5)
#define ADC_CHANNELS            (5) /* total number of ADC channels scanned */
#define ADC_DMA_CHANNEL         DMA1_Channel1

/* digital inputs */
#define CHARGER_CHARGING_PORT GPIOA // PA7: Charger, charging
#define CHARGER_CHARGING_PIN  GPIO_Pin_7
#define CHARGER_STANDBY_PORT  GPIOB // PB0: Charger, standby
#define CHARGER_STANDBY_PIN   GPIO_Pin_0
#define BUTTON_X_PORT         GPIOB // PB7: X Button
#define BUTTON_X_PIN          GPIO_Pin_7
#define BUTTON_A_PORT         GPIOB // PB8: A Button
#define BUTTON_A_PIN          GPIO_Pin_8
#define BUTTON_B_PORT         GPIOB // PB9: B Button
#define BUTTON_B_PIN          GPIO_Pin_9
#define BUTTON_Y_PORT         GPIOB // PB10: Y Button
#define BUTTON_Y_PIN          GPIO_Pin_10
#define BUTTON_MENU_PORT      GPIOC // PC15: Menu Button
#define BUTTON_MENU_PIN       GPIO_Pin_15

/* digital outputs */
#define AUX_POWER_PORT  GPIOB // PB6: Aux power
#define AUX_POWER_PIN   GPIO_Pin_6
#define LCD_RESET_PORT  GPIOB // PB11: LCD reset
#define LCD_RESET_PIN   GPIO_Pin_11
#define INT_OUTPUT_PORT GPIOC // PC17: Interrupt output
#define INT_OUTPUT_PIN  GPIO_Pin_17
#define LORA_RESET_PORT GPIOB // PB1/5: Lora reset pin
#define LORA_RESET_PIN  GPIO_Pin_1

// PWM
#define DEBUG_LED_PORT                GPIOB // PB3: debug LED
#define DEBUG_LED_PIN                 GPIO_Pin_3
#define DEBUG_LED_TIM                 TIM2                    // TIM2 Channel 3
#define DEBUG_LED_TIM_REMAP           GPIO_PartialRemap2_TIM2 // mapping 0b010
#define DEBUG_LED_TIM_CVR             TIM2->CH3CVR            // TIM2 Channel 3 compare register
#define DEBUG_LED_TIM_DMA_CHANNEL     DMA1_Channel2           // DMA channel for TIM2_UP
#define LCD_BACKLIGHT_PORT            GPIOB                   // PB12: LCD Backlight
#define LCD_BACKLIGHT_PIN             GPIO_Pin_12
#define LCD_BACKLIGHT_TIM             TIM1                    // TIM1 Channel 4
#define LCD_BACKLIGHT_TIM_REMAP       GPIO_PartialRemap2_TIM1 // mapping 0b010
#define LCD_BACKLIGHT_TIM_CVR         TIM1->CH4CVR            // TIM1 Channel 4 compare register
#define LCD_BACKLIGHT_TIM_DMA_CHANNEL DMA1_Channel5           // DMA channel for TIM1_UP

/* I2C slave interface towards the ESP32S3 on Fri3d badge 2026 */
#define SDA_PORT         GPIOC
#define SDA_PIN          GPIO_Pin_18
#define SCL_PORT         GPIOC
#define SCL_PIN          GPIO_Pin_19
#define I2C_ADDRESS      (0x50) /* 7-bit slave address; write=0xA0, read=0xA1 */
#define I2C_TIMEOUT      (-2)
#define I2C_TIMEOUT_TICK ((SystemCoreClock / 10) - 1) /* 100 ms timeout for I2C reads */
#define I2C_SPEED        (400000)

/*
 * Joystick ADC thresholds for converting analog position to digital direction bits.
 * The joystick ADC range is 0–4095 (12-bit).
 *   value > THRESHOLD_TOP    → pushed in positive direction (up / right)
 *   value < THRESHOLD_BOTTOM → pushed in negative direction (down / left)
 *   in between               → centered (no direction bit set)
 *
 * USB voltage threshold: the 5V USB rail is divided by 2 before the ADC.
 * At 5V: (5V/2) / 3.3V * 4095 ≈ 3100. Values > 3000 are treated as "USB present".
 */
#define JOYSTICK_THRESHOLD_TOP    (3000)
#define JOYSTICK_THRESHOLD_BOTTOM (1000)
#define USB_VOLTAGE_THRESHOLD     (3000)

/* TIM3 auto-reload value for a 100 Hz interrupt used for button debounce timing */
#define TIMER_FREQ ((SystemCoreClock / 10000) - 1)

/*
 * I2C register map layout (also the memory layout of addon_data_t / raw_data[]):
 *
 *   Offset  Size  Description
 *   ------  ----  -------------------------------------------
 *   0x00     3    Firmware version [major, minor, patch]   (READ-ONLY)
 *   0x03     1    Padding (required for 4-byte alignment of ADC buffer)
 *   0x04     2    Button/input state (buttons_t bitmask)   (READ-ONLY)
 *   0x08    10    ADC channels[0..4] as uint16_t           (READ-ONLY)
 *   0x12     2    LCD backlight brightness (uint16, 0–100) (READ-WRITE)
 *   0x14     2    Debug LED brightness    (uint16, 0–100)  (READ-WRITE)
 *   0x16     1    Output flags: aux_power, lcd_reset, reboot (READ-WRITE)
 *
 * Total: RESULT_BUFFER_SIZE = 23 bytes.
 */
#define BUTTON_SIZE        (2) /* sizeof(buttons_t) */
#define RESULT_BUFFER_SIZE (3 + 1 + BUTTON_SIZE + 2 + (ADC_CHANNELS * 2) + 2 + 2 + 1)
#define PWM_LCD_OFFSET     (3 + 1 + BUTTON_SIZE + 2 + (ADC_CHANNELS * 2)) /* byte offset of lcd_brightness */
#define PWM_LED_OFFSET     (PWM_LCD_OFFSET + 2)                       /* byte offset of led_brightness */
#define OUTPUTS_OFFSET     (PWM_LED_OFFSET + 2)                       /* byte offset of output flags */

/*
 * All digital input states packed into 2 bytes (BUTTON_SIZE).
 * Button bits are 1 = pressed/active. Joystick and USB plugged bits are derived from the ADC.
 */
typedef struct
{
    uint8_t charger_charging : 1; /* battery charger IC: actively charging */
    uint8_t charger_standby : 1;  /* battery charger IC: standby (charge complete) */
    uint8_t button_x : 1;         /* X button pressed */
    uint8_t button_y : 1;         /* Y button pressed */
    uint8_t button_a : 1;         /* A button pressed */
    uint8_t button_b : 1;         /* B button pressed */
    uint8_t button_menu : 1;      /* Menu button pressed */
    uint8_t joy_up : 1;           /* joystick pushed up    (Y ADC > THRESHOLD_TOP) */
    uint8_t joy_down : 1;         /* joystick pushed down  (Y ADC < THRESHOLD_BOTTOM) */
    uint8_t joy_left : 1;         /* joystick pushed left  (X ADC < THRESHOLD_BOTTOM) */
    uint8_t joy_right : 1;        /* joystick pushed right (X ADC > THRESHOLD_TOP) */
    uint8_t usb_plugged : 1;      /* USB 5V rail detected (USB_MONITOR ADC > threshold) */
    uint8_t reserved : 4;
} buttons_t;

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
    uint8_t version[3];                  /* firmware version [major, minor, patch] — READ-ONLY */
    uint8_t unused;                      /* alignment padding: keeps adc_channels[] at a 4-byte boundary */
    buttons_t inputs;                    /* all button/joystick/USB/charger states — READ-ONLY */
    uint16_t unused2;                    /* to keep backwards compatibility */
    uint16_t adc_channels[ADC_CHANNELS]; /* raw 12-bit ADC values, written by DMA — READ-ONLY */
    uint16_t lcd_brightness;             /* LCD backlight duty cycle 0–100; DMA copies to TIM1 CCR — READ-WRITE */
    uint16_t led_brightness;             /* debug LED duty cycle 0–100; DMA copies to TIM2 CCR — READ-WRITE */
    uint8_t aux_power : 1;               /* 1 = enable the AUX power rail — READ-WRITE */
    uint8_t lcd_reset : 1;               /* 1 = release LCD from reset (0 = held in reset) — READ-WRITE */
    uint8_t reboot : 1;                  /* write 1 to trigger a reboot into the bootloader — READ-WRITE */
    uint8_t remap : 1;                   /* write 1 to remap the SWD to the I2C pins — READ-WRITE */
    uint8_t lora_reset : 1;              /* 1 = release Lora module from reset (0 = held in reset) — READ-WRITE */
    uint8_t output_reserved : 3;
} addon_data_t;

/* Compile-time check: struct layout must match RESULT_BUFFER_SIZE exactly */
_Static_assert(sizeof(addon_data_t) == RESULT_BUFFER_SIZE, "raw data and struct size are not aligned!");

/*
 * Global firmware state.
 * Flags are set by interrupt handlers and consumed by the main loop.
 */
typedef struct
{
    uint8_t flag_update_outputs : 1;       /* set when aux_power/lcd_reset/reboot/remap/lora_reset were written via I2C */
    uint8_t flag_button_scan_halfway : 1;  /* set at the halfway point of a debounce cycle (used to clear the interrupt line) */
    uint8_t flag_button_state_changed : 1; /* set when a stable button state change is detected */
    uint8_t flag_slave_first_write : 1;    /* set on every ADDR phase; the next RXNE byte is the register offset. */
    uint8_t reserved : 4;                  /* reserved for future use */
    uint8_t slave_offset;                  /* register offset captured after the most recent ADDR+W. */
    uint8_t slave_position;                /* current read/write cursor, reset to offset on every ADDR (including repeated-START), so write-then-read works without special-casing. */
    uint8_t unused;
    union
    {
        addon_data_t data;                    /* structured access to the register map */
        uint8_t raw_data[RESULT_BUFFER_SIZE]; /* flat byte access for I2C transfers */
    };
} addon_state_t;

/* global state variable */
static addon_state_t state;

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
 * Configure TIM1 Channel 4 to generate a PWM signal on PB12 (LCD backlight).
 * The duty cycle is set by the CH4CVR compare register; values 0–arr map to 0–100% duty.
 * PWM mode 2 with polarity LOW: output is HIGH while CNT < CCR (active-high backlight).
 * The initial duty cycle is set to 'ccp'; afterwards it is updated via DMA automatically.
 *
 * @param arr  timer period (auto-reload value)
 * @param psc  timer prescaler
 * @param ccp  initial compare value (duty cycle)
 */
static void LCD_PWM_Init(uint16_t arr, uint16_t psc, uint16_t ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};

    /* enable timers and GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1 | RCC_APB2Periph_GPIOB, ENABLE);

    /* set pinmux */
    GPIO_PinRemapConfig(LCD_BACKLIGHT_TIM_REMAP, ENABLE); // set LCD backlight (PB12) on CH4 of TIM1

    GPIO_InitStructure.GPIO_Pin = LCD_BACKLIGHT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LCD_BACKLIGHT_PORT, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(LCD_BACKLIGHT_TIM, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // High until CNT < CCR
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp; // start duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init(LCD_BACKLIGHT_TIM, &TIM_OCInitStructure);

    TIM_OC4PreloadConfig(LCD_BACKLIGHT_TIM, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(LCD_BACKLIGHT_TIM, ENABLE);
}

/*********************************************************************
 * @fn      LCD_PWM_DMA_Init
 *
 * @brief   Configure DMA to automatically update the LCD backlight PWM duty cycle.
 *
 * This uses the "DMA-driven CCR" trick: TIM1 is configured to request a DMA
 * transfer on each Update event (TIM_DMA_Update). The DMA copies one uint16
 * from 'memadr' (= &state.data.lcd_brightness) directly into TIM1->CH4CVR.
 * Because DMA_Mode_Circular is used, this repeats every timer period, so any
 * change to state.data.lcd_brightness takes effect within one PWM period — no
 * interrupt or manual register write needed.
 *
 * @param memadr  address of the source value (state.data.lcd_brightness)
 */
static void LCD_PWM_DMA_Init(u32 memadr)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(LCD_BACKLIGHT_TIM_DMA_CHANNEL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&LCD_BACKLIGHT_TIM_CVR; /* destination: TIM1 CH4 compare reg */
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;                          /* source: state.data.lcd_brightness */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(LCD_BACKLIGHT_TIM_DMA_CHANNEL, &DMA_InitStructure);

    DMA_Cmd(LCD_BACKLIGHT_TIM_DMA_CHANNEL, ENABLE);
}

/*
 * Configure TIM2 Channel 3 to generate a PWM signal on PB3 (debug LED).
 * Same DMA-driven CCR mechanism as LCD_PWM_Init; see that function for details.
 * Note: polarity is HIGH here (vs LOW for the LCD), so the LED turns ON when CNT < CCR.
 *
 * @param arr  timer period (auto-reload value)
 * @param psc  timer prescaler
 * @param ccp  initial compare value (duty cycle)
 */
static void LED_PWM_Init(uint16_t arr, uint16_t psc, uint16_t ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    TIM_OCInitTypeDef TIM_OCInitStructure = {0};

    /* Enable Timer2 clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    /* enable timers and GPIO clocks */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);

    /* set pinmux */
    GPIO_PinRemapConfig(DEBUG_LED_TIM_REMAP, ENABLE); // set Debug LED (PB3) on CH3 of TIM2

    GPIO_InitStructure.GPIO_Pin = DEBUG_LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DEBUG_LED_PORT, &GPIO_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(DEBUG_LED_TIM, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; // High until CNT < CCR
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp; // start duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC3Init(DEBUG_LED_TIM, &TIM_OCInitStructure); // TIM2 channel 3

    TIM_OC3PreloadConfig(DEBUG_LED_TIM, TIM_OCPreload_Disable);
    TIM_ARRPreloadConfig(DEBUG_LED_TIM, ENABLE);
}

/*
 * Configure DMA to automatically update the debug LED PWM duty cycle.
 * Same mechanism as LCD_PWM_DMA_Init: TIM2 Update event triggers DMA1_Channel2
 * to copy state.data.led_brightness → TIM2->CH3CVR every timer period.
 *
 * @param memadr  address of the source value (state.data.led_brightness)
 */
static void LED_PWM_DMA_Init(u32 memadr)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DEBUG_LED_TIM_DMA_CHANNEL);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&DEBUG_LED_TIM_CVR; /* destination: TIM2 CH3 compare reg */
    DMA_InitStructure.DMA_MemoryBaseAddr = memadr;                      /* source: state.data.led_brightness */
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure.DMA_BufferSize = 1;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DEBUG_LED_TIM_DMA_CHANNEL, &DMA_InitStructure);

    DMA_Cmd(DEBUG_LED_TIM_DMA_CHANNEL, ENABLE);
}

/*
 * Initialize ADC1 in continuous scan mode with DMA.
 * All 5 analog channels are converted in a round-robin loop and written
 * directly into state.data.adc_channels[] via DMA1_Channel1 (circular mode).
 * The ADC clock is divided by 16 for stable readings.
 * Results are right-aligned 12-bit values (0–4095).
 */
static void ADC_MultiChannel_Init(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = JOYSTICK_X_PIN | JOYSTICK_Y_PIN | AIN0_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BATTERY_MONITOR_PIN | USB_MONITOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // turn off ADC before we configure it
    ADC_DeInit(ADC1);

    /* dial down the clock so that we have stable readings */
    ADC_CLKConfig(ADC1, ADC_CLK_Div16);

    // configure the ADC
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                  // operate in independent mode
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                        // scan multiple channels
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                  // continuous conversion
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; // no external trigger to start the conversion of regular channels
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;              // right align the ADC data
    ADC_InitStructure.ADC_NbrOfChannel = ADC_CHANNELS;                  // number of regular ADC channels to convert
    ADC_InitStructure.ADC_OutputBuffer = 0;                             // Not used on CH32X035? set to 0
    ADC_InitStructure.ADC_Pga = 0;                                      // Not used on CH32X035? set to 0
    ADC_Init(ADC1, &ADC_InitStructure);

    // configure the ADC channels
    ADC_RegularChannelConfig(ADC1, AIN0_CHANNEL, AIN0_RANK, ADC_SampleTime_11Cycles);
    ADC_RegularChannelConfig(ADC1, BATTERY_MONITOR_CHANNEL, BATTERY_MONITOR_RANK, ADC_SampleTime_11Cycles);
    ADC_RegularChannelConfig(ADC1, USB_MONITOR_CHANNEL, USB_MONITOR_RANK, ADC_SampleTime_11Cycles);
    ADC_RegularChannelConfig(ADC1, JOYSTICK_Y_CHANNEL, JOYSTICK_Y_RANK, ADC_SampleTime_11Cycles);
    ADC_RegularChannelConfig(ADC1, JOYSTICK_X_CHANNEL, JOYSTICK_X_RANK, ADC_SampleTime_11Cycles);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
}

/*
 * Configure a DMA channel for peripheral-to-memory transfer (e.g. ADC → RAM).
 * Uses circular mode so the channel auto-restarts after each complete scan,
 * keeping the memory buffer always up-to-date without CPU involvement.
 * VeryHigh priority ensures ADC data is not lost if other DMA channels are busy.
 */
static void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, uint32_t peripheralAddress, uint32_t memoryAddress, uint16_t bufferSize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = peripheralAddress;
    DMA_InitStructure.DMA_MemoryBaseAddr = memoryAddress;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufferSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
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
    if (pos >= PWM_LCD_OFFSET && pos < PWM_LCD_OFFSET + 2 + 2 + 1) return 1;
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
 *   - PWM_LCD_OFFSET  → next 2 bytes set lcd_brightness (uint16_t); DMA updates PWM automatically
 *   - PWM_LED_OFFSET  → next 2 bytes set led_brightness (uint16_t); DMA updates PWM automatically
 *   - OUTPUTS_OFFSET  → next 1 byte  sets aux_power/lcd_reset/reboot flags; flag_update_outputs raised
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
                if (state.slave_position == OUTPUTS_OFFSET)
                {
                    /* notify the main loop that the config has changed */
                    state.flag_update_outputs = 1;
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

/**
 * Sample all inputs and return a snapshot as a buttons_t struct.
 *
 * GPIO buttons (GPIOA/B/C) are active-LOW with internal pull-ups, so a pin
 * reading Bit_RESET (0) means the button is pressed. The ADC joystick axes
 * are compared against JOYSTICK_THRESHOLD_TOP / _BOTTOM to derive directional
 * bits, and the USB monitor channel is thresholded to detect VBUS presence.
 *
 * @return  buttons_t snapshot of all inputs at the moment of the call
 */
static buttons_t read_buttons(void)
{
    buttons_t res = {0};
    /* Snapshot entire GPIO port words to avoid reading individual pins */
    uint32_t a = GPIO_ReadInputData(GPIOA);
    uint32_t b = GPIO_ReadInputData(GPIOB);
    uint32_t c = GPIO_ReadInputData(GPIOC);
    /* take a local copy of the current Joystick and USB ADC values (written by DMA in background) */
    uint16_t joy_x = state.data.adc_channels[JOYSTICK_X_RANK - 1];
    uint16_t joy_y = state.data.adc_channels[JOYSTICK_Y_RANK - 1];
    uint16_t usb_voltage = state.data.adc_channels[USB_MONITOR_RANK - 1];

    // TODO: check polarities

    /* Charger status signals (active-LOW open-drain outputs of the charger IC) */
    if ((a & CHARGER_CHARGING_PIN) == (uint32_t)Bit_RESET)
    {
        res.charger_charging = 1;
    }

    if ((b & CHARGER_STANDBY_PIN) == (uint32_t)Bit_RESET)
    {
        res.charger_standby = 1;
    }

    /* Face buttons — active-LOW, pulled up internally */
    if ((b & BUTTON_X_PIN) == (uint32_t)Bit_RESET)
    {
        res.button_x = 1;
    }

    if ((b & BUTTON_A_PIN) == (uint32_t)Bit_RESET)
    {
        res.button_a = 1;
    }

    if ((b & BUTTON_B_PIN) == (uint32_t)Bit_RESET)
    {
        res.button_b = 1;
    }

    if ((b & BUTTON_Y_PIN) == (uint32_t)Bit_RESET)
    {
        res.button_y = 1;
    }

    if ((c & BUTTON_MENU_PIN) == (uint32_t)Bit_RESET)
    {
        res.button_menu = 1;
    }

    if (joy_y > JOYSTICK_THRESHOLD_TOP)
    {
        res.joy_up = 1;
    }

    if (joy_y < JOYSTICK_THRESHOLD_BOTTOM)
    {
        res.joy_down = 1;
    }

    if (joy_x > JOYSTICK_THRESHOLD_TOP)
    {
        res.joy_right = 1;
    }

    if (joy_x < JOYSTICK_THRESHOLD_BOTTOM)
    {
        res.joy_left = 1;
    }

    /* USB VBUS monitor: ADC channel reads a resistor-divided VBUS rail.
     * Threshold chosen so the bit is set only when USB is physically plugged in.
     */
    if (usb_voltage > USB_VOLTAGE_THRESHOLD)
    {
        res.usb_plugged = 1;
    }

    return res;
}

/*********************************************************************
 * @fn      Button_Scan
 *
 * @brief   Two-sample debounce state machine, called every 10 ms by TIM3.
 *
 * Debounce algorithm:
 *   - At tick%5  == 0 (every 50 ms): take first sample → previous_button_state
 *   - At tick%10 == 0 (every 100 ms): take second sample and compare
 *     If both samples match AND the result differs from the published state,
 *     update state.data.inputs and raise flag_button_state_changed so the main
 *     loop can pulse INT_OUTPUT to alert the ESP32S3.
 *
 * The INT_OUTPUT pin is deasserted at the halfway point (50 ms) so that the
 * ESP32S3 sees a clean pulse rather than a permanently asserted level.
 *
 * @return  none
 */
static void Button_Scan(void)
{
    static uint8_t scan_cnt = 0;
    static buttons_t previous_button_state = {0};

    scan_cnt++;
    if ((scan_cnt % 10) == 0) // every 100ms — second sample
    {
        /* reset the debounce counter */
        scan_cnt = 0;

        /* Take second sample and compare with first (debouncing):
         * Only accept the new state if both samples agree AND the result
         * is actually different from what is already published to the I2C master.
         */
        buttons_t button_state = read_buttons();
        if (memcmp(&button_state, &previous_button_state, BUTTON_SIZE) == 0 && memcmp(&button_state, &state.data.inputs, BUTTON_SIZE) != 0)
        {
            /* Stable new state: publish it and trigger an interrupt to the ESP32 */
            state.flag_button_state_changed = 1;
            state.data.inputs = button_state;
            memset(&previous_button_state, 0, BUTTON_SIZE);
        }
    }
    else if ((scan_cnt % 5) == 0) // every 50ms — first sample
    {
        /* Signal the main loop to deassert INT_OUTPUT (pulse width ends here) */
        state.flag_button_scan_halfway = 1;
        /* Save the first scan result for comparison at the 100 ms tick */
        previous_button_state = read_buttons();
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
    state.data.aux_power = 1;
    state.data.lcd_reset = 1;
    state.data.lora_reset = 1;
    state.flag_update_outputs = 1;
    state.data.lcd_brightness = 50;
    state.data.led_brightness = 50;

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
