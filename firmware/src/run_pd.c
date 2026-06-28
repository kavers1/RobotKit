// ===================================================================================
// Project:   USB PD Tester for CH32X035
// Version:   v1.3
// Year:      2024
// Author:    Stefan Wagner
// Github:    https://github.com/wagiminator
// EasyEDA:   https://easyeda.com/wagiminator
// License:   http://creativecommons.org/licenses/by-sa/3.0/
// ===================================================================================
//
// Description:
// ------------
// The USB PD Tester allows users to retrieve and test the capabilities of a connected
// USB Power Delivery Adapter.
//
// References:
// -----------
// - WCH Nanjing Qinheng Microelectronics: http://wch.cn
//
// Compilation Instructions:
// -------------------------
// - Make sure GCC toolchain (gcc-riscv64-unknown-elf, newlib) and Python3 with chprog
//   are installed. In addition, Linux requires access rights to the USB bootloader.
// - Press the BOOT button on the MCU board and keep it pressed while connecting it
//   via USB to your PC.
// - Run 'make flash'.


// ===================================================================================
// Libraries, Definitions and Macros
// ===================================================================================
#include <config.h>             // user configurations
#include <ssd1306_txt.h>        // OLED text functions
#include <usbpd_sink.h>         // USB PD sink functions

// Global variables
int8_t   pdstatus = -1;         // PD execution and connection status
uint8_t  select  = 1;           // selected PDO
uint8_t  active  = 1;           // active PDO
uint8_t  lstActive = 1;         // last active PDO
uint16_t voltage = 5000;        // selected voltage
uint16_t maxV = 5000;           // max voltage of selected PDO
uint16_t minV = 5000;           // min voltage of selected PDO
uint16_t maxI = 1000;           // max current of selected PDO

// ===================================================================================
// Functions
// ===================================================================================

// Set selected PDO marker
void setSelect(uint8_t pdo) {
  if(pdo > PD_getPDONum()) select = 1;
  else if(pdo < 1)         select = PD_getPDONum();
  else                     select = pdo;
}

// Set active PDO marker
void setActive(uint8_t pdo) {
  active = pdo;
}

// Set selected voltage
void setVoltage(uint16_t v) {
  if     (v <= PD_getPDOMinVoltage(select)) voltage = PD_getPDOMinVoltage(select);
  else if(v >= PD_getPDOMaxVoltage(select)) voltage = PD_getPDOMaxVoltage(select);
  else                                      voltage = v;
}

void describeSourceCap(uint_8 pdo)
{
  if(pdo <= PD_getFixedNum()) 
      maxV = PD_getPDOVoltage(pdo); 
      minV = maxV; // PDO fixed voltage
      maxI = PD_getPDOMaxCurrent(pdo);
    else
      maxV = PD_getPDOVoltage(pdo); 
      minV = PD_getPDOMinVoltage(pdo);
      maxI = PD_getPDOMaxCurrent(pdo);
}

// initialize PD
int init_PD(void){

/// TODO what is this   
  // Setup USB-PD
  #if DISABLE_SWJ
  RCC->APB2PCENR |= RCC_AFIOEN;        // enable AFIO clock
  AFIO->PCFR1 |= AFIO_PCFR1_SWJ_CFG_2; // disable SWJ on pins PC18 and PC19
  #endif
  pdstatus = 0; // init
  if(!PD_connect()) {
    pdstatus = -1; // failed
    return -1;
  }
  pdstatus = 1; // connected
  return 1;
}

int run_PD(void){
/// TODO check if we are connected else return

  // limit the value range of select and active
  if (select > PD_getPDONum() || select <= 0) select = 1
  if (active > PD_getPDONum() || active <= 0) active = lstActive;

  // try to set PDO 
  // is a PDO 
  if(active <= PD_getFixedNum()) {
    if(! PD_setPDO(active, PD_getPDOVoltage(active)))
        
        setActive(lstActive); // if not succesfull revert to last PDO
  }
  else { // it is a PPS set voltage
    PD_negotiate();
    if(!PD_setPDO(active, voltage)) {
      active = lstActive; // if not succesfull revert to last PDO
    }
  }  
  PD_negotiate();
}
  // Loop
  while(1) {

    if(!PIN_read(PIN_KEY_SLCT)) {
      else {
        printPPS();
        while(!PIN_read(PIN_KEY_SLCT));
        DLY_ms(10);
        while(PIN_read(PIN_KEY_SLCT)) {


          PD_negotiate();
        }

        if(PD_setPDO(select, voltage))
          active = select;
        printSourceCap();
      }
      while(!PIN_read(PIN_KEY_SLCT));
    }

    PD_negotiate();
    DLY_ms(10);
  }
}
