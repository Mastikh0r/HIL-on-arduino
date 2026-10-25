#include <Arduino.h>
#include <avr/interrupt.h>
#include "tdc_contract.h"
#include "spi_peripheral.h"

// Write spi_peripheral.cpp. You are an SPI peripheral (mode 0, MSB first) on the
// hardware pins: CS = D10 (PB2), COPI = D11, CIPO = D12 (PB4), SCK = D13.
// Do NOT call SPI.begin() (that makes you a controller). Use SPCR/SPDR and ISR(SPI_STC_vect).
extern bool intbFault; // when true, publish() never pulls INTB low

static const uint8_t A_CONFIG1    = 0x00;
static const uint8_t A_CONFIG2    = 0x01;
static const uint8_t A_INT_STATUS = 0x02;
static const uint8_t A_RES0       = 0x10;
static const uint8_t IMG_SIZE     = 0x30;


// Register image: keep a byte array (0x30 bytes is enough) for CONFIG1/CONFIG2/INT_STATUS/SEQ
// and a uint32_t array of N_RESULT_REGS for 0x10..0x1C (24-bit registers, sent MSB first).
static volatile uint8_t  regs[IMG_SIZE];
static volatile uint32_t results[N_RESULT_REGS];

static volatile bool startReq = false;

static volatile uint8_t state = 0;   // 0 = expect command, 1 = data bytes
static volatile bool isWrite = false;
static volatile bool autoInc = false;
static volatile uint8_t curAddr = 0;
static volatile uint8_t byteIdx = 0;


static inline bool isResult(uint8_t a) { 
    //check if the address is in the result register range of 13 regs
    return (a >= 0x10 && a < (0x10 + N_RESULT_REGS));
}


void intbRelease(){
    //clears bit pins
    DDRB  = DDRB & ~(1 << PB0); // make D8 an input
    PORTB = PORTB & ~(1 << PB0);
}

static inline void intbPullLow() {
    PORTB = PORTB & ~(1 << PB0);   // make D8 low
    DDRB  = DDRB | (1 << PB0);   // then become an output(i>o)
}

 // true once per START_MEAS write then clears
bool takeStartRequest() {
    if (!startReq) return false;
    startReq = false;
    return true;
}

static uint8_t nextReadByte() {
    // Case 1: The current address points to a multi-byte calculation result register
    if (isResult(curAddr)) {
        // get the raw 32-bit result and keep lower 23 bits
        uint32_t v = results[curAddr - A_RES0] & 0x7FFFFFUL;
        
        //get the specific byte requested by byteIdx (0 = High, 1 = Mid, 2 = Low byte)
        return (uint8_t)(v >> (16 - 8 * byteIdx));
    }
    // Case 2: The address points to stdrd regs or img mem
    if (curAddr < IMG_SIZE) {
        // Reading the interrupt status register automatically clears/releases the interrupt line
        if (curAddr == A_INT_STATUS) intbRelease();
        
        // Return the byte directly from the register array
        return regs[curAddr];
    }
    // Case 3: Out of bounds
    return 0;
}

static void storeWriteByte(uint8_t b) {
    if (isResult(curAddr)) return;                 // read-only
    if (curAddr == A_INT_STATUS) return;           // read-only
    if (curAddr >= IMG_SIZE) return; //out of bounds
    if (curAddr == A_CONFIG1) {
        if (b & CFG1_START_MEAS) {
        b = b & ~CFG1_START_MEAS;                // clearing
        regs[A_INT_STATUS] = 0;
        intbRelease();
        startReq = true;
        }
    }
    regs[curAddr] = b;
}

static void advance() {
    // how many bytes are in the layout results are 3 bytes others are 1
    uint8_t len = isResult(curAddr) ? 3 : 1;
    // Move to the next byte indx
    byteIdx++;
    // Check if all bytes have been transferred 
    if (byteIdx >= len) {
        // Reset 
        byteIdx = 0;
        // If  requested continuous streaming/reading, move to the next memory address
        if (autoInc) {
        // Advance address
        curAddr = (curAddr + 1) & 0x3F;
        } 
    }
}

ISR(SPI_STC_vect) {
    //input byte
    uint8_t in = SPDR;
    //command decode
    if (state == 0) {
        autoInc = (in & 0x80) != 0;
        isWrite = (in & 0x40) != 0;
        curAddr = in & 0x3F;
        byteIdx = 0;
        state = 1;
        SPDR = isWrite ? 0 : nextReadByte();         // first reply byte
    } else if (isWrite) {
        //write
        storeWriteByte(in);
        //load next byte
        advance();
        SPDR = 0;
        //blank reply
    } else {
        //wait for byte to finish
        advance();
        //load next byte
        SPDR = nextReadByte();
    }
}

// CS edge on D10 (PCINT2)
ISR(PCINT0_vect) {
  // reset state
  state = 0;
  
  // Check the physical level of the CS pin (PB2)
  if (PINB & (1 << PB2)) {
    // CS pin is HIGH
    // Set the CIPO pin as an Input
    DDRB = DDRB & ~(1 << PB4);                         
    
  } else {
    //CS pin is LOW 
    // set CIPO to be an Output
    DDRB = DDRB | (1 << PB4);                          
    
    // Clearthe so it's ready for the incoming SPI clock 
    SPDR = 0;
  }
}


//TODO
void spiPeripheralBegin();             // configure pins, SPCR, interrupts, defaults

//TODO
uint8_t getReg(uint8_t addr);          // read a byte from the register image

//TODO
// Copy r[] into the result registers, set INT_STATUS = status, then pull INTB low
// (unless intbFault). Do the copy with interrupts off so a read can't see half a result.
void publish(const uint32_t r[N_RESULT_REGS], uint8_t status);