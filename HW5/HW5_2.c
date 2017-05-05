#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
#include "HW5.h"
#define ADDRESS 0b0100000


// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = ON // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 000000000000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

void initExpander(void);
void setExpander(unsigned char);
unsigned char getExpander(unsigned char);
void i2c_write(unsigned char r, unsigned char d);
unsigned char i2c_read(unsigned char r);

int main() {

    __builtin_disable_interrupts();


    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;

    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
    initExpander();
    LATAbits.LATA4 = 0;
    __builtin_enable_interrupts();
    setExpander(0b00000000);


    while (1) {
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
        // remember the core timer runs at half the CPU speed

        unsigned char p = getExpander(9);

        if ((p & 0b10000000) == 0b10000000) {
            setExpander(1);
            LATAbits.LATA4 = 0;
        }
        else {
            setExpander(0);
            LATAbits.LATA4 = 1;
        }
    }
}

void initExpander(void) {
    i2c_write(0, 0b11110000);
    i2c_write(6, 0b11110000);
    i2c_write(9, 0b0);
}

void setExpander(unsigned char a) {
    i2c_write(9, a);
}

unsigned char getExpander(unsigned char r) {
    return i2c_read(r);
}

unsigned char i2c_read(unsigned char r) {
    i2c_master_start();
    i2c_master_send(ADDRESS << 1 | 0);
    i2c_master_send(r);
    i2c_master_restart();
    i2c_master_send(ADDRESS << 1 | 1);
    unsigned char b = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
    return b;
}

void i2c_write(unsigned char r, unsigned char d) {
    i2c_master_start();
    i2c_master_send(ADDRESS << 1 | 0);
    i2c_master_send(r);
    i2c_master_send(d);
    i2c_master_stop();

}