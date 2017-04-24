#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>
sgdafgsdfgh#includeILsdfasgasgfasfgsdfgsdfg

#define CS LATAbits.LATA4

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

    spi_init();


    __builtin_enable_interrupts();



  
    int i = 0;

            unsigned char sinewave[100];
            unsigned char rampwave[100];
            float temp;

    for (i = 0; i < 100; i++) {
        temp = (255.0 / 2.0)+(255.0 / 2.0) * sin(2.0 * 3.14 * i / 100.0);
                sinewave[i] = temp;
                temp = i * 255.0 / 100.0;
                rampwave[i] = temp;
    }
    i = 0;
    while (1) {
        // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
        // remember the core timer runs at half the CPU speed

        _CP0_SET_COUNT(0);
        while (_CP0_GET_COUNT() < 48000000 / 2 / 1000) {
        }
        write_dac(0, sinewave[i]);
                write_dac(1, rampwave[i]);

                i++;
        if (i == 100) {

            i = 0;
        }
    }




}


  void LCD_drawChar(char c, short x, short y, short colorfront, short colorback) {
      int i;
      int j;
      
      for (i = 0; i < 5; i++) {
            for (j = 0; j < 8; j++) {
                if (((ASCII[i][c - 0x20]) >>j) &1 == 1){
                        drawpixel(x + i, y + j, colorfront);
                }
                else {
                    LCD_drawPixel(x + i, y + j, colorback);
                }

                
            }
        }
    }