#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h>
#include<ILI9163.h>
#include<stdio.h>

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

 SPI1_init();
 LCD_init();
 LCD_clearScreen();
  
     __builtin_enable_interrupts();

    char msg[100];
    int g=0;
    
    while(1){
    sprintf(msg,"hello %d",g);
   
    LCD_drawString(msg, 2, 2, YELLOW, MAGENTA);
    LCD_bar(50, 50, g, 50, YELLOW, MAGENTA);

    g++;
    if(g==50){
        g=0;
    }
    
    _CP0_SET_COUNT(0);
    while(_CP0_GET_COUNT()<48000000/2/5){}
    }
  }

void LCD_drawString(char *msg, short x, short y, short ColorFront, short ColorBack) {
    int i = 0;
    while (msg[i] !=0) {
        
        LCD_drawChar(msg[i], x+(i*5), y, ColorFront, ColorBack);
        i++;
    }

}

void LCD_drawChar(char c, short x, short y, short ColorFront, short ColorBack) {
    int i;
    int j;

    for (i = 0; i < 5; i++) {
        for (j = 0; j < 8; j++) {
            if (((ASCII[c - 0 x 20][i]) >> j) & 1 == 1) {
                LCD_drawPixel(x + i, y + j, ColorFront);
            } else {
                LCD_drawPixel(x + i, y + j, ColorBack);
            }


        }
    }
}

