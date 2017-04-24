#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include <math.h>

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

void write_dac(int channel, unsigned char voltage);
void spi_init(void);
unsigned char spi_io(unsigned char o);

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
    
    for (i=0;i<100;i++){
        temp = (255.0/2.0)+(255.0/2.0)*sin(2.0*3.14*i/100.0);
        sinewave[i] = temp;
        temp = i*255.0/100.0;
        rampwave[i] = temp;
    }
    i = 0;
    while(1) {
	    // use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
		  // remember the core timer runs at half the CPU speed
      
      _CP0_SET_COUNT(0);
      while (_CP0_GET_COUNT() < 48000000/2/1000) {}
      write_dac(0,sinewave[i]);
      write_dac(1,rampwave[i]);
      
      i++;
      if(i==100){
          i=0;
      }
      }
    
    

      
    }
    
   

// send a byte via spi and return the response
unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void spi_init(void) {
    // setup spi1
  RPA1Rbits.RPA1R = 0b0011;
  
  
  SPI1CON = 0;              // turn off the spi module and reset it
  SPI1BUF;                  // clear the rx buffer by reading from it
  SPI1BRG = 0x1000;            // baud rate to 10 MHz [SPI4BRG = (80000000/(2*desired))-1]
  SPI1STATbits.SPIROV = 0;  // clear the overflow bit
  SPI1CONbits.CKE = 1;      // data changes when clock goes from hi to lo (since CKP is 0)
  SPI1CONbits.MSTEN = 1;    // master operation
  SPI1CONbits.ON = 1;       // turn on spi 1==
  
  TRISAbits.TRISA4 = 0;
  CS = 1;
}

void SPI1_init() {
	SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
    RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
    TRISBbits.TRISB7 = 0; // SS is B7
    LATBbits.LATB7 = 1; // SS starts high

    // A0 / DAT pin
    ANSELBbits.ANSB15 = 0;
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;
	
	SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi1
}


void LCD_clearScreen(unsigned short color) {
    int i;
    LCD_setAddr(0,0,_GRAMWIDTH,_GRAMHEIGH);
		for (i = 0;i < _GRAMSIZE; i++){
			LCD_data16(color);
		}
}


void write_dac(int channel, unsigned char voltage) {
    unsigned char b1=0, b2=0;
  
    b1 = (channel<<7);
    b1 = b1 | (0b111<<4);
    b1 = b1 | (voltage>>4);
    
    b2 = voltage<<4;
    
  CS = 0;                   // enable the ram
  spi_io(b1);             // ram write status
  spi_io(b2);             // sequential mode (mode = 0b01), hold disabled (hold = 0)
  CS = 1;                   // finish the command
}

void LCD_init() {
    int time = 0;
    LCD_command(CMD_SWRESET);//software reset
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/2) {} //delay(500);

	LCD_command(CMD_SLPOUT);//exit sleep
    time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_PIXFMT);//Set Color Format 16bit
	LCD_data(0x05);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_GAMMASET);//default gamma curve 3
	LCD_data(0x04);//0x04
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_GAMRSEL);//Enable Gamma adj
	LCD_data(0x01);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_NORML);

	LCD_command(CMD_DFUNCTR);
	LCD_data(0b11111111);
	LCD_data(0b00000110);

    int i = 0;
	LCD_command(CMD_PGAMMAC);//Positive Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(pGammaSet[i]);
	}

	LCD_command(CMD_NGAMMAC);//Negative Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(nGammaSet[i]);
	}

	LCD_command(CMD_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
	LCD_data(0x08);//0x0C//0x08
	LCD_data(0x02);//0x14//0x08
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_DINVCTR);//display inversion
	LCD_data(0x07);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
	LCD_data(0x0A);//4.30 - 0x0A
	LCD_data(0x02);//0x05
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL
	LCD_data(0x02);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML
	LCD_data(0x50);//0x50
	LCD_data(99);//0x5b
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMOFFS);
	LCD_data(0);//0x40
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_CLMADRS);//Set Column Address
	LCD_data16(0x00);
    LCD_data16(_GRAMWIDTH);

	LCD_command(CMD_PGEADRS);//Set Page Address
	LCD_data16(0x00);
    LCD_data16(_GRAMHEIGH);

	LCD_command(CMD_VSCLLDEF);
	LCD_data16(0); // __OFFSET
	LCD_data16(_GRAMHEIGH); // _GRAMHEIGH - __OFFSET
	LCD_data16(0);

	LCD_command(CMD_MADCTL); // rotation
    LCD_data(0b00001000); // bit 3 0 for RGB, 1 for GBR, rotation: 0b00001000, 0b01101000, 0b11001000, 0b10101000

	LCD_command(CMD_DISPON);//display ON
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_RAMWR);//Memory Write
}