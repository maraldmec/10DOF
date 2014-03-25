#include <p33FJ64MC202.h>


void initSPI()
{
	RPOR2=0x0700;						//Portb5 is Dataout rp5 0111
	RPINR20bits.SDI1R=0b00110;			//Portb6 is Data in rp6 0110
	RPINR20bits.SCK1R=0b00111;			//Portb4 is clock	rp7 0111
	//RPINR21bits.SS1R=0b01011;			//PIN19 PORTB11 is SS	rp11 1011

	SPI1BUF=0x00;						//Clear SPI Buffer
	SPI1STAT=0x00;
	SPI1STATbits.SPITBF=0;

	IFS0bits.SPI1IF=0;					//Clear int flag
	IEC0bits.SPI1IE=0;					//Disable int

	SPI1CON1bits.DISSCK=0;				//internal clock 
	SPI1CON1bits.DISSDO=0;				//SDO is active
	SPI1CON1bits.MODE16=1;				//Worde 16 bits wide communication
	SPI1CON1bits.SMP=0;					//Sample phase shift is not active. Data is sampled in the middel of the output.
	SPI1CON1bits.SSEN=0;				//No SS
	SPI1CON1bits.CKE=0;					//New data on clk idle  to active
	SPI1CON1bits.CKP=1;					//Clock is idle low
	SPI1CON1bits.MSTEN=0;				//Slave mode

	SPI1STATbits.SPIROV=0;				//Clear overflow bit
	IPC2bits.SPI1IP=0x02;				//Priority level is 2
	IFS0bits.SPI1IF=0;					//Clear int flag
	IEC0bits.SPI1IE=1;					//Enable int
	SPI1STATbits.SPIEN=1; 				//Put on SPI module
}

int readSPI()
{
	while (SPI1STATbits.SPIRBF==0)
    {
		;
    }			
    return (SPI1BUF);
}


int writeSPI(signed int spi_data)
{
	int dummy;
	dummy=SPI1BUF;
   while (SPI1STATbits.SPITBF==1)
   {
   }
	SPI1BUF=spi_data;	
	return 1;
}
