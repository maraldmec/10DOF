#include <p33FJ64MC202.h>

void initHardI2C()
{
	I2C1BRG=50;  //80mhz 1000khz=33 400khz=93, 100khz=393// 
    I2C1CONbits.I2CEN=1;				//I2c on.
    I2C1CONbits.SCLREL=0;				//Clock stretch on
    I2C1CONbits.A10M=0;					//7 bit adress
    I2C1CONbits.DISSLW=1;				//No slew rate
    I2C1CONbits.SMEN=0;
    I2C1CONbits.GCEN=0;
    I2C1CONbits.ACKDT=0; 
    I2C1CONbits.ACKEN=0;
    I2C1CONbits.RCEN=0;
    I2C1CONbits.PEN=0;
    I2C1CONbits.RSEN=0;
    I2C1CONbits.SEN=0;
}

void waitForIdleHardI2C(void)
{
    while ( IFS1bits.MI2C1IF==0 )
    {
    }
    IFS1bits.MI2C1IF=0;
}

void repStartHardI2C(void)
{
    I2C1CONbits.RSEN=1;
    waitForIdleHardI2C();
}

void startHardI2C(void)
{
    I2C1CONbits.SEN=1;
    waitForIdleHardI2C();
}

void stopHardI2C(void)
{
    I2C1CONbits.PEN=1;
    waitForIdleHardI2C();
}

unsigned char readHardI2C( void )
{
    I2C1CONbits.RCEN=1;
    waitForIdleHardI2C();
    return ( I2C1RCV );
}


unsigned char writeHardI2C( unsigned char i2cWD )
{
    while ( I2C1STATbits.TRSTAT==1 )
    {
    }
    I2C1TRN=i2cWD;
    waitForIdleHardI2C();
    return (!I2C1STATbits.ACKSTAT);
}

void ackHardI2C(void)
{
    I2C1CONbits.ACKDT=0;
    I2C1CONbits.ACKEN=1;
    waitForIdleHardI2C();
}

void nackHardI2C(void)
{
    I2C1CONbits.ACKDT=1;
    I2C1CONbits.ACKEN=1;
    waitForIdleHardI2C();
}
