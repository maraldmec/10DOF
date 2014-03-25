/*
 *  Name            :   main.c
 *  Function		:   10DOF
 *  Author          :   Marald Otten
 *  Date            :   July, 2011
 */

#define _LEGACY_HEADERS 
#include <p33fj64mc202.h>
#include <math.h>
#include <libpic30.h>
#include <delay.h>
#include <UART.h>
#include <stdio.h>
#include <stdlib.h>
#include "i2c.h"
#include "SPI.h"
#include "def.h"

_FOSCSEL(FNOSC_PRIPLL)
_FOSC(POSCMD_HS)
_FPOR(FPWRT_PWR128 )
_FICD(ICS_PGD1 & JTAGEN_OFF)
_FWDT(FWDTEN_OFF)

//offsets
float gyro_offset[3]={0.85,-0.53,-1.08};
int mag_off[3]={-28,-15,56};

//Pressure sensor
signed int AC1=0, AC2=0, AC3=0, B1=0, B2=0, MB=0, MC=0, MD=0;
unsigned int AC4=0, AC5=0, AC6=0;
signed long x1=0, x2=0, b5=0;
long p=0;
float lpAlt[15]={0};
float lpMult[15]={0.15,0.15,0.1,0.1,0.1,0.1,0.05,0.05,0.05,0.05,0.05,0.025,0.025,0.0,0.0};
float altitude=0;
float height=0;
signed long temperature=0;

//Results
float angle[3]={0};
float deg[3]={0};
float acc[3]={0};
float mag[3]={0};
float gyro[3]={0};

//Filter
float Q_angle=0.001,Q_gyro=0.003,R_angle=0.03;
float P_00[3]={0,0,0},P_01[3]={0,0,0},P_10[3]={0,0,0},P_11[3]={0,0,0},bias[3]={0,0,0};
float dt,y[3],S[3],K_0[3],K_1[3];			

//Others
float dt=0.002;
float vector;

int main()
{
///////////////////////////////////////////////////////////////////////////
////							Init								///////
///////////////////////////////////////////////////////////////////////////	

	initPic();
	LED1=1;
	DREADY=0;
	initSer();
	initHardI2C();
	delay50ms;
	
	initBMP085();
	initITG3200();
	initBMA180();
	initHMC5883();

	delay50ms;
	initINT();
	//initSPI();
	LED1=0;

///////////////////////////////////////////////////////////////////////////
////							MainLoop							///////
///////////////////////////////////////////////////////////////////////////
	int loop=0;
	while(1)
	{
		//get all data																				
	 	getITG3200();
		dt=getDt();
		getBMA180();
		if(loop==25||loop==50){
			getHMC5883();
			if(loop==25)
			{
				getTemp();
				BMP085Write(0xF4, 0x34);								//Request press
			}
			if(loop==50)
			{
				getAltitude();											//Calc height
				lpAltitude();
				BMP085Write(0xF4, 0x2E);
				loop=0;
				LED1=LED1^1;
				debug();
			}
		}
		accAngle();
		filter();
		DREADY=1;
		loop++;
	}   
}
///////////////////////////////////////////////////////////////////////////
////							Init								///////
///////////////////////////////////////////////////////////////////////////

void initPic()
{	
	//Init clock
    CLKDIVbits.PLLPRE=1;    									//12/3=4
    PLLFBD=48; 													//4*(48+2)=200
    CLKDIVbits.PLLPOST=0;   									//160/2=100 100/2=50

	//Init ports
	TRISA=0x0000;                   							//All output  											//All output, except for I2C and RX rp6
	TRISB=0b0000101111000000;      											//All output, except for I2C and RX rp6
	LATA=0x0000;                   								//Make all ports low
	LATB=0x0000;
}

void initINT()
{
	//Set up timer first
	T1CON=0b0000000000110000;										//Prescaler on 1:256, source is FCY. 1=40mhz/256=156250  max=0.41 seconds.
	T1CONbits.TON=0;												//Timer disabled
	TMR1=0;															//Clear Timer
	PR1=0xFFFF;														//Let the counter count untill max
	IPC0bits.T1IP=0x01;												//Low priority
	IFS0bits.T1IF=0;												//Clear tmr 1 int;
	IEC0bits.T1IE=1;												//Activate int 1
}

void initSer()
{
	RPINR18bits.U1RXR=0b00110;									//Portb6 is rx rp6
	RPOR2=0x0300;												//Portb5 is Tx rp5
	U1MODEbits.UARTEN=1;										//Uart is on
    U1MODEbits.RTSMD=1;											//Simplex mode
    U1MODEbits.UEN=0b00;										//Only Rx en Tx
    U1MODEbits.URXINV=0;										//Not inverted, idle high
    U1MODEbits.BRGH=0;											//Low speed
    U1MODEbits.PDSEL=0b00;										//8 bit, no parity
    U1MODEbits.STSEL=0;											//1 stopbit
    U1BRG=26;//270												//Baudrate = 9600; 40mhz/(16*19200)-1=146.9
    U1STAbits.UTXISEL1=0;										//Int when data is written to Tx
    U1STAbits.UTXISEL0=0;
    U1STAbits.UTXINV=0;											//Not inverted, idle 1
    U1STAbits.UTXEN=1;											//Set tx on.
   	U1STAbits.URXISEL=0b00;										//Interrupt when character is received
    U1STAbits.ADDEN=0; 											//No adress
    U1STAbits.OERR=0;											//Reset buffer overflow.

}

void debug()
{
	printf("X%i Y%i Z%i A%i T%iN",((short)(angle[1]*10)),((short)(angle[0]*10)),((short)(angle[2]*10)), ((short)(altitude*100)),((short)(temperature*100)));
	//printf("DT:%f",dt);
}
///////////////////////////////////////////////////////////////////////////
////							HMC5883								///////
///////////////////////////////////////////////////////////////////////////
void initHMC5883()
{
	writeHMC5883(0x01, 0b01100000);	
	writeHMC5883(0x02, 0x00);
	writeHMC5883(0x00, 0b01011000);
	delay50ms;
}

void writeHMC5883(unsigned char address, unsigned char data)
{
    startHardI2C();
    writeHardI2C(HMC5883_w);									//Write 0x3C
    writeHardI2C(address);										//Write register address
    writeHardI2C(data);											//Write value
    stopHardI2C();
}

void getHMC5883()
{	
	short int x,y,z;
	float n,fax,fay,hx,hy;

	startHardI2C();
	writeHardI2C(HMC5883_w);									
	writeHardI2C(0x03);	
	stopHardI2C();
	repStartHardI2C();
	writeHardI2C(HMC5883_r);									//Write 0x3D									
    x=((readHardI2C())<<8);										
    ackHardI2C();
    x+=(readHardI2C());
    ackHardI2C();
    z=((readHardI2C())<<8);
    ackHardI2C();
    z+=(readHardI2C());
    ackHardI2C();
    y=((readHardI2C())<<8);
    ackHardI2C();
    y+=(readHardI2C());
    nackHardI2C();
    stopHardI2C();

	mag[0]=((float)(x))-mag_off[0];
	mag[1]=((float)(y))-mag_off[1];
	mag[2]=((float)(z))-mag_off[2];
	n=sqrtf(powf(mag[0], 2)+powf(mag[1], 2)+powf(mag[2], 2));

	mag[0]/=n;
	mag[1]/=n;
	mag[2]/=n;
	
	fax=(angle[0])/PIDEG;
	fay=(angle[1])/PIDEG;

  	hx=mag[0]*cosf(fax)+mag[2]*sinf(fax);
  	hy=mag[0]*sinf(fay)*sinf(fax)+mag[1]*cosf(fay)-mag[2]*sinf(fay)*cosf(fax);

	deg[2]=atan2f(hy,hx)*PIDEG+HEADOFF;
	if(deg[2]<=-180)
		deg[2]=deg[2]+360;
}

///////////////////////////////////////////////////////////////////////////
////							ITG3200								///////
///////////////////////////////////////////////////////////////////////////
void initITG3200()
{
   	writeITG3200(0x3E, 0x80);									//Reset
	delay50ms;								
   	writeITG3200(0x15, 0x00);									//Sample time 8ms
   	writeITG3200(0x16, 0x19);									//DLPF_CFG = 1, FS_SEL = 3 BW=188Hz
   	writeITG3200(0x17, 0x00);									//No int
   	writeITG3200(0x3E, 0x01); 									//Clock X gyro
	delay50ms;
}

void writeITG3200(unsigned char address, unsigned char data)
{
    startHardI2C();
    writeHardI2C(ITG3200_w);									//Write 0xD0
    writeHardI2C(address);										//Write register address
    writeHardI2C(data);											//Write value
    stopHardI2C();
}

void getITG3200()
{ 
	int x,y,z;
    startHardI2C();
    writeHardI2C(ITG3200_w);									//Write adress gyro
    writeHardI2C(0x1D);											//Write register address
    
    repStartHardI2C();
    writeHardI2C(ITG3200_r);									//write 0xD1, read adress
  
    y=((readHardI2C())<<8);										//X gyro is Y acc and visa-versa
    ackHardI2C();
    y=y+(readHardI2C());
    ackHardI2C();
    x=((readHardI2C())<<8);
    ackHardI2C();
    x=x+(readHardI2C());
    ackHardI2C();
    z=((readHardI2C())<<8);
    ackHardI2C();
    z=z+(readHardI2C());
    nackHardI2C();
    stopHardI2C();
    
    gyro[0]=(float)(x/GYROSHIFT)-gyro_offset[0];
    gyro[1]=(float)(y/GYROSHIFT)-gyro_offset[1];
	gyro[2]=(float)(z/GYROSHIFT)-gyro_offset[2];
}

///////////////////////////////////////////////////////////////////////////
////							BMA180								///////
///////////////////////////////////////////////////////////////////////////

void initBMA180()
{
	int temp;
	temp = getRegBMA180(0x0D);										//Register with ee_wprintf("\nTemp: %i", temp);
	temp |= 0b00010000;
	writeBMA180(0x0D, temp); 										//Set ee_w = 1, so we can write
	
	temp = getRegBMA180(0x20);
	temp &= 0x0F;			
	writeBMA180(0x20, temp);										//Set band pass filter to 10 hz
	
	temp = getRegBMA180(0x35);
	temp &= 0b11110001;
	temp |= 0b000000100;  									
	writeBMA180(0x35, temp);										//Set range to 2g = 0.25mg/bit 4096 standard
	delay50ms;
}

void writeBMA180(unsigned char address, unsigned char data)
{
    startHardI2C();
    writeHardI2C(BMA180_w);										// write adress
    writeHardI2C(address);										// write register address
    writeHardI2C(data);
    stopHardI2C();
}

unsigned char getRegBMA180(unsigned char address) 
{
	startHardI2C();
    writeHardI2C(BMA180_w);										// write adress
    writeHardI2C(address);										// write register address
    repStartHardI2C();
    writeHardI2C(BMA180_r);										//Tell we want data
    unsigned char data = readHardI2C();							// Get byte from i2c
    nackHardI2C();
    stopHardI2C();
    return data;
}

void getBMA180()
{
	unsigned char Ym, Yl, Xl, Xm, Zl, Zm;
	int temp;
	float accRaw[3];
	
	startHardI2C();
	writeHardI2C(BMA180_w);	
	writeHardI2C(0x02);											//Write register address for acc data
	    
	repStartHardI2C();
	writeHardI2C(BMA180_r);
	   
	Yl = readHardI2C();											// Get YLSB
	ackHardI2C();
	Ym = readHardI2C();											// Get MLSB
	ackHardI2C();
	    
	Xl = readHardI2C();	
	ackHardI2C();
	Xm = readHardI2C();	
	ackHardI2C();
	  
	Zl = readHardI2C();	
	ackHardI2C();
	Zm = readHardI2C();	
	nackHardI2C();
	stopHardI2C();
			
	temp =  Xm << 8;
	temp |= Xl;
	temp = temp >> 2;					
	if((temp>>13)==1)
		accRaw[0] = temp|0xE000;
	else
		accRaw[0] = temp;
	
	temp =  Ym << 8;
	temp |= Yl;
	temp = temp >> 2;											//Last 2 lsb are garbage.
	if((temp>>13)==1)
		accRaw[1] = temp|0xE000;
	else
		accRaw[1]=temp;	

	temp =  Zm << 8;
	temp |= Zl;
	temp = temp >> 2;											
    if((temp>>13)==1)
		accRaw[2] = temp|0xE000;
	else
		accRaw[2] = temp;
        
    acc[0] = accRaw[0]/4096;
	acc[1] = accRaw[1]/4096;
	acc[2] = accRaw[2]/4096;
}


///////////////////////////////////////////////////////////////////////////
////							BMP085								///////
///////////////////////////////////////////////////////////////////////////

void initBMP085(){
	AC1=((BMP085Read(0xAA)<<8)|(BMP085Read(0xAB)));
    AC2=((BMP085Read(0xAC)<<8)|(BMP085Read(0xAD)));
    AC3=((BMP085Read(0xAE)<<8)|(BMP085Read(0xAF)));
    AC4=((BMP085Read(0xB0)<<8)|(BMP085Read(0xB1)));
    AC5=((BMP085Read(0xB2)<<8)|(BMP085Read(0xB3)));
    AC6=((BMP085Read(0xB4)<<8)|(BMP085Read(0xB5)));
    B1=((BMP085Read(0xB6)<<8)|(BMP085Read(0xB7)));
    B2=((BMP085Read(0xB8)<<8)|(BMP085Read(0xB9)));
    MB=((BMP085Read(0xBA)<<8)|(BMP085Read(0xBB)));
    MC=((BMP085Read(0xBC)<<8)|(BMP085Read(0xBD)));
    MD=((BMP085Read(0xBE)<<8)|(BMP085Read(0xBF)));
}

unsigned char BMP085Read(unsigned char address)
{
    unsigned char data;
    startHardI2C();
    writeHardI2C(BMP085_w);	
    writeHardI2C(address);										// write register address
    repStartHardI2C();
    writeHardI2C(BMP085_r);
    data = readHardI2C();										// Get MSB result
    nackHardI2C();
    stopHardI2C();
    return data;
}

void BMP085Write(unsigned char address, unsigned char data)
{
    startHardI2C();
    writeHardI2C(BMP085_w);	
    writeHardI2C(address);										
    writeHardI2C(data);
    stopHardI2C();
}

void getTemp()
{
 	signed long  ut=0; 
	ut =((BMP085Read(0xF6)<<8)+(BMP085Read(0xF7)));				//Get temp
	ut &= 0x0000FFFF;

 	x1 = (((long)ut - AC6)* AC5) >> 15;
  	x2 = ((long)MC << 11)/(x1 + MD);
  	b5 = x1 + x2;
	temperature = (b5 + 8) >> 4;
}


void getAltitude()
{
    long  up=0;
    long  b6, x3, b3;
    unsigned long b4, b7;
	float t;
    
    up=((BMP085Read(0xF6)<<8)+(BMP085Read(0xF7)));
	up &= 0x0000FFFF;
    
    b6 = b5 - 4000;
    x1 = (B2 * (b6 * b6 >> 12)) >> 11;
    x2 = AC2 * b6 >> 11;
    x3 = x1 + x2;
    b3 = (((unsigned long)AC1 * 4 + x3) + 2)/4;
    x1 = AC3 * b6 >> 13;
    x2 = (B1 * (b6 * b6 >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    b4 = (AC4 * (unsigned long) (x3 + 32768)) >> 15;
    b7 = ((unsigned long)up - b3) * (50000 >> 0);
    p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    p = p + ((x1 + x2 + 3791) >> 4);

	t=(float)p/PRESSOFF;
	height = (44330 * (1 - powf(t, 0.190295)));	// pressure at sealevel=101325,	0.190295=1/5.255
}

///////////////////////////////////////////////////////////////////////////
////							Filters							///////
///////////////////////////////////////////////////////////////////////////

void accAngle()
{
	vector = sqrtf(powf(acc[0],2)+powf(acc[1],2)+powf(acc[2],2));//Normalizing the angles here, first i calculate the lenght of the netto vector.

	acc[0] = acc[0]/vector;										//Then i calculate the relative lenght of each vector.
	acc[1] = acc[1]/vector;  
	acc[2] = acc[2]/vector; 
	
	deg[1]=((atan2f(acc[1],acc[2]))*PIDEG);

	if(deg[1]>90)
		deg[0]=((atan2f(acc[0],acc[1]))*PIDEG);
	else if(deg[1]<-90)
		deg[0]=((atan2f(acc[0],-acc[2]))*PIDEG);
	else 
		deg[0]=((atan2f(acc[0],acc[2]))*PIDEG);
}

void filter()
{
int k;
	for(k=0;k<3;k++)
	{   
		if(((vector<3&&vector>0.3)&&(k==0||k==1))||((angle[0]<85&&angle[0]>-85&&angle[1]<85&&angle[1]>-85)&&k==2))									//Check wether the accelerometer values are reliable or not.
		{                              
    		angle[k]+=dt*(gyro[k]-bias[k]);
    		P_00[k]+=-dt*(P_10[k]+P_01[k])+Q_angle*dt;
    		P_01[k]+=-dt*P_11[k];
    		P_10[k]+=-dt*P_11[k];
    		P_11[k]+=+Q_gyro*dt;
    
    		y[k]=deg[k]-angle[k];
    		S[k]=P_00[k]+R_angle;
   	 		K_0[k]=P_00[k]/S[k];
    		K_1[k]=P_10[k]/S[k];
    
    		angle[k]+=K_0[k]*y[k];
    		bias[k]+=K_1[k]*y[k];
    		P_00[k]-=K_0[k]*P_00[k];
    		P_01[k]-=K_0[k]*P_01[k];
    		P_10[k]-=K_1[k]*P_00[k];
    		P_11[k]-=K_1[k]*P_01[k];
		}
		else
		{
			angle[k]+=gyro[k]*dt;
		}
	}
}

void lpAltitude()
{
	int k=0;
	float t=0;
	for(k=13; k>=0; k--){
		lpAlt[k+1]=lpAlt[k];
		t+=lpAlt[k+1]*lpMult[k+1];
	} 
	lpAlt[0]=height;
	altitude = (t + (lpAlt[0]*lpMult[0]));
}


///////////////////////////////////////////////////////////////////////////
////							DT									///////
///////////////////////////////////////////////////////////////////////////
float getDt()
{
	float t;
	T1CONbits.TON=0;
	t=(float)TMR1;
	t/=195310;												//40mhz/256=156250 50mhz=195310
	TMR1=0;
	T1CONbits.TON=1;
	return t;
}

///////////////////////////////////////////////////////////////////////////
////							SPI Int								///////
///////////////////////////////////////////////////////////////////////////
void __attribute__((interrupt, auto_psv))  _SPI1Interrupt(void)
{
	int spi_out=0, timeout=0;
	unsigned int spi_in;
	IEC0bits.SPI1IE=0;												//Disable SPI Int
	IFS0bits.SPI1IF=0;
	
	SPI1STATbits.SPIROV=0;											//Clear overflow bit

	while(SPI1STATbits.SPIRBF==0)
	    {
			timeout++;
			if(timeout>400)
				break;
	    }
	DREADY=0;	
	
	spi_in=SPI1BUF;
	switch(spi_in)
	{
		case XANGLE: 
			LED1=LED1^1;
			spi_out=((signed int)(angle[1] * SHIFT));
			break;
		case YANGLE: 
			LED1=LED1^1;
			spi_out=((signed int)(angle[0] * SHIFT));
			break;
		case ZANGLE:
			LED1=LED1^1; 
			spi_out=((signed int)(angle[2] * SHIFT));
			break;

		case DT:
			LED1=LED1^1; 
			spi_out=((unsigned int)(dt*1000000));
			break;
		case ALTITUDE:
			LED1=LED1^1; 
			spi_out=((signed int)(altitude * SHIFT));
			break;
		case TEMP:
			LED1=LED1^1; 
			spi_out=((signed int)(temperature * SHIFT));
			break;
	

		case XACC:
			LED1=LED1^1;
			spi_out=((signed int)(acc[1]*LSHIFT));
			break;
		case YACC: 
			LED1=LED1^1;
			spi_out=((signed int)(acc[0]*LSHIFT));
			break;
		case ZACC:
			LED1=LED1^1;
			spi_out=((signed int)(acc[2]*LSHIFT));
			break;
	
		case XGYRO:
			LED1=LED1^1; 
			spi_out=((signed int)(gyro[1]*GYROSHIFT));
			break;
		case YGYRO: 
			LED1=LED1^1; 
			spi_out=((signed int)(gyro[0]*GYROSHIFT));
			break;
		case ZGYRO:
			LED1=LED1^1; 
			spi_out=((signed int)(gyro[2]*GYROSHIFT));
			break;

		case XMAG:
			LED1=LED1^1;
			spi_out=((signed int)(mag[1]*LSHIFT));
			break;
		case YMAG: 
			LED1=LED1^1;
			spi_out=((signed int)(mag[0]*LSHIFT));
			break;
		case ZMAG:
			LED1=LED1^1;
			spi_out=((signed int)(mag[2]*LSHIFT));
			break;	
	
		case RESET:
			delay5ms;
			asm("reset");										
			break;
	}	

	int dummy;
	dummy=SPI1BUF;

	while (SPI1STATbits.SPITBF==1)
	   {
		timeout++;
			if(timeout>800)
				break;
	   }
	SPI1BUF=spi_out;	

	IFS0bits.SPI1IF=0;										//Clear Int flag
	IEC0bits.SPI1IE=1;										//Set int on.							
}

///////////////////////////////////////////////////////////////////////////
////							Error								///////
///////////////////////////////////////////////////////////////////////////

void _ISR _T1Interrupt(void)								//Timer int, cycle took more then 0.419s to complete
{
	IEC0bits.T1IE=0;										
	IFS0bits.T1IF=0;										
	asm("RESET");											
}

void _ISR _OscillatorFail(void)
{
    asm("RESET");
}

void _ISR _AddressError(void)
{
    asm("RESET");
}

void _ISR _StackError(void)
{
    asm("RESET");
}

void _ISR _MathError(void)
{
    asm("RESET");
}


