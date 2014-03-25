//All defines used in the code

//Addresses chips
#define BMA180_r 0x81 				
#define BMA180_w 0x80				
#define ITG3200_w 0xD0
#define ITG3200_r 0xD1
#define HMC5883_w 0x3C
#define HMC5883_r 0x3D
#define BMP085_w 0xEE
#define BMP085_r 0xEF

//Ports connected to the leds
#define LED1 LATBbits.LATB10
#define DREADY LATBbits.LATB13

//Input codes SPI
#define XANGLE	0x0040
#define YANGLE 0x0041
#define ZANGLE 0x0042
#define DT 0x0050
#define TEMP 0x0051
#define ALTITUDE 0x0052
#define XGYRO 0x0060
#define YGYRO 0x0061
#define ZGYRO 0x0062
#define XACC 0x0070
#define YACC 0x0071
#define ZACC 0x0072
#define XMAG 0x0080
#define YMAG 0x0081
#define ZMAG 0x0082

#define RESET 0x00F4

//Other defines
#define PIDEG 57.29577951		//180/pi
#define SHIFT 128				//2^7 SHIFT used for data
#define LSHIFT 4096
#define GYROSHIFT 14.375
#define HEADOFF -74

#define PRESSOFF 101325			//Airpressure on sealevel

#define delay5ms 	__delay32(200000)
#define delay50ms 	__delay32(2000000)

//Init
void initPic();
void initITG3200();
void initBMA180();
void initHMC5883();
void initINT();
void initHardI2C();
void initBMP085();

//I2C hardware functions
void nackHardI2C(void);
void waitForIdleHardI2C(void);
void repStartHardI2C(void);
void startHardI2C(void);
void stopHardI2C(void);
unsigned char readHardI2C( void );
void ackHardI2C(void);
unsigned char writeHardI2C( unsigned char i2cWD);

//I2C software functions
int writeSPI(int data);
int readSPI();
void initSPI();
void initSer();

//Get data functions
void getITG3200();
void writeITG3200(unsigned char address, unsigned char data);
void getBMA180();
void writeBMA180(unsigned char address, unsigned char data);
unsigned char getRegBMA180(unsigned char address);
void writeHMC5883(unsigned char address, unsigned char data);
void getHMC5883();

//pressure sensor
unsigned char BMP085Read(unsigned char address);
void BMP085Write(unsigned char address, unsigned char data);
void getAltitude();
void lpAltitude();
void getTemp();

//Some other
void debug();
float getDt();

//Math functions
void accAngle();
void filter();

