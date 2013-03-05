#define BMP085_ADDR	        	0x77
#define BMP085_CTRL_REG                 0xF4
#define BMP085_TEMP_REG                 0x2E
#define BMP085_PRESSURE_REG             0x34		// oss =0
//#define BMP085_PRESSURE_REG_OSS1 0x74	// oss =1, longer delays needed 7.5ms
//#define BMP085_PRESSURE_REG_OSS2 0xB4 	// oss =2, 13.5ms
//#define BMP085_PRESSURE_REG_OSS3 0xF4 	// oss =3, 25.5ms
#define BMP085_MSB_REG	                0xF6
#define BMP085_LSB_REG	                0xF7
#define BMP085_CONV_REG_XLSB            0xF8

#define BMP085_ADDR        	0x77
#define HMC5883_ADDR		0x1E
#define L3G4200D_ADDR		0x69
#define ADXL345_ADDR		0x53


static const unsigned char init_hmc5883[]={0x02,0x00};
static const unsigned char init_l3g4200d_1[]={0x20,0x0F};
static const unsigned char init_l3g4200d_2[]={0x24,0x02};
static const unsigned char init_adxl345_1[]={0x31,0x01};
static const unsigned char init_adxl345_2[]={0x2D,0x08};

void init_sensors();
void get_hmc5883();
void get_l3g4200d();
void get_adxl345();

extern unsigned char RXbuffer[6];
extern unsigned char TXbuffer[6];

void bmp085_cal();
int read_ut();
long read_up();
int temperatura(void);
unsigned long presion(void);


   