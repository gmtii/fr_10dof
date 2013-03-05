#include "msp430fr5739.h"
#include "10dof.h"
#include "i2cmaster.h"
#include "lcd.h"
#include "math.h"
#include "comun.h"

int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;

long ut;
long b5;

long up;
   
void bmp085_cal()
{
        ac1 = i2c_readX(BMP085_ADDR,0xAA,2);
        __delay_cycles(1000);    

        ac2 = i2c_readX(BMP085_ADDR,0xAC,2);
        __delay_cycles(1000);

        ac3 = i2c_readX(BMP085_ADDR,0xAE,2);
        __delay_cycles(1000);

        ac4 = i2c_readX(BMP085_ADDR,0xB0,2);
        __delay_cycles(1000);   

        ac5 = i2c_readX(BMP085_ADDR,0xB2,2);              // 25860
        __delay_cycles(1000);

        ac6 = i2c_readX(BMP085_ADDR,0xB4,2);              // 16142
        __delay_cycles(1000);

        b1 =  i2c_readX(BMP085_ADDR,0xB6,2);
        __delay_cycles(1000);

        b2 =  i2c_readX(BMP085_ADDR,0xB8,2);
        __delay_cycles(1000);

        mb =  i2c_readX(BMP085_ADDR,0xBA,2);
        __delay_cycles(1000);

        mc =  i2c_readX(BMP085_ADDR,0xBC,2);              // -11075       
        __delay_cycles(1000);

        md =  i2c_readX(BMP085_ADDR,0xBE,2);              // 2432
        __delay_cycles(1000);
}
  
  
int temperatura()
{
   long x1;
   long x2;
   
      ut= read_ut();

      x1 = ((long)ut - ac6) * ac5 >> 15;
      x2 = ((long)mc << 11) / (x1 + md);
      b5 = x1 + x2;

      return ((b5 + 8) >> 4);
}    

unsigned long presion(void)
{
  
      long x1;
      long x2;
      long x3;
      long b6Temp;
      long x1Temp;
      long b3;
      unsigned long b4;
      long b6;
      unsigned long b7;
      long p;
      long bmp_pres = 0;

        up = read_up();
        
        // calc true pressure
        b6 = b5 - 4000;
    
	//x1 = (b2 * (b6 * b6) >> 12) >> 11; // won't work this way on the value line LSR_23 error
	b6Temp = b6 * b6;
	x1Temp = b2 * b6Temp;
	x1Temp = x1Temp >> 12;
	x1 = x1Temp >> 11;
	
	x2 = (ac2 * b6) >> 11;
	x3 = x1 + x2;
	
	b3 = ((long)ac1 * 4 + x3 + 2) >> 2;  // ????  so many 'corrections' on the web this one works though
	
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;
	b7 = ((unsigned long)up - b3) * 50000;
	
	if (b7  < 0x80000000)
	{ p = (b7 * 2) / b4;}
	else {p = (b7 / b4) *2;}

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	bmp_pres = p + ((x1 + x2 + 3791) >> 4);
        
        return ( bmp_pres );
}

int read_ut()
{
  
      i2c_send8( BMP085_ADDR , BMP085_CTRL_REG, BMP085_TEMP_REG );

      __delay_cycles(500000);
      
      return i2c_readX(BMP085_ADDR,BMP085_MSB_REG,2);
}

long read_up()
{
  
      i2c_send8( BMP085_ADDR , BMP085_CTRL_REG, BMP085_PRESSURE_REG );
      
      __delay_cycles(500000);
      
      return i2c_readX(BMP085_ADDR,BMP085_MSB_REG,3);
}

void init_sensors()
{
     bmp085_cal();

     i2c_send8( HMC5883_ADDR , init_hmc5883[0], init_hmc5883[1] );

     i2c_send8( L3G4200D_ADDR , init_l3g4200d_1[0], init_l3g4200d_1[1] );
     i2c_send8( L3G4200D_ADDR , init_l3g4200d_2[0], init_l3g4200d_2[1] );

     i2c_send8( ADXL345_ADDR , init_adxl345_1[0], init_adxl345_1[1] );
     i2c_send8( ADXL345_ADDR , init_adxl345_2[0], init_adxl345_2[1] );
     
}

void get_hmc5883() {

	int x=0,y=0,z=0;
	double angulo;

	x = i2c_readX(HMC5883_ADDR,0x03,2);
 	z = i2c_readX(HMC5883_ADDR,0x05,2);
	y = i2c_readX(HMC5883_ADDR,0x07,2);

	angulo= atan2((double)y,(double)x);
        
        if(angulo < 0)
	    angulo += 2*3.14159265;

	angulo *= 180/3.14159265;

	lcd_print("HMC5883L:",0,0);

              if((angulo < 22.5) || (angulo > 337.5 ))
                  lcd_print("Sur       ",0,2);
              if((angulo > 22.5) && (angulo < 67.5 ))
                  lcd_print("Sur-Oeste ",0,2);
              if((angulo > 67.5) && (angulo < 112.5 ))
                  lcd_print("Oeste     ",0,2);
              if((angulo > 112.5) && (angulo < 157.5 ))
                  lcd_print("Nor-Oeste ",0,2);
              if((angulo > 157.5) && (angulo < 202.5 ))
                  lcd_print("Norte     ",0,2);
              if((angulo > 202.5) && (angulo < 247.5 ))
                  lcd_print("Nor-Este  ",0,2);
              if((angulo > 247.5) && (angulo < 292.5 ))
                  lcd_print("Este      ",0,2);
              if((angulo > 292.5) && (angulo < 337.5 ))
                  lcd_print("Sur-Este  ",0,2);

          print_int((int)angulo, 4);
}

void get_l3g4200d() {

	int x=0,y=0,z=0;
        
        x = (int) i2c_readX(L3G4200D_ADDR,0x29,1) << 8 | (int) i2c_readX(L3G4200D_ADDR,0x28,1);
	y = (int) i2c_readX(L3G4200D_ADDR,0x2B,1) << 8 | (int) i2c_readX(L3G4200D_ADDR,0x2A,1);
 	z = (int) i2c_readX(L3G4200D_ADDR,0x2D,1) << 8 | (int) i2c_readX(L3G4200D_ADDR,0x2C,1);

	lcd_print("L3G4200D X:",0,0);
	escala (x, 1);
	lcd_print("Eje Y:",0,2);
	escala (y, 3);
	lcd_print("Eje Z:",0,4);
	escala (z, 5);
}

void get_adxl345() {

	int x=0,y=0,z=0;

        x = (int) i2c_readX(ADXL345_ADDR,0x33,1) << 8 | (int) i2c_readX(ADXL345_ADDR,0x32,1);
	y = (int) i2c_readX(ADXL345_ADDR,0x35,1) << 8 | (int) i2c_readX(ADXL345_ADDR,0x34,1);
 	z = (int) i2c_readX(ADXL345_ADDR,0x37,1) << 8 | (int) i2c_readX(ADXL345_ADDR,0x36,1);

	lcd_print("ADXL345 X:",0,0);
	escala2(x, 1);

	lcd_print("Eje Y:",0,2);
	escala2(y, 3);

	lcd_print("Eje Z:",0,4);
	escala2(z, 5);
}
     
  