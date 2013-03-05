/*
 * Funciones comunes para quitar mierda del main...
 */

// include the libraries  //

#include "msp430fr5739.h"
#include "comun.h"
#include "lcd.h"
   
 static const unsigned long dv[] = {             // Base 10 digit weights
      10000000,                                 // 8 digit maximum count
       1000000,                                 //
        100000,                                 //
         10000,                                 //
          1000,                                 //
           100,                                 //
            10,                                 //
             1,                                 //
             0                                  //
};

void SystemInit(void)
{
  // Startup clock system in max. DCO setting ~8MHz
  // This value is closer to 10MHz on untrimmed parts  
  CSCTL0_H = 0xA5;                          // Unlock register
  CSCTL1 |= DCOFSEL0 + DCOFSEL1;            // Set max. DCO setting
  CSCTL2 = SELA_1 + SELS_3 + SELM_3;        // set ACLK = vlo; MCLK = DCO
  CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;        // set all dividers 
  CSCTL0_H = 0x01;                       // Lock Register*/

  // Configure Pins for I2C
  P1SEL1 |= BIT6 + BIT7;                       // Pin init
  
  // Enable switches
  // P4.0 and P4.1 are configured as switches
  // Port 4 has only two pins    
  P4OUT |= BIT0 +BIT1;                      // Configure pullup resistor  
  P4DIR &= ~(BIT0 + BIT1);                  // Direction = input
  P4REN |= BIT0 + BIT1;                     // Enable pullup resistor
  P4IES &= ~(BIT0+BIT1);                    // P4.0 Lo/Hi edge interrupt  
  P4IE = BIT0+BIT1;                         // P4.0 interrupt enabled
  P4IFG = 0;        
  
  // Turn off temp. 
  REFCTL0 |= REFTCOFF; 
  REFCTL0 &= ~REFON;  
 
  // Terminate Unused GPIOs
  // P1.0 - P1.6 is unused
  // P1OUT &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT5 + BIT6 + BIT7);   
  // P1DIR &= ~(BIT0 + BIT1 + BIT2 + BIT3 + BIT5 + BIT6 + BIT7); 
  // P1REN |= (BIT0 + BIT1 + BIT2 + BIT3 + BIT5 + BIT6 + BIT7);   

  // P1.4 is used as input from NTC voltage divider
  // Set it to output low
  P1OUT &= ~BIT4;      
  P1DIR |= BIT4; 

  // P2.7 is used to power the voltage divider for the NTC thermistor
  P2OUT &= ~BIT7;     
  P2DIR |= BIT7; 

  // P3.0,P3.1 and P3.2 are accelerometer inputs
  P3OUT &= ~(BIT0 + BIT1 + BIT2);   
  P3DIR &= ~(BIT0 + BIT1 + BIT2); 
  P3REN |= BIT0 + BIT1 + BIT2;

  // PJ.0,1,2,3 are used as LEDs
  // crystal pins for XT1 are unused
  PJOUT &= ~(BIT4+BIT5);  
  PJDIR &= ~(BIT4+BIT5);
  PJREN |= BIT4 + BIT5;  

  // Enable LEDs
  P3OUT &= ~(BIT6+BIT7+BIT5+BIT4);
  P3DIR |= BIT6+BIT7+BIT5+BIT4;
  PJOUT &= ~(BIT0+BIT1+BIT2+BIT3);
  PJDIR |= BIT0 +BIT1+BIT2+BIT3;  
  
  // Para el LED del LCD
  P1OUT &= ~BIT0;      
  P1DIR |= BIT0; 
}
   
void StartUpSequence(void) 
{
  unsigned char flag=4,up=1,counter = 0; 
  unsigned char LED_ArrayPJ[] = {0x01,0x02,0x04,0x08};
  unsigned char LED_ArrayP3[] = {0x80,0x40,0x20,0x10};

  while (counter <10) 
    {    
      counter++;
      PJOUT &= ~(BIT0 +BIT1+BIT2+BIT3);
      P3OUT &= ~(BIT4 +BIT5+BIT6+BIT7);    
      if(up)// from the outside - in
        {
          while(flag)
            {
              P3OUT = LED_ArrayP3[flag-1];
              PJOUT = LED_ArrayPJ[flag-1];
              LongDelay();               
              flag--;
            }
            up=0;
        }
      else
        {
          while(flag<4)
            {
              P3OUT = LED_ArrayP3[flag];
              PJOUT = LED_ArrayPJ[flag];
              LongDelay();        
              flag++;
            }
            up = 1;
          }
    } 
  
  // Exit Loop, Clear LEDs
  PJOUT &= ~(BIT0 +BIT1+BIT2+BIT3);
  P3OUT &= ~(BIT4 +BIT5+BIT6+BIT7);    
}

void LongDelay()
{
  __delay_cycles(250000);
  __no_operation();
}

void print_int(unsigned long n, unsigned char fila)
{
    const unsigned long *dp = dv;
    unsigned x = 0;
    unsigned c;
    unsigned long d;

    while(n < *dp) {                            // Skip leading zeros
        if(*dp == 100000 || *dp == 100) x += 2; // Space between 3 digit groups
        ++dp;                                   //
        lcd_pd10(10, x, fila);                     // Print space
        x += 10;                                //
    }                                           //
    if(n) {                                     // Non-zero
        do {                                    //
            d = *dp++;                          // Get digit value
            c = 0;                              //
            while(n >= d) ++c, n -= d;          // Divide
            if(d == 100000 || d == 100) x += 2; // Space between 3 digit groups
            lcd_pd10(c, x, fila);                  // Print digit
            x += 10;                            //
        } while(!(d & 1));                      // Until all digits done
    } else                                      //
        lcd_pd10(0, x - 10, fila);                 // Print zero
}

void escala ( int eje, unsigned char fila) {


	if (eje < 0 && eje > -100  )
		lcd_print("      ##      ",0,fila);
	if (eje < -200)
		lcd_print("     ###      ",0,fila);
	if (eje < -400)
		lcd_print("    ####      ",0,fila);
	if (eje < -800)
		lcd_print("   #####      ",0,fila);
	if (eje < -2000)
		lcd_print("  ######      ",0,fila);
	if (eje < -5000)
		lcd_print(" #######      ",0,fila);
	if (eje < -10000)
		lcd_print("########      ",0,fila);

	if (eje > 0 && eje < 100  )
		lcd_print("      ##      ",0,fila);
	if (eje > 200)
		lcd_print("      ###     ",0,fila);
	if (eje > 400)
		lcd_print("      ####    ",0,fila);
	if (eje > 800)
		lcd_print("      #####   ",0,fila);
	if (eje > 2000)
		lcd_print("      ######  ",0,fila);
	if (eje > 5000)
		lcd_print("      ####### ",0,fila);
	if (eje > 10000)
		lcd_print("      ########",0,fila);


}

void escala2 ( int eje, unsigned char fila) {


	if (eje < 0 && eje > -10  )
		lcd_print("      ##      ",0,fila);
	if (eje < -10)
		lcd_print("     ###      ",0,fila);
	if (eje < -30)
		lcd_print("    ####      ",0,fila);
	if (eje < -50)
		lcd_print("   #####      ",0,fila);
	if (eje < -70)
		lcd_print("  ######      ",0,fila);
	if (eje < -90)
		lcd_print(" #######      ",0,fila);
	if (eje < -110)
		lcd_print("########      ",0,fila);

	if (eje > 0 && eje < 10  )
		lcd_print("      ##      ",0,fila);
	if (eje > 10)
		lcd_print("      ###     ",0,fila);
	if (eje > 30)
		lcd_print("      ####    ",0,fila);
	if (eje > 50)
		lcd_print("      #####   ",0,fila);
	if (eje > 70)
		lcd_print("      ######  ",0,fila);
	if (eje > 90)
		lcd_print("      ####### ",0,fila);
	if (eje > 110)
		lcd_print("      ########",0,fila);


}