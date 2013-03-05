#include "msp430fr5739.h"
#include "Fraunchpad.h"
#include "lcd.h"
#include "comun.h"


volatile unsigned char ThreshRange[3]={0,0,0};
extern volatile unsigned int ADCResult;
extern volatile unsigned char modo;
unsigned int CalValue =0;
unsigned int ADCTemp =0;
unsigned char temp = 0;

/**********************************************************************//**
 * @brief  Take ADC Measurement
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void TakeADCMeas(void)
{  
  while (ADC10CTL1 & BUSY); 
  ADC10CTL0 |= ADC10ENC | ADC10SC ;       // Start conversion 
  __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
  __no_operation();                       // For debug only
}

/**********************************************************************//**
 * @brief  Initializes Accelerometer
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void SetupAccel(void)
{  
  //Setup  accelerometer
  // ~20KHz sampling
  //Configure GPIO
  ACC_PORT_SEL0 |= ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN;    //Enable A/D channel inputs
  ACC_PORT_SEL1 |= ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN;
  ACC_PORT_DIR &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
  ACC_PWR_PORT_DIR |= ACC_PWR_PIN;              //Enable ACC_POWER
  ACC_PWR_PORT_OUT |= ACC_PWR_PIN;

  // Allow the accelerometer to settle before sampling any data 
  __delay_cycles(200000);   
  
  //Single channel, once, 
  ADC10CTL0 &= ~ADC10ENC;                        // Ensure ENC is clear
  ADC10CTL0 = ADC10ON + ADC10SHT_5;  
  ADC10CTL1 = ADC10SHS_0 + ADC10SHP + ADC10CONSEQ_0 + ADC10SSEL_0;  
  ADC10CTL2 = ADC10RES;    
  ADC10MCTL0 = ADC10SREF_0 + ADC10INCH_12;
  ADC10IV = 0x00;                          // Clear all ADC12 channel int flags  
  ADC10IE |= ADC10IE0;
  
  // Setup Thresholds for relative difference in accelerometer measurements
  ThreshRange[0]=25;
  ThreshRange[1]=50;
  ThreshRange[2]=75;
}

/**********************************************************************//**
 * @brief  ShutDownAccel
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void ShutDownAccel(void)
{  
  ACC_PORT_SEL0 &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
  ACC_PORT_SEL1 &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
  ACC_PORT_DIR &= ~(ACC_X_PIN + ACC_Y_PIN + ACC_Z_PIN);
  ACC_PWR_PORT_DIR &= ~ACC_PWR_PIN;            
  ACC_PWR_PORT_OUT &= ~ACC_PWR_PIN;
  ADC10CTL0 &= ~(ADC10ENC + ADC10ON);
  ADC10IE &= ~ADC10IE0;
  ADC10IFG = 0;
}

/**********************************************************************//**
 * @brief  Transmit 7 bytes
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void serialstring( unsigned char *string)
{

  // Configure UART 0
  UCA0CTL1 |= UCSWRST; 
  UCA0CTL1 = UCSSEL_2;                      // Set SMCLK as UCLk 
  UCA0BR0 = 52 ;                              // 9600 baud
  // 8000000/(9600*16) - INT(8000000/(9600*16))=0.083
  UCA0BR1 = 0; 
  // UCBRFx = 1, UCBRSx = 0x49, UCOS16 = 1 (Refer User Guide)
  UCA0MCTLW = 0x4911 ;                      
                                            
  UCA0CTL1 &= ~UCSWRST;                     // release from reset
  while(*string != '\0')
  {
    while (!(UCA0IFG&UCTXIFG));             // USCI_A0 TX buffer ready?
    UCA0TXBUF = *string;
    string++;
  }	
}

/**********************************************************************//**
 * @brief  Disables Switches
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void DisableSwitches(void)
{
  // disable switches
  P4IFG = 0;                                // P4 IFG cleared    
  P4IE &= ~(BIT0+BIT1);                     // P4.0 interrupt disabled
  P4IFG = 0;                                // P4 IFG cleared  
}

/**********************************************************************//**
 * @brief  Enables Switches 
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
void EnableSwitches(void)

{
  P4IFG = 0;                                // P4 IFG cleared  
  P4IE = BIT0+BIT1;                         // P4.0 interrupt enabled
}


/**********************************************************************//**
 * @brief  Sets up the Timer A1 as debounce timer 
 * 
 * @param  delay: pass 0/1 to decide between 250 and 750 ms debounce time 
 *  
 * @return none
 *************************************************************************/
void StartDebounceTimer(unsigned char delay)

{  
  // default delay = 0
  // Debounce time = 1500* 1/8000 = ~200ms
  TA1CCTL0 = CCIE;                          // TACCR0 interrupt enabled
  if(delay)
  	TA1CCR0 = 0xFFFF;
  else
    TA1CCR0 = 1500;
  TA1CTL = TASSEL_1 + MC_1;                 // ACLK, up mode    
}

void adxl335(void)
{ 
          
  // variable initialization
    
  ADCTemp = 0;
  temp = 0;
  int WriteCounter;
  
  // One time setup and calibration
  SetupAccel();
  CalValue = CalibrateADC();
  
  while ( modo == 3 )
    {  
      // Take 1 ADC Sample
      
      TakeADCMeas();
      
      if (ADCResult >= CalValue)        
        {
          temp = DOWN;
          ADCTemp = ADCResult - CalValue;
        }
      else
        {
          temp = UP;
          ADCTemp = CalValue - ADCResult; 
        }                                       

            WriteCounter++;
          if(WriteCounter > 300)
            {              
              LEDSequence(ADCTemp,temp);              
              WriteCounter = 0;
              print_int((int)ADCTemp, 4);
            }
      
    }
    // end while() loop
    // turn off Accelerometer for low power
    ShutDownAccel();
}

/**********************************************************************//**
 * @brief  Calibrate thermistor or accelerometer
 * 
 * @param  none 
 *  
 * @return none
 *************************************************************************/
unsigned int CalibrateADC(void)
{
  unsigned char CalibCounter =0;
  unsigned int Value = 0;

  // Disable interrupts & user input during calibration
  DisableSwitches();            
  while(CalibCounter <50)
    {
      P3OUT ^= BIT4;
      CalibCounter++;
      while (ADC10CTL1 & BUSY); 
      ADC10CTL0 |= ADC10ENC | ADC10SC ;       // Start conversion 
      __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exit
      __no_operation(); 
      Value += ADCResult;
    }
    Value = Value/50;
    // Reenable switches after calibration
    EnableSwitches();
    return Value;
}

/**********************************************************************//**
 * @brief  LED Toggle Sequence
 * 
 * @param  
 * DiffValue Difference between calibrated and current measurement
 * temp Direction of difference (positive or negative)
 * @return none
 *************************************************************************/
void LEDSequence(unsigned int DiffValue, unsigned char temp)
{  
  // The same scale is used for cold & hot and tilt up/down
  // only the thresholds are different
  P3OUT |= BIT4;                            // Light up the middle LEDs
  PJOUT |= BIT3;

  
  if(DiffValue < ThreshRange[0])            // Very close to CAL value
    {
      P3OUT |= BIT4;                          
      PJOUT |= BIT3;
      PJOUT &= ~(BIT0+BIT1+BIT2);
      P3OUT &= ~(BIT7+BIT6+BIT5);
    }
  
  if ((DiffValue >=ThreshRange[0]) && (DiffValue < ThreshRange[1]))
    {
      // Light up one LED  	
      if(temp == UP)                        // Tilt up, temp up
        {
          PJOUT |= BIT2;
          PJOUT &= ~(BIT1+BIT0);
          P3OUT &= ~(BIT7+BIT6+BIT5);
        }
      else                                  // Tilt down, temp down
        {
          PJOUT &= ~(BIT0+BIT1+BIT2);
          P3OUT |= BIT5; 
          P3OUT &= ~(BIT6+BIT7);
        }
    }
  if ((DiffValue >= ThreshRange[1]) && (DiffValue < ThreshRange[2]))
    {
      // Light up two LEDs  	    
      if(temp == UP)                        // Tilt up 2, temp up 2
        {
          PJOUT |= BIT2 + BIT1;
          PJOUT &= ~(BIT0);
          P3OUT &= ~(BIT7+BIT6+BIT5);
        }
      else                                  // Tilt down 2, temp down 2
        {
          PJOUT &= ~(BIT2+BIT1+BIT0);
          P3OUT |= BIT5 + BIT6; 
          P3OUT &= ~(BIT7);
        }
    }
  if (DiffValue > ThreshRange[2])
    {
      // Light up three LEDs  	        	
      if(temp == UP)                        // Tilt up 3, temp up 3
        {
          PJOUT |= BIT2 + BIT1 + BIT0;
          P3OUT &= ~(BIT7+BIT6+BIT5);
        }
      else                                  // Tilt down 3, temp down 3
        {
          P3OUT |= BIT5+BIT6+BIT7;
          PJOUT &= ~(BIT2+BIT1+BIT0);
        }
    }
}


