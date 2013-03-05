#include "msp430fr5739.h"
#include "i2cmaster.h"



void i2c_send8(unsigned char i2c_address , unsigned char address, unsigned char byte1 )
{
     UCB0I2CSA = i2c_address;
     TXbuffer[0]=address;
     TXbuffer[1]=byte1;;
     i2c_send(2);
}


unsigned long i2c_readX(unsigned char i2c_address , unsigned char address, unsigned char count)
{
  
    UCB0I2CSA = i2c_address;
    TXbuffer[0]=address;
    i2c_send(1);
    i2c_receive(count);
    
    switch(count){
  
    case 1:
        return ( RXbuffer[0] ); 
        break;  
        
    case 2:
        return ( (int) RXbuffer[0] << 8 | (int) RXbuffer[1] ); 
        break;
        
    case 3:
        return (( (long) RXbuffer[0] << 16 | (long) RXbuffer[1] <<8 | (long) RXbuffer[2] ) >> 8); 
        break;
    }
    
    return -1;
}

void i2c_send(unsigned char count)
{
    TXData= (unsigned char *)TXbuffer;
    TXByteCtr = count;                      // Load TX byte counter              
    
    UCB0IE |= UCTXIE0 + UCNACKIE;           //transmit and NACK interrupt enable
    
    while (UCB0CTLW0 & UCTXSTP);            // Ensure stop condition got sent

    UCB0CTLW0 |= UCTR;
    UCB0CTLW0 |= UCTXSTT;

    __bis_SR_register(LPM0_bits + GIE);        // Enter LPM0 w/ interrupts
    __no_operation();                       // Remain in LPM0 until all data
}

void i2c_receive(unsigned char count)
{
    RXData= (unsigned char *)RXbuffer;
    RXByteCtr = count;

    UCB0IE |= UCRXIE + UCNACKIE; 	

    while (UCB0CTL1 & UCTXSTP);             // Ensure stop condition got sent

    UCB0CTLW0 &= ~UCTR;
    UCB0CTLW0 |= UCTXSTT;
    
    __bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
    __no_operation();                       // Remain in LPM0 until all data
}

void i2c_init()
{
    UCB0CTLW0 |= UCSWRST;                        // put eUSCI_B in reset state
    UCB0CTLW0 |= UCMODE_3 + UCMST + UCSSEL_2;    // I2C master mode, SMCLk 
    UCB0BRW = 80;                                // baudrate = SMCLK / UCB0BRW
    //UCB0I2CSA = i2c_address;                     // address slave
    UCB0CTLW0 &=~ UCSWRST;	                 //clear reset register

}