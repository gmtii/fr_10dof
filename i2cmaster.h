extern unsigned char *TXData;                   // TXbuffer pointer             
extern unsigned char *RXData;                   // RXbuffer pointer       
extern unsigned char TXByteCtr,RXByteCtr;       // TX & RX counters
extern unsigned char RXbuffer[6];
extern unsigned char TXbuffer[6];

void i2c_init();                                // Init I2C UCSB
void i2c_send(unsigned char count);
void i2c_receive(unsigned char count);

void i2c_send8(unsigned char i2c_address , unsigned char address, unsigned char byte1 );
unsigned long i2c_readX(unsigned char i2c_address , unsigned char address, unsigned char count);
