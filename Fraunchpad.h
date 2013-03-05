// Pin Definitions
#define ACC_PWR_PIN       BIT7
#define ACC_PWR_PORT_DIR  P2DIR
#define ACC_PWR_PORT_OUT  P2OUT
#define ACC_PORT_DIR      P3DIR
#define ACC_PORT_OUT      P3OUT
#define ACC_PORT_SEL0     P3SEL0
#define ACC_PORT_SEL1     P3SEL1
#define ACC_X_PIN         BIT0
#define ACC_Y_PIN         BIT1
#define ACC_Z_PIN         BIT2

// Accelerometer Input Channel Definitions
#define ACC_X_CHANNEL     ADC10INCH_12
#define ACC_Y_CHANNEL     ADC10INCH_13
#define ACC_Z_CHANNEL     ADC10INCH_14

#define ADC_START_ADD 0xD400
#define ADC_END_ADD 0xF000

#define DOWN 0
#define UP 1

extern unsigned int CalibrateADC(void); 
extern void TakeADCMeas(void);
extern void SetupAccel(void); // setup ADC for Acc
extern void ShutDownAccel(void);
extern void SetupThermistor(void);
void serialstring( unsigned char *string);
void EnableSwitches(void);
void DisableSwitches(void);
void LongDelay();
void StartDebounceTimer(unsigned char delay);
void adxl335(void);
unsigned int CalibrateADC(void);
void LEDSequence(unsigned int DiffValue, unsigned char temp);