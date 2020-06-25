//
//  AnchorNode.ino
//
//

#ifndef _FOOTSTEP
#define _FOOTSTEP

// Board Parameters
#define ADCPIN          A1
#define RESET_PIN       9
#define CS_PIN          10
#define IRQ_PIN         2
#define CLOCK_RATE      48000000
#define SAMPLE_RATE     10000
#define BUFFER_SIZE     256
#define TSIZE           5
#define CONTROL_SIZE    5 
#define SEPARATOR_SIZE  2
#define LEN_DATA        123  // MSG_TYPE, DEVICE_ID, TO_DEVICE, TOKEN, RELEASE_TOKEN, TIMESTAMP_NUM*TIMESTAMP_LEN
#define LEN_ACC_DATA    120  //112 // 120, save 8 bytes for start and stop timestamps

#define DEVICE_ID 2
#define SDCD_CS_PIN     4
#define ADCPIN          A1

#endif

// Message Sent/Received State
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile byte msgId = 0;
volatile byte msgFrom = 0;
volatile byte msgTo = 0;

// Sampling Variables
volatile boolean writeLoc = false;
volatile boolean usingBuffA = true;
volatile int bufferIdx = 0;

// Protocol Error State
boolean protocolFailed = false;


// Data Buffer
byte bufferA[BUFFER_SIZE];
byte bufferB[BUFFER_SIZE];
byte data[LEN_DATA];
byte sysTime[TSIZE];
byte targetTime[TSIZE];
byte separator[SEPARATOR_SIZE] = {0xFF,0xFF};
byte separator2[SEPARATOR_SIZE] = {0xFE,0xFE};
int counter = 0;

// Watchdog and Reset Period
unsigned long lastActivity;
unsigned long resetPeriod = 4000;
volatile unsigned long timestamp = 0;
volatile unsigned long receiveTime = 0;
volatile unsigned long localReferenceTime = 0;
volatile unsigned long bufferTimestamp = 0;
volatile unsigned long funTime1 = 0;
volatile unsigned long funTime2 = 0;
volatile boolean updateTimestamp = false;
volatile boolean printTimestamp = false;
// reply times (same on both sides for symm. ranging)
unsigned int replyDelayTimeUS = 3000;
char msg[256];
unsigned int testByte;
int resetCount = 0;

/*
 * Data write start from here
 */

void ADC_Handler(){
//  __disable_irq();
  if(ADC->INTFLAG.reg & 0x01){  // RESRDY interrupt
    uint16_t value = ADC->RESULT.reg;
    readData(value);
  }
//  __enable_irq();
}

static __inline__ void ADCsync() __attribute__((always_inline, unused));

static void   ADCsync() {
  while (ADC->STATUS.bit.SYNCBUSY == 1); //Just wait till the ADC is free
}

void adc_init(){
  analogRead(ADCPIN);           // do some pin init  pinPeripheral()
  ADC->CTRLA.bit.ENABLE = 0x00; // Disable ADC
  ADCsync();
  ADC->INTENSET.reg = 0x01; // enable RESRDY interrupt
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_DIV2_Val;  // default
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC1_Val;
  ADCsync();    //  ref 31.6.16
  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCPIN].ulADCChannelNumber;
  ADCsync();
  ADC->AVGCTRL.reg = 0x00 ;       //no averaging
  ADC->SAMPCTRL.reg = 0x11;
//  ADC->SAMPCTRL.reg = 0x00;  ; //sample length in 1/2 CLK_ADC cycles
  ADCsync();
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV512 | ADC_CTRLB_FREERUN | ADC_CTRLB_RESSEL_10BIT;//ADC_CTRLB_PRESCALER_DIV256
  ADCsync();
  ADC->CTRLA.bit.ENABLE = 0x01;
  ADCsync();
  NVIC_EnableIRQ( ADC_IRQn ) ;
  NVIC_SetPriority(ADC_IRQn, 0);
}

void readData(uint16_t vibData) {
  fillBuffer(vibData);
  checkBufferSize();
}

void fillBuffer(uint16_t vibReading) {
  if (usingBuffA) {
    bufferA[bufferIdx] = vibReading & 0x00FF;
    bufferIdx += 1;
    vibReading >>= 8;
    bufferA[bufferIdx] = vibReading & 0x00FF;
    bufferIdx += 1;
  } else {
    bufferB[bufferIdx] = vibReading & 0x00FF;
    bufferIdx += 1;
    vibReading >>= 8;
    bufferB[bufferIdx] = vibReading & 0x00FF;
    bufferIdx += 1;
  }
}

void checkBufferSize() {
  if (bufferIdx >= BUFFER_SIZE) {
    usingBuffA = !usingBuffA;
    bufferIdx = 0;
    writeLoc = true;

    if(updateTimestamp){
      bufferTimestamp = (micros() - localReferenceTime) + timestamp;
      updateTimestamp = false;
      printTimestamp = true;
    }
  }
}

void float2Bytes(byte bytes_temp[4],float float_variable){ 
  union {
    float a;
    unsigned char bytes[4];
  } thing;
  thing.a = float_variable;
  memcpy(bytes_temp, thing.bytes, 4);
}

void printDistance(byte nodeID, float distance) {
  SerialUSB.write(separator, SEPARATOR_SIZE);
  SerialUSB.write(nodeID);
  byte bytesArray[4];
  float2Bytes(bytesArray, distance);
  SerialUSB.write(separator2, SEPARATOR_SIZE);
  SerialUSB.write(bytesArray, 4);
}

void printTheTimestamp(unsigned long t){
  SerialUSB.write(separator, SEPARATOR_SIZE);
  byte tempArray[4];
  tempArray[0] = t & 0xFF;
  tempArray[1] = (t>>8) & 0xFF;
  tempArray[2] = (t>>16) & 0xFF;
  tempArray[3] = (t>>24) & 0xFF;
  SerialUSB.write(tempArray, 4);
}

void printVibration() {
  SerialUSB.write(separator, SEPARATOR_SIZE);
  if (usingBuffA) {
    SerialUSB.write(bufferB, BUFFER_SIZE);
  } else {
    SerialUSB.write(bufferA, BUFFER_SIZE);
  }
  if(printTimestamp){
    printTheTimestamp(bufferTimestamp);
    printTimestamp = false;   
  }
}

void setup() {
    SerialUSB.begin(115200);
    delay(4000);
    adc_init();
    funTime1 = millis();
}

void loop() {
    if (writeLoc) {
      writeLoc = false;
      printVibration();
    }
}
