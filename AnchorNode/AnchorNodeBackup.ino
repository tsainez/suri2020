/*
 * Anchor Node: 
 * vibration sensing and synchronization
 */


// FOOTSTEP.H INCLUSION
#ifndef _FOOTSTEP
#define _FOOTSTEP

 // messages used in the ranging protocol
#define POLL         0
#define POLL_ACK        1
#define RANGE           2
#define RANGE_REPORT      3
#define GRANT_TOKEN       4
#define TOKEN_RELEASE   5
#define RESET_NETWORK   6
#define SYNC_REQ      7
#define SYNC_ACK      8
#define RANGE_FAILED      255

// base station status
#define BS_IDLE       0
#define BS_WAIT_FOR_TAG   1
#define BS_TAG_TIMEOUT    2 
#define BS_WAIT_FOR_ANCHOR  3
#define BS_ANCHOR_TIMEOUT 4
#define BS_MOVING_ON    5
#define BS_TAG_RESET_COUNT  6
#define BS_INTERVAL     800

// anchor status
#define ANCHOR_INTERVAL   50

// tag status
#define TAG_IDLE      0
#define TAG_WAIT_FOR_ANCHOR 1
#define TAG_INTERVAL    500 

// conditions
#define ANCHOR_DIED     1
#define TAG_DIED      2

// board parameters
#define ADCPIN        A1
#define RESET_PIN           9
#define CS_PIN              10
#define IRQ_PIN             2
#define CLOCK_RATE          48000000
#define SAMPLE_RATE         10000
#define BUFFER_SIZE         256
#define TSIZE               5
#define CONTROL_SIZE      5 
#define SEPARATOR_SIZE      2
#define LEN_DATA            123 // MSG_TYPE, DEVICE_ID, TO_DEVICE, TOKEN, RELEASE_TOKEN, TIMESTAMP_NUM*TIMESTAMP_LEN
#define LEN_ACC_DATA    120 //112 // 120, save 8 bytes for start and stop timestamps

////// change with the useExtendedFrameLength in DW1000.cpp
// #define LEN_DATA           123 // MSG_TYPE, DEVICE_ID, TO_DEVICE, TOKEN, RELEASE_TOKEN, TIMESTAMP_NUM*TIMESTAMP_LEN
// #define LEN_ACC_DATA   112 

// network setting
#define NETWORK_ID        10
#define ANCHOR_NUM        1
#define TAG_NUM       0//2 
#define ANCHOR_ID_OFFSET    1
#define TAG_ID_OFFSET       10
#define RADIO_RESET_COUNT 10

#endif
// END FOOTSTEP.H

#define DEVICE_ID 2

#define SDCD_CS_PIN     4
#define RADIO_CS_PIN    10
#define RADIO_RST_PIN   9
#define RADIO_IRQ_PIN   2
#define ADCPIN          A1
#define DW_MICROSECONDS 1

// message flow state
//volatile byte expectedMsgId = POLL;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
volatile byte msgId = 0;
volatile byte msgFrom = 0;
volatile byte msgTo = 0;
// sampling variables
volatile boolean writeLoc = false;
volatile boolean usingBuffA = true;
volatile int bufferIdx = 0;
// protocol error state
boolean protocolFailed = false;

// data buffer
byte bufferA[BUFFER_SIZE];
byte bufferB[BUFFER_SIZE];
byte data[LEN_DATA];
byte sysTime[TSIZE];
byte targetTime[TSIZE];
byte separator[SEPARATOR_SIZE] = {0xFF,0xFF};
byte separator2[SEPARATOR_SIZE] = {0xFE,0xFE};
int counter = 0;

// watchdog and reset period
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
  //DW1000.getSystemTimestamp(sysTime);
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


/*
 * Data write from here
 */

/*
 * RANGING ALGORITHMS
 * ------------------
 * Either of the below functions can be used for range computation (see line "CHOSEN
 * RANGING ALGORITHM" in the code).
 * - Asymmetric is more computation intense but least error prone
 * - Symmetric is less computation intense but more error prone to clock drifts
 *
 * The anchors and tags of this reference example use the same reply delay times, hence
 * are capable of symmetric ranging (and of asymmetric ranging anyway).
 */

 /*

void computeRangeAsymmetric() {
  // asymmetric two-way ranging (more computation intense, less error prone)
  DW1000Time round1 = (timePollAckReceived - timePollSent).wrap();
  DW1000Time reply1 = (timePollAckSent - timePollReceived).wrap();
  DW1000Time round2 = (timeRangeReceived - timePollAckSent).wrap();
  DW1000Time reply2 = (timeRangeSent - timePollAckReceived).wrap();
  DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
  // set tof timestamp
  timeComputedRange.setTimestamp(tof);
}

void computeRangeSymmetric() {
  // symmetric two-way ranging (less computation intense, more error prone on clock drift)
  DW1000Time tof = ((timePollAckReceived - timePollSent) - (timePollAckSent - timePollReceived) +
                    (timeRangeReceived - timePollAckSent) - (timeRangeSent - timePollAckReceived)) * 0.25f;
  // set tof timestamp
  timeComputedRange.setTimestamp(tof);
}

 */

/*
 * END RANGING ALGORITHMS
 * ----------------------
 */


void setup() {
    // DEBUG monitoring
    SerialUSB.begin(115200);
    delay(4000);
    SerialUSB.println("### DW1000-arduino-ranging-anchor ###");
    adc_init();
    funTime1 = millis();
}

void loop() {
    /*
    * SERIAL OUTPUT PART
    */
//    if (writeLoc) {
//      writeLoc = false;
//      printVibration();
//    }
}
