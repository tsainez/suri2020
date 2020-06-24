#include <SPI.h>
#include <DW1000.h>

const uint8_t PIN_RST = 9;  // reset pin
const uint8_t PIN_IRQ = 2;  // irq pin
const uint8_t PIN_SS = SS;  // spi select pin

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Initialize driver
  DW1000.begin(PIN_IRQ, PIN_RST);
  DW1000.begin(PIN_SS);
  Serial.println(F("DW1000 initialized ..."));

  // General configuration
  DW1000.newConfiguration();
  DW1000.setDeviceAddress(5);
  DW1000.setNetworkId(10);
  DW1000.commitConfiguration();
  Serial.println(F("Committed configuration ..."));

  delay(1000);
}

void loop() {
  // put your main code here, to run repeatedly:

  // debug
  char msg[128];
  DW1000.getPrintableDeviceIdentifier(msg);
  Serial.print("Device ID: "); Serial.println(msg);
  DW1000.getPrintableExtendedUniqueIdentifier(msg);
  Serial.print("Unique ID: "); Serial.println(msg);
  DW1000.getPrintableNetworkIdAndShortAddress(msg);
  Serial.print("Network ID & Device Address: "); Serial.println(msg);
  DW1000.getPrintableDeviceMode(msg);
  Serial.print("Device mode: "); Serial.println(msg);
  
  delay(10000);
}
