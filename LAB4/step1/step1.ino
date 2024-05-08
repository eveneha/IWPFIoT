#include <MKRWAN.h>
LoRaModem modem;
// Uncomment if using the Murata chip as a module
// LoRaModem modem(Serial1);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  while (!Serial);
 Serial.println("Welcome to MKR WAN 1300/1310 first configuration sketch");
  // change this to your regional band (eg. US915, AS923, ...)
  if (!modem.begin(EU868)) {
    Serial.println("Failed to start module");
    while (1) {}
  };
  Serial.print("Your device EUI is: ");
  Serial.println(modem.deviceEUI());
}
void loop() {
}
