#include <RCSwitchRmt.h>

constexpr int RX_PIN = 16;
RCSwitchRmt radio;

void setup() {

  Serial.begin(115200);
  delay(10000);
  Serial.println("\n[RCSwitchRmt] DemoReceiverCustom");
 

  if (!radio.enableReceive(RX_PIN)) {
    Serial.println("ERROR: enableReceive");
  }


  const uint16_t T_us = 330;
  const uint8_t  SYNC[] = {1,7,3,37};
  const uint8_t  ZERO[] = {1,4};
  const uint8_t  ONE[]  = {4,1};
  const bool     INVERT = false;
   // Configura protocolo RX explícito
  radio.setProtocolCustomRx(T_us, SYNC, 4, ZERO, 2, ONE, 2, INVERT); 
  radio.setExpectedBits(32); // esperar 32 bits


}

void loop() {
  if (radio.available()) {
    Serial.printf("RX code=%lu  bits=%u  proto=%u  T=%u us\n",
                  radio.getReceivedValue(),
                  radio.getReceivedBitlength(),
                  radio.getReceivedProtocol(),
                  radio.getReceivedDelay());
    radio.resetAvailable();
  }
}
