#include <RCSwitchRmt.h>

constexpr int TX_PIN = 2;
RCSwitchRmt radio;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[RCSwitchRmt] DemoTransmitterCustom");

  if (!radio.enableTransmit(TX_PIN)) {
    Serial.println("ERROR: enableTransmit");
  }

  const uint16_t T_us = 330;
  const uint8_t  SYNC[] = {1,7,3,37};
  const uint8_t  ZERO[] = {1,4};
  const uint8_t  ONE[]  = {4,1};
  const bool     INV    = false;

  if (!radio.setProtocolCustomTx(T_us, SYNC, 4, ZERO, 2, ONE, 2, INV)) {
    Serial.println("ERROR: setProtocolCustomTx");
  }

  radio.setRepeatTransmit(20);    // 20 ráfagas por envío
  Serial.println("Enviando cada 2 s...");
}

void loop() {
  static uint32_t code = 1234567890;  
  bool ok = radio.send(code, 32); // 32 BITS
  Serial.printf("Custom TX code=0x%03lX -> %s\n", code, ok?"OK":"FAIL");
  delay(3000);
}
