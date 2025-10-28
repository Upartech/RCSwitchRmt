#include <RCSwitchRmt.h>

constexpr int TX_PIN = 2;
RCSwitchRmt rf;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[RCSwitchRmt] DemoTransmitterbyNrepeater");

  if (!rf.enableTransmit(TX_PIN)) {
    Serial.println("ERROR: enableTransmit");
  }

  // Protocolo 1 del array interno (equivalente a rc-switch #1)
  if (!rf.setProtocolTx(1)) {
    Serial.println("ERROR: setProtocolTx");
  }

  rf.setRepeatTransmit(5);   // 5 repeticiones por send()
}

void loop() {
  uint32_t code = 0x00ABCDEF;   // 24 bits de ejemplo
  bool ok = rf.send(code, 24);
  Serial.printf("send(code=0x%06lX) -> %s  (txBusy=%d)\n", code, ok?"OK":"FAIL", rf.txBusy());

  delay(2000);
}
