#include <RCSwitchRmt.h>

constexpr int RX_PIN = 16;
RCSwitchRmt radio;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[RCSwitchRmt] DemoReceiverBuffer");

  radio.setProtocolAutoRx();    // busca protocolo definido en proto[], con 60% de tolerancia y 24 bits

  if (!radio.enableReceive(RX_PIN)) {
    Serial.println("ERROR: enableReceive");
  }
}

void loop() {
  // Consumimos el buffer completo cada pasada del loop
  Decoded d;
  while (radio.read(&d)) {                   // true mientras haya elementos en el buffer
    Serial.printf("[BUF] code=%lu bits=%u proto=%u T=%u us t_ms=%lu (pend=%u)\n",
                  d.code, d.bits, d.proto, d.T_us, d.t_ms,
                  (unsigned)radio.availableCount());
  }

  // Haz otras tareas pesadas tranquilamente (WiFi, ESP-NOW, MQTT, etc.)
  delay(500);
}
