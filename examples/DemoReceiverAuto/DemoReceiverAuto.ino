#include <RCSwitchRmt.h>

constexpr int RX_PIN = 16;
RCSwitchRmt radio;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[RCSwitchRmt] DemoReceiverAuto");

   radio.setProtocolAutoRx();                 // autodetección de proto[]: 60% tolerancia. 24 bits esperados.
   if (!radio.enableReceive(RX_PIN)) {
    Serial.println("ERROR: enableReceive");
  }
}

void loop() {

  if (radio.available()) { 
    Serial.printf("Code=%lu  Bitlen=%u  Proto=%u  T=%u us\n",
                  radio.getReceivedValue(), // el última valor recibido
                  radio.getReceivedBitlength(),
                  radio.getReceivedProtocol(),                  
                  radio.getReceivedDelay());
    radio.resetAvailable();
  }
}
