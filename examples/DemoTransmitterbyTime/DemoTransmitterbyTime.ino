#include <RCSwitchRmt.h>

constexpr int TX_PIN = 2;
RCSwitchRmt radio;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[RCSwitchRmt] DemoTransmitterbyTime");

  if (!radio.enableTransmit(TX_PIN)) {
    Serial.println("ERROR: enableTransmit");
  }

  // Protocolo 1 del array interno (equivalente a rc-switch #1)
  if (!radio.setProtocolTx(1)) {
    Serial.println("ERROR: setProtocolTx");
  }

  radio.setRepeatTransmit(1);      // SINGLE por repetición
  radio.setMinTransmitTime(1200);  // al menos 1.2 s transmitiendo
}

uint32_t code = 0x0055AA55;      
void loop() {
  
  // Espera a que termine la ventana de tiempo
 if(radio.txActive()==false){ // transmite si no está ocupado
      bool ok = radio.send(code, 24); // orden de enviar. Se perpeturá por 1.2s con solo hacer un llamado porque fué configurado
      Serial.printf("BY_TIME send -> %s  (txActive=%d)\n", ok?"OK":"FAIL", radio.txActive());
 }

 Serial.println("Other tasks, the transmission goes in another thread");
 delay(2000); 
  
}
