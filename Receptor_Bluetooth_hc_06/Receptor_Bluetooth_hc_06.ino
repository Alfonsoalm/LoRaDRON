#include <SoftwareSerial.h>

SoftwareSerial BTSerial(0, 1); // RX | TX

void setup() {
  Serial.begin(9600); // Esto es para la comunicación USB con el PC
  BTSerial.begin(9600); // Esto es para la comunicación con el HC-06
}

void loop() {
  if (BTSerial.available()) {
    String receivedData = BTSerial.readString();
    Serial.print("Received: ");
    Serial.println(receivedData);
  }
}
