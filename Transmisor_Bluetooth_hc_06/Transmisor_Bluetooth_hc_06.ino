#include <SoftwareSerial.h>

SoftwareSerial BTSerial(0, 1); // RX | TX

void setup() {
  // Serial.begin(9600);
  BTSerial.begin(9600);  // Velocidad de comunicación del HC-06
}

void loop() {
  BTSerial.println("Hello from Arduino 1");
  delay(1000);
}
