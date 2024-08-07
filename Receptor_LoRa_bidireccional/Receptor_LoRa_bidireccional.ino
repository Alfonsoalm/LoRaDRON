#include <SPI.h>
#include <LoRa.h>

// Pines de LoRa
const int DIO0_PIN = 8;   // Pin DIO0 para interrupciones
const int CS_PIN = 10;    // Pin CS (Chip Select)
const int RESET_PIN = 9; // Pin RESET

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  pinMode(DIO0_PIN, INPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(RESET_PIN, OUTPUT);

  digitalWrite(RESET_PIN, LOW);
  delay(100);
  digitalWrite(RESET_PIN, HIGH);
  delay(100);

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  LoRa.receive(); // Configurar el módulo para recibir datos
  attachInterrupt(digitalPinToInterrupt(DIO0_PIN), onReceive, RISING);
}

void onReceive() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    Serial.println("Packet received:");
    while (LoRa.available()) {
      char receivedChar = (char)LoRa.read();
      Serial.print(receivedChar);
    }
    Serial.println();

    sendAcknowledgement(); // Enviar ACK
  }
}

void sendAcknowledgement() {
  LoRa.beginPacket();
  LoRa.write(0x01); // Enviar un byte simple como ACK
  LoRa.endPacket();
  Serial.println("ACK sent");
}

void loop() {
  // No hace falta hacer nada en el loop para el receptor
}
