#include <SPI.h>
#include <LoRa.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Configurar parámetros LoRa para que coincidan con el transmisor
  LoRa.setSpreadingFactor(7); // Coincidir con el transmisor
  LoRa.setSignalBandwidth(125E3); // Coincidir con el transmisor
  LoRa.setCodingRate4(5); // Coincidir con el transmisor
}

void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    // Serial.print("Received packet with size: ");
    // Serial.println(packetSize);

    byte dataPacket[8];
    int index = 0;

    // Leer los bytes del paquete
    while (LoRa.available() && index < sizeof(dataPacket)) {
      dataPacket[index] = LoRa.read();
      index++;
    }

    // Verificar si se recibió el número correcto de bytes
    if (index == sizeof(dataPacket)) {
      // Convertir los bytes en valores de los joysticks
      int yawValue = (dataPacket[0] << 8) | dataPacket[1];
      int pitchValue = (dataPacket[2] << 8) | dataPacket[3];
      int rollValue = (dataPacket[4] << 8) | dataPacket[5];
      int throttleValue = (dataPacket[6] << 8) | dataPacket[7];

      // Imprimir los valores
      Serial.print("yaw:");      
      Serial.print(yawValue);
      Serial.print(", pitch:");
      Serial.print(pitchValue);
      Serial.print(", roll:");
      Serial.print(rollValue);
      Serial.print(", throttle:");
      Serial.println(throttleValue);
    } else {
      Serial.println("Error: paquete recibido incompleto.");
    }
  }

  // Pausa mínima para estabilizar la comunicación serial
  delay(1);
}
