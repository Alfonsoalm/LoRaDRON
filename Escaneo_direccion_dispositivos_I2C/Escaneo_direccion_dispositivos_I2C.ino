#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Wire.begin();
  Serial.println("Escaneando bus I2C...");
}

void loop() {
  byte error, address;
  int nDevices;
  Serial.println("Buscando dispositivos I2C...");
  nDevices = 0;
  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Dispositivo encontrado en la dirección 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.print(address, HEX);
      Serial.println("  !");
      nDevices++;
    } else if (error == 4) {
      Serial.print("Error al intentar comunicarse con el dispositivo en la dirección 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
    }
  }

  if (nDevices == 0) {
    Serial.println("No se encontraron dispositivos I2C.");
  } else {
    Serial.println("Escaneo completado.");
  }
  delay(2000); // Espera 5 segundos antes de volver a escanear
}
