#include <SPI.h>
#include <LoRa.h>

const int YAW_PIN = A6;         // VRY IZQUIERDO
const int PITCH_PIN = A1;       // VRX IZQUIERDO
const int ROLL_PIN = A2;        // VRX DERECHO
const int THROTTLE_PIN = A7;    // VRY DERECHO
const int BUTTON_A4_PIN = A4;   // Botón A4
const int BUTTON_A5_PIN = A5;   // Botón A5
const int LED1_PIN = 7;         // LED para señalización
const int LED2_PIN = 8;         // LED para señalización

unsigned long lastSendTime = 0;
const unsigned long sendInterval = 50; // Intervalo de envío en milisegundos
const unsigned long ledOnDuration = 2000; // Duración de los LEDs encendidos en milisegundos

unsigned long ledA4OnStart = 0;
unsigned long ledA5OnStart = 0;
bool ledA4On = false;
bool ledA5On = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Transmitter");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  
  // Configurar parámetros LoRa para alta velocidad
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  
  // Configurar pines de LEDs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  
  // Configurar pines de botones
  pinMode(BUTTON_A4_PIN, INPUT);
  pinMode(BUTTON_A5_PIN, INPUT);

  // Asegúrate de que los LEDs estén apagados al principio
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
}

void loop() {
  unsigned long currentTime = millis();

  // Leer estado de los botones
  bool buttonA4Pressed = digitalRead(BUTTON_A4_PIN); // HIGH significa presionado
  bool buttonA5Pressed = digitalRead(BUTTON_A5_PIN); // HIGH significa presionado

  // Serial.print("Button A4 State: ");
  // Serial.println(buttonA4Pressed ? "Pressed" : "Not Pressed");
  // Serial.print("Button A5 State: ");
  // Serial.println(buttonA5Pressed ? "Pressed" : "Not Pressed");

  // Manejar el LED del botón A4
  if (buttonA4Pressed) {
    if (!ledA4On) {
      digitalWrite(LED1_PIN, HIGH);
      ledA4On = true;
      ledA4OnStart = currentTime;
      // Serial.println("LED1 ON");
    }
  } else {
    if (ledA4On) {
      if (currentTime - ledA4OnStart >= ledOnDuration) {
        digitalWrite(LED1_PIN, LOW);
        ledA4On = false;
        // Serial.println("LED1 OFF");
      }
    }
  }

  // Manejar el LED del botón A5
  if (buttonA5Pressed) {
    if (!ledA5On) {
      digitalWrite(LED2_PIN, HIGH);
      ledA5On = true;
      ledA5OnStart = currentTime;
      // Serial.println("LED2 ON");
    }
  } else {
    if (ledA5On) {
      if (currentTime - ledA5OnStart >= ledOnDuration) {
        digitalWrite(LED2_PIN, LOW);
        ledA5On = false;
        // Serial.println("LED2 OFF");
      }
    }
  }

  // Enviar paquete LoRa cada sendInterval milisegundos
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;

    int yawValue = analogRead(YAW_PIN);
    int pitchValue = analogRead(PITCH_PIN);
    int rollValue = analogRead(ROLL_PIN);
    int throttleValue = analogRead(THROTTLE_PIN);

    // Crear un array de bytes para los datos
    byte dataPacket[10];
    dataPacket[0] = yawValue >> 8;
    dataPacket[1] = yawValue & 0xFF;
    dataPacket[2] = pitchValue >> 8;
    dataPacket[3] = pitchValue & 0xFF;
    dataPacket[4] = rollValue >> 8;
    dataPacket[5] = rollValue & 0xFF;
    dataPacket[6] = throttleValue >> 8;
    dataPacket[7] = throttleValue & 0xFF;
    dataPacket[8] = buttonA4Pressed ? 1 : 0; // Estado del botón A4
    dataPacket[9] = buttonA5Pressed ? 1 : 0; // Estado del botón A5

    Serial.println("Sending packet:");
    Serial.print("yaw:");
    Serial.print(yawValue);
    Serial.print(",pitch:");
    Serial.print(pitchValue);
    Serial.print(",roll:");
    Serial.print(rollValue);
    Serial.print(",throttle:");
    Serial.println(throttleValue);
    Serial.print(",buttonA4:");
    Serial.print(buttonA4Pressed);
    Serial.print(",buttonA5:");
    Serial.println(buttonA5Pressed);

    // Enviar paquete
    LoRa.beginPacket();
    LoRa.write(dataPacket, sizeof(dataPacket));
    LoRa.endPacket();
  }

// Recibir paquete LoRa
  // int packetSize = LoRa.parsePacket();
  // if (packetSize == sizeof(float) * 6) {  // Esperar 24 bytes (6 variables de tipo float de 4 bytes cada una)
  //   byte receivedPacket[24];
  //   int index = 0;

  //   while (LoRa.available() && index < 24) {
  //     receivedPacket[index++] = LoRa.read();
  //   }

  //   if (index == 24) {
  //     // Decodificar los datos recibidos
  //     float receivedPitch;
  //     float receivedRoll;
  //     float receivedESC1;
  //     float receivedESC2;
  //     float receivedESC3;
  //     float receivedESC4;

  //     memcpy(&receivedPitch, &receivedPacket[0], sizeof(receivedPitch));
  //     memcpy(&receivedRoll, &receivedPacket[4], sizeof(receivedRoll));
  //     memcpy(&receivedESC1, &receivedPacket[8], sizeof(receivedESC1));
  //     memcpy(&receivedESC2, &receivedPacket[12], sizeof(receivedESC2));
  //     memcpy(&receivedESC3, &receivedPacket[16], sizeof(receivedESC3));
  //     memcpy(&receivedESC4, &receivedPacket[20], sizeof(receivedESC4));

  //     Serial.println("Received packet:");
  //     Serial.print("pitch:");
  //     Serial.print(receivedPitch);
  //     Serial.print(",roll:");
  //     Serial.print(receivedRoll);
  //     Serial.print(",ESC1:");
  //     Serial.print(receivedESC1);
  //     Serial.print(",ESC2:");
  //     Serial.print(receivedESC2);
  //     Serial.print(",ESC3:");
  //     Serial.print(receivedESC3);
  //     Serial.print(",ESC4:");
  //     Serial.println(receivedESC4);
  //   } else {
  //     Serial.println("Received packet size mismatch");
  //   }
  // }

}
