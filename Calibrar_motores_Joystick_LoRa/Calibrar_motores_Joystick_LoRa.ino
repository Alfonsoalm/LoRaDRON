#include <SPI.h>
#include <LoRa.h>
#include <Servo.h>

// Definición de pines para los motores
const int motor1Pin = 3;  // Pin para motor 1
const int motor2Pin = 5;  // Pin para motor 2
const int motor3Pin = 6;  // Pin para motor 3
const int motor4Pin = 9;  // Pin para motor 4

Servo motor1, motor2, motor3, motor4;

// Valores PWM para calibración de ESC
const int ESC_MIN = 1000;  // Valor mínimo del PWM
const int ESC_MAX = 2000;  // Valor máximo del PWM

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("LoRa Receiver");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  // Attach Servo objects to motor pins
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);

  // Calibrar los ESC
  calibrateESC();
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
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

      // Controlar motores basados en los valores de los joysticks
      controlMotors(throttleValue);

      // Imprimir los valores de los joysticks y los motores
      Serial.print("Yaw: ");
      Serial.print(yawValue);
      Serial.print(", Pitch: ");
      Serial.print(pitchValue);
      Serial.print(", Roll: ");
      Serial.print(rollValue);
      Serial.print(", Throttle: ");
      Serial.println(throttleValue);
    } else {
      Serial.println("Error: paquete recibido incompleto.");
    }
  }
}

void calibrateESC() {
  Serial.println("Iniciando calibración de ESC...");
  Serial.println("Asegúrate de que no haya hélices instaladas.");
  Serial.println("Presiona Enter para continuar...");

  while (!Serial.available());  // Esperar hasta que esté disponible alguna entrada en el puerto serie
  while (Serial.available()) Serial.read();  // Limpiar el buffer de entrada

  Serial.println("Conecta la batería para alimentar los ESC.");
  delay(1000);  // Espera 1 segundo para conectar la batería

  // Establecer el valor máximo del ESC
  setAllPWM(ESC_MAX);
  Serial.println("Valor máximo del ESC establecido. Mantén esta configuración por 1 segundo.");
  delay(1000);  // Mantener el valor MAX durante 1 segundo

  // Establecer el valor mínimo del ESC
  setAllPWM(ESC_MIN);
  Serial.println("Valor mínimo del ESC establecido. Mantén esta configuración por 2 segundos.");
  delay(2000);  // Mantener el valor MIN durante 2 segundos

  Serial.println("Calibración finalizada.");
}

void setAllPWM(int pwmValue) {
  motor1.writeMicroseconds(pwmValue);
  motor2.writeMicroseconds(pwmValue);
  motor3.writeMicroseconds(pwmValue);
  motor4.writeMicroseconds(pwmValue);
}

void controlMotors(int throttleValue) {
  // Convertir el valor del throttle a PWM para los motores
  int pwmValue = map(throttleValue, 0, 1023, ESC_MIN, ESC_MAX);

  // Asignar el valor PWM a todos los motores
  setAllPWM(pwmValue);
}
