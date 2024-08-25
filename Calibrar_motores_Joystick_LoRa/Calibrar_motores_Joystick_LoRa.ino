#include <Wire.h>
#include <LoRa.h>
#include <Servo.h>

// Ciclo de ejecución de software en microsegundos (PWM)
#define usCiclo 20000 // 20000 en Servo

// Variables de LoRa
int yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target, throttle_target_map;
bool button_A4, button_A5;

// Pines para los motores
const int motorPins[4] = {A2, A0, A1, A3};
// Instancias de la clase Servo para cada motor
Servo motor1, motor2, motor3, motor4;

// Variables PWM de los motores
float ESC1, ESC2, ESC3, ESC4;
const int ESC_MIN = 950;
const int ESC_MAX = 2000;
const int pinLED = 7;

// Variables de tiempo
unsigned long loop_timer, tiempo_motores_start;

// Variable para gestionar el estado del modo actual
enum Mode { WAITING, CALIBRATION, FINISHED };
Mode currentMode = WAITING;

void setup() {
  Serial.begin(115200); 
  setupLoRa();
  setupMotorsServo();

  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, HIGH);

  Serial.println("Iniciando calibración de ESC...");
  Serial.println("Asegúrate de que no haya hélices instaladas.");
  Serial.println("Sube el throttle al máximo para comenzar la calibración...");

  while (throttle_target > 2100 || throttle_target < 1900) {
    readLoRa();
    throttle_target = map(throttle_target, 0, 1023, 900, 2000);
  }

  digitalWrite(pinLED, LOW);
}

void setupLoRa() {
  // Inicializar y configurar módulo LoRa a 433 MHz
  if (!LoRa.begin(433E6)) {
    Serial.println(F("Error al iniciar LoRa"));
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  Serial.println(F("LoRa inicializado"));
}

void setupMotors() {
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
  Serial.println(F("Motores inicializados"));
}

void setupMotorsServo() {
  // Asociar cada motor a su pin correspondiente
  motor1.attach(motorPins[0], ESC_MIN, ESC_MAX);
  motor2.attach(motorPins[1], ESC_MIN, ESC_MAX);
  motor3.attach(motorPins[2], ESC_MIN, ESC_MAX);
  motor4.attach(motorPins[3], ESC_MIN, ESC_MAX);

  Serial.println(F("Motores inicializados"));

  motor1.writeMicroseconds(2000);
  motor2.writeMicroseconds(2000);
  motor3.writeMicroseconds(2000);
  motor4.writeMicroseconds(2000);
}

void readLoRa() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    byte dataPacket[10];
    int index = 0;
    while (LoRa.available() && index < 10) {
      dataPacket[index++] = LoRa.read();
    }
    if (index == 10) {
      // Extraer datos del paquete
      yaw_angle_target = (dataPacket[0] << 8) | dataPacket[1];
      pitch_angle_target = (dataPacket[2] << 8) | dataPacket[3];
      roll_angle_target = (dataPacket[4] << 8) | dataPacket[5];
      throttle_target = (dataPacket[6] << 8) | dataPacket[7];
      button_A4 = dataPacket[8] == 1;
      button_A5 = dataPacket[9] == 1;
      // Serial.print("Throttle target: "); Serial.println(throttle_target);
    } else {
      Serial.println(F("Error en el tamaño del paquete recibido."));
    }
  }
}

void calibrateESC() {
  // Mapeo del throttle
  throttle_target_map = map(throttle_target, 0, 1023, ESC_MIN, ESC_MAX);
  Serial.print("Throttle: "); Serial.println(throttle_target_map);

  // Asignar el mismo valor a todos los motores
  ESC1 = ESC2 = ESC3 = ESC4 = throttle_target_map;
  
  // Tiempo de inicio del ciclo PWM
  tiempo_motores_start = micros();
  
  // Iniciar señales PWM (Ciclo de 20 ms para 50 Hz)
  while (micros() - tiempo_motores_start < usCiclo) {
    if (micros() - tiempo_motores_start < ESC1) digitalWrite(motorPins[0], HIGH);
    else digitalWrite(motorPins[0], LOW);

    if (micros() - tiempo_motores_start < ESC2) digitalWrite(motorPins[1], HIGH);
    else digitalWrite(motorPins[1], LOW);

    if (micros() - tiempo_motores_start < ESC3) digitalWrite(motorPins[2], HIGH);
    else digitalWrite(motorPins[2], LOW);

    if (micros() - tiempo_motores_start < ESC4) digitalWrite(motorPins[3], HIGH);
    else digitalWrite(motorPins[3], LOW);
  }
}

void calibrateESCServo() {
  // Mapeo del throttle
  throttle_target_map = map(throttle_target, 0, 1023, ESC_MIN, ESC_MAX);
  Serial.print("Throttle: "); Serial.println(throttle_target_map);

  // Asignar el mismo valor a todos los motores
  motor1.writeMicroseconds(throttle_target_map);
  motor2.writeMicroseconds(throttle_target_map);
  motor3.writeMicroseconds(throttle_target_map);
  motor4.writeMicroseconds(throttle_target_map);
}


void handleModes() {
  // Manejo del modo actual
  switch (currentMode) {
    case WAITING:
      Serial.println("Modo actual: ESPERA");
      Serial.println("Inicializando sistema...");
      //setAllPWM(0);
      // Verificar si se debe realizar calibración
      if (button_A4) {
        Serial.println("Iniciando calibración...");
        currentMode = CALIBRATION;
        return;
      }
      break;
    case CALIBRATION:
      Serial.println("Modo actual: CALIBRATION");
      calibrateESC();
      if (button_A5) {
        Serial.println("Calibración completada. Cambiando a FINISHED...");
        currentMode = FINISHED;
        return;
      }
      break;
    case FINISHED:
      Serial.println("Modo actual: FINISHED");
      digitalWrite(motorPins[0], LOW);
      digitalWrite(motorPins[1], LOW);
      digitalWrite(motorPins[2], LOW);
      digitalWrite(motorPins[3], LOW);
      // Asignar el mismo valor a todos los motores
      ESC1 = ESC2 = ESC3 = ESC4 = 800;
      // Iniciar señales PWM
      for (int i = 0; i < 4; i++) {
        digitalWrite(motorPins[i], HIGH);
      }
      tiempo_motores_start = micros();
      // Controlar señales PWM
      unsigned long endCycleTime = tiempo_motores_start + usCiclo;
      
      while (micros() < endCycleTime) {
        unsigned long currentTime = micros(); 
        if (currentTime - tiempo_motores_start >= ESC1) digitalWrite(motorPins[0], LOW);
        if (currentTime - tiempo_motores_start >= ESC2) digitalWrite(motorPins[1], LOW);
        if (currentTime - tiempo_motores_start >= ESC3) digitalWrite(motorPins[2], LOW);
        if (currentTime - tiempo_motores_start >= ESC4) digitalWrite(motorPins[3], LOW);
      }
      break;
    default:
      Serial.println("Modo actual: DESCONOCIDO");
      break;
  }
}

void loop() {
  readLoRa();

  calibrateESCServo();
  int us1 = motor1.readMicroseconds();
  int us2 = motor1.readMicroseconds();
  int us3 = motor1.readMicroseconds();
  int us4 = motor1.readMicroseconds();
  Serial.print("us1: "); Serial.println(us1);
  
}
