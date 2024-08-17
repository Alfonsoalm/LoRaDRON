#include <Wire.h>
#include <LoRa.h>

// Ciclo de ejecución de software en microsegundos (PWM)
#define usCiclo 6000

// Variables de LoRa
int yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target, throttle_target_map;
bool button_A4, button_A5;

// Pines para los motores
const int motorPins[4] = {A0, A1, A2, A3};

// Variables PWM de los motores
float ESC1, ESC2, ESC3, ESC4;
const int ESC_MIN = 900;
const int ESC_MAX = 1900;

// Variables de tiempo
unsigned long loop_timer, tiempo_motores_start;

// Variable para gestionar el estado del modo actual
enum Mode { WAITING, CALIBRATION, FINISHED };
Mode currentMode = WAITING;

void setup() {
  Serial.begin(9600); 
  setupLoRa();
  setupMotors();
  Serial.println("Iniciando calibración de ESC...");
  Serial.println("Asegúrate de que no haya hélices instaladas.");
  Serial.println("Sube el throttle al máximo para comenzar la calibración...");

  // while (throttle_target > 1100 || throttle_target < 900) {
  //   readLoRa();
  //   throttle_target = map(throttle_target, 0, 1023, 900, 2000);
  // }
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
  throttle_target_map = map(throttle_target, 0, 1023, 900, 2000);
  Serial.print("Throttle: "); Serial.println(throttle_target_map);
  // Asignar el mismo valor a todos los motores
  ESC1 = ESC2 = ESC3 = ESC4 = throttle_target_map;
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

  // Sincronización del ciclo
  while (micros() - loop_timer < usCiclo);
  loop_timer = micros();

  handleModes();
}
