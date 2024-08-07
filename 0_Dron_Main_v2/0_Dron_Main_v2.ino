#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <LoRa.h>
#include <MPU6050.h>
#include <MPU6500_WE.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>

// Configuración del puerto serie para el módulo Bluetooth
SoftwareSerial BTSerial(0, 1); // RX, TX - Asegúrate de conectar RX del HC-06 al pin 10 y TX al pin 11

#define PI 3.14159265358979323846
#define usCiclo 10000
#define MPU6500_ADDR 0x68
#define MPU6050_ADDR 0x69
#define BMP280_ADDR 0x76
#define ADS1115_ADDR 0x48

MPU6050 mpu;
// MPU6500_WE mpu_master = MPU6500_WE(MPU6500_ADDR);

// CONTROL PID ------------------------->>>>
// Estructura para almacenar variables del PID
struct PIDVariables {
  double setPoint;
  double input;
  double output;
  double previousInput;
  double error;
  double P, I, D;
  double Kp, Ki, Kd;
  double integral;
};
// Variables PID para ángulo
PIDVariables pitchPID, rollPID;

// Variables PID para velocidad
PIDVariables pitchRatePID, rollRatePID, yawRatePID;

// Parámetros PID de ángulo
const double Kp_Pitch = 0.5, Ki_Pitch = 0.05, Kd_Pitch = 10; // Kp_Pitch=0.5, Ki_Pitch=0.05, Kd_Pitch=10
const double Kp_Roll = 0.5, Ki_Roll = 0.05, Kd_Roll = 10;  // Kp_Roll=0.5, Ki_Roll=0.05, Kd_Roll=10

// Parámetros PID de velocidad angular
const double Kp_w_Pitch = 2.0, Ki_w_Pitch = 0.02, Kd_w_Pitch = 0; // Kp_w_Pitch = 2.0, Ki_w_Pitch = 0.02, Kd_w_Pitch = 0
const double Kp_w_Roll = 2.0, Ki_w_Roll = 0.02, Kd_w_Roll = 0; // Kp_w_Roll = 2.0, Ki_w_Roll = 0.02, Kd_w_Roll = 0;
const double Kp_w_Yaw = 2.0, Ki_w_Yaw = 0.02, Kd_w_Yaw = 0; // Kp_w_Yaw = 1.0, Ki_w_Yaw = 0.05, Kd_w_Yaw = 0;

// Variables de salida de los PID de velocidad angular
double w_Pitch_OUT = 0, w_Roll_OUT = 0, w_Yaw_OUT = 0;
double ang_Pitch_OUT = 0, ang_Roll_OUT = 0;
// CONTROL PID -------------------------<<<<

// LORA -------------------------------->>>>
// Variables de LoRa
int yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target, throttle_target_map;
bool button_A4, button_A5;
// LORA --------------------------------<<<<

// LEDS -------------------------------->>>>
// Pines donde están conectados los LEDs
const int ledPins[] = {2, 3, 4, 5, A5}; 
// Definir una estructura para los pines del LED RGB
struct RGBPins {
    int redPin;
    int greenPin;
    int bluePin;
};

// Crear una instancia de la estructura y asignar los pines
RGBPins rgbPins = {A6, 6, 7};  // Ajusta los pines según tu configuración

// Cantidad de LEDs
const int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);

// Rango de brillo de los LEDs
const int LED_MIN_BRIGHTNESS = 0;
const int LED_MAX_BRIGHTNESS = 255;

// Retardo entre cambios de LEDs (en milisegundos)
const unsigned long delay_Time =200;
unsigned long previousMillis = 0;

// Variable para almacenar el estado actual
int currentLed = 0;
// LEDS --------------------------------<<<<

// MOTORES ----------------------------->>>>
// Definición de pines para los motores y LEDs
const int motor1Pin = A1;
const int motor2Pin = A2;
const int motor3Pin = A0;
const int motor4Pin = A3;

// Instanciacion de motores brushless RS2205
Servo motor1, motor2, motor3, motor4;

// Variables PWM de los motores
float ESC1 = 0, ESC2 = 0, ESC3 = 0, ESC4 = 0;
const int ESC_MIN = 900; const int ESC_MAX = 1900;

// Calibración de ESC
bool calibrationDone = false;
int previousThrottleValue = 0;
// MOTORES ------------------------------<<<<

// MPU ---------------------------------->>>>
// Variables de medición del sensor
float gx_offset = 0, gy_offset = 0, gz_offset = 0;
float ax_offset = 0, ay_offset = 0, az_offset = 0;

const int filterSize = 10; // Tamaño del filtro
int filterIndex = 0;
// Arrays para almacenar las muestras del acelerómetro y giroscopio
int accXValues[filterSize];
int accYValues[filterSize];
int accZValues[filterSize];
int gyrXValues[filterSize];
int gyrYValues[filterSize];
int gyrZValues[filterSize];

// Variables promedio para almacenar los valores filtrados
float ax_avg = 0, ay_avg = 0, az_avg = 0;
float gx_avg = 0, gy_avg = 0, gz_avg = 0;

int ax, ay, az;
int gx, gy, gz;

float pitch, roll, yaw;
float pitch_prev, roll_prev;

long  loop_timer, timer_f1, timer_f2, timer_f3, timer_f4, timer_f5, reading_time_seconds;
bool set_gyro_angles;
// MPU ----------------------------------<<<<

// BAROMETRO ---------------------------->>>>
// Instacia de BMP280
Adafruit_BMP280 bmp;
// Variables medidas del barometro BMP280
float temperature, pressure, altitude;
// BAROMETRO ----------------------------<<<<

// MEDICIÓN_BATERIA --------------------->>>>
// Instacia de ADS1115
Adafruit_ADS1115 ads;
// Variables de la medicion de la bateria por el ADS1117
float batteryVoltage, batteryPercent;
// MEDICIÓN_BATERIA ---------------------<<<<

// MOTOR TEST MODE ---------------------->>>>
// Valores de PWM de ejemplo para la prueba
int pwmValues[5] = {1000, 1300, 1500, 2000, 1000};

// Array de motores para facilidad de manejo
Servo motors[] = {motor1, motor2, motor3, motor4};

// Índices para recorrer los arrays
int motorIndex = 0;
int pwmIndex = 0;
bool testComplete = false; // Bandera para indicar que la prueba ha terminado
// MOTOR TEST MODE ----------------------<<<<

// Variables para controlar el tiempo
unsigned long previousTime = 0;
const long interval = 5; // Intervalo de 5 ms

void setup() {
  // Serial.begin(9600);
  BTSerial.begin(9600); 

  // Inicializar sensores y comunicación
  setupSensors();
  setupLoRa();
  setupMotors();
  setupLEDs();

  // Lectura de la bateria dron
  readBattery();  
  if (batteryPercent < 15){
    // BTSerial.println("   Bateria Baja, vuelva a casa !!");
  }

  // Inicializar PID variables
  pitchPID.Kp = Kp_Pitch;
  pitchPID.Ki = Ki_Pitch;
  pitchPID.Kd = Kd_Pitch;

  rollPID.Kp = Kp_Roll;
  rollPID.Ki = Ki_Roll;
  rollPID.Kd = Kd_Roll;

  pitchRatePID.Kp = Kp_w_Pitch;
  pitchRatePID.Ki = Ki_w_Pitch;
  pitchRatePID.Kd = Kd_w_Pitch;

  rollRatePID.Kp = Kp_w_Roll;
  rollRatePID.Ki = Ki_w_Roll;
  rollRatePID.Kd = Kd_w_Roll;

  yawRatePID.Kp = Kp_w_Yaw;
  yawRatePID.Ki = Ki_w_Yaw;
  yawRatePID.Kd = Kd_w_Yaw;

}

void testLEDs() {
  // Verifica el tiempo transcurrido
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= delay_Time) {
    // Actualiza el tiempo anterior
    previousMillis = currentMillis;
    // Apaga todos los LEDs
    for (int i = 0; i < numLeds; i++) {
      digitalWrite(ledPins[i], LOW);
    }
    // Enciende el LED actual
    digitalWrite(ledPins[currentLed], HIGH);
    // Actualiza el LED actual
    currentLed++;
    // Reinicia el ciclo al llegar al último LED
    if (currentLed >= numLeds) {
      currentLed = 0;
    }
  }
}

void setupSensors() {
  // Inicializar y configurar MPU6500 (maestra)
  Wire.begin();

  // Inicializar y configurar MPU6050 (esclavo)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 registro 6B hex
  Wire.write(0x00);  // 00000000 para activar
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG registro 1B hex
  Wire.write(0x08);  // 00001000: 500dps
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG registro 1C hex
  Wire.write(0x10);  // 00010000: +/- 8g
  Wire.endTransmission();
  // BTSerial.println(F("MPU6050 (esclavo) inicializado"));

  // Calibracion MPU6050 esclava
  calibrarMPU6050();

  // Inicializar BMP280
  // if (!bmp.begin(BMP280_ADDR)) {
  //   Serial.println(F("No se encontró un sensor BMP280 válido"));
  //   while (1);
  // }

  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
  //                 Adafruit_BMP280::SAMPLING_X2,
  //                 Adafruit_BMP280::SAMPLING_X16,
  //                 Adafruit_BMP280::FILTER_X16,
  //                 Adafruit_BMP280::STANDBY_MS_500);
  // Serial.println(F("BMP280 inicializado"));

  // Inicializar ADS1115
  if (!ads.begin(ADS1115_ADDR)) {
    // BTSerial.println(F("No se encontró el ADS1115"));
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);
  // BTSerial.println(F("ADS1115 inicializado"));
}

void setupLoRa() {
  // inicializar y configurar modulo lora a 433 Mhz
  if (!LoRa.begin(433E6)) {
    // Serial.println(F("Error al iniciar LoRa"));
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  // BTSerial.println(F("LoRa inicializado"));
}

void setupMotors() {
  // configurar los pines de los motores como PWM
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
  delay(500);
  // BTSerial.println(F("Motores inicializados"));
}

void setupLEDs() {               
  // Configura todos los pines de LEDs como salida
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
  // BTSerial.println(F("LEDs inicializados"));
  // Configurar los pines del LED RGB como salidas
  pinMode(rgbPins.redPin, OUTPUT);
  pinMode(rgbPins.greenPin, OUTPUT);
  pinMode(rgbPins.bluePin, OUTPUT);
}

void setRGBColor(bool red, bool green, bool blue) {
    if (red) {
      digitalWrite(rgbPins.redPin, HIGH);
    } else {
      digitalWrite(rgbPins.redPin, LOW);
    }
    if (green){
      digitalWrite(rgbPins.greenPin, HIGH);
    } else {
      digitalWrite(rgbPins.greenPin, LOW);
    }
    if (blue){
      digitalWrite(rgbPins.bluePin, HIGH);
    } else {
      digitalWrite(rgbPins.bluePin, LOW);
    }
    
}

void readMPU6050(){
  // Los datos del giroscopio y el acelerómetro se encuentran de la dirección 3B a la 14
  Wire.beginTransmission(MPU6050_ADDR); // Empezamos comunicación
  Wire.write(0x3B); // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDR, 14); // Solicitar un total de 14 registros
  if (Wire.available() == 14) {
    ax = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    ay = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    az = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temperature = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  }
  ax_avg = ax; ay_avg = ay; az_avg = az;
  gx_avg = gx; gy_avg = gy; gz_avg = gz;
}

void calibrarMPU6050() {
  gx_offset = gy_offset = gz_offset = 0;
  ax_offset = ay_offset = az_offset = 0;
  // BTSerial.print(F("Calibrando MPU6050..."));
  for (int cal_int = 0; cal_int < 3000; cal_int++) {
    readMPU6050();
    gx_offset += gx; gy_offset += gy; gz_offset += gz;
    ax_offset += ax; ay_offset += ay; az_offset += az;
    delayMicroseconds(50);
  }
  gx_offset /= 3000; gy_offset /= 3000; gz_offset /= 3000; 
  ax_offset /= 3000; ay_offset /= 3000; az_offset /= 3000;
  // BTSerial.println(F("Calibración MPU6050 terminada."));
}

void processMPU6050() {
  // Restar valores de calibración del acelerómetro
  ax = (ax_avg - ax_offset);
  ay = (ay_avg - ay_offset);
  az = (az_avg - az_offset);
  az += 4096;

  // Restar valores de calibración del giroscopio y calcular 
  gx = (gx_avg - gx_offset) / 65.5; // Conversion de sensibilidad implicita
  gy = (gy_avg - gy_offset) / 65.5;
  gz = (gz_avg - gz_offset) / 65.5; 

  // Calcular angulos de inclinación con datos del giroscopio
  pitch += gx * reading_time_seconds / 1000;
  roll  += gy * reading_time_seconds / 1000;
  pitch += roll * sin((gz - gz_offset) * reading_time_seconds * 0.000000266); // pi/(65.5*180*1000)
  roll  -= pitch * sin((gz - gz_offset) * reading_time_seconds * 0.000000266); // pi/(65.5*180*1000)

  // Calcular angulos de inclinación con datos del acelerometro
  float acc_total_vector = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az, 2));
  float pitch_acc = asin((float)ay / acc_total_vector) * 180 / PI; // Convertir a grados
  float roll_acc  = asin((float)ax / acc_total_vector) * -180 / PI; // Convertir a grados

  // Filtro complementario
  if (set_gyro_angles) {
    pitch = pitch * 0.99 + pitch_acc * 0.01;
    roll  = roll * 0.99 + roll_acc  * 0.01;
  } else {
    pitch = pitch_acc;
    roll  = roll_acc;
    set_gyro_angles = true;
  }

  // int max = 100;
  // int min = -100;
  // Serial.print(" Pitch:"); Serial.print(pitch);
  // Serial.print(", Roll:"); Serial.print(roll);
  // Serial.print(", Max:"); Serial.print(max);
  // Serial.print(", Min:"); Serial.println(min);
}

void readBMP280() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(1013.25);  // Ajustar al valor de presión local
  // Serial.print(F("Temperatura: ")); Serial.print(temperature); Serial.println(F(" ºC"));
  // Serial.print(F("Presion: ")); Serial.print(pressure); Serial.println(F(" Pa"));
  // Serial.print(F("Altitud: ")); Serial.print(altitude); Serial.println(F(" m"));
}

void readBattery() {
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.1875 / 1000.0; // Convertir la lectura ADC a voltaje (ADS1115 tiene una resolución de 0.1875mV por bit)
  batteryVoltage = voltage * (11.1 / 5.0); // Ajustar por el divisor de voltaje (suponiendo que el divisor reduce a 5V para 11.1V de batería)
  batteryPercent = map(batteryVoltage, 9.0, 12.6, 0, 100); // Asumiendo un rango típico de 9.0V (descargada) a 12.6V (completamente cargada)
  batteryPercent = constrain(batteryPercent, 0, 100); // Asegurar que el porcentaje esté entre 0 y 100%
  // Serial.print(F("Voltaje de batería: ")); Serial.print(batteryVoltage); Serial.println(F(" V"));
  // Serial.print(F("Porcentaje: ")); Serial.print(batteryPercent); Serial.println(F(" %"));
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
    } else {
      // BTSerial.println(F("Error en el tamaño del paquete recibido."));
    }
  }
}

void sendBluetoothData() {
  // Enviar datos en formato JSON
  BTSerial.print(" PITCH:");
  BTSerial.print(pitch);
  BTSerial.print(", ROLL:");
  BTSerial.print(roll);
  BTSerial.print(", ESC1:");
  BTSerial.print(ESC1);
  BTSerial.print(", ESC2:");
  BTSerial.print(ESC2);
  BTSerial.print(", ESC3:");
  BTSerial.print(ESC3);
  BTSerial.print(", ESC4:");
  BTSerial.print(ESC4);
  delay(1000);
}


void calibrateESC() {
  // BTSerial.println(F("Iniciando calibración de ESC..."));
  setAllPWM(ESC_MAX); // Establece todos los ESC en el valor máximo para la calibración
  delay(2000);        // Mantén el valor máximo por 2 segundos
  setAllPWM(ESC_MIN); // Luego, establece todos los ESC en el valor mínimo
  delay(2000);        // Mantén el valor mínimo por 2 segundos
  // BTSerial.println(F("Calibración de ESC completada."));
  calibrationDone = true;

  // Apagar todos los LEDs después de la calibración
  setLEDs(LED_MIN_BRIGHTNESS);  
}

void setAllPWM(int pwmValue) {
    // // Array de motores para facilidad de manejo
    // Servo motors[] = {motor1, motor2, motor3, motor4};
    // // Poner todos los motores al valor especificado
    // for (int i = 0; i < 4; i++) {
    //     motors[i].writeMicroseconds(pwmValue);
    // }
    motor1.writeMicroseconds(pwmValue);
    motor2.writeMicroseconds(pwmValue);
    motor3.writeMicroseconds(pwmValue);
    motor4.writeMicroseconds(pwmValue);
}

void setLEDs(int value) {
  for (int i = 0; i < numLeds; i++) {
    analogWrite(ledPins[i], value);
  }
}

double calculatePID(PIDVariables &pid, double input, double setPoint, double integralSat, double outputSat) {
  pid.error = setPoint - input;
  pid.P = pid.Kp * pid.error;
  pid.I += pid.Ki * pid.error;
  pid.D = pid.Kd * (input - pid.previousInput);
  pid.output = pid.P + pid.I + pid.D;
  
  if (pid.output < -outputSat) {
    pid.output = -outputSat;
    pid.I -= pid.Ki * pid.error;
  } else if (pid.output > outputSat) {
    pid.output = outputSat;
    pid.I -= pid.Ki * pid.error;
  }
  pid.previousInput = input;
  return pid.output; 
}

void controlPID() {
  const double PID_ANG_SAT1 = 130;
  const double PID_ANG_SAT2 = 130;
  const double PID_W_SAT1 = 380;
  const double PID_W_SAT2 = 380;
  pitchPID.input = pitch; // angulo pitch calculado del pitch
  rollPID.input = roll; // angulo roll calculado del pitch
  // Angulo de pitch, roll y yaw enviados por Lora, mapeados entre -30 y 30 º
  pitchPID.setPoint = map(pitch_angle_target, 0, 1023, 30, -30); // mapeado del pitch º de LoRa
  rollPID.setPoint = map(roll_angle_target, 0, 1023, -30, 30); // mapeado del roll º de LoRa
  yawRatePID.setPoint = map(yaw_angle_target, 0, 1023, -30, 30); // Mapeado del yaw º/s
   if (pitchPID.setPoint > -3 && pitchPID.setPoint < 3) {
    pitchPID.setPoint = 0;
  }
  if (rollPID.setPoint > -3 && rollPID.setPoint < 3) {
    rollPID.setPoint = 0;
  }
   if (yawRatePID.setPoint > -3 && yawRatePID.setPoint < 3) {
    yawRatePID.setPoint = 0;
  }
  // Calculo de los PID de angulos pitch y roll
  ang_Pitch_OUT = calculatePID(pitchPID, pitchPID.input, pitchPID.setPoint, PID_ANG_SAT1, PID_ANG_SAT2);
  ang_Roll_OUT = calculatePID(rollPID, rollPID.input, rollPID.setPoint, PID_ANG_SAT1, PID_ANG_SAT2);
  // CALCULO PID DE VELOCIDADES
  pitchRatePID.input = gx; // velocidad angular del pitch de la MPU
  rollRatePID.input = gy;  // velocidad angular del roll de la MPU
  yawRatePID.input = gz;  // velocidad angular del yaw de la MPU
  // Establecer puntos de consigna de velocidad angular
  pitchRatePID.setPoint = ang_Pitch_OUT;
  rollRatePID.setPoint = ang_Roll_OUT;
  // Calcular PID de velocidad angular
  w_Pitch_OUT = calculatePID(pitchRatePID, pitchRatePID.input, pitchRatePID.setPoint, -PID_W_SAT1, PID_W_SAT1);
  w_Roll_OUT = calculatePID(rollRatePID, rollRatePID.input, rollRatePID.setPoint, -PID_W_SAT1, PID_W_SAT1);
  w_Yaw_OUT = calculatePID(yawRatePID, yawRatePID.input, yawRatePID.setPoint, -PID_W_SAT1, PID_W_SAT1);
  // Mapeado del Throttle
  throttle_target_map = map (throttle_target, 0, 1023, 970, 1800);
  if (throttle_target_map <= 1400) {
    pitchPID.I = 0;
    rollPID.I = 0;
    yawRatePID.I  = 0;
    rollRatePID.I = 0;
    pitchRatePID.I = 0;
    ESC1 = throttle_target_map;
    ESC2 = throttle_target_map;
    ESC3 = throttle_target_map;
    ESC4 = throttle_target_map;
    // Si lo motores giran con el stick de Throttle al mínimo, recudir el valor de 950us
    if (ESC1 < 1000) ESC1 = 950;
    if (ESC2 < 1000) ESC2 = 950;
    if (ESC3 < 1000) ESC3 = 950;
    if (ESC4 < 1000) ESC4 = 950;
  }
  // Si el throttle es mayor a 1300us, el control de estabilidad se activa.
  else {
    // Limitar throttle a 1800 para dejar margen a los PID
    if (throttle_target_map > 1800) throttle_target_map = 1800;
    // Calcular valores PWM de los motores
    ESC1 = throttle_target_map + w_Pitch_OUT - w_Roll_OUT - w_Yaw_OUT;
    ESC2 = throttle_target_map + w_Pitch_OUT + w_Roll_OUT + w_Yaw_OUT;
    ESC3 = throttle_target_map - w_Pitch_OUT + w_Roll_OUT - w_Yaw_OUT;
    ESC4 = throttle_target_map - w_Pitch_OUT - w_Roll_OUT + w_Yaw_OUT;
    // Limitar valores PWM dentro del rango permitido
    ESC1 = constrain(ESC1, ESC_MIN, ESC_MAX);
    ESC2 = constrain(ESC2, ESC_MIN, ESC_MAX);
    ESC3 = constrain(ESC3, ESC_MIN, ESC_MAX);
    ESC4 = constrain(ESC4, ESC_MIN, ESC_MAX);
  }
    // Enviar señales PWM a los motores
    motor1.writeMicroseconds(ESC1);
    motor2.writeMicroseconds(ESC2);
    motor3.writeMicroseconds(ESC3);
    motor4.writeMicroseconds(ESC4);
  // Serial.print(" throttle_target_map:"); Serial.print(throttle_target_map);
  // Enviar datos PIDS POR SERIAL
  // PID ANGULO PITCH, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print("Pitch_angle:"); Serial.print(pitchPID.input);
  // Serial.print(" Pitch_target:"); Serial.print(pitchPID.setPoint);
  // Serial.print(", ang_Pitch_OUT:"); Serial.print(ang_Pitch_OUT);

  // PID ANGULO ROLL, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print(", Roll_angle:"); Serial.print(rollPID.input);
  // Serial.print(", Roll_target:"); Serial.print(rollPID.setPoint);
  // Serial.print(", ang_Roll_OUT:"); Serial.print(ang_Roll_OUT);
  
  // PID VELOCIDAD ANGULAR PITCH, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print(", Pitch_rate_angle:"); Serial.print(pitchRatePID.input);
  // Serial.print(", Pitch_rate_target:"); Serial.print(pitchRatePID.setPoint);
  // Serial.print(", w_Pitch_OUT:"); Serial.print(w_Pitch_OUT);

  // PID VELOCIDAD ANGULAR ROLL, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print(", Roll_rate_angle:"); Serial.print(rollRatePID.input);
  // Serial.print(", Roll_rate_target:"); Serial.print(rollRatePID.setPoint);
  // Serial.print(", w_Roll_OUT:"); Serial.print(w_Roll_OUT);

  // PID VELOCIDAD ANGULAR YAW, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print(", Yaw_rate_angle:"); Serial.print(yawRatePID.input);
  // Serial.print(", Yaw_rate_target:"); Serial.print(yawRatePID.setPoint);
  // Serial.print(", w_Yaw_OUT:"); Serial.print(w_Yaw_OUT);
  // int Max = 1000;
  // int Min = 2000;
  // SEÑALES PWM DE LOS MOTORES EN US
  // Serial.print("ESC1:"); Serial.print(ESC1);
  // Serial.print("ESC2:"); Serial.print(ESC2);
  // Serial.print("ESC3:"); Serial.print(ESC3);
  // Serial.print("ESC4:"); Serial.println(ESC4);
}

void pidControlMode() {
  // Calcular el valor del PID
  controlPID();
}

void motorTestMode() {
    // No hacer nada si la prueba ya ha completado
    if (testComplete) {
        return;
    }
    unsigned long currentTime = millis();
    // Comprobar si ha pasado el intervalo de tiempo
    if (currentTime - previousTime >= interval) {
        // Actualizar el tiempo anterior
        previousTime = currentTime;
        // Escribir el valor de PWM al motor
        motors[motorIndex].writeMicroseconds(pwmValues[pwmIndex]);
        // Serial.print("Motor "); Serial.print(motorIndex + 1); Serial.print(": "); Serial.println(pwmValues[pwmIndex]);
        // Actualizar el índice de PWM
        pwmIndex++;
        // Si hemos recorrido todos los valores de PWM para el motor actual
        if (pwmIndex >= 5) {
            // Resetear el índice de PWM y pasar al siguiente motor
            pwmIndex = 0;
            motorIndex++;
            // Si hemos recorrido todos los motores
            if (motorIndex >= 4) {
                // Apagar todos los motores
                for (int i = 0; i < 4; i++) {
                    motors[i].writeMicroseconds(0); // Apagar motor
                }
                // Indicar que la prueba ha terminado
                testComplete = true;
                // Serial.println("Test completado. Todos los motores apagados.");
            }
        }
    }
}

// Variable para gestionar el estado del modo actual
enum Mode { WAITING, CALIBRATION, MOTOR_TEST, PID_CONTROL };
Mode currentMode = WAITING;

void handleModes() {
  // Verificar si se debe realizar calibración
  if (button_A4 && !calibrationDone) {
    // BTSerial.println("Iniciando calibración...");
    currentMode = CALIBRATION;
    return;
  }

  // Si la calibración está completa y estamos en el modo CALIBRATION, cambiar al modo MOTOR_TEST
  if (calibrationDone && currentMode == CALIBRATION) {
    // BTSerial.println("Calibración completada. Cambiando a MOTOR_TEST...");
    currentMode = MOTOR_TEST;
    return;
  }

  // Cambiar al modo PID_CONTROL si se presiona el botón A5 en MOTOR_TEST
  if (button_A5 && currentMode == MOTOR_TEST) {
    // BTSerial.println("Botón A5 presionado. Cambiando a PID_CONTROL...");
    currentMode = PID_CONTROL;
    return;
  }

  // Cambiar al modo ESPERA DE NUEVO si se presiona el botón A4 en PID_CONTROL
  if (button_A4 && currentMode == PID_CONTROL) {
    // BTSerial.println("Botón A5 presionado. Cambiando a PID_CONTROL...");
    currentMode = WAITING;
    return;
  }
  // Manejo del modo actual
  switch (currentMode) {
    case WAITING:
      // BTSerial.println("Modo actual: ESPERA");
      // BTSerial.println("Inicializando sistema...");
      setRGBColor(1, 1, 1);
      testLEDs();
      setAllPWM(ESC_MIN);
      break;
    case CALIBRATION:
      // BTSerial.println("Modo actual: CALIBRATION");
      setRGBColor(1, 0, 0);
      testLEDs();
      calibrateESC();
      break;
    case MOTOR_TEST:
      // BTSerial.println("Modo actual: MOTOR_TEST");
      setRGBColor(0, 1, 0);
      motorTestMode();
      break;
    case PID_CONTROL:
      // BTSerial.println("Modo actual: PID_CONTROL");
      setRGBColor(0, 0, 1);
      pidControlMode();
      break;
    default:
      // BTSerial.println("Modo actual: DESCONOCIDO");
      break;
  }
}

void loop() {
  // Calculo de tiempo para el calculo de los angulso pitch y roll
  do {
    reading_time_seconds = (micros() - loop_timer)/1000;
  } while (reading_time_seconds < usCiclo/1000);
  loop_timer = micros();

  // Serial.print(F("Ciclo: ")); Serial.println(reading_time_seconds);
  // Leer datos de LoRa
  // readLoRa();
  // Enviar datos a través de Bluetooth
  sendBluetoothData();

  // Llamar a las funciones de lectura y procesamiento
  // readMPU6050();
  // processMPU6050();

  // // Función que administra los modos de funcionamiento del dron
  // // timer_f5 = micros();
  // handleModes();
  // timer_f5 = micros() - timer_f5;
  // Serial.print(F("- Handler timer: ")); Serial.println(timer_f5);
}
