#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <LoRa.h>
#include <MPU6050.h>
#include <MPU6500_WE.h>
#include <PID_v1.h>

#define PI 3.14159265358979323846
#define usCiclo 5000  // Ciclo de ejecución de software en microsegundos
#define MPU6500_ADDR 0x68
#define MPU6050_ADDR 0x69
#define BMP280_ADDR 0x76
#define ADS1115_ADDR 0x48

MPU6050 mpu;
// MPU6500_WE mpu_master = MPU6500_WE(MPU6500_ADDR);

// -------------------------------------------------------------------------------------------------------------------------------------------------
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
const double Kp_Pitch = 2.0, Ki_Pitch = 0.05, Kd_Pitch = 3;
const double Kp_Roll = 2.0, Ki_Roll = 0.05, Kd_Roll = 3;

// Parámetros PID de velocidad angular
const double Kp_w_Pitch = 2.0, Ki_w_Pitch = 5.0, Kd_w_Pitch = 1.0;
const double Kp_w_Roll = 2.0, Ki_w_Roll = 5.0, Kd_w_Roll = 1.0;
const double Kp_wyaw = 2.0, Ki_wyaw = 5.0, Kd_wyaw = 1.0;

// Variables de salida de los PID de velocidad angular
double w_Pitch_OUT = 0, w_Roll_OUT = 0, w_Yaw_OUT = 0;
double ang_Pitch_OUT = 0, ang_Roll_OUT = 0;

// Variables de LoRa
int yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target;
bool button_A4, button_A5;

// Variables PWM de los motores
float ESC1, ESC2, ESC3, ESC4;
const int ESC_MIN = 1000; const int ESC_MAX = 1600;

// ------------------------------------------------------
// ------------------- LEDS -----------------------------
// Pines donde están conectados los LEDs
const int ledPins[] = {3, 4, 5, 6}; // Actualiza con los pines correctos si es necesario

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
// ------------------- LEDS -----------------------------
// ------------------------------------------------------


// ------------------------------------------------------
// ---------------- MOTORES -----------------------------
// Definición de pines para los motores y LEDs
const int motor1Pin = A1; //A3
const int motor2Pin = A0; //A1
const int motor3Pin = A3; //A2
const int motor4Pin = A2; //A0

// Instanciacion de motores brushless RS2205
Servo motor1, motor2, motor3, motor4;

// ----------------- MOTORES ----------------------------
// ------------------------------------------------------

// ------------------------------------------------------
// ------------------------- MPU-------------------------
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

long  loop_timer, reading_time_seconds;
bool set_gyro_angles;

// ------------------------- MPU-------------------------
// ------------------------------------------------------


// Instacia de BMP280
Adafruit_BMP280 bmp;

// Instacia de ADS1115
Adafruit_ADS1115 ads;

// Variables medidas del barometro BMP280
float temperature, pressure, altitude;

// Variables de la medicion de la bateria por el ADS1117
float batteryVoltage, batteryPercent;

// Calibración de ESC
bool calibrationDone = false;

// Variable para gestionar el estado del modo actual
enum Mode { WAITING, CALIBRATION, MOTOR_TEST, PID_CONTROL };
Mode currentMode = WAITING;


// ---------------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------------
//  FUNCION DE SETUP PRINCIPAL
// ---------------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Inicializar sensores y comunicación
  setupSensors();
  setupLoRa();
  setupMotors();
  setupLEDs();

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

  yawRatePID.Kp = Kp_wyaw;
  yawRatePID.Ki = Ki_wyaw;
  yawRatePID.Kd = Kd_wyaw;

  loop_timer = micros();

}
// -------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------------------------------
//  FIN FUNCION DE SETUP PRINCIPAL
// -------------------------------------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------------------------------------------------------------------------------------------------



// FUNCION DE ENCENDIDO DE LOS LEDS SEÑALIZADORES
// -------------------------------------------------------------------------------------------------------------------------------------------------
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
// -------------------------------------------------------------------------------------------------------------------------------------------------


// INICIALIZACION DE LOS SENSORES
// -------------------------------------------------------------------------------------------------------------------------------------------------
void setupSensors() {
  // Inicializar MPU6500 
  Wire.begin();
  // mpu.initialize();
  // Serial.println(mpu.testConnection() ? F("IMU iniciado correctamente") : F("Error al iniciar IMU"));
  // if (!mpu_master.init()) {
  //   Serial.println(F("MPU6500 master no responde"));
  // } else {
  //   Serial.println(F("MPU6500 master conectado"));
  // }
  // Inicializar y configurar MPU6500 (maestro)
  // mpu_master.autoOffsets(); // Calibrar y establecer offset automáticamente
  // mpu_master.enableGyrDLPF(); // Habilitar el Filtro de Paso Bajo (DLPF) del giroscopio
  // mpu_master.setGyrDLPF(MPU6500_DLPF_6); // Configurar el DLPF del giroscopio
  // mpu_master.setSampleRateDivider(5); // Configurar el divisor de la tasa de muestreo
  // mpu_master.setGyrRange(MPU6500_GYRO_RANGE_250); // Configurar el rango del giroscopio a ±250 º/s
  // mpu_master.setAccRange(MPU6500_ACC_RANGE_2G); // Configurar el rango del acelerómetro a ±2g
  // mpu_master.enableAccDLPF(true); // Habilitar el Filtro de Paso Bajo (DLPF) del acelerómetro
  // mpu_master.setAccDLPF(MPU6500_DLPF_6); // Configurar el DLPF del acelerómetro
  // Serial.println(F("MPU6500 (maestro) inicializado"));

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
  Serial.println(F("MPU6050 (esclavo) inicializado"));

  // Calibracion MPU6500 maestra
  // calibrarMPU6500();

  // Calibracion MPU6050 esclava
  calibrarMPU6050();

  // Inicializar BMP280
  if (!bmp.begin(BMP280_ADDR)) {
    Serial.println(F("No se encontró un sensor BMP280 válido"));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  Serial.println(F("BMP280 inicializado"));

  // Inicializar ADS1115
  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println(F("No se encontró el ADS1115"));
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);
  Serial.println(F("ADS1115 inicializado"));
}
// -------------------------------------------------------------------------------------------------------------------------------------------------

// INICIALIZACION MODULO LORA SX1278 DE RECEPCIÓN DATOS
// -------------------------------------------------------------------------------------------------------------------------------------------------
void setupLoRa() {
  if (!LoRa.begin(433E6)) {
    // Serial.println(F("Error al iniciar LoRa"));
    while (1);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  Serial.println(F("LoRa inicializado"));
}

// INICIALIZACION DE LOS MOTORES RS2205 / ESC
// -------------------------------------------------------------------------------------------------------------------------------------------------
void setupMotors() {
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
  delay(1000);
  // Serial.println(F("Motores inicializados"));
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// INICIALIZACION DE LOS LEDS SEÑALIZADORES
// -------------------------------------------------------------------------------------------------------------------------------------------------
void setupLEDs() {
  // Configura todos los pines de LEDs como salida
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
  // Serial.println(F("LEDs inicializados"));
}
// -------------------------------------------------------------------------------------------------------------------------------------------------

// LEER Y FILTRAR DATOS DEL ACELEROMETRO + GIROSCOPIO MPU6050 y NMP6500
// -------------------------------------------------------------------------------------------------------------------------------------------------
void readMPU6050(){
  // Los datos del giroscopio y el acelerómetro se encuentran de la dirección 3B a la 14
  Wire.beginTransmission(MPU6050_ADDR);       // Empezamos comunicación
  Wire.write(0x3B);                             // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_ADDR, 14);         // Solicitar un total de 14 registros

  if (Wire.available() == 14) {
    ax = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    ay = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    az = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temperature = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  }

  // // Guardar datos en un una lista para filtro de media
  // accXValues[filterIndex] = ax; accYValues[filterIndex] = ay; accZValues[filterIndex] = az;
  // gyrXValues[filterIndex] = gx; gyrYValues[filterIndex] = gy; gyrZValues[filterIndex] = gz;
   
  // // Calculo del indice del filtro y calculo de valores del filtro
  // filterIndex = (filterIndex + 1) % filterSize;

  // for (int i = 0; i < filterSize; i++) {
  //   ax_avg += accXValues[i]; ay_avg += accYValues[i]; az_avg += accZValues[i];
  //   gx_avg += gyrXValues[i]; gy_avg += gyrYValues[i]; gz_avg += gyrZValues[i];
  // }

  // // Division por el tamaño del filtro para hacer media
  // ax_avg /= filterSize; ay_avg /= filterSize; az_avg /= filterSize;
  // gx_avg /= filterSize; gy_avg /= filterSize; gz_avg /= filterSize;

  ax_avg = ax; ay_avg = ay; az_avg = az;
  gx_avg = gx; gy_avg = gy; gz_avg = gz;
}

// -------------------------------------------------------------------------------------------------------------------------------------------------
void readMPU6500() {
  // Leer los datosd del acelerometro y giroscopio
  // xyzFloat acc = mpu_master.getGValues(); // Obtener valores de aceleración
  // xyzFloat gyr = mpu_master.getGyrValues(); // Obtener valores de giroscopio

  // // Guardar datos en un una lista para filtro de media
  // accXValues_master[filterIndex] = acc.x; accYValues_master[filterIndex] = acc.y; accZValues_master[filterIndex] = acc.z;
  // gyrXValues_master[filterIndex] = gyr.x; gyrYValues_master[filterIndex] = gyr.y; gyrZValues_master[filterIndex] = gyr.z;
   
  // // Calculo del indice del filtro y calculo de valores del filtro
  // filterIndex = (filterIndex + 1) % filterSize;
  // ax_avg_master = ay_avg_master = az_avg_master = 0.0;
  // wx_avg_master = wy_avg_master = wz_avg_master = 0.0;

  // for (int i = 0; i < filterSize; i++) {
  //   ax_avg_master += accXValues_master[i]; ay_avg_master += accYValues_master[i]; az_avg_master += accZValues_master[i];
  //   wx_avg_master += gyrXValues_master[i]; wy_avg_master += gyrYValues_master[i]; wz_avg_master += gyrZValues_master[i];
  // }

  // // Division por el tamaño del filtro para hacer media
  // ax_avg_master /= filterSize; ay_avg_master /= filterSize; az_avg_master /= filterSize;
  // wx_avg_master /= filterSize; wy_avg_master /= filterSize; wz_avg_master /= filterSize;
  // ax_avg_master = acc.x; ay_avg_master = acc.y; az_avg_master = acc.z;
  // wx_avg_master = gyr.x; wy_avg_master = gyr.y; wz_avg_master = gyr.z;
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// CALIBRACION DE MPUS
// -------------------------------------------------------------------------------------------------------------------------------------------------
// CALIBRACION DEL ACELEROMETRO + GIROSCOPIO MPU6050 (esclavo)
void calibrarMPU6050() {
  gx_offset = gy_offset = gz_offset = 0;
  ax_offset = ay_offset = az_offset = 0;

  Serial.print(F("Calibrando MPU6050..."));

  for (int cal_int = 0; cal_int < 3000; cal_int++) {
    readMPU6050();
    gx_offset += gx;
    gy_offset += gy;
    gz_offset += gz;
    ax_offset += ax;
    ay_offset += ay;
    az_offset += az;
    
    delayMicroseconds(50);
  }

  gx_offset /= 3000;
  gy_offset /= 3000;
  gz_offset /= 3000;
  ax_offset /= 3000;
  ay_offset /= 3000;
  az_offset /= 3000;

  Serial.println(F("Calibración MPU6050 terminada."));
}

// CALIBRACION DEL ACELEROMETRO + GIROSCOPIO MPU6500 (maestro)
void calibrarMPU6500() {
  // wx_offset_master = wy_offset_master = wz_offset_master = 0;
  // ax_offset_master = ay_offset_master = az_offset_master = 0;
  // Serial.print(F("Calibrando MPU6500..."));

  // for (int cal_int = 0; cal_int < 3000; cal_int++) {
  //   readMPU6500();
  //   wx_offset_master += wx_avg_master;
  //   wy_offset_master += wy_avg_master;
  //   wz_offset_master += wz_avg_master;
  //   ax_offset_master += ax_avg_master;
  //   ay_offset_master += ay_avg_master;
  //   az_offset_master += az_avg_master;
    
  //   delayMicroseconds(50);
  // }

  // wx_offset_master /= 3000;
  // wy_offset_master /= 3000;
  // wz_offset_master /= 3000;
  // ax_offset_master /= 3000;
  // ay_offset_master /= 3000;
  // az_offset_master /= 3000;

  // Serial.println(F("Calibración MPU6500 terminada."));
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// PROCESAMIENTO DATOS DEL ACELEROMETRO + GIROSCOPIO MPU6050 y MPU6500
// -------------------------------------------------------------------------------------------------------------------------------------------------
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


  // Serial.print("pitch_slave:"); Serial.print(pitch);
  // Serial.print(", roll_slave:"); Serial.println(roll);
}


// PROCESAMIENTO MPU6500 y CALCULO DE ANGULOS YAW, PITCH, ROLL
// ------------------------------------------------------------------------------------------------------------------------------------------------
void processMPU6500() {
  // reading_time_seconds = (micros() - loop_timer) / 1000;

  // // Restar valores de calibración del acelerómetro
  // ax_master = (ax_avg_master - ax_offset_master);
  // ay_master = (ay_avg_master - ay_offset_master);
  // az_master = (az_avg_master - az_offset_master); // Ajuste para la aceleración en Z

  // // Restar valores de calibración del giroscopio y calcular 
  // wx_master = (wx_avg_master - wx_offset_master) / 65.5; // Conversion de sensibilidad implicita
  // wy_master = (wy_avg_master - wy_offset_master) / 65.5;
  // wz_master = (wz_avg_master - wz_offset_master) / 65.5; 

  // // Calcular angulos de inclinación con datos del giroscopio
  // pitch_gyr_master += wx_master * reading_time_seconds / 1000;
  // roll_gyr_master  += wy_master * reading_time_seconds / 1000;
  // yaw_gyr_slave   = wz_master * reading_time_seconds / 1000;

  // Convertir velocidad angular a radianes por segundo
  // float pitch_gyr_master_rad = pitch_gyr_master * PI / 180.0;
  // float roll_gyr_master_rad = roll_gyr_master * PI / 180.0;
  // float yaw_gyr_slave_rad = yaw_gyr_slave * PI / 180.0;

  // pitch_gyr_master += roll_gyr_master * sin((wz_avg_master - wz_offset_master) * reading_time_seconds * PI/(180*1000*65.5));
  // roll_gyr_master  -= pitch_gyr_master * sin((wz_avg_master - wz_offset_master) * reading_time_seconds * PI/(180*1000*65.5));

  // loop_timer = micros();

  // // Calcular angulos de inclinación con datos del acelerometro
  // float acc_total_vector = sqrt(pow(ax_master, 2) + pow(ay_master, 2) + pow(az_master, 2));
  // float pitch_acc = asin((float)ay_master / acc_total_vector) * 180 / PI; // Convertir a grados
  // float roll_acc  = asin((float)ax_master / acc_total_vector) * -180 / PI; // Convertir a grados

  // // Filtro complementario
  // if (set_gyro_angles_master) {
  //   pitch_master = pitch_gyr_master * 0.98 + pitch_acc * 0.02;
  //   roll_master  = roll_gyr_master  * 0.98 + roll_acc  * 0.02;
  // } else {
  //   pitch_master = pitch_acc;
  //   roll_master  = roll_acc;
  //   set_gyro_angles_master = true;
  // }
  // Serial.print(", pitch_gyr_master:"); Serial.print(pitch_gyr_master);
  // Serial.print(", roll_gyr_master:"); Serial.println(roll_gyr_master);


  // Serial.print("pitch_master:"); Serial.print(pitch_master);
  // Serial.print(", roll_master:"); Serial.println(roll_master);

}
// -------------------------------------------------------------------------------------------------------------------------------------------------

// FUNCION DE LECTURA DE LOS DATOS DEL BAROMETRO BMP280
// -------------------------------------------------------------------------------------------------------------------------------------------------
void readBMP280() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(1013.25);  // Ajustar al valor de presión local
}
// -------------------------------------------------------------------------------------------------------------------------------------------------

// LECTURA DEL VOLTAJE DEL BATERIA
// ------------------------------------------------------------------------------------------------------------------------------------------------
void readBattery() {
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float voltage = adc0 * 0.1875 / 1000.0; // Convertir la lectura ADC a voltaje (ADS1115 tiene una resolución de 0.1875mV por bit)
  batteryVoltage = voltage * (11.1 / 5.0); // Ajustar por el divisor de voltaje (suponiendo que el divisor reduce a 5V para 11.1V de batería)
  batteryPercent = map(batteryVoltage, 9.0, 12.6, 0, 100); // Asumiendo un rango típico de 9.0V (descargada) a 12.6V (completamente cargada)
  batteryPercent = constrain(batteryPercent, 0, 100); // Asegurar que el porcentaje esté entre 0 y 100%

  // Serial.print(F("Voltaje de batería: "));
  // Serial.print(batteryVoltage);
  // Serial.print(F("V, Porcentaje: "));
  // Serial.print(batteryPercent);
  // Serial.println(F("%"));
}
// -------------------------------------------------------------------------------------------------------------------------------------------------

// LECTURA DE DATOS LORA
// -------------------------------------------------------------------------------------------------------------------------------------------------
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
      // Serial.print(F(", Button_A4:")); Serial.print(button_A4);
      // Serial.print(F(", Button_A5:")); Serial.println(button_A5);
    } else {
      // Serial.println(F("Error en el tamaño del paquete recibido."));
    }
  }
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// CALIBRACION DE LOS ESC DE LOS MOTORES
// -------------------------------------------------------------------------------------------------------------------------------------------------
void calibrateESC() {
  Serial.println(F("Iniciando calibración de ESC..."));
  setAllPWM(ESC_MAX); // Establece todos los ESC en el valor máximo para la calibración
  delay(2000);        // Mantén el valor máximo por 2 segundos
  setAllPWM(ESC_MIN); // Luego, establece todos los ESC en el valor mínimo
  delay(2000);        // Mantén el valor mínimo por 2 segundos
  Serial.println(F("Calibración de ESC completada."));
  calibrationDone = true;

  // Apagar todos los LEDs después de la calibración
  setLEDs(LED_MIN_BRIGHTNESS);  
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// FUNCION DE ENVIO DE DATOS A TODOS LOS MOTORES POR IGUAL
// -------------------------------------------------------------------------------------------------------------------------------------------------
void setAllPWM(int pwmValue) {
  motor1.writeMicroseconds(pwmValue);
  motor2.writeMicroseconds(pwmValue);
  motor3.writeMicroseconds(pwmValue);
  motor4.writeMicroseconds(pwmValue);
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// FUNCION DE ESTABLECER VALOR A LEDS
// -------------------------------------------------------------------------------------------------------------------------------------------------
void setLEDs(int value) {
  for (int i = 0; i < numLeds; i++) {
    analogWrite(ledPins[i], value);
  }
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------------------------------------------
// CALCULO DE LOS VALORES DE LOS PID
// -------------------------------------------------------------------------------------------------------------------------------------------------
// Función genérica para calcular PID
double calculatePID(PIDVariables &pid, double input, double setPoint, double integralSat, double outputSat) {
  pid.error = setPoint - input;
  pid.P = pid.Kp * pid.error;
  pid.I += pid.Ki * pid.error;
  pid.I = constrain(pid.I, -integralSat, integralSat);
  pid.D = pid.Kd * (input - pid.previousInput);
  pid.output = pid.P + pid.I + pid.D;

  pid.output = constrain(pid.output, -outputSat, outputSat);

  pid.previousInput = input;

  return pid.output;
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// -------------------------------------------------------------------------------------------------------------------------------------------------
// Función para calcular los PID y actualizar los motores
void controlPID() {
  // Constantes de saturación para los PID
  const double PID_ANG_SAT1 = 130;
  const double PID_ANG_SAT2 = 130;
  const double PID_W_SAT1 = 380;
  const double PID_W_SAT2 = 380;

  // CALCULO PID DE ANGULOS
  pitchPID.input = pitch; // angulo pitch calculado del pitch
  rollPID.input = roll; // angulo roll calculado del pitch

  // Angulo de pitch, roll y yaw enviados por Lora, mapeados entre -30 y 30 º

  pitchPID.setPoint = map(pitch_angle_target, 0, 1023, -30, 30); // mapeado del pitch º de LoRa
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


  // Serial.print("Pitch_P:"); Serial.print(pitchPID.P);
  Serial.print(", Pitch_I:"); Serial.print(pitchPID.I);
  // Serial.print(", Pitch_D:"); Serial.print(pitchPID.D);
  // Serial.print(", Roll_P:"); Serial.print(rollPID.P);
  // Serial.print(", Roll_I:"); Serial.print(rollPID.I);
  // Serial.print(", Roll_D:"); Serial.print(rollPID.D);

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
  throttle_target = map (throttle_target, 0, 1023, ESC_MIN, ESC_MAX);

  if (throttle_target <= 1300) {
    pitchPID.I = 0;
    rollPID.I = 0;
    yawRatePID.I  = 0;
    rollRatePID.I = 0;
    pitchRatePID.I = 0;

    ESC1 = throttle_target;
    ESC2 = throttle_target;
    ESC3 = throttle_target;
    ESC4 = throttle_target;

    // Si lo motores giran con el stick de Throttle al mínimo, recudir el valor de 950us
    if (ESC1 < 1000) ESC1 = 950;
    if (ESC2 < 1000) ESC2 = 950;
    if (ESC3 < 1000) ESC3 = 950;
    if (ESC4 < 1000) ESC4 = 950;
  }

  // Si el throttle es mayor a 1300us, el control de estabilidad se activa.
  else {
    // Limitar throttle a 1800 para dejar margen a los PID
    if (throttle_target > 1800) throttle_target = 1800;
  
    // Calcular valores PWM de los motores
    ESC1 = throttle_target + w_Pitch_OUT - w_Roll_OUT - w_Yaw_OUT;
    ESC2 = throttle_target + w_Pitch_OUT + w_Roll_OUT - w_Yaw_OUT;
    ESC3 = throttle_target - w_Pitch_OUT + w_Roll_OUT - w_Yaw_OUT;
    ESC4 = throttle_target - w_Pitch_OUT - w_Roll_OUT + w_Yaw_OUT;

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

  
  // Enviar datos PIDS POR SERIAL

  // // PID ANGULO PITCH, ENTRADA, REFERENCIA Y SALIDA
  Serial.print(", Pitch_angle:"); Serial.print(pitchPID.input);
  // Serial.print(", Pitch_target:"); Serial.print(pitchPID.setPoint);
  Serial.print(", ang_Pitch_OUT:"); Serial.print(ang_Pitch_OUT);

  // // PID ANGULO ROLL, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print(", Roll_angle:"); Serial.print(rollPID.input);
  // Serial.print(", Roll_target:"); Serial.print(rollPID.setPoint);
  // Serial.print(", ang_Roll_OUT:"); Serial.print(ang_Roll_OUT);
  
  // // PID VELOCIDAD ANGULAR PITCH, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print(", Pitch_rate_angle:"); Serial.print(pitchRatePID.input);
  // Serial.print(", Pitch_rate_target:"); Serial.print(pitchRatePID.setPoint);
  // Serial.print(", w_Pitch_OUT:"); Serial.print(w_Pitch_OUT);

  // // PID VELOCIDAD ANGULAR ROLL, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print(", Roll_rate_angle:"); Serial.print(rollRatePID.input);
  // Serial.print(", Roll_rate_target:"); Serial.print(rollRatePID.setPoint);
  // Serial.print(", w_Roll_OUT:"); Serial.print(w_Roll_OUT);

  // // PID VELOCIDAD ANGULAR YAW, ENTRADA, REFERENCIA Y SALIDA
  // Serial.print(", Yaw_rate_angle:"); Serial.print(yawRatePID.input);
  // Serial.print(", Yaw_rate_target:"); Serial.print(yawRatePID.setPoint);
  // Serial.print(", w_Yaw_OUT:"); Serial.print(w_Yaw_OUT);

  int Max = 2000;
  int Min = 1000;
  // SEÑALES PWM DE LOS MOTORES EN US
  // Serial.print(", Max:"); Serial.print(Max);
  // Serial.print(", Min:"); Serial.print(Min);
  // Serial.print(", ESC1:"); Serial.print(ESC1);
  // Serial.print(", ESC2:"); Serial.print(ESC2);
  // Serial.print(", ESC3:"); Serial.print(ESC3);
  // Serial.print(", ESC4:"); Serial.println(ESC4);
}
// -------------------------------------------------------------------------------------------------------------------------------------------------

// FUNCION DEL MODO DE CONTROL AUTOMATICO PID
// -------------------------------------------------------------------------------------------------------------------------------------------------
void pidControlMode() {
  // Calcular el valor del PID
  controlPID();
}
// -------------------------------------------------------------------------------------------------------------------------------------------------


// FUNCION DEL MODO DE PRUEBA DE LOS MOTORES
// -------------------------------------------------------------------------------------------------------------------------------------------------
void motorTestMode() {
    // Enviar valores a los motores
    int motor1Value = constrain(map(yaw_angle_target, 0, 1023, ESC_MIN, ESC_MAX), ESC_MIN, ESC_MAX);
    int motor2Value = constrain(map(pitch_angle_target, 0, 1023, ESC_MIN, ESC_MAX), ESC_MIN, ESC_MAX);
    int motor3Value = constrain(map(roll_angle_target, 0, 1023, ESC_MIN, ESC_MAX), ESC_MIN, ESC_MAX);
    int motor4Value = constrain(map(throttle_target, 0, 1023, ESC_MIN, ESC_MAX), ESC_MIN, ESC_MAX);

    motor1.writeMicroseconds(motor1Value);
    motor2.writeMicroseconds(motor2Value);
    motor3.writeMicroseconds(motor3Value);
    motor4.writeMicroseconds(motor4Value);

    // Ajustar el brillo de los LEDs en función del PWM de los motores
    int ledBrightness[] = {
      map(motor1Value, ESC_MIN, ESC_MAX, LED_MIN_BRIGHTNESS, LED_MAX_BRIGHTNESS),
      map(motor2Value, ESC_MIN, ESC_MAX, LED_MIN_BRIGHTNESS + 40, LED_MAX_BRIGHTNESS),
      map(motor3Value, ESC_MIN, ESC_MAX, LED_MIN_BRIGHTNESS, LED_MAX_BRIGHTNESS),
      map(motor4Value, ESC_MIN, ESC_MAX, LED_MIN_BRIGHTNESS, LED_MAX_BRIGHTNESS)
    };

    // Aplicar el brillo calculado a cada LED
    for (int i = 0; i < numLeds; i++) {
      analogWrite(ledPins[i], ledBrightness[i]);
    }
}  
// -------------------------------------------------------------------------------------------------------------------------------------------------


// FUNCION DE MANEJO DE LOS MODOS DEL DRON
// -------------------------------------------------------------------------------------------------------------------------------------------------
void handleModes() {
  // Verificar si se debe realizar calibración
  if (button_A4 && !calibrationDone) {
    // Serial.println("Iniciando calibración...");
    currentMode = CALIBRATION;
    return; // Salir para evitar cambiar a otro modo inmediatamente
  }

  // Si la calibración está completa y estamos en el modo CALIBRATION, cambiar al modo MOTOR_TEST
  if (calibrationDone && currentMode == CALIBRATION) {
    // Serial.println("Calibración completada. Cambiando a MOTOR_TEST...");
    currentMode = MOTOR_TEST;
    return; // Salir para evitar cambiar a otro modo inmediatamente
  }

  // Cambiar al modo PID_CONTROL si se presiona el botón A5 en MOTOR_TEST
  if (button_A5 && currentMode == MOTOR_TEST) {
    // Serial.println("Botón A5 presionado. Cambiando a PID_CONTROL...");
    currentMode = PID_CONTROL;
    return; // Salir para evitar cambiar a otro modo inmediatamente
  }

  // Manejo del modo actual
  switch (currentMode) {
    case WAITING:
      Serial.println("Modo actual: ESPERA");
      Serial.println("Inicializando sistema...");
      // Prueba inicial de los leds mientras espera
      testLEDs();
      // Puede ser un modo de espera del dron
      break;

    case CALIBRATION:
      Serial.println("Modo actual: CALIBRATION");
      // Prueba inicial de los leds mientras calibra
      testLEDs();
      // Calibracion de los ESC
      calibrateESC();
      // Marcar calibración como completada
      calibrationDone = true;
      break;

    case MOTOR_TEST:
      Serial.println("Modo actual: MOTOR_TEST");
      motorTestMode();
      break;

    case PID_CONTROL:
      Serial.println("Modo actual: PID_CONTROL");
      pidControlMode();
      break;

    default:
      // Serial.println("Modo actual: DESCONOCIDO");
      break;
  }
}
// -------------------------------------------------------------------------------------------------------------------------------------------------

// -------------------------------------------------------------------------------------------------------------------------------------------------
// 
// FUNCION PRINCIPAL LOOP
// 
// -------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {

  reading_time_seconds = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  // Leer datos de LoRa
  readLoRa();

  Serial.print(F("Yaw:")); Serial.print(yaw_angle_target);
  Serial.print(F(", Pitch:")); Serial.print(pitch_angle_target);
  Serial.print(F(", Roll:")); Serial.print(roll_angle_target);
  Serial.print(F(", Throttle:")); Serial.println(throttle_target);

  // Leer y procesar MPU master
  // readMPU6500();
  // processMPU6500();

  // Leer y procesar MPU slave
  readMPU6050();
  processMPU6050(); 

  // Leer Barómetro
  // readBMP280();

  handleModes();

  // Esperar un momento antes de la siguiente lectura
  // delay(10);
}
// -------------------------------------------------------------------------------------------------------------------------------------------------
// 
// FIN FUNCION PRINCIPAL LOOP
// 
// -------------------------------------------------------------------------------------------------------------------------------------------------

