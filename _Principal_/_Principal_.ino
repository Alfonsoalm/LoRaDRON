#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <LoRa.h>
#include <MPU6050.h>
#include <MPU6500_WE.h>
#include <PID_v1.h>
#include <SoftwareSerial.h>
#include <MsTimer2.h>
// Configuración del puerto serie para el módulo Bluetooth
// SoftwareSerial Serial(0, 1); // RX, TX - Asegúrate de conectar RX del HC-06 al pin 10 y TX al pin 11
#define PI 3.14159265358979323846
#define usCiclo 6000
#define MPU6500_ADDR 0x68
#define MPU6050_ADDR 0x69
#define BMP280_ADDR 0x76
#define ADS1115_ADDR 0x48

#define TCICLO_MS (6.0)
#define TCICLO_US (TCICLO_MS * 1000.0)
#define TCICLO_S (TCICLO_MS / 1000.0)

#define KP_ang (5) // 2, 11                 MIN: 1    -->   MAX: 20
#define KI_ang (KP_ang * 0.001) //0.1, 0.002    MIN: 0.0  -->   MAX: 0.05
#define KD_ang (KP_ang * 1) //5, 80           MIN: 12   -->   MAX: 90

#define KP_w (0.4) // 1
#define KI_w (KP_w * 0.01)  // 0.002
#define KD_w (KP_w * 0) // 0

#define KP_w_yaw (3) // 1
#define KI_w_yaw (KP_w_yaw * 0.005)  // 0.002
#define KD_w_yaw (KP_w_yaw * 0) // 0

#define PID_ANG_SAT_INTEGRAL (40 * KI_ang) // 130
#define PID_ANG_SAT_OUTPUT (2000) // 130
#define PID_W_SAT_INTEGRAL (4000 * KI_w) // 380
#define PID_W_SAT_OUTPUT (380) // 380

MPU6050 mpu;

// MPU6500_WE mpu_master = MPU6500_WE(MPU6500_ADDR);

// LORA -------------------------------->>>>
int yaw_angle_target, pitch_angle_target, roll_angle_target, throttle_target, throttle_target_map;
bool button_A4, button_A5;
// LORA --------------------------------<<<<

// LEDS -------------------------------->>>>
const int ledPins[4] = {3, 4, 5, 6}; 
const int numLeds = 4;
struct RGBPins {
    int redPin;
    int greenPin;
    int bluePin;
}; RGBPins rgbPins = {7, A5, A4};
const int LED_MIN_BRIGHTNESS = 0;
const int LED_MAX_BRIGHTNESS = 255;
const unsigned long delay_Time = 400;
unsigned long previousMillis = 0;
int currentLed = 0;
// LEDS --------------------------------<<<<

// MOTORES ----------------------------->>>>
// Definición de pines para los motores y LEDs
const int motorPins[4] = {A3, 9, 8, A2};
unsigned long tiempo_motores_start;
float ESC1 = 0, ESC2 = 0, ESC3 = 0, ESC4 = 0;
const int ESC_MIN = 900; const int ESC_MAX = 1900;
int pwm1, pwm2, pwm3, pwm4;
bool calibrationDone = false;
int previousThrottleValue = 0;
// MOTORES ------------------------------<<<<

// MPU ---------------------------------->>>>
// Variables globales para almacenar datos
float pitch_6500, roll_6500, gx_6500, gy_6500, gz_6500;
float pitch_6050, roll_6050, gx_6050, gy_6050, gz_6050;
float gx_offset_6500, gy_offset_6500, gz_offset_6500;
float gx_offset_6050, gy_offset_6050, gz_offset_6050;
float ax_offset_6500, ay_offset_6500, az_offset_6500;
float ax_offset_6050, ay_offset_6050, az_offset_6050;
bool set_gyro_angles = false;
unsigned long loop_timer = 0, reading_time = 0, lastReadingTime = 0;
float weight_MPU6050 = 0.4;  // Peso para el MPU6050
float weight_MPU6500 = 0.6;  // Peso para el MPU6500
float pitch, roll, gx, gy, gz;
// MPU ----------------------------------<<<<

// BAROMETRO ---------------------------->>>>
Adafruit_BMP280 bmp;
float temperature, pressure, altitude;
// BAROMETRO ----------------------------<<<<

// MEDICIÓN_BATERIA --------------------->>>>
Adafruit_ADS1115 ads;
float batteryVoltage, batteryPercent;
// MEDICIÓN_BATERIA ---------------------<<<<

// MOTOR_TEST_MODE ---------------------->>>>
int pwmValues[5] = {1000, 1300, 1500, 2000, 1000};
int motorIndex = 0; int pwmIndex = 0; 
unsigned long previousTime = 0;
const long interval = 500; // Intervalo de 5 ms
bool testComplete = false; // Bandera para indicar que la prueba ha terminado
// MOTOR_TEST_MODE ----------------------<<<<

// CONTROL PID ------------------------->>>>
struct PIDVariables {// Estructura para almacenar variables del PID
  double setPoint;
  double input;
  double output;
  double previousInput;
  double error;
  double P, I, D;
  double Kp, Ki, Kd;
  double integral;
};PIDVariables pitchPID, rollPID, pitchRatePID, rollRatePID, yawRatePID;
const double Kp_Pitch = KP_ang, Ki_Pitch = KI_ang, Kd_Pitch = KD_ang; // Kp_Pitch=0.5, Ki_Pitch=0.05, Kd_Pitch=10
const double Kp_Roll = KP_ang, Ki_Roll = KI_ang, Kd_Roll = KD_ang;  // Kp_Roll=0.5, Ki_Roll=0.05, Kd_Roll=10
const double Kp_w_Pitch = KP_w, Ki_w_Pitch = KI_w, Kd_w_Pitch = KD_w; // Kp_w_Pitch = 2.0, Ki_w_Pitch = 0.02, Kd_w_Pitch = 0
const double Kp_w_Roll = KP_w, Ki_w_Roll = KI_w, Kd_w_Roll = KD_w; // Kp_w_Roll = 2.0, Ki_w_Roll = 0.02, Kd_w_Roll = 0;
const double Kp_w_Yaw = KP_w_yaw, Ki_w_Yaw = KI_w_yaw, Kd_w_Yaw = KD_w_yaw; // Kp_w_Yaw = 1.0, Ki_w_Yaw = 0.05, Kd_w_Yaw = 0;
double w_Pitch_OUT = 0, w_Roll_OUT = 0, w_Yaw_OUT = 0;
double ang_Pitch_OUT = 0, ang_Roll_OUT = 0;

unsigned long time_handle, time_handle_start = 0;

// CONTROL_PID -------------------------<<<<

// Variable para gestionar el estado del modo actual
enum Mode { WAITING, CALIBRATION, MOTOR_TEST, PID_CONTROL, SAFETYBRAKE };
Mode currentMode = WAITING;

void setup() {
  // Establecemos frecuencia de envío de caracteres en 115200 baudios
  Serial.begin(115200);
  // Inicializamos bus I2C
  Wire.begin();
  // Establecemos frecuencia de muestreo del I2C en 400 kHz
  Wire.setClock(400000);
  // Configuracion de LoRa recepción de datos
  setupLoRa();      
  // Inicializamos IMU para mediciones angulares
  setupIMUs();
  // Inicializamos Barometro para altitud, presión a. y temperatura
  //setupBAR();       
  // Inicializamos ADC para medir bateria
  //setupADC();       
  // Configuracion de motores
  setupMotors();    
  // Configuracion de LEDS
  setupLEDs();      
  // Lectura de la bateria del dron
  // readBattery();    
  // if (batteryPercent < 15){
  //   Serial.println("   Bateria Baja, vuelva a casa !!");
  // }
  // Meter los parametros del PID
  setupPIDparams();
  // Setup timer
  MsTimer2::set(TCICLO_MS, handleModes);  // Configura Timer2 para generar una interrupción cada 6 ms
  MsTimer2::start();  // Inicia el temporizador

  loop_timer = millis();
}


void loop() {
  unsigned long startTime, prev_loop_timer;
  // Sincronización del ciclo
  while (micros() - loop_timer < 10000);
  prev_loop_timer = loop_timer; loop_timer = micros();
  //Serial.print("Tiempo bucle: "); Serial.println(micros() - prev_loop_timer);
  // Leer datos de LoRa
  startTime = micros();
  readLoRa();
  //Serial.print("Tiempo readLORA: "); Serial.println(micros() - startTime);
  // Enviar datos a través de Bluetooth
  startTime = micros();
  sendBluetoothData();
  //Serial.print("Tiempo SEND: ");Serial.println(micros() - startTime);
}

void setupIMUs(){
  unsigned long startTime;
  startTime = micros();
  setupMPU(MPU6500_ADDR);
  //Serial.print(" setup_MPU6500:"); Serial.print(micros() - startTime); Serial.println(" us");  

  startTime = micros();
  setupMPU(MPU6050_ADDR);
  //Serial.print(" setup_MPU6050:"); Serial.print(micros() - startTime); Serial.println(" us");  

  startTime = micros();
  calibrateMPU(MPU6500_ADDR, gx_offset_6500, gy_offset_6500, gz_offset_6500, ax_offset_6500, ay_offset_6500, az_offset_6500);
  //Serial.print(" calibrate_MPU6500:"); Serial.print(micros() - startTime); Serial.println(" us");  
  
  startTime = micros();
  calibrateMPU(MPU6050_ADDR, gx_offset_6050, gy_offset_6050, gz_offset_6050, ax_offset_6050, ay_offset_6050, az_offset_6050);
  //Serial.print(" calibrate_MPU6050:"); Serial.print(micros() - startTime); Serial.println(" us");  
}

void setupMPU(uint8_t ADDR) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 registro 6B hex
  Wire.write(0x00);  // 00000000 para activar
  Wire.endTransmission();

  Wire.beginTransmission(ADDR);
  Wire.write(0x1B);  // GYRO_CONFIG registro 1B hex
  Wire.write(0x08);  // 00001000: 500 dps
  Wire.endTransmission();

  Wire.beginTransmission(ADDR);
  Wire.write(0x1C);  // ACCEL_CONFIG registro 1C hex
  Wire.write(0x10);  // 00010000: +/- 8g
  Wire.endTransmission();

  // Filtro de hardware de 42 Hz para reducir ruido
  Wire.beginTransmission(ADDR);
  Wire.write(0x1A); // Registro CONFIG
  Wire.write(0x03); // Valor para 42 Hz de frecuencia de corte
  Wire.endTransmission();
  /*
    Frecuencia de corte del filtro pasa bajos:
    256Hz(0ms):0x00
    188Hz(2ms):0x01
    98Hz(3ms):0x02
    42Hz(4.9ms):0x03
    20Hz(8.5ms):0x04
    10Hz(13.8ms):0x05
    5Hz(19ms):0x06
  */
}

void calibrateMPU(uint8_t ADDR, float &gx_offset, float &gy_offset, float &gz_offset,
                   float &ax_offset, float &ay_offset, float &az_offset) {
  int16_t gx, gy, gz, ax, ay, az;
  gx_offset = 0; gy_offset = 0; gz_offset = 0;
  ax_offset = 0; ay_offset = 0; az_offset = 0;
  for (int i = 0; i < 4000; i++) {
    // Leer datos del sensor
    Wire.beginTransmission(ADDR);
    Wire.write(0x3B); // Pedir el registro 0x3B (AcX)
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)ADDR, (uint8_t)14);
    if (Wire.available() == 14) {
      // Leer acelerómetro
      ax = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
      ay = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      az = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      // Leer giroscopio
      Wire.read(); Wire.read(); // Leer registros de temperatura (no usados)
      gx = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      gy = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      gz = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      // Sumar valores para calcular el offset
      gx_offset += gx; gy_offset += gy; gz_offset += gz;
      ax_offset += ax; ay_offset += ay; az_offset += az;
    }
  }
  // Calcular los offsets promedio
  gx_offset /= 4000.0; gy_offset /= 4000.0; gz_offset /= 4000.0;
  ax_offset /= 4000.0; ay_offset /= 4000.0; az_offset /= 4000.0;
}

void getMPUData(uint8_t ADDR, float gx_offset, float gy_offset, float gz_offset,
                float ax_offset, float ay_offset, float az_offset, 
                float &pitch_tot, float &roll_tot, float &gx_tot, float &gy_tot, float &gz_tot) {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  float pitch_acc, roll_acc;
  // Leer datos del sensor
  Wire.beginTransmission(ADDR);
  Wire.write(0x3B); // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)ADDR, (uint8_t)14);
  if (Wire.available() == 14) {
    ax_raw = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    ay_raw = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    az_raw = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Wire.read(); Wire.read(); // Leer registros de temperatura (no usados)
    gx_raw = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy_raw = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz_raw = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    float ax = (ax_raw - ax_offset);
    float ay = (ay_raw - ay_offset);
    float az = (az_raw - az_offset);
    gx_tot = (gx_raw - gx_offset) / 65.5;
    gy_tot = (gy_raw - gy_offset) / 65.5;
    gz_tot = (gz_raw - gz_offset) / 65.5;
    // Calcular el tiempo transcurrido desde la última lectura
    unsigned long currentReadingTime = micros();
    reading_time = currentReadingTime - lastReadingTime; // Tiempo transcurrido en microsegundos
    lastReadingTime = currentReadingTime; // Actualizar el tiempo de la última lectura
    // Calcular ángulos de inclinación con datos del giroscopio
    float dt = reading_time / 1000000.0; // Convertir microsegundos a segundos
    pitch_tot += gx_tot * dt;
    roll_tot  += gy_tot * dt;
    float gz_adjusted = gz * dt * PI / (65.5 * 180 * 1000);
    pitch_tot += roll_tot * sin(gz_adjusted);
    roll_tot  -= pitch_tot * sin(gz_adjusted);
    // Calcular ángulos de inclinación con datos del acelerómetro
    float acc_total_vector = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az + 4096, 2));
    pitch_acc = asin((float)ay / acc_total_vector) * 180 / PI; // Convertir a grados
    roll_acc  = asin((float)ax / acc_total_vector) * -180 / PI; // Convertir a grados
    // Filtro complementario
    if (set_gyro_angles) {
      pitch_tot = pitch_tot * 0.995 + pitch_acc * 0.005;
      roll_tot  = roll_tot * 0.995 + roll_acc  * 0.005;
    } else {
      pitch_tot = pitch_acc;
      roll_tot  = roll_acc;
      set_gyro_angles = true;
    }
  }
}

void getAveragedData(float pitch_6050, float roll_6050, float gx_6050, float gy_6050, float gz_6050,
                     float pitch_6500, float roll_6500, float gx_6500, float gy_6500, float gz_6500) {
    pitch = pitch_6050 * weight_MPU6050 + pitch_6500 * weight_MPU6500;
    roll  = roll_6050 * weight_MPU6050 + roll_6500 * weight_MPU6500;
    gx    = gx_6050 * weight_MPU6050 + gx_6500 * weight_MPU6500;
    gy    = gy_6050 * weight_MPU6050 + gy_6500 * weight_MPU6500;
    gz    = gz_6050 * weight_MPU6050 + gz_6500 * weight_MPU6500;
}

void setupBAR(){
  // Inicializar BMP280
  if (!bmp.begin(BMP280_ADDR)) {
    Serial.println(F("No se encontró un sensor BMP280 válido"));
    while (1);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);
  Serial.println(F("BMP280 inicializado"));
}

void setupADC(){
  // Inicializar ADS1115
  if (!ads.begin(ADS1115_ADDR)) {
    Serial.println(F("No se encontró el ADS1115"));
    while (1);
  }
  ads.setGain(GAIN_TWOTHIRDS);
  Serial.println(F("ADS1115 inicializado"));
}

void setupLoRa() {
  // inicializar y configurar modulo lora a 433 Mhz
  if (!LoRa.begin(433E6)) {
    Serial.println(F("Error al iniciar LoRa"));    
  } else {
    LoRa.setSpreadingFactor(7);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    Serial.println(F("LoRa inicializado"));
  }
}

void setupMotors() {
  for (int i = 0; i < 4; i++) {
    pinMode(motorPins[i], OUTPUT);
    digitalWrite(motorPins[i], LOW);
  }
  Serial.println(F("Motores inicializados"));
}

void setupLEDs() {               
  // Configura todos los pines de LEDs como salida
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
  pinMode(rgbPins.redPin, OUTPUT);
  pinMode(rgbPins.greenPin, OUTPUT);
  pinMode(rgbPins.bluePin, OUTPUT);
  Serial.println(F("LEDs inicializados"));
}

void setupPIDparams(){
  pitchPID.Kp = Kp_Pitch; pitchPID.Ki = Ki_Pitch; pitchPID.Kd = Kd_Pitch;
  rollPID.Kp = Kp_Roll; rollPID.Ki = Ki_Roll; rollPID.Kd = Kd_Roll;
  pitchRatePID.Kp = Kp_w_Pitch; pitchRatePID.Ki = Ki_w_Pitch; pitchRatePID.Kd = Kd_w_Pitch;
  rollRatePID.Kp = Kp_w_Roll; rollRatePID.Ki = Ki_w_Roll; rollRatePID.Kd = Kd_w_Roll;
  yawRatePID.Kp = Kp_w_Yaw; yawRatePID.Ki = Ki_w_Yaw; yawRatePID.Kd = Kd_w_Yaw;
}

void testLEDs() {
  unsigned long currentMillis = millis();
  if (currentMode == WAITING) {
    // Modo WAITING: Enciende un LED a la vez, en secuencia.
    if (currentMillis - previousMillis >= delay_Time) {
      previousMillis = currentMillis;     // Actualiza el tiempo anterior
      for (int i = 0; i < numLeds; i++) { // Apaga todos los LEDs
        digitalWrite(ledPins[i], LOW);
      }
      digitalWrite(ledPins[currentLed], HIGH); // Enciende el LED actual
      currentLed++; // Actualiza el LED actual
      if (currentLed >= numLeds) { // Reinicia el ciclo al llegar al último LED
        currentLed = 0;
      }
    }

  } else if (currentMode == CALIBRATION) {
    // Modo CALIBRATION: Enciende todos los LEDs, espera, y luego los apaga todos.
    if (currentMillis - previousMillis >= delay_Time) {
      previousMillis = currentMillis; // Actualiza el tiempo anterior
      static bool ledsOn = false; // Estado de los LEDs (encendidos o apagados)

      if (ledsOn) {
        // Apaga todos los LEDs
        for (int i = 0; i < numLeds; i++) {
          digitalWrite(ledPins[i], LOW);
        }
      } else {
        // Enciende todos los LEDs
        for (int i = 0; i < numLeds; i++) {
          digitalWrite(ledPins[i], HIGH);
        }
      }
      ledsOn = !ledsOn; // Alterna entre encender y apagar los LEDs
    }
  }
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

void readBMP280() {
  temperature = bmp.readTemperature();
  pressure = bmp.readPressure();
  altitude = bmp.readAltitude(1013.25); // Ajustar al valor de presión local
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
  /*yaw_angle_target = 0;
  pitch_angle_target = 0;
  roll_angle_target = 0;
  throttle_target = 0;*/
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
      //Serial.print("B4: "); Serial.println(button_A4);
      //Serial.print(", B5: "); Serial.println(button_A5);
    } else {
      Serial.println(F("Error en el tamaño del paquete recibido."));
    }
  }
}

char buffer[512];  // Asegúrate de que el tamaño del buffer sea suficiente

void sendBluetoothData() {
  Serial.print(F(" PITCH:")); Serial.println(-pitch); Serial.print(", PITCH_target:"); Serial.println(map(pitch_angle_target, 0, 1023, 35, -35)); Serial.print(", PITCH_output:"); Serial.println(ang_Pitch_OUT);
  Serial.print(F(", ROLL:")); Serial.println(roll); Serial.print(", ROLL_target:"); Serial.println(map(roll_angle_target, 0, 1023, 35, -35)); Serial.print(", ROLL_output:"); Serial.println(ang_Roll_OUT);
  Serial.print(", PITCH_RATE:"); Serial.println(gx); Serial.print(", PITCH_RATE_target:"); Serial.println(ang_Pitch_OUT); Serial.print(", PITCH_RATE_output:"); Serial.println(w_Pitch_OUT);
  Serial.print(", ROLL_RATE:"); Serial.println(gy); Serial.print(", ROLL_RATE_target:"); Serial.println(ang_Roll_OUT); Serial.print(", ROLL_RATE_output:"); Serial.println(w_Roll_OUT);
  Serial.print(", THROTTLE_target:"); Serial.println(map (throttle_target, 0, 1023, 970, 1800));
  Serial.print(F(", YAW:")); Serial.println(gz); Serial.print(", YAW_target:"); Serial.println(map(yaw_angle_target, 0, 1023, -35, 35));
  Serial.print(", Error_pitch_pid:"); Serial.println(pitchPID.error); Serial.print(", Error_roll_pid:"); Serial.println(rollPID.error);
  Serial.print(", Error_pitch_rate_pid:"); Serial.println(pitchRatePID.error); Serial.print(", Error_roll_rate_pid:"); Serial.println(rollRatePID.error); Serial.print(", Error_yaw_rate_pid:"); Serial.println(yawRatePID.error);
  if (currentMode == PID_CONTROL){
    Serial.print(F(", ESC1:")); Serial.println(ESC1);
    Serial.print(F(", ESC2:")); Serial.println(ESC2); 
    Serial.print(F(", ESC3:")); Serial.println(ESC3); 
    Serial.print(F(", ESC4:")); Serial.println(ESC4);
    Serial.print(F(", MAX:")); Serial.println(2000); 
    Serial.print(F(", MIN:")); Serial.println(900);  
  }
  Serial.print(", Tiempo handle: ");Serial.println(time_handle);
  switch (currentMode) {

    case WAITING:
      Serial.println(F("Modo actual: ESPERA"));
      break;

    case CALIBRATION:
      Serial.println(F("Modo actual: CALIBRATION"));
      break;

    case MOTOR_TEST:
      Serial.println(F("Modo actual: MOTOR_TEST"));
      break;

    case PID_CONTROL:
      Serial.println(F("Modo actual: PID_CONTROL"));
      break;

    case SAFETYBRAKE:
      Serial.println(F("Modo actual: SAFETYBRAKE"));
      break;

    default:
      Serial.println(F("Modo actual: DESCONOCIDO"));
      break;
  }
}

void calibrateESC() {
  //Serial.println(F("Iniciando calibración de ESC..."));
  // Mapeo del throttle
  throttle_target_map = map(throttle_target, 0, 1023, 900, 2000);
  //Serial.print("Throttle: "); Serial.println(throttle_target_map);
  ESC1 = ESC2 = ESC3 = ESC4 = throttle_target_map;  // Asignar el mismo valor a todos los motores
  // Iniciar señales PWM
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], HIGH);
  }
  tiempo_motores_start = micros();
  // Controlar señales PWM
  unsigned long endCycleTime = tiempo_motores_start + ESC_MAX;
  while (micros() < endCycleTime) {
    unsigned long currentTime = micros();
    if (currentTime - tiempo_motores_start >= ESC1) digitalWrite(motorPins[0], LOW);
    if (currentTime - tiempo_motores_start >= ESC2) digitalWrite(motorPins[1], LOW);
    if (currentTime - tiempo_motores_start >= ESC3) digitalWrite(motorPins[2], LOW);
    if (currentTime - tiempo_motores_start >= ESC4) digitalWrite(motorPins[3], LOW);
  }
  //Serial.println(F("Calibración de ESC completada."));
  calibrationDone = true;
  setLEDs(LED_MIN_BRIGHTNESS);  // Apagar todos los LEDs después de la calibración
}

void setAllPWM_anag(int pwmValue) {
    int pwmMapped = map(pwmValue, ESC_MIN, ESC_MAX, 100, 255);  // Mapeado de los pines PWM para las señales de los motores
    analogWrite(motorPins[0], pwmMapped);
    analogWrite(motorPins[1], pwmMapped);
    analogWrite(motorPins[2], pwmMapped);
    analogWrite(motorPins[3], pwmMapped);
}

void setAllPWM(int Value) {
  if (Value == 1){
    //digitalWrite(motorPins[0], HIGH);
    //digitalWrite(motorPins[1], HIGH);
    //digitalWrite(motorPins[2], HIGH);
    //digitalWrite(motorPins[3], HIGH);
    ESC1 = ESC2 = ESC3 = ESC4 = ESC_MAX; 
    envio_PWM();

  } else{
    //digitalWrite(motorPins[0], LOW);
    //digitalWrite(motorPins[1], LOW);
    //digitalWrite(motorPins[2], LOW);
    //digitalWrite(motorPins[3], LOW);
    ESC1 = ESC2 = ESC3 = ESC4 = ESC_MIN; 
    envio_PWM();
  }
}

void setLEDs(int value) {
  for (int i = 0; i < numLeds; i++) {  // Establecer el valor de los LEDS analogicos en función
    analogWrite(ledPins[i], value);
  }
}

double calculatePID(PIDVariables &pid, double input, double setPoint, double integralSat, double outputSat) {
  // Calculo del error y de los valores de la parte proporcional e integral
  pid.error = setPoint - input;
  pid.P = pid.error; // pid.P = pid.Kp * pid.error;
  pid.I += pid.Ki * pid.error;
  // Enfoque antiwindup 1 (Comentado)
  pid.D = pid.Kd * (input - pid.previousInput);// pid.I = constrain(pid.I, -integralSat, integralSat); // **Comentar con enfoque 2**
  if (pid.I < -integralSat) {
    pid.I = -integralSat;
  } else if (pid.I > integralSat) {
    pid.I = integralSat;
  }
  pid.output = pid.Kp * (pid.P + pid.I + pid.D);  // pid.output = constrain(pid.output, -outputSat, outputSat); // **Comentar con enfoque 2**
  //Enfoque antiwindup 2
  if (pid.output < -outputSat) {
    pid.output = -outputSat;
    pid.I -= pid.Ki * pid.error;
  } else if (pid.output > outputSat) {
    pid.output = outputSat;
    pid.I -= pid.Ki * pid.error;
  }
  pid.previousInput = input;  // Asignación de valor actual al previo para proximo ciclo
  return pid.output; 
}

void pidControlMode() {
  // Asignar valores de las entradas¡a los PID de angulos
  pitchPID.input = -pitch;
  rollPID.input = roll;
  // Mapeado de las referencias a valores entre -35 y 35º
  pitchPID.setPoint = map(pitch_angle_target, 0, 1023, 35, -35);
  rollPID.setPoint = map(roll_angle_target, 0, 1023, 35, -35);
  yawRatePID.setPoint = map(yaw_angle_target, 0, 1023, -35, 35);
  // Establecer valores pequeños en cero para evitar mover dron
  if (pitchPID.setPoint > -3 && pitchPID.setPoint < 3) {
    pitchPID.setPoint = 0;
  } if (rollPID.setPoint > -3 && rollPID.setPoint < 3) {
    rollPID.setPoint = 0;
  } if (yawRatePID.setPoint > -3 && yawRatePID.setPoint < 3) {
    yawRatePID.setPoint = 0;}

  // Establecer puntos de consigna de velocidad angular
  // Mapeado del Throttle
  throttle_target_map = map (throttle_target, 0, 1023, 0, 1850);
  // Calcular valores PWM de los motores y limitarlos
  if (throttle_target_map <= /*1180*/ 1150) {
    pitchPID.I = 0; rollPID.I = 0; yawRatePID.I  = 0; rollRatePID.I = 0; pitchRatePID.I = 0;
    pitchPID.previousInput = pitchPID.input; rollPID.previousInput = rollPID.input; yawRatePID.previousInput = yawRatePID.input;
    pitchRatePID.previousInput = pitchRatePID.input; rollRatePID.previousInput = rollRatePID.input;
    ESC1 = constrain(throttle_target_map, ESC_MIN, ESC_MAX);   
    ESC2 = constrain(throttle_target_map, ESC_MIN, ESC_MAX);
    ESC3 = constrain(throttle_target_map, ESC_MIN, ESC_MAX);
    ESC4 = constrain(throttle_target_map, ESC_MIN, ESC_MAX);
  } else {// Si el throttle es mayor a 1300us, el control de estabilidad se activa.

    // Calculo de los PID de angulos pitch y roll
    ang_Pitch_OUT = calculatePID(pitchPID, pitchPID.input, pitchPID.setPoint, PID_ANG_SAT_INTEGRAL, PID_ANG_SAT_OUTPUT);
    ang_Roll_OUT = calculatePID(rollPID, rollPID.input, rollPID.setPoint, PID_ANG_SAT_INTEGRAL, PID_ANG_SAT_OUTPUT);
    // Asignación de velocidades a los PID de velocidades
    pitchRatePID.input = -gx; // velocidad angular del pitch de la MPU
    rollRatePID.input = gy;  // velocidad angular del roll de la MPU
    yawRatePID.input = gz;  // velocidad angular del yaw de la MPU
    pitchRatePID.setPoint = ang_Pitch_OUT;
    rollRatePID.setPoint = ang_Roll_OUT;

    w_Pitch_OUT = calculatePID(pitchRatePID, pitchRatePID.input, pitchRatePID.setPoint, PID_W_SAT_INTEGRAL, PID_W_SAT_OUTPUT);
    w_Roll_OUT = calculatePID(rollRatePID, rollRatePID.input, rollRatePID.setPoint, PID_W_SAT_INTEGRAL, PID_W_SAT_OUTPUT);
    w_Yaw_OUT = calculatePID(yawRatePID, yawRatePID.input, yawRatePID.setPoint, PID_W_SAT_INTEGRAL, PID_W_SAT_OUTPUT);

    ESC1 = constrain(throttle_target_map + w_Pitch_OUT + w_Roll_OUT + w_Yaw_OUT, 1100, ESC_MAX); // - w_Yaw_OUT;
    ESC2 = constrain(throttle_target_map + w_Pitch_OUT - w_Roll_OUT - w_Yaw_OUT, 1100, ESC_MAX); // + w_Yaw_OUT;
    ESC3 = constrain(throttle_target_map - w_Pitch_OUT - w_Roll_OUT + w_Yaw_OUT, 1100, ESC_MAX); // - w_Yaw_OUT;
    ESC4 = constrain(throttle_target_map - w_Pitch_OUT + w_Roll_OUT - w_Yaw_OUT, 1100, ESC_MAX); // + w_Yaw_OUT;

    if((ESC1 <= 1100)||(ESC1 >= ESC_MAX)) {
      pitchRatePID.I -= pitchRatePID.Ki * pitchRatePID.error;
      rollRatePID.I -= rollRatePID.Ki * rollRatePID.error;
      yawRatePID.I -= yawRatePID.Ki * yawRatePID.error;
    } 
    if((ESC2 <= 1100)||(ESC2 >= ESC_MAX)) {
      pitchRatePID.I -= pitchRatePID.Ki * pitchRatePID.error;
      rollRatePID.I -= rollRatePID.Ki * rollRatePID.error;
      yawRatePID.I -= yawRatePID.Ki * yawRatePID.error;
    } 
    if((ESC3 <= 1100)||(ESC3 >= ESC_MAX)) {
      pitchRatePID.I -= pitchRatePID.Ki * pitchRatePID.error;
      rollRatePID.I -= rollRatePID.Ki * rollRatePID.error;
      yawRatePID.I -= yawRatePID.Ki * yawRatePID.error;
    } 
    if((ESC4 <= 1100)||(ESC4 >= ESC_MAX)) {
      pitchRatePID.I -= pitchRatePID.Ki * pitchRatePID.error;
      rollRatePID.I -= rollRatePID.Ki * rollRatePID.error;
      yawRatePID.I -= yawRatePID.Ki * yawRatePID.error;
    } 
  }
  // Funcion de envio de los valores PWM a los motores mapeados 100-255
  envio_PWM();
}

void envio_PWM() {
  // Para generar las 4 señales PWM, el primer paso es poner estas señales a 1 (HIGH).
  for (int i = 0; i < 4; i++) {
    digitalWrite(motorPins[i], HIGH);
  }
  tiempo_motores_start = micros();
  // Controlar señales PWM
  unsigned long endCycleTime = tiempo_motores_start + ESC_MAX;
  while (micros() < endCycleTime) {
    unsigned long currentTime = micros();
    if (currentTime - tiempo_motores_start >= ESC1) digitalWrite(motorPins[0], LOW);
    if (currentTime - tiempo_motores_start >= ESC2) digitalWrite(motorPins[1], LOW);
    if (currentTime - tiempo_motores_start >= ESC3) digitalWrite(motorPins[2], LOW);
    if (currentTime - tiempo_motores_start >= ESC4) digitalWrite(motorPins[3], LOW);
  }
}

void safetyBrakeMode(){
    Serial.println("Modo actual: SafetyBrake");
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
}

void motorTestMode() {
    // No hacer nada si la prueba ya ha completado
    if (testComplete) {
        return;
    }
    unsigned long currentTime = millis();
    // Comprobar si ha pasado el intervalo de tiempo
    if (currentTime - previousTime >= interval) {
        previousTime = currentTime; // Actualizar tiempo anterior
        int pwmValue = map(pwmValues[pwmIndex], ESC_MIN, ESC_MAX, 100, 255);
        analogWrite(motorPins[motorIndex], pwmValue);
        pwmIndex++; // Actualizar el índice de PWM
        if (pwmIndex >= 5) {
            pwmIndex = 0;
            motorIndex++; 
            if (motorIndex >= 4) { // Apagar los motores al pasar indices
                for (int i = 0; i < 4; i++) {
                    analogWrite(motorPins[i], 0);}
                testComplete = true;// Indicar que la prueba ha terminado
                Serial.println("Test completado. Todos los motores apagados.");
            }
        }
    }
}


void handleModes() {
  sei();
  time_handle_start = micros();
  if (currentMode != CALIBRATION){
    getMPUData(MPU6500_ADDR, gx_offset_6500, gy_offset_6500, gz_offset_6500, ax_offset_6500, ay_offset_6500, az_offset_6500, pitch_6500, roll_6500, gx_6500, gy_6500, gz_6500);
    getMPUData(MPU6050_ADDR, gx_offset_6050, gy_offset_6050, gz_offset_6050, ax_offset_6050, ay_offset_6050, az_offset_6050, pitch_6050, roll_6050, gx_6050, gy_6050, gz_6050);
    getAveragedData(pitch_6050, roll_6050, gx_6050, gy_6050, gz_6050,
                    pitch_6500, roll_6500, gx_6500, gy_6500, gz_6500);
  }
  // Manejo del modo actual
  switch (currentMode) {
    case WAITING:
      // Serial.println("Inicializando sistema...");
      // Serial.print(" button_A4:"); Serial.print(button_A4);
      // Serial.print(", button_A5:"); Serial.println(button_A5);
      //setRGBColor(0, 1, 1); // Ponemos color BLANCO en el RGB Led
      //testLEDs();
      if (button_A4) {
        // Serial.println(F("------------------------------------"));
        // Serial.println(F("Cuidado !!, Iniciando calibración..."));
        // Serial.println(F("------------------------------------"));
        currentMode = CALIBRATION;
        return;
      }
      break;

    case CALIBRATION:
      //setRGBColor(0, 1, 0); // Ponemos color Verde en el RGB Led
      //testLEDs();
      calibrateESC();
      // Si la calibración está completa y estamos en el modo CALIBRATION, cambiar al modo MOTOR_TEST
      if (button_A5) {
        // Serial.println(F("--------------------------------------------------"));
        // Serial.println(F("Calibración completada. Cambiando a PID_CONTROL..."));
        // Serial.println(F("--------------------------------------------------"));
        currentMode = PID_CONTROL;
        return;
      }
      break;

    case MOTOR_TEST:
      //setRGBColor(0, 0, 1); // Ponemos color Azul en el RGB Led
      motorTestMode();
      if (button_A4) {
        // Serial.println(F("-----------------------------------"));
        //Serial.println(F("Cuidado !!, Cambiando a PID_MODE..."));
        //Serial.println(F("-----------------------------------"));
        currentMode = PID_CONTROL;
        return;
      }
      break;

    case PID_CONTROL:
      //setRGBColor(0, 1, 0); // Ponemos color Azul Verdoso en el RGB Led
      pidControlMode();
      // Cambiar al modo PID_CONTROL si se presiona el botón A5 en MOTOR_TEST
      if (button_A4) {
        //Serial.println(F("----------------------------------------------"));
        //Serial.println(F("Apagando motores y Cambiando a SAFETY_BRAKE..."));
        //Serial.println(F("----------------------------------------------"));
        currentMode = SAFETYBRAKE;
        return;
      }
      break;

    case SAFETYBRAKE:
      //setRGBColor(0, 0, 0); // Ponemos color Rojo en el RGB Led
      safetyBrakeMode();
      delay(3000);
      if (button_A4) {
        Serial.println(F("-------------------------------------"));
        Serial.println(F("Reiniciado modos de funcionamiento..."));
        Serial.println(F("-------------------------------------"));
        currentMode = WAITING;
        return;
      }
      break;
    default:
      //Serial.println(F("Modo actual: DESCONOCIDO"));
      break;
  }
    time_handle = micros()-time_handle_start;
}
