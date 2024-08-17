#include <Wire.h>
#include <MPU6050.h>

#define usCiclo 6000
#define PI 3.14159265359
#define MPU6500_ADDR 0x68
#define MPU6050_ADDR 0x69

// Variables globales para almacenar datos
float pitch_6500, roll_6500, gx_6500, gy_6500, gz_6500;
float pitch_6050, roll_6050, gx_6050, gy_6050, gz_6050;
float gx_offset_6500, gy_offset_6500, gz_offset_6500;
float gx_offset_6050, gy_offset_6050, gz_offset_6050;
float ax_offset_6500, ay_offset_6500, az_offset_6500;
float ax_offset_6050, ay_offset_6050, az_offset_6050;
bool set_gyro_angles = false;
unsigned long loop_timer = 0, reading_time = 0, lastReadingTime = 0; // Tiempo de la última lectura en microsegundos

// Definir pesos
float weight_MPU6050 = 0.4;  // Peso para el MPU6050
float weight_MPU6500 = 0.6;  // Peso para el MPU6500
float pitch, roll, gx, gy, gz; // Inicializacion variables finales calculadas

void setup() {
  unsigned long startTime;
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  startTime = micros();
  setupMPU(MPU6500_ADDR);
  Serial.print(" setup_MPU6500:"); Serial.print(micros() - startTime); Serial.println(" us");  

  startTime = micros();
  setupMPU(MPU6050_ADDR);
  Serial.print(" setup_MPU6050:"); Serial.print(micros() - startTime); Serial.println(" us");  

  startTime = micros();
  calibrateMPU(MPU6500_ADDR, gx_offset_6500, gy_offset_6500, gz_offset_6500, ax_offset_6500, ay_offset_6500, az_offset_6500);
  Serial.print(" calibrate_MPU6500:"); Serial.print(micros() - startTime); Serial.println(" us");  
  
  startTime = micros();
  calibrateMPU(MPU6050_ADDR, gx_offset_6050, gy_offset_6050, gz_offset_6050, ax_offset_6050, ay_offset_6050, az_offset_6050);
  Serial.print(" calibrate_MPU6050:"); Serial.print(micros() - startTime); Serial.println(" us");  

}

void loop() {
  unsigned long startTime;
  while (micros() - loop_timer < usCiclo);
  loop_timer = micros();

  startTime = micros();
  getMPUData(MPU6500_ADDR, gx_offset_6500, gy_offset_6500, gz_offset_6500, ax_offset_6500, ay_offset_6500, az_offset_6500, pitch_6500, roll_6500, gx_6500, gy_6500, gz_6500);
  Serial.print(", getMPUdata_MPU6500:"); Serial.print(micros() - startTime); Serial.println(" us");  
  
  startTime = micros();
  getMPUData(MPU6050_ADDR, gx_offset_6050, gy_offset_6050, gz_offset_6050, ax_offset_6050, ay_offset_6050, az_offset_6050, pitch_6050, roll_6050, gx_6050, gy_6050, gz_6050);
  Serial.print(", getMPUdata_MPU6050:"); Serial.print(micros() - startTime); Serial.println(" us");  
  
  float avg_pitch, avg_roll, avg_gx, avg_gy, avg_gz;
  startTime = micros();
  getAveragedData(pitch_6050, roll_6050, gx_6050, gy_6050, gz_6050,
                  pitch_6500, roll_6500, gx_6500, gy_6500, gz_6500);
  Serial.print(", averagedData:"); Serial.print(micros() - startTime); Serial.println(" us");  
  
  Serial.print(" Avg Pitch:"); Serial.print(avg_pitch);
  Serial.print(", Avg Roll:"); Serial.print(avg_roll);
  Serial.print(", Avg gx:"); Serial.print(avg_gx);
  Serial.print(", Avg gy:"); Serial.print(avg_gy);
  Serial.print(", Avg gz:"); Serial.println(avg_gz);

  Serial.print(", min:"); Serial.print(-90); 
  Serial.print(", max:"); Serial.println(90);
  delay(10); // Espera 10 ms antes de la próxima lectura
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
    Wire.requestFrom(ADDR, 14);
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
                float &pitch, float &roll, float &gx, float &gy, float &gz) {
  int16_t ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw;
  float pitch_acc, roll_acc;
  // Leer datos del sensor
  Wire.beginTransmission(ADDR);
  Wire.write(0x3B); // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(ADDR, 14);
  if (Wire.available() == 14) {
    ax_raw = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    ay_raw = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    az_raw = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Wire.read(); Wire.read(); // Leer registros de temperatura (no usados)
    gx_raw = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy_raw = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz_raw = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    // Calibrar datos
    float ax = (ax_raw - ax_offset);
    float ay = (ay_raw - ay_offset);
    float az = (az_raw - az_offset);
    gx = (gx_raw - gx_offset) / 65.5;
    gy = (gy_raw - gy_offset) / 65.5;
    gz = (gz_raw - gz_offset) / 65.5;

    // Calcular el tiempo transcurrido desde la última lectura
    unsigned long currentReadingTime = micros();
    reading_time = currentReadingTime - lastReadingTime; // Tiempo transcurrido en microsegundos
    lastReadingTime = currentReadingTime; // Actualizar el tiempo de la última lectura


    // Calcular ángulos de inclinación con datos del giroscopio
    float dt = reading_time / 1000000.0; // Convertir microsegundos a segundos
    pitch += gx * dt;
    roll  += gy * dt;
    float gz_adjusted = gz * dt * PI / (65.5 * 180 * 1000);
    pitch += roll * sin(gz_adjusted);
    roll  -= pitch * sin(gz_adjusted);
    // Calcular ángulos de inclinación con datos del acelerómetro
    float acc_total_vector = sqrt(pow(ax, 2) + pow(ay, 2) + pow(az + 4096, 2));
    pitch_acc = asin((float)ay / acc_total_vector) * 180 / PI; // Convertir a grados
    roll_acc  = asin((float)ax / acc_total_vector) * -180 / PI; // Convertir a grados
    // Filtro complementario
    if (set_gyro_angles) {
      pitch = pitch * 0.995 + pitch_acc * 0.005;
      roll  = roll * 0.995 + roll_acc  * 0.005;
    } else {
      pitch = pitch_acc;
      roll  = roll_acc;
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
