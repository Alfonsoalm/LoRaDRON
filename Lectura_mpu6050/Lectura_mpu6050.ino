/*
   Leer sensor MPU6050 (datos raw o sin procesar)
   Más información en https://arduproject.es/mpu6050-y-su-programacion/
*/

#define usCiclo 5000  // Ciclo de ejecución de software en microsegundos
#include <Wire.h>

// MPU6050
#define MPU6050_adress 0x69

// Variables globales
float gyro_Z, gyro_X, gyro_Y, temperature;
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
float acc_X_cal, acc_Y_cal, acc_Z_cal;
float angulo_pitch, angulo_roll;
float ax, ay, az;  // Declarar las variables del acelerómetro como globales
int gx, gy, gz;    // Declarar las variables del giroscopio como globales
bool set_gyro_angles = false, accCalibOK = false;
long tiempo_ejecucion, loop_timer;

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Iniciar y calibrar el sensor MPU6050
  MPU6050_iniciar();
  calibrar_MPU6050();

  loop_timer = micros();
}

void loop() {
  // Esperar hasta que se complete el ciclo de ejecución
  while (micros() - loop_timer < usCiclo);

  tiempo_ejecucion = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  MPU6050_leer();     // Leer sensor MPU6050
  MPU6050_procesar(); // Procesar datos del sensor MPU6050

  int valor_max = 50;
  int valor_min = -50;

  // Monitor Serie
  // Serial.print("valor_max:"); Serial.print(valor_max);
  // Serial.print(", angulo_pitch:"); Serial.print(angulo_pitch);
  // Serial.print(", angulo_roll:"); Serial.print(angulo_roll);
  // Serial.print(", valor_min:"); Serial.println(valor_min);
}

// Iniciar sensor MPU6050
void MPU6050_iniciar() {
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x6B);  // PWR_MGMT_1 registro 6B hex
  Wire.write(0x00);  // 00000000 para activar
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);  // GYRO_CONFIG registro 1B hex
  Wire.write(0x08);  // 00001000: 500dps
  Wire.endTransmission();
  
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1C);  // ACCEL_CONFIG registro 1C hex
  Wire.write(0x10);  // 00010000: +/- 8g
  Wire.endTransmission();
}

// Calibrar sensor MPU6050
void calibrar_MPU6050() {
  gyro_X_cal = gyro_Y_cal = gyro_Z_cal = 0;
  acc_X_cal = acc_Y_cal = acc_Z_cal = 0;

  for (int cal_int = 0; cal_int < 3000; cal_int++) {
    MPU6050_leer();  // Leer sensor MPU6050
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    acc_X_cal  += ax;
    acc_Y_cal  += ay;
    acc_Z_cal  += az;
    delayMicroseconds(50); // Mantener una espera para obtener lecturas estables
  }

  gyro_X_cal /= 3000;
  gyro_Y_cal /= 3000;
  gyro_Z_cal /= 3000;
  acc_X_cal  /= 3000;
  acc_Y_cal  /= 3000;
  acc_Z_cal  /= 3000;
  accCalibOK = true;
}

// Leer sensor MPU6050
void MPU6050_leer() {
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x3B);  // Pedir el registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_adress, 14);  // Solicitar un total de 14 registros

  if (Wire.available() == 14) {
    ax = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    ay = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    az = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    temperature = Wire.read() << 8 | Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    gx = Wire.read() << 8 | Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    gy = Wire.read() << 8 | Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    gz = Wire.read() << 8 | Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  }
  // Serial.print("ax:"); Serial.print(ax);
  // Serial.print(", ay:"); Serial.print(ay);
  // Serial.print(", az:"); Serial.print(az);
  // Serial.print(", gx:"); Serial.println(gx);
  // Serial.print(", gy:"); Serial.print(gy);
  // Serial.print(", gz:"); Serial.println(gz);
}

// Procesar datos del sensor MPU6050
void MPU6050_procesar() {
  // Restar valores de calibración del acelerómetro
  ax -= acc_X_cal;
  ay -= acc_Y_cal;
  az -= acc_Z_cal;
  az += 4096;

  // Restar valores de calibración del giroscopio y calcular velocidad angular en º/s
  gyro_X = (gx - gyro_X_cal) / 65.5;
  gyro_Y = (gy - gyro_Y_cal) / 65.5;
  gyro_Z = (gz - gyro_Z_cal) / 65.5;

  // Calcular ángulo de inclinación con datos del giroscopio
  angulo_pitch += gyro_X * tiempo_ejecucion / 1000;
  angulo_roll += gyro_Y * tiempo_ejecucion / 1000;
  angulo_pitch += angulo_roll * sin((gz - gyro_Z_cal) * tiempo_ejecucion * 0.000000266); // pi/(65.5*180*1000)
  angulo_roll -= angulo_pitch * sin((gz - gyro_Z_cal) * tiempo_ejecucion * 0.000000266);

  // Calcular vector de aceleración
  float acc_total_vector = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  float angulo_pitch_acc = asin((float)ay / acc_total_vector) * 57.2958;
  float angulo_roll_acc = asin((float)ax / acc_total_vector) * -57.2958;

  // Filtro complementario
  if (set_gyro_angles) {
    angulo_pitch = angulo_pitch * 0.99 + angulo_pitch_acc * 0.01;
    angulo_roll = angulo_roll * 0.99 + angulo_roll_acc * 0.01;
  } else {
    angulo_pitch = angulo_pitch_acc;
    angulo_roll = angulo_roll_acc;
    set_gyro_angles = true;
  }
  Serial.print("angulo_pitch:");
  Serial.print(angulo_pitch);
  Serial.print(", angulo_roll:");
  Serial.println(angulo_roll);

}
