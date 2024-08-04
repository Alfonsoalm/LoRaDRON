#include <MPU6500_WE.h>
#include <Wire.h>
#define MPU6500_ADDR 0x68

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);

const int filterSize = 10; // Tamaño de la ventana del filtro
float accXValues[filterSize];
float accYValues[filterSize];
float accZValues[filterSize];
float gyrXValues[filterSize];
float gyrYValues[filterSize];
float gyrZValues[filterSize];
int filterIndex = 0;

const float accMin = -2.0;  // Rango mínimo de aceleración en g
const float accMax = 2.0;   // Rango máximo de aceleración en g
const float gyrMin = -250.0; // Rango mínimo de giroscopio en º/s
const float gyrMax = 250.0;  // Rango máximo de giroscopio en º/s

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  if (!myMPU6500.init()) {
    Serial.println("MPU6500 does not respond");
  } else {
    Serial.println("MPU6500 is connected");
  }

  Serial.println("Position your MPU6500 flat and don't move it - calibrating...");
  delay(100); // Espera un poco antes de la calibración
  myMPU6500.autoOffsets(); // Calibrar y establecer offset automáticamente
  Serial.println("Done!");

  myMPU6500.enableGyrDLPF(); // Habilitar el Filtro de Paso Bajo (DLPF) del giroscopio
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6); // Configurar el DLPF del giroscopio
  myMPU6500.setSampleRateDivider(5); // Configurar el divisor de la tasa de muestreo
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250); // Configurar el rango del giroscopio a ±250 º/s
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_2G); // Configurar el rango del acelerómetro a ±2g
  myMPU6500.enableAccDLPF(true); // Habilitar el Filtro de Paso Bajo (DLPF) del acelerómetro
  myMPU6500.setAccDLPF(MPU6500_DLPF_6); // Configurar el DLPF del acelerómetro

  // Inicializar arrays del filtro
  for (int i = 0; i < filterSize; i++) {
    accXValues[i] = 0.0;
    accYValues[i] = 0.0;
    accZValues[i] = 0.0;
    gyrXValues[i] = 0.0;
    gyrYValues[i] = 0.0;
    gyrZValues[i] = 0.0;
  }

  delay(200); // Espera un poco para estabilizar
}

void loop() {
  xyzFloat gValue = myMPU6500.getGValues(); // Obtener valores de aceleración
  xyzFloat gyr = myMPU6500.getGyrValues(); // Obtener valores de giroscopio
  // float temp = myMPU6500.getTemperature(); // Obtener la temperatura

  // Calcular la aceleración resultante
  float resultantG = sqrt((gValue.x * gValue.x) + (gValue.y * gValue.y) + (gValue.z * gValue.z));

  // Limitar los valores al rango esperado
  gValue.x = constrain(gValue.x, accMin, accMax);
  gValue.y = constrain(gValue.y, accMin, accMax);
  gValue.z = constrain(gValue.z, accMin, accMax);
  gyr.x = constrain(gyr.x, gyrMin, gyrMax);
  gyr.y = constrain(gyr.y, gyrMin, gyrMax);
  gyr.z = constrain(gyr.z, gyrMin, gyrMax);

  // Agregar los valores actuales al filtro
  accXValues[filterIndex] = gValue.x;
  accYValues[filterIndex] = gValue.y;
  accZValues[filterIndex] = gValue.z;
  gyrXValues[filterIndex] = gyr.x;
  gyrYValues[filterIndex] = gyr.y;
  gyrZValues[filterIndex] = gyr.z;

  filterIndex = (filterIndex + 1) % filterSize; // Incrementar el índice circularmente

  // Calcular los valores promedio del filtro
  float accXAvg = 0.0, accYAvg = 0.0, accZAvg = 0.0;
  float gyrXAvg = 0.0, gyrYAvg = 0.0, gyrZAvg = 0.0;
  for (int i = 0; i < filterSize; i++) {
    accXAvg += accXValues[i];
    accYAvg += accYValues[i];
    accZAvg += accZValues[i];
    gyrXAvg += gyrXValues[i];
    gyrYAvg += gyrYValues[i];
    gyrZAvg += gyrZValues[i];
  }
  accXAvg /= filterSize;
  accYAvg /= filterSize;
  accZAvg /= filterSize;
  gyrXAvg /= filterSize;
  gyrYAvg /= filterSize;
  gyrZAvg /= filterSize;

  // Imprimir valores de aceleración
  Serial.print("Acc_X:"); Serial.println(accXAvg); // m/s2 en x
  Serial.print("Acc_Y:"); Serial.println(accYAvg); // m/s2 en y
  Serial.print("Acc_Z:"); Serial.println(accZAvg); // m/s2 en z
  Serial.print("ResultantG:"); Serial.println(resultantG); // m/s2 resultante
  
  // Imprimir valores de giroscopio
  Serial.print("Gyro_X:"); Serial.println(gyrXAvg); // Pitch º/s
  Serial.print("Gyro_Y:"); Serial.println(gyrYAvg); // Roll º/s
  Serial.print("Gyro_Z:"); Serial.println(gyrZAvg); // Yaw º/s
  
  // Comentado la impresión de la temperatura para este ejemplo
  // Serial.print("Temp_C:"); Serial.println(temp); // °C

  delay(100); // Pausa antes de la siguiente lectura
}
