#include <Servo.h>

// Definición de pines para los motores
const int motor1Pin = A0;  // Pin para motor 1
const int motor2Pin = A1;  // Pin para motor 2
const int motor3Pin = A2;  // Pin para motor 3
const int motor4Pin = A3;  // Pin para motor 4

Servo motor1, motor2, motor3, motor4;

// Valores PWM para calibración de ESC
const int ESC_MIN = 1000;  // Valor mínimo del PWM
const int ESC_MAX = 2000;  // Valor máximo del PWM

bool calibrationDone = false;

void setup() {
  Serial.begin(9600);  // Iniciar comunicación serial
  
  // Attach Servo objects to motor pins
  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
  
  // Calibrar los ESC
  calibrateESC();
}

void loop() {
  if (!calibrationDone) {
    // Esperar hasta que la calibración esté completa
    return;
  }
  
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');  // Leer comando del puerto serie
    command.trim(); // Eliminar cualquier espacio en blanco adicional

    // Determinar el motor y el valor PWM deseado
    if (command.startsWith("motor")) {
      int motorNumber = command.substring(5, 6).toInt();  // Extraer el número del motor del comando
      int pwmValue = command.substring(7).toInt();  // Leer el valor PWM desde el puerto serie

      if (motorNumber >= 1 && motorNumber <= 5) {
        // Asegurar que el valor PWM esté en el rango válido (1000 a 2000)
        pwmValue = constrain(pwmValue, 1000, 2000);
        
        // Asignar el valor PWM al motor correspondiente
        switch (motorNumber) {
          case 1:
            motor1.writeMicroseconds(pwmValue);
            break;
          case 2:
            motor2.writeMicroseconds(pwmValue);
            break;
          case 3:
            motor3.writeMicroseconds(pwmValue);
            break;
          case 4:
            motor4.writeMicroseconds(pwmValue);
            break;
          case 5:
            motor1.writeMicroseconds(1000);
            motor2.writeMicroseconds(1000);
            motor3.writeMicroseconds(1000);
            motor4.writeMicroseconds(1000);
            break;
          default:
            break;
        }
        
        // Confirmación del ajuste
        Serial.print("Motor ");
        Serial.print(motorNumber);
        Serial.print(" ajustado a PWM ");
        Serial.println(pwmValue);
      } else {
        // Si se ingresa un número de motor inválido
        Serial.println("Número de motor inválido. Ingrese motor1, motor2, motor3 o motor4.");
      }
    } else {
      // Si el comando no coincide con "motorX"
      Serial.println("Comando no reconocido. Ingrese motor1, motor2, motor3 o motor4 seguido del valor PWM.");
    }
  }
  
  delay(100);  // Pequeña pausa para evitar lecturas rápidas del puerto serie
}

void calibrateESC() {
  Serial.println("Iniciando calibración de ESC...");
  Serial.println("Asegúrate de que no haya hélices instaladas.");
  Serial.println("Presiona Enter para continuar...");
  
  while (!Serial.available());  // Esperar hasta que esté disponible alguna entrada en el puerto serie
  while (Serial.available()) Serial.read();  // Limpiar el buffer de entrada
  
  Serial.println("Conecta la batería para alimentar los ESC.");
  delay(3000);  // Espera 3 segundos para conectar la batería

  // Establecer el valor máximo del ESC
  setAllPWM(ESC_MAX);
  Serial.println("Valor máximo del ESC establecido. Mantén esta configuración por 5 segundos.");
  delay(5000);  // Mantener el valor MAX durante 5 segundos

  // Establecer el valor mínimo del ESC
  setAllPWM(ESC_MIN);
  Serial.println("Valor mínimo del ESC establecido. Mantén esta configuración por 8 segundos.");
  delay(8000);  // Mantener el valor MIN durante 8 segundos

  Serial.println("Calibración finalizada.");
  calibrationDone = true;
}

void setAllPWM(int pwmValue) {
  motor1.writeMicroseconds(pwmValue);
  motor2.writeMicroseconds(pwmValue);
  motor3.writeMicroseconds(pwmValue);
  motor4.writeMicroseconds(pwmValue);
}
