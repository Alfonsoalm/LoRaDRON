#include <Wire.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads;  // Crear una instancia de objeto para ADS1115
const float voltaje_bat_100 = 12.6; // Voltaje de la batería completamente cargada
const float voltaje_bat_50 = 10.2;  // Voltaje de la batería al 50%
const float voltaje_bat_0 = 9.3;    // Voltaje de la batería completamente descargada

void setup(void) {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("Inicializando el ADS1115...");
  
  if (!ads.begin(0x48)) { // Inicializa el ADS1115 con la dirección I2C 0x48
    Serial.println("No se encontró el ADS1115, asegúrate de que esté conectado correctamente.");
    while (1);
  }

  Serial.println("ADS1115 encontrado y inicializado.");

  // Configuración del rango de voltaje de entrada
  ads.setGain(GAIN_TWOTHIRDS);  // ±6.144V range (default)
}

void loop(void) {
  int16_t adc0;
  float voltage, voltaje_bat, porcent_bat;

  adc0 = ads.readADC_SingleEnded(0);  // Leer el canal 0 en modo unipolar (single-ended)
  voltage = adc0 * 0.1875 / 1000.0;   // Convertir el valor ADC a voltaje (0.1875 mV por paso para ±6.144V range)
  voltaje_bat = voltage * (11.1 / 5.0); // Pasar a valor de antes del divisor de voltaje (Batería global de 11.1V)
  
  Serial.print("Voltaje leído por el ADS1115: ");
  Serial.print(voltage);
  Serial.println(" V");

  Serial.print("Voltaje en batería (antes del divisor): ");
  Serial.print(voltaje_bat);
  Serial.println(" V");

  // Calcular el porcentaje de batería usando interpolación
  if (voltaje_bat >= voltaje_bat_100) {
    porcent_bat = 100;
  } else if (voltaje_bat <= voltaje_bat_0) {
    porcent_bat = 0;
  } else if (voltaje_bat >= voltaje_bat_50) {
    porcent_bat = 50 + (voltaje_bat - voltaje_bat_50) * 50 / (voltaje_bat_100 - voltaje_bat_50);
  } else {
    porcent_bat = (voltaje_bat - voltaje_bat_0) * 50 / (voltaje_bat_50 - voltaje_bat_0);
  }
  
  Serial.print("% de batería: ");
  Serial.print(porcent_bat);   
  Serial.println(" %");

  delay(1000);  // Espera 1 segundo entre lecturas
}
