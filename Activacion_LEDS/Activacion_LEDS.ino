// Pines donde están conectados los LEDs
const int ledPins[] = {3, 4, 5, 6};

// Cantidad de LEDs
const int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);

// Retardo entre cambios de LEDs (en milisegundos)
const unsigned long delayTime = 300;
unsigned long previousMillis = 0;

// Variable para almacenar el estado actual
int currentLed = 0;

void setup() {
  // Configura todos los pines de LEDs como salida
  for (int i = 0; i < numLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {
  // Verifica el tiempo transcurrido
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= delayTime) {
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
