const int trigPin = 8;    // Pin digital para el Trig
const int echoPin = 9;   // Pin digital para el Echo

void setup() {
  Serial.begin(9600);     // Inicia la comunicación serie a 9600 bps
  pinMode(trigPin, OUTPUT);  // Configura el pin Trig como salida
  pinMode(echoPin, INPUT);   // Configura el pin Echo como entrada
}

void loop() {
  long duration;
  int distance;
  
  // Limpia el pin Trig
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Envía un pulso de 10 microsegundos desde el Trig
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Lee la duración del pulso desde el pin Echo
  duration = pulseIn(echoPin, HIGH);
  
  // Calcula la distancia en centímetros
  // Velocidad del sonido 343 m/s -> 34300 cm/s -> 0.03425 cm/us
  // V = D_ida+vuelta/T ->  2* D_ida = V * T -> D = T * V / 2
  distance = duration * 0.03425 / 2;

  // Muestra la distancia en el monitor serie
  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Pequeña pausa antes de la siguiente medición
  delay(100);
}
