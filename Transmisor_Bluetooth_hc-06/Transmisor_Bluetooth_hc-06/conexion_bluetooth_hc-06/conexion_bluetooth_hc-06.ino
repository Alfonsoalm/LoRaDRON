#include <SoftwareSerial.h>

SoftwareSerial hc06(10, 11);

const int YAW_PIN = A0;       // VRX1
const int PITCH_PIN = A1;     // VRY1
const int ROLL_PIN = A2;      // VRX2
const int THROTTLE_PIN = A3;  // VRY2

void setup(){
  //Initialize Serial Monitor
  Serial.begin(9600);
  Serial.println("ENTER AT Commands:");
  //Initialize Bluetooth Serial Port
  hc06.begin(9600);
}

void loop(){
  //Write data from HC06 to Serial Monitor
  if (hc06.available()){
    Serial.write(hc06.read());
  }                         
  //Write from Serial Monitor to HC06
  if (Serial.available()){
    hc06.write(Serial.read());                                                     
  }           
  delay(10);

  int yawValue = analogRead(YAW_PIN);
  int pitchValue = analogRead(PITCH_PIN);
  int rollValue = analogRead(ROLL_PIN);
  int throttleValue = analogRead(THROTTLE_PIN);

  // Enviar los datos en un formato comprensible
  String dataString = "Yaw:" + String(yawValue) + ",Pitch:" + String(pitchValue) + ",Roll:" + String(rollValue) + ",Throttle:" + String(throttleValue);
  // Enviar los datos al módulo Bluetoo
  hc06.println(dataString); th

  // Escribir datos por puerto serie para ver en Serial Plotter
  Serial.print("Yaw:");
  Serial.print(yawValue);
  Serial.print(",Pitch:");
  Serial.print(pitchValue);
  Serial.print(",Roll:");
  Serial.print(rollValue);
  Serial.print(",Throttle:");
  Serial.println(throttleValue);

  

  delay(100);
}
