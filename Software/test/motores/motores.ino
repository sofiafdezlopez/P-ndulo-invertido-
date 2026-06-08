#include "Wire.h"
#include "I2Cdev.h"

// Pines de motores
const int pinPWMA = 12;
const int pinAIN2 = 25;
const int pinAIN1 = 33;
const int pinPWMB = 15;
const int pinBIN1 = 27;
const int pinBIN2 = 26;

void setup() {

  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
}

void loop() {
  // RUEDA DERECHA
  digitalWrite(pinAIN1, LOW);
  digitalWrite(pinAIN2, HIGH);
  analogWrite(pinPWMA, 100); // Velocidad del motor A (0-255)

  // RUEDA IZQUIERDA
  digitalWrite(pinBIN1, HIGH);
  digitalWrite(pinBIN2, LOW);
  analogWrite(pinPWMB, 100); // Velocidad del motor A (0-255)
}