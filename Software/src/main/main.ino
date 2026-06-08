#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 sensor;

const int pinPWMA = 12;
const int pinAIN2 = 33;
const int pinAIN1 = 25;
const int pinPWMB = 15;
const int pinBIN1 = 26;
const int pinBIN2 = 27;

// Batería 9 V, motores 6 V → ciclo de trabajo máx = 6/9 ≈ 0.667 → 255 * 0.667 ≈ 170
const int PWM_MAX = 170;   // Nunca enviar más que esto a los motores

float Kp = 35.0f;
float Ki = 0.1f;
float Kd = 5.0f;

float desired_angle = 0.0f;

float integral = 0.0f;
const float INTEGRAL_MAX = 300.0f;   // Anti-windup: limita la acumulación del integral

// Si el robot se inclina más que esto respecto al equilibrio, está caído:
const float ANGULO_CAIDA = 24.0f;

int16_t ax, ay, az;
int16_t gx, gy, gz;

const float ALPHA = 0.98f;
float ang_y = 0.0f;
float ang_y_prev = 0.0f;
float ang_x = 0.0f;  

unsigned long tiempo_prev = 0;
float dt = 0.0f;

unsigned long ultimo_print = 0;

void moveMotor(int pinPWM, int pinIN1, int pinIN2, float speed);
void pararMotores();


void setup() {
  Serial.begin(115200);
  Wire.begin(18, 19);
  Wire.setClock(400000);   
  sensor.initialize();

  // Offsets calibrados del MPU6050
  sensor.setXAccelOffset(-736);
  sensor.setYAccelOffset(1299);
  sensor.setZAccelOffset(1522);
  sensor.setXGyroOffset(166);
  sensor.setYGyroOffset(250);
  sensor.setZGyroOffset(56);

  // Pines de motores
  pinMode(pinAIN2, OUTPUT);
  pinMode(pinAIN1, OUTPUT);
  pinMode(pinPWMA, OUTPUT);
  pinMode(pinBIN1, OUTPUT);
  pinMode(pinBIN2, OUTPUT);
  pinMode(pinPWMB, OUTPUT);
  pararMotores();

  if (!sensor.testConnection()) {
    Serial.println("ERROR: MPU6050 no detectado. Revisa el cableado.");
    while (true) { delay(1000); }
  }

  // Inicializar el ángulo con la lectura del acelerómetro para arrancar limpio
  // (evita el pico inicial del filtro y de la derivada).
  sensor.getAcceleration(&ax, &ay, &az);
  float ang_y_init = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * (180.0f / PI);
  ang_y = ang_y_init;
  ang_y_prev = ang_y_init;

  tiempo_prev = millis();   

  Serial.println("Iniciando control PID del pendulo invertido.");
  Serial.print("Kp="); Serial.print(Kp);
  Serial.print("  Ki="); Serial.print(Ki);
  Serial.print("  Kd="); Serial.println(Kd);
}


// ═══════════════════════════════════════════════════════════════════════════════
void loop() {
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  unsigned long ahora = millis();
  dt = (ahora - tiempo_prev) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;
  tiempo_prev = ahora;

  float gy_dps = gy / 131.0f;

  // ── Ángulo del acelerómetro (atan2 maneja todos los cuadrantes) ──────────
  float accel_ang_y = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * (180.0f / PI);

  // ── Filtro complementario sobre el eje de balanceo (Y) ───────────────────
  ang_y = ALPHA * (ang_y_prev + gy_dps * dt) + (1.0f - ALPHA) * accel_ang_y;
  ang_y_prev = ang_y;

  ang_x = atan2(ay, sqrt((long)ax * ax + (long)az * az)) * (180.0f / PI);

  float error = desired_angle - ang_y;

  // Si está demasiado caído: parar y resetear el integral
  if (fabs(error) > ANGULO_CAIDA) {
    pararMotores();
    integral = 0.0f;
    return;
  }

  // Término integral con anti-windup
  integral += error * dt;
  if (integral >  INTEGRAL_MAX) integral =  INTEGRAL_MAX;
  if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

 
  float P = Kp * error;
  float I = Ki * integral;
  float D = Kd * (-gy_dps);

  float salida = P + I + D;

  // Saturar al PWM máximo permitido─
  if (salida >  PWM_MAX) salida =  PWM_MAX;
  if (salida < -PWM_MAX) salida = -PWM_MAX;

  moveMotor(pinPWMA, pinAIN1, pinAIN2,  salida);
  moveMotor(pinPWMB, pinBIN1, pinBIN2, -salida);

  if (millis() - ultimo_print > 50) {
    ultimo_print = millis();
    Serial.print("ang_Y: "); Serial.print(ang_y, 2);
    Serial.print("  err: "); Serial.print(error, 2);
    Serial.print("  P: ");   Serial.print(P, 1);
    Serial.print("  I: ");   Serial.print(I, 1);
    Serial.print("  D: ");   Serial.print(D, 1);
    Serial.print("  PWM: ");  Serial.println(salida, 0);
  }
}

void moveMotor(int pinPWM, int pinIN1, int pinIN2, float speed) {
  int pwm = (int)fabs(speed);

  // Seguridad: nunca sobrepasar el máximo (protege los motores de 6 V)
  if (pwm > PWM_MAX) pwm = PWM_MAX;

  if (pwm == 0) {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, LOW);
    analogWrite(pinPWM, 0);
    return;
  }


  if (speed > 0) {
    digitalWrite(pinIN1, HIGH);
    digitalWrite(pinIN2, LOW);
  } else {
    digitalWrite(pinIN1, LOW);
    digitalWrite(pinIN2, HIGH);
  }
  analogWrite(pinPWM, pwm);
}

void pararMotores() {
  digitalWrite(pinAIN1, LOW);
  digitalWrite(pinAIN2, LOW);
  digitalWrite(pinBIN1, LOW);
  digitalWrite(pinBIN2, LOW);
  analogWrite(pinPWMA, 0);
  analogWrite(pinPWMB, 0);
}
