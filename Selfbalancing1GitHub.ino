/*
  SELF-BALANCING ROBOT — ESP32 + DRV8825 + MPU6050 + AccelStepper
   light (~1.3kg) bot, 22cm tall.

  WIRING:
   STEP1 -> GPIO18   DIR1 -> GPIO19
   STEP2 -> GPIO25   DIR2 -> GPIO26
   EN    -> GPIO27   
   MPU6050: SDA -> GPIO21, SCL -> GPIO22
   DRV8825 microstepping to 1/16 
*/

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <AccelStepper.h>

//  PIN DEFINITIONS 
#define STEP_PIN_1 18
#define DIR_PIN_1  19
#define STEP_PIN_2 25
#define DIR_PIN_2  26
#define EN_PIN     27   

AccelStepper stepper1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
AccelStepper stepper2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
Adafruit_MPU6050 mpu;


float Kp = 900.0;
float Ki = 0.6;
float Kd = 2.0;

float targetAngle = 0.0;      // will be set to the bot's true "balanced" angle after calibration
const float DEADBAND = 0.15;  // degrees; ignore tiny noise inside this band

// 1/16 microstepping -> 3200 steps/rev.
const float MAX_STEPS_PER_SEC = 6000.0;   // ~112 RPM cap
const float MAX_ACCEL_STEPS   = 12000.0;

const float FALL_ANGLE = 35.0;   // beyond this, cut motors — bot is down, not balancing

//  STATE 
float currentAngle = 0.0;
float gyroBiasY = 0.0;
float error = 0.0, lastError = 0.0, errorSum = 0.0;
float pidOutput = 0.0;
bool motorsEnabled = false;

unsigned long lastTime = 0;
unsigned long pidTimer = 0;
unsigned long debugTimer = 0;
const float PID_DT = 0.01; // 100Hz loop

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // keep motors OFF until calibration finishes

  stepper1.setMaxSpeed(MAX_STEPS_PER_SEC);
  stepper2.setMaxSpeed(MAX_STEPS_PER_SEC);
  stepper1.setAcceleration(MAX_ACCEL_STEPS);
  stepper2.setAcceleration(MAX_ACCEL_STEPS);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found. Check wiring.");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  calibrateMPU();

  Serial.println("Calibration done. Hold the bot upright to enable motors.");
  Serial.println("Serial tuning: type e.g.  kp70  ki0.8  kd3.2  t-1.5  (t = target angle offset)");

  lastTime = micros();
  pidTimer = millis();
  debugTimer = millis();
}

// Averages ~1000 samples to find: gyro Y bias, and the accel-only angle
// the bot sits at when we hold it balanced. That becomes targetAngle,
.
void calibrateMPU() {
  Serial.println("Calibrating... keep the bot still and BALANCED upright.");
  delay(1500);

  double gyroSum = 0, angleSum = 0;
  const int N = 1000;
  for (int i = 0; i < N; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroSum += g.gyro.y;
    angleSum += atan2(-a.acceleration.x,
                       sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z))
                * 180.0 / M_PI;
    delay(2);
  }
  gyroBiasY = gyroSum / N;
  targetAngle = angleSum / N;
  currentAngle = targetAngle;

  Serial.print("Gyro Y bias: "); Serial.println(gyroBiasY, 4);
  Serial.print("Target (balanced) angle: "); Serial.println(targetAngle, 2);
}

void loop() {
  readSerialTuning();

  if (millis() - pidTimer >= 10) {
    pidTimer = millis();

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    float accelAngle = atan2(-a.acceleration.x,
                              sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z))
                        * 180.0 / M_PI;
    float gyroRate = g.gyro.y - gyroBiasY;
    currentAngle = 0.98 * (currentAngle + gyroRate * 180.0 / M_PI * dt) + 0.02 * accelAngle;

    error = currentAngle - targetAngle;

    // Fall detection: cut motor power, don't just zero speed 
    if (abs(error) > FALL_ANGLE) {
      disableMotors();
      errorSum = 0;
      lastError = 0;
      return;
    } else if (!motorsEnabled) {
      enableMotors(); // re-enable once picked back up / balanced
    }

    // Deadband: ignore sensor noise near target 
    float effectiveError = (abs(error) < DEADBAND) ? 0.0 : error;

    errorSum += effectiveError * PID_DT;
    errorSum = constrain(errorSum, -50.0, 50.0); // anti-windup

    float dError = (effectiveError - lastError) / PID_DT;
    lastError = effectiveError;

    pidOutput = (Kp * effectiveError) + (Ki * errorSum) + (Kd * dError);
    pidOutput = constrain(pidOutput, -MAX_STEPS_PER_SEC, MAX_STEPS_PER_SEC);

    stepper1.setSpeed(pidOutput);
    stepper2.setSpeed(-pidOutput); // reversed mounting
  }

  stepper1.runSpeed();
  stepper2.runSpeed();

  if (millis() - debugTimer >= 200) {
    debugTimer = millis();
    Serial.print("Angle: "); Serial.print(currentAngle, 2);
    Serial.print(" | Err: "); Serial.print(error, 2);
    Serial.print(" | Out: "); Serial.print(pidOutput, 0);
    Serial.print(" | Kp="); Serial.print(Kp, 1);
    Serial.print(" Ki="); Serial.print(Ki, 2);
    Serial.print(" Kd="); Serial.println(Kd, 2);
  }
}

void enableMotors() {
  digitalWrite(EN_PIN, LOW); // DRV8825 EN is active LOW
  motorsEnabled = true;
}

void disableMotors() {
  digitalWrite(EN_PIN, HIGH);
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
  motorsEnabled = false;
}

// Change the values in serial Monitor: eg : kp70  ki0.8  kd3.2  t-1.5
void readSerialTuning() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() < 3) return;

  String prefix = cmd.substring(0, 2);
  float val = cmd.substring(2).toFloat();

  if (prefix == "kp") { Kp = val; Serial.print("Kp set to "); Serial.println(Kp); }
  else if (prefix == "ki") { Ki = val; Serial.print("Ki set to "); Serial.println(Ki); }
  else if (prefix == "kd") { Kd = val; Serial.print("Kd set to "); Serial.println(Kd); }
  else if (cmd.charAt(0) == 't') {
    targetAngle += cmd.substring(1).toFloat();
    Serial.print("Target angle set to "); Serial.println(targetAngle);
  }
}