#include <Arduino.h>

const int MOTOR_ENA = PA2;  
const int MOTOR_IN1 = PA3;
const int MOTOR_IN2 = PA4;

const int MOTOR_ENB = PA8;  
const int MOTOR_IN3 = PA5;
const int MOTOR_IN4 = PB0;

const int ENCODER_PIN = PA0;

volatile long pulseCount = 0;
float setpoint = 0; 
float inputRPM = 0;
float outputPWM = 0;

float Kp = 1.5; 
float Ki = 0.5; 
float Kd = 0.05; 

float error, lastError, cumError, rateError;

unsigned long lastTime = 0;
long lastCount = 0;
const int PPR = 20; 

void countPulse() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(MOTOR_ENA, OUTPUT);
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  
  pinMode(MOTOR_ENB, OUTPUT);
  pinMode(MOTOR_IN3, OUTPUT);
  pinMode(MOTOR_IN4, OUTPUT);
  
  pinMode(ENCODER_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), countPulse, RISING);

  digitalWrite(MOTOR_IN1, LOW);  digitalWrite(MOTOR_IN2, HIGH); 
  digitalWrite(MOTOR_IN3, HIGH); digitalWrite(MOTOR_IN4, LOW);  

  Serial.println("========================================");
  Serial.println("Sistem Konveyor Dual Motor Ready");
  Serial.println("Gunakan 'p[val]' untuk Manual (Open-Loop)");
  Serial.println("Gunakan 's[val]' untuk Target RPM (PID)");
  Serial.println("========================================");
}

void loop() {
  if (Serial.available() > 0) {
    char type = Serial.read();
    float val = Serial.parseFloat();
    
    if (type == 's') { 
      setpoint = val; 
      cumError = 0; 
      Serial.print("Target RPM diatur ke: "); Serial.println(val);
    } 
    else if (type == 'p') {
      setpoint = -1; 
      analogWrite(MOTOR_ENA, (int)val);
      analogWrite(MOTOR_ENB, (int)val);
      Serial.print("Manual PWM diatur ke: "); Serial.println(val);
    }
  }

  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;

  if (deltaTime >= 0.1) { 
    noInterrupts();
    long currentPulses = pulseCount;
    pulseCount = 0;
    interrupts();

    inputRPM = (currentPulses / (deltaTime / 60.0)) / PPR;

    if (setpoint >= 0) {
      error = setpoint - inputRPM;
      cumError += error * deltaTime;
      rateError = (error - lastError) / deltaTime;
      
      outputPWM = (Kp * error) + (Ki * cumError) + (Kd * rateError);
      
      outputPWM = constrain(outputPWM, 0, 255);

      analogWrite(MOTOR_ENA, (int)outputPWM);
      analogWrite(MOTOR_ENB, (int)outputPWM);
      
      lastError = error;
    }

    Serial.print("Setpoint:"); Serial.print(setpoint);
    Serial.print(",");
    Serial.print("RPM:"); Serial.println(inputRPM);

    lastTime = currentTime;
}
}