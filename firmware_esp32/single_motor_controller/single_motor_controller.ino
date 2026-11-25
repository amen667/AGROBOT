#include <Arduino.h>

// Motor driver pins
#define RPWM 23
#define LPWM 18

// Encoder pins
#define ENC_A 21
#define ENC_B 22

// Motor settings
const int MOTOR_SPEED = 130;
const unsigned long WATCHDOG_MS = 300;

volatile long encoder_count = 0;

unsigned long last_command = 0;
unsigned long last_report = 0;

bool motor_active = false;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_B) == HIGH) {
    encoder_count++;
  } else {
    encoder_count--;
  }
}

void stopMotor() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  motor_active = false;
}

void setMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(LPWM, 0);
    analogWrite(RPWM, speed);
  } else if (speed < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -speed);
  } else {
    stopMotor();
    return;
  }

  motor_active = true;
}

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);

  stopMotor();

  Serial2.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  attachInterrupt(
    digitalPinToInterrupt(ENC_A),
    encoderISR,
    RISING
  );

  Serial2.println("AGROBOT Single Motor Controller Ready");
  Serial2.println("Commands: f b s");
}

void loop() {
  if (Serial2.available()) {
    char cmd = Serial2.read();

    if (cmd == 'f') {
      last_command = millis();
      setMotor(MOTOR_SPEED);
    } else if (cmd == 'b') {
      last_command = millis();
      setMotor(-MOTOR_SPEED);
    } else if (cmd == 's') {
      stopMotor();
    }
  }

  if (motor_active &&
      millis() - last_command >= WATCHDOG_MS) {
    stopMotor();
    Serial2.println("WATCHDOG STOP");
  }

  if (millis() - last_report >= 100) {
    last_report = millis();

    Serial2.print("ENC:");
    Serial2.print(encoder_count);
    Serial2.print(" ACTIVE:");
    Serial2.println(motor_active ? 1 : 0);
  }
}
