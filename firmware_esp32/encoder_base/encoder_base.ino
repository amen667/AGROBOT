// AGROBOT Encoder Base Test
// ESP32 + 2 BTS7960 + 2 JGA25-371 encoder motors

// -------- Motor driver pins --------
// Left BTS7960
#define LEFT_RPWM   25
#define LEFT_LPWM   26

// Right BTS7960
#define RIGHT_RPWM  27
#define RIGHT_LPWM  14

// -------- Encoder pins --------
// Left encoder
#define LEFT_ENC_A  32
#define LEFT_ENC_B  33

// Right encoder
#define RIGHT_ENC_A 18
#define RIGHT_ENC_B 19

volatile long left_count = 0;
volatile long right_count = 0;

int motor_speed = 90; // 0 to 255. Start low because 620 RPM is fast.

void IRAM_ATTR leftEncoderISR() {
  int b = digitalRead(LEFT_ENC_B);
  if (b == HIGH) {
    left_count++;
  } else {
    left_count--;
  }
}

void IRAM_ATTR rightEncoderISR() {
  int b = digitalRead(RIGHT_ENC_B);
  if (b == HIGH) {
    right_count++;
  } else {
    right_count--;
  }
}

void setLeftMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(LEFT_RPWM, speed);
    analogWrite(LEFT_LPWM, 0);
  } else if (speed < 0) {
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, -speed);
  } else {
    analogWrite(LEFT_RPWM, 0);
    analogWrite(LEFT_LPWM, 0);
  }
}

void setRightMotor(int speed) {
  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    analogWrite(RIGHT_RPWM, speed);
    analogWrite(RIGHT_LPWM, 0);
  } else if (speed < 0) {
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, -speed);
  } else {
    analogWrite(RIGHT_RPWM, 0);
    analogWrite(RIGHT_LPWM, 0);
  }
}

void stopMotors() {
  setLeftMotor(0);
  setRightMotor(0);
}

void printStatus() {
  Serial.print("Left encoder: ");
  Serial.print(left_count);
  Serial.print(" | Right encoder: ");
  Serial.println(right_count);
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_RPWM, OUTPUT);
  pinMode(LEFT_LPWM, OUTPUT);
  pinMode(RIGHT_RPWM, OUTPUT);
  pinMode(RIGHT_LPWM, OUTPUT);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  stopMotors();

  Serial.println("AGROBOT encoder base test ready.");
  Serial.println("Commands:");
  Serial.println("f = forward");
  Serial.println("b = backward");
  Serial.println("l = turn left");
  Serial.println("r = turn right");
  Serial.println("s = stop");
  Serial.println("z = reset encoder counts");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'f') {
      Serial.println("Forward");
      setLeftMotor(motor_speed);
      setRightMotor(motor_speed);
    }

    else if (cmd == 'b') {
      Serial.println("Backward");
      setLeftMotor(-motor_speed);
      setRightMotor(-motor_speed);
    }

    else if (cmd == 'l') {
      Serial.println("Turn left");
      setLeftMotor(-motor_speed);
      setRightMotor(motor_speed);
    }

    else if (cmd == 'r') {
      Serial.println("Turn right");
      setLeftMotor(motor_speed);
      setRightMotor(-motor_speed);
    }

    else if (cmd == 's') {
      Serial.println("Stop");
      stopMotors();
    }

    else if (cmd == 'z') {
      left_count = 0;
      right_count = 0;
      Serial.println("Encoder counts reset");
    }
  }

  static unsigned long last_print = 0;
  if (millis() - last_print > 500) {
    printStatus();
    last_print = millis();
  }
}