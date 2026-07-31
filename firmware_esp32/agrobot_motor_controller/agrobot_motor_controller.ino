#define LEFT_RPWM 23
#define LEFT_LPWM 18

#define RIGHT_RPWM 25
#define RIGHT_LPWM 26

#define LEFT_ENC_A 21
#define LEFT_ENC_B 22

#define RIGHT_ENC_A 32
#define RIGHT_ENC_B 33

#define LEFT_INVERT false
#define RIGHT_INVERT true

volatile long left_count = 0;
volatile long right_count = 0;

int motor_speed = 130;

void IRAM_ATTR leftEncoderISR() {
  int b = digitalRead(LEFT_ENC_B);
  if (b == HIGH) left_count++;
  else left_count--;
}

void IRAM_ATTR rightEncoderISR() {
  int b = digitalRead(RIGHT_ENC_B);
  if (b == HIGH) right_count++;
  else right_count--;
}

void rawLeftMotor(int speed) {
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

void rawRightMotor(int speed) {
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

void setLeftMotor(int speed) {
  if (LEFT_INVERT) speed = -speed;
  rawLeftMotor(speed);
}

void setRightMotor(int speed) {
  if (RIGHT_INVERT) speed = -speed;
  rawRightMotor(speed);
}

void stopRobot() {
  setLeftMotor(0);
  setRightMotor(0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

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

  stopRobot();

  Serial.println("AGROBOT two-motor controller ready");
  Serial.println("Commands: f b l r s + - z");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'f') {
      setLeftMotor(motor_speed);
      setRightMotor(motor_speed);
      Serial.println("Forward");
    }

    else if (cmd == 'b') {
      setLeftMotor(-motor_speed);
      setRightMotor(-motor_speed);
      Serial.println("Backward");
    }

    else if (cmd == 'l') {
      setLeftMotor(-motor_speed);
      setRightMotor(motor_speed);
      Serial.println("Left turn");
    }

    else if (cmd == 'r') {
      setLeftMotor(motor_speed);
      setRightMotor(-motor_speed);
      Serial.println("Right turn");
    }

    else if (cmd == 's') {
      stopRobot();
      Serial.println("Stop");
    }

    else if (cmd == '+') {
      motor_speed += 20;
      if (motor_speed > 255) motor_speed = 255;
      Serial.print("Speed: ");
      Serial.println(motor_speed);
    }

    else if (cmd == '-') {
      motor_speed -= 20;
      if (motor_speed < 0) motor_speed = 0;
      Serial.print("Speed: ");
      Serial.println(motor_speed);
    }

    else if (cmd == 'z') {
      left_count = 0;
      right_count = 0;
      Serial.println("Encoder counts reset");
    }
  }

  static unsigned long last_print = 0;

  if (millis() - last_print >= 500) {
    Serial.print("L:");
    Serial.print(left_count);
    Serial.print(" R:");
    Serial.print(right_count);
    Serial.print(" Speed:");
    Serial.println(motor_speed);
    last_print = millis();
  }
}
