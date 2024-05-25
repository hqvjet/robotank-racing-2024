#include <SoftwareSerial.h>

struct L298N {
  int IN1 = 2;
  int ENA = 3;  // PWM
  int IN2 = 4;
  int IN3 = 5;  // PWM
  int ENB = 6;  // PWM
  int IN4 = 7;
};

struct HW871 {
  // Analog
  int OUT1 = 1;
  int OUT2 = 2;
  int OUT3 = 3;
  int OUT4 = 4;
  int OUT5 = 5;
  int OUT6 = 8;
  int OUT7 = 9;
  int THRESHOLD = 900;
};

L298N l298n;
HW871 hw871;
SoftwareSerial btSerial(10, 11);  // TX, RX

bool stringComplete = false;
String inputString = "";
bool permission_to_run = true;
bool finish = false;

// PID
float kp = 15, ki = 0.00001, kd = 40;
float p_val, i_val, d_val;
int p, i, d, pre_error, pid, error;
int BASE_SPEED = 150;
int MIN_SPEED = 75;
bool first_run = true;
bool left_wrapping = false;
bool right_wrapping = false;

void setup() {
  pinMode(13, OUTPUT);

  pinMode(l298n.IN1, OUTPUT);
  pinMode(l298n.IN2, OUTPUT);
  pinMode(l298n.ENA, OUTPUT);
  pinMode(l298n.IN3, OUTPUT);
  pinMode(l298n.IN4, OUTPUT);
  pinMode(l298n.ENB, OUTPUT);

  pinMode(hw871.OUT1, INPUT);
  pinMode(hw871.OUT2, INPUT);
  pinMode(hw871.OUT3, INPUT);
  pinMode(hw871.OUT4, INPUT);
  pinMode(hw871.OUT5, INPUT);
  pinMode(hw871.OUT6, INPUT);
  pinMode(hw871.OUT7, INPUT);

  analogWrite(l298n.ENA, BASE_SPEED);  // Speed control
  analogWrite(l298n.ENB, BASE_SPEED);  // Speed control

  Serial.begin(9600);
  btSerial.begin(9600);
  inputString.reserve(200);

  move();
}

void move() {
  digitalWrite(l298n.IN1, 1);
  digitalWrite(l298n.IN2, 0);
  digitalWrite(l298n.IN3, 1);
  digitalWrite(l298n.IN4, 0);
}

void stop() {
  digitalWrite(l298n.IN1, 0);
  digitalWrite(l298n.IN2, 0);
  digitalWrite(l298n.IN3, 0);
  digitalWrite(l298n.IN4, 0);
}

void turn_back() {
  delay(600);
  digitalWrite(l298n.IN1, 1);
  digitalWrite(l298n.IN2, 0);
  digitalWrite(l298n.IN3, 0);
  digitalWrite(l298n.IN4, 1);

  analogWrite(l298n.ENA, 255);
  analogWrite(l298n.ENB, 255);
  delay(400);
  while (!check()) {
    continue;
  }
  permission_to_run = false;
  stop();
}

void sharpRight() {
  digitalWrite(l298n.IN1, 1);
  digitalWrite(l298n.IN2, 0);
  digitalWrite(l298n.IN3, 0);
  digitalWrite(l298n.IN4, 1);
}

void sharpLeft() {
  digitalWrite(l298n.IN1, 0);
  digitalWrite(l298n.IN2, 1);
  digitalWrite(l298n.IN3, 1);
  digitalWrite(l298n.IN4, 0);
}

bool checkNumber(String string) {
  for (int i = 0; i < string.length(); ++i) {
    if ((string[i] < '0' || string[i] > '9') && string[i] != '-' && string[i] != '.')
      return false;
  }

  return true;
}

bool check() {
  int sensor_val[5] = { 0, 0, 0, 0, 0 };

  for (int i(1); i <= 5; ++i)
    sensor_val[i - 1] = analogRead(i) < hw871.THRESHOLD ? 0 : 1;

  int count = 0;
  for (int i(0); i < 5; ++i)
    if (sensor_val[i] == 1) {
      ++count;
    }

  if (count == 0)
    return false;
  return true;
}

String lower(String string) {
  for (int i = 0; i < string.length(); ++i) {
    if (string[i] >= 'A' && string[i] <= 'Z')
      string[i] = (char)((int)string[i] + 32);
  }

  return string;
}


// Drive motor in range of PID sensors
void motor_drive(int lsp, int rsp) {
  move();
  analogWrite(l298n.ENA, abs(lsp));  // Speed control
  analogWrite(l298n.ENB, abs(rsp));
}

// Drive motor at alert case, sharp wrapping
void wrap(int speed, bool left_motor) {
  if (!left_motor) {
    sharpRight();
    analogWrite(l298n.ENA, abs(speed));
    analogWrite(l298n.ENB, abs(speed));
  } else {
    sharpLeft();
    analogWrite(l298n.ENB, abs(speed));
    analogWrite(l298n.ENA, abs(speed));
  }
}

// Calculate PID value for driving two motors
void cal_PID() {
  if (left_wrapping) {
    p = 0, i = 0, d = 0, pre_error = 0;
    if (error > 6)
      wrap(80, true);
    else if (error < -6)
      wrap(80, false);
  }

  else {
    p = error;
    i += error;
    d = error - pre_error;

    float pid_val = kp * p + ki * i + kd * d;
    pre_error = error;

    int lsp = BASE_SPEED - pid_val;
    int rsp = BASE_SPEED + pid_val;

    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < -255) {
      lsp = -255;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < -255) {
      rsp = -255;
    }

    motor_drive(lsp, rsp);
  }
}

// Getting error based on five sensors
int getError() {
  int sensor_val[5] = { 0, 0, 0, 0, 0 };
  int sharp_sensor_val[2] = { 0, 0 };

  for (int i(1); i <= 5; ++i)
    sensor_val[i - 1] = analogRead(i) < hw871.THRESHOLD ? 0 : 1;

  for (int i(0); i < 2; ++i)
    sharp_sensor_val[i] = digitalRead(i + 8);

  if (sharp_sensor_val[0] == 1)
    return 20;
  if (sharp_sensor_val[1] == 1)
    return -20;

  float avg = 0;
  int count = 0;
  for (int i(0); i < 5; ++i)
    if (sensor_val[i] == 1) {
      ++count;
      avg += i;
    }

  if (count == 0) {
    if (!finish) {
      analogWrite(l298n.ENA, 0);
      analogWrite(l298n.ENB, 0);
    }
    return pre_error;
  }
  if (count == 5) {
    finish = true;
    return;
  }

  avg = avg / (1 * count) * 2 - 4;
  return avg;
}


// Main loop
void loop() {
  if (permission_to_run) {
    error = getError();
    if (!finish)
      cal_PID();
    else {
      digitalWrite(13, 1);
      turn_back();
    }
  } else {
    stop();
  }
}