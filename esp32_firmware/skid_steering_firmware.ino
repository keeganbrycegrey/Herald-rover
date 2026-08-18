#include <Arduino.h>

#define LEFT_RPWM   25   // forward PWM
#define LEFT_LPWM   26   // reverse PWM
#define LEFT_REN    27   // right enable
#define LEFT_LEN    4    // left enable

#define RIGHT_RPWM  32
#define RIGHT_LPWM  33
#define RIGHT_REN   14
#define RIGHT_LEN   13

#define PWM_FREQ_HZ     20000   // 20kHz, above audible range
#define PWM_RESOLUTION  8       // 0-255 duty

#define ENC_LEFT_A   34   // input-only pins, no internal pullup on 34-39
#define ENC_LEFT_B   35
#define ENC_RIGHT_A  36   // VP
#define ENC_RIGHT_B  39   // VN

volatile long encLeftTicks = 0;
volatile long encRightTicks = 0;
long lastReportedLeftTicks = 0;
long lastReportedRightTicks = 0;

const unsigned long ENCODER_REPORT_MS = 50;  // 20 Hz feedback
unsigned long lastReportTime = 0;

const unsigned long CMD_TIMEOUT_MS = 500;
unsigned long lastCmdTime = 0;

const size_t SERIAL_BUFFER_MAX_LEN = 32;

String serialBuffer = "";

void IRAM_ATTR onEncLeftA() {
  bool a = digitalRead(ENC_LEFT_A);
  bool b = digitalRead(ENC_LEFT_B);
  encLeftTicks += (a == b) ? 1 : -1;
}

void IRAM_ATTR onEncRightA() {
  bool a = digitalRead(ENC_RIGHT_A);
  bool b = digitalRead(ENC_RIGHT_B);
  encRightTicks += (a == b) ? 1 : -1;
}

void setMotorPWM(int leftPWM, int rightPWM) {
  leftPWM = constrain(leftPWM, -255, 255);
  rightPWM = constrain(rightPWM, -255, 255);

  if (leftPWM >= 0) {
    ledcWrite(LEFT_RPWM, leftPWM);
    ledcWrite(LEFT_LPWM, 0);
  } else {
    ledcWrite(LEFT_RPWM, 0);
    ledcWrite(LEFT_LPWM, -leftPWM);
  }

  if (rightPWM >= 0) {
    ledcWrite(RIGHT_RPWM, rightPWM);
    ledcWrite(RIGHT_LPWM, 0);
  } else {
    ledcWrite(RIGHT_RPWM, 0);
    ledcWrite(RIGHT_LPWM, -rightPWM);
  }
}

void stopMotors() {
  setMotorPWM(0, 0);
}

void handleSerialLine(String line) {
  line.trim();
  if (line.length() < 2 || line.charAt(0) != 'M') return;

  int commaIdx = line.indexOf(',');
  if (commaIdx == -1) return;

  int leftVal = line.substring(1, commaIdx).toInt();
  int rightVal = line.substring(commaIdx + 1).toInt();

  setMotorPWM(leftVal, rightVal);
  lastCmdTime = millis();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX2=16, TX2=17 -- explicit, don't rely on defaults

  pinMode(LEFT_REN, OUTPUT);
  pinMode(LEFT_LEN, OUTPUT);
  pinMode(RIGHT_REN, OUTPUT);
  pinMode(RIGHT_LEN, OUTPUT);
  digitalWrite(LEFT_REN, HIGH);
  digitalWrite(LEFT_LEN, HIGH);
  digitalWrite(RIGHT_REN, HIGH);
  digitalWrite(RIGHT_LEN, HIGH);

  ledcAttach(LEFT_RPWM, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(LEFT_LPWM, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(RIGHT_RPWM, PWM_FREQ_HZ, PWM_RESOLUTION);
  ledcAttach(RIGHT_LPWM, PWM_FREQ_HZ, PWM_RESOLUTION);

  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), onEncLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), onEncRightA, CHANGE);

  stopMotors();
  lastReportTime = millis();
  lastCmdTime = millis();
}

void loop() {
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      handleSerialLine(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
      if (serialBuffer.length() > SERIAL_BUFFER_MAX_LEN) {
        serialBuffer = "";  // drop garbled/incomplete command, wait for next newline
      }
    }
  }

  if (millis() - lastCmdTime > CMD_TIMEOUT_MS) {
    stopMotors();
  }

  unsigned long now = millis();
  if (now - lastReportTime >= ENCODER_REPORT_MS) {
    noInterrupts();
    long curLeft = encLeftTicks;
    long curRight = encRightTicks;
    interrupts();

    long dLeft = curLeft - lastReportedLeftTicks;
    long dRight = curRight - lastReportedRightTicks;
    unsigned long dt = now - lastReportTime;

    Serial2.print('E');
    Serial2.print(dLeft);
    Serial2.print(',');
    Serial2.print(dRight);
    Serial2.print(',');
    Serial2.println(dt);

    lastReportedLeftTicks = curLeft;
    lastReportedRightTicks = curRight;
    lastReportTime = now;
  }
}
