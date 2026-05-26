/*
 * Smart Wheelchair Firmware - Arduino Nano
 * Chức năng: Điều khiển động cơ PID, Đọc Encoder, Giao tiếp ROS, Ultrasonic E-Stop, Heartbeat
 */

// Định nghĩa chân kết nối (Phù hợp với Driver BTS7960/L298N)
#define L_PWM 5
#define L_DIR 6
#define R_PWM 9
#define R_DIR 10

#define L_ENCODER_A 2 // Interrupt
#define L_ENCODER_B 4
#define R_ENCODER_A 3 // Interrupt
#define R_ENCODER_B 7

#define TRIG_PIN 11 // Cảm biến siêu âm
#define ECHO_PIN 12

// Biến toàn cục Encoder
volatile long left_ticks = 0;
volatile long right_ticks = 0;
long prev_left_ticks = 0;
long prev_right_ticks = 0;

// Tham số PID
double Kp = 1.5, Ki = 0.05, Kd = 0.01;
double input_left = 0, output_left = 0, setpoint_left = 0;
double input_right = 0, output_right = 0, setpoint_right = 0;

// Biến tính toán PID
unsigned long last_time;
double errSum_L = 0, lastErr_L = 0;
double errSum_R = 0, lastErr_R = 0;
String inputString = "";

// Biến an toàn
unsigned long last_cmd_time = 0;
bool e_stop_active = false;

void setup() {
  Serial.begin(57600);
  
  pinMode(L_PWM, OUTPUT); pinMode(L_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_DIR, OUTPUT);
  pinMode(L_ENCODER_A, INPUT_PULLUP); pinMode(L_ENCODER_B, INPUT_PULLUP);
  pinMode(R_ENCODER_A, INPUT_PULLUP); pinMode(R_ENCODER_B, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), updateEncoderR, RISING);
  
  last_time = millis();
  last_cmd_time = millis();
}

void loop() {
  unsigned long now = millis();

  // 1. Kiểm tra Heartbeat (Nếu mất tín hiệu từ Pi quá 1s -> Dừng)
  if (now - last_cmd_time > 1000) {
    setpoint_left = 0;
    setpoint_right = 0;
  }

  // 2. Đọc lệnh từ ROS
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      parseCommand(inputString);
      inputString = "";
      last_cmd_time = now; // Reset heartbeat
    } else {
      inputString += c;
    }
  }
  
  // 3. Đọc siêu âm (Hardware Interlock)
  long distance = readUltrasonic();
  if (distance > 0 && distance < 50) { // Vật cản < 50cm
    e_stop_active = true;
    setpoint_left = 0;
    setpoint_right = 0;
    output_left = 0;
    output_right = 0;
  } else {
    e_stop_active = false;
  }
  
  // 4. Tính toán PID (Chu kỳ 50ms)
  if (now - last_time >= 50) {
    // Tính vận tốc (delta ticks)
    input_left = left_ticks - prev_left_ticks;
    input_right = right_ticks - prev_right_ticks;
    
    prev_left_ticks = left_ticks;
    prev_right_ticks = right_ticks;

    if (!e_stop_active) {
      computePID_Left();
      computePID_Right();
    }
    
    setMotor(L_PWM, L_DIR, output_left);
    setMotor(R_PWM, R_DIR, output_right);
    
    // Gửi dữ liệu về ROS: Ticks trái, Ticks phải, Trạng thái E-Stop
    Serial.print("E,");
    Serial.print(left_ticks); Serial.print(",");
    Serial.print(right_ticks); Serial.print(",");
    Serial.print(e_stop_active ? "1" : "0");
    Serial.println();
    
    last_time = now;
  }
}

long readUltrasonic() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Giảm timeout xuống 6000 micro giây (~1 mét tối đa) để giảm thời gian chờ chặn xuống tối đa 6ms (thay vì 30ms),
  // tránh gây trễ chu kỳ điều khiển PID (50ms).
  long duration = pulseIn(ECHO_PIN, HIGH, 6000); // Timeout 6ms (6000us)
  return duration * 0.034 / 2;
}

void parseCommand(String cmd) {
  if (cmd.startsWith("V")) {
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    setpoint_left = cmd.substring(firstComma + 1, secondComma).toDouble();
    setpoint_right = cmd.substring(secondComma + 1).toDouble();
  }
}

void computePID_Left() {
  double error = setpoint_left - input_left;
  errSum_L += error;
  errSum_L = constrain(errSum_L, -1000, 1000); // Chống Windup
  double dErr = error - lastErr_L;
  
  // Áp dụng công thức PID với thành phần D giúp "triệt tiêu rung lắc"
  output_left = Kp * error + Ki * errSum_L + Kd * dErr;
  lastErr_L = error;
}

void computePID_Right() {
  double error = setpoint_right - input_right; 
  errSum_R += error;
  errSum_R = constrain(errSum_R, -1000, 1000); // Chống Windup
  double dErr = error - lastErr_R;
  
  output_right = Kp * error + Ki * errSum_R + Kd * dErr;
  lastErr_R = error;
}

void setMotor(int pwmPin, int dirPin, double spd) {
  if (spd > 255) spd = 255;
  if (spd < -255) spd = -255;
  
  if (spd >= 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, spd);
  } else {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, -spd);
  }
}

void updateEncoderL() {
  if (digitalRead(L_ENCODER_B) == LOW) left_ticks++;
  else left_ticks--;
}

void updateEncoderR() {
  if (digitalRead(R_ENCODER_B) == LOW) right_ticks++;
  else right_ticks--;
}
