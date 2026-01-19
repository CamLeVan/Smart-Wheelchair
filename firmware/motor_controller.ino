/*
 * Smart Wheelchair Firmware - Arduino Nano
 * Chức năng: Điều khiển động cơ PID, Đọc Encoder, Giao tiếp ROS qua Serial
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

// Biến toàn cục Encoder
volatile long left_ticks = 0;
volatile long right_ticks = 0;

// Tham số PID
double Kp = 1.5, Ki = 0.05, Kd = 0.01;
double input_left = 0, output_left = 0, setpoint_left = 0;
double input_right = 0, output_right = 0, setpoint_right = 0;

// Biến tính toán PID
unsigned long last_time;
double errSum_L, lastErr_L;
double errSum_R, lastErr_R;
String inputString = ""; // Buffer nhận lệnh serial

void setup() {
  Serial.begin(57600); // Baudrate giao tiếp với Raspberry Pi
  
  // Cấu hình chân
  pinMode(L_PWM, OUTPUT); pinMode(L_DIR, OUTPUT);
  pinMode(R_PWM, OUTPUT); pinMode(R_DIR, OUTPUT);
  pinMode(L_ENCODER_A, INPUT_PULLUP); pinMode(L_ENCODER_B, INPUT_PULLUP);
  pinMode(R_ENCODER_A, INPUT_PULLUP); pinMode(R_ENCODER_B, INPUT_PULLUP);
  
  // Ngắt Encoder
  attachInterrupt(digitalPinToInterrupt(L_ENCODER_A), updateEncoderL, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENCODER_A), updateEncoderR, RISING);
  
  last_time = millis();
}

void loop() {
  // 1. Đọc lệnh từ ROS (Non-blocking)
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      parseCommand(inputString);
      inputString = "";
    } else {
      inputString += c;
    }
  }
  
  // 2. Tính toán PID (Chu kỳ 50ms)
  unsigned long now = millis();
  if (now - last_time >= 50) {
    computePID_Left();
    computePID_Right();
    setMotor(L_PWM, L_DIR, output_left);
    setMotor(R_PWM, R_DIR, output_right);
    
    // 3. Gửi phản hồi về ROS (Odometry data)
    Serial.print("E,");
    Serial.print(left_ticks); Serial.print(",");
    Serial.print(right_ticks); Serial.println();
    
    last_time = now;
  }
}

void parseCommand(String cmd) {
  // Format: "V,100,100" -> Setpoint L=100, R=100
  if (cmd.startsWith("V")) {
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    
    setpoint_left = cmd.substring(firstComma + 1, secondComma).toDouble();
    setpoint_right = cmd.substring(secondComma + 1).toDouble();
  }
}

void computePID_Left() {
  // Đơn giản hóa: Input giả định là số tick đo được trong 50ms
  // Thực tế cần reset tick hoặc đo delta
  // Code thực tế cần phức tạp hơn để xử lý vận tốc m/s
  double error = setpoint_left - input_left; // input_left cần tính từ delta ticks
  errSum_L += (error * 50);
  double dErr = (error - lastErr_L) / 50;
  
  output_left = Kp * error + Ki * errSum_L + Kd * dErr;
  lastErr_L = error;
}

void computePID_Right() {
  // Tương tự Left
  double error = setpoint_right - input_right; 
  errSum_R += (error * 50);
  double dErr = (error - lastErr_R) / 50;
  
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
