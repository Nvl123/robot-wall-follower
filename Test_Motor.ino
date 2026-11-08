/*
  =============================================================================
  TEST MOTOR & BELOK (Berdasarkan MotorTests.ino dari Repo GitHub)
  Disesuaikan dengan Driver L298N & Pin Robot Kamu
  =============================================================================
  Urutan Tes:
  1. Maju lurus 2.5 detik
  2. Berhenti 1 detik
  3. Belok Kanan 90° (pivot di tempat)
  4. Maju lurus 1 detik
  5. Belok Kiri 90° (pivot di tempat)
  6. Maju lurus 1 detik
  7. Putar balik 180°
  8. Berhenti 3 detik, lalu mengulang
  =============================================================================
*/

// ================== PIN MOTOR L298N ==================
const int ENA = 11;   // PWM Motor Kiri
const int IN1 = 8;    // Direction Motor Kiri
const int IN2 = 9;    // Direction Motor Kiri
const int ENB = 10;   // PWM Motor Kanan
const int IN3 = 12;   // Direction Motor Kanan
const int IN4 = 13;   // Direction Motor Kanan

// ================== KECEPATAN & DURASI KALIBRASI ==================
// Diturunkan 40% (menjadi 60% dari nilai awal)
const int SPEED_NORMAL = 80;   // Kecepatan maju lurus (sebelumnya 135)
const int SPEED_TURN   = 95;   // Kecepatan putar di tempat (sebelumnya 150)

// Durasi putaran disesuaikan karena kecepatan putar lebih rendah:
const unsigned long TIME_TURN_90  = 600;   // Durasi putar 90° (sebelumnya 380 ms)
const unsigned long TIME_TURN_180 = 1200;  // Durasi putar 180° (sebelumnya 760 ms)

const float TRIM_KIRI  = 1.00;
const float TRIM_KANAN = 1.00;

void setMotor(int kiri, int kanan) {
  kiri  = constrain((int)(kiri  * TRIM_KIRI),  -255, 255);
  kanan = constrain((int)(kanan * TRIM_KANAN), -255, 255);

  digitalWrite(IN1, (kiri >= 0) ? HIGH : LOW);
  digitalWrite(IN2, (kiri >= 0) ? LOW  : HIGH);
  analogWrite(ENA, abs(kiri));

  digitalWrite(IN3, (kanan >= 0) ? HIGH : LOW);
  digitalWrite(IN4, (kanan >= 0) ? LOW  : HIGH);
  analogWrite(ENB, abs(kanan));
}

void stopRobot(int d = 0) {
  setMotor(0, 0);
  if (d > 0) delay(d);
}

void rem() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);   analogWrite(ENB, 255);
  delay(35);
  stopRobot(0);
}

void forward(int speed, int ms) {
  setMotor(speed, speed);
  delay(ms);
}

void turnRight(int ms) {
  rem();
  delay(40);
  setMotor(SPEED_TURN, -SPEED_TURN);
  delay(ms);
  rem();
  delay(40);
}

void turnLeft(int ms) {
  rem();
  delay(40);
  setMotor(-SPEED_TURN, SPEED_TURN);
  delay(ms);
  rem();
  delay(40);
}

void turnAround180(int ms) {
  rem();
  delay(40);
  setMotor(SPEED_TURN, -SPEED_TURN);
  delay(ms);
  rem();
  delay(40);
}

void setup() {
  Serial.begin(115200);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  stopRobot(0);
  Serial.println(F("--- TES MOTOR & BELOK DIMULAI DALAM 3 DETIK ---"));
  delay(3000);
}

void loop() {
  // 1. Maju lurus 2.5 detik
  Serial.println(F("1. Maju lurus 2.5 detik..."));
  forward(SPEED_NORMAL, 2500);

  // 2. Berhenti 1 detik
  Serial.println(F("2. Berhenti 1 detik..."));
  rem();
  stopRobot(1000);

  // 3. Belok Kanan 90°
  Serial.println(F("3. Belok Kanan 90°..."));
  turnRight(TIME_TURN_90);
  stopRobot(800);

  // 4. Maju lurus 1 detik
  Serial.println(F("4. Maju lurus 1 detik..."));
  forward(SPEED_NORMAL, 1000);
  rem();
  stopRobot(800);

  // 5. Belok Kiri 90°
  Serial.println(F("5. Belok Kiri 90°..."));
  turnLeft(TIME_TURN_90);
  stopRobot(800);

  // 6. Maju lurus 1 detik
  Serial.println(F("6. Maju lurus 1 detik..."));
  forward(SPEED_NORMAL, 1000);
  rem();
  stopRobot(800);

  // 7. Putar balik 180°
  Serial.println(F("7. Putar balik 180°..."));
  turnAround180(TIME_TURN_180);
  stopRobot(1000);

  Serial.println(F("Tes selesai. Siklus akan diulang dalam 3 detik...\n"));
  delay(3000);
}
