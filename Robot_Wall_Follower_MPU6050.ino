/*
  =============================================================================
  WALL MAZE SOLVER ROBOT - MPU-6050 (GY-521) GYROSCOPE EDITION
  Arduino UNO + L298N + 3x HC-SR04 + MPU-6050 (I2C)
  =============================================================================
  Pembaruan Utama dengan Sensor Gyroscope GY-521 (MPU-6050):
  1. BELOK 90° & 180° CLOSED-LOOP MUTLAK:
     - Tidak lagi mengandalkan tebak-tebakan milidetik delay(ms).
     - Robot berputar membaca sudut rotasi derajat nyata dari sensor Gyro (Sumbu Z/Yaw).
     - Dilengkapi deselerasi cerdas: saat sudut mendekati 75°, kecepatan putar
       melambat perlahan sehingga pengereman di 90.0° pas tanpa overshoot/kebablasan.
  2. HEADING LOCK (STABILISASI MAJU LURUS):
     - Masalah roda cenderung belok ke kiri teratasi otomatis.
     - Gyro mendeteksi deviasi sudut saat maju lurus dan langsung melakukan
       mikro-koreksi agar robot meluncur lurus sempurna seperti di atas rel.
  3. KOMUNIKASI I2C MURNI (Wire.h):
     - Tidak membutuhkan instalasi library eksternal tambahan.
     - Cukup hubungkan 4 kabel: VCC->5V, GND->GND, SDA->Pin A4, SCL->Pin A5.
  4. KALIBRASI OTOMATIS SAAT STARTUP:
     - Saat robot pertama kali dinyalakan (2 detik awal), gyro mengukur offset
       bias nol secara otomatis saat robot diam.
  =============================================================================
*/

#include <Wire.h>

// ======================== PILIH MODE ========================
// 0: Normal Maze Solver (Gyro Aktif)
// 1: Tes Sensor Ultrasonik (Serial Monitor @115200, motor mati)
// 2: Tes Belok Gyro 90° & 180° (Uji presisi sudut di Serial Monitor)
// 3: Tes Maju Lurus Gyro (Uji Heading Lock)
const int MODE_OPERASI = 0;

// ======================== KONFIGURASI PIN ========================
// Motor Driver L298N:
const int ENA = 11;   // PWM Motor Kiri
const int IN1 = 8;    // Direction Motor Kiri
const int IN2 = 9;    // Direction Motor Kiri
const int ENB = 10;   // PWM Motor Kanan
const int IN3 = 12;   // Direction Motor Kanan
const int IN4 = 13;   // Direction Motor Kanan

// Sensor Ultrasonik HC-SR04:
const int TRIG_KANAN = 3;
const int ECHO_KANAN = 2;

const int TRIG_DEPAN = 5;
const int ECHO_DEPAN = 4;

const int TRIG_KIRI  = 7;
const int ECHO_KIRI  = 6;

// I2C Pin MPU-6050 (Hardware Arduino Uno):
// SDA = Pin A4
// SCL = Pin A5
const int MPU_ADDR = 0x68;  // Alamat I2C default GY-521

// Pin Opsional (INPUT_PULLUP):
const int PIN_JUMPER_DINDING = A0;  // LOW = Dinding KIRI, HIGH = KANAN
const int PIN_TOMBOL_START   = A1;  // Tekan untuk mulai

// ======================== PARAMETER ARENA & AMBANG ========================
const float LEBAR_LORONG   = 40.0;   // Lebar lintasan (cm)
const float LEBAR_ROBOT    = 16.0;   // Lebar robot (cm)
const float TARGET_TENGAH  = (LEBAR_LORONG - LEBAR_ROBOT) / 2.0;  // 12.0 cm

// Zona toleransi tengah (cm)
const float AMBANG_DEKAT   = 11.0;   // < 11 cm: terlalu dekat dinding
const float AMBANG_JAUH    = 13.0;   // > 13 cm: mulai menjauhi dinding
const float AMBANG_TERBUKA = 22.0;   // > 22 cm: dinding hilang / simpangan
const float DEPAN_STOP     = 15.0;   // Berhenti saat jarak depan <= 15 cm

// ======================== KECEPATAN MOTOR ========================
const int SPEED_NORMAL     = 80;     // Kecepatan maju lurus
const int SPEED_SLOW       = 70;     // Koreksi belok halus lambat
const int SPEED_FAST       = 85;     // Koreksi belok halus cepat
const int SPEED_TURN_FAST  = 95;     // Kecepatan putar awal
const int SPEED_TURN_SLOW  = 75;     // Kecepatan putar mendekati target sudut (anti-overshoot)
const int SPEED_BACK       = 70;     // Kecepatan mundur sedikit sebelum belok

// Trim Motor fisik
const float TRIM_KIRI      = 0.89;   // Disesuaikan dengan fisik motor Anda
const float TRIM_KANAN     = 1.00;

// Opsi Aturan Dinding Utama:
const bool IKUTI_KANAN_DEFAULT  = true;
const bool PAKAI_JUMPER_DINDING = false;
const bool PAKAI_TOMBOL_START   = false;

bool ikutiKanan = IKUTI_KANAN_DEFAULT;

// ======================== VARIABEL GYROSCOPE (MPU-6050) ========================
float gyroZ_offset = 0.0;     // Nilai kalibrasi diam
float yawAngle = 0.0;         // Sudut hadap akumulatif (derajat)
unsigned long prevGyroTime = 0;

// ======================== FUNGSI MPU-6050 ========================
void initMPU6050() {
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // Register PWR_MGMT_1
  Wire.write(0x00);  // Bangunkan MPU-6050 dari mode sleep
  Wire.endTransmission(true);

  // Set sensitivitas Gyroscope ke +/- 250 deg/s (FS_SEL = 0)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);  // Register GYRO_CONFIG
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);
}

// Kalibrasi otomatis gyro saat robot diam
void kalibrasiGyro() {
  Serial.println(F("\n[GYRO] Memulai kalibrasi offset... JANGAN GERAKKAN ROBOT!"));
  long sumZ = 0;
  const int NUM_SAMPLES = 400;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);  // Register GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);

    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    sumZ += rawZ;
    delay(4);
  }

  // Sensitivitas +/- 250 deg/s adalah 131.0 LSB/(deg/s)
  gyroZ_offset = (float)sumZ / (float)NUM_SAMPLES / 131.0;
  yawAngle = 0.0;
  prevGyroTime = micros();

  Serial.print(F("[GYRO] Kalibrasi Selesai! Offset Z: "));
  Serial.println(gyroZ_offset, 4);
}

// Membaca laju putar sumbu Z (deg/s)
float bacaGyroRateZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 2, true);

  if (Wire.available() >= 2) {
    int16_t rawZ = (Wire.read() << 8) | Wire.read();
    float rateZ = ((float)rawZ / 131.0) - gyroZ_offset;
    // Deadzone filter untuk noise getaran halus (< 0.15 deg/s diabaikan)
    if (abs(rateZ) < 0.15) rateZ = 0.0;
    return rateZ;
  }
  return 0.0;
}

// Memperbarui akumulasi sudut yaw
void updateYaw() {
  unsigned long now = micros();
  float dt = (now - prevGyroTime) / 1000000.0;
  prevGyroTime = now;

  float rateZ = bacaGyroRateZ();
  yawAngle += rateZ * dt;
}

void resetYaw() {
  yawAngle = 0.0;
  prevGyroTime = micros();
}

// ======================== FUNGSI SENSOR ULTRASONIK ========================
float pingCm(int trigPin, int echoPin, unsigned long timeoutUs = 8000) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long durasi = pulseIn(echoPin, HIGH, timeoutUs);
  if (durasi == 0) return 999.0;
  return durasi / 58.2;
}

float bacaDepan() {
  float d = pingCm(TRIG_DEPAN, ECHO_DEPAN);
  return (d >= 999.0) ? 200.0 : d;
}

float bacaKanan() {
  float d = pingCm(TRIG_KANAN, ECHO_KANAN);
  return (d >= 999.0) ? 200.0 : d;
}

float bacaKiri() {
  float d = pingCm(TRIG_KIRI, ECHO_KIRI);
  return (d >= 999.0) ? 200.0 : d;
}

// ======================== KONTROL MOTOR ========================
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

void forward(int l, int r) {
  setMotor(l, r);
}

void back(int l, int r) {
  setMotor(-l, -r);
}

void stopRobot(int delayMs = 0) {
  setMotor(0, 0);
  if (delayMs > 0) delay(delayMs);
}

void rem() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);   analogWrite(ENB, 255);
  delay(35);
  stopRobot(0);
}

// ======================== MANUVER BELOK TERTUTUP (CLOSED-LOOP GYRO) ========================
// Belok Kanan tepat targetDeg derajat menggunakan umpan balik Gyroscope Z
void turnRightGyro(float targetDeg = 90.0) {
  Serial.print(F("[GYRO] Belok Kanan Target: ")); Serial.print(targetDeg); Serial.println(F("°"));
  rem();
  delay(40);
  resetYaw();

  unsigned long startTime = millis();
  while (true) {
    updateYaw();
    // Putaran ke kanan menghasilkan nilai yaw negatif (atau positif tergantung orientasi)
    float sudutTercapai = abs(yawAngle);

    // Jika mendekati 15 derajat terakhir, perlambat putaran agar rem tepat sasaran
    int currentSpeed = (sudutTercapai >= (targetDeg - 15.0)) ? SPEED_TURN_SLOW : SPEED_TURN_FAST;
    setMotor(currentSpeed, -currentSpeed);

    if (sudutTercapai >= targetDeg) {
      break;
    }

    // Safety timeout: jika roda tersangkut > 2.5 detik, hentikan paksa
    if (millis() - startTime > 2500) {
      Serial.println(F("[GYRO] Timeout belok kanan!"));
      break;
    }
    delay(2);
  }

  rem();
  delay(50);
  Serial.print(F("[GYRO] Sudut Akhir Kanan: ")); Serial.print(abs(yawAngle), 1); Serial.println(F("°"));
}

// Belok Kiri tepat targetDeg derajat menggunakan umpan balik Gyroscope Z
void turnLeftGyro(float targetDeg = 90.0) {
  Serial.print(F("[GYRO] Belok Kiri Target: ")); Serial.print(targetDeg); Serial.println(F("°"));
  rem();
  delay(40);
  resetYaw();

  unsigned long startTime = millis();
  while (true) {
    updateYaw();
    float sudutTercapai = abs(yawAngle);

    int currentSpeed = (sudutTercapai >= (targetDeg - 15.0)) ? SPEED_TURN_SLOW : SPEED_TURN_FAST;
    setMotor(-currentSpeed, currentSpeed);

    if (sudutTercapai >= targetDeg) {
      break;
    }

    if (millis() - startTime > 2500) {
      Serial.println(F("[GYRO] Timeout belok kiri!"));
      break;
    }
    delay(2);
  }

  rem();
  delay(50);
  Serial.print(F("[GYRO] Sudut Akhir Kiri: ")); Serial.print(abs(yawAngle), 1); Serial.println(F("°"));
}

// Putar Balik 180° menggunakan umpan balik Gyroscope Z
void turn180Gyro() {
  Serial.println(F("[GYRO] Putar Balik 180°"));
  if (ikutiKanan) {
    turnRightGyro(180.0);
  } else {
    turnLeftGyro(180.0);
  }
}

// ======================== MAJU LURUS TERKUNCI (HEADING LOCK) ========================
// Menjaga robot meluncur benar-benar lurus dengan kompensasi sudut Gyro
void forwardWithHeadingLock(int baseSpeed, float targetHeading = 0.0) {
  updateYaw();
  float errorAngle = yawAngle - targetHeading; // positif: serong kanan, negatif: serong kiri

  // Gain koreksi heading (proporsional)
  float kP_gyro = 2.0;
  int koreksi = (int)(errorAngle * kP_gyro);

  int pwmKiri  = baseSpeed - koreksi;
  int pwmKanan = baseSpeed + koreksi;

  setMotor(constrain(pwmKiri, 65, 110), constrain(pwmKanan, 65, 110));
}

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);

  // Pin Motor
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Pin Ultrasonik
  pinMode(TRIG_KANAN, OUTPUT); pinMode(ECHO_KANAN, INPUT);
  pinMode(TRIG_DEPAN, OUTPUT); pinMode(ECHO_DEPAN, INPUT);
  pinMode(TRIG_KIRI,  OUTPUT); pinMode(ECHO_KIRI,  INPUT);

  // Inisialisasi & Kalibrasi Gyroscope MPU-6050
  initMPU6050();
  kalibrasiGyro();

  if (PAKAI_JUMPER_DINDING) {
    pinMode(PIN_JUMPER_DINDING, INPUT_PULLUP);
    ikutiKanan = (digitalRead(PIN_JUMPER_DINDING) == HIGH);
  } else {
    ikutiKanan = IKUTI_KANAN_DEFAULT;
  }

  stopRobot(0);

  Serial.println(F("========================================"));
  Serial.println(F("ROBOT WALL MAZE SOLVER (GYRO MPU-6050)"));
  Serial.print(F("Mode Operasi: ")); Serial.println(MODE_OPERASI);
  Serial.print(F("Aturan Dinding: ")); Serial.println(ikutiKanan ? F("KANAN") : F("KIRI"));
  Serial.println(F("========================================"));

  // Mode Pengujian
  if (MODE_OPERASI == 1) {
    jalankanTesSensor();
  } else if (MODE_OPERASI == 2) {
    jalankanTesBelokGyro();
  } else if (MODE_OPERASI == 3) {
    jalankanTesHeadingLock();
  }

  if (PAKAI_TOMBOL_START) {
    pinMode(PIN_TOMBOL_START, INPUT_PULLUP);
    Serial.println(F("Menunggu tombol START..."));
    while (digitalRead(PIN_TOMBOL_START) == HIGH) delay(50);
    delay(1000);
  } else {
    delay(1500);
  }

  resetYaw();
  Serial.println(F("Robot Mulai Berjalan!"));
}

// ======================== LOOP UTAMA ========================
void loop() {
  // Update akumulasi yaw dari gyro
  updateYaw();

  // 1. Baca ketiga sensor ultrasonik secara berurutan
  float cmF  = bacaDepan();
  delay(5);
  float cmRF = bacaKanan();
  delay(5);
  float cmLF = bacaKiri();

  // Tampilkan data ke Serial Monitor
  Serial.print(F("LF: ")); Serial.print(cmLF, 1);
  Serial.print(F(" | F: ")); Serial.print(cmF, 1);
  Serial.print(F(" | RF: ")); Serial.print(cmRF, 1);
  Serial.print(F(" | Yaw: ")); Serial.print(yawAngle, 1);

  // -------------------------------------------------------------
  // KONDISI A: DEPAN BEBAS (cmF > DEPAN_STOP) -> WALL FOLLOWING
  // -------------------------------------------------------------
  if (cmF > DEPAN_STOP) {
    if (ikutiKanan) {
      // ===== LOGIKA PENELUSURAN DINDING KANAN =====
      if (cmRF >= AMBANG_DEKAT && cmRF <= AMBANG_JAUH) {
        // Posisi ideal di tengah lorong: maju lurus terkunci heading
        forward(SPEED_NORMAL, SPEED_NORMAL);
        Serial.println(F(" -> Maju Lurus (Ideal ~12cm)"));
      }
      else if (cmRF > AMBANG_JAUH) {
        if (cmRF >= AMBANG_TERBUKA) {
          // Dinding kanan hilang (celah/simpang kanan): ikuti dinding kiri
          if (cmLF >= AMBANG_DEKAT && cmLF <= AMBANG_JAUH) {
            forward(SPEED_NORMAL, SPEED_NORMAL);
            Serial.println(F(" -> Maju Lurus (Pemandu Kiri)"));
          }
          else if (cmLF > AMBANG_JAUH && cmLF < AMBANG_TERBUKA) {
            forward(SPEED_SLOW, SPEED_FAST);  // Serong ke kiri
            Serial.println(F(" -> Koreksi Kiri Halus"));
          }
          else if (cmLF < AMBANG_DEKAT) {
            forward(SPEED_FAST, SPEED_SLOW);  // Serong ke kanan
            Serial.println(F(" -> Koreksi Kanan Halus"));
          }
          else {
            forward(SPEED_NORMAL, SPEED_NORMAL);
            Serial.println(F(" -> Maju Lurus (Simpang Terbuka)"));
          }
        } else {
          // Masih ada dinding kanan tapi agak menjauh -> belok kanan halus
          forward(SPEED_FAST, SPEED_SLOW);
          Serial.println(F(" -> Koreksi Kanan Halus"));
        }
      }
      else if (cmRF < AMBANG_DEKAT) {
        // Terlalu dekat ke dinding kanan -> belok kiri halus menjauh
        forward(SPEED_SLOW, SPEED_FAST);
        Serial.println(F(" -> Koreksi Kiri Halus (Terlalu Dekat Kanan)"));
      }

    } else {
      // ===== LOGIKA PENELUSURAN DINDING KIRI =====
      if (cmLF >= AMBANG_DEKAT && cmLF <= AMBANG_JAUH) {
        forward(SPEED_NORMAL, SPEED_NORMAL);
        Serial.println(F(" -> Maju Lurus (Kiri Ideal)"));
      }
      else if (cmLF > AMBANG_JAUH) {
        if (cmLF >= AMBANG_TERBUKA) {
          if (cmRF >= AMBANG_DEKAT && cmRF <= AMBANG_JAUH) {
            forward(SPEED_NORMAL, SPEED_NORMAL);
            Serial.println(F(" -> Maju Lurus (Pemandu Kanan)"));
          }
          else if (cmRF > AMBANG_JAUH && cmRF < AMBANG_TERBUKA) {
            forward(SPEED_FAST, SPEED_SLOW);
            Serial.println(F(" -> Koreksi Kanan Halus"));
          }
          else if (cmRF < AMBANG_DEKAT) {
            forward(SPEED_SLOW, SPEED_FAST);
            Serial.println(F(" -> Koreksi Kiri Halus"));
          }
          else {
            forward(SPEED_NORMAL, SPEED_NORMAL);
            Serial.println(F(" -> Maju Lurus (Simpang Terbuka)"));
          }
        } else {
          forward(SPEED_SLOW, SPEED_FAST);
          Serial.println(F(" -> Koreksi Kiri Halus"));
        }
      }
      else if (cmLF < AMBANG_DEKAT) {
        forward(SPEED_FAST, SPEED_SLOW);
        Serial.println(F(" -> Koreksi Kanan Halus (Terlalu Dekat Kiri)"));
      }
    }
  }

  // -------------------------------------------------------------
  // KONDISI B: DEPAN TERHALANG (cmF <= DEPAN_STOP) -> MANUVER GYRO
  // -------------------------------------------------------------
  else {
    Serial.println(F("\n[HALANGAN] Depan Terhalang! Melakukan manuver belok Gyro..."));

    // Mundur sejenak memberi ruang putar
    back(SPEED_BACK, SPEED_BACK);
    delay(120);
    rem();
    stopRobot(200);

    // Baca ulang sisi samping
    delay(10);
    cmRF = bacaKanan();
    delay(10);
    cmLF = bacaKiri();

    bool kananBebas = (cmRF > AMBANG_TERBUKA);
    bool kiriBebas  = (cmLF > AMBANG_TERBUKA);

    Serial.print(F("Status Sisi -> Kiri Bebas: ")); Serial.print(kiriBebas ? F("YA") : F("TIDAK"));
    Serial.print(F(" | Kanan Bebas: ")); Serial.println(kananBebas ? F("YA") : F("TIDAK"));

    // 1. Kiri dinding, Kanan terbuka -> Belok Kanan tepat 90° via Gyro
    if (!kiriBebas && kananBebas) {
      turnRightGyro(90.0);
    }
    // 2. Kiri terbuka, Kanan dinding -> Belok Kiri tepat 90° via Gyro
    else if (kiriBebas && !kananBebas) {
      turnLeftGyro(90.0);
    }
    // 3. Simpang T (Kedua sisi terbuka) -> Belok 90° sesuai preferensi
    else if (kiriBebas && kananBebas) {
      if (ikutiKanan) {
        turnRightGyro(90.0);
      } else {
        turnLeftGyro(90.0);
      }
    }
    // 4. Jalan Buntu -> Putar Balik tepat 180° via Gyro
    else {
      turn180Gyro();
    }

    // Maju sejenak menjauhi sudut setelah selesai berputar
    resetYaw();
    forward(SPEED_NORMAL, SPEED_NORMAL);
    delay(200);
  }

  delay(15);
}

// ======================== MODUL TES GYRO & SENSOR ========================

// MODE 1: Tes Sensor Ultrasonik + Gyro
void jalankanTesSensor() {
  Serial.println(F("\n--- [MODE 1] TES SENSOR ULTRASONIK & GYRO (Motor MATI) ---"));
  while (true) {
    updateYaw();
    float f  = bacaDepan(); delay(10);
    float ka = bacaKanan(); delay(10);
    float ki = bacaKiri();  delay(10);

    Serial.print(F("KIRI: ")); Serial.print(ki, 1);
    Serial.print(F(" cm | DEPAN: ")); Serial.print(f, 1);
    Serial.print(F(" cm | KANAN: ")); Serial.print(ka, 1);
    Serial.print(F(" cm | YAW GYRO: ")); Serial.print(yawAngle, 1);
    Serial.println(F("°"));
    delay(200);
  }
}

// MODE 2: Tes Belok Presisi Gyro 90° & 180°
void jalankanTesBelokGyro() {
  Serial.println(F("\n--- [MODE 2] TES BELOK GYROSCOPE 90° & 180° ---"));
  Serial.println(F("Robot akan: Belok Kanan 90° -> Stop -> Belok Kiri 90° -> Stop -> Putar 180°"));
  delay(3000);

  while (true) {
    Serial.println(F("\n>> Mulai Belok Kanan 90° via Gyro..."));
    turnRightGyro(90.0);
    stopRobot(1500);

    Serial.println(F("\n>> Mulai Belok Kiri 90° via Gyro..."));
    turnLeftGyro(90.0);
    stopRobot(1500);

    Serial.println(F("\n>> Mulai Putar Balik 180° via Gyro..."));
    turn180Gyro();
    stopRobot(2500);

    Serial.println(F("\nSatu siklus selesai. Mengulang dalam 3 detik..."));
    delay(3000);
  }
}

// MODE 3: Tes Maju Lurus Terkunci Heading Lock
void jalankanTesHeadingLock() {
  Serial.println(F("\n--- [MODE 3] TES HEADING LOCK MAJU LURUS ---"));
  delay(3000);

  while (true) {
    resetYaw();
    Serial.println(F("Maju lurus terkunci Gyro selama 3 detik..."));
    unsigned long start = millis();
    while (millis() - start < 3000) {
      forwardWithHeadingLock(SPEED_NORMAL, 0.0);
      delay(10);
    }
    rem();
    stopRobot(2000);
  }
}
