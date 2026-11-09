/*
  =============================================================================
  WALL MAZE SOLVER ROBOT - MPU-6050 (GY-521) GYROSCOPE EDITION (OPTIMIZED)
  Arduino UNO + L298N + 3x HC-SR04 + MPU-6050 (I2C)
  =============================================================================
  Fitur & Optimasi Utama:
  1. BELOK 90° & 180° CLOSED-LOOP ABSOLUT (ANTI MUTER-MUTER):
     - Menggunakan perubahan sudut absolut |yawAngle - startYaw| sehingga kebal
       terhadap inversi tanda (+/-) maupun orientasi pemasangan sensor.
     - Dilengkapi deselerasi bertahap (melambat 20° sebelum target) dan kompensasi
       inersia pengereman (ANTISIPASI_REM_DERAJAT) agar berhenti tepat di 90.0°.
     - Proteksi Ganda (Fail-Safe): Jika sensor Gyro lepas atau roda tertahan,
       otomatis berhenti sesuai durasi waktu terkalibrasi (250 ms) sehingga
       robot TIDAK AKAN PERNAH berputar-putar tanpa henti.
  2. FILTER HARDWARE DLPF 44Hz & RENTANG +/- 500 DEG/S:
     - Mengaktifkan Digital Low-Pass Filter internal MPU-6050 (Register 0x1A)
       untuk meredam getaran mekanis motor DC dan sasis.
     - Rentang pengukuran dinaikkan ke +/- 500 deg/s agar sensor tidak mengalami
       saturasi (clipping) saat robot berputar cepat.
  3. KALIBRASI CERDAS DETEKSI DIAM (AUTO-STATIONARY CALIBRATION):
     - Saat dinyalakan, robot menunggu hingga benar-benar diletakkan diam di
       lantai sebelum mengunci titik nol Gyro (mencegah error akibat goyangan
       tangan saat menekan tombol saklar ON).
  4. PROTEKSI ANTI-LONJAKAN INTEGRASI WAKTU (DT CLAMPING):
     - Mencegah lonjakan sudut palsu setelah jeda delay() panjang.
  =============================================================================
*/

#include <Wire.h>

// ======================== PILIH MODE ========================
// 0: Normal Maze Solver (Gyro Aktif)
// 1: Tes Sensor Ultrasonik & Gyro (Serial Monitor @115200, motor mati)
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

// ======================== KECEPATAN & KALIBRASI MOTOR ========================
const int SPEED_NORMAL     = 80;     // Kecepatan maju lurus
const int SPEED_SLOW       = 70;     // Koreksi belok halus lambat
const int SPEED_FAST       = 85;     // Koreksi belok halus cepat
const int SPEED_TURN_FAST  = 90;     // Kecepatan putar awal
const int SPEED_TURN_SLOW  = 72;     // Kecepatan putar saat mendekati target 90°
const int SPEED_BACK       = 70;     // Kecepatan mundur memberi ruang putar

// Trim Motor fisik (disesuaikan agar kedua roda seimbang)
const float TRIM_KIRI      = 0.89;
const float TRIM_KANAN     = 1.00;

// Parameter Presisi Belok Gyro & Fail-Safe Waktu
const float ANTISIPASI_REM_DERAJAT = 2.5;  // Rem aktif 2.5° sebelum 90° untuk kompensasi inersia
const unsigned long TURN_90_MS     = 250;  // Durasi cadangan 90° (ms) jika Gyro terlepas
const unsigned long TURN_180_MS    = 500;  // Durasi cadangan 180° (ms) jika Gyro terlepas

// Opsi Aturan Dinding Utama:
const bool IKUTI_KANAN_DEFAULT  = true;
const bool PAKAI_JUMPER_DINDING = false;
const bool PAKAI_TOMBOL_START   = false;

bool ikutiKanan = IKUTI_KANAN_DEFAULT;

// ======================== VARIABEL GYROSCOPE (MPU-6050) ========================
// I2C Pin MPU-6050 pada Arduino Uno: SDA = Pin A4, SCL = Pin A5
int   mpuAddr        = 0x68;   // Alamat I2C (otomatis cek 0x68 dan 0x69)
bool  mpuDitemukan   = false;  // Status deteksi sensor
float gyroZ_offset   = 0.0;    // Nilai kalibrasi diam (deg/s)
float yawAngle       = 0.0;    // Sudut hadap kumulatif (derajat)
unsigned long prevGyroTime = 0;

// Skala sensitivitas untuk FS_SEL = 1 (+/- 500 deg/s) adalah 65.5 LSB/(deg/s)
const float GYRO_SCALE = 65.5;

// ======================== FUNGSI MPU-6050 ========================
bool cekAlamatI2C(int addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void tulisRegisterMPU(byte reg, byte val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

void initMPU6050() {
  Wire.begin();
  #if defined(WIRE_HAS_TIMEOUT)
    // Mencegah jalur I2C nge-freeze jika ada interferensi elektrik motor DC
    Wire.setWireTimeout(3000, true);
  #endif

  // Deteksi otomatis alamat I2C MPU-6050 (0x68 jika AD0=LOW, 0x69 jika AD0=HIGH)
  if (cekAlamatI2C(0x68)) {
    mpuAddr = 0x68;
    mpuDitemukan = true;
  } else if (cekAlamatI2C(0x69)) {
    mpuAddr = 0x69;
    mpuDitemukan = true;
  } else {
    mpuDitemukan = false;
    Serial.println(F("[GYRO ERROR] MPU-6050 TIDAK TERDETEKSI di 0x68 maupun 0x69!"));
    Serial.println(F("-> Periksa kabel VCC (5V), GND, SDA (A4), SCL (A5)."));
    Serial.println(F("-> Robot otomatis menggunakan mode belok waktu cadangan (250ms)."));
    return;
  }

  Serial.print(F("[GYRO] MPU-6050 Terdeteksi Normal di alamat 0x"));
  Serial.println(mpuAddr, HEX);

  // 1. Bangunkan MPU-6050 dari mode sleep (Register 0x6B PWR_MGMT_1 = 0x01 PLL X-axis gyro ref)
  tulisRegisterMPU(0x6B, 0x01);
  delay(50);

  // 2. Aktifkan Digital Low-Pass Filter (DLPF) 44Hz (Register 0x1A CONFIG = 0x03)
  //    Meredam getaran mekanis dari gearbox motor dan roda
  tulisRegisterMPU(0x1A, 0x03);

  // 3. Set rentang Gyroscope ke +/- 500 deg/s (Register 0x1B GYRO_CONFIG = 0x08)
  //    Mencegah saturasi (clipping) saat robot berputar cepat (> 250 deg/s)
  tulisRegisterMPU(0x1B, 0x08);

  delay(50);
}

bool bacaRawGyroZ(int16_t &rawZ) {
  if (!mpuDitemukan) return false;

  Wire.beginTransmission(mpuAddr);
  Wire.write(0x47);  // Register GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  byte count = Wire.requestFrom(mpuAddr, 2, true);
  if (count >= 2 && Wire.available() >= 2) {
    rawZ = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
  }
  return false;
}

// Kalibrasi otomatis dengan deteksi diam (menunggu tangan lepas dari saklar)
void kalibrasiGyro() {
  if (!mpuDitemukan) return;

  Serial.println(F("\n[GYRO] Persiapan kalibrasi... Letakkan robot dalam keadaan DIAM."));
  delay(800);  // Beri waktu 0.8 detik setelah saklar ON ditekan

  // Tunggu sampai robot benar-benar diam (variasi pembacaan rendah)
  for (int percobaan = 0; percobaan < 15; percobaan++) {
    int16_t minVal = 32767;
    int16_t maxVal = -32768;
    bool valid = true;

    for (int i = 0; i < 40; i++) {
      int16_t rZ = 0;
      if (bacaRawGyroZ(rZ)) {
        if (rZ < minVal) minVal = rZ;
        if (rZ > maxVal) maxVal = rZ;
      } else {
        valid = false;
      }
      delay(4);
    }

    // Jika selisih max-min < 350 LSB (~5.3 deg/s), berarti robot sudah diam di lantai
    if (valid && (maxVal - minVal < 350)) {
      break;
    }
    Serial.println(F("[GYRO] Robot terdeteksi masih bergerak, menunggu diam..."));
  }

  Serial.println(F("[GYRO] Mengukur offset titik nol... JANGAN GERAKKAN ROBOT!"));
  long sumZ = 0;
  int sampelBerhasil = 0;
  const int TARGET_SAMPEL = 300;

  for (int i = 0; i < TARGET_SAMPEL; i++) {
    int16_t rawZ = 0;
    if (bacaRawGyroZ(rawZ)) {
      sumZ += rawZ;
      sampelBerhasil++;
    }
    delay(3);
  }

  if (sampelBerhasil > 50) {
    gyroZ_offset = ((float)sumZ / (float)sampelBerhasil) / GYRO_SCALE;
    Serial.print(F("[GYRO] Kalibrasi Selesai! Offset Z: "));
    Serial.print(gyroZ_offset, 3);
    Serial.println(F(" deg/s"));
  } else {
    mpuDitemukan = false;
    Serial.println(F("[GYRO ERROR] Pembacaan data gagal saat kalibrasi! Beralih ke mode waktu."));
  }

  yawAngle = 0.0;
  prevGyroTime = micros();
}

// Membaca laju putar sumbu Z (deg/s)
float bacaGyroRateZ() {
  int16_t rawZ = 0;
  if (bacaRawGyroZ(rawZ)) {
    float rateZ = ((float)rawZ / GYRO_SCALE) - gyroZ_offset;
    // Deadzone filter untuk mengabaikan derau statis (< 0.25 deg/s)
    if (abs(rateZ) < 0.25) rateZ = 0.0;
    return rateZ;
  }
  return 0.0;
}

// Memperbarui akumulasi sudut yaw dengan pengaman dt
void updateYaw() {
  if (!mpuDitemukan) return;

  unsigned long now = micros();
  float dt = (now - prevGyroTime) / 1000000.0;
  prevGyroTime = now;

  // Pengaman: Jika jeda antar pemanggilan > 50ms (misal sehabis fungsi baca sensor/delay),
  // jangan kalikan dengan dt besar agar tidak menimbulkan lonjakan sudut palsu
  if (dt > 0.05 || dt <= 0.0) {
    return;
  }

  float rateZ = bacaGyroRateZ();
  yawAngle += rateZ * dt;
}

void resetYaw() {
  yawAngle = 0.0;
  prevGyroTime = micros();
}

// Fungsi pengganti delay() yang tetap memantau Gyro secara kontinu setiap 2ms
void delayWithGyro(unsigned long durasiMs) {
  unsigned long start = millis();
  while (millis() - start < durasiMs) {
    updateYaw();
    delay(2);
  }
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
  if (delayMs > 0) delayWithGyro(delayMs);
}

void rem() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);   analogWrite(ENB, 255);
  delayWithGyro(35);
  stopRobot(0);
}

// ======================== MANUVER BELOK CLOSED-LOOP GYRO ========================
// Fungsi internal eksekusi putaran presisi dengan proteksi ganda
void eksekusiPutarGyro(int arahPutar, float targetDeg) {
  // arahPutar: +1 = Belok Kanan (kiri maju, kanan mundur)
  //            -1 = Belok Kiri  (kiri mundur, kanan maju)
  unsigned long fallbackMs = (unsigned long)(TURN_90_MS * (targetDeg / 90.0));

  if (!mpuDitemukan) {
    Serial.println(F("[FALLBACK] Belok menggunakan waktu terkalibrasi..."));
    setMotor(arahPutar * SPEED_TURN_FAST, -arahPutar * SPEED_TURN_FAST);
    delay(fallbackMs);
    rem();
    delay(50);
    return;
  }

  rem();
  delayWithGyro(40);

  // Reset timer integrasi tepat sebelum motor berputar
  prevGyroTime = micros();
  float startYaw = yawAngle;
  unsigned long startTime = millis();

  float batasBerhenti = max(5.0f, targetDeg - ANTISIPASI_REM_DERAJAT);
  float batasLambat   = max(5.0f, targetDeg - 20.0f);
  unsigned long timeoutMax = (targetDeg > 100.0) ? 1300 : 750;

  while (true) {
    updateYaw();
    float rotated = abs(yawAngle - startYaw);
    unsigned long elapsed = millis() - startTime;

    // 1. Target sudut tercapai -> langsung rem!
    if (rotated >= batasBerhenti) {
      break;
    }

    // 2. Proteksi Fail-Safe: Jika sudah berjalan sepanjang waktu belok normal (250ms)
    //    tapi bacaan Gyro tidak bergerak (< 3 derajat, misal kabel SDA/SCL kendor),
    //    langsung hentikan tepat waktu agar robot tidak pernah muter-muter!
    if (elapsed >= fallbackMs && rotated < 3.0) {
      Serial.println(F("[GYRO FAIL-SAFE] Sinyal sudut tidak berubah, berhenti tepat di waktu cadangan!"));
      break;
    }

    // 3. Batas waktu maksimum keamanan
    if (elapsed >= timeoutMax) {
      Serial.println(F("[GYRO] Batas waktu putar tercapai!"));
      break;
    }

    // Deselerasi bertahap saat mendekati target 20° terakhir
    int spd = (rotated >= batasLambat) ? SPEED_TURN_SLOW : SPEED_TURN_FAST;
    setMotor(arahPutar * spd, -arahPutar * spd);

    delay(2);
  }

  rem();
  delayWithGyro(50);  // Tetap pantau sudut selama inersia berhenti

  Serial.print(F("[GYRO] Selesai Belok. Total Rotasi Terukur: "));
  Serial.print(abs(yawAngle - startYaw), 1);
  Serial.println(F("°"));
}

void turnRightGyro(float deltaDeg = 90.0) {
  Serial.print(F("[GYRO] Belok Kanan ")); Serial.print(deltaDeg, 0); Serial.println(F("°"));
  eksekusiPutarGyro(1, deltaDeg);
}

void turnLeftGyro(float deltaDeg = 90.0) {
  Serial.print(F("[GYRO] Belok Kiri ")); Serial.print(deltaDeg, 0); Serial.println(F("°"));
  eksekusiPutarGyro(-1, deltaDeg);
}

void turn180Gyro() {
  Serial.println(F("[GYRO] Putar Balik 180°"));
  if (ikutiKanan) {
    eksekusiPutarGyro(1, 180.0);
  } else {
    eksekusiPutarGyro(-1, 180.0);
  }
}

// ======================== MAJU LURUS TERKUNCI (HEADING LOCK) ========================
// Digunakan untuk Mode 3 (Uji jalan lurus terbuka dengan kompensasi Gyro)
// Catatan: Pada MPU-6050, serong kanan membuat yawAngle negatif, serong kiri membuat positif.
void forwardWithHeadingLock(int baseSpeed, float targetHeading = 0.0) {
  if (!mpuDitemukan) {
    forward(baseSpeed, baseSpeed);
    return;
  }
  updateYaw();
  float errorAngle = yawAngle - targetHeading;

  // Jika errorAngle positif (serong ke kiri) -> tambah roda kiri, kurangi roda kanan
  // Jika errorAngle negatif (serong ke kanan) -> kurangi roda kiri, tambah roda kanan
  float kP_gyro = 1.8;
  int koreksi = constrain((int)(errorAngle * kP_gyro), -20, 20);

  int pwmKiri  = baseSpeed + koreksi;
  int pwmKanan = baseSpeed - koreksi;

  setMotor(constrain(pwmKiri, 65, 105), constrain(pwmKanan, 65, 105));
}

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);

  // Pin Motor
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopRobot(0);

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

  Serial.println(F("========================================"));
  Serial.println(F("ROBOT WALL MAZE SOLVER (GYRO MPU-6050)"));
  Serial.print(F("Mode Operasi  : ")); Serial.println(MODE_OPERASI);
  Serial.print(F("Status Gyro   : ")); Serial.println(mpuDitemukan ? F("AKTIF") : F("FALLBACK WAKTU"));
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
    delay(800);
  } else {
    delay(1000);
  }

  resetYaw();
  Serial.println(F("Robot Mulai Berjalan!"));
}

// ======================== LOOP UTAMA ========================
void loop() {
  updateYaw();

  // 1. Baca ketiga sensor ultrasonik secara berurutan
  float cmF  = bacaDepan();
  delayWithGyro(5);
  float cmRF = bacaKanan();
  delayWithGyro(5);
  float cmLF = bacaKiri();

  // Filter anti-glitch: Jika depan terdeteksi dekat (<= DEPAN_STOP),
  // lakukan 1x baca ulang cepat untuk memastikan bukan pantulan suara palsu
  if (cmF <= DEPAN_STOP) {
    delayWithGyro(5);
    float cekUlangF = bacaDepan();
    if (cekUlangF > DEPAN_STOP) {
      cmF = cekUlangF;
    }
  }

  // Tampilkan telemetri ke Serial Monitor
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
        forward(SPEED_NORMAL, SPEED_NORMAL);
        Serial.println(F(" -> Maju Lurus (Ideal)"));
      }
      else if (cmRF > AMBANG_JAUH) {
        if (cmRF >= AMBANG_TERBUKA) {
          // Dinding kanan terbuka (simpang): gunakan pemandu dinding kiri
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
          // Dinding kanan masih ada tapi agak menjauh (13 - 22 cm) -> serong kanan halus
          forward(SPEED_FAST, SPEED_SLOW);
          Serial.println(F(" -> Koreksi Kanan Halus"));
        }
      }
      else {
        // cmRF < AMBANG_DEKAT: terlalu dekat ke dinding kanan -> serong kiri halus
        forward(SPEED_SLOW, SPEED_FAST);
        Serial.println(F(" -> Koreksi Kiri Halus (Menjauhi Kanan)"));
      }

    } else {
      // ===== LOGIKA PENELUSURAN DINDING KIRI =====
      if (cmLF >= AMBANG_DEKAT && cmLF <= AMBANG_JAUH) {
        forward(SPEED_NORMAL, SPEED_NORMAL);
        Serial.println(F(" -> Maju Lurus (Kiri Ideal)"));
      }
      else if (cmLF > AMBANG_JAUH) {
        if (cmLF >= AMBANG_TERBUKA) {
          // Dinding kiri terbuka: gunakan pemandu dinding kanan
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
      else {
        // cmLF < AMBANG_DEKAT: terlalu dekat ke dinding kiri -> serong kanan halus
        forward(SPEED_FAST, SPEED_SLOW);
        Serial.println(F(" -> Koreksi Kanan Halus (Menjauhi Kiri)"));
      }
    }
  }

  // -------------------------------------------------------------
  // KONDISI B: DEPAN TERHALANG (cmF <= DEPAN_STOP) -> MANUVER GYRO
  // -------------------------------------------------------------
  else {
    Serial.println(F("\n[HALANGAN] Depan Terhalang! Berhenti & mengecek simpangan..."));

    // 1. Berhenti di titik persimpangan terlebih dahulu
    rem();
    stopRobot(120);

    // 2. Baca jarak samping saat posisi di persimpangan (rata-rata 2x agar akurat)
    float ka1 = bacaKanan(); delayWithGyro(8);
    float ki1 = bacaKiri();  delayWithGyro(8);
    float ka2 = bacaKanan(); delayWithGyro(8);
    float ki2 = bacaKiri();  delayWithGyro(8);
    float fSekarang = bacaDepan();

    // Gunakan rata-rata 2x pembacaan saat berhenti di persimpangan (sebelum mundur)
    // agar pembacaan stabil dan celah belokan tidak terlewat
    float jarakKanan = (ka1 + ka2) * 0.5f;
    float jarakKiri  = (ki1 + ki2) * 0.5f;

    bool kananBebas = (jarakKanan > AMBANG_TERBUKA);
    bool kiriBebas  = (jarakKiri  > AMBANG_TERBUKA);

    // 3. Mundur sedikit untuk ruang putar bodi depan
    if (fSekarang < 11.0 || (!kananBebas && !kiriBebas)) {
      back(SPEED_BACK, SPEED_BACK);
      delayWithGyro(110);
      rem();
      stopRobot(120);
    }

    Serial.print(F("Status Sisi -> Kiri: ")); Serial.print(jarakKiri, 1);
    Serial.print(F("cm (")); Serial.print(kiriBebas ? F("BEBAS") : F("DINDING"));
    Serial.print(F(") | Kanan: ")); Serial.print(jarakKanan, 1);
    Serial.print(F("cm (")); Serial.print(kananBebas ? F("BEBAS") : F("DINDING"));
    Serial.println(F(")"));

    // 4. Eksekusi keputusan belok dengan Gyroscope
    // Kasus 1: Kiri tertutup, Kanan terbuka -> Belok Kanan 90°
    if (!kiriBebas && kananBebas) {
      turnRightGyro(90.0);
    }
    // Kasus 2: Kiri terbuka, Kanan tertutup -> Belok Kiri 90°
    else if (kiriBebas && !kananBebas) {
      turnLeftGyro(90.0);
    }
    // Kasus 3: Simpang T (Kedua sisi terbuka) -> Sesuai aturan dinding
    else if (kiriBebas && kananBebas) {
      if (ikutiKanan) {
        turnRightGyro(90.0);
      } else {
        turnLeftGyro(90.0);
      }
    }
    // Kasus 4: Jalan Buntu (Kedua sisi tertutup) -> Putar Balik 180°
    else {
      turn180Gyro();
    }

    // 5. Maju sejenak memasuki lorong baru setelah selesai berputar
    forward(SPEED_NORMAL, SPEED_NORMAL);
    delayWithGyro(150);
  }

  delayWithGyro(12);
}

// ======================== MODUL TES GYRO & SENSOR ========================

// MODE 1: Tes Sensor Ultrasonik + Gyro
void jalankanTesSensor() {
  Serial.println(F("\n--- [MODE 1] TES SENSOR ULTRASONIK & GYRO (Motor MATI) ---"));
  Serial.println(F("Putar robot dengan tangan untuk melihat perubahan sudut YAW GYRO."));
  while (true) {
    float f  = bacaDepan(); delayWithGyro(10);
    float ka = bacaKanan(); delayWithGyro(10);
    float ki = bacaKiri();  delayWithGyro(10);

    Serial.print(F("KIRI: ")); Serial.print(ki, 1);
    Serial.print(F(" cm | DEPAN: ")); Serial.print(f, 1);
    Serial.print(F(" cm | KANAN: ")); Serial.print(ka, 1);
    Serial.print(F(" cm | YAW GYRO: ")); Serial.print(yawAngle, 1);
    Serial.println(F("°"));
    delayWithGyro(150);
  }
}

// MODE 2: Tes Belok Presisi Gyro 90° & 180°
void jalankanTesBelokGyro() {
  Serial.println(F("\n--- [MODE 2] TES BELOK GYROSCOPE 90° & 180° ---"));
  Serial.println(F("Robot akan: Belok Kanan 90° -> Stop -> Belok Kiri 90° -> Stop -> Putar 180°"));
  delayWithGyro(2500);

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
    delayWithGyro(3000);
  }
}

// MODE 3: Tes Maju Lurus Terkunci Heading Lock
void jalankanTesHeadingLock() {
  Serial.println(F("\n--- [MODE 3] TES HEADING LOCK MAJU LURUS ---"));
  delayWithGyro(2500);

  while (true) {
    resetYaw();
    Serial.println(F("Maju lurus terkunci Gyro selama 3 detik..."));
    unsigned long start = millis();
    while (millis() - start < 3000) {
      forwardWithHeadingLock(SPEED_NORMAL, 0.0);
      delay(5);
    }
    rem();
    stopRobot(2000);
  }
}
