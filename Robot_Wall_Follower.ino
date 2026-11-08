/*
  =============================================================================
  WALL MAZE SOLVER ROBOT - Arduino UNO + L298N + 3x HC-SR04 (90° Side Sensors)
  =============================================================================
  Diadaptasi dari algoritma 'WallMazeSolver' GitHub dengan:
  - Konfigurasi Pin asli milik robot kamu (L298N & HC-SR04).
  - Sensor samping Kiri & Kanan terpasang TEGAK LURUS 90° (tanpa rumus trigonometri/cos45).
  - Dimensi Lintasan: Lebar lorong = 40 cm.
  - Dimensi Robot: Lebar bodi = 16 cm.
  - Saat posisi pas di tengah lorong:
      Jarak sensor Kanan = (40 - 16) / 2 = 12 cm.
      Jarak sensor Kiri  = (40 - 16) / 2 = 12 cm.
  - Ambang batas target tengah: 11 - 13 cm.
  - Deteksi celah / jalan terbuka: > 22 cm.
  - Henti depan: 12 cm (mundur sejenak lalu lakukan evaluasi belokan 90° / 180°).

  Mode Kalibrasi / Tes (ubah nilai MODE_OPERASI di bawah):
    0 = Jalankan Robot Wall Maze Solver Normal
    1 = Tes Sensor Ultrasonik (Serial Monitor @115200, motor mati)
    2 = Tes Belok 90° & 180° (Kalibrasi waktu putar agar pas)
    3 = Tes Motor Maju - Mundur - Stop
  =============================================================================
*/

// ======================== PILIH MODE ========================
const int MODE_OPERASI = 0;  // 0: Normal, 1: Tes Sensor, 2: Tes Belok, 3: Tes Motor

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

// Pin Opsional (INPUT_PULLUP, hubungkan ke GND jika ingin aktif):
const int PIN_JUMPER_DINDING = A0;  // LOW = Aturan Dinding KIRI, HIGH/Lepas = KANAN
const int PIN_TOMBOL_START   = A1;  // Tekan untuk mulai jalan

// ======================== PARAMETER DIMENSI & AMBANG ========================
const float LEBAR_LORONG   = 40.0;   // Lebar lintasan (cm)
const float LEBAR_ROBOT    = 16.0;   // Lebar robot (cm)

// Saat robot pas di tengah lorong: (40 - 16) / 2 = 12 cm
const float TARGET_TENGAH  = (LEBAR_LORONG - LEBAR_ROBOT) / 2.0;  // 12.0 cm

// Zona toleransi tengah (cm)
const float AMBANG_DEKAT   = 11.0;   // < 11 cm: terlalu dekat dinding
const float AMBANG_JAUH    = 13.0;   // > 13 cm: mulai menjauhi dinding
const float AMBANG_TERBUKA = 22.0;   // > 22 cm: dinding hilang / ada celah / simpang

// Jarak sensor depan untuk berhenti sebelum belok (cm) - dinaikkan ke 15 cm agar aman tidak nabrak
const float DEPAN_STOP     = 15.0;   // Berhenti saat jarak depan <= 15 cm (ada ruang rem aman)

// ======================== KECEPATAN & MOTOR ========================
// Diturunkan 40% (menjadi 60% dari kecepatan awal) agar sensor sempat mendeteksi
const int SPEED_NORMAL     = 80;     // Kecepatan maju lurus (sebelumnya 135)
const int SPEED_SLOW       = 70;     // Kecepatan belok halus lambat (sebelumnya 115, batas aman deadband L298N)
const int SPEED_FAST       = 85;     // Kecepatan belok halus cepat (sebelumnya 140)
const int SPEED_TURN       = 95;     // Kecepatan putar di tempat (sebelumnya 150)
const int SPEED_BACK       = 70;     // Kecepatan mundur sedikit sebelum putar (sebelumnya 110)

// Kalibrasi putaran berbasis waktu (titik tengah pas antara 200ms [45°] dan 300ms [135°]):
const unsigned long TURN_90_MS  = 250;  // Titik tengah pas 90°
const unsigned long TURN_180_MS = 500;  // Titik pas putar balik 180°

// Trim motor jika salah satu motor lebih cepat dari yang lain:
const float TRIM_KIRI  = 0.89;
const float TRIM_KANAN = 1.00;

// Opsi Aturan Dinding Utama (true = Ikuti Dinding Kanan, false = Ikuti Dinding Kiri):
const bool IKUTI_KANAN_DEFAULT  = true;
const bool PAKAI_JUMPER_DINDING = false;
const bool PAKAI_TOMBOL_START   = false;

bool ikutiKanan = IKUTI_KANAN_DEFAULT;

// ======================== FUNGSI SENSOR ========================
// Mengukur jarak ultrasonik (cm). Timeout 8000 us (~137 cm) agar loop cepat dan tidak nge-freeze
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

  // Motor Kiri
  digitalWrite(IN1, (kiri >= 0) ? HIGH : LOW);
  digitalWrite(IN2, (kiri >= 0) ? LOW  : HIGH);
  analogWrite(ENA, abs(kiri));

  // Motor Kanan
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
  // Rem aktif singkat untuk menghentikan inersia roda
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);   analogWrite(ENB, 255);
  delay(35);
  stopRobot(0);
}

// Belok putar di tempat (pivot turn)
void turnRight90() {
  Serial.println(F("[GERAK] Belok Kanan 90°"));
  rem();
  delay(50);
  setMotor(SPEED_TURN, -SPEED_TURN);
  delay(TURN_90_MS);
  rem();
  delay(50);
}

void turnLeft90() {
  Serial.println(F("[GERAK] Belok Kiri 90°"));
  rem();
  delay(50);
  setMotor(-SPEED_TURN, SPEED_TURN);
  delay(TURN_90_MS);
  rem();
  delay(50);
}

void turn180() {
  Serial.println(F("[GERAK] Putar Balik 180°"));
  rem();
  delay(50);
  // Putar balik ke arah sesuai preferensi
  if (ikutiKanan) {
    setMotor(SPEED_TURN, -SPEED_TURN);
  } else {
    setMotor(-SPEED_TURN, SPEED_TURN);
  }
  delay(TURN_180_MS);
  rem();
  delay(50);
}

// ======================== SETUP ========================
void setup() {
  Serial.begin(115200);

  // Konfigurasi pin Motor
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // Konfigurasi pin Ultrasonik
  pinMode(TRIG_KANAN, OUTPUT); pinMode(ECHO_KANAN, INPUT);
  pinMode(TRIG_DEPAN, OUTPUT); pinMode(ECHO_DEPAN, INPUT);
  pinMode(TRIG_KIRI,  OUTPUT); pinMode(ECHO_KIRI,  INPUT);

  // Jumper arah dinding (jika dipakai)
  if (PAKAI_JUMPER_DINDING) {
    pinMode(PIN_JUMPER_DINDING, INPUT_PULLUP);
    ikutiKanan = (digitalRead(PIN_JUMPER_DINDING) == HIGH);
  } else {
    ikutiKanan = IKUTI_KANAN_DEFAULT;
  }

  stopRobot(0);

  Serial.println(F("========================================"));
  Serial.println(F("WALL MAZE SOLVER ROBOT SIAP"));
  Serial.print(F("Mode Operasi: ")); Serial.println(MODE_OPERASI);
  Serial.print(F("Aturan Dinding: ")); Serial.println(ikutiKanan ? F("KANAN") : F("KIRI"));
  Serial.println(F("========================================"));

  // Jalankan Mode Kalibrasi jika dipilih
  if (MODE_OPERASI == 1) {
    jalankanTesSensor();
  } else if (MODE_OPERASI == 2) {
    jalankanTesBelok();
  } else if (MODE_OPERASI == 3) {
    jalankanTesMotor();
  }

  // Tunggu tombol start atau delay persiapan awal
  if (PAKAI_TOMBOL_START) {
    pinMode(PIN_TOMBOL_START, INPUT_PULLUP);
    Serial.println(F("Menunggu tombol START ditekan..."));
    while (digitalRead(PIN_TOMBOL_START) == HIGH) {
      delay(50);
    }
    delay(1000);
  } else {
    delay(2000);  // Jeda 2 detik sebelum robot mulai bergerak
  }

  Serial.println(F("Robot Mulai Bergerak!"));
}

// ======================== LOOP UTAMA (LOGIKA REPO) ========================
void loop() {
  // 1. Baca ketiga sensor ultrasonik secara berurutan
  float cmF  = bacaDepan();
  delay(6);
  float cmRF = bacaKanan();
  delay(6);
  float cmLF = bacaKiri();

  // Tampilkan data ke Serial Monitor
  Serial.print(F("LF: ")); Serial.print(cmLF, 1);
  Serial.print(F(" | F: ")); Serial.print(cmF, 1);
  Serial.print(F(" | RF: ")); Serial.print(cmRF, 1);

  // -------------------------------------------------------------
  // KONDISI A: DEPAN BEBAS (cmF > DEPAN_STOP) -> WALL FOLLOWING
  // -------------------------------------------------------------
  if (cmF > DEPAN_STOP) {
    if (ikutiKanan) {
      // ===== LOGIKA PENELUSURAN DINDING KANAN =====
      // Ideal tengah: 11 - 13 cm
      if (cmRF >= AMBANG_DEKAT && cmRF <= AMBANG_JAUH) {
        forward(SPEED_NORMAL, SPEED_NORMAL);
        Serial.println(F(" -> Maju Lurus (Kanan Ideal)"));
      }
      else if (cmRF > AMBANG_JAUH) {
        if (cmRF >= AMBANG_TERBUKA) {
          // Dinding kanan hilang (celah kanan / simpang)
          // Berpindah sementara memandu dengan dinding kiri (seperti di WallMazeSolver.ino)
          if (cmLF >= AMBANG_DEKAT && cmLF <= AMBANG_JAUH) {
            forward(SPEED_NORMAL, SPEED_NORMAL);
            Serial.println(F(" -> Maju Lurus (Pemandu Kiri)"));
          }
          else if (cmLF > AMBANG_JAUH && cmLF < AMBANG_TERBUKA) {
            forward(SPEED_SLOW, SPEED_FAST);  // Agak jauh dari kiri -> serong ke kiri
            Serial.println(F(" -> Koreksi Kiri Halus"));
          }
          else if (cmLF < AMBANG_DEKAT) {
            forward(SPEED_FAST, SPEED_SLOW);  // Terlalu dekat ke kiri -> serong ke kanan
            Serial.println(F(" -> Koreksi Kanan Halus"));
          }
          else {
            // Kedua sisi terbuka (lapangan terbuka) -> tetap maju lurus
            forward(SPEED_NORMAL, SPEED_NORMAL);
            Serial.println(F(" -> Maju Lurus (Kedua Sisi Terbuka)"));
          }
        } else {
          // Dinding kanan masih ada tapi agak menjauh (13 - 22 cm) -> belok kanan halus
          forward(SPEED_FAST, SPEED_SLOW);
          Serial.println(F(" -> Koreksi Kanan Halus"));
        }
      }
      else if (cmRF < AMBANG_DEKAT) {
        // Terlalu dekat ke dinding kanan (< 11 cm) -> belok kiri halus menjauh
        forward(SPEED_SLOW, SPEED_FAST);
        Serial.println(F(" -> Koreksi Kiri Halus (Terlalu Mepet Kanan)"));
      }

    } else {
      // ===== LOGIKA PENELUSURAN DINDING KIRI =====
      // Ideal tengah: 11 - 13 cm
      if (cmLF >= AMBANG_DEKAT && cmLF <= AMBANG_JAUH) {
        forward(SPEED_NORMAL, SPEED_NORMAL);
        Serial.println(F(" -> Maju Lurus (Kiri Ideal)"));
      }
      else if (cmLF > AMBANG_JAUH) {
        if (cmLF >= AMBANG_TERBUKA) {
          // Dinding kiri hilang -> gunakan pemandu dinding kanan
          if (cmRF >= AMBANG_DEKAT && cmRF <= AMBANG_JAUH) {
            forward(SPEED_NORMAL, SPEED_NORMAL);
            Serial.println(F(" -> Maju Lurus (Pemandu Kanan)"));
          }
          else if (cmRF > AMBANG_JAUH && cmRF < AMBANG_TERBUKA) {
            forward(SPEED_FAST, SPEED_SLOW);  // Serong ke kanan
            Serial.println(F(" -> Koreksi Kanan Halus"));
          }
          else if (cmRF < AMBANG_DEKAT) {
            forward(SPEED_SLOW, SPEED_FAST);  // Serong ke kiri
            Serial.println(F(" -> Koreksi Kiri Halus"));
          }
          else {
            forward(SPEED_NORMAL, SPEED_NORMAL);
            Serial.println(F(" -> Maju Lurus (Kedua Sisi Terbuka)"));
          }
        } else {
          // Dinding kiri agak menjauh -> belok kiri halus
          forward(SPEED_SLOW, SPEED_FAST);
          Serial.println(F(" -> Koreksi Kiri Halus"));
        }
      }
      else if (cmLF < AMBANG_DEKAT) {
        // Terlalu dekat ke dinding kiri -> belok kanan halus menjauh
        forward(SPEED_FAST, SPEED_SLOW);
        Serial.println(F(" -> Koreksi Kanan Halus (Terlalu Mepet Kiri)"));
      }
    }
  }

  // -------------------------------------------------------------
  // KONDISI B: DEPAN TERHALANG (cmF <= DEPAN_STOP) -> MANUVER BELOK
  // -------------------------------------------------------------
  else {
    Serial.println(F("\n[HALANGAN] Depan Terhalang! Melakukan manuver..."));

    // Mundur sedikit untuk memberi ruang manuver putar (sesuai logika repo)
    back(SPEED_BACK, SPEED_BACK);
    delay(120);
    rem();
    stopRobot(250);

    // Baca ulang jarak samping untuk memastikan kondisi ruang
    delay(10);
    cmRF = bacaKanan();
    delay(10);
    cmLF = bacaKiri();

    bool kananBebas = (cmRF > AMBANG_TERBUKA);
    bool kiriBebas  = (cmLF > AMBANG_TERBUKA);

    Serial.print(F("Status Sisi -> Kiri Bebas: ")); Serial.print(kiriBebas ? F("YA") : F("TIDAK"));
    Serial.print(F(" | Kanan Bebas: ")); Serial.println(kananBebas ? F("YA") : F("TIDAK"));

    // 1. Kiri dinding, Kanan terbuka -> Belok Kanan 90°
    if (!kiriBebas && kananBebas) {
      turnRight90();
    }
    // 2. Kiri terbuka, Kanan dinding -> Belok Kiri 90°
    else if (kiriBebas && !kananBebas) {
      turnLeft90();
    }
    // 3. Persimpangan T (Kedua sisi terbuka)
    else if (kiriBebas && kananBebas) {
      if (ikutiKanan) {
        turnRight90();  // Prioritas Kanan
      } else {
        turnLeft90();   // Prioritas Kiri
      }
    }
    // 4. Jalan Buntu (Kedua sisi tertutup dinding) -> Putar Balik 180°
    else {
      turn180();
    }

    // Setelah belok, maju sedikit agar keluar dari zona persimpangan/tikungan
    forward(SPEED_NORMAL, SPEED_NORMAL);
    delay(200);
  }

  delay(20);  // Jeda siklus loop
}

// ======================== MODUL TES & KALIBRASI ========================

// MODE 1: Tes Pembacaan Sensor Ultrasonik
void jalankanTesSensor() {
  Serial.println(F("\n--- [MODE 1] TES SENSOR ULTRASONIK (Motor MATI) ---"));
  Serial.println(F("Letakkan robot di lorong 40 cm untuk menguji pembacaan:"));
  Serial.println(F("Target ideal saat di tengah: Kiri ~12cm, Kanan ~12cm.\n"));

  while (true) {
    float f  = bacaDepan();
    delay(15);
    float ka = bacaKanan();
    delay(15);
    float ki = bacaKiri();

    Serial.print(F("KIRI: "));
    if (ki >= 999.0) Serial.print(F("TIMEOUT")); else Serial.print(ki, 1);
    Serial.print(F(" cm | DEPAN: "));
    if (f >= 999.0) Serial.print(F("TIMEOUT")); else Serial.print(f, 1);
    Serial.print(F(" cm | KANAN: "));
    if (ka >= 999.0) Serial.print(F("TIMEOUT")); else Serial.print(ka, 1);
    Serial.print(F(" cm  ==> "));

    // Status analisis
    if (f <= DEPAN_STOP) {
      Serial.println(F("[DEPAN TERHALANG - PERLU BELOK]"));
    } else if (ka < AMBANG_DEKAT) {
      Serial.println(F("[TERLALU DEKAT KANAN -> KOREKSI KIRI]"));
    } else if (ka > AMBANG_TERBUKA) {
      Serial.println(F("[CELAH KANAN TERBUKA]"));
    } else if (ka >= AMBANG_DEKAT && ka <= AMBANG_JAUH) {
      Serial.println(F("[POSISI TENGAH OKE]"));
    } else {
      Serial.println(F("[MENJAUHI KANAN -> KOREKSI KANAN]"));
    }

    delay(250);
  }
}

// MODE 2: Tes Belok 90° dan 180° (Kalibrasi Waktu Putar)
void jalankanTesBelok() {
  Serial.println(F("\n--- [MODE 2] TES BELOK 90° & 180° ---"));
  Serial.println(F("Robot akan melakukan: Belok Kanan 90° -> Jeda -> Belok Kiri 90° -> Jeda -> Putar 180°"));
  Serial.println(F("Jika sudut belum pas 90° atau 180°, ubah nilai TURN_90_MS dan TURN_180_MS."));
  delay(3000);

  while (true) {
    Serial.println(F("Mulai: Belok Kanan 90°..."));
    turnRight90();
    stopRobot(1500);

    Serial.println(F("Mulai: Belok Kiri 90° (kembali lurus)..."));
    turnLeft90();
    stopRobot(1500);

    Serial.println(F("Mulai: Putar Balik 180°..."));
    turn180();
    stopRobot(2500);

    Serial.println(F("Satu siklus selesai. Mengulang dalam 3 detik...\n"));
    delay(3000);
  }
}

// MODE 3: Tes Motor (Maju, Berhenti, Mundur seperti di MotorTests.ino repo)
void jalankanTesMotor() {
  Serial.println(F("\n--- [MODE 3] TES MOTOR (Maju - Stop - Mundur) ---"));
  delay(3000);

  while (true) {
    Serial.println(F("Maju lurus selama 2 detik..."));
    forward(SPEED_NORMAL, SPEED_NORMAL);
    delay(2000);

    Serial.println(F("Berhenti 1 detik..."));
    rem();
    stopRobot(1000);

    Serial.println(F("Mundur selama 1.5 detik..."));
    back(SPEED_NORMAL, SPEED_NORMAL);
    delay(1500);

    Serial.println(F("Berhenti 2 detik...\n"));
    rem();
    stopRobot(2000);
  }
}
