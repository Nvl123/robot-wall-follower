/*
  WALL FOLLOWER v2 - Arduino UNO + L298N + 3x HC-SR04
  Sensor samping serong 45° (kiri & kanan), 1 sensor depan.
  Roda: 2 motor penggerak + 1 caster (roda koper) di depan.
  Arena: lebar jalan 40 cm, sel 40x40 cm, dinding 15 cm.

  Perbaikan utama dibanding v1:
  - Jarak diukur dari GARIS TENGAH robot (cos45 + offset sensor), bukan dari sensor.
  - Dua dinding terlihat -> robot menjaga posisi di TENGAH jalan (error = (ka-ki)/2).
    Satu dinding terlihat -> pakai target 20 cm dari garis tengah.
  - Belokan di celah: maju dulu sampai poros roda tepat di tengah celah
    (sensor serong "melihat" celah ~1 sel lebih awal), baru putar 90°.
  - Debounce + ping ulang untuk dropout pantulan 45° (anti salah deteksi celah).
  - Belok depan terhalang: pilih arah sesuai sisi yang terbuka, buntu -> putar balik 180°.
  - Putar berbasis waktu terkalibrasi + verifikasi sensor depan.
  - Jadwal loop tetap 40 ms (turunan PD konsisten), perlambatan mendekati dinding depan.
  - Anti macet (mundur + putar kecil), rem aktif, trim motor, mode kalibrasi.
*/

// ================== PIN ==================
const int ENA = 11;   // PWM motor kiri
const int IN1 = 8;
const int IN2 = 9;
const int ENB = 10;   // PWM motor kanan
const int IN3 = 12;
const int IN4 = 13;

const int TRIG_KANAN = 3;
const int ECHO_KANAN = 2;
const int TRIG_DEPAN = 5;
const int ECHO_DEPAN = 4;
const int TRIG_KIRI  = 7;
const int ECHO_KIRI  = 6;

// Opsional (INPUT_PULLUP, sambungkan ke GND untuk aktif)
const int PIN_JUMPER_DINDING = A0;  // LOW = ikuti dinding KIRI, HIGH/terbuka = KANAN
const int PIN_TOMBOL_START   = A1;  // tekan = mulai jalan

// ================== PENGATURAN ==================
const bool IKUTI_KANAN_DEFAULT   = true;   // dipakai jika jumper tidak dipakai
const bool PAKAI_JUMPER_DINDING  = false;
const bool PAKAI_TOMBOL_START    = false;  // false = tunggu 2 detik lalu jalan
const bool DEBUG                 = true;

// 0 = normal, 1 = maju 2 detik (ukur jarak -> MS_PER_CM), 2 = putar kanan 4x90° (harus pas 360°)
// 3 = TES SENSOR saja (motor mati, cetak nilai mentah ke Serial Monitor @115200 baud)
const byte MODE_KALIBRASI = 0;

// ---- Geometri (cm) -> UKUR di robot kamu ----
const float LEBAR_JALAN = 40.0;
const float LEBAR_ROBOT = 14.0;  // lebar badan robot
const float SENSOR_LAT  = LEBAR_ROBOT / 2.0;  // sensor serong di tepi badan; jika dipasang lebih ke dalam, isi manual (ukur dari garis tengah)
const float DEPAN_FWD   = 8.0;   // jarak sensor DEPAN di depan poros roda (UKUR!)
const float SENSOR_FWD  = 8.0;   // jarak sensor serong di depan poros roda
const float TARGET_LAT  = LEBAR_JALAN / 2.0;  // posisi ideal: tepat di tengah jalan (20 cm)
const float COS45       = 0.7071;

// ---- Ambang ----
// Lateral > LAT_HILANG -> dinding hilang / ada celah.
// Robot yang mentok ke satu dinding masih melihat dinding seberang di 20 + (40-14)/2 = 33 cm, +4 cm margin.
const float LAT_HILANG  = TARGET_LAT + (LEBAR_JALAN - LEBAR_ROBOT) / 2.0 + 4.0;
const float LAT_MAX     = 120.0; // nilai "tidak ada dinding"
const float DEPAN_STOP  = LEBAR_JALAN / 2.0 - DEPAN_FWD;  // berhenti saat poros roda tepat di tengah sel
const float DEPAN_AMAN  = DEPAN_STOP + 9.0;
const float DEPAN_PERLAMBAT = 45.0;

// ---- Kecepatan (0-255) ----
const int BASE_SPEED    = 160;
const int SLOW_SPEED    = 110;
const int TURN_SPEED    = 170;
const int MAX_SPEED     = 240;
const int MIN_PWM_JALAN = 70;     // di bawah ini motor biasanya tidak bergerak (deadband L298N)
const float TRIM_KIRI   = 1.00;   // kompensasi motor tidak identik (mis. kanan lebih kencang -> TRIM_KANAN 0.95)
const float TRIM_KANAN  = 1.00;

// ---- PD (error dalam cm, siklus 40 ms) ----
const float KP = 6.0;
const float KD = 14.0;
const unsigned long LOOP_MS = 40;

// ---- Kalibrasi gerak (WAJIB disetel di lintasan asli) ----
const float MS_PER_CM            = 32.0;  // ms untuk maju 1 cm pada BASE_SPEED
const unsigned long PIVOT_90_MS  = 360;   // ms putar di tempat 90° pada TURN_SPEED
const unsigned long GRACE_MS     = 300;   // maju lurus setelah belok (deteksi celah dimatikan)

// ---- Logika ----
const int  HILANG_KONFIRMASI   = 2;       // siklus berturut-turut sebelum dianggap celah
const int  PREF_STABIL_MIN     = 5;       // dinding acuan harus terlihat stabil sekian siklus sebelum "celah" boleh dideteksi
const unsigned long STUCK_MS   = 1500;

// ---- Timeout sensor (us) ----
const unsigned long TIMEOUT_DEPAN_US = 8000;  // ~137 cm
const unsigned long TIMEOUT_SISI_US  = 5000;  // ~86 cm (cukup untuk deteksi celah)

// ================== VARIABEL ==================
bool  ikutiKanan = IKUTI_KANAN_DEFAULT;
int   prefDir = 1;                // +1 = kanan, -1 = kiri
float lastErr = 0;
bool  adaErrLalu = false;
float lastPrefLat = TARGET_LAT;
int   hilangCnt = 0, prefStabilCnt = 0;
bool  prefPernahStabil = false;   // true = sebelumnya benar-benar sedang mengikuti dinding
float refDepan = -999, refSisi = 0;
unsigned long tRef = 0, tLoop = 0, tDebug = 0;

// ================== SENSOR ==================
// Mengembalikan cm mentah, 999 jika tidak ada pantulan
float pingCm(int trig, int echo, unsigned long timeoutUs) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  unsigned long dur = pulseIn(echo, HIGH, timeoutUs);
  if (dur == 0) return 999.0;
  return dur / 58.0;
}

float bacaDepan() {
  float d = pingCm(TRIG_DEPAN, ECHO_DEPAN, TIMEOUT_DEPAN_US);
  return (d >= 999.0) ? 200.0 : d;
}

// Jarak lateral dari GARIS TENGAH robot ke dinding (cm).
// raw * cos45 = jarak tegak lurus dari sensor; + SENSOR_LAT = dari garis tengah.
float bacaLateral(int trig, int echo) {
  float d = pingCm(trig, echo, TIMEOUT_SISI_US);
  float lat = (d >= 999.0) ? LAT_MAX : d * COS45 + SENSOR_LAT;
  if (lat > LAT_HILANG) {             // bisa jadi dropout pantulan 45° -> ping ulang sekali
    delay(2);
    float d2 = pingCm(trig, echo, TIMEOUT_SISI_US);
    if (d2 < 999.0) {
      float l2 = d2 * COS45 + SENSOR_LAT;
      if (l2 < lat) lat = l2;
    }
  }
  return lat;
}

// err > 0 => robot harus belok KANAN, err < 0 => belok KIRI
bool hitungError(float kananLat, float kiriLat, float &err) {
  bool ka = kananLat <= LAT_HILANG;
  bool ki = kiriLat  <= LAT_HILANG;
  if (ka && ki)  err = (kananLat - kiriLat) / 2.0;  // jaga posisi tengah
  else if (ka)   err = kananLat - TARGET_LAT;
  else if (ki)   err = TARGET_LAT - kiriLat;
  else return false;
  return true;
}

// ================== MOTOR ==================
void setMotor(int kiri, int kanan) {
  kiri  = constrain((int)(kiri  * TRIM_KIRI),  -255, 255);
  kanan = constrain((int)(kanan * TRIM_KANAN), -255, 255);

  digitalWrite(IN1, kiri >= 0);
  digitalWrite(IN2, kiri < 0);
  analogWrite(ENA, abs(kiri));

  digitalWrite(IN3, kanan >= 0);
  digitalWrite(IN4, kanan < 0);
  analogWrite(ENB, abs(kanan));
}

void berhenti() { setMotor(0, 0); }

void rem() {                       // rem aktif singkat
  digitalWrite(IN1, HIGH); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255);   analogWrite(ENB, 255);
  delay(30);
  berhenti();
}

void resetState() {
  adaErrLalu = false;
  lastErr = 0;
  hilangCnt = 0;
  refDepan = -999;
  tLoop = millis();
}

void ikuti(float err, int base) {
  float d = adaErrLalu ? (err - lastErr) : 0;   // tanpa "kick" turunan di awal
  float koreksi = KP * err + KD * d;
  lastErr = err;
  adaErrLalu = true;

  int kiri  = base + (int)koreksi;
  int kanan = base - (int)koreksi;
  setMotor(constrain(kiri,  MIN_PWM_JALAN, MAX_SPEED),
           constrain(kanan, MIN_PWM_JALAN, MAX_SPEED));
}

// Putar di tempat. arah: +1 kanan, -1 kiri.
// cekDepan=true : minimal 90% target, lanjut sampai depan bebas (maks 1.6x target)
// cekDepan=false: persis sesuai waktu target
void putar(int arah, unsigned long target, bool cekDepan) {
  rem();
  delay(40);
  unsigned long mulai = millis();
  unsigned long minMs = (target * 9) / 10;
  unsigned long maxMs = (target * 16) / 10;
  while (true) {
    unsigned long t = millis() - mulai;
    setMotor(arah * TURN_SPEED, -arah * TURN_SPEED);
    if (cekDepan) {
      if (t >= maxMs) break;
      if (t >= minMs && bacaDepan() > DEPAN_AMAN) break;
    } else {
      if (t >= target) break;
    }
  }
  rem();
  delay(40);
  resetState();
}

// Maju lurus selama ms, koreksi pakai dinding yang masih terlihat, berhenti jika depan terhalang
void majuLurus(unsigned long ms) {
  unsigned long mulai = millis();
  while (millis() - mulai < ms) {
    unsigned long t0 = millis();
    if (bacaDepan() < DEPAN_STOP) break;
    delay(3);
    float ka = bacaLateral(TRIG_KANAN, ECHO_KANAN);
    delay(3);
    float ki = bacaLateral(TRIG_KIRI, ECHO_KIRI);
    float err;
    if (hitungError(ka, ki, err)) ikuti(err, BASE_SPEED);
    else setMotor(BASE_SPEED, BASE_SPEED);
    while (millis() - t0 < LOOP_MS) {}
  }
  resetState();
}

void pulihDariMacet() {
  setMotor(-BASE_SPEED, -BASE_SPEED);
  delay(350);
  rem();
  putar(-prefDir, PIVOT_90_MS / 3, false);   // putar kecil menjauhi dinding acuan
}

// ================== SETUP & LOOP ==================
void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(TRIG_KANAN, OUTPUT); pinMode(ECHO_KANAN, INPUT);
  pinMode(TRIG_DEPAN, OUTPUT); pinMode(ECHO_DEPAN, INPUT);
  pinMode(TRIG_KIRI,  OUTPUT); pinMode(ECHO_KIRI,  INPUT);

  if (PAKAI_JUMPER_DINDING) {
    pinMode(PIN_JUMPER_DINDING, INPUT_PULLUP);
    ikutiKanan = (digitalRead(PIN_JUMPER_DINDING) == HIGH);
  }
  prefDir = ikutiKanan ? 1 : -1;

  Serial.begin(115200);
  berhenti();

  // MODE 3: tes sensor saja (motor mati). Nilai mentah tanpa filter, update tiap ~200 ms.
  // 999 = sensor tidak menerima pantulan sama sekali.
  if (MODE_KALIBRASI == 3) {
    while (true) {
      float d  = pingCm(TRIG_DEPAN,  ECHO_DEPAN,  25000UL); delay(40);
      float ka = pingCm(TRIG_KANAN,  ECHO_KANAN,  25000UL); delay(40);
      float ki = pingCm(TRIG_KIRI,   ECHO_KIRI,   25000UL); delay(40);
      Serial.print("Depan:"); Serial.print(d);
      Serial.print("  Kanan:"); Serial.print(ka);
      Serial.print("  Kiri:"); Serial.println(ki);
      delay(80);
    }
  }

  if (PAKAI_TOMBOL_START) {
    pinMode(PIN_TOMBOL_START, INPUT_PULLUP);
    while (digitalRead(PIN_TOMBOL_START) == HIGH) {}
    delay(1000);
  } else {
    delay(2000);   // waktu meletakkan robot di start
  }

  if (MODE_KALIBRASI == 1) {
    setMotor(BASE_SPEED, BASE_SPEED);
    delay(2000);
    rem();
    while (true) {}   // ukur jarak tempuh -> MS_PER_CM = 2000 / jarak_cm
  }
  if (MODE_KALIBRASI == 2) {
    for (int i = 0; i < 4; i++) { putar(1, PIVOT_90_MS, false); delay(800); }
    while (true) {}   // harus kembali menghadap arah awal; sesuaikan PIVOT_90_MS
  }

  resetState();
}

void loop() {
  while (millis() - tLoop < LOOP_MS) {}
  tLoop = millis();

  // --- Baca sensor (berurutan + jeda agar tidak crosstalk) ---
  float depan = bacaDepan();
  delay(3);
  float kananLat = bacaLateral(TRIG_KANAN, ECHO_KANAN);
  delay(3);
  float kiriLat  = bacaLateral(TRIG_KIRI,  ECHO_KIRI);

  bool  kananAda = kananLat <= LAT_HILANG;
  bool  kiriAda  = kiriLat  <= LAT_HILANG;
  float prefLat  = ikutiKanan ? kananLat : kiriLat;
  bool  prefAda  = ikutiKanan ? kananAda : kiriAda;
  bool  oppAda   = ikutiKanan ? kiriAda  : kananAda;

  if (prefAda) lastPrefLat = prefLat;
  hilangCnt = prefAda ? 0 : hilangCnt + 1;
  if (prefAda) {
    if (prefStabilCnt < 250) prefStabilCnt++;
    if (prefStabilCnt >= PREF_STABIL_MIN) prefPernahStabil = true;
  } else {
    prefStabilCnt = 0;
  }

  if (DEBUG && millis() - tDebug > 200) {
    tDebug = millis();
    Serial.print("D:");  Serial.print(depan);
    Serial.print(" Ka:"); Serial.print(kananLat);
    Serial.print(" Ki:"); Serial.println(kiriLat);
  }

  // --- 0) Anti macet ---
  if (depan < 100.0 && fabs(depan - refDepan) < 1.5 && fabs(prefLat - refSisi) < 1.5) {
    if (millis() - tRef > STUCK_MS) {
      pulihDariMacet();
      return;
    }
  } else {
    refDepan = depan;
    refSisi = prefLat;
    tRef = millis();
  }

  // --- 1) Dinding depan terhalang ---
  if (depan < DEPAN_STOP) {
    if (!prefAda)      putar(prefDir,  PIVOT_90_MS, true);       // sisi acuan terbuka -> ke sana
    else if (!oppAda)  putar(-prefDir, PIVOT_90_MS, true);       // sisi lawan terbuka
    else               putar(-prefDir, PIVOT_90_MS * 2, true);   // buntu -> putar balik 180°
    majuLurus(GRACE_MS);
    prefPernahStabil = false;   // harus "menemukan" dinding acuan lagi sebelum boleh deteksi celah
    prefStabilCnt = 0;
    return;
  }

  // --- 2) Dinding acuan hilang (celah/tikungan) ---
  // Hanya jika sebelumnya memang sedang mengikuti dinding. Tanpa dinding sama sekali -> lurus.
  if (hilangCnt >= HILANG_KONFIRMASI && prefPernahStabil) {
    // Sensor serong melihat celah lebih awal. Maju sampai poros roda tepat di tengah celah.
    float p = lastPrefLat - SENSOR_LAT;            // jarak tegak lurus sensor-dinding terakhir
    p = constrain(p, 3.0, 15.0);
    float jarakCm = SENSOR_FWD + p + LEBAR_JALAN / 2.0;
    long ms = (long)(jarakCm * MS_PER_CM) - (long)hilangCnt * (long)LOOP_MS;
    if (ms > 0) majuLurus((unsigned long)ms);

    putar(prefDir, PIVOT_90_MS, false);
    majuLurus(GRACE_MS);
    prefPernahStabil = false;
    prefStabilCnt = 0;
    return;
  }

  // --- 3) Normal: PD, perlambat mendekati dinding depan ---
  int base = BASE_SPEED;
  if (depan < DEPAN_PERLAMBAT) {
    base = map((long)depan, (long)DEPAN_STOP, (long)DEPAN_PERLAMBAT, SLOW_SPEED, BASE_SPEED);
    base = constrain(base, SLOW_SPEED, BASE_SPEED);
  }

  float err;
  if (hitungError(kananLat, kiriLat, err)) ikuti(err, base);
  else setMotor(base, base);   // tidak ada dinding sama sekali: lurus sampai ketemu dinding
}
