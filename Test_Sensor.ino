/*
  =============================================================================
  TEST SENSOR ULTRASONIK (3x HC-SR04 dengan Samping 90°)
  Disesuaikan dengan Pin Robot Kamu
  =============================================================================
  - Menampilkan jarak cm untuk Sensor Kiri, Depan, dan Kanan.
  - Menampilkan pembacaan real-time di Serial Monitor (@115200 baud).
  - Menguji kalibrasi jarak di lorong 40 cm (robot 16 cm).
    Ideal di tengah: Kiri ~12 cm, Kanan ~12 cm.
  =============================================================================
*/

// ================== PIN SENSOR ULTRASONIK ==================
const int TRIG_KANAN = 3;
const int ECHO_KANAN = 2;

const int TRIG_DEPAN = 5;
const int ECHO_DEPAN = 4;

const int TRIG_KIRI  = 7;
const int ECHO_KIRI  = 6;

// Ambang batas referensi
const float TARGET_TENGAH  = 12.0;  // (40 - 16) / 2 = 12 cm
const float AMBANG_DEKAT   = 11.0;
const float AMBANG_JAUH    = 13.0;
const float AMBANG_TERBUKA = 22.0;
const float DEPAN_STOP     = 15.0;  // Jarak henti aman 15 cm

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

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_KANAN, OUTPUT); pinMode(ECHO_KANAN, INPUT);
  pinMode(TRIG_DEPAN, OUTPUT); pinMode(ECHO_DEPAN, INPUT);
  pinMode(TRIG_KIRI,  OUTPUT); pinMode(ECHO_KIRI,  INPUT);

  Serial.println(F("=================================================="));
  Serial.println(F("       TES PEMBACAAN 3 SENSOR ULTRASONIK          "));
  Serial.println(F("=================================================="));
  Serial.println(F("Buka Serial Monitor pada Baud Rate: 115200"));
  Serial.println(F("Letakkan robot di lorong 40 cm untuk melihat nilai."));
  Serial.println(F("--------------------------------------------------\n"));
  delay(1000);
}

void loop() {
  float f  = pingCm(TRIG_DEPAN, ECHO_DEPAN);
  delay(15);
  float ka = pingCm(TRIG_KANAN, ECHO_KANAN);
  delay(15);
  float ki = pingCm(TRIG_KIRI,  ECHO_KIRI);

  Serial.print(F("KIRI: "));
  if (ki >= 999.0) Serial.print(F("---")); else Serial.print(ki, 1);
  Serial.print(F(" cm\t| DEPAN: "));
  if (f >= 999.0) Serial.print(F("---")); else Serial.print(f, 1);
  Serial.print(F(" cm\t| KANAN: "));
  if (ka >= 999.0) Serial.print(F("---")); else Serial.print(ka, 1);
  Serial.print(F(" cm\t==> "));

  // Analisis Kondisi
  if (f <= DEPAN_STOP && f > 0) {
    Serial.println(F("Depan MENTOK! Siap belok."));
  } else if (ka > AMBANG_TERBUKA && ki > AMBANG_TERBUKA) {
    Serial.println(F("Lorong Terbuka / Persimpangan Luas."));
  } else if (ka > AMBANG_TERBUKA) {
    Serial.println(F("Celah Kanan Terbuka."));
  } else if (ki > AMBANG_TERBUKA) {
    Serial.println(F("Celah Kiri Terbuka."));
  } else if (ka >= AMBANG_DEKAT && ka <= AMBANG_JAUH) {
    Serial.println(F("Posisi Pas di TENGAH (OK)."));
  } else if (ka < AMBANG_DEKAT) {
    Serial.println(F("Terlalu Mepet Dinding Kanan!"));
  } else {
    Serial.println(F("Agak Jauh dari Kanan."));
  }

  delay(200);
}
