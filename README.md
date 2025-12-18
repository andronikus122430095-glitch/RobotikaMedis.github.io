# Smart Nurse Call System  
**Mata Kuliah: Robotika Medis**

Smart Nurse Call System merupakan proyek Robotika Medis berbasis
mikrokontroler ESP32 dan Internet of Things (IoT) yang dirancang untuk
meningkatkan komunikasi antara pasien dan tenaga medis melalui sistem
panggilan otomatis yang responsif dan efisien.

---

## 📌 Latar Belakang
Dalam lingkungan rumah sakit, sistem pemanggil perawat konvensional masih
memiliki keterbatasan, seperti minimnya informasi kondisi pasien dan
keterlambatan respon. Oleh karena itu, diperlukan sistem berbasis robotika
medis dan IoT yang mampu meningkatkan kualitas pelayanan kesehatan.

---

## 🎯 Tujuan Sistem
- Mengembangkan sistem pemanggil perawat berbasis ESP32
- Menerapkan konsep IoT pada sistem pelayanan kesehatan
- Meningkatkan kecepatan dan akurasi respon perawat
- Sebagai media pembelajaran implementatif Robotika Medis

---

## 🧠 Arsitektur Sistem
Sistem terdiri dari ESP32 sebagai pengendali utama yang menerima input dari
tombol pasien dan memberikan output berupa indikator LED, buzzer, serta
tampilan LCD. Sistem dapat dikembangkan ke jaringan IoT dan middleware robotik.

---

## 🛠️ Komponen yang Digunakan

### Perangkat Keras
- ESP32
- Push Button
- LCD I2C
- LED
- Buzzer
- Breadboard dan kabel jumper

### Perangkat Lunak
- Arduino IDE
- Library LiquidCrystal_I2C
- GitHub Pages
- HTML, CSS, JavaScript

---

## 🔧 Tahapan Pembuatan Sistem

### 1. Instalasi Perangkat Lunak
- Menginstal Arduino IDE
- Menambahkan board ESP32 pada Boards Manager
- Menginstal library pendukung LCD
- Menghubungkan ESP32 ke komputer

### 2. Konfigurasi Jaringan dan Keamanan
- Mengatur koneksi WiFi pada ESP32
- Menyesuaikan SSID dan password jaringan
- Membatasi akses jaringan untuk menjaga keamanan sistem

### 3. Perakitan Perangkat Keras
- Memasang ESP32 pada breadboard
- Menghubungkan tombol sebagai input
- Menghubungkan LED dan buzzer sebagai output
- Menghubungkan LCD melalui komunikasi I2C
- Memastikan tidak terjadi kesalahan koneksi

### 4. Pemrograman Firmware (ESP32)
Firmware dikembangkan menggunakan Arduino IDE untuk membaca input tombol,
mengaktifkan indikator, dan menampilkan informasi pada LCD.

```cpp
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define BTN1 14
#define BTN2 27
#define BTN3 26
#define LED 33
#define BUZZER 25

void setup() {
  pinMode(BTN1, INPUT_PULLDOWN);
  pinMode(BTN2, INPUT_PULLDOWN);
  pinMode(BTN3, INPUT_PULLDOWN);
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.print("Nurse Call Ready");
}

void loop() {
  if (digitalRead(BTN1)) callNurse("Pasien 1");
  if (digitalRead(BTN2)) callNurse("Pasien 2");
  if (digitalRead(BTN3)) callNurse("Pasien 3");
}

void callNurse(String pasien) {
  digitalWrite(LED, HIGH);
  digitalWrite(BUZZER, HIGH);
  lcd.clear();
  lcd.print("Call:");
  lcd.setCursor(0,1);
  lcd.print(pasien);
  delay(3000);
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER, LOW);
  lcd.clear();
}
