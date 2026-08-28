# ADE7880 Read Test — Firmware Diagnostik

Firmware minimal yang **hanya** membaca ADE7880. Semua subsistem lain
(modem EC25, MQTT, FRAM, RTC, ADC, TIM, relay) dihilangkan.

Tujuannya satu: memisahkan variabel. Kalau di firmware ini pembacaan
**stabil**, berarti masalah di project utama berasal dari interaksi dengan
subsistem lain (paling mungkin interrupt UART3 modem yang menyela bit-bang
SPI). Kalau di sini **tetap tidak stabil**, berarti masalahnya murni di
chip / rangkaian / driver SPI, dan modem tidak ada hubungannya.

---

## Yang Dipertahankan Persis Seperti Project Asli

| Aspek | Nilai |
|---|---|
| MCU | STM32F103RGTx |
| Clock | **Kristal eksternal HSE + PLL ×9 → 72 MHz** (identik) |
| Watchdog | **IWDG Prescaler 256 / Reload 4095 ≈ 26 detik** (identik) |
| Peta pin ADE7880 | identik |
| Driver ADE7880 | file yang sama, hanya ditambah penangkap nilai mentah |
| Konstanta kalibrasi | identik |

## Yang Dihilangkan

USART3 (modem), UART4, MQTT, I2C/FRAM, RTC, ADC, TIM2, relay.

Konsekuensi penting: **tidak ada interrupt UART3**, sehingga bit-bang SPI ke
ADE7880 tidak mungkin diinterupsi oleh trafik modem.

---

## Peta Pin

```
ADE7880                       Debug / lain
  PB12  CS                      PA9   USART1_TX  (115200 8N1)
  PB13  SCK                     PA10  USART1_RX
  PB14  MISO                    PA5   LED (kedip tiap pembacaan)
  PB15  MOSI
  PC6   PM1                     PA13  SWDIO
  PC7   PM0                     PA14  SWCLK
  PC8   RESET
  PA8   IRQ0
  PC9   IRQ1
```

---

## Cara Build

**STM32CubeIDE** — `File > Import > Existing Projects into Workspace`,
pilih folder ini, lalu Build (Ctrl+B).
Jangan klik *Generate Code* dari `.ioc`; project ini tidak memakainya.

**Command line** — `make` (butuh `arm-none-eabi-gcc` di PATH).
Hasil: `build/ADE7880_Test.hex`

Hasil build yang sudah jadi tersedia di `build/`.
Ukuran: 38.7 KB flash, 2.7 KB RAM.

---

## Perintah Serial (USART1, 115200, akhiri Enter)

Huruf besar/kecil bebas.

| Perintah | Fungsi |
|---|---|
| `HELP` | daftar perintah |
| `READ` | baca sekali sekarang |
| `RAW` | tampilkan nilai mentah terakhir |
| `CAL` | tampilkan konstanta kalibrasi |
| `STAT` | statistik: jumlah baca, jumlah nilai tidak wajar |
| `INT,<ms>` | ubah interval baca, mis. `INT,5000` |
| `POW,ON` / `POW,OFF` | hidupkan/matikan `getDataPOW` (ruang **0xE5xx**) |
| `PFF,ON` / `PFF,OFF` | hidupkan/matikan `getDataPFf` (ruang **0xE9xx**) |
| `DLY,<setup>,<hold>,<inter>` | ubah timing bit-bang saat runtime |
| `RECFG` | jalankan ulang `ADE7880_Config()` |
| `RESET` | restart MCU |

**Default saat boot: `getDataPOW` MATI** (meniru KWH_EC25_V1c yang
pembacaannya sudah tepat).

---

## Contoh Keluaran

```
===== #12  t=61234 ms  durasi=63 ms =====
-- RAW (langsung dari SPI, sebelum konversi) --
  AVRMS=0x0026D3B6 (  2544566)   AIRMS=0x00007345 (    29509)
  BVRMS=0x0026D400 (  2544640)   BIRMS=0x00007350 (    29520)
  CVRMS=0x0026D3F0 (  2544624)   CIRMS=0x00007348 (    29512)
  NIRMS=0x00000123 (      291)
  APF=0x7A3D BPF=0x7A40 CPF=0x7A3E | APER=0x1400 BPER=0x1400 CPER=0x1400
-- HASIL KALIBRASI --
  R: V=  231.05  I=   0.529  PF=0.955  F= 50.00  P=    0.00
  S: V=  231.07  I=   0.529  PF=0.956  F= 50.00  P=    0.00
  T: V=  231.09  I=   0.529  PF=0.955  F= 50.00  P=    0.00
  N: I=   0.002
```

Nilai **RAW** dicetak apa adanya sebelum konversi dan kalibrasi. Ini yang
paling penting: dari situ ketahuan apakah nilai rusak sudah keluar dari chip,
atau baru muncul saat konversi di software.

Sebagai pembanding, nilai mentah normal untuk jaringan 220–235 V adalah
sekitar **2.540.000–2.580.000** (0x26xxxx). Nilai rusak yang tercatat di
lapangan sebelumnya adalah **≈ 6.808.000** (0x67E0xx) — jadi mudah dikenali.

---

## Rencana Pengujian yang Disarankan

Kerjakan berurutan, catat hasil tiap tahap.

**Tahap 1 — baseline (getPOW mati).**
Flash, biarkan jalan 10–15 menit. Perhatikan apakah nilai RAW ketiga fasa
stabil dan wajar. Lalu ketik `STAT` untuk melihat berapa kali nilai di luar
batas wajar terjadi.

**Tahap 2 — konsistensi antar boot.**
Ketik `RESET` sebanyak 5–10 kali. Setiap boot, periksa apakah ketiga fasa
membaca nilai yang sama. Ini menguji apakah gejala "tiap boot beda" masih ada
tanpa modem.

**Tahap 3 — uji hipotesis ruang 0xE5xx.**
Ketik `POW,ON`. Amati 5–10 menit.
- Kalau nilai mulai kacau → **hipotesis terbukti**, `getDataPOW` memang
  pemicunya, dan menonaktifkannya di project utama adalah solusi yang benar.
- Kalau tetap stabil → hipotesis gugur, penyebabnya interaksi dengan modem.

Ketik `POW,OFF` lagi untuk kembali ke kondisi aman.

**Tahap 4 — uji sensitivitas timing.**
Coba timing lebih longgar: `DLY,400,100,800`
Lalu coba timing asli yang bermasalah: `DLY,0,0,0`
Kalau `DLY,0,0,0` membuat pembacaan rusak, berarti chip memang sensitif
terhadap jeda antar transaksi.

---

## Variabel untuk Live Expression

Nilai mentah:
```
ade_raw_AVRMS  ade_raw_BVRMS  ade_raw_CVRMS
ade_raw_AIRMS  ade_raw_BIRMS  ade_raw_CIRMS  ade_raw_NIRMS
ade_raw_APF    ade_raw_BPF    ade_raw_CPF
ade_raw_APER   ade_raw_BPER   ade_raw_CPER
ade_raw_AWATT  ade_raw_BWATT  ade_raw_CWATT
```

Saklar dan timing (bisa **diubah langsung** saat debug tanpa flash ulang):
```
ade_en_getData      ade_en_getPFf      ade_en_getPOW
ade_dly_cs_setup    ade_dly_cs_hold    ade_dly_interframe
```

Statistik:
```
read_count   bad_V   bad_I   bad_F   read_interval_ms
```

---

## Catatan

Kalau di STM32CubeIDE angka desimal tercetak sebagai `?` atau kosong, aktifkan
dukungan float pada printf: `Project > Properties > C/C++ Build > Settings >
MCU/MPU Settings`, centang **Use float with printf from newlib-nano**.
(Pada build lewat `make`, hal ini sudah ditangani flag `-u _printf_float`.)
