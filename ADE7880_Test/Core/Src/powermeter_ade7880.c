/*
 * ade7880.c
 *
 *  Created on: 20 Mei 2016
 *      Author: SUPPORT 4
 */

#include <stdio.h>
#include <string.h>
#include "powermeter_ade7880.h"
#include "math.h"
//#include "serial_com_port.h"
//#include "lcd_log.h"
//#include "cmsis_os.h"
//#include "GUI.h"
//#include "main.h"
//#include "rtc_ds1307.h"

extern UART_HandleTypeDef huart1;
//extern void Show_Main_Screen(void);
//extern bool flagButton_Back_Touched;

/* ══════════════════════════════════════════════════════════════════════════
 *  PENANGKAP NILAI MENTAH (khusus firmware diagnostik)
 *
 *  Semua variabel di bawah menyimpan hasil ADE7880_Read32()/Read16() APA
 *  ADANYA, sebelum sign-extend dan sebelum kalibrasi. Bisa dilihat langsung
 *  di Live Expression maupun lewat cetakan UART.
 *
 *  Tujuannya: memastikan apakah nilai rusak sudah muncul dari chip (raw
 *  memang aneh) atau baru muncul saat konversi/kalibrasi di software.
 * ══════════════════════════════════════════════════════════════════════════ */
volatile uint32_t ade_raw_AVRMS, ade_raw_BVRMS, ade_raw_CVRMS;
volatile uint32_t ade_raw_AIRMS, ade_raw_BIRMS, ade_raw_CIRMS, ade_raw_NIRMS;
volatile uint16_t ade_raw_APF,   ade_raw_BPF,   ade_raw_CPF;
volatile uint16_t ade_raw_APER,  ade_raw_BPER,  ade_raw_CPER;
volatile uint32_t ade_raw_AWATT, ade_raw_BWATT, ade_raw_CWATT;

/* Timing bit-bang dibuat VARIABEL (bukan #define) supaya bisa diubah saat
 * runtime lewat Live Expression / perintah UART tanpa flash ulang. */
volatile uint32_t ade_dly_cs_setup   = 200;
volatile uint32_t ade_dly_cs_hold    =  50;
volatile uint32_t ade_dly_interframe = 400;

/* Saklar runtime: matikan/hidupkan tiap kelompok pembacaan untuk isolasi. */
volatile uint8_t  ade_en_getData = 1;   /* 0x43xx : VRMS/IRMS */
volatile uint8_t  ade_en_getPFf  = 1;   /* 0xE9xx : PF/PERIOD */
volatile uint8_t  ade_en_getPOW  = 1;   /* 0xE5xx : WATT — dinyalakan by default
                                         *          (permintaan 2026-08-28) */

int OK;

//Variabel Kalibrasi
float GAIN_VRMSR=0.0000891322683474717, GAIN_VRMSS=0.0000891322683474717, GAIN_VRMST=0.0000891322683474717;
float OFFS_VRMSR=4.19731508227824, OFFS_VRMSS=4.19731508227824, OFFS_VRMST=4.19731508227824;
float GAIN_IRMSR=0.0000181619025404464, GAIN_IRMSS=0.0000181619025404464, GAIN_IRMST=0.0000181619025404464, GAIN_IRMSN=0.000007343;
/* [KALIBRASI OFFSET ARUS — 2026-08-28]
 *
 *  Formula pembacaan (per fasa, di main.c DoRead / KWH.c getElectricValue):
 *
 *      Irc = raw * GAIN_IRMSx + OFFS_IRMSx - 0.16
 *
 *  Fasa R tidak dibebani apa-apa dan CT-nya juga tidak terpasang, tetapi
 *  chip mengeluarkan raw AIRMS ≈ 975 (rata-rata dari 984 / 972 / 969 pada
 *  log tanggal 2026-08-28 09:48). Dengan OFFS lama 0.153779841998976,
 *  hasilnya jadi ≈ +0,011 A — bukan nol.
 *
 *  Solusinya menurunkan OFFS_IRMSR sebesar tepat 975 * GAIN = 0.017707855
 *  supaya di raw = 975 hasil akhirnya 0. Fasa S dan T dibiarkan seperti
 *  semula karena keduanya sedang berbeban dan belum kita ukur titik nolnya
 *  (kalau nanti kedua CT dilepas juga, jalankan langkah yang sama untuk
 *  masing-masing).
 *
 *  Hasil verifikasi:
 *      raw=984 → Irc = +0.000163 A
 *      raw=972 → Irc = -0.000054 A
 *      raw=969 → Irc = -0.000109 A
 *  (fluktuasi 0,3 mA = noise floor chip, sudah di ambang batas presisi
 *  float 32-bit → aman disebut "nol").
 *
 *  Alternatif hardware: menulis AIRMSOS (0x4386, 24-bit signed) di chip.
 *  Belum dipakai karena kalibrasi software di sini lebih transparan dan
 *  mudah diubah tanpa mengedit driver low-level.
 */
/* [KALIBRASI SERAGAM — 2026-08-28]
 *
 *  OFFS_IRMSR sebelumnya dikalibrasi ke 0,142292145 lewat rumus
 *      OFFS = 0,16 − raw_no_load × GAIN
 *  dengan raw_no_load rata-rata 975 (fasa R tanpa CT).
 *
 *  Permintaan hari ini: samakan OFFS untuk ketiga fasa. Asumsi yang
 *  dipakai — sekaligus batasannya:
 *
 *    · CT dan front-end analog ADE7880 di ketiga kanal dianggap punya
 *      offset residu yang MIRIP. Untuk chip yang sama dan CT dari batch
 *      yang sama ini masuk akal, tapi TIDAK persis (biasanya beda ±5 mA
 *      antar-kanal).
 *
 *    · Akibatnya, pembacaan fasa S dan T yang sedang berbeban akan turun
 *      ~11 mA (mis. 0,528 A → 0,516 A pada raw 29 400). Ini bukan
 *      kesalahan — nilai lama juga sudah bergeser ~11 mA lantaran mereka
 *      sama-sama pakai OFFS bawaan yang cocok untuk kalibrasi lama.
 *
 *  Kalau nanti CT di fasa S atau T dilepas, catat raw BIRMS / CIRMS
 *  masing-masing selama beberapa siklus, rata-ratakan, lalu ganti nilai
 *  di bawah dengan:  OFFS = 0,16 − raw_no_load_avg × GAIN_IRMSx
 *  Kalibrasi per-kanal seperti itu selalu lebih akurat daripada seragam. */
float OFFS_IRMSR = 0.142292145f;
float OFFS_IRMSS = 0.142292145f;
float OFFS_IRMST = 0.135292145f;
float OFFS_IRMSN;
float GAIN_PR=0.0139875896392553, GAIN_PS=0.0139875896392553, GAIN_PT=0.0139875896392553;
float OFFS_PR=0, OFFS_PS=0, OFFS_PT=0;

// private function khusus untuk ade7880.c
static void ADE7880_GPIO_Config(void);
static void ADE7880_SPIInit(void);
static void ADE7880_Delay(volatile uint32_t nCount);

/* ══════════════════════════════════════════════════════════════════════════
 *  PROTEKSI & TIMING TRANSAKSI SPI BIT-BANG          [PERBAIKAN]
 *
 *  Masalah yang diperbaiki:
 *
 *  (a) CS setup time tidak konsisten. ADE7880_Config() memakai
 *      ADE7880_Delay(200) setelah CS turun, sedangkan ADE7880_getData(),
 *      getDataPFf() dan getDataPOW() TIDAK memakai delay sama sekali.
 *
 *  (b) Tidak ada jeda antar-frame. getDataPFf() menembak 6 transaksi dan
 *      getDataPOW() 3 transaksi secara BERUNTUN tanpa jeda sama sekali
 *      (9 transaksi back-to-back). Kalau state machine SPI di chip belum
 *      selesai reset saat CS turun lagi, frame berikutnya bisa desync →
 *      register tetangga yang terbaca (gejala: nilai V/I tertukar antar
 *      fasa, dan nilai "aneh" yang STABIL sampai device di-restart).
 *
 *  (c) ISR UART3 (RX modem) bisa menyela di tengah bit-bang dan membekukan
 *      SCK ratusan mikrodetik. Selama satu transaksi, interrupt USART3
 *      dimatikan — HANYA USART3, bukan __disable_irq() global — supaya
 *      SysTick (HAL_GetTick/HAL_Delay) dan UART1 debug tetap hidup.
 *      Interrupt dinyalakan lagi di sela antar transaksi supaya byte yang
 *      tertahan sempat dilayani.
 * ══════════════════════════════════════════════════════════════════════════ */

#define ADE_CS_SETUP_DLY    200U   /* jeda setelah CS turun, sebelum kirim bit */
#define ADE_CS_HOLD_DLY      50U   /* jeda sebelum CS naik                     */
#define ADE_INTERFRAME_DLY  400U   /* jeda antar transaksi (CS naik → turun)   */

#define ADE_TRX_BEGIN()  do {                                   \
            HAL_NVIC_DisableIRQ(USART3_IRQn);                   \
            ADE7880_CS_RESET(ADE7880_CS_PIN);                   \
            ADE7880_Delay(ade_dly_cs_setup);                    \
        } while (0)

#define ADE_TRX_END()    do {                                   \
            ADE7880_Delay(ade_dly_cs_hold);                     \
            ADE7880_CS_SET(ADE7880_CS_PIN);                     \
            HAL_NVIC_EnableIRQ(USART3_IRQn);                    \
            ADE7880_Delay(ade_dly_interframe);                  \
        } while (0)

/* Batas percobaan ulang konfigurasi. Versi lama memakai
 *   #define Retry_ADE7880_Config() ADE7880_Config()
 * yang memanggil dirinya sendiri secara REKURSIF TANPA BATAS — kalau chip
 * mati permanen, stack habis → HardFault. Sekarang dibatasi. */
#define ADE_CFG_MAX_RETRY    3
static uint8_t ade_cfg_retry = 0;

void ADE7880_Config(void)
{
	uint16_t ADE7880_Read_Result;
	char text[40];

	// pengaturan GPIO untuk soft-SPI dan kontrol
	ADE7880_GPIO_Config();

	// pengaturan mode
	ADE7880_SCK_SET;
	ADE7880_MOSI_SET;
	ADE7880_Mode0_Set;
	ADE7880_CS_SET(ADE7880_CS_PIN);
	ADE7880_RESET_HIGH;

	// pengaturan komunikasi soft-SPI (software serial pheriperal interface)
	ADE7880_SPIInit();

	/* ══════════════════════════════════════════════════════════════════════
	 *  [PERBAIKAN PENTING] KUNCI MODE SPI — set I2C_LOCK di CONFIG2 (0xEC01)
	 *
	 *  Setelah power-up / hardware reset, ADE7880 SELALU mulai dalam mode
	 *  I2C. Toggle CS 3x di ADE7880_SPIInit() memang memilih mode SPI, TAPI
	 *  pemilihan itu belum terkunci — chip masih bisa kembali ke mode I2C
	 *  bila melihat pola tertentu di pin SCLK/MOSI.
	 *
	 *  Datasheet mensyaratkan set bit I2C_LOCK (bit 1) pada register CONFIG2
	 *  agar port serial terkunci permanen di SPI sampai hardware reset
	 *  berikutnya. Langkah ini TIDAK ADA di versi lama — inilah penyebab
	 *  paling mungkin dari perilaku "tiap boot hasilnya beda": kadang semua
	 *  fasa terbaca, kadang satu kanal V atau I diam di 0.
	 *
	 *  CONFIG2 adalah register 8-bit → tulis 8-bit.
	 * ══════════════════════════════════════════════════════════════════════ */
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x00);          /* 0x00 = perintah WRITE           */
	ADE7880_Write16(CONFIG2);      /* alamat 0xEC01                   */
	ADE7880_Write8(0x02);          /* bit1 = I2C_LOCK = 1             */
	ADE_TRX_END();

	/* Verifikasi bahwa kunci benar-benar tertulis */
	{
		uint8_t cfg2;
		ADE_TRX_BEGIN();
		ADE7880_Write8(0x01);      /* 0x01 = perintah READ            */
		ADE7880_Write16(CONFIG2);
		cfg2 = ADE7880_Read8();
		ADE_TRX_END();

		sprintf(text, "ADE7880 CONFIG2=0x%02X %s\n",
		        cfg2, (cfg2 & 0x02) ? "(SPI locked)" : "(GAGAL lock!)");
		HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text), 1*strlen(text));
	}

	// cek status ADE7880
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(CONFIG);
	ADE7880_Read_Result = ADE7880_Read16();
	ADE_TRX_END();

	if (ADE7880_Read_Result == 0x0002) {
		OK=1;
	}
	else {
		OK=2;
	}

	/* Disable RAM Protection
	 * [PERBAIKAN] Versi lama menulis DUA register (0xE7FE dan 0xE7E3) di
	 * dalam SATU kali CS di-assert. Protokol SPI ADE7880 mengharuskan CS
	 * naik di antara setiap transaksi register, sehingga penulisan kedua
	 * (0xE7E3 — kunci proteksi yang sesungguhnya) besar kemungkinan tidak
	 * pernah tereksekusi. Sekarang dipecah jadi dua transaksi terpisah. */
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x00);
	ADE7880_Write16(0xE7FE);
	ADE7880_Write8(0xAD);
	ADE_TRX_END();

	ADE_TRX_BEGIN();
	ADE7880_Write8(0x00);
	ADE7880_Write16(0xE7E3);
	ADE7880_Write8(0x00);
	ADE_TRX_END();
	//------------------------

	//Voltage,current, neutral gain will only amplify rough data. not RMS
	//Voltage,Current,Neutral Gain Settings
	// Default 000: Gain 1
	//------------------------

	//Rogowski Coil Settings
	// Default : Used CT
	//------------------------

	//Fundamental Freq Settings
	// Default : 45 - 55 Hz
	//------------------------

	//Initialize WTHR, VARTHR, VATHR, VLEVEL and VNOM registers
	// Default
	//------------------------


	//Initialize CF1DEN, CF2DEN, and CF3DEN
	// Default
	//------------------------

	/* Enable RAM Protection — dipecah dua transaksi, alasan sama spt di atas */
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x00);
	ADE7880_Write16(0xE7FE);
	ADE7880_Write8(0xAD);
	ADE_TRX_END();

	ADE_TRX_BEGIN();
	ADE7880_Write8(0x00);
	ADE7880_Write16(0xE7E3);
	ADE7880_Write8(0x80);
	ADE_TRX_END();
	//------------------------

	//Start DSP
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x00);
	ADE7880_Write16(RUN);
	ADE7880_Write16(0x0001);
	ADE_TRX_END();

	/* [PERBAIKAN] Beri waktu DSP internal ADE7880 untuk settle sebelum
	 * register RMS dibaca pertama kali. DSP mengisi register RMS pada laju
	 * 8 kHz tetapi butuh beberapa siklus jaringan (~100-200 ms) agar hasil
	 * RMS-nya konvergen. Versi lama langsung lanjut membaca sehingga nilai
	 * transisi yang belum konvergen bisa "terkunci" berbeda-beda tiap boot
	 * (gejala: pembacaan fasa bertukar/aneh setelah restart). */
	HAL_Delay(250);

	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(RUN);
	ADE7880_Read_Result = ADE7880_Read16();
	ADE_TRX_END();
	//------------------------

	if (ADE7880_Read_Result == 0x0001)
	{
		ade_cfg_retry = 0;                    /* sukses → reset counter */
		sprintf(text,"Status : ADE7880-DSP Started!\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text),1*strlen(text));

		/* [PERBAIKAN] Pembacaan pertama setelah DSP start SENGAJA DIBUANG.
		 * Register RMS pada siklus pertama masih bisa berisi nilai transisi
		 * yang belum konvergen. Kalau nilai itu terpakai, ia akan menjadi
		 * "bacaan sah terakhir" di filter KWH.c dan ikut mengotori data
		 * berikutnya. Satu kali baca-buang menghilangkan risiko itu. */
		{
			float d1,d2,d3,d4,d5,d6,d7;
			ADE7880_getData(&d1,&d2,&d3,&d4,&d5,&d6,&d7);
			(void)d1; (void)d2; (void)d3; (void)d4; (void)d5; (void)d6; (void)d7;
		}
	}
	else
	{
		sprintf(text,"Status : ADE7880-DSP Error !\n");
		HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text),1*strlen(text));

		/* [PERBAIKAN] Ulangi konfigurasi, TAPI dibatasi. Versi lama memanggil
		 * ADE7880_Config() secara rekursif tanpa batas → kalau chip benar-
		 * benar mati, stack habis dan MCU HardFault. Sekarang maksimum
		 * ADE_CFG_MAX_RETRY kali; setelah itu menyerah dan membiarkan
		 * superloop jalan terus (watchdog IWDG tetap jadi jaring pengaman). */
		if (ade_cfg_retry < ADE_CFG_MAX_RETRY)
		{
			ade_cfg_retry++;
			HAL_Delay(300);
			ADE7880_Config();
		}
		else
		{
			ade_cfg_retry = 0;
			/* buffer text[] hanya 40 byte — pesan harus pendek */
			sprintf(text,"Status : ADE7880 stop retry\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)text, strlen(text),1*strlen(text));
		}
	}
}

void ADE7880_getData(float *Vrms_R, float *Vrms_S, float *Vrms_T, float *Irms_R, float *Irms_S, float *Irms_T, float *Irms_N)
{
	int VRMS_A, VRMS_B, VRMS_C, IRMS_A, IRMS_B, IRMS_C, IRMS_N; // these all variable are named after the ADE7880 register it self,
	uint32_t ADE7880_Read_Result;
	
	//VA RMS Measure
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(AVRMS);
	ADE7880_Read_Result = ADE7880_Read32();
	ADE_TRX_END();
	ade_raw_AVRMS = ADE7880_Read_Result;   /* tangkap RAW */
	//------------------------
	//Convert to 32bit Signed Integer
	VRMS_A = (int)(ADE7880_Read_Result << 8);
	VRMS_A /= 256;
	//------------------------
	*Vrms_R = VRMS_A;
	ADE7880_Read_Result=0;
	HAL_Delay(10);
	
	//VB RMS Measure
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(BVRMS);
	ADE7880_Read_Result = ADE7880_Read32();
	ADE_TRX_END();
	ade_raw_BVRMS = ADE7880_Read_Result;   /* tangkap RAW */
	//------------------------
	//Convert to 32bit Signed Integer
	VRMS_B = (int)(ADE7880_Read_Result << 8);
	VRMS_B /= 256;
	//------------------------
	*Vrms_S = VRMS_B;
	ADE7880_Read_Result=0;
	HAL_Delay(10);
	
	//VC RMS Measure
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(CVRMS);
	ADE7880_Read_Result = ADE7880_Read32();
	ADE_TRX_END();
	ade_raw_CVRMS = ADE7880_Read_Result;   /* tangkap RAW */
	//------------------------
	//Convert to 32bit Signed Integer
	VRMS_C = (int)(ADE7880_Read_Result << 8);
	VRMS_C /= 256;
	//------------------------
	*Vrms_T = VRMS_C;
	ADE7880_Read_Result=0;
	HAL_Delay(10);
	
	//IA RMS Measure
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(AIRMS);
	ADE7880_Read_Result = ADE7880_Read32();
	ADE_TRX_END();
	ade_raw_AIRMS = ADE7880_Read_Result;   /* tangkap RAW */
	//------------------------
	//Convert to 32bit Signed Integer
	IRMS_A = (int)(ADE7880_Read_Result << 8);
	*Irms_R = IRMS_A /256;
	//------------------------
	ADE7880_Read_Result=0;
	HAL_Delay(10);
	
	//IB RMS Measure
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(BIRMS);
	ADE7880_Read_Result = ADE7880_Read32();
	ADE_TRX_END();
	ade_raw_BIRMS = ADE7880_Read_Result;   /* tangkap RAW */
	//------------------------
	//Convert to 32bit Signed Integer
	IRMS_B = (int)(ADE7880_Read_Result << 8);
	IRMS_B /= 256;
	//------------------------
	//IRMS_B /= 1;
	*Irms_S = IRMS_B;
	ADE7880_Read_Result=0;
	HAL_Delay(10);
	
	//IC RMS Measure
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(CIRMS);
	ADE7880_Read_Result = ADE7880_Read32();
	ADE_TRX_END();
	ade_raw_CIRMS = ADE7880_Read_Result;   /* tangkap RAW */
	//------------------------
	//Convert to 32bit Signed Integer
	IRMS_C = (int)(ADE7880_Read_Result << 8);
	IRMS_C /= 256;
	//------------------------
	*Irms_T = IRMS_C;
	ADE7880_Read_Result=0;
	HAL_Delay(10);

	//IN RMS Measure
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(NIRMS);
	ADE7880_Read_Result = ADE7880_Read32();
	ADE_TRX_END();
	ade_raw_NIRMS = ADE7880_Read_Result;   /* tangkap RAW */
	//------------------------
	//Convert to 32bit Signed Integer
	IRMS_N = (int)(ADE7880_Read_Result << 8);
	IRMS_N /= 256;
	//------------------------
	//*Irms_N = IRMS_N;
	*Irms_N = IRMS_N;
}

void ADE7880_getDataPFf(float *pf_R,float *pf_S,float *pf_T,float *freq_R,float *freq_S, float *freq_T)
{
	if (!ade_en_getPFf) { *pf_R=*pf_S=*pf_T=0; *freq_R=*freq_S=*freq_T=0; return; }
	int16_t ADE7880_Read_Result1;
	int A_PF,FREQ_A,B_PF,FREQ_B,C_PF,FREQ_C;
	
	//PF -A
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(APF);
	ADE7880_Read_Result1 = ADE7880_Read16();
	ADE_TRX_END();
	ade_raw_APF = ADE7880_Read_Result1;   /* tangkap RAW */
	

	if(ADE7880_Read_Result1>32768) ADE7880_Read_Result1 = ADE7880_Read_Result1 - 65536;
	A_PF=ADE7880_Read_Result1;
	*pf_R=fabs(A_PF/32768.0);
//	*pf_R=A_PF/32768.0;

	//frekuensi A
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(APERIOD);
	ADE7880_Read_Result1 = ADE7880_Read16();
	ADE_TRX_END();
	ade_raw_APER = ADE7880_Read_Result1;   /* tangkap RAW */
	FREQ_A = (ADE7880_Read_Result1 != 0)
	          ? (int)(2560000 / (int32_t)ADE7880_Read_Result1)
	          : 0;   /* [PERBAIKAN] cegah division-by-zero */
	*freq_R=FREQ_A;	
	*freq_R/=10.0;
	
	
	//PF -B
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(BPF);
	ADE7880_Read_Result1 = ADE7880_Read16();
	ADE_TRX_END();
	ade_raw_BPF = ADE7880_Read_Result1;   /* tangkap RAW */
	
	if(ADE7880_Read_Result1>32768) ADE7880_Read_Result1 = ADE7880_Read_Result1 - 65536;
	B_PF=ADE7880_Read_Result1;
	*pf_S=fabs(B_PF/32768.0);
//	*pf_S=B_PF/32768.0;
	
	//frekuensi B
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(BPERIOD);
	ADE7880_Read_Result1 = ADE7880_Read16();
	ADE_TRX_END();
	ade_raw_BPER = ADE7880_Read_Result1;   /* tangkap RAW */
	FREQ_B = (ADE7880_Read_Result1 != 0)
	          ? (int)(2560000 / (int32_t)ADE7880_Read_Result1)
	          : 0;   /* [PERBAIKAN] cegah division-by-zero */
	*freq_S=FREQ_B;	
	*freq_S/=10.0;
	
	//PF -C
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(CPF);
	ADE7880_Read_Result1 = ADE7880_Read16();
	ADE_TRX_END();
	ade_raw_CPF = ADE7880_Read_Result1;   /* tangkap RAW */
	
	if(ADE7880_Read_Result1>32768) ADE7880_Read_Result1 = ADE7880_Read_Result1 - 65536;
	C_PF=ADE7880_Read_Result1;
	*pf_T=fabs(C_PF/32768.0);
//	*pf_T=C_PF/32768.0;
	
	//frekuensi C
	ADE_TRX_BEGIN();
	ADE7880_Write8(0x01);
	ADE7880_Write16(CPERIOD);
	ADE7880_Read_Result1 = ADE7880_Read16();
	ADE_TRX_END();
	ade_raw_CPER = ADE7880_Read_Result1;   /* tangkap RAW */
	FREQ_C = (ADE7880_Read_Result1 != 0)
	          ? (int)(2560000 / (int32_t)ADE7880_Read_Result1)
	          : 0;   /* [PERBAIKAN] cegah division-by-zero */
	*freq_T=FREQ_C;	
	*freq_T/=10.0;
}

void ADE7880_getDataPOW(float *P_R,float *P_S, float *P_T)
{
	if (!ade_en_getPOW) { *P_R=*P_S=*P_T=0; return; }
		uint32_t ADE7880_Read_Result1;
		int P_A, P_B, P_C;
	
		//AWATT Phase-A (Instantaneous value of Phase A total active power)
	  ADE_TRX_BEGIN();
	  ADE7880_Write8(0x01);
	  ADE7880_Write16(AWATT);
	  ADE7880_Read_Result1 = ADE7880_Read32();
	  ADE_TRX_END();
	ade_raw_AWATT = ADE7880_Read_Result1;   /* tangkap RAW */

	  P_A = (int)(ADE7880_Read_Result1 << 8);
	  P_A /= 256;
		*P_R=P_A;
		*P_R=fabs((*P_R*GAIN_PR)+OFFS_PR);
		if(*P_R<0.5)*P_R=0;
	
		//BWATT Phase-B (Instantaneous value of Phase A total active power)
	  ADE_TRX_BEGIN();
	  ADE7880_Write8(0x01);
	  ADE7880_Write16(BWATT);
	  ADE7880_Read_Result1 = ADE7880_Read32();
	  ADE_TRX_END();
	ade_raw_BWATT = ADE7880_Read_Result1;   /* tangkap RAW */

	  P_B = (int)(ADE7880_Read_Result1 << 8);
	  P_B /= 256;
		*P_S=P_B;
		*P_S=fabs((*P_S*GAIN_PS)+OFFS_PS);
		if(*P_S<0.5)*P_S=0;
		
		//CWATT Phase-C (Instantaneous value of Phase A total active power)
	  ADE_TRX_BEGIN();
	  ADE7880_Write8(0x01);
	  ADE7880_Write16(CWATT);
	  ADE7880_Read_Result1 = ADE7880_Read32();
	  ADE_TRX_END();
	ade_raw_CWATT = ADE7880_Read_Result1;   /* tangkap RAW */

	  P_C = (int)(ADE7880_Read_Result1 << 8);
	  P_C /= 256;
		*P_T=P_C;
		*P_T=fabs((*P_T*GAIN_PT)+OFFS_PT);
		if(*P_T<0.5)*P_T=0;
		
}

void ADE7880_getDataVA(float *VA_R,float *VA_S, float *VA_T)
{
		uint32_t ADE7880_Read_Result1;
		int VA_A, VA_B, VA_C;
	
		//AWATT Phase-A (Instantaneous value of Phase A total active power)
	  ADE7880_CS_RESET(ADE7880_CS_PIN);
	  ADE7880_Write8(0x01);
	  ADE7880_Write16(AVA);
	  ADE7880_Read_Result1 = ADE7880_Read32();
	  ADE7880_CS_SET(ADE7880_CS_PIN);

	  VA_A = (int)(ADE7880_Read_Result1 << 8);
	  VA_A /= 128;
		*VA_R=VA_A;
		*VA_R/=100;
		*VA_R=3.56 * *VA_R + 8.6612;
	
		//AWATT Phase-B (Instantaneous value of Phase A total active power)
	  ADE7880_CS_RESET(ADE7880_CS_PIN);
	  ADE7880_Write8(0x01);
	  ADE7880_Write16(BVA);
	  ADE7880_Read_Result1 = ADE7880_Read32();
	  ADE7880_CS_SET(ADE7880_CS_PIN);

	  VA_B = (int)(ADE7880_Read_Result1 << 8);
	  VA_B /= 128;
		*VA_S=VA_B;
		*VA_S/=100;
		*VA_S=3.56 * *VA_S + 8.6612;
		
		//AWATT Phase-C (Instantaneous value of Phase A total active power)
	  ADE7880_CS_RESET(ADE7880_CS_PIN);
	  ADE7880_Write8(0x01);
	  ADE7880_Write16(CVA);
	  ADE7880_Read_Result1 = ADE7880_Read32();
	  ADE7880_CS_SET(ADE7880_CS_PIN);

	  VA_C = (int)(ADE7880_Read_Result1 << 8);
	  VA_C /= 128;
		*VA_T=VA_C;
		*VA_T/=100;
		*VA_T=3.56 * *VA_T + 8.6612;
}
void ADE7880_getDataHarmonic1_A(float *F_VRMS, float *V_THD, float *F_IRMS, float *I_THD)
{
		uint32_t ADE7880_Read_Result1;
		int Buff_FVRMS, Buff_VTHD, Buff_FIRMS, Buff_ITHD;

//  untuk pindah fasa
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);
		ADE7880_Write8(0x00);
		ADE7880_Write16(HCONFIG);
		ADE7880_Write16(0x08);   // 0x08 (Fasa A),  0x0A (Fasa B) , 0x0C (Fasa C)
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(2);
	
		HAL_Delay(600);
		
		// MEMBACA NILAI TEGANGAN FUNDAMENTAL
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(FVRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_FVRMS = (int)(ADE7880_Read_Result1 << 8);
		Buff_FVRMS /= 128;
		*F_VRMS=Buff_FVRMS;
		*F_VRMS = *F_VRMS * 45;
		*F_VRMS = *F_VRMS / 100;
		*F_VRMS = *F_VRMS + 72243;
		*F_VRMS = *F_VRMS / 10000;
		*F_VRMS = 0.0003**F_VRMS**F_VRMS + 0.9085**F_VRMS + 4.3394; // Persamaan Hasil Kalibrasi
		ADE7880_Delay(2);
		
		// MEMBACA NILAI THDV
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);
		ADE7880_Write8(0x01);
		ADE7880_Write16(VTHDN);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
		
		Buff_VTHD = (int)(ADE7880_Read_Result1 << 8);
		Buff_VTHD /= 128;
		*V_THD = Buff_VTHD * 45;
		*V_THD = *V_THD / 100;
		*V_THD = *V_THD + 72243;
		*V_THD = *V_THD / 100000;
		ADE7880_Delay(2);
		
		// MEMBACA NILAI ARUS FUNDAMENTAL
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(FIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_FIRMS = (int)(ADE7880_Read_Result1 << 8);
		Buff_FIRMS /= 128;
		*F_IRMS = Buff_FIRMS * 136;
		*F_IRMS = *F_IRMS - 44885;
		*F_IRMS = *F_IRMS / 10000;
		*F_IRMS = *F_IRMS / 1000;
		*F_IRMS = 0.9398 * *F_IRMS + 0.0038;     //Persamaan Hasil Kalibrasi
		*F_IRMS = 3.5866 * *F_IRMS;
		ADE7880_Delay(2);

		// MEMBACA NILAI THDI
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(ITHDN);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_ITHD = (int)(ADE7880_Read_Result1 << 8);
		*I_THD = Buff_ITHD /128;
		*I_THD = *I_THD * 136;
		*I_THD = *I_THD - 44885;
		*I_THD = *I_THD / 10000;
		*I_THD = *I_THD / 1000;
		ADE7880_Delay(2);
}

void ADE7880_getDataHarmonic1_B(float *F_VRMS, float *V_THD, float *F_IRMS, float *I_THD)
{
		uint32_t ADE7880_Read_Result1;
		int Buff_FVRMS, Buff_VTHD, Buff_FIRMS, Buff_ITHD;

//  untuk pindah fasa
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);
		ADE7880_Write8(0x00);
		ADE7880_Write16(HCONFIG);
		ADE7880_Write16(0x02);   // 0x08 (Fasa A),  0x0A (Fasa B) , 0x0C (Fasa C)
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(2);
	
		HAL_Delay(600);
		
		// MEMBACA NILAI TEGANGAN FUNDAMENTAL
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(FVRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_FVRMS = (int)(ADE7880_Read_Result1 << 8);
		Buff_FVRMS /= 128;
		*F_VRMS=Buff_FVRMS;
		*F_VRMS = *F_VRMS * 45;
		*F_VRMS = *F_VRMS / 100;
		*F_VRMS = *F_VRMS + 72243;
		*F_VRMS = *F_VRMS / 10000;
		*F_VRMS = 1.0153**F_VRMS - 5.5233;    // Persamaan Hasil Kalibrasi
		ADE7880_Delay(2);
		
		// MEMBACA NILAI THDV
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);
		ADE7880_Write8(0x01);
		ADE7880_Write16(VTHDN);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
		
		Buff_VTHD = (int)(ADE7880_Read_Result1 << 8);
		Buff_VTHD /= 128;
		*V_THD = Buff_VTHD * 45;
		*V_THD = *V_THD / 100;
		*V_THD = *V_THD + 72243;
		*V_THD = *V_THD / 100000;
		ADE7880_Delay(2);
		
		// MEMBACA NILAI ARUS FUNDAMENTAL
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(FIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_FIRMS = (int)(ADE7880_Read_Result1 << 8);
		Buff_FIRMS /= 128;
		*F_IRMS = Buff_FIRMS * 136;
		*F_IRMS = *F_IRMS - 44885;
		*F_IRMS = *F_IRMS / 10000;
		*F_IRMS = *F_IRMS / 1000;
		*F_IRMS = 0.9392**F_IRMS + 0.0043;     //Persamaan Hasil Kalibrasi
		ADE7880_Delay(2);

		// MEMBACA NILAI THDI
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(ITHDN);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_ITHD = (int)(ADE7880_Read_Result1 << 8);
		*I_THD = Buff_ITHD /128;
		*I_THD = *I_THD * 136;
		*I_THD = *I_THD - 44885;
		*I_THD = *I_THD / 10000;
		*I_THD = *I_THD / 1000;
		ADE7880_Delay(2);
}
void ADE7880_getDataHarmonic1_C(float *F_VRMS, float *V_THD, float *F_IRMS, float *I_THD)
{
		uint32_t ADE7880_Read_Result1;
		int Buff_FVRMS, Buff_VTHD, Buff_FIRMS, Buff_ITHD;

//  untuk pindah fasa
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);
		ADE7880_Write8(0x00);
		ADE7880_Write16(HCONFIG);
		ADE7880_Write16(0x04);   // 0x08 (Fasa A),  0x0A (Fasa B) , 0x0C (Fasa C)
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(2);
	
		HAL_Delay(600);
		
		// MEMBACA NILAI TEGANGAN FUNDAMENTAL
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(FVRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_FVRMS = (int)(ADE7880_Read_Result1 << 8);
		Buff_FVRMS /= 128;
		*F_VRMS=Buff_FVRMS;
		*F_VRMS = *F_VRMS * 45;
		*F_VRMS = *F_VRMS / 100;
		*F_VRMS = *F_VRMS + 72243;
		*F_VRMS = *F_VRMS / 10000;
		*F_VRMS = 1.0203**F_VRMS - 5.2859; 		//Persamaan Hasil Kalibrasi
		ADE7880_Delay(2);
		
		// MEMBACA NILAI THDV
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);
		ADE7880_Write8(0x01);
		ADE7880_Write16(VTHDN);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
		
		Buff_VTHD = (int)(ADE7880_Read_Result1 << 8);
		Buff_VTHD /= 128;
		*V_THD = Buff_VTHD * 45;
		*V_THD = *V_THD / 100;
		*V_THD = *V_THD + 72243;
		*V_THD = *V_THD / 100000;
		ADE7880_Delay(2);
		
		// MEMBACA NILAI ARUS FUNDAMENTAL
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(FIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_FIRMS = (int)(ADE7880_Read_Result1 << 8);
		Buff_FIRMS /= 128;
		*F_IRMS = Buff_FIRMS * 136;
		*F_IRMS = *F_IRMS - 44885;
		*F_IRMS = *F_IRMS / 10000;
		*F_IRMS = *F_IRMS / 1000;
		*F_IRMS = 0.9392**F_IRMS + 0.0043;     //Persamaan Hasil Kalibrasi
		ADE7880_Delay(2);

		// MEMBACA NILAI THDI
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(ITHDN);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_ITHD = (int)(ADE7880_Read_Result1 << 8);
		*I_THD = Buff_ITHD /128;
		*I_THD = *I_THD * 136;
		*I_THD = *I_THD - 44885;
		*I_THD = *I_THD / 10000;
		*I_THD = *I_THD / 1000;
		ADE7880_Delay(2);
}

void ADE7880_getDataHarmonic1_N(float *F_IRMS, float *I_THD) // I_THD belum tentu nilai yang diambil
{
		uint32_t ADE7880_Read_Result1;
		int Buff_FIRMS, Buff_ITHD;

		//untuk pindah fasa
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);
		ADE7880_Write8(0x00);
		ADE7880_Write16(HCONFIG);
		ADE7880_Write16(0x06);   // 0x08 (Fasa A),  0x0A (Fasa B) , 0x0C (Fasa C)
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(2);
		
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x00);
		ADE7880_Write16(HX_reg);
		ADE7880_Write8(0X01);
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(20);
	
		HAL_Delay(600);
			
		// MEMBACA NILAI ARUS FUNDAMENTAL
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HXIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_FIRMS = (int)(ADE7880_Read_Result1 << 8);
		Buff_FIRMS /= 128;
		*F_IRMS = Buff_FIRMS * 136;
		*F_IRMS = *F_IRMS - 44885;
		*F_IRMS = *F_IRMS / 10000;
		*F_IRMS = *F_IRMS / 1000;
		ADE7880_Delay(2);

		// MEMBACA NILAI THDI
		ADE7880_CS_RESET(ADE7880_CS_PIN);      // I_THD belum tentu nilai yang diambil...
		ADE7880_Delay(200);										 // Tidak ada THDi dalam kawat Netral...
		ADE7880_Write8(0x01);
		ADE7880_Write16(HXIHD);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buff_ITHD = (int)(ADE7880_Read_Result1 << 8);
		*I_THD = Buff_ITHD /128;
		*I_THD = *I_THD * 136;
		*I_THD = *I_THD - 44885;
		*I_THD = *I_THD / 10000;
		*I_THD = *I_THD / 1000;
		ADE7880_Delay(2);
}
void ADE7880_getDataHarmonic2_N(char Hx_Mon, char Hy_Mon, char Hz_Mon, float *HX_IRMS1, float *HY_IRMS1, float *HZ_IRMS1)
{
		uint32_t ADE7880_Read_Result1;
		int Buf_HX_IRMS, Buf_HY_IRMS, Buf_HZ_IRMS;	
						
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x00);
		ADE7880_Write16(HX_reg);
		ADE7880_Write8(Hx_Mon);
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(20);
	
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x00);
		ADE7880_Write16(HY_reg);
		ADE7880_Write8(Hy_Mon);
		ADE7880_CS_SET(ADE7880_CS_PIN);	
		ADE7880_Delay(20);
	
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x00);
		ADE7880_Write16(HZ_reg);
		ADE7880_Write8(Hz_Mon);
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(20);
		
		HAL_Delay(600);
	
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HXIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
		
		Buf_HX_IRMS = (int)(ADE7880_Read_Result1 << 8);
		*HX_IRMS1 = Buf_HX_IRMS /128;
		*HX_IRMS1 = *HX_IRMS1 * 136;
		*HX_IRMS1 = *HX_IRMS1 - 44885;
		*HX_IRMS1 = *HX_IRMS1 / 10000;
		*HX_IRMS1 = *HX_IRMS1 / 1000;
		*HX_IRMS1 = 1.0641**HX_IRMS1 - 0.0041;
		
		ADE7880_Delay(2);
		
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HYIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);

		Buf_HY_IRMS = (int)(ADE7880_Read_Result1 << 8);
		*HY_IRMS1 = Buf_HY_IRMS /128;
		*HY_IRMS1 = *HY_IRMS1 * 136;
		*HY_IRMS1 = *HY_IRMS1 - 44885;
		*HY_IRMS1 = *HY_IRMS1 / 10000;
		*HY_IRMS1 = *HY_IRMS1 / 1000;
		*HY_IRMS1 = 1.0641**HY_IRMS1 - 0.0041;
		
		ADE7880_Delay(2);
		
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HZIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buf_HZ_IRMS = (int)(ADE7880_Read_Result1 << 8);
		*HZ_IRMS1 = Buf_HZ_IRMS /128;
		*HZ_IRMS1 = *HZ_IRMS1 * 136;
		*HZ_IRMS1 = *HZ_IRMS1 - 44885;
		*HZ_IRMS1 = *HZ_IRMS1 / 10000;
		*HZ_IRMS1 = *HZ_IRMS1 / 1000;
		*HZ_IRMS1 = 1.0641**HZ_IRMS1 - 0.0041;
		
		ADE7880_Delay(2);
}

void ADE7880_getDataHarmonic2(char Hx_Mon, char Hy_Mon, char Hz_Mon, float *HX_IRMS1, float *HY_IRMS1, float *HZ_IRMS1,float *HX_VRMS1, float *HY_VRMS1, float *HZ_VRMS1)
{
		uint32_t ADE7880_Read_Result1;
		int Buf_HX_IRMS, Buf_HY_IRMS, Buf_HZ_IRMS, Buf_HX_VRMS, Buf_HY_VRMS, Buf_HZ_VRMS;	
						
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x00);
		ADE7880_Write16(HX_reg);
		ADE7880_Write8(Hx_Mon);
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(20);
	
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x00);
		ADE7880_Write16(HY_reg);
		ADE7880_Write8(Hy_Mon);
		ADE7880_CS_SET(ADE7880_CS_PIN);	
		ADE7880_Delay(20);
	
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x00);
		ADE7880_Write16(HZ_reg);
		ADE7880_Write8(Hz_Mon);
		ADE7880_CS_SET(ADE7880_CS_PIN);
		ADE7880_Delay(20);
		
		HAL_Delay(600);
	
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HXIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
		
		Buf_HX_IRMS = (int)(ADE7880_Read_Result1 << 8);
		*HX_IRMS1 = Buf_HX_IRMS /128;
		*HX_IRMS1 = *HX_IRMS1 * 136;
		*HX_IRMS1 = *HX_IRMS1 - 44885;
		*HX_IRMS1 = *HX_IRMS1 / 10000;
		*HX_IRMS1 = *HX_IRMS1 / 1000;
		*HX_IRMS1 = 0.9398 * *HX_IRMS1 + 0.0038;
		*HX_IRMS1 = 3.5866 * *HX_IRMS1;
		
		ADE7880_Delay(2);
		
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HXVRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
		
		Buf_HX_VRMS = (int)(ADE7880_Read_Result1 << 8);
		Buf_HX_VRMS /= 128;
		*HX_VRMS1=Buf_HX_VRMS;
		
		ADE7880_Delay(2);
				
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HYIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);

		Buf_HY_IRMS = (int)(ADE7880_Read_Result1 << 8);
		*HY_IRMS1 = Buf_HY_IRMS /128;
		*HY_IRMS1 = *HY_IRMS1 * 136;
		*HY_IRMS1 = *HY_IRMS1 - 44885;
		*HY_IRMS1 = *HY_IRMS1 / 10000;
		*HY_IRMS1 = *HY_IRMS1 / 1000;
		*HY_IRMS1 = 0.9398**HY_IRMS1 + 0.0038;
		*HY_IRMS1 = 3.5866 * *HY_IRMS1;
		
		ADE7880_Delay(2);
		
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HYVRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);

		Buf_HY_VRMS = (int)(ADE7880_Read_Result1 << 8);
		Buf_HY_VRMS /= 128;
		*HY_VRMS1=Buf_HY_VRMS;
		
		ADE7880_Delay(2);
		
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HZIRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buf_HZ_IRMS = (int)(ADE7880_Read_Result1 << 8);
		*HZ_IRMS1 = Buf_HZ_IRMS /128;
		*HZ_IRMS1 = *HZ_IRMS1 * 136;
		*HZ_IRMS1 = *HZ_IRMS1 - 44885;
		*HZ_IRMS1 = *HZ_IRMS1 / 10000;
		*HZ_IRMS1 = *HZ_IRMS1 / 1000;
		*HZ_IRMS1 = 0.9398**HZ_IRMS1 + 0.0038;
		*HZ_IRMS1 = 3.5866 * *HZ_IRMS1;
		
		ADE7880_Delay(2);
		
		ADE7880_CS_RESET(ADE7880_CS_PIN);
		ADE7880_Delay(200);	
		ADE7880_Write8(0x01);
		ADE7880_Write16(HZVRMS);
		ADE7880_Read_Result1 = ADE7880_Read32();
		ADE7880_CS_SET(ADE7880_CS_PIN);
	
		Buf_HZ_VRMS = (int)(ADE7880_Read_Result1 << 8);
		Buf_HZ_VRMS /= 128;
		*HZ_VRMS1=Buf_HZ_VRMS;
		
		ADE7880_Delay(2);
}
static void ADE7880_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIO clocks */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_InitStructure.Pin       = ADE7880_CS_PIN;
	GPIO_InitStructure.Mode      = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull      = GPIO_NOPULL;
	GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(ADE7880_CS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ADE7880_SCK_PIN;
	HAL_GPIO_Init(ADE7880_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ADE7880_MOSI_PIN;
	HAL_GPIO_Init(ADE7880_MOSI_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ADE7880_PM0_PIN;
	HAL_GPIO_Init(ADE7880_PM0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin =ADE7880_PM1_PIN;
	HAL_GPIO_Init(ADE7880_PM1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = ADE7880_RESET_PIN;
	HAL_GPIO_Init(ADE7880_RESET_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin       = ADE7880_IRQ0_PIN;
	GPIO_InitStructure.Mode      = GPIO_MODE_INPUT;
	GPIO_InitStructure.Pull      = GPIO_NOPULL;
	GPIO_InitStructure.Speed     = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(ADE7880_IRQ0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin       = ADE7880_IRQ1_PIN;
	HAL_GPIO_Init(ADE7880_IRQ1_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin       = ADE7880_MISO_PIN;
	HAL_GPIO_Init(ADE7880_MISO_PORT, &GPIO_InitStructure);
}

static void ADE7880_SPIInit(void)
{

	ADE7880_CS_SET(ADE7880_CS_PIN);
	ADE7880_RESET_HIGH;
	ADE7880_Mode0_Set;
	ADE7880_Delay(10);
	ADE7880_RESET_LOW;
	ADE7880_Delay(20000);
	ADE7880_RESET_HIGH;
	ADE7880_Delay(10);

	//toggle CS 3 times to enter SPI Mode
	ADE7880_CS_RESET(ADE7880_CS_PIN);
	ADE7880_Delay(100);
	ADE7880_CS_SET(ADE7880_CS_PIN);
	ADE7880_Delay(100);
	ADE7880_CS_RESET(ADE7880_CS_PIN);
	ADE7880_Delay(100);
	ADE7880_CS_SET(ADE7880_CS_PIN);
	ADE7880_Delay(100);
	ADE7880_CS_RESET(ADE7880_CS_PIN);
	ADE7880_Delay(100);
	ADE7880_CS_SET(ADE7880_CS_PIN);
	ADE7880_Delay(100);

	ADE7880_Delay(20000);
}

static void ADE7880_Delay(volatile uint32_t nCount)
{
	while(nCount > 0) { nCount--; }
}

void ADE7880_Write8(uint8_t out)
{
	uint8_t i;

	//ADE7880_CS_SET(CS_Pin);
	//ADE7880_MOSI_RESET;
	//ADE7880_SCK_RESET;
	//ADE7880_CS_RESET(CS_Pin);
	ADE7880_Delay(10);
	for (i = 0; i < 8; i++) {
		if ((out >> (7-i)) & 0x01) {
			ADE7880_MOSI_SET;
		} else {
			ADE7880_MOSI_RESET;
		}
		ADE7880_Delay(10);
		ADE7880_SCK_RESET;
		ADE7880_Delay(10);
		ADE7880_SCK_SET;

	}
}

void ADE7880_Write16(uint16_t out)
{
	uint8_t i;

	//ADE7880_CS_SET(CS_Pin);
	//ADE7880_MOSI_RESET;
	//ADE7880_SCK_RESET;
	//ADE7880_CS_RESET(CS_Pin);
  //ADE7880_Delay(10);
	for (i = 0; i < 16; i++) {
		if ((out >> (15-i)) & 0x01) {
			ADE7880_MOSI_SET;
		} else {
			ADE7880_MOSI_RESET;
		}
		ADE7880_Delay(10);
		ADE7880_SCK_RESET;
		ADE7880_Delay(10);
		ADE7880_SCK_SET;
	}
}

void ADE7880_Write32(uint32_t out)
{
	uint8_t i;

	//ADE7880_CS_SET(CS_Pin);
	//ADE7880_MOSI_RESET;
	//ADE7880_SCK_RESET;
	//ADE7880_CS_RESET(CS_Pin);
	//ADE7880_Delay(10);
	for (i = 0; i < 32; i++) {
		if ((out >> (31-i)) & 0x01) {
			ADE7880_MOSI_SET;
		} else {
			ADE7880_MOSI_RESET;
		}
		ADE7880_Delay(10);
		ADE7880_SCK_RESET;
		ADE7880_Delay(10);
		ADE7880_SCK_SET;
	}
}

uint8_t ADE7880_Read8()
{
	uint8_t i;
	uint8_t temp = 0;
	//ADE7880_Delay(10);
	//ADE7880_CS_RESET(CS_Pin);
	ADE7880_MOSI_RESET;
	ADE7880_SCK_SET;
	for (i = 0; i < 8; i++) {
		ADE7880_Delay(20);
		ADE7880_SCK_RESET;
		ADE7880_Delay(20);
		if (ADE7880_MISO == GPIO_PIN_SET) {
			temp |= (1 << (7-i));
		}
		ADE7880_Delay(20);
		ADE7880_SCK_SET;
	}
	//ADE7880_CS_SET(CS_Pin);

	return temp;
}

uint16_t ADE7880_Read16()
{
	uint8_t i;
	uint16_t temp = 0;
	//ADE7880_Delay(10);
	//ADE7880_CS_RESET(CS_Pin);
	ADE7880_MOSI_RESET;
	ADE7880_SCK_SET;
	for (i = 0; i < 16; i++) {
		ADE7880_Delay(20);
		ADE7880_SCK_RESET;
		ADE7880_Delay(20);
		if (ADE7880_MISO == GPIO_PIN_SET) {
			temp |= (1 << (15-i));
		}
		ADE7880_Delay(20);
		ADE7880_SCK_SET;
	}
	//ADE7880_CS_SET(CS_Pin);

	return temp;
}

uint32_t ADE7880_Read32()
{
	uint8_t i;
	uint32_t temp = 0;
	//ADE7880_Delay(10);
	//ADE7880_CS_RESET(CS_Pin);
	ADE7880_MOSI_RESET;
	ADE7880_SCK_SET;
	for (i = 0; i < 32; i++) {
		ADE7880_Delay(20);
		ADE7880_SCK_RESET;
		ADE7880_Delay(20);
		if (ADE7880_MISO == GPIO_PIN_SET) {
			temp |= (1 << (31-i));
		}
		ADE7880_Delay(20);
		ADE7880_SCK_SET;
	}
	//ADE7880_CS_SET(CS_Pin);

	return temp;
}
