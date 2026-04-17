/*
 * HLW8110_Access.h
 *
 * Driver HLW8110/HLW8112 - Power Metering
 *
 * Mekanisme RX: State Machine (upgrade dari CARA A)
 *   - Setiap byte dari ISR diproses oleh HLW_OnRxByte()
 *   - Validasi: start byte 0xA5 → address → data → checksum
 *   - Data hanya masuk ke regbuffer jika checksum valid
 *   - Self-recovering: byte noise tidak merusak frame berikutnya
 *
 * Koneksi Hardware:
 *   HLW TX  → USART3 RX (PB11)
 *   HLW RX  → USART3 TX (PB10)
 *   SELECT_HLWIN1 → PA7
 *   SELECT_HLWIN2 → PB0
 *   USART3: 9600 baud, 9-bit word, Parity EVEN
 *
 * Integrasi ke uart.c:
 *   Di HAL_UART_RxCpltCallback, bagian USART3 ganti menjadi:
 *
 *     else if (huart->Instance == USART3)
 *     {
 *         HLW_OnRxByte(rxtxVar.dataRx);
 *         HAL_UART_Receive_IT(&huart3, &rxtxVar.dataRx, 1);
 *     }
 *
 * Inisialisasi di main.c (tidak berubah):
 *     HLW_Activate_Freq_Measurement(&huart3);
 *     HAL_Delay(10);
 *     HAL_UART_Receive_IT(&huart3, (uint8_t*)&rxtxVar.dataRx, 1);
 *
 * Pemanggilan di loop (tidak berubah):
 *     HLW_Request(&huart3, REG_RmsU);
 *     HAL_Delay(10);
 *     param.RMSU[i] = Val_Vrms();  ← dari HLW8110_Calc.h
 */

#ifndef INC_HLW8110_ACCESS_H_
#define INC_HLW8110_ACCESS_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * ALAMAT REGISTER HLW8110/8112
 * (sama persis dengan project asli — tidak diubah)
 * ============================================================ */
#define REG_EMUstatus   0x00
#define REG_RmsU        0x01
#define REG_RmsIA       0x02
#define REG_RmsIB       0x03
#define REG_PowerPA     0x04
#define REG_PowerPB     0x05
#define REG_PowerS      0x06
#define REG_PF          0x08
#define REG_Ufreq       0x09
#define REG_RmsUC       0x0A
#define REG_RmsIAC      0x0B
#define REG_RmsIBC      0x0C
#define REG_PowerPAC    0x0D
#define REG_PowerPBC    0x0E
#define REG_PowerSC     0x0F

/* ============================================================
 * STRUCT
 * (sama persis dengan project asli — tidak diubah)
 * ============================================================ */

/**
 * @brief Buffer register mentah dari HLW chip
 *        Bisa dipantau di Live Expression: regbuffer.Vreg
 */
typedef struct {
    uint32_t Vreg;
    uint16_t Vcoeff;
    uint32_t Ireg;
    uint16_t Icoeff;
    uint32_t Preg;
    uint16_t Pcoeff;
    uint32_t Sreg;
    uint16_t Scoeff;
    uint32_t PFreg;
    uint16_t Ufreqreg;
    uint32_t EMUstatus_reg;
} RegBuffer;

/**
 * @brief Buffer TX/RX untuk komunikasi HLW
 *        dataRx   : byte target HAL_UART_Receive_IT ← TIDAK BERUBAH
 *        bufferRx : buffer data per transaksi register
 */
typedef struct {
    uint8_t dataRx;         /* ← TARGET HAL_UART_Receive_IT (tidak berubah) */
    uint8_t bufferRx[5];    /* Akumulasi byte per transaksi */
} RxTxVar;

/**
 * @brief State machine parser untuk frame response HLW
 *
 * Format frame HLW response:
 *   [0xA5] [addr] [data_0..N] [checksum]
 *
 * Transitions:
 *   PARSE_WAIT_START  —(0xA5)——→  PARSE_GOT_ADDR
 *   PARSE_GOT_ADDR    —(addr)——→  PARSE_COLLECT_DATA
 *   PARSE_COLLECT_DATA—(full)——→  PARSE_GOT_CHECKSUM
 *   PARSE_GOT_CHECKSUM—(OK)————→  merge + rcv_flag → PARSE_WAIT_START
 *                     —(FAIL)——→  PARSE_WAIT_START
 */
typedef enum {
    PARSE_WAIT_START,
    PARSE_GOT_ADDR,
    PARSE_COLLECT_DATA,
    PARSE_GOT_CHECKSUM
} parse_state_t;

/* ============================================================
 * VARIABEL GLOBAL
 * Semua bisa dipantau di Live Expression
 * ============================================================ */
extern RegBuffer    regbuffer;      /* Raw register hasil baca HLW */
extern RxTxVar      rxtxVar;        /* Buffer RX — dataRx = target IT */
extern uint8_t      reqADDR;        /* Register yang sedang diminta */
extern bool         rcv_flag;       /* true = 1 register berhasil dibaca */

/* External UART handle */
extern UART_HandleTypeDef huart3;

/* ============================================================
 * DEKLARASI FUNGSI
 * ============================================================ */

/**
 * @brief Aktifkan WaveEN + ZXEN di chip HLW (EMUCON2 = 0x0025)
 *        Panggil SEKALI sebelum main loop
 * @param huart  Handle UART (&huart3)
 */
void HLW_Activate_Freq_Measurement(UART_HandleTypeDef *huart);

/**
 * @brief Tulis register 16-bit ke HLW
 * @param huart    Handle UART
 * @param address  Alamat register
 * @param data     Nilai 16-bit
 */
void HLW_WriteReg(UART_HandleTypeDef *huart, uint8_t address, uint16_t data);

/**
 * @brief Kirim request baca register ke HLW
 *        Setelah ini, ISR → HLW_OnRxByte() → dataMerge() → rcv_flag = true
 * @param huart    Handle UART (&huart3)
 * @param address  Alamat register (REG_RmsU, dll)
 */
void HLW_Request(UART_HandleTypeDef *huart, uint8_t address);

/**
 * @brief Proses satu byte yang diterima dari UART HLW.
 *        WAJIB dipanggil dari HAL_UART_RxCpltCallback saat huart == USART3.
 *        Menjalankan state machine 4-state untuk parsing frame response.
 *
 *        Keunggulan vs CARA A (indx/length):
 *        - Checksum diverifikasi → data korup tidak masuk regbuffer
 *        - Self-recovering → byte noise tidak merusak frame berikutnya
 *        - Tidak perlu variabel indx/length yang bisa race condition
 *
 * @param b  Byte yang diterima (isi dari rxtxVar.dataRx)
 */
void HLW_OnRxByte(uint8_t b);

/**
 * @brief Gabungkan isi bufferRx[] ke regbuffer berdasarkan alamat register
 *        Dipanggil otomatis dari HLW_OnRxByte() saat checksum valid
 * @param address  Alamat register yang sesuai dengan data di bufferRx
 */
void dataMerge(uint8_t address);

/**
 * @brief Cek kondisi no-load (bit 19 EMUstatus)
 * @return true jika tidak ada beban
 */
bool Is_NopldA_Active(void);

/**
 * @brief Baca raw EMU status register
 */
uint32_t Read_EMUStatus_Raw(void);

#endif /* INC_HLW8110_ACCESS_H_ */
