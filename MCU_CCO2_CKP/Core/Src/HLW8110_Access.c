/*
 * HLW8110_Access.c
 *
 * Driver HLW8110/HLW8112 - Power Metering
 * Upgrade dari CARA A ke State Machine Parser
 *
 * Yang BERUBAH dari versi sebelumnya:
 *   - Variabel indx dan length DIHAPUS (tidak diperlukan lagi)
 *   - Tambah HLW_OnRxByte() → state machine 4-state
 *   - dataMerge() tetap sama persis
 *   - Semua fungsi lain (HLW_Request, HLW_WriteReg, dll) tetap sama
 *
 * Yang TIDAK BERUBAH:
 *   - Nama semua fungsi publik
 *   - Struct RegBuffer, RxTxVar
 *   - Semua define REG_xxx
 *   - Cara pemanggilan di main.c dan uart.c (hanya 1 baris ganti di uart.c)
 */

#include "HLW8110_Access.h"

/* ============================================================
 * VARIABEL GLOBAL
 * ============================================================ */
RegBuffer   regbuffer;
RxTxVar     rxtxVar;
uint8_t     reqADDR  = 0;
bool        rcv_flag = false;

/* ============================================================
 * STATE MACHINE — variabel internal (tidak perlu diakses luar)
 * ============================================================ */
static parse_state_t    _state        = PARSE_WAIT_START;
static uint8_t          _frame_addr   = 0;
static uint8_t          _expected_len = 0;
static uint8_t          _data_buf[5]  = {0};
static uint8_t          _data_idx     = 0;

/* ============================================================
 * FUNGSI INTERNAL (private/static)
 * ============================================================ */

/**
 * @brief Aktifkan mode tulis ke register HLW (Write Enable)
 *        Harus dipanggil sebelum HLW_WriteReg()
 */
static void HLW_EnableWrite(UART_HandleTypeDef *huart)
{
    uint8_t frame[4];
    frame[0] = 0xA5;
    frame[1] = 0xEA;
    frame[2] = 0xE5;
    uint8_t cs = frame[0] + frame[1] + frame[2];
    frame[3] = ~cs;
    HAL_UART_Transmit(huart, frame, 4, HAL_MAX_DELAY);
}

/**
 * @brief Tentukan jumlah byte data untuk suatu register address
 *        Sesuai dengan datasheet HLW8110/8112
 */
static uint8_t _get_expected_len(uint8_t addr)
{
    switch (addr)
    {
        /* 16-bit → 2 byte data */
        case REG_RmsUC:
        case REG_RmsIAC:
        case REG_RmsIBC:
        case REG_PowerPAC:
        case REG_PowerPBC:
        case REG_PowerSC:
        case REG_Ufreq:
            return 2;

        /* 32-bit → 4 byte data */
        case REG_PowerPA:
        case REG_PowerPB:
        case REG_PowerS:
            return 4;

        /* 24-bit → 3 byte data (default) */
        case REG_RmsU:
        case REG_RmsIA:
        case REG_RmsIB:
        case REG_PF:
        case REG_EMUstatus:
        default:
            return 3;
    }
}

/**
 * @brief Verifikasi checksum frame HLW
 *
 * Algoritma checksum HLW8110/8112:
 *   sum = 0xA5 + addr + data[0] + ... + data[n-1]
 *   valid jika (sum & 0xFF) == checksum_byte
 *
 * @param addr      Byte address frame
 * @param data      Pointer ke byte-byte data
 * @param len       Jumlah byte data
 * @param cs        Byte checksum yang diterima
 * @return true jika checksum valid
 */
static bool _verify_checksum(uint8_t addr, uint8_t *data, uint8_t len, uint8_t cs)
{
    uint32_t sum = 0xA5 + addr;
    for (uint8_t i = 0; i < len; i++)
        sum += data[i];
    return ((uint8_t)(sum & 0xFF) == cs);
}

/* ============================================================
 * FUNGSI PUBLIK
 * ============================================================ */

/**
 * @brief Tulis nilai 16-bit ke register HLW
 *        Format frame: [0xA5][0x80|addr][data_hi][data_lo][~checksum]
 */
void HLW_WriteReg(UART_HandleTypeDef *huart, uint8_t address, uint16_t data)
{
    uint8_t frame[5];
    uint8_t cmd = 0x80 | address;

    frame[0] = 0xA5;
    frame[1] = cmd;
    frame[2] = (data >> 8) & 0xFF;
    frame[3] = data & 0xFF;

    uint8_t cs = frame[0] + frame[1] + frame[2] + frame[3];
    frame[4] = ~cs;

    HAL_UART_Transmit(huart, frame, 5, HAL_MAX_DELAY);
}

/**
 * @brief Aktifkan WaveEN + ZXEN di register EMUCON2 (0x13)
 *        Diperlukan agar REG_Ufreq (frekuensi) bisa dibaca
 *        Panggil SEKALI di awal sebelum main loop
 */
void HLW_Activate_Freq_Measurement(UART_HandleTypeDef *huart)
{
    HLW_EnableWrite(huart);
    HAL_Delay(5);
    HLW_WriteReg(huart, 0x13, 0x0025);
}

/**
 * @brief Kirim request baca register ke chip HLW
 *        Format TX: [0xA5][address]
 *
 *        Setelah fungsi ini:
 *          ISR terima byte → HLW_OnRxByte() → state machine
 *          → jika frame valid → dataMerge() → rcv_flag = true
 *
 * @param huart    Handle UART (&huart3)
 * @param address  Alamat register yang ingin dibaca
 */
void HLW_Request(UART_HandleTypeDef *huart, uint8_t address)
{
    rcv_flag = false;
    reqADDR  = address;

    uint8_t frame[2];
    frame[0] = 0xA5;
    frame[1] = address;
    HAL_UART_Transmit(huart, frame, 2, HAL_MAX_DELAY);
}

/**
 * @brief State machine parser untuk frame response HLW.
 *        Dipanggil dari HAL_UART_RxCpltCallback saat huart == USART3.
 *
 * State transitions:
 *
 *  PARSE_WAIT_START
 *    Tunggu byte 0xA5. Semua byte lain diabaikan.
 *    → Terima 0xA5 : pindah ke PARSE_GOT_ADDR
 *
 *  PARSE_GOT_ADDR
 *    Byte ini adalah address register.
 *    Tentukan expected_len dari address.
 *    → expected_len > 0 : pindah ke PARSE_COLLECT_DATA
 *    → expected_len = 0 : kembali ke PARSE_WAIT_START
 *
 *  PARSE_COLLECT_DATA
 *    Kumpulkan byte data ke _data_buf[] satu per satu.
 *    → Sudah terkumpul expected_len byte : pindah ke PARSE_GOT_CHECKSUM
 *
 *  PARSE_GOT_CHECKSUM
 *    Byte ini adalah checksum.
 *    Verifikasi: sum(0xA5 + addr + data[]) & 0xFF == checksum
 *    → Valid   : salin ke bufferRx → dataMerge() → rcv_flag=true
 *    → Invalid : buang frame (tidak masuk regbuffer)
 *    → Selalu kembali ke PARSE_WAIT_START
 *
 * @param b  Byte yang diterima dari UART (isi rxtxVar.dataRx)
 */
void HLW_OnRxByte(uint8_t b)
{
    switch (_state)
    {
        /* ------------------------------------------------
         * State 0: Tunggu start byte 0xA5
         * ------------------------------------------------ */
        case PARSE_WAIT_START:
            if (b == 0xA5)
                _state = PARSE_GOT_ADDR;
            break;

        /* ------------------------------------------------
         * State 1: Terima address, tentukan panjang data
         * ------------------------------------------------ */
        case PARSE_GOT_ADDR:
            _frame_addr   = b;
            _expected_len = _get_expected_len(_frame_addr);
            _data_idx     = 0;

            if (_expected_len == 0)
                _state = PARSE_WAIT_START;
            else
                _state = PARSE_COLLECT_DATA;
            break;

        /* ------------------------------------------------
         * State 2: Kumpulkan byte data
         * ------------------------------------------------ */
        case PARSE_COLLECT_DATA:
            _data_buf[_data_idx++] = b;
            if (_data_idx >= _expected_len)
                _state = PARSE_GOT_CHECKSUM;
            break;

        /* ------------------------------------------------
         * State 3: Verifikasi checksum, simpan jika valid
         * ------------------------------------------------ */
        case PARSE_GOT_CHECKSUM:
            if (_verify_checksum(_frame_addr, _data_buf, _expected_len, b))
            {
                /* Salin data yang sudah terverifikasi ke bufferRx */
                for (uint8_t i = 0; i < _expected_len; i++)
                    rxtxVar.bufferRx[i] = _data_buf[i];

                /* Update reqADDR dan gabungkan ke regbuffer */
                reqADDR = _frame_addr;
                dataMerge(_frame_addr);
                rcv_flag = true;
            }
            /* Kembali ke awal, siap terima frame berikutnya */
            _state = PARSE_WAIT_START;
            break;

        default:
            _state = PARSE_WAIT_START;
            break;
    }
}

/**
 * @brief Gabungkan isi bufferRx[] ke regbuffer berdasarkan alamat register
 *        Dipanggil otomatis dari HLW_OnRxByte() saat checksum valid
 *        Isi sama persis dengan versi sebelumnya
 *
 * @param address  Alamat register yang sesuai dengan data di bufferRx
 */
void dataMerge(uint8_t address)
{
    uint32_t val = 0;

    if (address == REG_RmsU)
    {
        val = ((uint32_t)rxtxVar.bufferRx[0] << 16) |
              ((uint32_t)rxtxVar.bufferRx[1] <<  8) |
              ((uint32_t)rxtxVar.bufferRx[2]);
        regbuffer.Vreg = val;
    }
    else if (address == REG_RmsUC)
    {
        val = ((uint16_t)rxtxVar.bufferRx[0] << 8) |
               (uint16_t)rxtxVar.bufferRx[1];
        regbuffer.Vcoeff = (uint16_t)val;
    }
    else if (address == REG_RmsIA)
    {
        val = ((uint32_t)rxtxVar.bufferRx[0] << 16) |
              ((uint32_t)rxtxVar.bufferRx[1] <<  8) |
              ((uint32_t)rxtxVar.bufferRx[2]);
        regbuffer.Ireg = val;
    }
    else if (address == REG_RmsIAC)
    {
        val = ((uint16_t)rxtxVar.bufferRx[0] << 8) |
               (uint16_t)rxtxVar.bufferRx[1];
        regbuffer.Icoeff = (uint16_t)val;
    }
    else if (address == REG_PowerPA)
    {
        val = ((uint32_t)rxtxVar.bufferRx[0] << 24) |
              ((uint32_t)rxtxVar.bufferRx[1] << 16) |
              ((uint32_t)rxtxVar.bufferRx[2] <<  8) |
              ((uint32_t)rxtxVar.bufferRx[3]);
        regbuffer.Preg = val;
    }
    else if (address == REG_PowerPAC)
    {
        val = ((uint16_t)rxtxVar.bufferRx[0] << 8) |
               (uint16_t)rxtxVar.bufferRx[1];
        regbuffer.Pcoeff = (uint16_t)val;
    }
    else if (address == REG_PowerS)
    {
        val = ((uint32_t)rxtxVar.bufferRx[0] << 24) |
              ((uint32_t)rxtxVar.bufferRx[1] << 16) |
              ((uint32_t)rxtxVar.bufferRx[2] <<  8) |
              ((uint32_t)rxtxVar.bufferRx[3]);
        regbuffer.Sreg = val;
    }
    else if (address == REG_PowerSC)
    {
        val = ((uint16_t)rxtxVar.bufferRx[0] << 8) |
               (uint16_t)rxtxVar.bufferRx[1];
        regbuffer.Scoeff = (uint16_t)val;
    }
    else if (address == REG_PF)
    {
        val = ((uint32_t)rxtxVar.bufferRx[0] << 16) |
              ((uint32_t)rxtxVar.bufferRx[1] <<  8) |
              ((uint32_t)rxtxVar.bufferRx[2]);
        regbuffer.PFreg = val & 0x00FFFFFF;
    }
    else if (address == REG_Ufreq)
    {
        val = ((uint16_t)rxtxVar.bufferRx[0] << 8) |
               (uint16_t)rxtxVar.bufferRx[1];
        regbuffer.Ufreqreg = (uint16_t)val;
    }
    else if (address == REG_EMUstatus)
    {
        val = ((uint32_t)rxtxVar.bufferRx[0] << 16) |
              ((uint32_t)rxtxVar.bufferRx[1] <<  8) |
              ((uint32_t)rxtxVar.bufferRx[2]);
        regbuffer.EMUstatus_reg = val;
    }
}

/**
 * @brief Cek kondisi no-load dari bit 19 EMUstatus
 *        NopldA = No Power Load detected on channel A
 * @return true jika tidak ada beban
 */
bool Is_NopldA_Active(void)
{
    return (regbuffer.EMUstatus_reg & (1UL << 19)) ? true : false;
}

/**
 * @brief Baca raw EMU status register
 */
uint32_t Read_EMUStatus_Raw(void)
{
    return regbuffer.EMUstatus_reg;
}
