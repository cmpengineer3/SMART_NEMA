/*
 * modem_check.h — Pengecekan status modem Quectel EC25/EG25 saat startup.
 *
 * Meniru urutan inisialisasi & query status milik QNavigator:
 *   AT sync → ATI → IPR → GSN(IMEI) → CPIN → CIMI → QCCID(ICCID) →
 *   CSQ → CREG → CGREG → COPS → CEREG
 *
 * Dependency: uart.h (modul UART kamu yang sudah jalan).
 * Panggil Modem_SyncAndCheck() SETELAH power-on modem, SEBELUM MQTT connect.
 */

#ifndef INC_MODEM_CHECK_H_
#define INC_MODEM_CHECK_H_

#include <stdbool.h>
#include <stdint.h>

/* Hasil ringkas status modem (opsional dipakai di main). */
typedef struct {
    bool sim_ready;       /* CPIN: READY                        */
    bool registered;      /* CREG/CGREG/CEREG stat = 1 atau 5   */
    int  csq;             /* RSSI dari +CSQ (0..31, 99=unknown) */
    int  ipr;             /* baudrate dari +IPR                 */
    char imei[20];        /* dari AT+GSN                        */
    char imsi[20];        /* dari AT+CIMI                       */
    char iccid[24];       /* dari AT+QCCID                      */
    char operator_name[40];/* dari AT+COPS                      */
} ModemStatus;

/*
 * Sinkronisasi AT (kirim AT tiap 500ms max 10x sampai OK) lalu
 * jalankan seluruh query status. Isi hasil ke *st (boleh NULL).
 *
 * return true  = modem merespon & SIM ready & terdaftar jaringan
 *        false = gagal sync / SIM tidak ready / tidak terdaftar
 */
bool Modem_SyncAndCheck(ModemStatus *st);

/* Set baudrate modem ke 115200 permanen (AT+IPR=115200 lalu AT&W).
 * Panggil hanya jika +IPR bukan 115200. */
bool Modem_FixBaudrate115200(void);

#endif /* INC_MODEM_CHECK_H_ */
