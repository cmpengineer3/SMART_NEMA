/*
 * uid_config.h — Kelola UID device via UART debug (huart1) + simpan ke FRAM.
 *
 * Perintah via serial huart1 (115200), diakhiri Enter (\r atau \n):
 *     SETUID:KWH01310000099
 *
 * Alur:
 *   - UID_Load()  dipanggil di startup → baca UID dari FRAM (kalau valid),
 *                 kalau belum ada pakai UID_DEFAULT.
 *   - UID_FeedByte() dipanggil dari HAL_UART_RxCpltCallback saat byte huart1
 *                 masuk → kumpulkan jadi baris perintah.
 *   - UID_Process() dipanggil di loop utama → kalau ada perintah lengkap,
 *                 parse "SETUID:...", simpan ke FRAM, lalu (default) restart.
 */

#ifndef INC_UID_CONFIG_H_
#define INC_UID_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

/* Panjang maksimum UID (termasuk NUL). Sesuaikan bila UID lebih panjang. */
#define UID_MAXLEN   24

/* UID default kalau FRAM belum pernah diisi UID valid. */
#define UID_DEFAULT  "KWH0426DEMO0"

/* Buffer UID aktif (dipakai di main.c untuk topic & payload). */
extern char device_uid[UID_MAXLEN];

/* Baca UID dari FRAM ke device_uid. Kalau tidak valid → pakai UID_DEFAULT.
 * Panggil sekali di startup, SEBELUM menyusun topic MQTT. */
void UID_Load(void);

/* Simpan UID ke FRAM (persisten). return true kalau sukses. */
bool UID_Save(const char *new_uid);

/* Dipanggil dari HAL_UART_RxCpltCallback (cabang USART1) untuk tiap byte. */
void UID_FeedByte(uint8_t b);

/* Dipanggil di loop utama. Kalau ada perintah "SETUID:..." lengkap:
 * simpan UID ke FRAM lalu restart device (agar topic tersusun ulang). */
void UID_Process(void);

#endif /* INC_UID_CONFIG_H_ */
