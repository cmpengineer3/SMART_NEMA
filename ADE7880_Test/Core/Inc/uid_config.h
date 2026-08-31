/*
 * uid_config.h
 *
 *  Created on: Aug 31, 2026
 *      Author: firza
 */

/*
 * uid_config.h — Persistensi UID device di FRAM.
 *
 * device_uid[] sendiri tetap didefinisikan sebagai global di main.c (dipakai
 * juga untuk menyusun topic MQTT/CCO) supaya cuma ada satu sumber kebenaran.
 * Modul ini hanya menangani load-dari-FRAM & simpan-ke-FRAM-nya, dipanggil
 * dari perintah debug SETUID:xxxxxxxxxxxx (lihat ProcessCmd() di main.c).
 */
#ifndef INC_UID_CONFIG_H_
#define INC_UID_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#define UID_MAXLEN 24   /* harus sama dengan ukuran device_uid[] di main.c */

/* Baca UID tersimpan dari FRAM ke device_uid. Kalau FRAM belum pernah
 * ditulis (magic byte tidak cocok) atau isinya kosong/rusak, device_uid
 * TIDAK diubah — nilai default bawaan main.c yang tetap dipakai. */
void UID_Load(void);

/* Validasi (harus alfanumerik A-Z a-z 0-9, panjang 1..23 char) lalu tulis
 * ke FRAM + update device_uid. Return false kalau format tidak valid atau
 * new_uid NULL — device_uid TIDAK berubah pada kasus ini. */
bool UID_Save(const char *new_uid);

#endif /* INC_UID_CONFIG_H_ */
