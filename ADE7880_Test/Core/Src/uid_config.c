/*
 * uid_config.c
 *
 *  Created on: Aug 31, 2026
 *      Author: firza
 */


/*
 * uid_config.c — Persistensi UID device di FRAM.
 *
 * Layout FRAM (alamat mulai addr_sn, lihat fram.h):
 *   addr_sn + 0        : magic byte  (0xA5 kalau UID pernah disimpan lewat SETUID)
 *   addr_sn + 1 .. +23 : string UID, NUL-terminated
 *
 * Dipakai oleh perintah debug "SETUID:xxxxxxxxxxxx" di ProcessCmd() (main.c).
 */
#include "uid_config.h"
#include "fram.h"
#include <string.h>

#define UID_MAGIC       0xA5
#define UID_ADDR_MAGIC  addr_sn
#define UID_ADDR_STR    (addr_sn + 1)

/* device_uid[] didefinisikan di main.c — dipakai juga untuk topic MQTT/CCO. */
extern char device_uid[UID_MAXLEN];

static bool uid_char_ok(char c)
{
    return (c >= '0' && c <= '9') ||
           (c >= 'A' && c <= 'Z') ||
           (c >= 'a' && c <= 'z');
}

void UID_Load(void)
{
    uint8_t magic = ReadData_FRAM(UID_ADDR_MAGIC);
    if (magic != UID_MAGIC) return;   /* belum pernah disimpan → pakai default */

    char tmp[UID_MAXLEN] = {0};
    for (int i = 0; i < UID_MAXLEN - 1; i++)
    {
        uint8_t c = ReadData_FRAM(UID_ADDR_STR + i);
        if (c == '\0' || c == 0xFF) { tmp[i] = '\0'; break; }
        tmp[i]     = (char)c;
        tmp[i + 1] = '\0';
    }

    if (tmp[0] != '\0')
    {
        strncpy(device_uid, tmp, UID_MAXLEN - 1);
        device_uid[UID_MAXLEN - 1] = '\0';
    }
}

bool UID_Save(const char *new_uid)
{
    if (new_uid == NULL) return false;

    size_t len = strlen(new_uid);
    if (len == 0 || len >= UID_MAXLEN) return false;

    for (size_t i = 0; i < len; i++)
        if (!uid_char_ok(new_uid[i])) return false;

    WriteChar_FRAM(UID_ADDR_STR, (char *)new_uid);
    WriteData_FRAM(UID_ADDR_MAGIC, UID_MAGIC);

    strncpy(device_uid, new_uid, UID_MAXLEN - 1);
    device_uid[UID_MAXLEN - 1] = '\0';
    return true;
}
