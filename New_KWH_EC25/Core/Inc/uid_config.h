

#ifndef INC_UID_CONFIG_H_
#define INC_UID_CONFIG_H_

#include <stdint.h>
#include <stdbool.h>

#define UID_MAXLEN   24

#define UID_DEFAULT  "KWH012DEMO05" // UID BAWAAN

extern char device_uid[UID_MAXLEN];

void UID_Load(void);

bool UID_Save(const char *new_uid);

void UID_FeedByte(uint8_t b);

void UID_Process(void);

void Debug_PrintStatus(void);

#endif /* INC_UID_CONFIG_H_ */
