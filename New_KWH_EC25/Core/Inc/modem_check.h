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


bool Modem_SyncAndCheck(ModemStatus *st);


bool Modem_FixBaudrate115200(void);

bool Modem_AutoBaudrate(uint32_t target_baud);

#endif /* INC_MODEM_CHECK_H_ */
