/*
 * config.h
 *
 *  Created on: Sep 1, 2025
 *      Author: firza
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "mqtt.h"

extern char broker_host[64];
extern int  broker_port;
extern char client_id[64];
extern char username[64];
extern char password[64];

extern char mqtt_topic_pub[128];
extern char mqtt_topic_pubs[128];
extern char mqtt_topic_sub[128];
extern char cco_topic_reg[128];
extern char sta_topic_reg[128];

extern char mac_cco[24];
/* mqtt_msg_id removed — managed internally by mqtt.c (auto-increment) */

/* Helper: build MQTT_Config struct from config variables above */
MQTT_Config Config_GetMQTT(void);

#endif
