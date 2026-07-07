/*
 * config.h
 *
 *  Created on: Sep 1, 2025
 *      Author: firza
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32g4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

extern UART_HandleTypeDef huart1;

extern char broker_host[64];
extern int  broker_port;
extern char client_id[64];
extern char username[64];
extern char password[64];

extern char mqtt_topic_pub[128];
extern char mqtt_topic_sub[128];
extern char cco_topic_reg[128];
extern char sta_topic_reg[128];

extern int mqtt_msg_id;
extern char mac_cco[24];

#endif
