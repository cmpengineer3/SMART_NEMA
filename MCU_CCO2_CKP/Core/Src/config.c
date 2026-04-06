/*
 * config.c
 *
 *  Created on: Sep 1, 2025
 *      Author: firza
 */

#include "config.h"
//#include "stdint.h"
//==============================================================================================================
/* ——— Default Config (silakan ubah di runtime) ——— */
char broker_host[64] = "staging.smartsiklon.co.id";
int  broker_port     = 1883;


/* Jika ingin menempel UID ke client_id, gabungkan sendiri di main sebelum connect */
char client_id[64]   = "PJUTS_Client";
char username[64]    = "admin";      /* kosong = tanpa auth */
char password[64]    = "xK645P7Np6LR";
//==============================================================================================================
////// Konfigurasi MQTT Broker
//char broker_host[64] = "broker.hivemq.com";
//int  broker_port     = 1883;
//char client_id[64]   = "stm32Client";
//char username[64]    = "";  // kosong = tanpa auth
//char password[64]    = "";

//==============================================================================================================
// Topic MQTT
char mqtt_topic_pub[128] = "smartsiklon/000100000001/uplink";
char mqtt_topic_sub[128] = "smartsiklon/000100000001/downlink";
char cco_topic_reg[128] = "smartsiklon/cco/register";
char sta_topic_reg[128] = "smartsiklon/000100000001/sta/register";

// Message ID counter
int mqtt_msg_id = 1;
char mac_cco[24];

//==============================================================================================================
//char header_x[64]	 = "E";
//char header_y[64]	 = "D";
//
//int  client_idx   = 0;
//char mqtt_topic_pub[96];
//char mqtt_topic_sub[96];
//int  mqtt_qos        = 1;
//int  mqtt_retain     = 0;
//
//float latitude		= 0.0f;
//float longitude		= 0.0f;
//int year			= 0;
//int month			= 0;
//int day				= 0;
//int hour			= 0;
//int minute			= 0;
//int second			= 0;



