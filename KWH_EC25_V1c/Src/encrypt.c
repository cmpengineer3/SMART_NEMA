/*
 * encrypt.c
 *
 *  Created on: Jul 28, 2019
 *      Author: user
 */
#include "encrypt.h"
#include "main.h"
#include "usart.h"

//const char SEC[] =  "B75B4C0825573E172A4B3DC8B80AD84169C74345"; //KWH01310000018
//const char SEC[] =  "04D2C4C60234916F134C7CE7978C7B04DF765D1D"; //KWH01310000019
//const char SEC[] =  "A2B8ED97E137977BF2E47AC400B7AB0621589808"; //KWH01310000020
//const char SEC[] =  "01E08583120A745329C2CFFD837B83CA397F2860"; //KWH01310000021
const char SEC[] =  "4E828FAF6D3B23BB89208B6EFDE030617F08CEDD"; //KWH01310000022
//const char SEC[] =  "A509908BAE0BB95B58D6000785E52F95D9BFFADC"; //KWH01310000023
//const char SEC[] =  "D67A290F7B92753B808B9F480E3F944D967F3D73"; //KWH01310000024
//const char SEC[] =  "B9715EC40820B49B667D0C09734516F0DAE1E664"; //KWH01310000025
//const char SEC[] =  "C91C32F669266BF0F2712524445B4E8EBFBB4326"; //KWH01310000026
//const char SEC[] =  "1DDEEB5806CF938BB2E2FE79A5AD4E67333995C9"; //KWH01310000027
//const char SEC[] =  "7D705D8F92B72796D425A782B37319896658517C"; //KWH01310000028
//const char SEC[] =  "55FBC096824B54E74D90DCF8948FCB3920F45E01"; //UJIKWH69-2


//const char SEC[] =  "B873187C193B9D7D029B836873DAC239C790A918"; //Demo Smart KWH 03 EG95
//const char SEC[] =  "521CCC597F6C21ECAFFB89D6C7A97EF3B0F56509"; //Demo Smart KWH EG95 ID1
//const char SEC[] =  "C08A2C6CFC19098E6BB2B3D7B72B150BE905B76C"; //Demo Smart KWH EG95 ID1

//const char SEC[] =  "11348AD41AC784A505502F1B480AB17BCCDBB717"; //Demo Smart KWH 02
//const char SEC[] =  "818465E51C292486A4132BD5DF20D60B1ECB453A"; //Demo Smart KWH 01



//const char SEC[] =  "AFF444991EE8294C67AE44E975F85E21CAFCFD29"; //8
//const char SEC[] =  "54EB6B5431D0EDB147E9666A703FAA456F5DC639"; //7
//const char SEC[] =  "9FD74977CDE17170C0BA930C0D4F2FF9EC6E6D25"; //6
//const char SEC[] =  "F3ADB75D757CD642576054F5649B1937D9DB295E"; //5
//const char SEC[] =  "B36757B56685C6498B905755F9BF0B41EE3AA168"; //4
//const char SEC[] =  "595985DF27DAC2AB15AECAE97F4A6FD800DE5BCC"; //3
//const char SEC[] =  "D020FD807A812C882773C971B7BD0A9CB0DFD063"; //2
//const char SEC[] =  "2E87E88C476465C8C91405C18F0BC2FB97CD65FF"; //1

#define MAX_DATA      1024
#define MAX_KEY       10
#define MAX_RAND_KEY  39

char e_plain_data[MAX_DATA];
char e_plain_key[MAX_KEY];
char e_encrypted_data[MAX_DATA];
char e_encrypted_key[MAX_KEY];

char d_plain_data[MAX_DATA];
char d_plain_key[MAX_KEY];
char d_encrypted_data[MAX_DATA];
char d_encrypted_key[MAX_KEY];

char r, c;

long previousTime = 0;
long interval = 10000;

int count = 0;

void encrypt(char *pDataIn,char *pEncryptedKey,char *pEncryptedData, int dl) {
    char data[200];

	int data_len = dl;//strlen(pDataIn);

//  	uint16_t n = sprintf(data,"data length %d\r\n",data_len);
//  	HAL_UART_Transmit(&huart1,data,n,1000);

    int r_idx[MAX_KEY];
    int r_key[MAX_KEY];

    for (int i=0; i<MAX_KEY; i++) {
        r_idx[i] = rand() % MAX_RAND_KEY;
        r_key[i] = (int) (SEC[r_idx[i]]);
        pEncryptedKey[i] = r_idx[i] + 65 + i;

        if (i < (MAX_KEY - 1)) {
        	pEncryptedKey[i + 1] = 0;
            r_idx[i + 1] = 0;
            r_key[i + 1] = 0;
        }
    }

    int loop = 0;

    for (int i=0; i<data_len; i++) {
    	pEncryptedData[i] = (int) pDataIn[i] + r_key[loop] - 34;

    	//add more backslash
        if (i < data_len - 1) {
        	pEncryptedData[i + 1] = 0;
        }

        loop++;
        if (loop >= MAX_KEY) {
            loop = 0;
        }
    }
}

void decrypt(char *pEncryptedKey,char *pEncryptedData,char *pDataOut, int dl,int kl) {
    int key_len  = kl;//strlen(pEncryptedKey);//kl;//
    int data_len = dl-key_len-1;//strlen(pEncryptedData);////dl-key_len;//

    int r_idx[key_len];
    int r_key[key_len];

    for (int i=0; i<key_len; i++) {
        r_idx[i] = (int) pEncryptedKey[i] - i - 65;
        r_key[i] = (int) (SEC[r_idx[i]]);
        d_plain_key[i] = SEC[r_idx[i]];

        if (i < (key_len - 1)) {
            d_plain_key[i + 1] = 0;
            r_idx[i + 1] = 0;
            r_key[i + 1] = 0;
        }
    }

    int loop = 0;

    for (int i=0; i<data_len; i++) {
    	pDataOut[i] = (int) pEncryptedData[i] - r_key[loop] + 34;

        if (i < data_len - 1) {
        	pDataOut[i + 1] = 0;
        }

        loop++;
        if (loop >= key_len) {
            loop = 0;
        }
    }
}

