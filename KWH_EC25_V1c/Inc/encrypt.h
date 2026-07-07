#ifndef __ENCRYPT_H
#define __ENCRYPT_H


void encrypt(char *pDataIn,char *pEncryptedKey,char *pEncryptedData,int dl);
void decrypt(char *pEncryptedKey,char *pEncryptedData,char *pDataOut,int dl,int kl);

#endif
