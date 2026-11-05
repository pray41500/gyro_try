#ifndef __MPU6050_h
#define __MPU6050_h

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);
void MPU6050_ReadRegs(uint8_t Reg, uint8_t *Buf, uint8_t Len);
void MPU6050_Init(void);
void MPU6050_GetData(int16_t *Accx,int16_t *Accy,int16_t *Accz,int16_t *Gyrox,int16_t *Gyroy,int16_t *Gyroz);

#endif
