#include "main.h"
#include "i2c.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS 0xD0

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{	
	HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDRESS,RegAddress,I2C_MEMADD_SIZE_8BIT,&Data,1,100);	
}

void MPU6050_ReadRegs(uint8_t Reg, uint8_t *Buf, uint8_t Len)
{
    HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDRESS, Reg, I2C_MEMADD_SIZE_8BIT, Buf, Len, 100);
}

void MPU6050_Init(void)
{
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);
	MPU6050_WriteReg(MPU6050_CONFIG,0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x00);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x00);
}

void MPU6050_GetData(int16_t *Accx,int16_t *Accy,int16_t *Accz,int16_t *Gyrox,int16_t *Gyroy,int16_t *Gyroz)
{
	uint8_t DataH,DataL;
	
	MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H,&DataH,1);
	MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_L,&DataL,1);
	*Accx=(uint16_t)((DataH<<8)|DataL);
	
	MPU6050_ReadRegs(MPU6050_ACCEL_YOUT_H,&DataH,1);
	MPU6050_ReadRegs(MPU6050_ACCEL_YOUT_L,&DataL,1);
	*Accy=(uint16_t)((DataH<<8)|DataL);
	
	MPU6050_ReadRegs(MPU6050_ACCEL_ZOUT_H,&DataH,1);
	MPU6050_ReadRegs(MPU6050_ACCEL_ZOUT_L,&DataL,1);
	*Accz=(uint16_t)((DataH<<8)|DataL);
	
	MPU6050_ReadRegs(MPU6050_GYRO_XOUT_H,&DataH,1);
	MPU6050_ReadRegs(MPU6050_GYRO_XOUT_L,&DataL,1);
	*Gyrox=(uint16_t)((DataH<<8)|DataL);
	
	MPU6050_ReadRegs(MPU6050_GYRO_YOUT_H,&DataH,1);
	MPU6050_ReadRegs(MPU6050_GYRO_YOUT_L,&DataL,1);
	*Gyroy=(uint16_t)((DataH<<8)|DataL);
	
	MPU6050_ReadRegs(MPU6050_GYRO_ZOUT_H,&DataH,1);
	MPU6050_ReadRegs(MPU6050_GYRO_ZOUT_L,&DataL,1);
	*Gyroz=(uint16_t)((DataH<<8)|DataL);
}
