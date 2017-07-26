
#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define	SMPLRT_DIV		0x19
#define	CONFIG				0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B
#define	MPU6050SlaveAddress	0xD0

#define ACCEL_RANGE ((float)(2 * 9.8))

struct kine_state {
	float x, y, z;
	float vx, vy, vz;
	float ax, ay, az;
};

struct kine_state mpu6050_get_kine_state(struct kine_state *now_state);

void mpu6050_init(I2C_HandleTypeDef *device);
uint8_t mpu6050_read(uint8_t addr, uint8_t reg);
signed int mpu6050_get_data(uint8_t reg);
float mpu6050_get_exact_data(uint8_t reg);

#endif /* MPU6050_H_ */
