#include "mpu6050.h"

float axd, ayd, azd, wxd, wyd, wzd;
I2C_HandleTypeDef *mpu6050_i2c_device;

float int2float(signed int i)
{
	float result;
	if(i > 32767) {
		result =  (float)((unsigned int) i) - 65536.0;
	} else {
		result = (float) i;
	}
	return result;
}

uint8_t mpu6050_read(uint8_t addr, uint8_t reg)
{
	uint8_t data;

	while(HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_READY && HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_BUSY_RX);
	HAL_I2C_Master_Transmit(mpu6050_i2c_device, addr, &reg, 1, -1);
	while(HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_READY && HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_BUSY_TX);
	HAL_I2C_Master_Receive(mpu6050_i2c_device, addr, &data, 1, -1);

	return data;
}

uint8_t mpu6050_write(uint8_t addr, uint8_t reg, uint8_t data)
{
	uint8_t msg[2];
	msg[0] = reg;
	msg[1] = data;
	while(HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_READY && HAL_I2C_GetState(mpu6050_i2c_device) != HAL_I2C_STATE_BUSY_RX);
	HAL_I2C_Master_Transmit(mpu6050_i2c_device, addr, msg, 2, -1);
	return 0;
}

signed int mpu6050_get_data(uint8_t reg)
{
	return ((uint16_t)mpu6050_read(MPU6050SlaveAddress, reg) << 8) | mpu6050_read(MPU6050SlaveAddress, reg + 1);
}

float mpu6050_get_exact_data(uint8_t reg)
{
	float result = int2float(mpu6050_get_data(reg));
	switch(reg)
	{
		case ACCEL_XOUT_H:
			result -= axd;
			break;

		case ACCEL_YOUT_H:
			result -= ayd;
			break;

		case ACCEL_ZOUT_H:
			result -= azd;
			break;

		case GYRO_XOUT_H:
			result -= wxd;
			break;

		case GYRO_YOUT_H:
			result -= wyd;
			break;

		case GYRO_ZOUT_H:
			result -= wzd;
			break;

		default:
			break;
	}
	return result;
}

void mpu6050_set_average_values(void)
{
	uint16_t i;

	axd = 0;
	ayd = 0;
	azd = 0;
	wxd = 0;
	wyd = 0;
	wzd = 0;

	#define MPU_SUM 250

	for(i = 0; i < MPU_SUM; i++)
	{
		axd += int2float(mpu6050_get_data(ACCEL_XOUT_H)) / MPU_SUM;
		ayd += int2float(mpu6050_get_data(ACCEL_YOUT_H)) / MPU_SUM;
		azd += int2float(mpu6050_get_data(ACCEL_ZOUT_H)) / MPU_SUM;
		wxd += int2float(mpu6050_get_data(GYRO_XOUT_H)) / MPU_SUM;
		wyd += int2float(mpu6050_get_data(GYRO_YOUT_H)) / MPU_SUM;
		wzd += int2float(mpu6050_get_data(GYRO_ZOUT_H)) / MPU_SUM;
	}
}

struct kine_state mpu6050_get_kine_state(struct kine_state *now_state)
{
	struct kine_state result;
	float ax, ay, az;

	ax = mpu6050_get_exact_data(ACCEL_XOUT_H);
	ay = mpu6050_get_exact_data(ACCEL_YOUT_H);
	az = mpu6050_get_exact_data(ACCEL_ZOUT_H);

	result.ax = ax * ACCEL_RANGE / 32767;
	result.ay = ay * ACCEL_RANGE / 32767;
	result.az = az * ACCEL_RANGE / 32767;

	return result;
}

void mpu6050_init(I2C_HandleTypeDef *device)
{
	mpu6050_i2c_device = device;

	mpu6050_write(MPU6050SlaveAddress, PWR_MGMT_1, 0x00);
	mpu6050_write(MPU6050SlaveAddress, SMPLRT_DIV, 0x00);
	mpu6050_write(MPU6050SlaveAddress, CONFIG, 0x00);
	mpu6050_write(MPU6050SlaveAddress, GYRO_CONFIG, 0x18);
	mpu6050_write(MPU6050SlaveAddress, ACCEL_CONFIG, 0x00);

	mpu6050_set_average_values();
}
