/*
 * mpu9250_imu.h
 *
 * Created: 11/10/2017 12:07:23 AM
 *  Author: Alex Bennett
 */ 


#ifndef MPU9250_IMU_H_
#define MPU9250_IMU_H_

#include "mpu9250_register_map.h"

#define MPU9250_ADDR_DEFAULT		0x68

/**
 * \brief Return codes for MPU-9250 interface.
 */
enum mpu9250_return_code {
	MPU9250_SUCCESS = 0,
	MPU9250_RW_ERROR = 1,
	MPU9250_INIT_ERROR = 2
} typedef mpu9250_return_code_t;

enum mpu9250_accel_range {
	MPU9250_ACCEL_RANGE_2G = 0x00,
	MPU9250_ACCEL_RANGE_4G = 0x01,
	MPU9250_ACCEL_RANGE_8G = 0x02,
	MPU9250_ACCEL_RANGE_16G = 0x03
} typedef mpu9250_accel_range_t;

enum mpu9250_gyro_range {
	MPU9250_GYRO_RANGE_250DPS = 0x00,
	MPU9250_GYRO_RANGE_500DPS = 0x01,
	MPU9250_GYRO_RANGE_1000DPS = 0x02,
	MPU9250_GYRO_RANGE_2000DPS = 0x03
} typedef mpu9250_gyro_range_t;

struct mpu9250_config {
	uint8_t mpu9250_chip_addr;
	uint8_t ak8963_chip_addr;
	mpu9250_accel_range_t accel_range;
	mpu9250_gyro_range_t gyro_range;
} typedef mpu9250_config_t;

struct mpu9250_calibration {
	float accel_scale;
	float gyro_scale;
	float mag_scale[3];
} typedef mpu9250_calibration_t;

mpu9250_return_code_t mpu9250_init(const mpu9250_config_t* config, mpu9250_calibration_t* calibration);
mpu9250_return_code_t mpu9250_write_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data);
mpu9250_return_code_t mpu9250_read_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* dest);
uint8_t mpu9250_whoami(const mpu9250_config_t* config);
mpu9250_return_code_t mpu9250_write_register_ak8963(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data);
mpu9250_return_code_t mpu9250_read_register_ak8963(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* dest);
uint8_t mpu9250_whoami_ak8963(const mpu9250_config_t* config);

mpu9250_return_code_t mpu9250_read_gyro(const mpu9250_config_t* config, int16_t* dest);
mpu9250_return_code_t mpu9250_read_accel(const mpu9250_config_t* config, int16_t* dest);
mpu9250_return_code_t mpu9250_read_mag(const mpu9250_config_t* config, int16_t* dest);

#endif /* MPU9250_IMU_H_ */