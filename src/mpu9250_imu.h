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

#define MPU9250_SAMPLE_FREQ			200.0
#define MPU9250_KP				(2.0f * 5.0f)	// 2 * proportional gain
#define MPU9250_KI				(2.0f * 0.0f)	// 2 * integral gain

#define AK8963_MAG_MODE				0x06

/**
 * \brief Return codes for MPU-9250 interface.
 */
enum mpu9250_return_code {
	MPU9250_SUCCESS = 0,
	MPU9250_RW_ERROR = 1,
	MPU9250_INIT_ERROR = 2,
	AK8963_CALIBRATION_ERROR = 3,
	AK8963_RW_ERROR = 4,
	AK8963_NO_DATA_ERROR = 5,
	AK8963_OVERFLOW_ERROR = 6
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

enum mpu9250_mag_range {
	MPU9250_MAG_RANGE_14BITS = 0x00,
	MPU9250_MAG_RANGE_16BITS = 0x01
} typedef mpu9250_mag_range_t;

struct mpu9250_config {
	uint8_t mpu9250_chip_addr;
	uint8_t ak8963_chip_addr;
	mpu9250_accel_range_t accel_range;
	mpu9250_gyro_range_t gyro_range;
	mpu9250_mag_range_t mag_range;
	ak8963_cntl1_mode_t mag_mode;
} typedef mpu9250_config_t;

struct mpu9250_calibration {
	// Resolution
	float accel_res;
	float gyro_res;
	float mag_res;

	// Biases
	float gyro_bias[3];
	float accel_bias[3];
	float mag_bias[3];
	
	// Magnetometer-specific
	float mag_scale[3];
	float factory_mag_calibration[3];
} typedef mpu9250_calibration_t;

mpu9250_return_code_t mpu9250_init(const mpu9250_config_t* config, mpu9250_calibration_t* calibration);
mpu9250_return_code_t mpu9250_self_test(const mpu9250_config_t* config, int16_t* dest);
mpu9250_return_code_t mpu9250_calibrate(const mpu9250_config_t* config, mpu9250_calibration_t* calibration);
mpu9250_return_code_t mpu9250_read_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* dest);
mpu9250_return_code_t mpu9250_write_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data);
mpu9250_return_code_t mpu9250_write_register_bytes(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* data);
mpu9250_return_code_t mpu9250_read_gyro(const mpu9250_config_t* config, int16_t* dest);
mpu9250_return_code_t mpu9250_read_gyro_float(const mpu9250_config_t* config, const mpu9250_calibration_t* calibration, float* dest);
mpu9250_return_code_t mpu9250_read_accel(const mpu9250_config_t* config, int16_t* dest);
mpu9250_return_code_t mpu9250_read_accel_float(const mpu9250_config_t* config, const mpu9250_calibration_t* calibration, float* dest);
mpu9250_return_code_t mpu9250_read_mag(const mpu9250_config_t* config, int16_t* dest);
mpu9250_return_code_t mpu9250_read_mag_float(const mpu9250_config_t* config, const mpu9250_calibration_t* calibration, float* dest);
uint8_t mpu9250_whoami(const mpu9250_config_t* config);

mpu9250_return_code_t ak8963_init(const mpu9250_config_t* config, mpu9250_calibration_t* calibration);
mpu9250_return_code_t ak8963_calibrate(const mpu9250_config_t* config, mpu9250_calibration_t* calibration);
mpu9250_return_code_t ak8963_read_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* dest);
mpu9250_return_code_t ak8963_write_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data);
mpu9250_return_code_t ak8963_write_register_bytes(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* data);
uint8_t ak8963_whoami(const mpu9250_config_t* config);

mpu9250_return_code_t mpu9250_madgwick_quaternion(const mpu9250_config_t* config, float* q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float delta_t);
mpu9250_return_code_t mpu9250_mahony_quaternion(const mpu9250_config_t* config, float* q, float* feedback, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float delta_t);

mpu9250_return_code_t mpu9250_write_register_ak8963(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data);
mpu9250_return_code_t mpu9250_read_register_ak8963(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* dest);


float pca9685_inv_sqrt(float x);

#endif /* MPU9250_IMU_H_ */