/*
 * mpu9250_imu.c
 *
 * Created: 11/10/2017 1:07:29 AM
 *  Author: Alex Bennett
 */ 

#include <asf.h>
#include "mpu9250_imu.h"

mpu9250_return_code_t mpu9250_init(const mpu9250_config_t* config, mpu9250_calibration_t* calibration)
{	
	// Wake device
	if(mpu9250_write_register(config, MPU9250_PWR_MGMT_1, 0x00) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Wait for full startup
	delay_ms(100);
	
	// Select clock source
	if(mpu9250_write_register(config, MPU9250_PWR_MGMT_1, (1 << PWR_MGMT_1_CLKSEL)) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Configure device
	if(mpu9250_write_register(config, MPU9250_CONFIG, 0x03) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Set sample rate
	if(mpu9250_write_register(config, MPU9250_SMPLRT_DIV, 0x04) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Set gyroscope range
	uint8_t buf[1] = { 0 };
	if(mpu9250_read_register(config, MPU9250_GYRO_CONFIG, 1, buf) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	buf[0] = buf[0] & ~0x02; // Clear FCHOICE
	buf[0] = buf[0] & ~0x18; // Clear AFS bits
	buf[0] = buf[0] | (MPU9250_GYRO_RANGE_2000DPS << 3);
	if(mpu9250_write_register(config, MPU9250_GYRO_CONFIG, buf[0]) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;	
	
	// Set accelerometer range
	if(mpu9250_read_register(config, MPU9250_ACCEL_CONFIG, 1, buf) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	buf[0] = buf[0] & ~0x18; // Clear AFS bits
	buf[0] = buf[0] | (MPU9250_ACCEL_RANGE_4G << 3);
	if(mpu9250_write_register(config, MPU9250_ACCEL_CONFIG, buf[0]) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
		
	// Set accelerometer sample rate
	if(mpu9250_read_register(config, MPU9250_ACCEL_CONFIG_2, 1, buf) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	buf[0] = buf[0] & ~0x0F; // Clear AFS bits
	buf[0] = buf[0] | 0x03;
	if(mpu9250_write_register(config, MPU9250_ACCEL_CONFIG_2, buf[0]) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
		
	// If all went well, return success
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_write_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data)
{
	// Create packet
	twihs_packet_t send_packet;
	
	// Set chip address
	send_packet.chip = config->mpu9250_chip_addr;
	
	// Setup register address
	send_packet.addr[0] = reg_addr;
	send_packet.addr_length = 1;
	
	// Setup data
	uint8_t buf[1] = { data };
	send_packet.buffer = buf;
	send_packet.length = 1;
	
	// Send the packet
	if(twihs_master_write(BOARD_BASE_TWIHS_EEPROM, &send_packet) != TWIHS_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_read_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* dest)
{
	// Create packet
	twihs_packet_t receive_packet;
	
	// Set chip address
	receive_packet.chip = config->mpu9250_chip_addr;
	
	// Setup register address
	receive_packet.addr[0] = reg_addr;
	receive_packet.addr_length = 1;
	
	// Setup data return buffer
	receive_packet.buffer = dest;
	receive_packet.length = num_bytes;
	
	// Send the packet
	if(twihs_master_read(BOARD_BASE_TWIHS_EEPROM, &receive_packet) != TWIHS_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	return MPU9250_SUCCESS;
}

uint8_t mpu9250_whoami(const mpu9250_config_t* config)
{
	// Create return buffer
	uint8_t buf[1] = { 0 };
	
	// Read register
	if(mpu9250_read_register(config, MPU9250_WHO_AM_I, sizeof(buf), buf) != MPU9250_SUCCESS)
	{
		return 0xFF;
	}
	
	// Return WHO_AM_I byte
	return buf[0];
}

mpu9250_return_code_t mpu9250_write_register_ak8963(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data)
{
	// Return buffer for future verification
	uint8_t buf[1] = { 0 };
	
	// Set slave 0 of the MPU I2C bus to the on-board AK8963 magnetometer address
	if(mpu9250_write_register(config, MPU9250_I2C_SLV0_ADDR, (AK8963_I2C_ADDR << I2C_ID)) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	// Set the desired AK8963 address
	if(mpu9250_write_register(config, MPU9250_I2C_SLV0_REG, reg_addr) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	// Store data to be written
	if(mpu9250_write_register(config, MPU9250_I2C_SLV0_DO, data) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	// Enable I2C and send data
	if(mpu9250_write_register(config, MPU9250_I2C_SLV0_CTRL, (1 << I2C_SLV_EN) | 1) != MPU9250_SUCCESS) // The ORed "1" represents a length of 1 byte sent
	{
		return MPU9250_RW_ERROR;
	} 
	
	// Read register to confirm written data
	mpu9250_read_register_ak8963(config, reg_addr, sizeof(buf), buf);
	
	// Validate data
	if(buf[0] == data)
	{
		return MPU9250_SUCCESS;
	}
	else
	{
		return MPU9250_RW_ERROR;
	}
}

mpu9250_return_code_t mpu9250_read_register_ak8963(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* dest)
{	
	// Set slave 0 of the MPU I2C bus and OR read flag
	if(mpu9250_write_register(config, MPU9250_I2C_SLV0_ADDR, (AK8963_I2C_ADDR << I2C_ID) | (1 << I2C_SLV_RNW)) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	// Set the desired AK8963 address
	if(mpu9250_write_register(config, MPU9250_I2C_SLV0_REG, reg_addr) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	// Enable I2C and request bytes
	if(mpu9250_write_register(config, MPU9250_I2C_SLV0_CTRL, (1 << I2C_SLV_EN) | (num_bytes << I2C_SLV_LENG)) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	// Delay to allow registers to fill
	delay_us(100);
	
	// Read the registers
	if(mpu9250_read_register(config, MPU9250_EXT_SENS_DATA_00, num_bytes, dest) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	return MPU9250_SUCCESS;
}

uint8_t mpu9250_whoami_ak8963(const mpu9250_config_t* config)
{
	// Create return buffer
	uint8_t buf[1] = { 0 };
	
	// Read register
	if(mpu9250_read_register_ak8963(config, AK8963_WIA, sizeof(buf), buf) != MPU9250_SUCCESS)
	{
		return 0xFF;
	}
	
	// Return WHO_AM_I byte
	return buf[0];
}

mpu9250_return_code_t mpu9250_read_gyro(const mpu9250_config_t* config, int16_t* dest)
{
	// Create buffer for raw data
	uint8_t buf[6] = { 0 };
	
	// Read the bytes
	if(mpu9250_read_register(config, MPU9250_GYRO_XOUT_H, sizeof(buf), buf) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	else
	{
		dest[0] = (int16_t) (((int16_t) buf[0] << 8) | buf[1]);
		dest[1] = (int16_t) (((int16_t) buf[2] << 8) | buf[3]);
		dest[2] = (int16_t) (((int16_t) buf[4] << 8) | buf[5]);
	}
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_read_accel(const mpu9250_config_t* config, int16_t* dest)
{
	// Create buffer for raw data
	uint8_t buf[6] = { 0 };
	
	// Read the bytes
	if(mpu9250_read_register(config, MPU9250_ACCEL_XOUT_H, sizeof(buf), buf) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	else
	{
		dest[0] = (int16_t) (((int16_t) buf[0] << 8) | buf[1]);
		dest[1] = (int16_t) (((int16_t) buf[2] << 8) | buf[3]);
		dest[2] = (int16_t) (((int16_t) buf[4] << 8) | buf[5]);
	}
	
	return MPU9250_SUCCESS;
}