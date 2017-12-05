/*
 * mpu9250_imu.c
 *
 * Created: 11/10/2017 1:07:29 AM
 *  Author: Alex Bennett
 */ 

#include <asf.h>
#include <math.h>
#include "mpu9250_imu.h"

mpu9250_return_code_t mpu9250_init(const mpu9250_config_t* config, mpu9250_calibration_t* calibration)
{	
	uint8_t buf[1] = { 0 };

	// Wake device
	if(mpu9250_write_register(config, MPU9250_PWR_MGMT_1, 0x00) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Wait for full startup
	delay_ms(100);
	
	// Select clock source
	if(mpu9250_write_register(config, MPU9250_PWR_MGMT_1, (1 << PWR_MGMT_1_CLKSEL)) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Wait for clock selection
	delay_ms(200);
		
	// Configure device
	if(mpu9250_write_register(config, MPU9250_CONFIG, 0x03) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Set sample rate
	if(mpu9250_write_register(config, MPU9250_SMPLRT_DIV, 0x04) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Set gyroscope range
	if(mpu9250_read_register(config, MPU9250_GYRO_CONFIG, 1, buf) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	//buf[0] = buf[0] & ~0xE0; // Clear self-test
	buf[0] = buf[0] & ~0x02; // Clear FCHOICE
	buf[0] = buf[0] & ~0x18; // Clear GFS bits
	buf[0] = buf[0] | (config->gyro_range << 3);
	if(mpu9250_write_register(config, MPU9250_GYRO_CONFIG, buf[0]) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
			
	// Set accelerometer range
	if(mpu9250_read_register(config, MPU9250_ACCEL_CONFIG, 1, buf) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	//buf[0] = buf[0] & ~0xE0; // Clear self-test
	buf[0] = buf[0] & ~0x18; // Clear AFS bits
	buf[0] = buf[0] | (config->accel_range << 3);
	if(mpu9250_write_register(config, MPU9250_ACCEL_CONFIG, buf[0]) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;

	// Set accelerometer sample rate
	if(mpu9250_read_register(config, MPU9250_ACCEL_CONFIG_2, 1, buf) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	buf[0] = buf[0] & ~0x0F; // Clear AFS bits
	buf[0] = buf[0] | 0x03;
	if(mpu9250_write_register(config, MPU9250_ACCEL_CONFIG_2, buf[0]) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Setup interrupts
	if(mpu9250_write_register(config, MPU9250_INT_PIN_CFG, 0x22) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	if(mpu9250_write_register(config, MPU9250_INT_ENABLE, 0x01) != MPU9250_SUCCESS) return MPU9250_INIT_ERROR;
	
	// Allow changes to propagate
	delay_ms(100);
		
	// Store accelerometer resolution
	switch(config->accel_range)
	{
		case MPU9250_ACCEL_RANGE_2G:
			calibration->accel_res = 2.0 / 32768.0;
			break;
		case MPU9250_ACCEL_RANGE_4G:
			calibration->accel_res = 4.0 / 32768.0;
			break;
		case MPU9250_ACCEL_RANGE_8G:
			calibration->accel_res = 8.0 / 32768.0;
			break;
		case MPU9250_ACCEL_RANGE_16G:	
			calibration->accel_res = 16.0 / 32768.0;
			break;
	}

	// Store gyroscope resolution
	switch(config->gyro_range)
	{
		case MPU9250_GYRO_RANGE_250DPS:
			calibration->gyro_res = 250.0 / 32768.0;
			break;
		case MPU9250_GYRO_RANGE_500DPS:
			calibration->gyro_res = 500.0 / 32768.0;
			break;
		case MPU9250_GYRO_RANGE_1000DPS:
			calibration->gyro_res = 1000.0 / 32768.0;
			break;
		case MPU9250_GYRO_RANGE_2000DPS:
			calibration->gyro_res = 2000.0 / 32768.0;
			break;	
	}
	
	// Ensure everything is ready
	delay_ms(100);
		
	// If all went well, return success
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t ak8963_init(const mpu9250_config_t* config, mpu9250_calibration_t* calibration)
{
	// Buffer
	uint8_t buf[3];
	
	// Power down
	if(ak8963_write_register(config, AK8963_CNTL1, 0x00) != MPU9250_SUCCESS) return AK8963_CALIBRATION_ERROR;
	delay_ms(10);
    	
	// Enter Fuse ROM access mode
	if(ak8963_write_register(config, AK8963_CNTL1, AK8963_FSE_ROM << AK8963_MODE) != MPU9250_SUCCESS) return AK8963_CALIBRATION_ERROR;
	delay_ms(10);
    	
	// Read calibration values
	if(ak8963_read_register(config, AK8963_ASAX, 3, buf) != MPU9250_SUCCESS) return AK8963_CALIBRATION_ERROR;
	
	// Store calibration values
	calibration->factory_mag_calibration[0] = (float) (buf[0] - 128) / 256.0f + 1.0f;
	calibration->factory_mag_calibration[1] = (float) (buf[1] - 128) / 256.0f + 1.0f;
	calibration->factory_mag_calibration[2] = (float) (buf[2] - 128) / 256.0f + 1.0f;
	
	// Power down
	if(ak8963_write_register(config, AK8963_CNTL1, 0x00) != MPU9250_SUCCESS) return AK8963_CALIBRATION_ERROR;
	delay_ms(10);
    	
	// Configure magnetometer
	if(ak8963_write_register(config, AK8963_CNTL1, (config->mag_range << AK8963_BIT) | config->mag_mode) != MPU9250_SUCCESS) return AK8963_CALIBRATION_ERROR;
	delay_ms(10);
        
    // Store accelerometer resolution
    switch(config->gyro_range)
    {
        case MPU9250_MAG_RANGE_14BITS:
            calibration->mag_res = 10.0f * 4912.0f / 8190.0f;
            break;
        case MPU9250_MAG_RANGE_16BITS:
            calibration->mag_res = 10.0f * 4912.0f / 32760.0f;
            break;
        default:
            break;
    }
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_self_test(const mpu9250_config_t* config, int16_t* dest)
{
	// Buffers
	int16_t gyro_buf[3] = { 0 };
	int16_t accel_buf[3] = { 0 };
		
	// Averages
	int32_t gyro_avg[3] = { 0 };
	int32_t gyro_st_avg[3] = { 0 };
	int32_t accel_avg[3] = { 0 };
	int32_t accel_st_avg[3] = { 0 };

	// Self-test register buffer
	uint8_t self_test_buf[6] = { 0 };
	
	// Factory trim information
	float factory_trim[6] = { 0 };

	// Set 1 kHz sample rate
	mpu9250_write_register(config, MPU9250_SMPLRT_DIV, 0x00);
	
	// Set DLPF
	mpu9250_write_register(config, MPU9250_CONFIG, 0x02);
	
	// Set gyro range
	mpu9250_write_register(config, MPU9250_GYRO_CONFIG, MPU9250_GYRO_RANGE_250DPS << GYRO_CONFIG_GYRO_FS_SEL);
	
	// Set accel range
	mpu9250_write_register(config, MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_RANGE_2G << ACCEL_CONFIG_ACCEL_FS_SEL);
	
	// Set accel rate and bandwidth (0x02 = 92 Hz / 1 kHz)
	mpu9250_write_register(config, MPU9250_ACCEL_CONFIG_2, 0x02 << ACCEL_CONFIG_2_A_DLPFCFG);
	
	// Collect 200 samples
	for(uint8_t i = 0; i < 200; i++)
	{
		// Read data
		mpu9250_read_gyro(config, gyro_buf);
		mpu9250_read_accel(config, accel_buf);

		// Add gyro to average
		gyro_avg[0] += gyro_buf[0];
		gyro_avg[1] += gyro_buf[1];
		gyro_avg[2] += gyro_buf[2];
				
		// Add accel to average
		accel_avg[0] += accel_buf[0];
		accel_avg[1] += accel_buf[1];
		accel_avg[2] += accel_buf[2];
	}
	
	// Average the values
	for(uint8_t i = 0; i < 3; i++)
	{
		// Gyro
		gyro_avg[i] /= 200;
		
		// Accelerometer
		accel_avg[i] /= 200;
	}
	
	// Configure devices to for self-test
	mpu9250_write_register(config, MPU9250_GYRO_CONFIG, (1 << GYRO_CONFIG_ZGYRO_CTEN) | (1 << GYRO_CONFIG_YGYRO_CTEN) | (1 << GYRO_CONFIG_XGYRO_CTEN));
	mpu9250_write_register(config, MPU9250_ACCEL_CONFIG, (1 << ACCEL_CONFIG_AZ_ST_EN) | (1 << ACCEL_CONFIG_AY_ST_EN) | (1 << ACCEL_CONFIG_AX_ST_EN));
	
	// Wait for stabilization
	delay_ms(25);
	
	// Self-test complete, collect 200 samples
	for(uint8_t i = 0; i < 200; i++)
	{
		// Read data
		mpu9250_read_gyro(config, gyro_buf);
		mpu9250_read_accel(config, accel_buf);

		// Add gyro to average
		gyro_st_avg[0] += gyro_buf[0];
		gyro_st_avg[1] += gyro_buf[1];
		gyro_st_avg[2] += gyro_buf[2];
			
		// Add accel to average
		accel_st_avg[0] += accel_buf[0];
		accel_st_avg[1] += accel_buf[1];
		accel_st_avg[2] += accel_buf[2];
	}
	
	// Average the values
	for(uint8_t i = 0; i < 3; i++)
	{
		// Gyro
		gyro_st_avg[i] /= 200;
		
		// Accelerometer
		accel_st_avg[i] /= 200;
	}
	
	// Configure devices to for normal operation
	mpu9250_write_register(config, MPU9250_GYRO_CONFIG, 0x00);
	mpu9250_write_register(config, MPU9250_ACCEL_CONFIG, 0x00);
	
	// Wait for stabilization
	delay_ms(25);
	
	// Get factory self-test values
	mpu9250_read_register(config, MPU9250_SELF_TEST_X_GYRO, 3, self_test_buf);
	mpu9250_read_register(config, MPU9250_SELF_TEST_X_ACCEL, 3, self_test_buf + 3);
	
	// Calculate trim
	for(uint8_t i = 0; i < 6; i++)
	{
		factory_trim[i] = (float) (2620.0f / 1.0f) * (pow(1.01, ((float) self_test_buf[i] - 1.0)));
	}
	
	// Write results to destination
	for(uint8_t i = 0; i < 3; i++)
	{
		dest[i] = 100.0f * ((float) (gyro_st_avg[i] - gyro_avg[i])) / factory_trim[i] - 100.0f;
		dest[i + 3] = 100.0f * ((float) (accel_st_avg[i] - accel_avg[i])) / factory_trim[i + 3] - 100.0f;
	}
}

// TODO: Error handling
mpu9250_return_code_t mpu9250_calibrate(const mpu9250_config_t* config, mpu9250_calibration_t* calibration)
{
	// Control variables
	uint16_t packet_count;
	uint16_t fifo_count;
	uint8_t fifo_buf[12];

	// Biases
	int32_t gyro_bias[3] = { 0 };
	int32_t accel_bias[3] = { 0 };
		
	// Reset the device
	mpu9250_write_register(config, MPU9250_PWR_MGMT_1, (1 << PWR_MGMT_1_H_RESET));
	
	// Wait for reset
	delay_ms(200);
	
	// Configure chip for bias calculation
	mpu9250_write_register(config, MPU9250_INT_ENABLE, 0x00); // disable interrupts
	mpu9250_write_register(config, MPU9250_FIFO_EN, 0x00); // disable FIFO
	mpu9250_write_register(config, MPU9250_PWR_MGMT_1, 0x00); // turn on internal clock
	mpu9250_write_register(config, MPU9250_I2C_MST_CTRL, 0x00); // disable I2C
	mpu9250_write_register(config, MPU9250_USER_CTRL, 0x00); // disable FIFO and I2C master modes
	mpu9250_write_register(config, MPU9250_USER_CTRL, (1 << FIFO_RST)); // reset FIFO
	
	// Wait for completion
	delay_ms(15);
	
	// Configure MEMS for bias calculation
	mpu9250_write_register(config, MPU9250_CONFIG, (1 << DLPF_CFG)); // set DLPF to 188 Hz
	mpu9250_write_register(config, MPU9250_SMPLRT_DIV, 0x00); // set sample rate to 1 kHz
	
	// Set gyro range
	mpu9250_write_register(config, MPU9250_GYRO_CONFIG, MPU9250_GYRO_RANGE_250DPS << GYRO_CONFIG_GYRO_FS_SEL);
		
	// Set accel range
	mpu9250_write_register(config, MPU9250_ACCEL_CONFIG, MPU9250_ACCEL_RANGE_2G << ACCEL_CONFIG_ACCEL_FS_SEL);
	
	uint16_t gyro_sensitivity = 131; // 131 LSB/deg/s
	uint16_t accel_sensitivity = 16384; // 131 LSB/deg/s
	
	// Enable FIFO
	mpu9250_write_register(config, MPU9250_USER_CTRL, (1 << FIFO_EN));
	
	// Configure FIFO to receive gyro and accel information
	mpu9250_write_register(config, MPU9250_FIFO_EN, 0x78);
	
	// Wait for samples
	delay_ms(40); // ~480 bytes
	
	// Disable FIFO
	mpu9250_write_register(config, MPU9250_FIFO_EN, 0x00);
	
	// 2 byte buffer
	uint8_t buf[2] = { 0 };
		
	// Read FIFO sample count
	mpu9250_read_register(config, MPU9250_FIFO_COUNTH, 2, buf);
	
	// Combine bytes
	fifo_count = ((uint16_t) buf[0] << 8) | buf[1];
	
	// Number of packets (12 bytes per packet)
	packet_count = fifo_count / 12;
	
	for(uint16_t i = 0; i < packet_count; i++)
	{
		// Create buffers
		int16_t fifo_gyro_buf[3] = { 0 };
		int16_t fifo_accel_buf[3] = { 0 };
			
		// Get FIFO data
		mpu9250_read_register(config, MPU9250_FIFO_R_W, 12, fifo_buf);
		
		// Set to temporary buffers
		fifo_accel_buf[0] = (int16_t) (((int16_t) fifo_buf[0] << 8) | fifo_buf[1]);
		fifo_accel_buf[1] = (int16_t) (((int16_t) fifo_buf[2] << 8) | fifo_buf[3]);
		fifo_accel_buf[2] = (int16_t) (((int16_t) fifo_buf[4] << 8) | fifo_buf[5]);
		fifo_gyro_buf[0] = (int16_t) (((int16_t) fifo_buf[6] << 8) | fifo_buf[7]);
		fifo_gyro_buf[0] = (int16_t) (((int16_t) fifo_buf[8] << 8) | fifo_buf[9]);
		fifo_gyro_buf[0] = (int16_t) (((int16_t) fifo_buf[10] << 8) | fifo_buf[11]);
		
		// Sum biases into bias arrays
		accel_bias[0] += (int32_t) fifo_accel_buf[0];
		accel_bias[1] += (int32_t) fifo_accel_buf[1];
		accel_bias[2] += (int32_t) fifo_accel_buf[2];
		gyro_bias[0] += (int32_t) fifo_gyro_buf[0];
		gyro_bias[1] += (int32_t) fifo_gyro_buf[1];
		gyro_bias[2] += (int32_t) fifo_gyro_buf[2];
	}
	
	// Normalize
	for(uint8_t i = 0; i < 3; i++)
	{
		accel_bias[i] /= (int32_t) packet_count;
		gyro_bias[i] /= (int32_t) packet_count;
	}
	
	// Remove gravity
	if(accel_bias[2] > 0L)
	{
		accel_bias[2] -= (int32_t) accel_sensitivity;
	}
	else
	{
		accel_bias[2] += (int32_t) accel_sensitivity;
	}
	
	// Construct gyro biases for writing to device
	fifo_buf[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
	fifo_buf[1] = (-gyro_bias[0] / 4) & 0xFF;
	fifo_buf[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	fifo_buf[3] = (-gyro_bias[1] / 4) & 0xFF;
	fifo_buf[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	fifo_buf[5] = (-gyro_bias[2] / 4) & 0xFF;
	
	// Write to device
	mpu9250_write_register_bytes(config, MPU9250_XG_OFFSET_H, 6, fifo_buf);
	
	// Store to calibration struct
	calibration->gyro_bias[0] = (float) gyro_bias[0] / (float) gyro_sensitivity;
	calibration->gyro_bias[1] = (float) gyro_bias[1] / (float) gyro_sensitivity;
	calibration->gyro_bias[2] = (float) gyro_bias[2] / (float) gyro_sensitivity;
	
	// Grab factory accel biases
	mpu9250_read_register(config, MPU9250_XA_OFFSET_H, 6, fifo_buf);
	
	// Convert to actual biases
	int32_t factory_accel_bias[3] = { 0 };
	factory_accel_bias[0] = (int16_t) (((int16_t) fifo_buf[0] << 8) | fifo_buf[1]);
	factory_accel_bias[1] = (int16_t) (((int16_t) fifo_buf[2] << 8) | fifo_buf[3]);
	factory_accel_bias[2] = (int16_t) (((int16_t) fifo_buf[4] << 8) | fifo_buf[5]);
	
	// Temperature compensation mask
	uint8_t mask_bit[3] = { 0 };
		
	// Get temperature compensation bit
	for(uint8_t i = 0; i < 3; i++)
	{
		if(factory_accel_bias[i] & (uint32_t) 1ul)
		{
			mask_bit[i] = 1;
		}
	}
	
	// Construct accel biases
	for(uint8_t i = 0; i < 3; i++)
	{
		factory_accel_bias[i] -= accel_bias[i] / 8;
	}

	// Setup data for writing to device
	fifo_buf[0] = (factory_accel_bias[0] >> 8) & 0xFF;
	fifo_buf[1] = factory_accel_bias[0] & 0xFF;
	fifo_buf[1] = fifo_buf[1] | mask_bit[0];
	fifo_buf[2] = (factory_accel_bias[1] >> 8) & 0xFF;
	fifo_buf[3] = factory_accel_bias[1] & 0xFF;
	fifo_buf[3] = fifo_buf[3] | mask_bit[1];
	fifo_buf[4] = (factory_accel_bias[2] >> 8) & 0xFF;
	fifo_buf[5] = factory_accel_bias[2] & 0xFF;
	fifo_buf[5] = fifo_buf[5] | mask_bit[2];
	
	// Write to device
	//mpu9250_write_register_bytes(config, MPU9250_XA_OFFSET_H, 6, fifo_buf);
	
	// Store to calibration struct
	calibration->accel_bias[0] = (float) accel_bias[0] / (float) accel_sensitivity;
	calibration->accel_bias[1] = (float) accel_bias[1] / (float) accel_sensitivity;
	calibration->accel_bias[2] = (float) accel_bias[2] / (float) accel_sensitivity;
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t ak8963_calibrate(const mpu9250_config_t* config, mpu9250_calibration_t* calibration)
{
	uint16_t sample_count = 0;
	int32_t mag_bias[3];
	int32_t mag_scale[3];
	int16_t mag_max[3] = { 0x8000 };
	int16_t mag_min[3] = { 0x7FFF };
	int16_t mag_buf[3] = { 0 };
		
	// Determine samples required for ~15 seconds
	switch(config->mag_mode)
	{
		case AK8963_CNT_MEAS1:
			sample_count = 128;
			break;
		case AK8963_CNT_MEAS2:
			sample_count = 1500;
			break;
		default:
			break;
	}
    
    // Grab samples
    for(uint16_t i = 0; i < sample_count; i++)
    {
        // Read data
        mpu9250_read_mag(config, mag_buf);
        
        // Record values
        for(uint8_t j = 0; j < 3; j++)
        {
            if(mag_buf[j] > mag_max[j]) mag_max[j] = mag_buf[j];
            if(mag_buf[j] < mag_min[j]) mag_max[j] = mag_buf[j];
        }
        
        // Delay appropriate amount to achieve desired number of samples
        switch(config->mag_mode)
        {
            case AK8963_CNT_MEAS1:
                delay_ms(135);
                break;
            case AK8963_CNT_MEAS2:
                delay_ms(12);
                break;
            default:
                break;
        }
    }
    
    // Get hard iron corrections
    for(uint8_t i = 0; i < 3; i++)
    {
        mag_bias[i] = (float) (mag_max[i] + mag_min[i]) / 2.0f;
    }
    
    // Save mag biases
    for(uint8_t i = 0; i < 3; i++)
    {
        calibration->mag_bias[i] = (float) mag_bias[i] * calibration->mag_res * calibration->factory_mag_calibration[i];
    }
    
    // Get soft iron corrections
    for(uint8_t i = 0; i < 3; i++)
    {
        mag_scale[i] = (float) (mag_max[i] - mag_min[i]) / 2.0f;
    }
    
    // Find average
    float avg = (mag_scale[0] + mag_scale[1] + mag_scale[2]) / 3.0f;
    
    // Save mag axis scaling
    for(uint8_t i = 0; i < 3; i++)
    {
        calibration->mag_scale[i] = avg / (float) mag_scale[i];
    }
    
    // Done
    return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_write_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data)
{
	// Create data buffer
	uint8_t buf[1] = { data };
	
	// Forward to write register bytes
	return mpu9250_write_register_bytes(config, reg_addr, 1, buf);
}

mpu9250_return_code_t mpu9250_write_register_bytes(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* data)
{
	// Create packet
	twihs_packet_t send_packet;
	
	// Set chip address
	send_packet.chip = config->mpu9250_chip_addr;
	
	// Setup register address
	send_packet.addr[0] = reg_addr;
	send_packet.addr_length = 1;
	
	// Setup data
	send_packet.buffer = data;
	send_packet.length = num_bytes;
	
	// Send the packet
	if(twihs_master_write(BOARD_BASE_TWIHS, &send_packet) != TWIHS_SUCCESS)
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
	if(twihs_master_read(BOARD_BASE_TWIHS, &receive_packet) != TWIHS_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t ak8963_write_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t data)
{
    // Create data buffer
    uint8_t buf[1] = { data };
    
    // Forward to write register bytes
    return ak8963_write_register_bytes(config, reg_addr, 1, buf);
}

mpu9250_return_code_t ak8963_write_register_bytes(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* data)
{
    // Create packet
    twihs_packet_t send_packet;
    	
    // Set chip address
    send_packet.chip = config->ak8963_chip_addr;
    	
    // Setup register address
    send_packet.addr[0] = reg_addr;
    send_packet.addr_length = 1;
    	
    // Setup data
    send_packet.buffer = data;
    send_packet.length = num_bytes;
    	
    // Send the packet
    if(twihs_master_write(BOARD_BASE_TWIHS, &send_packet) != TWIHS_SUCCESS)
    {
        return AK8963_RW_ERROR;
    }
    	
    return MPU9250_SUCCESS;
}

mpu9250_return_code_t ak8963_read_register(const mpu9250_config_t* config, uint8_t reg_addr, uint8_t num_bytes, uint8_t* dest)
{
    // Create packet
    twihs_packet_t receive_packet;
    
    // Set chip address
    receive_packet.chip = config->ak8963_chip_addr;
    
    // Setup register address
    receive_packet.addr[0] = reg_addr;
    receive_packet.addr_length = 1;
    
    // Setup data return buffer
    receive_packet.buffer = dest;
    receive_packet.length = num_bytes;
    
    // Send the packet
    if(twihs_master_read(BOARD_BASE_TWIHS, &receive_packet) != TWIHS_SUCCESS)
    {
        return MPU9250_RW_ERROR;
    }
    
    return MPU9250_SUCCESS;
}

/**
 * /deprecated
 */
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

/**
 * /deprecated
 */
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

uint8_t ak8963_whoami(const mpu9250_config_t* config)
{
	// Create return buffer
	uint8_t buf[1] = { 0 };
	
	// Read register
	if(ak8963_read_register(config, AK8963_WIA, sizeof(buf), buf) != MPU9250_SUCCESS)
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
	if(mpu9250_read_register(config, MPU9250_GYRO_XOUT_H, 6, buf) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	else
	{
		// Byte order is HIGH then LOW for each measurement
		dest[0] = (int16_t) (((int16_t) buf[0] << 8) | buf[1]);
		dest[1] = (int16_t) (((int16_t) buf[2] << 8) | buf[3]);
		dest[2] = (int16_t) (((int16_t) buf[4] << 8) | buf[5]);
	}
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_read_gyro_float(const mpu9250_config_t* config, const mpu9250_calibration_t* calibration, float* dest)
{	
	// Create raw gyro data array
	int16_t gyro_raw[3] = { 0 };
		
	// Grab raw gyro data
	if(mpu9250_read_gyro(config, gyro_raw) != MPU9250_SUCCESS) return MPU9250_RW_ERROR;
	
	// Convert to float and store
	dest[0] = (float) gyro_raw[0] * calibration->gyro_res;
	dest[1] = (float) gyro_raw[1] * calibration->gyro_res;
	dest[2] = (float) gyro_raw[2] * calibration->gyro_res;
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_read_accel(const mpu9250_config_t* config, int16_t* dest)
{
	// Create buffer for raw data
	uint8_t buf[6] = { 0 };
	
	// Read the bytes
	if(mpu9250_read_register(config, MPU9250_ACCEL_XOUT_H, 6, buf) != MPU9250_SUCCESS)
	{
		return MPU9250_RW_ERROR;
	}
	else
	{
		// Byte order is HIGH then LOW for each measurement
		dest[0] = (int16_t) (((int16_t) buf[0] << 8) | buf[1]);
		dest[1] = (int16_t) (((int16_t) buf[2] << 8) | buf[3]);
		dest[2] = (int16_t) (((int16_t) buf[4] << 8) | buf[5]);
	}
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_read_accel_float(const mpu9250_config_t* config, const mpu9250_calibration_t* calibration, float* dest)
{
	// Create raw gyro data array
	int16_t raw_buf[3] = { 0 };
	
	// Grab raw gyro data
	if(mpu9250_read_accel(config, raw_buf) != MPU9250_SUCCESS) return MPU9250_RW_ERROR;
	
	// Convert to float and store
	dest[0] = (float) raw_buf[0] * calibration->accel_res;// - calibration->accel_bias[0];
	dest[1] = (float) raw_buf[1] * calibration->accel_res;// - calibration->accel_bias[1];
	dest[2] = (float) raw_buf[2] * calibration->accel_res;// - calibration->accel_bias[2];
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_read_mag(const mpu9250_config_t* config, int16_t* dest)
{
	// Create buffers
	uint8_t buf[1] = { 0 };
	uint8_t raw_buf[7] = { 0 };
	
	// Check for available magnetometer data
	if(ak8963_read_register(config, AK8963_ST1, 1, buf) != MPU9250_SUCCESS) return AK8963_RW_ERROR;

	// Is data available?
	if(buf[0] & 0x01)
	{
		// Grab the data
		if(ak8963_read_register(config, AK8963_HXL, 7, raw_buf) != MPU9250_SUCCESS) return AK8963_RW_ERROR;
		
		// Check for overflow
		if(!(raw_buf[6] & 0x08))
		{
			// Byte order is LOW then HIGH for each measurement
			dest[0] = (int16_t) (((int16_t) raw_buf[1] << 8) | raw_buf[0]);
			dest[1] = (int16_t) (((int16_t) raw_buf[3] << 8) | raw_buf[2]);
			dest[2] = (int16_t) (((int16_t) raw_buf[5] << 8) | raw_buf[4]);
		}
		else
		{
			// Overflow, discard data
			return AK8963_OVERFLOW_ERROR;
		}
	}
	else
	{
		return AK8963_NO_DATA_ERROR;
	}

	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_read_mag_float(const mpu9250_config_t* config, const mpu9250_calibration_t* calibration, float* dest)
{
    // Create raw gyro data array
    int16_t raw_buf[3] = { 0 };
    
    // Grab raw gyro data
    if(mpu9250_read_accel(config, raw_buf) != MPU9250_SUCCESS) return MPU9250_RW_ERROR;
    
    // Convert to float and store
    for(uint8_t i = 0; i < 3; i++)
    {
        dest[i] = ((float) raw_buf[i] * calibration->mag_res * calibration->factory_mag_calibration[i] - calibration->mag_bias[i]);// * calibration->mag_scale[i];        
    }
    
    return MPU9250_SUCCESS;
}

/**
 * @see http://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 */
mpu9250_return_code_t mpu9250_madgwick_quaternion(const mpu9250_config_t* config, float* q, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float delta_t)
{	
	// Define calculation constants
	const float pca9685_gyro_meas_error = M_PI * (40.0f / 180.0f); // 40 deg/s -> rad
	const float pca9685_madgwick_beta = sqrt(3.0f / 4.0f) * pca9685_gyro_meas_error;
	
	// Redefine for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalize accelerometer measurement
    norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalize magnetometer measurement
    norm = sqrtf(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrtf(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    
    // Normalize step magnitude
    norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - pca9685_madgwick_beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - pca9685_madgwick_beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - pca9685_madgwick_beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - pca9685_madgwick_beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * delta_t;
    q2 += qDot2 * delta_t;
    q3 += qDot3 * delta_t;
    q4 += qDot4 * delta_t;
    
    // Normalize quaternion
    norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
	
	return MPU9250_SUCCESS;
}

mpu9250_return_code_t mpu9250_mahony_quaternion(const mpu9250_config_t* config, float* q, float* feedback, float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float delta_t)
{
	// short name local variable for readability
	float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
	float norm;
	float hx, hy, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float pa, pb, pc;

	// Auxiliary variables to avoid repeated arithmetic
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q1q4 = q1 * q4;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q2q4 = q2 * q4;
	float q3q3 = q3 * q3;
	float q3q4 = q3 * q4;
	float q4q4 = q4 * q4;

	// Normalize accelerometer measurement
	norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return; // Handle NaN
	norm = 1.0f / norm;       // Use reciprocal for division
	ax *= norm;
	ay *= norm;
	az *= norm;

	// Normalize magnetometer measurement
	norm = sqrt(mx * mx + my * my + mz * mz);
	if (norm == 0.0f) return; // Handle NaN
	norm = 1.0f / norm;       // Use reciprocal for division
	mx *= norm;
	my *= norm;
	mz *= norm;

	// Reference direction of Earth's magnetic field
	hx = 2.0f * mx * (0.5f - q3q3 - q4q4) + 2.0f * my * (q2q3 - q1q4) + 2.0f * mz * (q2q4 + q1q3);
	hy = 2.0f * mx * (q2q3 + q1q4) + 2.0f * my * (0.5f - q2q2 - q4q4) + 2.0f * mz * (q3q4 - q1q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = 2.0f * mx * (q2q4 - q1q3) + 2.0f * my * (q3q4 + q1q2) + 2.0f * mz * (0.5f - q2q2 - q3q3);

	// Estimated direction of gravity and magnetic field
	vx = 2.0f * (q2q4 - q1q3);
	vy = 2.0f * (q1q2 + q3q4);
	vz = q1q1 - q2q2 - q3q3 + q4q4;
	wx = 2.0f * bx * (0.5f - q3q3 - q4q4) + 2.0f * bz * (q2q4 - q1q3);
	wy = 2.0f * bx * (q2q3 - q1q4) + 2.0f * bz * (q1q2 + q3q4);
	wz = 2.0f * bx * (q1q3 + q2q4) + 2.0f * bz * (0.5f - q2q2 - q3q3);

	// Error is cross product between estimated direction and measured direction of gravity
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	if (MPU9250_KI > 0.0f)
	{
    	feedback[0] += ex;      // accumulate integral error
    	feedback[1] += ey;
    	feedback[2] += ez;
	}
	else
	{
    	feedback[0] = 0.0f;     // prevent integral wind up
    	feedback[1] = 0.0f;
    	feedback[2] = 0.0f;
	}

	// Apply feedback terms
	gx = gx + MPU9250_KP * ex + MPU9250_KI * feedback[0];
	gy = gy + MPU9250_KP * ey + MPU9250_KI * feedback[1];
	gz = gz + MPU9250_KP * ez + MPU9250_KI * feedback[2];
	
	// Integrate rate of change of quaternion
	pa = q2;
	pb = q3;
	pc = q4;
	q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * delta_t);
	q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * delta_t);
	q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * delta_t);
	q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * delta_t);

	// Normalize quaternion
	norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	norm = 1.0f / norm;
	q[0] = q1 * norm;
	q[1] = q2 * norm;
	q[2] = q3 * norm;
	q[3] = q4 * norm;
}

float pca9685_inv_sqrt(float x) 
{
	 uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
	 float tmp = *(float*)&i;
	 return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}