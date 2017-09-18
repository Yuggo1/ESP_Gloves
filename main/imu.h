/*
 * imu.h
 *
 *  Created on: 11 de set de 2017
 *      Author: yuggo
 */

#ifndef MAIN_IMU_H_
#define MAIN_IMU_H_
#include "bno055.h"

void i2c_master_init();

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);

void BNO055_delay_msek(u32 msek);

void imu_init();



#endif /* MAIN_IMU_H_ */
