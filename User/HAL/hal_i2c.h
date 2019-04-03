/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    hal_i2c.h
  */

#ifndef __HAL_I2C_H_
#define __HAL_I2C_H_

#define	I2C_ACK_TRIES		5000U
#define	I2C_BUSY_TRIES		5000U
#define	I2C_ACK_TRIES_MAX	10000U

#ifdef __cplusplus

class HAL_I2C
{
public:
	uint32_t err_count;
	void init(void);
	bool write_multi_bytes(uint8_t slave_addr, uint8_t write_addr, uint8_t len, uint8_t *data);
	bool read_multi_bytes(uint8_t slave_addr, uint8_t read_addr, uint8_t len, uint8_t *data);
private:
	bool wait_for_event(uint32_t event);
	bool check_bus_busy(void);
};

extern HAL_I2C hal_i2c;
#endif

#endif

