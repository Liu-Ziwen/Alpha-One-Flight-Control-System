/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    hal_spi.h
  */

#ifndef __HAL_SPI_H_
#define __HAL_SPI_H_

#ifdef __cplusplus

class HAL_SPI
{
public:
	void init(uint16_t baud);
	uint8_t rw_byte(uint8_t txdata);
	void set_reg(uint8_t addr, uint8_t data);
	uint8_t read_one_reg(uint8_t addr);
	void read_multi_reg(uint8_t addr, uint8_t len, uint8_t *data);
};

extern HAL_SPI hal_spi;
#endif

#endif
