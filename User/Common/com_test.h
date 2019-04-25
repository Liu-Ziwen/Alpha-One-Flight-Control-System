/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    12-April-2019
  * @file    com_test.h
  */ 

#ifndef __COM_TEST_H_
#define __COM_TEST_H_


extern bool test_flag;
extern uint8_t optical_flow_data[14];
bool optical_flow_module_data_protocol(uint8_t rcvdata);
void optical_flow_module_data_handle(void);

#ifdef __cplusplus

class TEST_OPTICAL_FLOW
{
public:
	bool init(void);
//	bool data_process(uint8_t rcvdata, uint8_t *data);
private:
	void stage3_init(uint8_t address, uint8_t data);
};

extern TEST_OPTICAL_FLOW optical_flow_module;




#endif

#endif

