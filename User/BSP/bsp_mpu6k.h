/**
  * @project Alpha One Flight Control System 
  * @author  Dr.Sheng Team
  * @version V1.0.0
  * @date    03-April-2019
  * @file    bsp_mpu6000.h
  */

#ifndef __BSP_MPU6K_H_
#define __BSP_MPU6K_H_


#define MPU6K_ENABLE          	GPIOB->BSRRH = GPIO_Pin_14
#define MPU6K_DISABLE         	GPIOB->BSRRL = GPIO_Pin_14
#define MPU6K_DATA_READY		(GPIOC->IDR&GPIO_Pin_5)				//用于数据更新

//检查数据是否准备就绪
#define INT_STATUS              0x3A        //Register 58 - Interrupt Status
#define RAW_DATA_RDY_INT        0x01        //              bit[0]
//读取MPU6000器件号
#define WHO_AM_I                0x75        //Register 117 - Who Am I
#define WHO_AM_I_VALUE          0x68        //               default value
//初始化用到的寄存器及相关位操作
//初始化 IMU
#define SGN_PATH_RST            0x68    //Register 104 - 0x07
#define PWR_MGMT_1              0x6B    //Register 107 - Power Management 1
#define DEVICE_RESET            0x80    //               Reset the internal registers and restores the default settings.
#define CLKSEL_CODE3            0x03    //               Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
#define PWR_MGMT_2              0x6C    //Register 108 - Power Management 2
#define PWR_MGMT_2_RESET        0x00    //
#define USER_CTRL               0x6A    //Register 106 - User Control             
#define I2C_IF_DIS              0x10    //
#define CONFIG                  0x1A    //Register 26 - Configuration->DLPF:Digital Low Pass Filter
#define DLPF_CFG_4              0x04    //              Accelerometer:21Hz;Gyroscope:20Hz;
#define DLPF_CFG_3				0x03	//              Accelerometer:44Hz;Gyroscope:43Hz;
#define DLPF_CFG_5				0x05	//              Accelerometer:10Hz;Gyroscope:10Hz;
#define SMPLRT_DIV              0x19    //Register 25 - Sample Rate Divider
#define SMPLRT_1KHZ             0x00    //              SAMPLE_RATE = Internal_Sample_Rate / (1 + SMPLRT_DIV)
#define GYRO_CONFIG             0x1B    //Register 27 - Gyroscope Configuration
#define GYRO_200DPS             0x18    //
#define ACCEL_CONFIG            0x1C    //Register 28 - Accelerometer Configuration
#define ACCEL_8G				0x10    //
#define INT_ENABLE              0x38    //Register 56 - Interrupt Enable
#define RAW_RDY_EN              0x01    //
#define INT_PIN_CFG             0x37    //Register 55 - INT Pin / Bypass Enable Configuration
#define INT_ANYRD_2CLEAR        0x10    //              When this bit is equal to 1, interrupt status bits are cleared on any read operation
#define LATCH_INT_EN        	0x20    //              When this bit is equal to 1, the INT pin is held high until the interrupt is cleared.

//9轴传感器地址
#define	ACCEL_XOUT_H			0x3B	//加速度计
#define	ACCEL_XOUT_L			0x3C          
#define	ACCEL_YOUT_H			0x3D
#define	ACCEL_YOUT_L			0x3E
#define	ACCEL_ZOUT_H			0x3F
#define	ACCEL_ZOUT_L			0x40
#define	TEMP_OUT_H				0x41	//温度传感器   
#define	TEMP_OUT_L				0x42
#define	GYRO_XOUT_H				0x43	//陀螺仪
#define	GYRO_XOUT_L				0x44	
#define	GYRO_YOUT_H				0x45
#define	GYRO_YOUT_L				0x46
#define	GYRO_ZOUT_H				0x47
#define	GYRO_ZOUT_L				0x48		

#define MPU6K_ACCEL_SCALE    	(GRAVITY_MSS / 4096.0f)
#define MPU6K_GYRO_SCALE    	(DEG_TO_RAD / 16.4f)

#ifdef __cplusplus

typedef struct
{
	int32_t ACC_X;	
	int32_t ACC_Y;
	int32_t ACC_Z;
	int32_t IMU_TEMP;		//MPU读温度的
	int32_t GYR_X;
	int32_t GYR_Y;
	int32_t GYR_Z;	
}MPU6K_SensorData_Raw_Structer;

class BSP_MPU6K
{
public:
	bool have_imu;
	MPU6K_SensorData_Raw_Structer raw_data;
	MPU6K_SensorData_Raw_Structer raw_offset;
	void init_with_check(void);
	void update(void);
	void get_offset(void);
private:
	void read_len(uint8_t addr, uint8_t len, uint8_t *data);
	void set_reg(uint8_t addr, uint8_t data);
	void init(void);
	void init_io(void);
	bool check_WHOAMI(void);
	void read_offset(void);
};

extern BSP_MPU6K mpu6k;

#endif

#endif


