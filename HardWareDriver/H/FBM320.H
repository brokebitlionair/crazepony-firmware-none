#ifndef __FBM320_H
#define __FBM320_H

#include "stm32f10x.h"

#define FBM320_I2C_ADDR		0xD8
#define FBM320_DEFAULT_CHIP_ID	0x42
#define FBM320_CHIP_ID_REG	0x6B

#define SCTemperature  0x01   //开始 温度转换
#define CTemperatureing  0x02  //正在转换温度
#define SCPressure  0x03      //开始转换 气压
#define CPressureing  0x04   //正在转换气压值

typedef struct FMTI_Sensor
{
	uint8_t Version;																					//Sensor version
	uint8_t RPC;
	int32_t UP;
	int32_t UT;
	int32_t RP;																								//Unit: Pa
	int32_t RT;																								//Unit: 0.01 degree
	float Altitude;																						//Unit: meter
	uint16_t C0, C1, C2, C3, C6, C8, C9, C10, C11, C12; 
	uint32_t C4, C5, C7;
}FMTI_Sensor;

extern struct FMTI_Sensor FBM320;
extern uint8_t Baro_ALT_Updated;
extern uint8_t altOffsetInited;

void FBM320_Thread(void);
void FBM320_Initial(void);
void FBM320_START_UT(void);
void FBM320_GET_UT(void);
void FBM320_START_UP(void);
void FBM320_GET_UP(void);
void FBM320_ReadCoefficient(void);
void FBM320_Calculate(int32_t UP, int32_t UT);
int32_t Abs_Altitude(int32_t Press);
void WaitBaroInitOffset(void);

#endif
