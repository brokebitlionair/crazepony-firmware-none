/*    
      ____                      _____                  +---+
     / ___\                     / __ \                 | R |
    / /                        / /_/ /                 +---+
   / /   ________  ____  ___  / ____/___  ____  __   __
  / /  / ___/ __ `/_  / / _ \/ /   / __ \/ _  \/ /  / /
 / /__/ /  / /_/ / / /_/  __/ /   / /_/ / / / / /__/ /
 \___/_/   \__,_/ /___/\___/_/    \___ /_/ /_/____  /
                                                 / /
                                            ____/ /
                                           /_____/
ReceiveData.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.接收函数文件，包括接收2.4G数据，UART1的数据流
2.解析数据包，分配给对应的控制量
------------------------------------
*/

#include "config.h" 
#include "ReceiveData.h"
#include "imu.h"
#include "moto.h"
#include "led.h"
#include "MPU6050.h"
#include "extern_variable.h"
#include "UART1.h"
#include "control.h"
#include "stmflash.h"
#include "dmp.h"
#include "stm32f10x_it.h"
#include "SysConfig.h"
#include "CommApp.h"
#include "RF.h"
#include "delay.h"
#include "ConfigTable.h"

uint8_t FLY_ENABLE=0;//aircraft enable
uint8_t imuCaliFlagTime,FlyEnableTime,FlyDisableTime;

RC_GETDATA  RC_DATA;//={0,0,0,0},RC_DATA_RAW={0,0,0,0};	// RC_DATA是处理后的期望四通

extern uint32_t lastGetRCTime;
extern uint8_t RF_RXDATA[7];

int culValue(int input, int inMin, int inMax, int outMin, int outMax) 
{
    	return (int)((input*1.0 - inMin*1.0) * (outMax*1.0 - outMin*1.0) / (inMax*1.0 - inMin*1.0) + outMin);
}

//函数名：ReceiveDataFormRF()
//输入：无
//输出: 无
//描述：将收到的2.4G遥控数据赋值给对应的变量
//作者：马骏
void ReceiveDataFormRF(void)
{
		if((RF_RXDATA[0] == 0xfe) && (RF_RXDATA[6] == 0xfa))
		{
			rcData[THROTTLE]=culValue(RF_RXDATA[3],10,245,1100,1900);
			rcData[YAW]=culValue(RF_RXDATA[4],10,245,1900,1100);
			rcData[PITCH]=culValue(RF_RXDATA[1],10,245,1800,1200);
			rcData[ROLL]=culValue(RF_RXDATA[2],10,245,1200,1800);
		 
//			if(RF_RXDATA[5]&0x10) SetHeadFree(1);
//			else SetHeadFree(0);
			
			
			if((RF_RXDATA[3] < 0x3A) && (RF_RXDATA[4] < 0x3A)) //左摇杆右下
			{
//				if((RF_RXDATA[1] < 0x3A) && (RF_RXDATA[2] < 0x3A))  //右摇杆右下
//				{
					imuCaliFlagTime++;
					if(imuCaliFlagTime > 80)
					{
						imuCaliFlagTime = 0;
						imuCaliFlag = 1;
					}
//				}
//				else
//				{
//					FlyDisableTime++;
//					if(FlyDisableTime > 80)
//					{
//						FlyDisableTime = 0;
//						armState =REQ_ARM;
//					}
//			}
			}
		 
			else if((RF_RXDATA[3] < 0x3A) && (RF_RXDATA[4] > 0xBA)) //左摇杆左下
			{
				FlyEnableTime++;
				if(FlyEnableTime > 80)
				{
					FlyEnableTime = 0;
					armState =REQ_ARM;
				}
			}
		 
			else 
			{
				imuCaliFlagTime = 0;
				FlyEnableTime = 0;
				FlyDisableTime = 0;
			}
		}
		lastGetRCTime=millis();		//ms
}



