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
led.c file
编写者：小马  (Camel)
作者E-mail：375836945@qq.com
编译环境：MDK-Lite  Version: 4.23
初版时间: 2014-01-28
功能：
1.飞机四个臂上led IO口初始化
2.初始化默认关闭所有LED灯
------------------------------------
*/

#include "Led.h"
#include "UART1.h"
#include "config.h"
#include "imu.h"
#include "FailSafe.h"

LED_t LEDCtrl;
//接口显存
LEDBuf_t LEDBuf;

extern int LostRCFlag;


/********************************************
              Led初始化函数
功能：
1.配置Led接口IO输出方向
2.关闭所有Led(开机默认方式)
描述：
Led接口：
Led1-->PA11
Led2-->PA8
Led3-->PB1
Led4-->PB3
对应IO=1，灯亮
********************************************/
void LedInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB,ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_SetBits(GPIOA, GPIO_Pin_11);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_Init(GPIOB, &GPIO_InitStructure);
		GPIO_SetBits(GPIOB, GPIO_Pin_9|GPIO_Pin_8|GPIO_Pin_12);
}

//底层更新 ，10Hz
void LEDReflash(void)
{
 
		if(LEDBuf.bits.A)
			LedA_on;
		else
			LedA_off;
		
		if(LEDBuf.bits.B)
			LedB_on;
		else
			LedB_off;
		
		if(LEDBuf.bits.C)
			LedC_on;
		else
			LedC_off;
		
		if(LEDBuf.bits.D)
			LedD_on;
		else
			LedD_off;
}

//事件驱动层
void LEDFSM(void)
{
	//闪烁状态由几个系统的标志决定,优先级依次按判断顺序上升
	LEDCtrl.event=E_READY;

//	if(!imu.ready)		//开机imu准备
//		LEDCtrl.event=E_CALI;
	
	if(1 == LostRCFlag)
			LEDCtrl.event=E_LOST_RC;	

	if(!imu.caliPass)
		LEDCtrl.event=E_CALI_FAIL;
	
	if(Battery.alarm)
		LEDCtrl.event=E_BAT_LOW;
	
	if(imuCaliFlag)
		LEDCtrl.event=E_CALI;
	
	if((Battery.chargeSta))			//battery charge check
		LEDCtrl.event = E_BatChg;
	
	if(LANDING == altCtrlMode){
		LEDCtrl.event = E_AUTO_LANDED;
	}
		
	switch(LEDCtrl.event)
	{
		case E_READY:
				LEDBuf.byte =LA|LB|LC|LD;
			break;
		case E_CALI:
				LEDBuf.byte=LA|LB;
			break;
		case E_BAT_LOW:
				if(++LEDCtrl.cnt >= 3)		//0 1  in loop
					LEDCtrl.cnt=0;
				if(LEDCtrl.cnt==0)
						LEDBuf.byte =0x0f;
				else
						LEDBuf.byte =0;
			break;
		case E_CALI_FAIL:
				if(++LEDCtrl.cnt >= 4)
					LEDCtrl.cnt=0;
				if(LEDCtrl.cnt<2)
						LEDBuf.byte =LC|LD;
				else
						LEDBuf.byte =LA|LB;
			break;
		case E_LOST_RC:
				if(++LEDCtrl.cnt >= 4)
					LEDCtrl.cnt=0;
				LEDBuf.byte= 1<<LEDCtrl.cnt ;
			break;
		case E_AUTO_LANDED:
				 LEDBuf.byte=0x0f;
			break;
		
		case E_BatChg:
				 LEDBuf.byte=0x00;
			break;
		
	}
	
	LEDReflash();
}


