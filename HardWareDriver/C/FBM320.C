#include "config.h"
#include "FBM320.h"
#include <math.h>
#include "stm32f10x_it.h"

FMTI_Sensor	FBM320;

uint8_t  Now_doing = SCPressure;
uint32_t Start_Convert_Time;
uint8_t Baro_ALT_Updated = 0;
uint8_t fbm320_chip_id = 0;

//#define MOVAVG_SIZE  10	   //保存最近10组数据
#define ALT_OFFSET_INIT_NUM 50

//static float Temp_buffer[MOVAVG_SIZE],Press_buffer[MOVAVG_SIZE],Alt_buffer[MOVAVG_SIZE];
static uint8_t temp_index=0,press_index=0; //队列指针
float Alt_offset=0;
uint8_t altOffsetInited=0;

void FBM320_Thread(void)
{
	switch(Now_doing)
	{
		case SCPressure:
			FBM320_START_UP();		//Convert Pressure ADC
			Start_Convert_Time = micros(); //计时开始
            Now_doing = CPressureing;//下一个状态
			break;

		case CPressureing:
			if((micros()-Start_Convert_Time) > 12000)
			{
				FBM320_GET_UP();		//Receive Pressure ADC value
				FBM320_START_UT();		//Convert Temperature ADC
				Start_Convert_Time = micros();
				Now_doing = CTemperatureing;
			}
			break;

		case CTemperatureing:
			if((micros()-Start_Convert_Time) > 4000)
			{
				FBM320_GET_UT();		//Receive Temperature ADC value																														
				FBM320_Calculate(FBM320.UP, FBM320.UT);		//Calculate RP, RT
				if(!altOffsetInited)
					FBM320.Altitude = ((float)Abs_Altitude(FBM320.RP))/1000;
				else 
					FBM320.Altitude = ((float)Abs_Altitude(FBM320.RP))/1000 - Alt_offset;		//Calculate Altitude
				Baro_ALT_Updated = 0xff;
				FBM320_START_UP();
				Start_Convert_Time = micros(); //计时开始
				Now_doing = CPressureing;//下一个状态
			}
			break;

		default:
			Now_doing = CPressureing;
			break;
	}
}

void FBM320_Initial(void)
{
	FBM320.RPC = 3;
	fbm320_chip_id = I2C_ReadOneByte(FBM320_I2C_ADDR, FBM320_CHIP_ID_REG);
	if (fbm320_chip_id == FBM320_DEFAULT_CHIP_ID)
		printf("FBM320 found...\r\n");
	FBM320.Version = ((I2C_ReadOneByte(FBM320_I2C_ADDR, 0xA5) & 0x70) >> 2) | ((I2C_ReadOneByte(FBM320_I2C_ADDR, 0xF4) & 0xC0) >> 6);
	FBM320_ReadCoefficient();	//Read FBM320 coefficient
}

void FBM320_START_UT(void)
{
    // start temperature measurement
    IICwriteByte(FBM320_I2C_ADDR, 0xF4, 0x2E);
}

void FBM320_GET_UT(void)
{
   	uint8_t data[3];
    IICreadBytes(FBM320_I2C_ADDR, 0xF6, 3, data);
    FBM320.UT = (int32_t)((((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
}

void FBM320_START_UP(void)
{
    // start pressure measurement
    IICwriteByte(FBM320_I2C_ADDR, 0xF4, 0xF4);
}

void FBM320_GET_UP(void)
{
    uint8_t data[3];
    IICreadBytes(FBM320_I2C_ADDR, 0xF6, 3, data);
    FBM320.UP = (int32_t)((((uint32_t)(data[0])) << 16) | (((uint32_t)(data[1])) << 8) | ((uint32_t)data[2]));
}

void FBM320_ReadCoefficient(void)		//FMTI sensor receive R0~R9 and calibrate coefficient C0~C12
{
	uint8_t i;
	uint8_t data[18];
	uint16_t R[10]={0};
	
	IICreadBytes(FBM320_I2C_ADDR, 0xAA, 18, data);
	for(i=0; i<9; i++)
		R[i] = (data[i*2]<<8) | data[i*2+1];
	
	/*	Use R0~R9 calculate C0~C12 of FBM320	*/
	FBM320.C0 = R[0] >> 4;
	FBM320.C1 = ((R[1] & 0xFF00) >> 5) | (R[2] & 7);
	FBM320.C2 = ((R[1] & 0xFF) << 1) | (R[4] & 1);
	FBM320.C3 = R[2] >> 3;
	FBM320.C4 = ((uint32_t)R[3] << 2) | (R[0] & 3);
	FBM320.C5 = R[4] >> 1;
	FBM320.C6 = R[5] >> 3;
	FBM320.C7 = ((uint32_t)R[6] << 3) | (R[5] & 7);
	FBM320.C8 = R[7] >> 3;
	FBM320.C9 = R[8] >> 2;
	FBM320.C10 = ((R[9] & 0xFF00) >> 6) | (R[8] & 3);
	FBM320.C11 = R[9] & 0xFF;
	FBM320.C12 = ((R[0] & 0x0C) << 1) | (R[7] & 7);
}

void FBM320_Calculate(int32_t UP, int32_t UT)		//Calculate Real pressure & temperautre
{
	int32_t DT, DT2, X01, X02, X03, X11, X12, X13, X21, X22, X23, X24, X25, X26, X31, X32, CF, PP1, PP2, PP3, PP4;
	int32_t dC05, dRT, DPC2, DP;
	int32_t BT1, BT2, DPTS1, DPTI1, DPTS2, DPTI2, DPC2S0, DPC2S1;

	if(FBM320.Version == 3)		//Version: FBM320-03, RPC-01+02 coefficient
	{
		DPC2S0 = 559;
		DPC2S1 = 549;
		BT1 = 2000;
		BT2 = 700;
		DPTS1 = -1513;
		DPTI1 = 46;
		DPTS2 = 819;
		DPTI2 = -9;
	}

	DT	=	((UT - 8388608) >> 4) + (FBM320.C0 << 4);
	X01	=	(FBM320.C1 + 4459) * DT >> 1;
	X02	=	((((FBM320.C2 - 256) * DT) >> 14) * DT) >> 4;
	X03	=	(((((FBM320.C3 * DT) >> 18) * DT) >> 18) * DT);
	FBM320.RT	=	((2500 << 15) - X01 - X02 - X03) >> 15;
				
	DT2	=	(X01 + X02 + X03) >> 12;
				
	X11	=	((FBM320.C5 - 4443) * DT2);
	X12	=	(((FBM320.C6 * DT2) >> 16) * DT2) >> 2;
	X13	=	((X11 + X12) >> 10) + ((FBM320.C4 + 120586) << 4);
				
	X21	=	((FBM320.C8 + 7180) * DT2) >> 10;
	X22	=	(((FBM320.C9 * DT2) >> 17) * DT2) >> 12;
	if(X22 >= X21)
		X23 = X22 - X21;
	else
		X23 = X21 - X22;

	X24	=	(X23 >> 11) * (FBM320.C7 + 166426);
	X25	=	((X23 & 0x7FF) * (FBM320.C7 + 166426)) >> 11;
	X26 = (X21 >= X22) ? (((0 - X24 - X25) >> 11) + FBM320.C7 + 166426) : (((X24 + X25) >> 11) + FBM320.C7 + 166426);

	PP1	=	((UP - 8388608) - X13) >> 3;
	PP2	=	(X26 >> 11) * PP1;
	PP3	=	((X26 & 0x7FF) * PP1) >> 11;
	PP4	=	(PP2 + PP3) >> 10;
				
	CF	=	(2097152 + FBM320.C12 * DT2) >> 3;
	X31	=	(((CF * FBM320.C10) >> 17) * PP4) >> 2;
	X32	=	(((((CF * FBM320.C11) >> 15) * PP4) >> 18) * PP4);
	FBM320.RP	=	((X31 + X32) >> 15) + PP4 + 100000;

	if(FBM320.Version ==3)
	{
		if((FBM320.RPC & 1) == 1)		//Use RPC-01
		{
			dC05 = FBM320.C5 - 16384;
			dRT = FBM320.RT - 2500;
			DPC2 = (((DPC2S1 * dC05) >> 9) + DPC2S0) >> 4;
			DP = (((DPC2 * dRT) >> 15) * dRT) >> 11;
			FBM320.RP -= DP;
		}

		if((FBM320.RPC & 2) == 2)		//Use RPC-02
		{
			if(FBM320.RT < BT1) 
			{
				DP = ((FBM320.RT * DPTS1) >> 16) + DPTI1;
				if(FBM320.RT < BT2) 
					DP = ((FBM320.RT * DPTS2) >> 16) + DPTI2 + DP;
				FBM320.RP -= DP;
			}
		}
	}
}

int32_t Abs_Altitude(int32_t Press)		//Calculate absolute altitude, unit: mm
{
	int8_t P0;			
	int16_t hs1, dP0;			
	int32_t h0, hs0, HP1, HP2;			
					
	if(Press >= 103000)
	{	
		P0	=	103;
		h0	=	-138507;
		hs0	=	-21007;
		hs1	=	311;
	}	
	else if(Press >= 98000)
	{	
		P0	=	98;
		h0	=	280531;
		hs0	=	-21869;
		hs1	=	338;
	}	
	else if(Press >= 93000)
	{	
		P0	=	93;
		h0	=	717253;
		hs0	=	-22813;
		hs1	=	370;
	}	
				
	else if(Press >= 88000)
	{	
		P0	=	88;
		h0	=	1173421;
		hs0	=	-23854;
		hs1	=	407;
	}	
	else if(Press >= 83000)
	{	
		P0	=	83;
		h0	=	1651084;
		hs0	=	-25007;
		hs1	=	450;
	}	
	else if(Press >= 78000)
	{	
		P0	=	78;
		h0	=	2152645;
		hs0	=	-26292;
		hs1	=	501;
	}	
	else if(Press >= 73000)
	{	
		P0	=	73;
		h0	=	2680954;
		hs0	=	-27735;
		hs1	=	560;
	}	
	else if(Press >= 68000)
	{	
		P0	=	68;
		h0	=	3239426;
		hs0	=	-29366;
		hs1	=	632;
	}	
	else if(Press >= 63000)
	{	
		P0	=	63;
		h0	=	3832204;
		hs0	=	-31229;
		hs1	=	719;
	}	
	else if(Press >= 58000)
	{	
		P0	=	58;
		h0	=	4464387;
		hs0	=	-33377;
		hs1	=	826;
	}	
	else if(Press >= 53000)
	{	
		P0	=	53;
		h0	=	5142359;
		hs0	=	-35885;
		hs1	=	960;
	}		
	else if(Press >= 48000)
	{	
		P0	=	48;
		h0	=	5874268;
		hs0	=	-38855;
		hs1	=	1131;
	}	
	else if(Press >= 43000)
	{	
		P0	=	43;
		h0	=	6670762;
		hs0	=	-42434;
		hs1	=	1354;
	}	
	else if(Press >= 38000)
	{	
		P0	=	38;
		h0	=	7546157;
		hs0	=	-46841;
		hs1	=	1654;
	}	
	else if(Press >= 33000)
	{	
		P0	=	33;
		h0	=	8520395;
		hs0	=	-52412;
		hs1	=	2072;
	}	
	else
	{	
		P0	=	28;
		h0	=	9622536;
		hs0	=	-59704;
		hs1	=	2682;
	}
					
	dP0	=	Press - P0 * 1000;
	HP1	=	(hs0 * dP0) >> 2;
	HP2	=	(((hs1 * dP0) >> 10)* dP0) >> 4;			

	return	((h0 << 6) + HP1 + HP2) >> 6;		//Return absolute altitude
}

void WaitBaroInitOffset(void)
{
	uint16_t altInitCnt=0;	
	uint32_t startTime=0;
	uint32_t now=0;	
	
	Alt_offset = 0;
	startTime=micros();	//us
	
  while(!altOffsetInited)
	{
			FBM320_Thread();
			now=micros();
			if(Baro_ALT_Updated == 0xff)
			{
				Alt_offset = Alt_offset + ((float)Abs_Altitude(FBM320.RP))/1000;
				altInitCnt++;
				Baro_ALT_Updated = 0;
			}
			if((now-startTime)/1000 >= ALT_OFFSET_INIT_NUM * 50)	//超时
			{
				Alt_offset = Alt_offset/altInitCnt;
				altOffsetInited = 1;
			}
	}
}

