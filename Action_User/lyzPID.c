/*==============================================头文件声明区============================================*/
#include "lyzPID.h"
#include "can.h"
#include "stdio.h"
#include "usart.h"
#include "math.h"
#include "elmo.h"
#include <stdbool.h>
#include "stm32f4xx_adc.h"
#include "timer.h"
#include "moveBase.h"

/*==============================================全局变量声明区============================================*/
extern POSITION_T Position_t;
extern int g_plan;


/*================================================函数定义区==============================================*/


/*======================================================================================
函数定义	：		通过激光判断是否开始
函数参数	：		无
函数返回值：		0	:为激光未触发
								1	:为右侧激光触发,逆时针
								-1:为左侧激光触发,顺时针
=======================================================================================*/
int IfStart()
{
	
	if(Get_Adc_Average(RIGHT_LASER,30) < 500)			//右侧激光触发
		return 1;
	else if(Get_Adc_Average(LEFT_LASER,30) < 500)		//左侧激光触发
		return -1;
	else
		return 0;
}

/*======================================================================================
函数定义	：		计算点到直线的距离
函数参数	：		直线上一点x，y坐标，直线角度
函数返回值：		返回距离值
=======================================================================================*/
float Piont2Straight(float aimx,float aimy,float angle)
{
	float dis;	//返回值
	float k;		//斜率
	
	if(angle == 0)
		dis = Position_t.X - aimx;
	else if(angle == 180||angle == -180)
		dis = aimx - Position_t.X;
	else
	{
		k = tan((PI*(angle+90)) / 180);
		dis = (k*Position_t.X-Position_t.Y-k*aimx+aimy)/sqrt(1+k*k);
		if(angle > 0 && angle < 180)
			dis = -dis;
	}
	
	return dis;
}
/*======================================================================================
函数定义	：		直线闭环
函数参数	：		直线上一点x，y坐标，直线角度，速度(mm)
函数返回值：		无
(fabs 为取float型变量的绝对值)
=======================================================================================*/
void StaightCLose(float aimx,float aimy,float angle,float speed)
{
	float Ddis,Dangle;							//距离与角度差
	static float LDdis,LDangle;			//上次差值
	float Dinput,Ainput;						//距离与角度PID计算结果

		//计算距离输出
		Ddis = Piont2Straight(aimx,aimy,angle);
		if(fabs(speed) <= 500)
			Dinput = 9 * Ddis + 1.5 * (Ddis - LDdis);
		else if(fabs(speed) <= 1000)
			Dinput = 12 * Ddis + 3 * (Ddis - LDdis);
		else if(fabs(speed) <= 1500)
			Dinput = 18 * Ddis + 3 * (Ddis - LDdis);
		else
			Dinput = 24 * Ddis + 3 * (Ddis - LDdis);
		LDdis = Ddis;
		
		//计算角度输出
		Dangle = (angle - Position_t.angle);
		if(Dangle > 180)  Dangle -= 360;
		if(Dangle < -180) Dangle += 360;
		
		if(fabs(speed) <= 500)
			Ainput = 100*Dangle + (Dangle - LDangle);			
		else if(fabs(speed) <= 1000)
			Ainput = 160*Dangle + 30*(Dangle - LDangle);			
		else if(fabs(speed) <= 1500)
			Ainput = 250*Dangle + 30*(Dangle - LDangle);	
		else
			Ainput = 350*Dangle + 30*(Dangle - LDangle);
		LDangle = Dangle;
		
		//计算脉冲
		if(speed >= 0)
		{
			VelCrl(CAN1,1,  (int)(speed * SP2PULSE) + Ainput + Dinput);
			VelCrl(CAN1,2, -(int)(speed * SP2PULSE) + Ainput + Dinput);
		}
		else
		{
			VelCrl(CAN1,1,  (int)(speed * SP2PULSE) + Ainput - Dinput);
			VelCrl(CAN1,2, -(int)(speed * SP2PULSE) + Ainput - Dinput);
		}
}
/*======================================================================================
函数定义	：		开始跑场
函数参数	：		方案：暂定1为逆时针(右侧激光触发)，-1为顺时针(左侧激光触发)
函数返回值：		无
=======================================================================================*/
void GoGoGo()
{
	static int state = 1;//应该执行的状态
	switch(state)
	{
		case 1:
		{
			if(FirstRound(FIRST_SPEED) == 1)
				state = 2;
		}break;
		case 2:
		{
			VelCrl(CAN1, 1, 0);
			VelCrl(CAN1, 2, 0);
		}break;
	}
}

/*======================================================================================
函数定义	：				第一圈
函数参数	：				plan:方案，speed:速度(mm)
函数返回值：				0未结束，1第一圈结束
(WIDTH为小车宽度)
=======================================================================================*/
int FirstRound(float speed)
{
	int ret = 0;	//返回值
	static int state = 1;

		switch(state)
		{
			case 1:
			{
				StaightCLose(g_plan * (275 + WIDTH/2 + 100),0,0,FIRST_SPEED);
				if(Position_t.Y >= 3100 + WIDTH/2 - FIR_ADV)
					state = 2;
			}break;
			case 2:
			{
				StaightCLose(0,3100 + WIDTH/2 + 100,90,FIRST_SPEED);
				if(Position_t.X <= -275 - WIDTH/2 + FIR_ADV)
					state = 3;
			}break;
			case 3:
			{
				StaightCLose(g_plan * (-275 - WIDTH/2 - 150),0,180,FIRST_SPEED);
				if(Position_t.Y <= 1700 - WIDTH/2 + FIR_ADV - 300)
					state = 4;
			}break;
			case 4:
			{
				StaightCLose(0,1700 - WIDTH/2 - 100,-90,RUN_SPEED);
				if(Position_t.X >= 275 + WIDTH/2)
				ret = 1;
			}break;
		}
	
	return ret;
}

