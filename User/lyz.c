/*==============================================头文件声明区============================================*/
#include "lyz.h"
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
extern POSITION_T Position_t;			//校正后定位
extern POSITION_T getPosition_t;	//获得的定位
extern int g_plan;
extern int g_camera;
float angleError = 0,xError = 0,yError = 0;
extern char arr1[20];
extern unsigned char arr2[20];
extern int arr_number;

/*================================================函数定义区==============================================*/


/*======================================================================================
函数定义	：		通过激光判断是否开始
函数参数	：		无
函数返回值：		0	:为激光未触发
				        1 :为右侧激光触发,逆时针
				       -1 :为左侧激光触发,顺时针
=======================================================================================*/
int IfStart()
{
	if(Get_Adc_Average(RIGHT_LASER,30) < 500)				//右侧激光触发
		return 1;
	else if(Get_Adc_Average(LEFT_LASER,30) < 500)		//左侧激光触发
		return -1;
	else
		return 0;
}

/*======================================================================================
函数定义		：		计算点到直线的距离
函数参数		：		直线上一点x，y坐标，直线角度
函数返回值	：		返回距离值
=======================================================================================*/
float Piont2Straight(float aimx,float aimy,float angle)
{
	float dis;	//返回值
	float k;	//斜率
	
	if(angle == 0)
		dis = Position_t.X - aimx;
	else if(angle == 180)
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
(fabs 为取float型变量的绝对值)  PID的值需调节
=======================================================================================*/
void StaightCLose(float aimx,float aimy,float angle,float speed)
{
	float Ddis,Dangle;							//距离与角度差
	float Dinput,Ainput;						//距离与角度PID计算结果
	static float LDdis,LDangle;			//上一次距离与角度差

		//计算距离输出
		Ddis = Piont2Straight(aimx,aimy,angle);

		if		 (fabs(speed) <=  500) 	Dinput = 9 	* Ddis + 1.5 * (Ddis - LDdis);
		else if(fabs(speed) <= 1000) 	Dinput = 12 * Ddis +   3 * (Ddis - LDdis);
		else if(fabs(speed) <= 1500) 	Dinput = 18 * Ddis +   3 * (Ddis - LDdis);
		else 													Dinput = 24 * Ddis +   3 * (Ddis - LDdis);

		LDdis = Ddis;
		
		//计算角度输出
		Dangle = (angle - Position_t.angle);
		if(Dangle >  180) Dangle -= 360;
		if(Dangle < -180) Dangle += 360;
		

		if		 (fabs(speed) <=  500) 	Ainput = 100*Dangle +  1 * (Dangle - LDangle);
		else if(fabs(speed) <= 1000) 	Ainput = 160*Dangle + 30 * (Dangle - LDangle);
		else if(fabs(speed) <= 1500) 	Ainput = 250*Dangle + 30 * (Dangle - LDangle);
		else 													Ainput = 350*Dangle + 30 * (Dangle - LDangle);

		LDangle = Dangle;
		
		//计算脉冲
		if(speed >= 0)
		{
			VelCrl(CAN1,1,  (int)(speed * SP2PULSE) + g_plan * (Ainput + Dinput));
			VelCrl(CAN1,2, -(int)(speed * SP2PULSE) + g_plan * (Ainput + Dinput));
		}
		else
		{
			VelCrl(CAN1,1,  (int)(speed * SP2PULSE) + g_plan * (Ainput - Dinput));
			VelCrl(CAN1,2, -(int)(speed * SP2PULSE) + g_plan * (Ainput - Dinput));
		}
		
}

/*======================================================================================
函数定义	：		开始跑场
函数参数	：		方案：暂定1为逆时针(右侧激光触发)，-1为顺时针(左侧激光触发)
函数返回值：		无
=======================================================================================*/
void GoGoGo()
{
	static int state = 1;							//应该执行的状态
	static int length = WIDTH/2,wide = WIDTH/2;		//长方形跑场参数
	switch(state)
	{
		//第一圈放球区附近跑场
		case 1:
		{
			if(FirstRound(FIRST_SPEED) == 1)
			{
				length += SPREAD_DIS;				//初始化长方形跑场参数
				wide	 += SPREAD_DIS;
				state = 2;
			}
		}break;

		//向外扩散扫场
		case 2:
		{
			if(RunRectangle(length,wide,RUN_SPEED))
			{
				length += SPREAD_DIS;				//逐渐增加长方形跑场参数
				wide	 += SPREAD_DIS;
				if(length >= 1700 - WIDTH/2 - 100) length = 1700 - WIDTH/2 - 100;
				if(wide		>= 2125 - WIDTH/2 - 100) wide		= 2125 - WIDTH/2 - 100;
			}
			if(length >= 1700 - WIDTH/2 - 100 && wide >= 2125 - WIDTH/2 - 100)
				state = 3;
		}break;

		//进行坐标校正
		case 3:
		{
			CheckPosition();
		}break;
		case 4:
		{
			//射球
		}break;
		case 5:
		{
			RunCamera();
		}break;
	}
}

/*======================================================================================
函数定义		：			第一圈
函数参数		：			plan:方案，speed:速度(mm)
函数返回值	：			false未结束，true第一圈结束
用时				：			未测算
(WIDTH为小车宽度)
=======================================================================================*/
bool FirstRound(float speed)
{
	static int state = 1;
		switch(state)
		{
			//右边，目标角度0度
			case 1:
			{
				StaightCLose((275 + WIDTH/2 + 100),0,0,FIRST_SPEED);
				if(Position_t.Y >= 3100 + WIDTH/2 - FIR_ADV)
					state = 2;
			}break;
			
			//上边，目标角度90度
			case 2:
			{
				StaightCLose(0,3100 + WIDTH/2 + 100,90,FIRST_SPEED);
				if(Position_t.X <= -275 - WIDTH/2 + FIR_ADV)
					state = 3;
			}break;
			
			//左边，目标角度180度
			case 3:
			{
				StaightCLose((-275 - WIDTH/2 - 150),0,180,FIRST_SPEED);
				if(Position_t.Y <= 1700 - WIDTH/2 + FIR_ADV - 500)
					state = 4;
			}break;
			
			//下边，目标角度-90度
			case 4:
			{
				StaightCLose(0,1700 - WIDTH/2 - 100,-90,RUN_SPEED);
				if(Position_t.X >= 275 + WIDTH/2 - FIR_ADV)
				return true;
			}break;
		}
	
	return false;
}


/*======================================================================================
函数定义		：			长方形扫场
函数参数		：			length	:	小车中线与放球区y=y1的距离
										wide		:	小车中线与放球区x=x1的距离
										speed		:	速度
函数返回值	    ：	true扫场结束,false未结束
暂时未加入x的镜像对称
=======================================================================================*/
bool	RunRectangle(int length,int wide,float speed)
{
	static int state = 1;
	switch(state)
	{
		//长方形右边，目标角度0度
		case 1:
		{
			if(Position_t.Y >= 3100 + length - ADV_TUEN)
				state = 2;
			StaightCLose(275 + wide,0,0,speed);
		}break;
		
		//长方形上边，目标角度90度
		case 2:
		{
			if(Position_t.X <= -275 - wide + ADV_TUEN)
				state = 3;
			StaightCLose(0,3100 + length,90,speed);
		}break;
			
		//长方形左边，目标角度180度
		case 3:
		{
			if(Position_t.Y <= 1700 - length + ADV_TUEN)
				state = 4;
			StaightCLose(-275 - wide,0,180,speed);
		}break;
		
		//长方形下边，目标角度-90度
		case 4:
		{
			if(Position_t.X >= 275 + wide - ADV_TUEN)
			{
				state = 1;
				return true;
			}
			StaightCLose(0,1700 - length,-90,speed);
		}break;
	}
	return false;
}

/*======================================================================================
函数定义		：			判断小车是否卡住不动，
函数参数		：			无
函数返回值	：			false 未卡住，true卡住了
=======================================================================================*/
bool IfStuck()
{
	static int count = 0;
	static int lx = 0,ly = 0;	//记录上一次的坐标
	if((int)Position_t.X == lx && (int)Position_t.Y == ly)
	{
		count++;
		if(count >= 100 * STUCK_TIME)	//卡住了
		{
			count = 0;
			return true;
		}
	}
	else count = 0;
	
	//保存上一次坐标
	lx = (int)Position_t.X;
	ly = (int)Position_t.Y;
	return false;
}

/*======================================================================================
函数定义		：			判断小车是否靠墙不动，
函数参数		：			无
函数返回值	：			false 未卡住，true卡住了
当行程开关加入后，删除此函数。此函数为临时函数
=======================================================================================*/
bool IfStuck2()
{
	static int count = 0;
	static int lx = 0,ly = 0;	//记录上一次的坐标
	if((int)Position_t.X == lx && (int)Position_t.Y == ly)
	{
		count++;
		if(count >= 100 * (STUCK_TIME - 0.3))	//卡住了  这里-0.3是防止程序进入逃逸模式
		{
			count = 0;
			return true;
		}
	}
	else count = 0;
	
	//保存上一次坐标
	lx = (int)Position_t.X;
	ly = (int)Position_t.Y;
	return false;
}

/*======================================================================================
函数定义		：			坐标校正
函数参数		：			无
函数返回值	：			无
=======================================================================================*/
void CheckPosition()
{
	static int state = 1;
	static int count = 0;
	static int tempx = 0,tempy = 0;
	count++;
	switch(state)
	{
		//后退到 x = 0
		case 1:
		{
			StaightCLose(0,500,-90,-500);
			if(Position_t.X <= 0)				//在y = 0处两个激光不容易射到球或射出去
			{
				count = 0;
				state = 2;
			}
		}break;
		
		//原地旋转至0度
		case 2:
		{
			TurnAngle(0,3000);
			if(count >= 120)
			{
				tempx = Position_t.X;			//记录当前坐标用于闭环后退，防止角度被撞歪后开环后退不准
				tempy = Position_t.Y;
				state = 3;
			}
		}break;
		
		//后退靠墙
		case 3:
		{
			StaightCLose(tempx,tempy,0,-800);
			if(IfStuck2())								//到时候改成两个行程开关被触发
			{
				state = 4;
			}
		}break;
		
		//激光校正
		case 4:
		{
			if(LaserCheck())	
			{
				state = 5;	//矫正成功，开始第二阶段跑场
			  USART_OUT(USART1,(u8*)"x%d\t y%d\t angle%d\r\n",(int)Position_t.X,(int)Position_t.Y,(int)Position_t.angle);
			}
			else							
				state = 6;	//矫正失败，继续矫正
		}break;
		
		//第二阶段跑场
		case 5:
		{
			if(RunRectangle(1000,800,1500) == 1)
				state = 1;
		}break;
		
		//继续矫正
		case 6:
		{
			
		}break;
	}
}

/*======================================================================================
函数定义		：			利用摄像头跑场
函数参数		：			无
函数返回值	：			无
=======================================================================================*/
void	RunCamera()
{

	if(g_camera == 2)
	{
		VelCrl(CAN1, 1,  500*SP2PULSE);
		VelCrl(CAN1, 2, -500*SP2PULSE);
	} 
	else if(g_camera == 1)

	;
}

/*======================================================================================
函数定义		：			原地旋转到指定角度
函数参数		：			目标角度，旋转速度
函数返回值	：			无
=======================================================================================*/
void TurnAngle(float angel,int speed)
{
	float Dangel = 0;							//角度差值
	float Input 	= 0;						//pid控制输出
	Dangel = (angel- Position_t.angle);
	
	//纠正角度
	if(Dangel >  180) Dangel -= 360;
	if(Dangel < -180) Dangel += 360;
	Input = 30*Dangel;
	
	//判断是否反向转更快
	if(Dangel < 0 && speed >0)
		speed = -speed;
		
	//角度大于10，speed转
	if(Dangel > 15 || Dangel < -15)

	{
		VelCrl(CAN1,2, g_plan * speed);
		VelCrl(CAN1,1, g_plan * speed);
	}
	
	//角度小于10，pid控制输出
	else
	{
		VelCrl(CAN1,2, g_plan * Input);
		VelCrl(CAN1,1, g_plan * Input);
	}
}
/*======================================================================================
函数定义		：			利用激光矫正坐标
函数参数		：			无
函数返回值	：			矫正成功返回1，失败返回0
=======================================================================================*/
int	LaserCheck()
{
	int laserGetRight=0,laserGetLeft=0;
	laserGetRight = Get_Adc_Average(RIGHT_LASER,20);
	laserGetLeft = Get_Adc_Average(LEFT_LASER,20);

	//如果激光被挡，返回 0 
	if(laserGetRight+laserGetLeft<4700)
	{
		return 0;
	}
	
	//没有被挡，返回
	else
	{
		angleError = angleError + Position_t.angle;		//纠正角度坐标
		yError = (getPosition_t.Y*cos(Angel2PI(angleError))-getPosition_t.X*sin(Angel2PI(angleError)));
		xError = (getPosition_t.X*cos(Angel2PI(angleError))+getPosition_t.Y*sin(Angel2PI(angleError)))-(2400-laserGetRight);//纠正X坐标
		USART_OUT(USART1,(u8*)"laserGet%d\r\n",(int)laserGetRight);
		return 1;
	}
}

//角度变换函数
float Angel2PI(float angel)
{
	float res;
	res = PI*(angel) / 180;
	return res;
}

