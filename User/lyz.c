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
#include "stm32f4xx_it.h"
#include "c0.h"

/*==============================================全局变量声明区============================================*/
extern POSITION_T Position_t;
extern int g_plan;
extern int g_camera;


/*================================================函数定义区==============================================*/


/*======================================================================================
函数定义	：		通过激光判断是否开始
函数参数	：		无
函数返回值：		0	:为激光未触发
								1	:为右侧激光触发,逆时针
								-1:为左侧激光触发,顺时针
=======================================================================================*/
int IfStart(void)
{
	
	if(Get_Adc_Average(RIGHT_LASER,30) < 500)				//右侧激光触发
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
(fabs 为取float型变量的绝对值)  PID的值需调节
=======================================================================================*/
void StaightCLose(float aimx,float aimy,float angle,float speed)
{
	float Ddis,Dangle;							//距离与角度差
	static float LDdis,LDangle;			//上次差值
	float Dinput,Ainput;						//距离与角度PID计算结果

		//计算距离输出
		Ddis = Piont2Straight(aimx,aimy,angle);
		if		 (fabs(speed) <=  500) 	Dinput = 9 * Ddis;
		else if(fabs(speed) <= 1000) 	Dinput = 12 * Ddis;
		else if(fabs(speed) <= 1500) 	Dinput = 15 * Ddis;
		else 													Dinput = 18 * Ddis;
		LDdis = Ddis;
		
		//计算角度输出
		Dangle = (angle - Position_t.angle);
		if(Dangle >  180)  Dangle -= 360;
		if(Dangle < -180) Dangle += 360;
		
		if		 (fabs(speed) <=  500) 	Ainput = 100*Dangle;
		else if(fabs(speed) <= 1000) 	Ainput = 160*Dangle;
		else if(fabs(speed) <= 1500) 	Ainput = 250*Dangle;
		else 													Ainput = 350*Dangle;
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
void GoGoGo(void)
{
	static int state = 1;													//应该执行的状态
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
				if(length >= 1700 - WIDTH) length = 1700 - WIDTH;
			}
			  if(length >= 1700 - WIDTH && wide >= 2125 - WIDTH)
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
函数返回值	：			0未结束，1第一圈结束
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
				if(Position_t.Y <= 1700 - WIDTH/2 + FIR_ADV - 300)
					state = 4;
			}break;
			
			//下边，目标角度-90度
			case 4:
			{
				StaightCLose(0,1700 - WIDTH/2 - 100,-90,RUN_SPEED);
				if(Position_t.X >= 275 + WIDTH/2)
				return true;
			}break;
		}
	
	return false;
}

/*======================================================================================
函数定义		：			判断小车是否卡住不动，
函数参数		：			判断卡住时长(s)
函数返回值	：			false 未卡住，true卡住了
=======================================================================================*/
bool IfStuck(void)
{
	static int count = 0;
	static int lx = 0,ly = 0;	//记录上一次的坐标
	if((int)Position_t.X == lx && (int)Position_t.Y == ly)
	{
		count++;
		if(count >= 100 * STUCK_TIME)
		{
			count = 0;
			return true;					//卡住了
		}
	}
	else count = 0;
	
	//保存上一次坐标
	lx = (int)Position_t.X;
	ly = (int)Position_t.Y;
	return false;
}

/*======================================================================================
函数定义		：			长方形扫场
函数参数		：			length	:小车中线与放球区y=y1的距离
										wide		:小车中线与放球区x=x1的距离
										speed		:速度
函数返回值	：			true扫场结束,false未结束
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
			if(Position_t.Y <= 1700 - wide + ADV_TUEN)
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
函数定义		：			坐标校正
函数参数		：			无
函数返回值	：			无
=======================================================================================*/
void CheckPosition(void)
{
	VelCrl(CAN1, 1, 0);
	VelCrl(CAN1, 2, 0);
}

/*======================================================================================
函数定义		：			利用摄像头跑场
函数参数		：			无
函数返回值	：			无
=======================================================================================*/
void	RunCamera(void)
{
	extern  int8_t arr1[20];
	extern uint8_t arr2[20]; 
	extern int go,arr_number;
	int haveBall=0,run=0,ballAngle,trace[10][10]={0},s[10][10],stagger=0,left,right,up,down;
	int * s0=s[0],* s1=s[1],* s2=s[2],* s3=s[3],* s4=s[4],* s5=s[5],* s6=s[6],* s7=s[7],* s8=s[8],* s9=s[9];
	//中断里接收到数据结束位0xc9时 go置一 算出目标角度
	if(go==1)
	{
		go=0;
		if(arr_number==0)		
		{
			haveBall=0;
		}
		else 
		{
			haveBall=1;
			ballAngle=AngCamera2Gyro(Closer_Point(arr1,arr2,arr_number).dis,Closer_Point(arr1,arr2,arr_number).ang);
	//		USART_OUT(USART1,(uint8_t*) "%d\t%d\r\n",ballAngle,arr_number);
		}			
	}
	
	//到边界要拐弯了
	if(fabs(Position_t.X)>2000||Position_t.Y<400||Position_t.Y>4400)
	{
		haveBall=0;
	}
	
	//一环连一环 上部分是偶数则加一 下部分是奇数则加一 让stagger(错开)等于一
	if(Position_t.X>-115&&Position_t.X<-85&&Position_t.Y<1700)
	{
		if(((int)(run/2)+(int)(run/2))!=run)
		{
			 run++;
			 stagger=1;
		}	
	}
	if(Position_t.X>-115&&Position_t.X<-85&&Position_t.Y>3100)
	{
		if(((int)(run/2)+(int)(run/2))==run)
		{
			 run++;
		}
	}
	
	//记录走过的位置（区域）
	trace[Zoning(Position_t.X,Position_t.Y).hor][Zoning(Position_t.X,Position_t.Y).ver]=1;
	//计算走过的路线 寻找一条错开的路线
	if(run>0&&((int)(run/2)+(int)(run/2))==run&&stagger==1)
	{ 
		stagger=0;
    change(s0,trace,0); 
    change(s1,trace,1); 
    change(s2,trace,2); 
    change(s3,trace,3); 
    change(s4,trace,4); 
    change(s5,trace,5); 
    change(s6,trace,6);
		change(s7,trace,7);
    change(s8,trace,8);
    change(s9,trace,9);
    left=Least_S(trace[0],trace[1],trace[2],trace[3]);
    right=Least_S(trace[6],trace[7],trace[8],trace[9]);	
    down=Least_H(s[0],s[1],s[2]);	
    up=Least_H(s[7],s[8],s[9]);	
	}

  switch(haveBall)
	{
		case 0:
		{
			if(run<2)
			{
				First_Scan();
			}
      else 
			{
				
			}
	  }break;				 
		case 1:
	  {
       ClLineAngle((Position_t.angle+ballAngle),800);
		}break;
	  default:
		 break;
	}	

	
	//走有最多球的地方
	/*
	if(g_camera == 2)
	{
		VelCrl(CAN1, 1,  500*SP2PULSE);
		VelCrl(CAN1, 2, -500*SP2PULSE);
	}
	else if(g_camera == 1)
	{
		VelCrl(CAN1, 1,  500*SP2PULSE - 200);
		VelCrl(CAN1, 2, -500*SP2PULSE + 200);
	}
	else if(g_camera == 3)
	{
		VelCrl(CAN1, 1,  500*SP2PULSE + 200);
		VelCrl(CAN1, 2, -500*SP2PULSE - 200);
	}
	else
	{
		VelCrl(CAN1, 1,  200*SP2PULSE);
		VelCrl(CAN1, 2, -200*SP2PULSE);
	}
	*/
}
