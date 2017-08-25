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
#include "stm32f4xx_gpio.h"

/*==============================================全局变量声明区============================================*/
extern POSITION_T Position_t;			//校正后定位
extern POSITION_T getPosition_t;	//获得的定位
extern int g_plan;
extern int g_camera;
float angleError = 0,xError = 0,yError = 0;
int cameraScheme=0;
int shootStart,ballColor;



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
			VelCrl(CAN2,1,  (int)(speed * SP2PULSE) + g_plan * (Ainput + Dinput));
			VelCrl(CAN2,2, -(int)(speed * SP2PULSE) + g_plan * (Ainput + Dinput));
		}
		else
		{
			VelCrl(CAN2,1,  (int)(speed * SP2PULSE) + g_plan * (Ainput - Dinput));
			VelCrl(CAN2,2, -(int)(speed * SP2PULSE) + g_plan * (Ainput - Dinput));
		}
		
}
extern int carRun;
/*======================================================================================
函数定义	：		开始跑场
函数参数	：		方案：暂定1为逆时针(右侧激光触发)，-1为顺时针(左侧激光触发)
函数返回值：		无
=======================================================================================*/
void GoGoGo(void)
{
	static int state = 1,shootTime=0;							//应该执行的状态
	static int length = WIDTH/2,wide = WIDTH/2;		//长方形跑场参数
	switch(state)
	{
		//第一圈放球区附近跑场
		case 1:
		{
			shootStart=0;
			carRun=1;
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
				if(length >= 1700 - WIDTH - 100) length = 1700 - WIDTH - 100;
				if(wide		>= 2125 - WIDTH - 100) wide		= 2125 - WIDTH - 100;
			}
			if(length >= 1700 - WIDTH - 100 && wide >= 2125 - WIDTH - 100)
				state = 3;
		}break;

		//进行坐标校正
		case 3:
		{
			carRun=0;
			if(CheckPosition())
			{
				state=4;
			}
		}break;
		case 4:
		{
			carRun=0;
			shootStart=1;
			if(ShootBall())
			{
		    shootStart=0;
				shootTime++;
				switch(shootTime)
				{
					case 1:
					{   
						state=5;
						if(cameraScheme==0)
						{
							cameraScheme=1;
							GPIO_ResetBits(GPIOE,GPIO_Pin_4);
							GPIO_SetBits(GPIOE,GPIO_Pin_6);							
						}
//						else if(cameraScheme==1)
//						{
//							cameraScheme=2;
//							GPIO_SetBits(GPIOE,GPIO_Pin_4);
//							GPIO_SetBits(GPIOE,GPIO_Pin_6);
//						}
						else if(cameraScheme==1)
						{
							cameraScheme=3;
							GPIO_SetBits(GPIOE,GPIO_Pin_4);
							GPIO_ResetBits(GPIOE,GPIO_Pin_6);
						}
						else{}
						
					}break;
					case 2:state=6;break;
					default:break;	
      			}				
			}
			else
			{
				
			}
		}break; 
		case 5:
		{
			carRun=1;
			if(RunCamera())
			{
				state=3;
			}			
		}break;
		case 6:
		{ 
			carRun=1;
			if(RunEdge())
			{
				state=3;
				shootTime=0;
			}
			
		}break;
		default:
		 break;
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
				StaightCLose((275 + WIDTH/2 + 100),0,0,speed);
				if(Position_t.Y >= 3100 + WIDTH/2 - FIR_ADV)
					state = 2;
			}break;
			
			//上边，目标角度90度
			case 2:
			{
				StaightCLose(0,3100 + WIDTH/2 + 100,90,speed);
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
函数定义		：			判断小车是否卡住不动，
函数参数		：			无
函数返回值	：			false 未卡住，true卡住了
=======================================================================================*/
bool IfStuck(void)
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
int x1,x2,y1,y2;
/*======================================================================================
函数定义		：			坐标校正
函数参数		：			无
函数返回值	：			1:           已完成矫正
                    0:           未完成矫正
=======================================================================================*/
int CheckPosition(void)
{
	static int state = 1;
	static int tempx = 0,tempy = 0;
	int keepgo=0;
	switch(state)
	{
		//后退到 y = 0
		case 1:
		{
			StaightCLose(0,500,-90,-500);
			if(Position_t.X <= 1000)				//在y = 0处两个激光不容易射到球或射出去
			{
				state = 2;
			}
		}break;
		
		//原地旋转至0度
		case 2:
		{
			TurnAngle(0,5000);
			if(fabs(Position_t.angle)<=5)
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
//			if(IfStuck2())								//到时候改成两个行程开关被触发
//			{
//		//		USART_OUT(USART1,(uint8_t*) "case = 4\r\n");
//				state = 4;
//			}
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==0)
			{
				state = 4;
			}
		}break;
		
		//激光校正
		case 4:
		{
			if(LaserCheck())
			{
				keepgo=1;
				state = 1;
	      tempx = 0,tempy = 0;
			}								
		//		state = 5;	//矫正成功，开始第二阶段跑场
			else							
				state = 6;	//矫正失败，继续矫正
		}break;
		
//		//第二阶段跑场
//		case 5:
//		{
//			
//			if(RunRectangle(1000,800,1500) == 1)
//				state = 1;
//		}break;
//		
		//继续矫正
		//前进
		case 6:
		{
			 	VelCrl(CAN2, 1, 8000);
	      VelCrl(CAN2, 2, -8000);
			  if(Position_t.Y>=1000)
					state=7;
		}break;
		//转向90度
		case 7:
		{
			TurnAngle(90,5000);	
			if(Position_t.angle>=85&&Position_t.angle<=95)
			{
				tempx = Position_t.X;			//记录当前坐标用于闭环后退，防止角度被撞歪后开环后退不准
				tempy = Position_t.Y;
				state = 8;
			}
		}break;
		//后退
		case 8:
		{
			StaightCLose(tempx,tempy,90,-800);
			if(GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)==0&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==0)
			{
				 x2=getPosition_t.X;
		     y2=getPosition_t.Y;
				 xError=2400-POSYSTEM_TO_BACK-x2*cos(ANGTORAD(angleError))-y2*sin(ANGTORAD(angleError));
				 yError=-y1*cos(ANGTORAD(angleError))+x1*sin(ANGTORAD(angleError));	
         keepgo=1;	
				 state = 1;
	       tempx = 0,tempy = 0;				
			}
		}break;
		
		
	}
	return keepgo;
}

//方案1 发三个区域的球数
extern int ballN_L,ballN_M,ballN_R;
//方案2 发球数最多的那个角度
extern float bestAngle;
//方案3 最近球的极坐标
extern float nearestAngle,nearestDis;
//方案4 所有球的角度和距离
extern int8_t arr1[20];
extern uint8_t arr2[20];
extern int go,arr_number;
/*======================================================================================
函数定义		：			利用摄像头跑场
                    根据场上球数来走不同方案：                    
                    1：（球很多时）  先判断球数最多的那个方向 再转到那个角度
                    2：（球中等时）  规划一条最优路线 尽可能吃到更多的球
                    3：（球少时）    每次的目标都是最近的球
函数参数		：			
函数返回值	：			1:               已完成一次摄像头扫场
                    0:               还未完成
=======================================================================================*/
int	RunCamera(void)
{
	static int i,d1=0,d2=0,haveBall=0,run=0,ballAngle,traceH[10][10]={0},traceS[10][10]={0},stagger=0,left=1,right=1,up=1,down=1;
  static float cameraX,cameraY,TraceX[20],TraceY[20],bestTraceX[20],bestTraceY[20],bestTraceAngle[20],bestSum;
	int finish=0;
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
	traceH[Zoning(Position_t.X,Position_t.Y).hor][Zoning(Position_t.X,Position_t.Y).ver]=1;
	traceS[Zoning(Position_t.X,Position_t.Y).ver][Zoning(Position_t.X,Position_t.Y).hor]=1;
	//扫描一下是否全已走过
	if(ScanTrace(traceH))
	{
		finish=1;run=0;stagger=0;
	}		
	//计算走过的路线 寻找一条错开的路线
	if(run>0&&((int)(run/2)+(int)(run/2))==run&&stagger==1)
	{ 
		stagger=0;
		left=Least_S(traceH[0],traceH[1],traceH[2],traceH[3]);
		right=Least_S(traceH[6],traceH[7],traceH[8],traceH[9]);	
		down=Least_H(traceS[0],traceS[1],traceS[2]);	
		up=Least_H(traceS[7],traceS[8],traceS[9]);	
	}
	
	switch(cameraScheme)
	{
		case 1: 
 		{
			if(go)
			{
				go=0;
				if(arr_number==0)		
				{
					haveBall=0;
				}
				else  
				{
					haveBall=1;
					cameraX=Position_t.X-CAMERATOGYRO*sin(Position_t.angle);
					cameraY=Position_t.Y+CAMERATOGYRO*cos(Position_t.angle);	
					ballAngle=AvoidOverAngle(Position_t.angle+bestAngle);			  
				}				
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
						New_Route(down,right,up,left);
					}
				}break;				 
				case 1:
				{
						StaightCLose(cameraX, cameraY,ballAngle,800);
				}break;
				default:
				 break;
			}	
		}break;
		case 2:
		{		
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
					cameraX=Position_t.X-CAMERATOGYRO*sin(Position_t.angle);
					cameraY=Position_t.Y+CAMERATOGYRO*cos(Position_t.angle);						
					bestAngle=MostSector();
					Left2Right();
					arr1[0] -= asin(200/arr2[0]);
          TraceX[0]=cameraX-arr2[0]*sin(Position_t.angle+arr1[0]);
					TraceY[0]=cameraY+arr2[0]*sin(Position_t.angle+arr1[0]);					
					arr1[(arr_number-1)] += asin(200/arr2[(arr_number-1)]);
          TraceX[1]=cameraX-arr2[(arr_number-1)]*sin(Position_t.angle+arr1[(arr_number-1)]);
					TraceY[1]=cameraY+arr2[(arr_number-1)]*sin(Position_t.angle+arr1[(arr_number-1)]);							
					Down2Up();
					for(i=2;i<arr_number;i++)
					{
						TraceX[i]=cameraX-arr2[i-1]*sin(Position_t.angle+arr1[i-1]);
						TraceY[i]=cameraY+arr2[i-1]*sin(Position_t.angle+arr1[i-1]);										
					}
					for(i=1;i<(arr_number-1);i++)
					{
						if(arr2[i]<=arr2[0])
						{
							d1++;
						}
						if(arr2[i]<=arr2[(arr_number-1)])
						{
							d2++;
						}
					}
					if(d1>d2)
					{
						if(d2==1)
						{
							bestTraceX[0]=TraceX[2];bestTraceY[0]=TraceY[2];
              bestTraceX[1]=TraceX[1];bestTraceY[1]=TraceY[1];
              if(d1==2)
							{
								bestTraceX[2]=TraceX[3];bestTraceY[2]=TraceY[3];
								bestTraceX[3]=TraceX[0];bestTraceY[3]=TraceY[0];
								
							  for(i=4;i<(arr_number-1);i++)
							  {
								  if(P2P(TraceX[i],TraceY[i],TraceX[i+1],TraceY[i+1])<450)
								  {
									   bestTraceX[i]=(TraceX[i]+TraceX[i+1])/2;
								     bestTraceY[i]=(TraceY[i]+TraceY[i+1])/2;
							  	}
                  else 
								  {
								  	if(fabs(arr1[i-1]-arr1[i-2])<fabs(arr1[i]-arr1[i-2]))
								  	{
									  	bestTraceX[i]=TraceX[i];
									  	bestTraceY[i]=TraceY[i];
									  }
									  else
									  {
									  	bestTraceX[i]=TraceX[i+1];
								  		bestTraceY[i]=TraceY[i+1];										
								  	}
							  	}
							  }              							
						  }									
              else 
							{
								for(i=2;i<d1;i++)
								{
									if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
									{
										bestTraceX[i]=(TraceX[i+1]+TraceX[i+2])/2;
										bestTraceY[i]=(TraceY[i+1]+TraceY[i+2])/2;
									}
									else 
									{
										if(fabs(arr1[i]-arr1[i-1])<fabs(arr1[i+1]-arr1[i-1]))
										{
											bestTraceX[i]=TraceX[i+1];
											bestTraceY[i]=TraceY[i+1];
										}
										else 
										{
											bestTraceX[i]=TraceX[i+2];
											bestTraceY[i]=TraceY[i+2];										
										}
									}
							  }
								bestTraceX[d1]=TraceX[0];
							  bestTraceY[d1]=TraceY[0];											
							  for(i=(d1+1);i<(arr_number-2);i++)
							  {
								  if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								  {
								  	 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								     bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								  }
                  else 
								  {
									  if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									  {
										  bestTraceX[i]=TraceX[i+2];
										  bestTraceY[i]=TraceY[i+2];
									  }
									  else
									  {
										  bestTraceX[i]=TraceX[i+3];
										  bestTraceY[i]=TraceY[i+3];										
									  }
								  }
							 }              							
						 }
			    }
            else if(d2==0)
						{
							bestTraceX[0]=TraceX[1];bestTraceY[0]=TraceY[1];
							
						}
						else 
						{
							for(i=0;i<(d2-1);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i+3])<fabs(arr1[i+2]-arr1[i+3]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else 
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
							bestTraceX[d2-1]=TraceX[1];
							bestTraceY[d2-1]=TraceY[1];
							for(i=d2;i<(d1-1);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else 
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
							bestTraceX[d1-1]=TraceX[0];
							bestTraceY[d1-1]=TraceY[0];
							for(i=d1;i<(arr_number-3);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
						}
				
					}
					else if(d1==d2)
					{
						if(d1==0)
						{
							if(bestAngle>0)
							{
								  bestTraceX[0]=TraceX[0];
									bestTraceY[0]=TraceY[0];
							}
							else 
							{
									bestTraceX[0]=TraceX[1];
									bestTraceY[0]=TraceY[1];									
							}						
							for(i=1;i<(arr_number-4);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
						}
            else
						{
							for(i=0;i<(d2-1);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i+3])<fabs(arr1[i+2]-arr1[i+3]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else 
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
							if(P2P(TraceX[0],TraceY[0],TraceX[1],TraceY[1])<450)
							{
								bestTraceX[d2-1]=(TraceX[0]+TraceX[1])/2;
								bestTraceY[d2-1]=(TraceY[0]+TraceY[1])/2;
							}
							else 
							{
								if(bestAngle>0)
								{
									bestTraceX[d2-1]=TraceX[0];
									bestTraceY[d2-1]=TraceY[0];
								}
								else 
								{
									bestTraceX[d2-1]=TraceX[1];
									bestTraceY[d2-1]=TraceY[1];									
								}
							}
							for(i=d1;i<(arr_number-4);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}							
						}
					}
										
          else 
					{
						if(d1==0)
						{
							bestTraceX[0]=TraceX[2];bestTraceY[0]=TraceY[2];bestTraceAngle[0]=arr1[1];		
							for(i=d2;i<(d1-1);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else 
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
							bestTraceX[d1-1]=TraceX[0];
							bestTraceY[d1-1]=TraceY[0];
							for(i=d1;i<(arr_number-3);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}              							
						}
						else 
						{
							for(i=0;i<(d2-1);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i+3])<fabs(arr1[i+2]-arr1[i+3]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else 
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
							bestTraceX[d2-1]=TraceX[1];
							bestTraceY[d2-1]=TraceY[1];
							for(i=d2;i<(d1-1);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else 
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
							bestTraceX[d1-1]=TraceX[0];
							bestTraceY[d1-1]=TraceY[0];
							for(i=d1;i<(arr_number-3);i++)
							{
								if(P2P(TraceX[i+2],TraceY[i+2],TraceX[i+3],TraceY[i+3])<450)
								{
									 bestTraceX[i]=(TraceX[i+2]+TraceX[i+3])/2;
								   bestTraceY[i]=(TraceY[i+2]+TraceY[i+3])/2;
								}
                else 
								{
									if(fabs(arr1[i+1]-arr1[i])<fabs(arr1[i+2]-arr1[i]))
									{
										bestTraceX[i]=TraceX[i+2];
										bestTraceY[i]=TraceY[i+2];
									}
									else
									{
										bestTraceX[i]=TraceX[i+3];
										bestTraceY[i]=TraceY[i+3];										
									}
								}
							}
						}
							
				
				  }
			  }
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
						New_Route(down,right,up,left);
					}
				}break;				 
				case 1:
				{
						StaightCLose(cameraX, cameraY,ballAngle,800);
				}break;
				default:
				 break;
			}	
		}break;
		case 3:
		{
			if(go)
			{
				go=0;
				if(arr_number==0)		
				{
					haveBall=0;
				}
				else  
				{
					haveBall=1;
					cameraX=Position_t.X-CAMERATOGYRO*sin(Position_t.angle);
					cameraY=Position_t.Y+CAMERATOGYRO*cos(Position_t.angle);
			    if(Vehicle_Width(nearestDis,nearestAngle))
					{
						ballAngle=AvoidOverAngle(Position_t.angle);
					}
					else 
					{
						ballAngle=AvoidOverAngle(Position_t.angle+nearestAngle);
				  }				
			  }
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
						New_Route(down,right,up,left);
					}
				}break;				 
				case 1:
				{
						StaightCLose(cameraX, cameraY,ballAngle,800);
				}break;
				default:
				 break;
			}	
		}break;		
		default:
		 break;
	} 
	return finish;
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
		VelCrl(CAN2,2, g_plan * speed);
		VelCrl(CAN2,1, g_plan * speed);
	}
	
	//角度小于10，pid控制输出
	else
	{
		VelCrl(CAN2,2, g_plan * Input);
		VelCrl(CAN2,1, g_plan * Input);
	}
}

/*======================================================================================
函数定义		：			利用激光矫正坐标
函数参数		：			无
函数返回值	：			矫正成功返回true，矫正不成功记录角度与y的误差后返回false
=======================================================================================*/
bool	LaserCheck()
{
	int laserGet,laserLong;
	laserLong=Get_Adc_Average(RIGHT_LASER,20)+Get_Adc_Average(LEFT_LASER,20);
	if(laserLong>4780&&laserLong<4820)
	{
		angleError = angleError + Position_t.angle;		//纠正角度坐标
	  yError = (getPosition_t.Y*cos(Angel2PI(angleError))+getPosition_t.X*sin(Angel2PI(angleError)));
	  laserGet = (Get_Adc_Average(LEFT_LASER,20)-Get_Adc_Average(RIGHT_LASER,20))/2;
	  xError = (getPosition_t.X*cos(Angel2PI(angleError))-getPosition_t.Y*sin(Angel2PI(angleError)))-laserGet;//纠正X坐标
		return true;
	}
  else 
	{
		x1=getPosition_t.X;
		y1=getPosition_t.Y;
		return false;
	}		
}
 
//角度变换函数
float Angel2PI(float angel)
{
	float res;
	res = PI*(angel) / 180;
	return res;
}


/*======================================================================================
函数定义	  ：    射球函数(shiling加的)
函数参数	  ：    
                  
                           
函数返回值  ：	  无
=======================================================================================*/
int ShootBall(void)
{
	  float horizonDis_W=2400,horizonDis_B=2400,shoot_PX,shoot_PY,tendAngle_W=90,tendAngle_B=90;
	  static int tim=0,noball=0,rev;
    int finish=0;
	  tim++;	  
	  if(tim<=100)
		{
			PushBallReset();			
		}
		if(tim<200&&tim>100)
		{
			PushBall();
		}
		if(tim>=200)
		{
			tim=0;			
		}
//	  if(fabs(Position_t.angle)<1)
//	  {
//		  shoot_PX =(Get_Adc_Average(LEFT_LASER,20)-Get_Adc_Average(RIGHT_LASER,20))/2;
//		  shoot_PY = POSYSTEM_TO_GUN;
//		  horizonDis_W = sqrt(PF(shoot_PX-BASKE_LOCATION_WX) + PF(shoot_PY-BASKE_LOCATION_WY));
//			tendAngle_W = atan2((BASKE_LOCATION_WY-shoot_PY),(BASKE_LOCATION_WX-shoot_PX));
//			tendAngle_W = RADTOANG(tendAngle_W);
//			horizonDis_B = sqrt(PF(shoot_PX-BASKE_LOCATION_BX) + PF(shoot_PY-BASKE_LOCATION_BY));
//			tendAngle_B = atan2((BASKE_LOCATION_BY-shoot_PY),(BASKE_LOCATION_BX-shoot_PX));
//			tendAngle_B = RADTOANG(tendAngle_B);
//	  }
//		if(Position_t.angle>89&&Position_t.angle<91)
//		{
//			shoot_PX = 2400 - GUN_TO_BACK;
//			shoot_PY = (Get_Adc_Average(LEFT_LASER,20)-Get_Adc_Average(RIGHT_LASER,20))/2+2400;
//		  horizonDis_W = sqrt(PF(shoot_PX-BASKE_LOCATION_WX) + PF(shoot_PY-BASKE_LOCATION_WY));
//			tendAngle_W = atan2((shoot_PX-BASKE_LOCATION_WX),(BASKE_LOCATION_WY-shoot_PY));
//			tendAngle_W = RADTOANG(tendAngle_W);
//			horizonDis_B = sqrt(PF(shoot_PX-BASKE_LOCATION_BX) + PF(shoot_PY-BASKE_LOCATION_BY));
//			tendAngle_B = atan2((shoot_PX-BASKE_LOCATION_BX),(BASKE_LOCATION_BY-shoot_PY));
//			tendAngle_B = RADTOANG(tendAngle_B);			
//		}
//	  if(fabs(Position_t.angle)>179)
//		{
//			shoot_PX = -(Get_Adc_Average(LEFT_LASER,20)-Get_Adc_Average(RIGHT_LASER,20))/2;
//		  shoot_PY = 4800 - POSYSTEM_TO_BACK - GUN_TO_BACK;	
//		  horizonDis_W = sqrt(PF(shoot_PX-BASKE_LOCATION_WX) + PF(shoot_PY-BASKE_LOCATION_WY));
//			tendAngle_W = atan2((shoot_PY-BASKE_LOCATION_WY),(shoot_PX-BASKE_LOCATION_WX));
//			tendAngle_W = RADTOANG(tendAngle_W);
//			horizonDis_B = sqrt(PF(shoot_PX-BASKE_LOCATION_BX) + PF(shoot_PY-BASKE_LOCATION_BY));
//			tendAngle_B = atan2((shoot_PY-BASKE_LOCATION_BY),(shoot_PX-BASKE_LOCATION_BX));
//			tendAngle_B = RADTOANG(tendAngle_B);      			
//		}
//	  if(Position_t.angle>-91&&Position_t.angle<-89)
//		{
//			shoot_PX = - 2400 + GUN_TO_BACK;
//			shoot_PY = 2400 - (Get_Adc_Average(LEFT_LASER,20)-Get_Adc_Average(RIGHT_LASER,20))/2;
//		  horizonDis_W = sqrt(PF(shoot_PX-BASKE_LOCATION_WX) + PF(shoot_PY-BASKE_LOCATION_WY));
//			tendAngle_W = atan2((BASKE_LOCATION_WX-shoot_PX),(shoot_PY-BASKE_LOCATION_WY));
//			tendAngle_W = RADTOANG(tendAngle_W);
//			horizonDis_B = sqrt(PF(shoot_PX-BASKE_LOCATION_BX) + PF(shoot_PY-BASKE_LOCATION_BY));
//			tendAngle_B = atan2((BASKE_LOCATION_BX-shoot_PX),(shoot_PY-BASKE_LOCATION_BY));
//			tendAngle_B = RADTOANG(tendAngle_B);
//		}	
		
		if(ballColor==0)
		{
			noball++;
		}
		else if(ballColor==1)
		{
			rev=60*sqrt(((horizonDis_W-1500)/2+900)/900);
			YawAngleCtr(90-tendAngle_W);
			ShootCtr(80);
		}
		else if(ballColor==2)
		{
			rev=60*sqrt(((horizonDis_B-1500)/2+900)/900);
			YawAngleCtr(90-tendAngle_B);
			ShootCtr(80);			
		}
		else 
		{
			
		}
			
		if(noball>=500)
		{
			noball=0;
			finish=1;
		}
		
	  return finish;
}
