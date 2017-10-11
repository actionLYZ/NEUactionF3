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
#include "gpio.h"
#include "wan.h"
#include "stm32f4xx_it.h"
#include "c0.h"
#include "stm32f4xx_gpio.h"
#include "MotionCard.h"
#include "stdlib.h"
#include "string.h"

/*==============================================全局变量声明区============================================*/
extern POSITION_T Position_t;     //校正后定位
extern POSITION_T getPosition_t;  //获得的定位
extern int        g_plan, g_camera, bestSum;
float             angleError = 0, xError = 0, yError = 0;
int               cameraScheme = 0;
extern int        shootStart, ballColor;
extern int         lastPlan;
extern int32_t     g_rightPulse ;
extern int32_t     g_leftPulse ;
extern float       firstLine;
extern int ballNumber;
extern int32_t     g_pushPosition;
extern uint8_t     shootNum;
static bool ifPrintPositionCheck = true;
extern char g_carState[50];
int leftfirst=2400,rightfirst=2400;
extern float carDeVel;
extern int triggerTime,beginTrigger;
extern int shootBegin;
extern uint8_t     blueToothError;
extern int resetStep;
int notcount=0;
extern int staticShoot;
/*================================================函数定义区==============================================*/

/*======================================================================================
   函数定义	：		判断小车状态并输出日志
   函数参数	：		即将进入的状态
	 其他：当小车已经处于此状态时，不输出当前状态
   =======================================================================================*/

void JudgeState(char state[])
{
	//如果当前状态不为state
	if(strcmp(g_carState,state))
	{
		USART_OUT(UART5,(u8*)"==============================================================\r\n");
		USART_OUT(UART5,(u8*)"note%s:\t%s\r\n",__TIME__,state);
		USART_OUT(UART5,(u8*)"==============================================================\r\n");
	}
	strcpy(g_carState,state);
}



/*======================================================================================
   函数定义	：		通过激光判断是否开始
   函数参数	：		无
   函数返回值：		0	:为激光未触发
                1 :为右侧激光触发,逆时针
               -1 :为左侧激光触发,顺时针
   =======================================================================================*/
int IfStart(void)
{
	u16 right = 0, left = 0;
	int success=0;
	right = Get_Adc_Average(RIGHT_LASER, 30);
	left = Get_Adc_Average(LEFT_LASER, 30);
//	USART_OUT(UART5,(u8*)"r%d\tl%d\r\n",(int)right,(int)left);
	
//	USART_OUT(UART5,(u8*)"r %d\tl %d\r\n",(int)right,(int)left);
	if((leftfirst+rightfirst)>=4700)
	{
		if (right < 1000)     //右侧激光触发
		{
			beginTrigger=1;
			if(triggerTime>50)
			{
				success= 1;
			}
			
		}
			
		else if (left < 1000) //左侧激光触发
		{
			beginTrigger=1;
			if(triggerTime>50)
			{
				success= -1;
			}
		}
			
		else
		{
			beginTrigger=0;
			success= 0;
		}
					
	}
	else if((leftfirst+rightfirst)<4700&&leftfirst<700)
	{
		if (right < 1000)     //右侧激光触发
		{
			beginTrigger=1;
			if(triggerTime>50)
			{
				success= 1;
			}
		}
		else
		{
			beginTrigger=0;
			success= 0;
		}		
	}
	else if((leftfirst+rightfirst)<4700&&rightfirst<700)
	{
		beginTrigger=1;
		if(triggerTime>50)
		{
			success= -1;
		}
			
		else
		{
			beginTrigger=0;
			success= 0;
		}		
	}  
	else if((leftfirst+rightfirst)<4700&&leftfirst<700&&rightfirst<700)
	{
		if (SWITCHE2 == 1 || SWITCHC0 == 1)     
		{
			beginTrigger=1;
			if(triggerTime>50)
			{
				success= 1;
			}
		}
			
		else
		{
			beginTrigger=0;
			success= 0;
		}					
	}
//	 USART_OUT(UART5,(u8*)"%d\t%d\r\n",(int)beginTrigger,(int)triggerTime);
	return success;
}


/*======================================================================================
   函数定义		：		计算点到直线的距离
   函数参数		：		直线上一点x，y坐标，直线角度
   函数返回值	：		返回距离值
   =======================================================================================*/
float Piont2Straight(float aimx, float aimy, float angle)
{
	float dis;  //返回值
	float k;    //斜率

	if (angle == 0)
	{
		dis = Position_t.X - aimx;
	}
	else if (angle == 180)
	{
		dis = aimx - Position_t.X;
	}
	else
	{
		k   = tan((PI * (angle + 90)) / 180);
		dis = (k * Position_t.X - Position_t.Y - k * aimx + aimy) / sqrt(1 + k * k);
		if (angle > 0 && angle < 180)
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
void StaightCLose(float aimx, float aimy, float angle, float speed)
{
	float         Ddis, Dangle;     //距离与角度差
	float         Dinput, Ainput;   //距离与角度PID计算结果

	//计算距离输出
	Ddis = Piont2Straight(aimx, aimy, angle);

	// if (fabs(speed) <= 500)
	// 	Dinput = 9 * Ddis; 
	// else if (fabs(speed) <= 1000)
	// 	Dinput = 12 * Ddis ;
	// else if (fabs(speed) <= 1500)
	// 	Dinput = 18 * Ddis;
	// else
	// 	Dinput = 15 * Ddis;
	
		Dinput = 25 * Ddis;


	//计算角度输出
	Dangle = (angle - Position_t.angle);
	if (Dangle > 180)
		Dangle -= 360;
	if (Dangle < -180)
		Dangle += 360;

	// if (fabs(speed) <= 500)
	// 	Ainput = 100 * Dangle;
	// else if (fabs(speed) <= 1000)
	// 	Ainput = 160 * Dangle;
	// else if (fabs(speed) <= 1500)
	// 	Ainput = 250 * Dangle;
	// else
	// 	Ainput = 260 * Dangle;
	if(speed < 1500)
	{
		Ainput = 260 * Dangle;
	}
	else
	{
		Ainput = 400 * Dangle;
	}


	//计算脉冲
	if (speed >= 0)
	{
		VelCrl(CAN2, 1, (int)(speed * SP2PULSE) + g_plan * (Ainput + Dinput));
		VelCrl(CAN2, 2, -(int)(speed * SP2PULSE) + g_plan * (Ainput + Dinput));
	}
	else
	{
		VelCrl(CAN2, 1, (int)(speed * SP2PULSE) + g_plan * (Ainput - Dinput));
		VelCrl(CAN2, 2, -(int)(speed * SP2PULSE) + g_plan * (Ainput - Dinput));
	}
}
extern int carRun,fighting;
int changeState=0;
/*======================================================================================
   函数定义	：		开始跑场
   函数参数	：		方案：暂定1为逆时针(右侧激光触发)，-1为顺时针(左侧激光触发)
   函数返回值：		无
   =======================================================================================*/

void GoGoGo(float fLine,int stat)
{
	static int  state = 10, shootTime = 0, count = 0,full=0,laserLeft = 0, laserRight = 0,time = 0,hitNum = 0,side = 0; //应该执行的状态
	static int  length = WIDTH / 2, wide = WIDTH / 2; //长方形跑场参数
  static float aimAngle = 0,tempx = 0,tempy = 0;
	static int leftCar = 0,rightCar = 0;
//	if(ballNumber>35&&full==0)
//	{
//		state=4;
//	  full=1;
//	}
	if(changeState==1)
	{
		changeState=0;
		shootTime=0;
		state = stat;
	}
	if(carRun)
	{
//		USART_OUT(UART5,(u8*)"CarV%d\r\n",(int)carDeVel);
		if (stuckCar(70,300))
		{
			side = JudgeSide();
			if(side == 1)
			{
				aimAngle = 0;
			}
			else if(side == 2)
			{
				aimAngle = 90;
			}
			else if(side == 3)
			{
				aimAngle = 180;
			}
			else
			{
				aimAngle = -90;
			}
			state = 11;
		}
	}	
	switch (state)
	{
		//第一圈放球区附近跑场
		case 1:
		{
			g_plan = lastPlan;
			shootBegin = 1;
			notcount=0;
			//先启动3s
			count++;
			if(count >= 300)
			{
				shootStart  = 0;
				carRun      = 1;
				count = 300;
			}
			LOG_NOTE JudgeState("FirstRound running....");//开始第一圈跑场
			if (FirstRound(firstLine) == 1)
			{
				//初始化长方形跑场参数
				length  += SPREAD_DIS;
				wide    += SPREAD_DIS;
				count   = 0;
				state   = 2;
			}
		}
		break;

		//向外扩散扫场
		case 2:
		{
			shootBegin = 1;
			notcount=0;
			g_plan = lastPlan;
			LOG_NOTE JudgeState("Circle running....");//开始第一圈跑场
			carRun = 1;
	//		if (RunRectangle(length, wide, RUN_SPEED))
	//		{
	//			//逐渐增加长方形跑场参数
	//			length  += SPREAD_DIS;
	//			wide    += SPREAD_DIS;
	//			if (length >= 1700 - WIDTH / 2 - 100)
	//				length = 1700 - WIDTH / 2 - 100;
	//			if (wide >= 2125 - WIDTH / 2 - 100)
	//				wide = 2125 - WIDTH / 2 - 100;
	//		}
	//		if (length >= 1700 - WIDTH / 2 - 100 && wide >= 2125 - WIDTH / 2 - 100)
			if(sweepYuan(1800, 1200, 2, 1))
			{
				state = 3;
			}
		}
		break;
		
		//紧随画圆后矩形扫场
		case 3:
		{
			shootBegin = 1;
			notcount=0;
			LOG_NOTE JudgeState("Rectangle running....");//开始第一圈跑场
			carRun = 1;
			g_plan = lastPlan;
			if(AfterCircle(1800))
			{
				//之后的矫正坐标函数令g_plan = 1;
				g_plan = 1;
				state = 4;
			}
		}
			break;
		//进行坐标校正
		case 4:
		{
			shootBegin = 0;
			notcount=0;
			carRun = 0;
			count=0;
			g_plan = 1;
			if (CheckPosition())
			{
				//如果蓝牙坏了,矫正完成后，进入state12,准备倒车进入出发区
				if(blueToothError)
				{
					//计算当前坐标和(0,1000)目标点坐标的角度，作为目标角度
					aimAngle = AvoidOverAngle(RADTOANG(atan2(1000 - Position_t.Y,-Position_t.X)) - 90);
					state = 12;
				}
				else
				{
					state = 5;
				}
				VelCrl(CAN2, 1, 0);
				VelCrl(CAN2, 2, 0);
			}
				
		} break;
		case 5:
		{
			shootBegin = 0;
			carRun      = 0;
			shootStart  = 1;
			VelCrl(CAN2, 1, 0);
			VelCrl(CAN2, 2, 0);
			
			
			//10ms检测一次激光
      time++;
			time = time % 10;
			if(time == 1)
			{
				POS_NOTE USART_OUT(UART5,(u8*)"%d\r\n",(int)(laserRight - Get_Adc_Average(RIGHT_LASER, 100)));
				if((laserRight - Get_Adc_Average(RIGHT_LASER, 100)) > 15 || (laserLeft - Get_Adc_Average(LEFT_LASER, 100)) > 15)
				{
					count++;
					if(count > 5)
					{
						hitNum++;
						tempx = Position_t.X;
						tempy = Position_t.Y;
						aimAngle = Position_t.angle;
						state = 9;
						count = 0;
					}
				}
				else
				{
					count = 0;
				}
				
				//检测到左右侧是否有车来
//				if((laserRight - Get_Adc_Average(RIGHT_LASER, 100)) > 100 )
//				{
//					state = 14;
//					rightCar = 1;
//				}
//				if((laserLeft - Get_Adc_Average(LEFT_LASER, 100)) > 100)
//				{
//					state = 14;
//					leftCar = 1;
//				}
				laserLeft =  Get_Adc_Average(LEFT_LASER, 100);
				laserRight = Get_Adc_Average(RIGHT_LASER, 100);
			}
			if (ShootBallW())
			{			
				CollectBallVelCtr(60);
				shootStart = 0;
				if(full==0)
				{
					shootTime++;
				}
				switch (shootTime)
				{
					case 3:
					{
						state = 6;
					} 
					break;

					case 1: 
						state = 7;
					break;						
					case 0:
					{
						state=7;
						shootTime=1;
					}break;
					case 2:
					{
						state=6;
					}break;
					default: 
					{
						state = 6;
					}
					break;
				}
	       
				//球数等于进的减去射出去的
        ballNumber=ballNumber-shootNum;
				photoElectricityCount=photoElectricityCount-shootNum;
				shootNum=0;
				full=0;
			}
			else
			{
			}
		} break;

		case 6: 
		{
			shootBegin = 1;
			notcount=0;
			//先启动3s
			count++;
			if(count >= 300)
			{
				shootStart  = 0;
				carRun      = 1;
				count = 300;
			}
			//	GPIO_SetBits(GPIOE,GPIO_Pin_7);
			if(RunCamera())
			{
				count = 0;
				state = 4;
			}
		} break;

		case 7:
		{
			shootBegin = 1;
			notcount=0;
			count++;
			if(count >= 300)
			{
				shootStart  = 0;
				carRun      = 1;
				count = 300;
			}
			if (RunEdge())
			{
				state     = 4;
				//shootTime = 0;
				count = 0;
			}
		} break;
		
		case 8:
		{
			shootBegin = 1;
			 count++;
			 RunWithCamera1(2);
			 if(count>=300)
			 {
			 	carRun=1;
			 	count=0;
			 }
		}
		case 9:
		{
			notcount=0;
			//第一次有车来
			if(hitNum == 1)
			{
				count++;
				if(sqrt(PF(Position_t.X - tempx) + PF(Position_t.Y - tempy)) > 800 || count > 100)
				{
					//此时让车静止射球
					staticShoot = 1;
					count = 0;
					state = 5;
				}
				angClose(1000,aimAngle,150);
			}
			else if(hitNum == 2)
			{
				count++;
				if(sqrt(PF(Position_t.X - tempx) + PF(Position_t.Y - tempy)) > 800 || count > 100)
				{
					staticShoot = 0;
					hitNum = 0;
					count = 0;
					state = 4;
				}
				angClose(-1000,aimAngle,150);
			}
			else
			{
//				POS_NOTE	USART_OUT(UART5,(u8*)"hitNum3");
			}
		}
			break;
		case 10:
		{
       notcount=0;
			 count++;			 
			 if(count>=300)
			 {
				 shootStart  = 0;
			 	carRun=1;
			 	count=300;
			 }
			shootBegin = 1;
			if(SweepIn())
			{
				state=4;
				count=0;
			}
		}break;
		case 11:
		{
			notcount=0;
			carRun = 1;
			angClose(-1100,aimAngle,150);
			if (stuckCar(100,100))
			{
				state = 4;
			}
		}
			break;
		case 12:
			notcount=0;
			//目标点(0,1000)
		  StaightCLose(0, 1000, aimAngle, 1200);
		
			//快到目标点了
		if(sqrt((Position_t.Y - 1000) * (Position_t.Y - 1000) + Position_t.X * Position_t.X) < 200)
		{
			state = 13;
		}
			
			break;	
		case 13:
			notcount=0;
			//倒车进入出发区
			StaightCLose(0, 0, 0, -600);
		
			//行程开关触发，开始射球
	    if(SWITCHC0 && SWITCHE2)
			{
				state = 5;
			}
			break;
		case 14:
		{
			if(leftCar)
			{
				aimAngle = AvoidOverAngle(Position_t.angle + 45);
				state = 15;
			}
			if(rightCar)
			{
				state = 15;
				aimAngle = AvoidOverAngle(Position_t.angle - 45);
			}
		}
			break;
		case 15:
		{
			angClose(1500,aimAngle,200);
			
			//快到墙的时候再靠墙矫正射击
			if(Position_t.X > 1200 || Position_t.X < -1200 || Position_t.Y < 1200 || Position_t.Y > 3600)
			{
				state = 4;
			}
		}
		break;
		default:
		{
			state=6;
		}break;
	}
  USART_OUT(UART5,(u8*)"gogogostate %d\r\n",state);
}

/*======================================================================================
   函数定义		：			第一圈
   函数参数		：			plan:方案，speed:速度(mm)
   函数返回值	：			false未结束，true第一圈结束
   用时				：			未测算
   (WIDTH为小车宽度)
   =======================================================================================*/
bool FirstRound(float firstLine)
{
	static int state = 0;
  static float speed = 1500;
	speed += 5;
	if(speed > 1800)
	{
		speed = 1800;
	}
	
	//第一圈贴框走成功极限条件
	if(firstLine < 650)
	{
		firstLine = 650;
	}
	switch (state)
	{
		case 0:
		{
			angClose(1200,-45,200);
			if(Position_t.Y > 500)
			{
				state = 1;
			}
		}
		break;
		//右边，目标角度0度
		case 1:
		{
			StaightCLose(firstLine, 0, 0, speed);
			if (Position_t.Y >= 3200 + WIDTH / 2 - 1100)
        state = 2;
		} break;

		//上边，目标角度90度
		case 2:
		{
			StaightCLose(0, 3200 + WIDTH / 2 + 50, 90, speed);
			if (Position_t.X <= -1000 + FIR_ADV)
				state = 3;
		} break;

	//	//左边，目标角度180度
		case 3:
		{
			if(fighting==0)
			{
        StaightCLose(-1000, 0, 180, speed);
			}
			else if(fighting==1)
			{
				StaightCLose(-1000, 0, 180, speed);
			}
			if (Position_t.Y <= 1150 + FIR_ADV)
				state = 4;
		} break;

	//	//下边，目标角度-90度
		case 4:
		{
			StaightCLose(0, 1150, -90, speed);
//			if (Position_t.X >= 275 + WIDTH / 2 - FIR_ADV)
			if (Position_t.X >= 0)
				return true;
		} break;
	}
//	USART_OUT(UART5,(u8*)"%d\t%d\r\n",(int)state,(int)speed);
	return false;
}

/*======================================================================================
   函数定义		：			判断小车是否卡住不动，
   函数参数		：			无
   函数返回值	：			false 未卡住，true卡住了
   =======================================================================================*/
bool IfStuck(void)
{
	static int  count = 0;
	static int  lx = 0, ly = 0; //记录上一次的坐标

	if ((int)Position_t.X == lx && (int)Position_t.Y == ly)
	{
		count++;
		if (count >= 100 * STUCK_TIME) //卡住了
		{
			count = 0; 
			return true;
		}
	}
	else
	{
		count = 0;
	}

	//保存上一次坐标
	lx  = (int)Position_t.X;
	ly  = (int)Position_t.Y;
	return false;
}
/*======================================================================================
   函数定义		：			判断小车是否靠墙不动，
   函数参数		：			无
   函数返回值	：			false 未卡住，true卡住了
   当行程开关加入后，删除此函数。此函数为临时函数
   =======================================================================================*/
bool IfStuck2(void)
{
	static int  count = 0;
	static int  lx = 0, ly = 0; //记录上一次的坐标

	if ((int)Position_t.X == lx && (int)Position_t.Y == ly)
	{
		count++;
		if (count >= 100 * (STUCK_TIME - 0.3)) //卡住了  这里-0.3是防止程序进入逃逸模式
		{
			count = 0;
			return true;
		}
	}
	else
	{
		count = 0;
	}

	//保存上一次坐标
	lx  = (int)Position_t.X;
	ly  = (int)Position_t.Y;
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
bool  RunRectangle(int length, int wide, float speed)
{
	static int state = 1;

	switch (state)
	{
	//长方形右边，目标角度0度
	case 1:
	{
		if (Position_t.Y >= 3100 + length - ADV_TUEN)
			state = 2;
		StaightCLose(275 + wide, 0, 0, speed);
	} break;

	//长方形上边，目标角度90度
	case 2:
	{
		if (Position_t.X <= -275 - wide + ADV_TUEN)
			state = 3;
		StaightCLose(0, 3100 + length, 90, speed);
	} break;

	//长方形左边，目标角度180度
	case 3:
	{
		if (Position_t.Y <= 1700 - length + ADV_TUEN)
			state = 4;
		StaightCLose(-275 - wide, 0, 180, speed);
	} break;

	//长方形下边，目标角度-90度
	case 4:
	{
		if (Position_t.X >= 275 + wide - ADV_TUEN)
		{
			state = 1;
			return true;
		}
		StaightCLose(0, 1700 - length, -90, speed);
	} break;
	}
	return false;
}
/*======================================================================================
   函数定义		：			坐标校正(考虑了顺逆时针)
   函数参数		：			无
   函数返回值	：			1:           已完成矫正
                      0:           未完成矫正
   =======================================================================================*/

int CheckPosition(void)
{
	static int  state = 0, count = 0, side = 0, checkError = 0, switchError = 0, switchNoError = 1,corner = 0;
	static float  tempx = 0, tempy = 0;
	int         keepgo = 0;
  static float aimAngle = 0;
	
	if(ifPrintPositionCheck)
	{
		LOG_NOTE JudgeState("Position Check");
		ifPrintPositionCheck = false;
	}
	switch (state)
	{
		//判断自己不再死角时再矫正
		case 0:
		{
		  //距离四个角太近，进入
			if(Position_t.X > 1400 && Position_t.Y < 1000)
			{
				corner = 1;
				state = 15;
			}
			else if(Position_t.X > 1400 && Position_t.Y > 3800)
			{
				corner = 2;
				state = 15;
			}
			else if(Position_t.X < -1400 && Position_t.Y > 3800)
			{
				corner = 3;
				state = 15;
			}
			else if(Position_t.X < -1400 && Position_t.Y < 1000)
			{
				corner = 4;
				state = 15;
			}
			
			//不再四个角落中，进入正常矫正状态
			else
			{
				state = 1;
			}
		}
		break;
		//判断距离哪面墙最近
		case 1:
		{
			shootBegin = 0;
			LOG_NOTE JudgeState("Judge which wall is the nearest");
			carRun = 0;
			side = JudgeSide();
			if(side == 1)
			{
				aimAngle = 0;
			}
			else if(side == 2)
			{
				aimAngle = 90;
			}
			else if(side == 3)
			{
				aimAngle = 180;
			}
			else
			{
				aimAngle = -90;
			}
			state = 2;
		} break;

		//原地旋转至目标角度
		case 2:
		{
			shootBegin = 0;
			LOG_NOTE JudgeState("Turn to right angle");
			carRun = 0;
			if(AngleStuck())
			{
				aimAngle = Position_t.angle;
				//state = 16;
			}
			TurnAngle(aimAngle, 15000);
			if (fabs(Position_t.angle - aimAngle) <= 25)
			{
				//记录当前坐标用于闭环后退，防止角度被撞歪后开环后退不准
				tempx = Position_t.X;       
				tempy = Position_t.Y;
				state = 3;
			}
		} break;

		//后退靠墙
		case 3:
		{
			shootBegin = 0;
			LOG_NOTE JudgeState("Go back until against the wall");
			carRun = 0;
			StaightCLose(tempx, tempy, aimAngle, -700);
			POS_NOTE USART_OUT(UART5,(u8*)"SWITCH %d\t%d\t%d\t%d\t%d\r\n",(int)switchNoError,(int)SWITCHC0,(int)SWITCHE2,(int)count,(int)state);
			//如果行程开关没有坏
			if(switchNoError)
			{
				POS_NOTE USART_OUT(UART5,(u8*)"qiang\r\n");
				//后退车被困住（被困的条件比较严苛）
				if(stuckCar(200,2000))
				{
					POS_NOTE USART_OUT(UART5,(u8*)"stuckCar %d\t%d\r\n",(int)SWITCHC0,(int)SWITCHE2);
					
					//行程开关没有全部触发
					if(SWITCHE2 == 0 || SWITCHC0 == 0)
					{
						switchError++;
						if(switchError >= 4)
						{
							state = 3;
							switchNoError = 0;
							switchError = 0;
						}
						aimAngle = Position_t.angle;
						tempx = Position_t.X;
						tempy = Position_t.Y;
						state = 9;
					}
				}
				
				//两个行程开关触发,则进入下一次状态进行激光矫正
				if(SWITCHE2 == 1 && SWITCHC0 == 1)
				{
					switchError = 0;
					count++;
				}
				if(count >= 10)
				{
					count = 0;
					state = 4;
				}
			}
			else
			{
				if(stuckCar(50,200))
				{
					side = JudgeSide();
					if(side == 1)
					{
						//坐标误差过大，说明后面有车
						if(Position_t.Y > 400)
						{
							state = 10;
						}
						else
						{
							state = 4;
						}
					}
					else if(side == 2)
					{
						if(Position_t.X < 2000)
						{
							state = 10;
						}
						else
						{
							state = 4;
						}
					}
					else if(side == 3)
					{
						if(Position_t.Y < 4300)
						{
							state = 10;
						}
						else
						{
							state = 4;
						}
					}
					else
					{
						if(Position_t.X > -2100)
						{
							state = 10;
						}
						else
						{
							state = 4;
						}
					}
				}
			}
		} break;

		//激光校正
		case 4:
		{
			shootBegin = 0;
			LOG_NOTE JudgeState("Position Check with laser");
			carRun = 0;
		  int laserCheck = 0;
		  laserCheck = LaserCheck();
			POS_NOTE USART_OUT(UART5,(u8*)"lasercheck%d\r\n",(int)laserCheck);
			if (laserCheck == 1)
			{
				switchError = 0;
				keepgo = 1;
				state   = 0;
				tempx   = 0, tempy = 0;
			}
			
			//矫正失败
			else if(laserCheck == 0)
			{
				
				//矫正失败，继续矫正
				state = 5;    
				tempx = Position_t.X;
				tempy = Position_t.Y;
				
        //如果激光矫正失败，先利用一次靠墙矫正一次坐标
				if(side == 1)
				{
					angleError  += Position_t.angle;
					yError      = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError)));
					
					//计算下一刻的目标角度,下同
					if(Position_t.X > 0)
					{
						aimAngle = -60;
					}
					else
					{
						aimAngle = 60;
					}
				}
				else if(side == 2)
				{
					angleError  += Position_t.angle - 90;
					xError = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (2400 - 64.65);
					if(Position_t.Y > 2335.35)
					{
						aimAngle = 30;
					}
					else
					{
						aimAngle = 150;
					}
				}
				else if(side == 3)
				{
					angleError  += Position_t.angle - 180;
					angleError  = AvoidOverAngle(angleError);
					yError      = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - 64.65);
					if(Position_t.X > 0)
					{
						aimAngle = -120;
					}
					else
					{
						aimAngle =120;
					}
				}
				else
				{
					angleError  += Position_t.angle + 90;
					xError      = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (-2400 + 64.65);
					if(Position_t.Y > 2335.35)
					{
						aimAngle = -30;
					}
					else
					{
						aimAngle = -150;
					}
				}
			}
			
			//如果车在死角，两侧激光测距均超出激光测距范围，则进入state12
			else if(laserCheck == 3)
			{
				state = 12;
			}
		}
			break;

		//继续矫正,前进
		case 5:
			{
				shootBegin = 1;
				LOG_NOTE JudgeState("continue Check , go ahead");
				carRun = 1;
				angClose(1800, aimAngle, 250);
				
				//判断距离第二面墙1米时准备靠墙
				if(side == 1)
				{
					if(aimAngle > 0)
					{
						if(Position_t.X < -1600)
						{
							state = 6;
						}
					}
					else
					{
						if(Position_t.X > 1600)
						{
							state = 6;
						}
					}
				}
				else if(side == 2)
				{
					if(aimAngle > 90)
					{
						if(Position_t.Y < 750)
						{
							state = 6;
						}
					}
					else
					{
						if(Position_t.Y > 3900)
						{
							state = 6;
						}
					}
				}
				else if(side == 3)
				{
					if(aimAngle < 0)
					{
						if(Position_t.X > 1400)
						{
							state = 6;
						}
					}
					else
					{
						if(Position_t.X < -1400)
						{
							state = 6;
						}
					}
				}
				else
				{
					if(aimAngle < -90)
					{
						if(Position_t.Y < 950)
						{
							state = 6;
						}
					}
					else
					{
						if(Position_t.Y > 3700)
						{
							state = 6;
						}
					}
				}
			} 
			break;
			
		//通过坐标判断车距离哪面墙近
		case 6:
		{
			shootBegin = 0;
			LOG_NOTE JudgeState("Judge which wall is the nearest");
			carRun = 0;
			side = JudgeSide();
			if(side == 1)
			{
				aimAngle = 0;
			}
			else if(side == 2)
			{
				aimAngle = 90;
			}
			else if(side == 3)
			{
				aimAngle = 180;
			}
			else
			{
				aimAngle = -90;
			}
			state = 7;
		}
		break;
			
		//转向目标角度
		case 7:
		{
			shootBegin = 0;
			LOG_NOTE JudgeState("Turn to right angle");
			carRun = 0;
			TurnAngle(aimAngle, 5000);
			if (fabs(Position_t.angle - aimAngle) <5)
			{
				tempx = Position_t.X;       //记录当前坐标用于闭环后退，防止角度被撞歪后开环后退不准
				tempy = Position_t.Y;
				state = 8;
			}
		} break;

		//后退
		case 8:
		{
			shootBegin = 0;
			LOG_NOTE JudgeState("GO back until against the wall");
			carRun = 0;
			StaightCLose(tempx, tempy, aimAngle, -700);
			if(SWITCHE2==1 && SWITCHC0==1)
			{
				count++;
			}
			if(count >= 10)
			{
				count = 0;
				state = 1;
				side = JudgeSide();
				//利用第二面墙矫正
				LOG_NOTE JudgeState("Use the second wall to Check");
				if(side == 1)
				{
					yError  = getPosition_t.Y *cos(ANGTORAD(angleError)) - getPosition_t.X * sin(ANGTORAD(angleError));
				}
				else if(side == 2)
				{
					xError = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (2400 - 64.65);
				}
				else if(side == 3)
				{
					yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - 64.65);
				}
				else
				{
					xError = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (-2400 + 64.65);
				}
				switchError = 0;
				state = 0;
				keepgo  = 1;
				tempx   = 0, tempy = 0;
			}
		}
		 break;
		case 9:
		{
			shootBegin = 0;
			carRun = 0;
			shootBegin = 0;
//		  side = JudgeSide();
//			tempx = Position_t.X;
//			tempy = Position_t.Y;
//			 if(side == 1)
//			 {
//				 if(Position_t.X > 0)
//				 {
//					 aimAngle = 90;
//				 }
//				 else
//				 {
//					 aimAngle = -90;
//				 }
//			 }
//			 else if(side == 2)
//			 {
//				 if(Position_t.Y > 2335.35)
//				 {
//					 aimAngle = 180;
//				 }
//				 else
//				 {
//					 aimAngle = 0;
//				 }
//			 }
//			 else if(side == 3)
//			 {
//				 if(Position_t.X > 0)
//				 {
//					 aimAngle = 90;
//				 }
//				 else
//				 {
//					 aimAngle = -90;
//				 }
//			 }
//			 else
//			 {
//				 if(Position_t.Y > 2335.35)
//				 {
//					 aimAngle = 180;
//				 }
//				 else
//				 {
//					 aimAngle = 0;
//				 }
//			 }
//			 state = 14;
//		 }
			//前进0.5s
			count++;
			if(count >= 50 || sqrt(PF(tempx - Position_t.X) + PF(tempy - Position_t.Y)) > 500)
			{
				//记录靠墙不成功次数
				checkError++;
				count = 0;
				state = 3;
			}
			
			//靠墙不成功次数超过2次,转到10状态
			if(checkError >= 2)
			{
				//记录当前距离车最近的墙
				side = JudgeSide();
				checkError = 0;
				state = 10;
			}
			angClose(1000,(aimAngle - 45),150);
		}
			break;
		case 10:
		{
			shootBegin = 0;
			carRun = 0;
			if(side == 1)
			{
				//计算下一刻的目标角度,下同
				if(Position_t.X > 0)
				{
					aimAngle = -60;
				}
				else
				{
					aimAngle = 60;
				}
			}
			else if(side == 2)
			{
				if(Position_t.Y > 2335.35)
				{
					aimAngle = 30;
				}
				else
				{
					aimAngle = 150;
				}
			}
			else if(side == 3)
			{
				if(Position_t.X > 0)
				{
					aimAngle = -120;
				}
				else
				{
					aimAngle =120;
				}
			}
			else
			{
				if(Position_t.Y > 2335.35)
				{
					aimAngle = -30;
				}
				else
				{
					aimAngle = -150;
				}
			}
			state = 11;
		}
			break;
		case 11:
		{
			count++;
			if(count >= 300)
			{
				count = 300;
				shootBegin = 1;
				carRun = 1;
			}
			else
			{
				carRun = 0;
			}
			
			angClose(1600, aimAngle, 250);
		
			//判断距离第二面墙1米时准备靠墙
			if(side == 1)
			{
				if(aimAngle > 0)
				{
					if(Position_t.X < -1400)
					{
						//返回状态1，从新矫正
						count = 0;
						state = 1;
					}
				}
				else
				{
					if(Position_t.X > 1400)
					{
						count = 0;
						state = 1;
					}
				}
			}
			else if(side == 2)
			{
				if(aimAngle > 90)
				{
					if(Position_t.Y < 950)
					{
						count = 0;
						state = 1;
					}
				}
				else
				{
					if(Position_t.Y > 3750)
					{
						count = 0;
						state = 1;
					}
				}
			}
			else if(side == 3)
			{
				if(aimAngle < 0)
				{
					if(Position_t.X > 1400)
					{
						count = 0;
						state = 1;
					}
				}
				else
				{
					if(Position_t.X < -1400)
					{
						count = 0;
						state = 1;
					}
				}
			}
			else
			{
				if(aimAngle < -90)
				{
					if(Position_t.Y < 950)
					{
						count = 0;
						state = 1;
					}
				}
				else
				{
					if(Position_t.Y > 3750)
					{
						count = 0;
						state = 1;
					}
				}
			}
		}
		break;
		case 12:
		{
		
			//记录当前的角度值
			tempx = Position_t.X;
			tempy = Position_t.Y;
			aimAngle = Position_t.angle;
		  state = 13;
		}
			break;
		case 13:
		{
			shootBegin = 1;
			carRun = 1;
			angClose(1200,aimAngle,150);
		  
			//前进1.2m
			if(sqrt((Position_t.X - tempx) * (Position_t.X - tempx) + (Position_t.Y - tempy) * (Position_t.Y - tempy)) > 1200)
			{
				state = 1;
			}
		}
		break;
		case 14:
		{
			shootBegin = 1;
			carRun = 1;
			count++;
		  if(count > 200 || sqrt(PF(tempx - Position_t.X) + PF(tempy - Position_t.Y)) > 500)
			{
				checkError++;
				state = 1;
				count = 0;
			}
			angClose(1500,aimAngle,150);
			
			//靠墙不成功次数超过2次,转到10状态
			if(checkError >= 2)
			{
				//记录当前距离车最近的墙
				side = JudgeSide();
				checkError = 0;
				count = 0;
				state = 10;
			}
		}
		break;
		
		//确保不在角落中投球
		case 15:
		{
			shootBegin = 1;
			carRun = 1;
			if(corner == 1)
			{
				aimAngle = 0;
			}
			else if(corner == 2)
			{
				aimAngle = 90;
			}
			else if(corner == 3)
			{
				aimAngle = 180;
			}
			else
			{
				aimAngle = -90;
			}
			angClose(1500,aimAngle,200);
			
			if(corner == 1)
			{
			  if(Position_t.Y > 1400)
				{
					aimAngle = 90;
					state = 2;
				}
			}
			else if(corner == 2)
			{
				if(Position_t.X < 1000)
				{
					aimAngle = 180;
					state = 2;
				}
			}
			else if(corner == 3)
			{
				if(Position_t.Y < 3400)
				{
					aimAngle = -90;
					state = 2;
				}
			}
			else
			{
				if(Position_t.X > -1000)
				{
					aimAngle = 0;
					state = 2;
				}
			}
		}
		break;
		case 16:
		{
			count++;
			if(count >= 80)
			{
				state = 2;
				count = 0;
			}
			angClose(-1000,aimAngle,150);
		}
		break;
	}
//	USART_OUT(UART5,(u8*)"%d\r\n",(int)state);
	return keepgo;
}


//方案1 发三个区域的球数
extern int    ballN_L, ballN_M, ballN_R;
//方案2 发球数最多的那个角度
extern int8_t  bestAngle,  nearestAngle;
//方案3 最近球的极坐标
extern float nearestDis;
//方案4 所有球的角度和距离
extern int    go, arr_number;
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
extern float bestTraX[20], bestTraY[20];
Pose_t bestTra[20] = {0};

int RunCamera(void)
{
	static int    haveBall = 0, run = 0, ballAngle, traceH[10][10] = { 0 }, traceS[10][10] = { 0 }, stagger = 0, left = 1, right = 1, up = 1, down = 1,border=0,porm=0,chuqu=0,edge[4]={0},cameratime=0;
	static float  cameraX, cameraY;
	int           finish = 0;
  cameraScheme = 1;
	cameratime++;
	if(cameratime>=3000)
	{
		cameratime=0;
		run = 0; stagger = 0;
		finish = 1;
	}
  g_plan=1;
	
	//一环连一环 上部分是偶数则加一 下部分是奇数则加一 让stagger(错开)等于一
	if (Position_t.Y < 2400)
	{
		if (((int)(run / 2) + (int)(run / 2)) != run)
		{
			run++;
			stagger = 1;
		}
	}
	if (Position_t.Y > 2400)
	{
		if (((int)(run / 2) + (int)(run / 2)) == run)
			run++;
	}

	//记录走过的位置（区域）
	traceH[Zoning(Position_t.X, Position_t.Y).hor][Zoning(Position_t.X, Position_t.Y).ver]  = 1;
	traceS[Zoning(Position_t.X, Position_t.Y).ver][Zoning(Position_t.X, Position_t.Y).hor]  = 1;
	//扫描一下是否全已走过
	if (ScanTrace(traceH))
	{
		finish = 1; run = 0; stagger = 0;cameratime=0;
	}
	//计算走过的路线 寻找一条错开的路线
	if (run > 0 && ((int)(run / 2) + (int)(run / 2)) == run && stagger == 1)
	{
		stagger = 0;
		left    = Least_S(traceH[0], traceH[1], traceH[2], traceH[3]);
		right   = Least_S(traceH[6], traceH[7], traceH[8], traceH[9]);
		down    = Least_H(traceS[0], traceS[1], traceS[2]);
		up      = Least_H(traceS[7], traceS[8], traceS[9]);
	}
	 POS_NOTE USART_OUT(UART5,(u8*)"trace %d\t%d\t%d\t%d\r\n",down,right,up,left);
	switch (cameraScheme)
	{
		case 1:
		{
			if ( go)
			{
				go = 0;
				if (arr_number == 0)
				{
					haveBall = 0;porm=0;
				}
				else
				{
					haveBall  = 1;
					if(bestAngle>0)
					{
						porm=1;
					}
					else if(bestAngle<0)
					{
						porm=-1;
					}
					cameraX   = Position_t.X - CAMERATOGYRO * sin(Position_t.angle);
					cameraY   = Position_t.Y + CAMERATOGYRO * cos(Position_t.angle);
					ballAngle = AvoidOverAngle(Position_t.angle + bestAngle);
					POS_NOTE USART_OUT(UART5,(u8*)"bestangle %d\r\n",bestAngle);
				}
			}
			

			
			//到边界要拐弯了
			if (fabs(Position_t.X) >= 1500 || Position_t.Y <= 900 || Position_t.Y >= 3900)
			{
				if(chuqu==0)
				{
					haveBall = 0;
					if(run>2)
					{
						border=1;
						if(edge[0]==1&&edge[1]==1&&edge[2]==1&&edge[3]==1)
						{
							border=0;
						}					
					}
				}				
			}
			else
			{
				border=0;
				chuqu=0;
			}
			//到达中间危险区域的标志
			if(fabs(Position_t.X)<1000&&Position_t.Y>1000&&Position_t.Y<3800)
				haveBall = 0;
			switch (haveBall)
			{
				case 0:
				{
					if(border==0)
					{
						if (run < 2)
							First_Scan();
						else
							New_Route(down, right, up, left);
					}
					else
					{
						if(Position_t.Y<=1100&&Position_t.X<1300)
						{
							edge[1]=1;
              switch(porm)
							{
								case 0:
								{
									StaightCLose(0, 450, -90, cameraSpeed);
								}break;
								case 1:
								{
									chuqu=1;
								}break;
								case -1:
								{
									StaightCLose(0, 300, -90, cameraSpeed);
								}break;
								default:
								 break;
							}
						}
						if(Position_t.X>=1300&&Position_t.Y<3600)
						{
							edge[2]=1;
              switch(porm)
							{
								case 0:
								{
									StaightCLose(1950, 0, 0, cameraSpeed);
								}break;
								case 1:
								{
									chuqu=1;
								}break;
								case -1:
								{
									StaightCLose(2100, 0, 0, cameraSpeed);
								}break;
								default:
								 break;
							}							
						}
						if(Position_t.Y>=3600&&Position_t.X>-1200)
						{
							edge[3]=1;
              switch(porm)
							{
								case 0:
								{
									StaightCLose(0, 4200, 90, cameraSpeed);
								}break;
								case 1:
								{
									chuqu=1;
								}break;
								case -1:
								{
									StaightCLose(0, 4400, 90, cameraSpeed);
								}break;
								default:
								 break;
							}							
						}
						if(Position_t.X<=-1200&&Position_t.Y>1100)
						{
							edge[0]=1;
							switch(porm)
							{
								case 0:
								{
									StaightCLose(-1800, 0, 180, cameraSpeed);
								}break;
								case 1:
								{
									chuqu=1;
								}break;
								case -1:
								{
									StaightCLose(-2000, 0, 180, cameraSpeed);
								}break;
								default:
								 break;
							}
						}
					}

				} break;

				case 1:
				{
					StaightCLose(cameraX, cameraY, ballAngle, cameraSpeed);	
          //angClose(cameraSpeed,bestAngle,100);	
     					
				} break;
				default:
					break;
				}
		} break;

	
			case 3:
		{
			if (go)
			{
				go = 0;
				if (arr_number == 0)
				{
					haveBall = 0;
				}
				else
				{
					haveBall  = 1;
					cameraX   = Position_t.X - CAMERATOGYRO * sin(Position_t.angle);
					cameraY   = Position_t.Y + CAMERATOGYRO * cos(Position_t.angle);
					if (Vehicle_Width(nearestDis, nearestAngle))
						ballAngle = AvoidOverAngle(Position_t.angle);
					else
						ballAngle = AvoidOverAngle(Position_t.angle + nearestAngle);
				}
			}
				//到边界要拐弯了
	if (fabs(Position_t.X) > 1700 || Position_t.Y < 700 || Position_t.Y > 4100)
		haveBall = 0;
	//到达中间危险区域的标志
  if(fabs(Position_t.X)<800&&Position_t.Y>1300&&Position_t.Y<3500)
		haveBall = 0;
			switch (haveBall)
			{
				case 0:
				{
					if (run < 2)
						First_Scan();
					else
						New_Route(down, right, up, left);
				} break;

				case 1:
				{
					StaightCLose(cameraX, cameraY, ballAngle, cameraSpeed);
				} break;

				default:
					break;
			}
		} break;

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
void TurnAngle(float angel, int speed)
{
	float Dangel  = 0;            //角度差值
	float Input   = 0;            //pid控制输出
	Dangel = (angel - Position_t.angle);

	//纠正角度
	if (Dangel > 180)
		Dangel -= 360;
	if (Dangel < -180)
		Dangel += 360;
	Input = 30 * Dangel;

	//判断是否反向转更快
	if (Dangel < 0 && speed > 0)
		speed = -speed;

	//角度大于10，speed转
	if (Dangel > 15 || Dangel < -15)

	{
		VelCrl(CAN2, 2, g_plan * speed);
		VelCrl(CAN2, 1, g_plan * speed);
	}

	//角度小于10，pid控制输出
	else
	{
		VelCrl(CAN2, 2, g_plan * Input);
		VelCrl(CAN2, 1, g_plan * Input);
	}
}

/*======================================================================================
   函数定义		：			利用激光矫正坐标
   函数参数		：			无
   函数返回值	：			矫正失败返回0, 成功返回1，还在矫正返回2，车在死角，超出激光测量范围返回3
   =======================================================================================*/
int LaserCheck(void)
{
	static int laserGetRight = 0, laserGetLeft = 0;
	int  side = 0, success = 2;
	static u8 step = 3, left = 1, right = 1;
	
	//每次调用都要重置一次step
	if(resetStep)
	{
		step = 3;
		resetStep = 0;
	}
	switch(step)
	{
		case 0:
		{
			//如果激光被挡,进入step = 2
			if (laserGetRight + laserGetLeft < 4700)
			{	
				right = 1;
				left = 1;
				step = 3;
				success = 0;
			}
			
			//没有被挡,step = 1, 开始矫正
			else
			{
				step = 1;
			}
		}
			break;
		case 1:
		{
			//靠的墙是Y=0
			if (Position_t.angle < 45 && Position_t.angle > -45)
			{
				angleError += Position_t.angle;  //纠正角度坐标
				
				//右侧没有被挡
				if(right)
				{
					success = 1;
					xError = (g_plan * getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (2400 - laserGetRight);
				}
				else if(left)
				{
					success = 1;
					xError = (g_plan * getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (laserGetLeft - 2400);
				}
				
				//左右侧激光都有问题，则返回0
				else
				{
					//置1，为了下次激光矫正时left right 初始值为1
					right = 1;
					left = 1;
					step = 3;
					//左右侧都被挡，succese置0
					success = 0;
					//return 0;
				}
				yError      = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError)));
				
				//right left置1，为了下次激光矫正时left right 初始值为1
				right = 1;
				left = 1;
				step = 3;
				//return 1;
			}

			//靠X=g_plan * 2400的墙
			else if (Position_t.angle < 135 && Position_t.angle > 45)
			{
				angleError  += Position_t.angle - 90;
				xError      = (g_plan * getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (2400 - 64.65);
				
				//逆时针用右激光
				if(g_plan == 1)
				{
					if(right)
					{
						success = 1;
						yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - laserGetRight);
					}
					else if(left)
					{
						success = 1;
						yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError))) - (laserGetLeft - 64.65);
					}
					else
					{
						right = 1;
						left = 1;
						step = 3;
						success = 0;
						//return 0;
					}
				}
				else
				{
					if(right)
					{
						success = 1;
						yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError))) - (laserGetRight - 64.65);
					}
					else if(left)
					{
						success = 1;
						yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - laserGetLeft);
					}
					else
					{
						right = 1;
						left = 1;
						step = 3;
						success = 0;
						//return 0;
					}
				}
				right = 1;
				left = 1;
				step = 3;
//				return 1;
			}

			//靠Y=4800的墙
			else if (Position_t.angle > 135 || Position_t.angle < -135)
			{
				angleError  += Position_t.angle - 180;
				angleError  = AvoidOverAngle(angleError);
				if(right)
				{
					success = 1;
					xError = (g_plan * getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (laserGetRight - 2400);
				}
				else if(left)
				{
					success = 1;
					xError = (g_plan * getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (2400 - laserGetLeft);
				}
				else
				{
					right = 1;
					left = 1;
					step = 3;
					success = 0;
					//return 0;
				}
				yError      = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - 64.65);
				right = 1;
				left = 1;
				step = 3;
				//return 1;
			}

			//靠X=-g_plan * 2400的墙
			else
			{
				angleError  += Position_t.angle + 90;
				xError      = (g_plan * getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (-2400 + 64.65);
				if(g_plan == 1)
				{
					if(right)
					{
						success = 1;
						yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError))) - (laserGetRight - 64.65);
					}
					else if(left)
					{
						success = 1;
						yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 -laserGetLeft);
					}
					else
					{
						right = 1;
						left = 1;
						step = 3;
						success = 0;
						//return 0;
					}
				}
				else
				{
					if(right)
					{
						success = 1;
						yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 -laserGetRight);
					}
					else if(left)
					{
						success = 1;
						yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - g_plan * getPosition_t.X * sin(Angel2PI(angleError))) - (laserGetLeft - 64.65);
					}
					else
					{
						right = 1;
						left = 1;
						step = 3;
						success = 0;
						//return 0;
					}
				}
				right = 1;
				left = 1;
				step = 3;
				//return 1;
			}
		}
			break;
		case 2:
		{
			//两侧激光值均小于1000，说明车在死角，距离都超出了激光的测量范围，此时函数返回数值3
			if(laserGetRight < 1000 || laserGetLeft < 1000)
			{
				step = 3;
				success = 3;
			}
			
		  //判断在哪面墙
			side = JudgeSide();
		  if(side == 1)
			{	
				//误差超过300mm，认为右侧被挡
				if(fabs(Position_t.X - (g_plan * (2400 - laserGetRight))) > 200)
				{
					right = 0;
				}
				
				//左侧被挡
				if(fabs(Position_t.X - (g_plan * (laserGetLeft - 2400))) > 200)
				{
					left = 0;
				}
			}
			else if(side == 2)
			{
				if(g_plan == 1)
				{
					if(fabs(Position_t.Y - (4800 - 64.65 - laserGetRight)) > 200)
					{
						right = 0;
					}
					if(fabs(Position_t.Y - (laserGetLeft - 64.65)) > 200)
					{
						left = 0;
					}
				}
				else
				{
					if(fabs(Position_t.Y - (laserGetRight - 64.65)) > 200)
					{
						right = 0;
					}
					if(fabs(Position_t.Y - (4800 - 64.65 - laserGetLeft)) > 200)
					{
						left = 0;
					}
				}
			}
			else if(side == 3)
			{
				if(fabs(Position_t.X - (g_plan * (laserGetRight - 2400))) > 200)
				{
					right = 0;
				}
				if(fabs(Position_t.X - (g_plan * (2400 - laserGetLeft))) > 200)
				{
					left = 0;
				}
			}
			else
			{
				if(g_plan == 1)
				{
					if(fabs(Position_t.Y - (laserGetRight - 64.65)) > 200)
					{
						right = 0;
					}
					if(fabs(Position_t.Y - (4800 - 64.65 - laserGetLeft)) > 200)
					{
						left = 0;
					}
				}
				else
				{
					if(fabs(Position_t.Y - (4800 - 64.65 - laserGetRight)) > 200)
					{
						right = 0;
					}
					if(fabs(Position_t.Y - (laserGetLeft - 64.65)) > 200)
					{
						left = 0;
					}
				}
			}
			step = 1;
		}
			break;
		case 3:
			
			//采集激光值
			laserGetRight = Get_Adc_Average(RIGHT_LASER, 100);
			laserGetLeft  = Get_Adc_Average(LEFT_LASER, 100);
			step = 0;
			break;
	}
	POS_NOTE USART_OUT(UART5,(u8*)"laser%d\t%d\t%d\t%d\r\n",(int)laserGetRight,(int)laserGetLeft,(int)success,(int)step);
	return success;
}

//角度变换函数
float Angel2PI(float angel)
{
	float res;

	res = PI * (angel) / 180;
	return res;
}
