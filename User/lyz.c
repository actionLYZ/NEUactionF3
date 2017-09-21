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

/*==============================================全局变量声明区============================================*/
extern POSITION_T Position_t;     //校正后定位
extern POSITION_T getPosition_t;  //获得的定位
extern int        g_plan, g_camera, bestSum;
float             angleError = 0, xError = 0, yError = 0;
int               cameraScheme = 0;
extern int        shootStart, ballColor;
extern int32_t     g_rightPulse ;
extern int32_t     g_leftPulse ;
extern float       firstLine;
/*================================================函数定义区==============================================*/


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
	right = Get_Adc_Average(RIGHT_LASER, 30);
	left = Get_Adc_Average(LEFT_LASER, 30);
//	USART_OUT(UART5,(u8*)"r%d\tl%d\r\n",(int)right,(int)left);
	if (right < 2000)     //右侧激光触发
	{
		return 1;
	}
		
	else if (left < 2000) //左侧激光触发
	{
		return -1;
	}
		
	else
		return 0;
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
	static float  LDdis, LDangle;   //上一次距离与角度差

	//计算距离输出
	Ddis = Piont2Straight(aimx, aimy, angle);

	if (fabs(speed) <= 500)
		Dinput = 9 * Ddis + 1.5 * (Ddis - LDdis);
	else if (fabs(speed) <= 1000)
		Dinput = 12 * Ddis + 3 * (Ddis - LDdis);
	else if (fabs(speed) <= 1500)
		Dinput = 18 * Ddis + 3 * (Ddis - LDdis);
	else
		Dinput = 24 * Ddis + 3 * (Ddis - LDdis);
	LDdis = Ddis;

	//计算角度输出
	Dangle = (angle - Position_t.angle);
	if (Dangle > 180)
		Dangle -= 360;
	if (Dangle < -180)
		Dangle += 360;

	if (fabs(speed) <= 500)
		Ainput = 100 * Dangle + 1 * (Dangle - LDangle);
	else if (fabs(speed) <= 1000)
		Ainput = 160 * Dangle + 30 * (Dangle - LDangle);
	else if (fabs(speed) <= 1500)
		Ainput = 250 * Dangle + 30 * (Dangle - LDangle);
	else
		Ainput = 350 * Dangle + 30 * (Dangle - LDangle);
	LDangle = Dangle;

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
extern int carRun;
/*======================================================================================
   函数定义	：		开始跑场
   函数参数	：		方案：暂定1为逆时针(右侧激光触发)，-1为顺时针(左侧激光触发)
   函数返回值：		无
   =======================================================================================*/
void GoGoGo(float firstLine)
{
	static int  state = 2, shootTime = 0;             //应该执行的状态
	static int  length = WIDTH / 2, wide = WIDTH / 2; //长方形跑场参数

	switch (state)
	{
		//第一圈放球区附近跑场
		case 1:
		{
			shootStart  = 0;
			carRun      = 1;
			if (FirstRound(firstLine) == 1)
			{
				//初始化长方形跑场参数
				length  += SPREAD_DIS;
				wide    += SPREAD_DIS;
				state   = 2;
			}
		}
		break;

		//向外扩散扫场
		case 2:
		{
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
			if(sweepYuan(1800, 1000, 3, 1))
				state = 3;
		}
		break;
		
		//紧随画圆后矩形扫场
		case 3:
			if(AfterCircle())
				state = 4;
			break;
		//进行坐标校正
		case 4:
		{
			carRun = 0;
			if (CheckPosition())
			{
				state = 5;
				VelCrl(CAN2, 1, 0);
				VelCrl(CAN2, 2, 0);
			}
				
		} break;

		case 5:
		{
			carRun      = 0;
			shootStart  = 1;
			if (ShootBallW())
			{
				state = 6;
				shootStart = 0;
				shootTime++;
				switch (shootTime)
				{
					case 1:
					{
						state = 6;
						if (cameraScheme == 0)
						{
							cameraScheme = 1;
							GPIO_SetBits(GPIOE, GPIO_Pin_4);
							GPIO_ResetBits(GPIOE, GPIO_Pin_6);
						}
						else if (cameraScheme == 1)
						{
							cameraScheme = 2;
							GPIO_SetBits(GPIOE, GPIO_Pin_4);
							GPIO_SetBits(GPIOE, GPIO_Pin_6);
						}
						else if (cameraScheme == 2)
						{
							cameraScheme = 3;
							
							GPIO_ResetBits(GPIOE, GPIO_Pin_4);
							GPIO_SetBits(GPIOE, GPIO_Pin_6);
						}
						else
						{
						}
					} 
					break;

					case 2: 
						state = 7;
					break;

				default: break;
				}
			}
			else
			{
			}
		} break;

		case 6:
		{
			carRun      = 1;
			if(RunWithCamera1(2))
			{
				state = 4;
			}
		} break;

		case 7:
		{
			carRun = 1;
			if (RunEdge())
			{
				state     = 4;
				shootTime = 0;
			}
		} break;
		
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
bool FirstRound(float firstLine)
{
	static int state = 1;
  static float speed = 800;
	float advance = 0;
	//第一条目标直线距离铁框太近,就让它贴铁框走
	speed += 5;
	if(speed >= 1800)
	{
		speed = 1800;
	}
	advance = 1000;
	switch (state)
	{
		//右边，目标角度0度
		case 1:
		{
			StaightCLose(firstLine, 0, 0, speed);
			if (Position_t.Y >= 3100 + WIDTH / 2 - advance)
        state = 2;
		} break;

		//上边，目标角度90度
		case 2:
		{
			StaightCLose(0, 3100 + WIDTH / 2 + 50, 90, speed);
			if (Position_t.X <= -275 - WIDTH / 2 + FIR_ADV)
				return true;
		} break;

	//	//左边，目标角度180度
	//	case 3:
	//	{
	//		StaightCLose((-275 - WIDTH / 2 - 150), 0, 180, FIRST_SPEED);
	//		if (Position_t.Y <= 1700 - WIDTH / 2 + FIR_ADV - 500)
	//			state = 4;
	//	} break;

	//	//下边，目标角度-90度
	//	case 4:
	//	{
	//		StaightCLose(0, 1700 - WIDTH / 2 - 100, -90, FIRST_SPEED);

	//		if (Position_t.X >= 275 + WIDTH / 2 - FIR_ADV)
	//			return true;
	//	} break;
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
int x1 = 0, x2 = 0, y1 = 0, y2 = 0;
/*======================================================================================
   函数定义		：			坐标校正(考虑了顺逆时针)
   函数参数		：			无
   函数返回值	：			1:           已完成矫正
                      0:           未完成矫正
   =======================================================================================*/
int CheckPosition(void)
{
	static int  state = 1, count = 0, side = 0;
	static int  tempx = 0, tempy = 0;
	int         keepgo = 0;
  static float aimAngle = 0;
	switch (state)
	{
		//判断距离哪面墙最近
		case 1:
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
			state = 2;
			USART_OUT(UART5,(u8*)"%d\t%d\t%d\t%d\r\n",(int)side,(int)aimAngle,(int)Position_t.X,(int)Position_t.Y);
		} break;

		//原地旋转至目标角度
		case 2:
		{
			TurnAngle(aimAngle, 5000);
			if (fabs(Position_t.angle - aimAngle) <= 15)
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
			StaightCLose(tempx, tempy, aimAngle, -500);
			
			//后退时如果被困，则进入状态9
//			if(stuckCar(100))
//			{
//				state = 10;
//			}
			//两个行程开关触发,则进入下一次状态进行激光矫正
			if(SWITCHC2 == 1 && SWITCHC0 == 1)
			{
				count++;
			}
			if(count >= 10)
			{
				count = 0;
				state = 4;
			}
		} break;

		//激光校正
		case 4:
		{
			if (LaserCheck())
			{
				keepgo  = 1;
				state   = 1;
				tempx   = 0, tempy = 0;
			}
			else
			{
				//矫正失败，继续矫正
				state = 5;    
				
        //如果激光矫正失败，先利用一次靠墙矫正一次坐标
				if(side == 1)
				{
					angleError  += Position_t.angle;
					yError  = getPosition_t.Y *cos(ANGTORAD(angleError)) - getPosition_t.X * sin(ANGTORAD(angleError));
				}
				else if(side == 2)
				{
					angleError += Position_t.angle - 90;
					xError = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - (2400 - 64.65);
				}
				else if(side == 3)
				{
					angleError += Position_t.angle - 180;
					yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - 64.65);
				}
				else
				{
					angleError += Position_t.angle + 90;
					xError = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - (-2400 + 64.65);
				}
			}
		} break;

		//继续矫正,前进
		case 5:
			{
				count++;
				VelCrl(CAN2, 1, 8000);
				VelCrl(CAN2, 2, -8000);
				if (count > 20)
				{
					count = 0;
					state = 6;
				}
			} 
			break;
			
			//通过坐标判断车距离哪面墙近
		case 6:
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
			break;
			
		//转向目标角度
		case 7:
		{
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
			StaightCLose(tempx, tempy, aimAngle, -800);
			if(SWITCHC2==1 && SWITCHC0==1)
			{
				count++;
			}
			if(count >= 10)
			{
				count = 0;
				state = 1;
				
				//利用第二面墙矫正
				if(side == 1)
				{
					yError  = getPosition_t.Y *cos(ANGTORAD(angleError)) - getPosition_t.X * sin(ANGTORAD(angleError));
				}
				else if(side == 2)
				{
					xError = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - (2400 - 64.65);
				}
				else if(side == 3)
				{
					yError = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - 64.65);
				}
				else
				{
					xError = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - (-2400 + 64.65);
				}

				keepgo  = 1;
				state   = 2;
				tempx   = 0, tempy = 0;
			}
		} break;
//		case 9:
//			
//			VelCrl(CAN2, 1, 5000);
//			VelCrl(CAN2, 2, -5000);
//			break;
	}
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
	static int    gone = 1, haveBall = 0, run = 0, ballAngle, traceH[10][10] = { 0 }, traceS[10][10] = { 0 }, stagger = 0, left = 1, right = 1, up = 1, down = 1;
	static float  cameraX, cameraY;
	int           finish = 0, circulate;
	POSITION_T    basePoint;
   cameraScheme = 1;
	//到边界要拐弯了
	if (fabs(Position_t.X) > 2000 || Position_t.Y < 400 || Position_t.Y > 4400)
		haveBall = 0;

	//一环连一环 上部分是偶数则加一 下部分是奇数则加一 让stagger(错开)等于一
	if (Position_t.X > -115 && Position_t.X < -85 && Position_t.Y < 1700)
	{
		if (((int)(run / 2) + (int)(run / 2)) != run)
		{
			run++;
			stagger = 1;
		}
	}
	if (Position_t.X > -115 && Position_t.X < -85 && Position_t.Y > 3100)
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
		finish = 1; run = 0; stagger = 0;
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
	switch (cameraScheme)
	{
	case 1:
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
				ballAngle = AvoidOverAngle(Position_t.angle + bestAngle);
				USART_OUT(UART5,(u8*)"bestangle%d\r\n",bestAngle);
			}
		}
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

	case 2:
	{
		cameraX = Position_t.X - CAMERATOGYRO * sin(Position_t.angle);
		cameraY = Position_t.Y + CAMERATOGYRO * cos(Position_t.angle);
		if (go == 1)
		{
			go = 0;
			if (gone == 1)
			{
				basePoint.X     = cameraX;
				basePoint.Y     = cameraY;
				basePoint.angle = Position_t.angle;
			}
			//判断是否已走过该区域，继续扫面下一个区域
			if (P2P(cameraX, cameraY, basePoint.X, basePoint.Y) >= 1900 || Position_t.angle >= basePoint.angle + 25 || Position_t.angle <= basePoint.angle - 25)
				gone = 1;
			else
				gone = 0;
		}
		if (go == 1 && gone == 1)
		{
			if (arr_number == 0)
			{
				haveBall = 0;
			}
			else
			{
				haveBall = 1;
				PathPlan(cameraX, cameraY);
				ClearRingBuffer();
				for(circulate=0;circulate<bestSum;circulate++)
				{
					bestTra[circulate].point.x = bestTraX[circulate];
					bestTra[circulate].point.y = bestTraY[circulate];
				}	    
			}
		}
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
				InputPoints2RingBuffer(bestTra,bestSum);
				CaculatePath();
				PathFollowing(1);		
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
   函数返回值	：			矫正成功返回1，失败返回0
   =======================================================================================*/
int LaserCheck(void)
{
	int laserGetRight = 0, laserGetLeft = 0;
	GPIO_SetBits(GPIOE,GPIO_Pin_7);
	laserGetRight = Get_Adc_Average(RIGHT_LASER, 20);
	laserGetLeft  = Get_Adc_Average(LEFT_LASER, 20);
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	
	//如果激光被挡，返回 0
	if (laserGetRight + laserGetLeft < 4700)
	{	
		x1  = getPosition_t.X;
		y1  = getPosition_t.Y;
		
		return 0;		
	}

	//没有被挡，返回
	else
	{
		//靠的墙是Y=0
		if (Position_t.angle < 45 && Position_t.angle > -45)
		{
			angleError  += Position_t.angle;  //纠正角度坐标
			xError      = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (2400 - laserGetRight);
			yError      = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError)));
			return 1;
		}

		//靠X=g_plan * 2400的墙
		else if (Position_t.angle < 135 && Position_t.angle > 45)
		{
			angleError  += Position_t.angle - g_plan * 90;
			xError      = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (2400 - 64.65);
			yError      = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - laserGetRight);
			return 1;
		}

		//靠Y=4800的墙
		else if (Position_t.angle > 135 && Position_t.angle < -135)
		{
			angleError  += Position_t.angle - 180;
			angleError  = AvoidOverAngle(angleError);
			xError      = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (2400 - laserGetLeft);
			yError      = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError))) - (4800 - 64.65 - 64.65);
			return 1;
		}

		//靠X=-g_plan * 2400的墙
		else
		{
			angleError  += Position_t.angle + g_plan * 90;
			xError      = (getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError))) - g_plan * (-2400 + 64.65);
			yError      = (getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError))) - (laserGetRight - 64.65);
			return 1;
		}
	}
}
	

//角度变换函数
float Angel2PI(float angel)
{
	float res;

	res = PI * (angel) / 180;
	return res;
}
