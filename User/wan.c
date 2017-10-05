#include "wan.h"
#include "lyz.h"
#include "math.h"
#include "elmo.h"
#include "usart.h"
#include "moveBase.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_it.h"
#include "c0.h"
#include "stm32f4xx_adc.h"
#include "stdlib.h"
/*==============================================全局变量声明区============================================*/

extern POSITION_T Position_t;
extern int8_t     g_cameraFin;
extern int8_t     g_cameraNum;
extern int8_t     g_cameraAng[50];
extern uint8_t    g_cameraDis[50];
static int8_t     leftAng[50]   = { 0 };
static int8_t     midAng[50]    = { 0 };
static int8_t     rightAng[50]  = { 0 };
static uint8_t    leftDis[50]   = { 0 };
static uint8_t    midDis[50]    = { 0 };
static uint8_t    rightDis[50]  = { 0 };
static uint8_t    leftFlag      = 0;
static uint8_t    rightFlag     = 0;
static uint8_t    midFlag       = 0;
float             aangle        = 0;
extern int8_t     whiteBall;      //白球信号
extern int8_t     blackBall;      //黑球信号
extern uint8_t    g_cameraPlan;   //摄像头接球方案
extern uint8_t    g_ballSignal;
extern int32_t    g_shootV;       //串口接收到的速度
extern int32_t    g_shootFactV;   //发射电机的实时转速
extern int32_t    g_shootAngle;   //返回的航向角的真实角度(脉冲)
extern int        ballColor;
extern int32_t     btV;
extern int32_t     btAngle;
extern int32_t     g_rightPulse ;
extern int32_t     g_leftPulse ;
extern int32_t     g_collectVel;
extern int32_t     g_pushPosition;
extern int         g_plan;
extern uint8_t    circleFlag;
extern uint8_t     shootNum;
extern float             angleError, xError, yError;
/*======================================================================================
   函数定义	  ：		Send Get函数
   函数参数	  ：
   函数返回值  ：
   =======================================================================================*/
void SendAng(float ang)
{
	aangle = ang;
}
float GetAng(void)
{
	return aangle;
}
/*======================================================================================
   函数定义	  ：		获取行驶的距离
   函数参数	  ：		无
   函数返回值  ：		小车行驶的距离
   =======================================================================================*/
float GetDistance(POSITION_T startPoint)
{
	float distance = 0.0f;

	distance = sqrt((Position_t.X - startPoint.X) * (Position_t.X - startPoint.X) + (Position_t.Y - startPoint.Y) * (Position_t.Y - startPoint.Y));
	return distance;
}
/*======================================================================================
   函数定义	  ：		避免角度溢出
   函数参数	  ：		当前角度
   函数返回值    ：		修正后的角度
   =======================================================================================*/
float AvoidOverAngle(float angle)
{
	if (angle <= -180)
		angle += 360;
	if (angle > 180)
		angle -= 360;
	return angle;
}
/*======================================================================================
   函数定义	  ：		角度闭环
   函数参数	  ：		V          小车速度
                  aimAngle   目标角度
                  Kp         P参数
   函数返回值  ：	  无
   =======================================================================================*/
void angClose(float V, float aimAngle, float Kp)
{
	float angError, angOutput;

	angError  = aimAngle - Position_t.angle;
	angError  = AvoidOverAngle(angError);
	angOutput = g_plan * angError * Kp;
	VelCrl(CAN2, 1, V * SP2PULSE + angOutput);
	VelCrl(CAN2, 2, -V * SP2PULSE + angOutput);
}
/*======================================================================================
   函数定义	  ：		矫正函数
   函数参数	  ：		无
   函数返回值    ：	    成功返回1，失败返回0
   =======================================================================================*/
void CheckError(void)
{
	static int8_t signal = 0;
	static int8_t step = 2, count = 0, angle = 0;

	switch (step)
	{
	case 0:

		//为了保护电机
		signal++;
		VelCrl(CAN2, 1, 0);
		VelCrl(CAN2, 1, 0);
		if (signal > 15)
		{
			signal  = 0;
			step    = 1;
		}
		break;

	case 1:
		angle = AvoidOverAngle((count - 1) * 90 - 70);
		angClose(1500, angle, 150);

		//靠不同的墙，step=2的条件也不一样
		if (count == 1)
		{
			if (Position_t.X > 1400)
				step = 2;
		}
		else if (count == 2)
		{
			if (Position_t.Y > 3800)
				step = 2;
		}
		else if (count == 3)
		{
			if (Position_t.X < -1400)
				step = 2;
		}
		else if (count == 4)
		{
			if (Position_t.Y < 1000)
				step = 2;
		}
		else
		{
		}
		break;

	case 2:

		//为了保护电机
		signal++;
		VelCrl(CAN2, 1, 0);
		VelCrl(CAN2, 1, 0);
		if (signal > 15)
		{
			signal  = 0;
			step    = 3;
		}
		break;

	case 3:
		angle = AvoidOverAngle(count * 90);
		angClose(-1000, angle, 100);

		//行程开关触发
		if (SWITCHE2 == 1 && SWITCHC0 == 1)
			step = 4;
		break;
	case 4:

		//如果矫正成功
		if (LaserCheck())
		{
			step = 5;
		}
		else
		{
			step = 0;
			count++;
			if (count == 5)
				count = 1;
		}
		break;

	case 5:

		// 投球
		ShootBallW();
		break;
	}
}
/*======================================================================================
   函数定义	  ：		将小球相对于摄像头的角度转换成相对于陀螺仪的角度
   函数参数	  ：		diatance     小球距离摄像头的距离(mm)
                        angle        小球相对于摄像头的角度（°）
   函数返回值    ：	    aimAngle     小球相对于陀螺仪的角度(单位：度)
   =======================================================================================*/
//float AngCamera2Gyro(float distance,float angle)
//{
//	float ThirdSide=0,rad=0,aimAngle=0;
//	rad=ANGTORAD(180-angle);
//
//	//余弦定理求第三边
//	ThirdSide=sqrt(CAMERATOGYRO*CAMERATOGYRO+distance*distance-2*distance*CAMERATOGYRO*cos(rad));
//
//	//正弦定理求目标角度(弧度)
//	aimAngle=asin(distance*sin(rad)/ThirdSide);
//    return RADTOANG(aimAngle);
//}
/*======================================================================================
   函数定义	  ：		将小球相对于摄像头的距离转换成相对于陀螺仪的距离
   函数参数	  ：		diatance     小球相对于摄像头的距离（mm）
                  angle        小球相对于摄像头的角度（°）
   函数返回值  ：	  ThirdSide    小球相对于陀螺仪的距离(mm)
   =======================================================================================*/
float DisBall2Gyro(float distance, float angle)
{
	float ThirdSide = 0, rad = 0;

	rad = ANGTORAD(180 - angle);

	//余弦定理求第三边
	ThirdSide = sqrt(CAMERATOGYRO * CAMERATOGYRO + distance * distance - 2 * distance * CAMERATOGYRO * cos(rad));


	return ThirdSide;
}
/*======================================================================================
   函数定义	  ：		计算左中右各区域小球的数量
   函数参数	  ：		无
   备注        :     超出中间区域边界线(左8cm)(右10cm)左右，小车仍会认为小球还在中间区域
   函数返回值  ：	  Ballnum.leftNum   左区域的球数
                  Ballnum.midNum    中间区域的球数
                  Ballnum.rightNum  右区域的球数
   =======================================================================================*/
BALLNUM_T SeekMostBall(void)
{
	static BALLNUM_T  ballNum = { 0, 0, 0 };
	int               j       = 0;
	float             verDis  = 0;

	//g_cameraFin为1，说明接收数据完成，开始处理
	if (g_cameraFin)
	{
		ballNum.leftNum   = 0;
		ballNum.midNum    = 0;
		ballNum.rightNum  = 0;

		//消除(摄像头大约60ms更新一次数据，而主函数10ms一个周期，故摄像头更新数据前大约会返回5-7个ballNum的初始化值)
		for (j = 0; j < g_cameraNum; j++)
		{
			//将距离单位cm转换成mm(g_cameraDis[j]*10)
			//将g_cameraAng[j]转换成弧度
			verDis = g_cameraDis[j] * 10 * sin(g_cameraAng[j] * PI / 180);
			USART_OUT(USART1, (u8 *)"verDis%d\r\n", (int)verDis);

			//判定左边有球
			if (g_cameraAng[j] > 8.5)
			{
				//leftFlag置1，表明左边有球。下同
				leftFlag                  = 1;
				leftAng[ballNum.leftNum]  = g_cameraAng[j];
				leftDis[ballNum.leftNum]  = g_cameraDis[j] * 10;
				ballNum.leftNum++;
			}

			//判定右边有球
			else if (g_cameraAng[j] < -8.5)
			{
				rightFlag                   = 1;
				rightAng[ballNum.rightNum]  = g_cameraAng[j];
				rightDis[ballNum.rightNum]  = g_cameraDis[j] * 10;
				ballNum.rightNum++;
			}

			//判定中间有球
			else
			{
				midFlag                 = 1;
				midAng[ballNum.midNum]  = g_cameraAng[j];
				midDis[ballNum.midNum]  = g_cameraDis[j] * 10;
				ballNum.midNum++;
			}
		}

		//将摄像头发完数据瞬间的角度发送出去，用于调整接下来60ms车的角度
		SendAng(Position_t.angle);

		//清零
		g_cameraFin = 0;
	}

	USART_OUT(USART1, (u8 *)"left%d\tmid%d\tright%d\r\n", ballNum.leftNum, ballNum.midNum, ballNum.rightNum);
	return ballNum;
}
/*======================================================================================
   函数定义		：			将摄像机中球的极坐标转换成XY坐标
   函数参数		：		  小球相对于摄像头的角度（°），距离（mm）

   函数返回值	：	    小球在坐标系中的坐标值
   =======================================================================================*/
POSXY_T BallPosXY(uint8_t distance, int8_t angle)
{
	static float    dis = 0, ang = 0;
	static POSXY_T  position = { 0, 0 };

	dis         = DisBall2Gyro(distance, angle);
	ang         = ANGTORAD(AngCamera2Gyro(distance, angle) + Position_t.angle + 90);
	position.X  = Position_t.X + dis * cos(ang);
	position.Y  = Position_t.Y + dis * cos(ang);
	return position;
}
/*======================================================================================
   函数定义	  ：		求得距离数组中的最大值
   函数参数	  ：		无

   函数返回值    ：	    最大的距离值
   =======================================================================================*/
float Max(uint8_t arr[50], int n)
{
	float maxDis  = 0;
	int   i       = 0;

	maxDis = arr[0];
	if (n == 1)
	{
		return maxDis;
	}
	else
	{
		for (i = 1; i < n; i++)
			maxDis = (maxDis >= arr[i] ? maxDis : arr[i]);
		return maxDis;
	}
}
/*======================================================================================
   函数定义	  ：		求得距离数组中的最小值
   函数参数	  ：		无

   函数返回值  ：	  最小的距离值
   =======================================================================================*/
float Min(uint8_t arr[50], int n)
{
	float minDis  = 0;
	int   i       = 0;

	minDis = arr[0];
	if (n == 1)
	{
		return minDis;
	}
	else
	{
		for (i = 1; i < n; i++)
			minDis = (minDis <= arr[i] ? minDis : arr[i]);
		return minDis;
	}
}
/*======================================================================================
   函数定义	  ：		在球最多的区域收集球(精细化调整)(未验证)
   函数参数	  ：		无

   函数返回值  ：
   =======================================================================================*/
void CollectMostBall(void)
{
	float     aveAngle = 0, sumAngle = 0, nowAngle = 0, aimAngle = 0, distance = 0;
	int       i   = 0;
	BALLNUM_T num = { 0, 0, 0 };

	num = SeekMostBall();

	//走中间区域
	if (num.midNum >= num.leftNum && num.midNum >= num.rightNum)
	{
		for (i = 0; i < num.midNum; i++)
			sumAngle += midAng[i];

		//注意midNum可能为0，取平均值
		if (num.midNum != 0)
		{
			aveAngle = sumAngle / num.midNum;

			//求出区域中小球的最长距离
			distance = Max(midDis, num.midNum);

			//将小球相对于摄像头的距离转换成相对于陀螺仪的距离
			aveAngle = DisBall2Gyro(distance, aveAngle);
		}
		if (num.midNum == 0)
		{
			aveAngle = 0;

			//随便设的
			distance = 10;
		}
	}

	//走右边区域
	else if (num.rightNum > num.midNum && num.rightNum >= num.leftNum)
	{
		for (i = 0; i < num.rightNum; i++)
			sumAngle += rightAng[i];
		aveAngle  = sumAngle / num.rightNum;
		distance  = Max(rightDis, num.rightNum);
		aveAngle  = DisBall2Gyro(distance, aveAngle);
	}

	//走左边区域
	else
	{
		for (i = 0; i < num.leftNum; i++)
			sumAngle += leftAng[i];
		aveAngle  = sumAngle / num.leftNum;
		distance  = Max(leftDis, num.leftNum);
		aveAngle  = DisBall2Gyro(distance, aveAngle);
	}
	nowAngle  = GetAng();
	aimAngle  = nowAngle + aveAngle;
	aimAngle  = AvoidOverAngle(aimAngle);
	USART_OUT(USART1, (u8 *)"left%d\tmid%d\tright%d\taveAngle%d\tnowAngle%d\r\n", num.leftNum, num.midNum, num.rightNum, aveAngle, nowAngle);
	angClose(500, aimAngle, 100);
}
/*======================================================================================
   函数定义	  ：		在球最多的区域收集球(粗略调整角度方案)(未验证)
   函数参数	  ：		无

   函数返回值  ：
   =======================================================================================*/
void CollecMostBall(void)
{
	float     angleAdjust = 0, nowAngle = 0, aimAngle = 0;
	BALLNUM_T num = { 0, 0, 0 };

	num = SeekMostBall();

	//走中间区域
	if (num.midNum >= num.leftNum && num.midNum >= num.rightNum)
		angleAdjust = 0;

	//走右边区域
	else if (num.rightNum > num.midNum && num.rightNum >= num.leftNum)
		angleAdjust = -12.5;

	//走左边区域
	else
		angleAdjust = 12.5;
	nowAngle  = GetAng();
	aimAngle  = nowAngle + angleAdjust;
	aimAngle  = AvoidOverAngle(aimAngle);
	USART_OUT(USART1, (u8 *)"left%d\tmid%d\tright%d\tangleAdjust%d\tnowAngle%d\r\n", num.leftNum, num.midNum, num.rightNum, angleAdjust, nowAngle);
	angClose(500, aimAngle, 100);
}
/*======================================================================================
   函数定义	  ：		摄像头返回的数值是球最多区域的角度,直接走那个角度
   函数参数	  ：		无

   函数返回值  ：	  无
   =======================================================================================*/
void CollecMostBall1(void)
{
	static float aimAngle = 0;

	//拉低PE4，拉高PE6的电平，接收球最多区域的角度
	GPIO_SetBits(GPIOE, GPIO_Pin_4);
	GPIO_ResetBits(GPIOE, GPIO_Pin_6);
	g_cameraPlan  = 2;
	aimAngle      = Position_t.angle + g_cameraAng[0];
	aimAngle      = AvoidOverAngle(aimAngle);
//	USART_OUT(UART5,(u8*)"a%d\r\n",(int)g_cameraAng[0]);
	angClose(1000, aimAngle, 200);
} 
/*======================================================================================
   函数定义		：			利用摄像头收集球最多的区域的小球,基本走形回字形
   函数参数		：			预定用摄像头的找球圈数
   函数返回值	：			无
   =======================================================================================*/
int RunWithCamera1(uint8_t circleNum)
{
	static uint8_t  i       = 0, count = 0, side = 0;
	static uint8_t  circle  = 0;
  uint8_t success = 0;
	float aimAngle = 0;
	
	//拉低PE6，拉高PE4的电平，接收球最多区域的角度
	GPIO_SetBits(GPIOE, GPIO_Pin_4);
	GPIO_ResetBits(GPIOE, GPIO_Pin_6);
	g_cameraPlan = 2;
	//USART_OUT(UART5,(u8*)"X%d\ti%d\r\n",(int)Position_t.X,(int)i);
	switch (i)
	{
		case 0:
			side = JudgeSide();
			count++;
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
			angClose(1000,aimAngle,100);
			
		  //直行0.6s
		  if(count >= 60)
			{
				count = 0;
				
				//判断车现在在哪一面墙,从而决定下一步i的值
				if(side == 1)
				{
					i = 1;
				}
				else if(side == 2)
				{
					i = 3;
				}
				else if(side == 3)
				{
					i = 5;
				}
				else
				{
					i = 7;
				}
			}
			break;

		//起步先转弯到-90°，然后i++;进入下一个状态
		case 1:
			angClose(TURNSPEED, -90, 100);
			if (fabs(Position_t.angle + 90) < ANGLEADJUST)
				i++;
			break;

		//在-90°的角度上收集球，当Position_t.X>1600时，转弯，然后下一个角度收集球。以后的步骤类同
		case 2:
			CollecMostBall1();
			if (Position_t.X > (200 + circle * CAMERA_DIS))
				i++;
			break;
		case 3:
			angClose(TURNSPEED, 0, 100);
			if (fabs(Position_t.angle) < ANGLEADJUST)
				i++;
			break;
		case 4:
			CollecMostBall1();
			if (Position_t.Y > (2600 + circle * CAMERA_DIS))
				i++;
			break;
		case 5:
			angClose(TURNSPEED, 90, 100);
			if (fabs(Position_t.angle - 90) < ANGLEADJUST)
				i++;
			break;
		case 6:
			CollecMostBall1();
			if (Position_t.X < -(200 + circle * CAMERA_DIS))
				i++;
			break;
		case 7:
			angClose(TURNSPEED, 180, 100);
			if (fabs(Position_t.angle - 180) < ANGLEADJUST)
				i++;
			break;
		case 8:
			CollecMostBall1();
			if (Position_t.Y < (1900 - circle * CAMERA_DIS))
				i++;
			break;
		case 9:
			angClose(TURNSPEED, -90, 100);
			if (fabs(Position_t.angle + 90) < ANGLEADJUST)
			{
				i = 2;
				circle++;
				
				// 达到预定圈数返回1，然后各种清零
				if(circle == circleNum)
				{
					i = 0;
					circle = 0;
					success = 1;
				}
			}
			break;
	}
	return success;
}
/*======================================================================================
   函数定义		：			第二套摄像头方案
   函数参数		：		  无

   函数返回值	：	    视野中没球返回0
   =======================================================================================*/
int RunWithCamera2(void)
{
	static BALLNUM_T  num = { 0, 0, 0 };
	static POSITION_T startPoint = { 0, 0 };
	static float      angleAdjust = 0, aimAngle = 0, distance = 0;
	static uint8_t    flag = 1, step = 0, posFlag = 1;
	static float      leftDisMax = 0, midDisMax = 0, rightDisMax = 0, disMax = 0, disMin = 0;
	int               success = 1;

	//拉高PE4，PE6的电平，接收所有球的极坐标
	GPIO_SetBits(GPIOE, GPIO_Pin_4);
	GPIO_SetBits(GPIOE, GPIO_Pin_6);
	g_cameraPlan = 3;
	if (flag)
	{
		num = SeekMostBall();

		//如果中间区域球最多,记录球的最大距离，最小距离(同时转化为球相对于陀螺仪的距离(角度值统一用12.5°粗略估计))以及小车的目标角度。下同
		if (num.midNum >= num.leftNum && num.midNum >= num.rightNum)
		{
			//num.midNum=0，表明视野中没球，返回0
			if (num.midNum == 0)
			{
				success = 0;
			}
			else
			{
				disMax  = DisBall2Gyro(Max(midDis, num.midNum), 12.5);
				disMin  = DisBall2Gyro(Min(midDis, num.midNum), 12.5);

				//中间球最多，角度调整为0
				angleAdjust = 0;

				//GetAng()的返回值是摄像头闭眼前一瞬间小车的角度
				aimAngle = AvoidOverAngle(GetAng() + angleAdjust);
			}
		}
		else if (num.rightNum > num.midNum && num.rightNum >= num.leftNum)
		{
			disMax  = DisBall2Gyro(Max(rightDis, num.rightNum), 12.5);
			disMin  = DisBall2Gyro(Min(rightDis, num.rightNum), 12.5);
			disMin  = disMin;//fix me
			//右边球最多，角度调整为-10
			angleAdjust = -10;
			aimAngle    = AvoidOverAngle(GetAng() + angleAdjust);
		}
		else
		{
			disMax  = DisBall2Gyro(Max(leftDis, num.leftNum), 12.5);
			disMin  = DisBall2Gyro(Min(leftDis, num.leftNum), 12.5);

			//左边球最多，角度调整为10
			angleAdjust = 10;
			aimAngle    = AvoidOverAngle(GetAng() + angleAdjust);
		}

		//清零，保证在一次视野中摄像头只睁眼一次
		flag = 0;
	}

	switch (step)
	{
	//第一步，先收集球最多区域的球
	case 0:

		//判断走的是左中右的那个区域
		if (angleAdjust > 5)
			//表明是左边的区域。将leftFlag清0，表明走过了。下同
			leftFlag = 0;
		else if (angleAdjust < -5)
			rightFlag = 0;
		else
			midFlag = 0;

		//记录开始找球时陀螺仪的坐标
		if (posFlag)
		{
			startPoint.X  = Position_t.X;
			startPoint.Y  = Position_t.Y;
			posFlag       = 0;
		}
		distance = GetDistance(startPoint);
		angClose(500, aimAngle, 100);

		//小车行驶的距离超过小球的最大距离
		if (distance > disMax)
		{
			//如果左中右任一区域有球，则进入下一状态,后退
			if (leftFlag == 1 || rightFlag == 1 || midFlag == 1)
			{
				step++;
				posFlag = 1;
			}

			//否则flag=1，摄像头打开。
			else
			{
				step    = 0;
				posFlag = 1;
				flag    = 1;
			}
		}
		break;

	case 1:
		if (posFlag)
		{
			startPoint.X  = Position_t.X;
			startPoint.Y  = Position_t.Y;
			posFlag       = 0;
		}
		distance = GetDistance(startPoint);
		angClose(-500, aimAngle, 100);

		//后退,后退的距离为上次前进的距离
		if (distance > disMax)
		{
			step++;
			posFlag = 1;
		}
		break;

	case 2:

		//如果左边还有球,计算球距离小车的最大距离以及小车的目标角度
		if (leftFlag)
		{
			leftDisMax  = DisBall2Gyro(Max(leftDis, num.leftNum), 12.5);
			aimAngle    = AvoidOverAngle(GetAng() + 10);
			angClose(500, aimAngle, 100);
		}

		//如果左边没有球，则直接进入step4，判断中间有没有球
		else
		{
			step = 4;
		}

		if (posFlag)
		{
			startPoint.X  = Position_t.X;
			startPoint.Y  = Position_t.Y;
			posFlag       = 0;
		}
		distance = GetDistance(startPoint);

		//小车行驶的距离超过左边球的最大距离，step++,进入下一个状态
		if (distance > leftDisMax)
		{
			//如果右边中间还有球，step++，进入下一状态
			if (rightFlag == 1 || midFlag == 1)
			{
				leftFlag = 0;
				step++;
				posFlag = 1;
			}
			else
			{
				step    = 0;
				posFlag = 1;
				flag    = 1;
			}
		}
		break;

	case 3:
		if (posFlag)
		{
			startPoint.X  = Position_t.X;
			startPoint.Y  = Position_t.Y;
			posFlag       = 0;
		}
		distance = GetDistance(startPoint);
		angClose(-500, aimAngle, 100);

		//后退,后退的距离为上次前进的距离
		if (distance > leftDisMax)
		{
			step++;
			posFlag = 1;
		}
		break;

	case 4:

		//如果中间还有球,计算球距离小车的最大距离以及小车的目标角度
		if (midFlag)
		{
			midDisMax = DisBall2Gyro(Max(midDis, num.midNum), 12.5);
			aimAngle  = AvoidOverAngle(GetAng() + 10);
		}

		//没球，step=6
		else
		{
			step = 6;
		}
		angClose(500, aimAngle, 100);
		if (posFlag)
		{
			startPoint.X  = Position_t.X;
			startPoint.Y  = Position_t.Y;
			posFlag       = 0;
		}
		distance = GetDistance(startPoint);

		//小车行驶的距离超过左边球的最大距离，step++,进入下一个状态
		if (distance > midDisMax)
		{
			//如果右边还有球
			if (rightFlag == 1)
			{
				midFlag = 0;
				step++;
				posFlag = 1;
			}
			else
			{
				step    = 0;
				posFlag = 1;
				flag    = 1;
			}
		}
		break;

	case 5:
		if (posFlag)
		{
			startPoint.X  = Position_t.X;
			startPoint.Y  = Position_t.Y;
			posFlag       = 0;
		}
		distance = GetDistance(startPoint);
		angClose(-500, aimAngle, 100);

		//后退,后退的距离为上次前进的距离
		if (distance > leftDisMax)
		{
			step++;
			posFlag = 1;
		}
		break;

	case 6:

		//如果右边还有球,计算球距离小车的最大距离以及小车的目标角度
		if (rightFlag)
		{
			rightDisMax = DisBall2Gyro(Max(rightDis, num.rightNum), 12.5);
			aimAngle    = AvoidOverAngle(GetAng() + 10);
		}
		else
		{
			step  = 0;
			flag  = 1;
		}
		angClose(500, aimAngle, 100);
		if (posFlag)
		{
			startPoint.X  = Position_t.X;
			startPoint.Y  = Position_t.Y;
			posFlag       = 0;
		}
		distance = GetDistance(startPoint);

		//小车行驶的距离超过左边球的最大距离，step++,进入下一个状态
		if (distance > rightDisMax)
		{
			rightFlag = 0;
			step      = 0;
			posFlag   = 1;
		}
		break;
	}
	return success;
}
/*======================================================================================
   函数定义		：			第一圈(第二套方案，回字形中轴线不在X=0上)
   函数参数		：			plan:方案，speed:速度(mm)
   函数返回值	：			false未结束，true第一圈结束
   用时				：			未测算
   (WIDTH为小车宽度)
   =======================================================================================*/
bool FirstRoundW(void)
{
	static int state = 1;

	switch (state)
	{
	//右边，目标角度0度
	case 1:
	{
		StaightCLose((275 + WIDTH / 2 + 100), 0, 0, FIRST_SPEED);
		if (Position_t.Y >= 3100 + WIDTH / 2 - FIR_ADV)
			state = 2;
	}
	break;

	//上边，目标角度90度
	case 2:
	{
		StaightCLose(0, 3100 + WIDTH / 2 + 100, 90, RUN_SPEED);
		if (Position_t.X <= -275 - WIDTH / 2 - SPREAD_DIS + ADV_TUEN)
			state = 3;
	}
	break;

	//左边，目标角度180度
	case 3:
	{
		StaightCLose((-275 - WIDTH / 2 - SPREAD_DIS), 0, 180, RUN_SPEED);
		if (Position_t.Y <= 1700 - WIDTH / 2 - SPREAD_DIS + ADV_TUEN)
			state = 4;
	}
	break;

	//下边，目标角度-90度
	case 4:
	{
		StaightCLose(0, 1700 - WIDTH / 2 - SPREAD_DIS, -90, RUN_SPEED);
		if (Position_t.X >= 275 + WIDTH / 2 + SPREAD_DIS - ADV_TUEN)
			return true;
	}
	break;
	}

	return false;
}
/*======================================================================================
   函数定义		：			长方形扫场(第二套长方形方案,回字形中轴线不在X=0上)
   函数参数		：			length	:	小车中线与放球区y=y1的距离
                    wide		:	小车中线与放球区x=x1的距离
                    speed		:	速度
   函数返回值	    ：	true扫场结束,false未结束
   暂时未加入x的镜像对称
   =======================================================================================*/
bool  RunRectangleW(int length, int wide, float speed)
{
	static int state = 1;

	switch (state)
	{
	//长方形右边，目标角度0度
	case 1:
	{
		StaightCLose(275 + wide, 0, 0, speed);
		if (Position_t.Y >= 3100 + length - ADV_TUEN)
			state = 2;
	}
	break;

	//长方形上边，目标角度90度
	case 2:
	{
		StaightCLose(0, 3100 + length, 90, speed);
		if (wide + SPREAD_DIS >= 2125 - WIDTH / 2 - 100)
			wide = 2125 - WIDTH / 2 - SPREAD_DIS - 100;
		if (Position_t.X <= -275 - wide - SPREAD_DIS + ADV_TUEN)
			state = 3;
	}
	break;

	//长方形左边，目标角度180度
	case 3:
	{
		StaightCLose(-275 - wide - SPREAD_DIS, 0, 180, speed);
		if (length + SPREAD_DIS >= 1700 - WIDTH / 2 - 100)
			length = 1700 - WIDTH / 2 - SPREAD_DIS - 100;
		if (Position_t.Y <= 1700 - length - SPREAD_DIS + ADV_TUEN)
			state = 4;
	}
	break;

	//长方形下边，目标角度-90度
	case 4:
	{
		StaightCLose(0, 1700 - length - SPREAD_DIS, -90, speed);
		if (Position_t.X >= 275 + wide + SPREAD_DIS - ADV_TUEN)
		{
			state = 1;
			return true;
		}
	}
	break;
	}
	return false;
}
/*======================================================================================
   函数定义		：			计算右车头的坐标(顺时针跑场)
   函数参数		：		    无

   函数返回值	    ：	        右车头的坐标结构体
   暂时未加入x的镜像对称
   =======================================================================================*/
POSXY_T RightHeadPos(void)
{
	float   angle = 0;
	POSXY_T position;

	//计算右车尖与陀螺仪连线在直角坐标系中与X轴的夹角
	angle       = AvoidOverAngle(Position_t.angle - ANGRIGHTGYRO + 90);
	angle       = ANGTORAD(angle);
	position.X  = Position_t.X + DISRIGHTGYRO * cos(angle);
	position.Y  = Position_t.Y + DISRIGHTGYRO * sin(angle);
	return position;
}
/*======================================================================================
   函数定义		：			计算投球点的坐标
   函数参数		：		  无

   函数返回值	：	    投球点的坐标
   =======================================================================================*/
POSXY_T ShootPointPos(void)
{
	float   angle     = 0;
	POSXY_T position  = { 0, 0 };

	angle       = AvoidOverAngle(g_plan * Position_t.angle + 90);
	angle       = ANGTORAD(angle);
	position.X  = g_plan * Position_t.X + DISSHOOTTOGYRO * cos(angle);
	position.Y  = Position_t.Y + DISSHOOTTOGYRO * sin(angle);
	return position;
}

/*======================================================================================
   函数定义		：			逃逸函数
   函数参数		：		  无

   函数返回值	：	    逃逸完成返回1，未完成返回0
   =======================================================================================*/
int IfEscape(void)
{
	static int  backSignal = 0, Step = 1;
	int         success = 0;

	switch (Step)
	{
	case 1:
		VelCrl(CAN2, 1, -8000);
		VelCrl(CAN2, 2, 8000);
		backSignal++;

		//倒车1s，则进入下一状态
		if (backSignal > 100)
		{
			backSignal  = 0;
			Step        = 2;
		}
		break;

	case 2:
		backSignal++;

		//表明为大圈，左转
		if (Position_t.X <= -1200 || Position_t.X >= 1200 || Position_t.Y >= 3600 || Position_t.Y <= 1200)
		{
			VelCrl(CAN2, 1, 4500);
			VelCrl(CAN2, 2, 8100);
		}

		//否则为小圈，右转
		else
		{
			VelCrl(CAN2, 1, -8100);
			VelCrl(CAN2, 2, -4500);
		}

		//返回值为1，表明避障初步完成
		if (backSignal >= 20)
		{
			Step        = 1;
			backSignal  = 0;
			success     = 1;
		}
		break;
	}
	return success;
}

/*======================================================================================
   函数定义		：		一系列控制发射电机用的函数
   函数参数		：		无

   函数返回值	    ：	    无
   =======================================================================================*/
//发射航向角转换函数 由度转换为脉冲
float YawTransform(float yawAngle)
{
	return yawAngle * YAW_REDUCTION_RATIO * COUNT_PER_DEGREE;
}

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void YawAngleCtr(float yawAngle)
{
	PosCrl(CAN1, GUN_YAW_ID, POS_ABS, YawTransform(yawAngle));
}


//送弹推球函数
void PushBall(void)
{
	PosCrl(CAN1, PUSH_BALL_ID, POS_ABS, PUSH_POSITION);
}

//送弹推球收回函数
void PushBallReset(void)
{
	PosCrl(CAN1, PUSH_BALL_ID, POS_ABS, PUSH_RESET_POSITION);
}

//收球电机速度转换函数 由转每秒转换为脉冲
float CollectBallVelTrans(float round)
{
	return round * COUNT_PER_ROUND;
}

//收球电机速度控制函数 单位：转每秒
void CollectBallVelCtr(float round)
{
	VelCrl(CAN1, COLLECT_BALL_ID, CollectBallVelTrans(round));
}

//发射电机速度转换函数 由转每秒转换为脉冲
int32_t shootVelTrans(float roundPerS)
{
	return (int32_t)-roundPerS * COUNT_PER_ROUND;
}

//发射电机速度控制函数 单位：转每秒
void ShootCtr(float rps)
{
	shootPara_t shootPara;

	shootPara.velInt32 = shootVelTrans(rps);

	//起始位
	USART_SendData(USART1, 'A');
	//通过串口1发数
	USART_SendData(USART1, shootPara.velUint8[0]);
	USART_SendData(USART1, shootPara.velUint8[1]);
	USART_SendData(USART1, shootPara.velUint8[2]);
	USART_SendData(USART1, shootPara.velUint8[3]);
	//终止位
	USART_SendData(USART1, 'J');
}
/*======================================================================================
   函数定义		：			定点投球方案
   函数参数		：		    

   函数返回值	：	        1 表明射球完成
   =======================================================================================*/
extern int ballColor,youqiu;
extern int ballSpeed,need;
int ShootBallD(float wx,float by,float bx)
{
	static uint16_t noBall = 0, flag = 0, notShoot = 0;
  POSXY_T  posShoot = { 0, 0 };
	int      success = 0;
	static float  shootAngle = 0, distance = 2300,aimAngle = 0, V = 0, rps = 0;
	static int32_t lastPosition = 0, notMove = 0;
	static int8_t step = 0, ifCount = 0;
	
	//问询航向电机角度、收球电机速度、推球电机的位置
  ReadActualPos(CAN1, GUN_YAW_ID);
	ReadActualVel(CAN1, COLLECT_MOTOR_ID);
	ReadActualPos(CAN1, PUSH_BALL_ID);

	//计算投球点的坐标
	posShoot  = ShootPointPos();
	switch(step)
	{
		case 0:
			
			//球是白球
			if (ballColor == WHITE)
			{
				noBall = 0;
				distance = sqrt((posShoot.X - wx) * (posShoot.X - wx) + (posShoot.Y - by) * (posShoot.Y - by));

				//将角度装换成陀螺仪角度坐标系里的角度值
				aimAngle  = atan2(BALLY - posShoot.Y, wx - posShoot.X);
				aimAngle  = RADTOANG(aimAngle) - 90;

				//枪顺时针转为正，逆时针为负
				aimAngle = AvoidOverAngle(aimAngle);
				shootAngle = AvoidOverAngle(g_plan * Position_t.angle - aimAngle) + 2;
			}

			//球是黑球
			else if (ballColor == BLACK)
			{
				noBall = 0;
				distance = sqrt((posShoot.X - bx) * (posShoot.X - bx) + (posShoot.Y - by) * (posShoot.Y - by));

				//将角度转换成陀螺仪角度坐标系里的角度值
				aimAngle  = atan2(BALLY - posShoot.Y, bx - posShoot.X);
				aimAngle  = RADTOANG(aimAngle) - 90;
				aimAngle  = AvoidOverAngle(aimAngle);
				shootAngle = AvoidOverAngle(g_plan * Position_t.angle - aimAngle) + 2;
			}
		
			// 没球,来回拨动几次
			else if (ballColor == NO)
			{
//				USART_OUT(UART5,(u8*)"n%d\t%d\r\n",(int)noBall,(int)success);
				noBall++;
				if(noBall > 120 && noBall < 130)
				{
						PushBall();
				}
				if(noBall > 240 && noBall < 250)
				{		
						PushBallReset();	
				}
				if(noBall > 360 && noBall < 370)
				{	
						PushBall();	
				}
				if(noBall > 480 && noBall < 490)
				{					
						PushBallReset();					
				}
				if(noBall > 600 && noBall < 610)
				{	
						PushBall();	
				}
				if(noBall > 300)
				{
					noBall = 0;
					
					//射球完成，shootNum置0
					notMove = 0;
					step = 0;
//					shootNum = 0;
					success = 1;
				}
			}
		
			//球出射速度(mm/s)与投球点距离篮筐的距离的关系
			V = sqrt(12372.3578 * distance * distance / (distance * 1.2349 - 424.6));
			
			//自己测的关系
			rps = 0.01402f * V - 5.457f + 2.8;
		
			
			// 表明射球蓝牙没有收到主控发送的数据
			if (fabs(rps + g_shootV / 4096) > 0.1)
			{
					ShootCtr(rps);
			}
			
			//控制发射航向角(20ms发一次)
			flag++;
			flag = flag % 2;
			if(flag == 1)
			{
				YawAngleCtr(shootAngle);
			}
			
		  //记录射球的个数
			if(fabs(rps + (g_shootFactV / 4096)) < 1)
			{
				ifCount = 1;
			}
			if(ifCount)
			{
				if(rps + (g_shootFactV / 4096) > 2.7)
				{
					shootNum++;
					ifCount = 0;
				}
			}
			
			//枪的角度和转速到位,推球
			if(fabs(shootAngle - g_shootAngle * 90 / 4096) < 2.0f && fabs(rps + g_shootFactV / 4096) < 2 && ballColor)
			{
				//位置正常，reset推球电机
				if(g_pushPosition > 3200)
				{
					PushBallReset();
				}
				
				//位置正常，push推球电机
				if(g_pushPosition < 800)
				{
					PushBall();
				}
			}
		
			//判断球是否卡死
			if(abs(g_pushPosition - lastPosition) < 5 )
			{
				notShoot++;
				
				//0.2s依然卡死，切换到step = 1;
				if(notShoot > 20)
				{
					notShoot = 0;
					step = 1;
				}
			}
			else
			{
				notShoot = 0;
			}
			
//			USART_OUT(UART5,(u8*)"l%d\t%d\t%d\r\n",(int)g_pushPosition,(int)lastPosition,(int)notMove);
			lastPosition = g_pushPosition;
			break;
			
		//推球电机卡死，特殊处理的步骤
		case 1:
			notMove++;
			if(g_pushPosition >= 2000)
			{
				PushBall();
			}
			else
			{
				PushBallReset();
			}
			
			//连续发10次命令
			if(notMove >= 10)
			{
				notMove = 0;
				step = 0;
			}
			break;
		
	}
//	USART_OUT(UART5,(u8*)"%d\tf%d\t%d\tf%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)shootAngle,(int)(g_shootAngle * 90 / 4096),(int)rps,(int)g_shootFactV/4096,(int)(g_shootV / 4096),(int)distance,(int)Position_t.X,(int)Position_t.Y,(int)Position_t.angle,(int)xError,(int)yError);
//	USART_OUT(UART5,(u8*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)shootNum,ballColor,noBall,success,(int)g_pushPosition,(int)notMove,(int)notShoot);
//	USART_OUT(UART5,(u8*)"%d\t%d\t%d\r\n",(int)rps,(int)g_shootFactV/4096,(int)shootNum);
//	USART_OUT(UART5,(u8*)"%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)Position_t.X,(int)Position_t.Y,(int)Position_t.angle,(int)xError,(int)yError,(int)angleError);
	return success;
}
int ShootBallW(void)
{
	static uint16_t noBall = 0, flag = 0, notShoot = 0;
  POSXY_T  posShoot = { 0, 0 };
	int      success = 0, side = 0;
	static float  shootAngle = 0, distance = 2300,aimAngle = 0, V = 0, rps = 0;
	static int32_t lastPosition = 0, notMove = 0;
	static int8_t step = 0, ifCount = 0;
	
	//问询航向电机角度、收球电机速度、推球电机的位置
  ReadActualPos(CAN1, GUN_YAW_ID);
	ReadActualVel(CAN1, COLLECT_MOTOR_ID);
	ReadActualPos(CAN1, PUSH_BALL_ID);
	 
	//行程开关都触发，一直进行角度矫正
  if(SWITCHC0 == 1 && SWITCHE2 == 1)
	{
		side = JudgeSide();
		
	}
	//计算投球点的坐标
	posShoot  = ShootPointPos();
	switch(step)
	{
		case 0:
			
			//球是白球
			if (ballColor == WHITE)
			{
				noBall = 0;
				distance = sqrt((posShoot.X - WHITEX) * (posShoot.X - WHITEX) + (posShoot.Y - BALLY) * (posShoot.Y - BALLY));

				//将角度装换成陀螺仪角度坐标系里的角度值
				aimAngle  = atan2(BALLY - posShoot.Y, WHITEX - posShoot.X);
				aimAngle  = RADTOANG(aimAngle) - 90;

				//枪顺时针转为正，逆时针为负
				aimAngle = AvoidOverAngle(aimAngle);
				shootAngle = AvoidOverAngle(g_plan * Position_t.angle - aimAngle) + 3;
			}

			//球是黑球
			else if (ballColor == BLACK)
			{
				noBall = 0;
				distance = sqrt((posShoot.X - BLACKX) * (posShoot.X - BLACKX) + (posShoot.Y - BALLY) * (posShoot.Y - BALLY));

				//将角度转换成陀螺仪角度坐标系里的角度值
				aimAngle  = atan2(BALLY - posShoot.Y, BLACKX - posShoot.X);
				aimAngle  = RADTOANG(aimAngle) - 90;
				aimAngle  = AvoidOverAngle(aimAngle);
				shootAngle = AvoidOverAngle(g_plan * Position_t.angle - aimAngle) + 3;
			}
		
			// 没球,来回拨动几次
			else if (ballColor == NO)
			{
//				USART_OUT(UART5,(u8*)"n%d\t%d\r\n",(int)noBall,(int)success);
				noBall++;
				if(noBall > 160 && noBall < 180)
				{
						PushBall();
				}
				if(noBall > 320 && noBall < 340)
				{		
						PushBallReset();	
				}
				if(noBall > 480 && noBall < 500)
				{	
						PushBall();	
				}

				if(noBall > 520)
				{
					noBall = 0;
					
					//射球完成，shootNum置0
					notMove = 0;
					step = 0;
//					shootNum = 0;
					success = 1;
				}
			}
		
			//球出射速度(mm/s)与投球点距离篮筐的距离的关系
			V = sqrt(12372.3578 * distance * distance / (distance * 1.2349 - 424.6));
			
			//自己测的关系
			rps = 0.01499f * V - 7.494f;
		
			
			// 表明射球蓝牙没有收到主控发送的数据
			if (fabs(rps + g_shootV / 4096) > 0.1)
			{
					ShootCtr(rps);
			}
			
			//控制发射航向角(30ms发一次)
			flag++;
			flag = flag % 3;
			if(flag == 1)
			{
				YawAngleCtr(shootAngle);
			}
			
		  //记录射球的个数
			if(fabs(rps + (g_shootFactV / 4096)) < 1)
			{
				ifCount = 1;
			}
			if(ifCount)
			{
				if(rps + (g_shootFactV / 4096) > 2.7)
				{
					shootNum++;
					ifCount = 0;
				}
			}
			
			//枪的角度和转速到位,推球
			if(fabs(shootAngle - g_shootAngle * 90 / 4096) < 2.0f && fabs(rps + g_shootFactV / 4096) < 2 && ballColor)
			{
				//位置正常，reset推球电机
				if(g_pushPosition > 4000)
				{
					PushBallReset();
				}
				
				//位置正常，push推球电机
				if(g_pushPosition < 400)
				{
					PushBall();
				}
			}
		
			//判断球是否卡死
			if(abs(g_pushPosition - lastPosition) < 5 )
			{
				notShoot++;
				
				//0.2s依然卡死，切换到step = 1;
				if(notShoot > 20)
				{
					notShoot = 0;
					step = 1;
				}
			}
			else
			{
				notShoot = 0;
			}
			
//			USART_OUT(UART5,(u8*)"l%d\t%d\t%d\r\n",(int)g_pushPosition,(int)lastPosition,(int)notMove);
			lastPosition = g_pushPosition;
			break;
			
		//推球电机卡死，特殊处理的步骤
		case 1:
			notMove++;
			if(g_pushPosition >= 2400)
			{
				PushBall();
			}
			else
			{
				PushBallReset();
			}
			
			//连续发10次命令
			if(notMove >= 10)
			{
				notMove = 0;
				step = 0;
			}
			break;
	}
//	USART_OUT(UART5,(u8*)"%d\tf%d\t%d\tf%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)shootAngle,(int)(g_shootAngle * 90 / 4096),(int)rps,(int)g_shootFactV/4096,(int)(g_shootV / 4096),(int)distance,(int)Position_t.X,(int)Position_t.Y,(int)Position_t.angle,(int)xError,(int)yError);
//	USART_OUT(UART5,(u8*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)shootNum,ballColor,noBall,success,(int)g_pushPosition,(int)notMove,(int)notShoot);
//	USART_OUT(UART5,(u8*)"%d\t%d\t%d\r\n",(int)rps,(int)g_shootFactV/4096,(int)shootNum);
//	USART_OUT(UART5,(u8*)"%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)Position_t.X,(int)Position_t.Y,(int)Position_t.angle,(int)xError,(int)yError,(int)angleError);
	USART_OUT(UART5,(u8*)"%d\r\n",(int)g_pushPosition);
	return success;
}


/*======================================================================================
   函数定义		：			圆形跑场
   函数参数		：		  V                跑场速度
                      R                跑场第一圈的半径
                      circleNum        预定的跑场圈数
                      status           扩大还是缩小扫场

   函数返回值	：	    扫完预定的圈数返回1
 
=======================================================================================*/
int sweepYuan(float V, float R, uint8_t circleNum, uint8_t status)
{
  float disError = 0, disOutput = 0, aimAng = 0, angError = 0, angOutput = 0, V1 = 0, V2 = 0;
	uint8_t success = 0;
	static uint8_t Flag = 0, circle = 0;
	static uint16_t R1 = 0;
	
	//只是为了让开始时R1 = R
	circleFlag++;
  if(circleFlag <= 100)
	{
		R1 = R;
	}
  if(circleFlag > 100)
	{
		circleFlag = 100;
	}		

	// 逆时针，先缓慢加速
	if(g_plan == 1)
	{
		V1 = 4096 * V * (R1 + WHEEL_TREAD / 2) / (106.8 * PI * R1);
		V2 = 4096 * V * (R1 - WHEEL_TREAD / 2) / (106.8 * PI * R1);	
	}
	
	//顺时针
	else
	{
		V2 = 4096 * V * (R1 + WHEEL_TREAD / 2) / (106.8 * PI * R1);
		V1 = 4096 * V * (R1 - WHEEL_TREAD / 2) / (106.8 * PI * R1);	
	}
	
	//让小车走一圈半径改变一次
	if(0 < Position_t.X && 1950 < Position_t.Y && Position_t.Y < 2100)
	{
		Flag++;
	}
	else
	{
		Flag = 0;
	}
	if(Flag == 1)
	{
		//记录扫场的圈数
		circle++;
		
		//status=1,扩大扫场
		if(status == 1)
		{
		  R1 += 350;
		}
		
		//否则,缩小扫场
		else
		{
			R1 -= 350;
		}
		
		//达到预定圈数,success置1,acceSpeed置0
		if(circle == circleNum)
		{
			circleFlag = 0;
			success = 1;
		}
	}
	
  //陀螺仪到圆心的距离
	disError = sqrt((Position_t.X - 0) * (Position_t.X - 0) + (Position_t.Y - 2335.35) * (Position_t.Y - 2335.35)) - R1;
	
	//距离P调节系数
	disOutput = g_plan * disError * 15;

	//目标角度与小车位置到圆心的角度相同
	aimAng = atan2(Position_t.Y - 2335.35,Position_t.X - 0) * 180.0 / PI;
	
	//防止第二象限被撞后出现原地大角度调节到目标角度的现象
	if(aimAng < 180 && aimAng > 0 && Position_t.angle < -90 && Position_t.angle > -180)
	{
		Position_t.angle += 360;
	}
	
	//防止第三象限别撞后出现原地大角度调节到目标角度的现象
	if(aimAng < 0 && aimAng > -180 && Position_t.angle < 180 && Position_t.angle > 90)
	{
		Position_t.angle -= 360;
	}
	angError = aimAng - Position_t.angle;
	
	//角度P调节系数
	angOutput = g_plan * angError * 350;
	
	VelCrl(CAN2, 1,V1+disOutput+angOutput);
	VelCrl(CAN2, 2,-V2+disOutput+angOutput);
	return success;
}
/*======================================================================================
   函数定义		：			走一段圆弧
   函数参数		：		  V                跑场速度
                      R                跑场的半径

   函数返回值	：	    扫完预定的圈数返回1
 
=======================================================================================*/
int Arc(float V, float R)
{
	float disError = 0, disOutput = 0, aimAng = 0, angError = 0, angOutput = 0, V1 = 0, V2 = 0;
	uint8_t success = 0;
	
	// 逆时针，先缓慢加速
	if(g_plan == 1)
	{
		V1 = 4096 * V * (R + WHEEL_TREAD / 2) / (106.8 * PI * R);
		V2 = 4096 * V * (R - WHEEL_TREAD / 2) / (106.8 * PI * R);	
		
	}
	
	//顺时针
	else
	{
		V2 = 4096 * V * (R + WHEEL_TREAD / 2) / (106.8 * PI * R);
		V1 = 4096 * V * (R - WHEEL_TREAD / 2) / (106.8 * PI * R);	
	}
	//陀螺仪到圆心的距离
	disError = sqrt((Position_t.X - 0) * (Position_t.X - 0) + (Position_t.Y - 2335.35) * (Position_t.Y - 2335.35)) - R;
	
	//距离P调节系数
	disOutput = g_plan * disError * 10;

	//目标角度与小车位置到圆心的角度相同
	aimAng = atan2(Position_t.Y - 2335.35,Position_t.X - 0) * 180.0 / PI;
	
	//防止第二象限被撞后出现原地大角度调节到目标角度的现象
	if(aimAng < 180 && aimAng > 0 && Position_t.angle < -90 && Position_t.angle > -180)
	{
		Position_t.angle += 360;
	}
	
	//防止第三象限别撞后出现原地大角度调节到目标角度的现象
	if(aimAng < 0 && aimAng > -180 && Position_t.angle < 180 && Position_t.angle > 90)
	{
		Position_t.angle -= 360;
	}
	angError = aimAng - Position_t.angle;
	
	//角度P调节系数
	angOutput = g_plan * angError * 200;
	
	VelCrl(CAN2, 1,V1+disOutput+angOutput);
	VelCrl(CAN2, 2,-V2+disOutput+angOutput);
//	USART_OUT(UART5,(u8*)"%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\r\n",(int)aimAng,(int)angOutput,(int)disOutput,(int)R1,(int)Position_t.X,(int)Position_t.Y,(int)Position_t.angle,(int)(V1+disOutput+angOutput),(int)(-V2+disOutput+angOutput));
	return success;
}
/*======================================================================================
   函数定义		：		  计算小车当前的实时速度（mm/s）
   函数参数		：		  无

   函数返回值	：	    小车当前的速度
 =====================================================================================*/

float RealVel(void)
{
	float V = 0;
	static POSITION_T lastPoint={0,0};
	float distance = 0.0f;
	distance = sqrt((Position_t.X - lastPoint.X) * (Position_t.X - lastPoint.X) + (Position_t.Y - lastPoint.Y) * (Position_t.Y - lastPoint.Y));
	
	//主函数运行4次，进入此函数1次，故*50计算每秒的速度
	V = distance * 100;
	
	//将小车当前的坐标记录下来
	lastPoint = Position_t;
	return V;
}
/*======================================================================================
   函数定义		：		  利用轮子返回的当前转速判断车是否卡住
   函数参数		：		  设定的认为车子被困的速度,反应时间

   函数返回值	：	    如果车被卡,返回1
 =====================================================================================*/
int stuckCar(uint16_t stuckV, uint16_t time)
{
	static float V = 0;
	static uint8_t count = 0;
	uint8_t success = 0;
	
	//获取车当前的速度
	V = RealVel();
	USART_OUT(UART5,(u8*)"c%d\tV%d\r\n",count,(int)V);
	
	//车速小于stuckV,认为车被困,count++
	if(V < stuckV)
	{
		count++;
	}
	else
	{
		count = 0;
	}
	
	//时间累积到time ms认为车被困
	if(count >= (time/10))
	{
		success = 1;
		count = 0;
	}
	return success;
}
/*======================================================================================
   函数定义		：		  判断车距离哪面墙最近
   函数参数		：		  
   备注       ：      顺时针时墙的1234顺时针排序，逆时针时逆时针排序
   函数返回值	：	    墙的标志
 =====================================================================================*/
int JudgeSide(void)
{
	uint8_t side = 0;
	if(Position_t.X > 0 && Position_t.Y < 2335.35)
	{
		if(Position_t.Y + 64.65 > (2400 - Position_t.X))
		{
			side = 2;
		}
		else
		{
			side = 1;
		}
	}
	else if(Position_t.X > 0 && Position_t.Y >= 2335.35)
	{
		if((4800 - 64.65 - Position_t.Y) > (2400 - Position_t.X))
		{
			side = 2;
		}
		else
		{
			side = 3;
		}
	}
	else if(Position_t.X < 0 && Position_t.Y > 2335.35)
	{
		if((4800 - 64.65 - Position_t.Y) > (2400 + Position_t.X))
		{
			side = 4;
		}
		else
		{
			side = 3;
		}
	}
	else
	{
		if((Position_t.Y + 64.65) > (2400 + Position_t.X))
		{
			side = 4; 
		}
		else
		{
			side = 1;
		}
	}
	//USART_OUT(UART5,(u8*)"%d\t%d\t%d\r\n",(int)side,(int)Position_t.X,(int)Position_t.Y);
	return side;
}
/*======================================================================================
   函数定义		：		  画圆之后的矩形扫场
   函数参数		：		  
   
   函数返回值	：	    完成之后返回1
 =====================================================================================*/
int AfterCircle(uint16_t speed)
{
	static uint8_t step = 5;
	uint8_t success = 0;
	static float tempx = 0;
	switch(step)
	{
		case 0:
			StaightCLose(tempx, 0, 0, speed);
			if(Position_t.Y > 2400)
				step++;
			break;
		case 1:
			StaightCLose(0, 4200, 90, speed);
			if(Position_t.X < -100)
				step++;
			break;
		case 2:
			StaightCLose(-1900, 0, 180, speed);
			if(Position_t.Y < 1700)
				step++;
			break;
		case 3:
			StaightCLose(0, 600, -90, speed);
			if(Position_t.X > 100)
				step++;
			break;
		case 4:
			StaightCLose(1900, 0, 0, speed);
			if(Position_t.Y > 2100)
			{
				step = 0;
				success = 1;
			}
			break;
			
			//记录当前的X坐标，以便更好的切换到矩形扫场
		case 5:
			tempx = Position_t.X;
		  step = 0;
			break;
	}
	return success;
}
/*======================================================================================
   函数定义		：		  激光触发
   函数参数		：		  
   
   函数返回值	：	    返回激光距离
 =====================================================================================*/
u16 LaserTrigger(void)
{
	u16 laser = 0, lastLaser = 0, count = 0;
	GPIO_SetBits(GPIOE,GPIO_Pin_7);
	//等待激光被触发
	do
	{
		g_plan = IfStart();
	}while(g_plan == 0);
  GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	//等待激光值稳定
	if(g_plan == 1)
	{
		do
		{
			laser = Get_Adc_Average(RIGHT_LASER, 10);
			USART_OUT(UART5,(u8*)"1%d\t%d\t%d\r\n",lastLaser,laser,count);
			if(abs(laser - lastLaser) < 10)
			{
				count++;
			}
			lastLaser = laser;
		}while(count < 20);
	}
	else
	{
		do
		{
			laser = Get_Adc_Average(LEFT_LASER, 10);
			USART_OUT(UART5,(u8*)"-1%d\t%d\t%d\r\n",lastLaser,laser,count);
			if(abs(laser - lastLaser) < 10)
			{
				count++;
			}
			lastLaser = laser;
		}while(count < 20);
	}
	count = 0; 
	return laser;
}
/*======================================================================================
   函数定义		：		  逃逸函数
   函数参数		：		  back	后退时间
											turn	转弯时间
   函数返回值	：	    
 =====================================================================================*/
int Escape(u16 back,u16 turn)
{
	static u16 time = 0, step = 0;
	static int status = 0;
	static float aimAngle = 0;
	u8 success = 0;
	switch(step)
	{
		case 0:
			status = In_Or_Out();
		  aimAngle = Position_t.angle;
		
			//车子不在四个死角
			if(status == 0 || status == -1)
			{
				step = 1;
			}
			
			//车子在四个死角
			else
			{
				step = 3;
			}
			break;
		case 1:
			
			//后退
			time++;
			if (time < back)
			{
				angClose(-1000,aimAngle,100);
			}
			else
			{
				step = 2;
				time = 0;
			}
			break;
		case 2:
			time++;
		
			//内圈转弯
			if(status == 0)
			{
				angClose(1000,(aimAngle - 70),100);
			}
			
			//外圈转弯
			else
			{
				angClose(1000,(aimAngle + 70),100);
			}
			if(time > turn)
			{
				time = 0;
				step = 0;
				success = 1;
			}
		 break;
		case 3:
			if(status == 1)
			{
				//如果偏转角度过大，进入第四步，特殊调整
				if(fabs(Position_t.angle) > 135)
				{
					aimAngle = -90;
					step = 4;
				}
				
				//否则，返回第一步，正常调整
				else
				{
					step = 1;
				}
			}
			else if(status == 2)
			{
				if(Position_t.angle < -45)
				{
					aimAngle = 0;
					step = 4;
				}
				else
				{
					step = 1;
				}
			}
			else if(status == 3)
			{
				if(Position_t.angle < 45)
				{
					aimAngle = 90;
					step = 4;
				}
				else
				{
					step = 1;
				}
			}
			else
			{
				if(Position_t.angle < 135 && Position_t.angle > 45)
				{
					aimAngle = 180;
					step = 4;
				}
				else
				{
					step = 1;
				}
			}
			break;
		case 4:
			time++;
			angClose(-1200,aimAngle,100);
		  if(time > turn)
			{
				time = 0;
				
				//计算下一刻的目标角度
				if(status == 1)
				{
					aimAngle = 0;
				}
				else if(status == 2)
				{
					aimAngle = 90;
				}
				else if(status == 3)
				{
					aimAngle = 180;
				}
				else
				{
					aimAngle = -90;
				}
				step = 5;
			}
			break;
		case 5:
			time++;
			angClose(1200,aimAngle,120);
		
			//转弯
			if(time > turn)
			{
				time = 0;
				step = 0;
				success = 1;
			}
			break;
	}
	return success;
}


/*======================================================================================
   函数定义		：	检测车是否被平移	  
   函数参数		：		  
   
   函数返回值	：	被平移返回1，否则返回0    
 =====================================================================================*/
int IfTranslate(float distance)
{
	static u8 step = 0, side = 0, count = 0;
	static float firstx = 0, firsty = 0, lastx = 0, lasty = 0;
	u8 success = 0;
	switch(step)
	{
		case 0:
			
		  //判断在哪面墙
			side = JudgeSide();
		  step = 1;
			break;
		case 1:
			
		  //第一或第三面墙
			if(side == 1 || side == 3)
			{
				//Y坐标基本不变，X坐标持续增大，认为被平移了
				if(fabs(Position_t.Y -firsty) < 200 && fabs(Position_t.X - lastx) > 0.3)
				{
					count++;
				}
				else
				{
					count = 0;
				}
				
				//1s后，进入下一步
				if(count > 100)
				{
					step = 3;
				}
			}
	
			//其他的墙
			else
			{
				//X坐标基本不变，X坐标持续增大，认为被平移了
				if(fabs(Position_t.X -firstx) < 200 && fabs(Position_t.Y - lasty) > 0.3)
				{
					count++;
				}
				else
				{
					count = 0;
				}
				
				//1s后，进入下一步
				if(count > 100)
				{
					step = 3;
				}
			}
			break;
		case 3:
			
		  //平移距离大于distance，认为被平移太远了
			if(side == 1 || side == 3)
			{
				if(fabs(Position_t.X - firstx) > distance)
					success = 1;
			}
			else
			{
				if(fabs(Position_t.Y - firsty) > distance)
					success = 1;
			}
			break;
	}
	lastx = Position_t.X;
	lasty = Position_t.Y;
	return success;
}
/*======================================================================================
   函数定义		：	检测车转弯过程中角度是否变化	  
   函数参数		：		  
   
   函数返回值	：	转弯时被卡返回1    
 =====================================================================================*/
int AngleStuck(void)
{
	int success = 0;
	static float lastAngle = 0;
	static u16 count = 0;
	if(fabs(Position_t.angle - lastAngle) < 0.1)
	{
		count++;
		if(count > 20)
		{
			count = 0;
			success = 1;
		}
	}
	else
	{
		count = 0;
	}
	lastAngle = Position_t.angle;
	return success;
}
