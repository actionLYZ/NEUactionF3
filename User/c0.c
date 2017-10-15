#include "c0.h"
#include "stdlib.h"
#include "wan.h"
#include "usart.h"
extern POSITION_T Position_t;
extern int        g_plan;
float             angleP, angleD, distantP, pid1, pid2;
int               yiquan, line, SPE = 0,finishShoot=0;
extern int8_t     arr1[20];
extern float    arr2[20];
extern int        arr_number;
extern int32_t g_gather;
extern int ifEscape;
int ballNumber=0;

/*======================================================================================
   函数定义	  ：		将小球相对于摄像头的角度转换成相对于陀螺仪的角度(万典学长的函数)
   函数参数	  ：		diatance     小球距离摄像头的距离(mm)
                  angle        小球相对于摄像头的角度
   函数返回值  ：	  aimAngle     小球相对于陀螺仪的角度(单位：度)
   =======================================================================================*/
float AngCamera2Gyro(float distance, float angle)
{
	float ThirdSide = 0, rad = 0, aimAngle = 0;

	rad = ANGTORAD(180 - angle);

	//余弦定理求第三边
	ThirdSide = sqrt(CAMERATOGYRO * CAMERATOGYRO + distance * distance - 2 * distance * CAMERATOGYRO * cos(rad));

	//正弦定理求目标角度(弧度)
	aimAngle = asin(distance * sin(rad) / ThirdSide);
	return RADTOANG(aimAngle);
}
/*======================================================================================
   函数定义	  ：		避免角度溢出(万典学长的函数)
   函数参数	  ：		当前角度
   函数返回值    ：		修正后的角度
   =======================================================================================*/
//float AvoidOverAngle(float angle)
//{
//	if(angle<=-180)
//	{
//		angle+=360;
//	}
//	if(angle>180)
//	{
//		angle-=360;
//	}
//	return angle;
//}
//为了保证能循环
void  GoOn(void)
{
	yiquan = 0; line = 1;
}
//一点点加速(米每秒)
int SlowSpeedUp(int topspeed)
{
	SPE = SPE + 20;
	if (SPE >= topspeed)
		SPE = topspeed;
	return SPE;
}
//脉冲
int SlowSpeedUp2(int topspeed)
{
	SPE = SPE + 300;
	if (SPE >= topspeed)
		SPE = topspeed;
	return SPE;
}

//角度PID调节
float PidAngle(float exAngle, float actAngle)
{
	static float error = 0, error_old = 0, kp, kd = 0, adAngle = 0;

	kp    = angleP;
	kd    = angleD;
	error = exAngle - actAngle;

	if (error > 180)
		error = error - 360;
	else if (error < -180)
		error = 360 + error;
	adAngle   = kp * error + kd * (error - error_old);
	error_old = error;
	return adAngle;
}
//距离PID调节
float PidCoordinate(float ex, float act)
{
	static float error = 0, error_old = 0, kp, kd = 0, ad = 0;

	kp        = distantP;
	error     = ex - act;
	ad        = kp * error + kd * (error - error_old);
	error_old = error;
	return ad;
}
/*======================================================================================
   函数定义	  ：		走指定角度的直线闭环
   函数参数	  ：		lineAngle     指定的角度（以车的视角为标准）
                  speed         小车的速度
   函数返回值  ：	  无
   =======================================================================================*/
void ClLineAngle(float lineAngle, int speed)
{
	if (speed <= 500)
	{
		angleP    = 80;
		angleD    = 0;
		distantP  = 5;
	}
	else if (speed > 500 && speed < 1000)
	{
		angleP    = 100;
		angleD    = 20;
		distantP  = 7;
	}
	else if (speed >= 1000 && speed <= 1200)
	{
		angleP    = 150;
		angleD    = 30;
		distantP  = 12;
	}
	else if (speed > 1200 && speed <= 1400)
	{
		angleP    = 200;
		angleD    = 30;
		distantP  = 14;
	}
	else if (speed > 1400 && speed < 2000)
	{
		angleP    = 250;
		angleD    = 30;
		distantP  = 15;
	}
	else
	{
		angleP    = 280;
		angleD    = 50;
		distantP  = 20;
	}
	VelCrl(CAN2, 1, (speed * COUNTS_PER_ROUND) / (WHEEL_DIAMETER * PI) + PidAngle(lineAngle, Position_t.angle));
	VelCrl(CAN2, 2, -(speed * COUNTS_PER_ROUND) / (WHEEL_DIAMETER * PI) + PidAngle(lineAngle, Position_t.angle));
	//查看PID调节量
	pid2 = PidAngle(lineAngle, Position_t.angle);
}
/*======================================================================================
   函数定义	  ：		正方向走指定的任意直线闭环
   函数参数	  ：    aimX          直线过的定点的X坐标
                  aimY          直线过的定点的Y坐标
                  lineAngle     指定的角度（以车的视角为标准）
                  speed         小车的速度
   函数返回值  ：	  无
   =======================================================================================*/
void ClLine(float aimX, float aimY, float lineAngle, int speed)
{
	static double distant = 0, k = 0, degree = 0, impulse = 0;

	if (speed <= 500)
	{
		angleP    = 80;
		angleD    = 0;
		distantP  = 5;
	}
	else if (speed > 500 && speed < 1000)
	{
		angleP    = 100;
		angleD    = 20;
		distantP  = 7;
	}
	else if (speed >= 1000 && speed <= 1200)
	{
		angleP    = 150;
		angleD    = 30;
		distantP  = 12;
	}
	else if (speed > 1200 && speed <= 1400)
	{
		angleP    = 200;
		angleD    = 30;
		distantP  = 14;
	}
	else if (speed > 1400 && speed < 2000)
	{
		angleP    = 250;
		angleD    = 30;
		distantP  = 15;
	}
	else if(speed >=2000)
	{
		angleP    = 350;
		angleD    = 0;
		distantP  = 15;		
	}

	if (fabs(lineAngle) <= 0.0001)
	{
		distant = aimX - Position_t.X;
	}
	else if (lineAngle >= 179.9 || lineAngle <= -179.9)
	{
		distant = Position_t.X - aimX;
	}
	else
	{
		degree  = ANGTORAD(lineAngle + 90);
		k       = tan(degree);
		distant = (k * Position_t.X - Position_t.Y - k * aimX + aimY) / (sqrt(1 + k * k));
		if (lineAngle < 0 && lineAngle >= -180)
			distant = -distant;
	}
	impulse = (speed * COUNTS_PER_ROUND) / (WHEEL_DIAMETER * PI);
	VelCrl(CAN2, 1, impulse + PidCoordinate(0, distant) + PidAngle(lineAngle, Position_t.angle));
	VelCrl(CAN2, 2, -impulse + PidCoordinate(0, distant) + PidAngle(lineAngle, Position_t.angle));
	pid1  = PidCoordinate(0, distant);                      //查看PID调节量
	pid2  = PidAngle(lineAngle, Position_t.angle);
}
/*======================================================================================
   函数定义	  ：		反方向走指定的任意直线闭环（倒着走）
   函数参数	  ：    aimX          直线过的定点的X坐标
                  aimY          直线过的定点的Y坐标
                  lineAngle     指定的角度（以车的视角为标准）（车头指向）
                  speed         小车的速度
   函数返回值  ：	  无
   =======================================================================================*/
void ClLine2(float aimX, float aimY, float lineAngle, int speed)
{
	static double distant = 0, k = 0, degree = 0, impulse = 0;

	if (speed <= 500)
	{
		angleP    = 80;
		angleD    = 0;
		distantP  = 5;
	}
	else if (speed > 500 && speed < 1000)
	{
		angleP    = 100;
		angleD    = 20;
		distantP  = 7;
	}
	else if (speed >= 1000 && speed <= 1200)
	{
		angleP    = 150;
		angleD    = 30;
		distantP  = 12;
	}
	else if (speed > 1200 && speed <= 1400)
	{
		angleP    = 200;
		angleD    = 30;
		distantP  = 14;
	}
	else if (speed > 1400 && speed < 2000)
	{
		angleP    = 250;
		angleD    = 30;
		distantP  = 15;
	}
	else
	{
		angleP    = 280;
		angleD    = 50;
		distantP  = 20;
	}
	if (fabs(lineAngle) <= 0.0001)
	{
		distant = aimX - Position_t.X;
	}
	else if (lineAngle >= 179.9 || lineAngle <= -179.9)
	{
		distant = Position_t.X - aimX;
	}
	else
	{
		degree  = ANGTORAD(lineAngle + 90);
		k       = tan(degree);
		distant = (k * Position_t.X - Position_t.Y - k * aimX + aimY) / (sqrt(1 + k * k));
		if (lineAngle < 0 && lineAngle >= -180)
			distant = -distant;
	}
	impulse = (speed * COUNTS_PER_ROUND) / (WHEEL_DIAMETER * PI);
	VelCrl(CAN2, 1, impulse - PidCoordinate(0, distant) + PidAngle(lineAngle, Position_t.angle));
	VelCrl(CAN2, 2, -impulse - PidCoordinate(0, distant) + PidAngle(lineAngle, Position_t.angle));
	pid1  = PidCoordinate(0, distant);                      //查看PID调节量
	pid2  = PidAngle(lineAngle, Position_t.angle);
}
/*======================================================================================
   函数定义	  ：		顺时针的正方形闭环
   函数参数	  ：    speed         小车的速度
                  lineLong      正方形的边长
                  beginX        正方形左下角的点的X坐标
                  beginY        正方形左下角的点的Y坐标
   函数返回值  ：	  无
   =======================================================================================*/
void ShunClSquare(int speed, float lineLong, float beginX, float beginY)
{
	if (line == 1)
		ClLine(beginX, 0, 0, speed);
	if (line == 1 && Position_t.Y > (beginY + lineLong - AD_HIGH_SP))
		line = 2;
	if (line == 2)
		ClLine(0, (beginY + lineLong), -90, speed);

	if (line == 2 && Position_t.X > (beginX + lineLong - AD_HIGH_SP))
		line = 3;
	if (line == 3)
		ClLine((beginX + lineLong), 0, 180, speed);

	if (line == 3 && Position_t.Y < (beginY + AD_HIGH_SP))
	{
		line = 4; yiquan = 1;
	}
	if (line == 4)
		ClLine(0, beginY, 90, speed);
}

/*======================================================================================
   函数定义	  ：		逆时针的正方形闭环
   函数参数	  ：    speed         小车的速度
                  lineLong      正方形的边长
                  beginX        正方形右下角的点的X坐标
                  beginY        正方形右下角的点的Y坐标
   函数返回值  ：	  无
   =======================================================================================*/
void NiClSquare(int speed, float lineLong, float beginX, float beginY)
{
	if (line == 1)
		ClLine(beginX, 0, 0, speed);
	if (line == 1 && Position_t.Y > (beginY + lineLong - AD_HIGH_SP))
		line = 2;
	if (line == 2)
		ClLine(0, (beginY + lineLong), 90, speed);

	if (line == 2 && Position_t.X < (beginX - lineLong + AD_HIGH_SP))
		line = 3;
	if (line == 3)
		ClLine((beginX - lineLong), 0, 180, speed);

	if (line == 3 && Position_t.X < (beginY + AD_HIGH_SP))
	{
		line = 4; yiquan = 1;
	}
	if (line == 4)
		ClLine(0, beginY, -90, speed);
}
/*======================================================================================
   函数定义	  ：    比较三个值的大小，取最大值，返回最大值是第几个数
   函数参数	  ：    number1        第一个数
                  number2        第二个数
                  number3        第三个数
   函数返回值  ：	  mas            第几个数最大
   =======================================================================================*/
int Mas(int number1, int number2, int number3)
{
	int mas;

//	if (number1 == 0 && number2 == 0 && number3 == 0)
//	{
//		mas = 0;
//	}
//	else
	{
		mas = (number1 > number2) ? number1 : number2;
		mas = (mas > number3) ? mas : number3;

		if (mas == number2)
			mas = 2;

		else if (mas == number1)
			mas = 1;
		else if (mas == number3)
			mas = 3;
	}
	return mas;
}
/*======================================================================================
   函数定义	  ：    比较四个值的大小，取最大值，返回最大值是第几个数
   函数参数	  ：    number1        第一个数
                  number2        第二个数
                  number3        第三个数
                  number4        第四个数
   函数返回值  ：	  mas            第几个数最大
   =======================================================================================*/
int Mas2(int number1, int number2, int number3, int number4)
{
	int mas;

//	if (number1 == 0 && number2 == 0 && number3 == 0 && number4)
//	{
//		mas = 0;
//	}
//	else
	{
		mas = (number1 > number2) ? number1 : number2;
		mas = (mas > number3) ? mas : number3;
		mas = (mas > number4) ? mas : number4;


		if (mas == number3)
			mas = 3;
		else if (mas == number2)
			mas = 2;
		else if (mas == number1)
			mas = 1;		
		else if (mas == number4)
			mas = 4;
	}
	return mas;
}
/*======================================================================================
   函数定义	  ：    取出最近点的角度和距离
   函数参数	  ：    a[20]          一组点的角度
                  b[20]          一组点的距离
                  sum            这组数据有对应的几个点
   函数返回值  ：	  极坐标结构体（有角度和距离）
   =======================================================================================*/
PolarCoo_t Closer_Point(int8_t a[20], uint8_t b[20], int sum)
{
	int         z, min, q;
	PolarCoo_t  closer;

	if (sum == 1)
	{
		q = 0;
	}
	else if (sum == 2)
	{
		q = (b[0] < b[1]) ? 0 : 1;
	}
	else
	{
		min = (b[0] < b[1]) ? b[0] : b[1];
		q   = (b[0] < b[1]) ? 0 : 1;
		for (z = 2; z < sum; z++)
		{
			q   = (min < b[z]) ? q : z;
			min = (min < b[z]) ? min : b[z];
		}
	}
	closer.ang  = a[q];
	closer.dis  = b[q];
	return closer;
}
/*======================================================================================
   函数定义	  ：    将场地划分成10*10的100个格子
   函数参数	  ：    X            点的X坐标
                  Y            点的Y坐标
   函数返回值  ：	  含有对应横竖的第几个格子的结构体
   =======================================================================================*/
 Coo_t Zoning(float X, float Y)
{
	Coo_t wirte;
	int   m = 0, o = 0;
  if(fabs(X)<1800)
	{
    if(X<=-1300)
		{
			m=1;
		}
		else if(X>-1300&&X<=-800)
		{
			m=2;
		}
		else if(X>-800&&X<=-300)
		{
			m=3;
		}
		else if(X>300&&X<=800)
		{
			m=6;
		}
		else if(X>800&&X<=1300)
		{
			m=7;
		}
		else if(X>1300&&X<1800)
		{
			m=8;
		}
		
		else
		{
			while ((X - (m+1) * 480) > -2400)
			  m++;
		}
		
	}
  else
	{
		if(X>=1800)
		{
			m=9;
		}
		if(X<=-1800)
		{
			m=0;
		}
	}
	wirte.hor = m;
	if(Y<600)
	{
		o=0;
	}
	else if(Y>600&&Y<=1100)
	{
		o=1;
	}
	else if(Y>1100&&Y<=1700)
	{
		o=2;
	}
	else if(Y>3100&&Y<=3600)
	{
		o=7;
	}
	else if(Y>3600&&Y<4200)
	{
		o=8;
	}
	else if(Y>=4200)
	{
		o=9;
	}
	else 
	{
		if(Y>=600&&Y<1700)
		{
			while ((Y - (o+1) * 400) > 160)
				o++;
		}
		else if(Y>=1700&&Y<3240)
		{
			while ((Y - (o+1) * 480) > 0)
				o++;
		}
		else if(Y>=3240&&Y<4200)
		{
			while ((Y - (o+1) * 480) > -120)
				o++;
		}	
	}


	wirte.ver = o;
	return wirte;
}
/*======================================================================================
   函数定义	  ：    摄像头第一圈找球，无球时车在不同区域要走的方向
   函数参数	  ：    无

   函数返回值  ：	  无
   =======================================================================================*/
void First_Scan(void)
{
	static int area;

	//划分区域
	if (g_plan == -1)
	{
		if (Position_t.X <= -275 && Position_t.Y <= 3100)
			area = 1;
		else if (Position_t.X >= -275 && Position_t.Y < 1700)
			area = 2;
		else if (Position_t.X > 275 && Position_t.Y >= 1700)
			area = 3;
		else
			area = 4;
	}
	if (g_plan == 1)
	{
		if (Position_t.X >= 275 && Position_t.Y < 3100)
			area = 1;
		else if (Position_t.X > -275 && Position_t.Y >= 3100)
			area = 2;
		else if (Position_t.X <= -275 && Position_t.Y > 1700)
			area = 3;
		else
			area = 4;
	}
	switch (area)
	{
		case 1:
		{
			ClLineAngle(0, cameraSpeed);
		} break;

		case 2:
		{
			ClLineAngle(90, cameraSpeed);
		} break;

		case 3:
		{
			ClLineAngle(180, cameraSpeed);
		} break;

		case 4:
		{
			ClLineAngle(-90, cameraSpeed);
		} break;

		default:
			break;
	}
	POS_NOTE USART_OUT(UART5,(u8*)"area%d\r\n",area);
}
/*======================================================================================
   函数定义	  ：    将场地分成内外，用于逃逸
   函数参数	  ：    无

   函数返回值  ：	  
   =======================================================================================*/

int In_Or_Out(void)
{
	int finish = 0;
  //USART_OUT(UART5,(u8*)"%d\t%d\r\n",(int)Position_t.X,(int)Position_t.Y);
	
	//一号区域
  if(Position_t.X > 1700 && Position_t.Y < 600)
	{
		if(Position_t.angle>-45&&Position_t.angle<135)
			finish=9;
		else
			finish = 1;
		
	}
	
	//二号区域
	else if(Position_t.X > 1700 && Position_t.Y > 4100)
	{
		if(Position_t.angle>-135&&Position_t.angle<45)
		  finish = 2;
		else
			finish=9;
	}
	
	//三号区域
	else if(Position_t.X < -1700 && Position_t.Y > 4100)
	{
		if(Position_t.angle>-45&&Position_t.angle<135 )
		finish = 3;
		else
		finish = 9;
	}
	
	//四号区域
	else if(Position_t.X < -1700 && Position_t.Y < 600)
	{
		if(Position_t.angle> -135&&Position_t.angle<45)
		finish = 9;
		else
		finish= 4;
	}
	else if (Position_t.X >= -875 && Position_t.X <= 275 && Position_t.Y >= 1100 && Position_t.Y < 1700)
	{
			finish = 5;
	}
	else if(Position_t.X > 275 && Position_t.X <= 875 && Position_t.Y <= 3035.35 && Position_t.Y >= 1100)
	{
		finish = 6;
	}
	else if(Position_t.X >= -275 && Position_t.X <= 875 && Position_t.Y > 3035.35 && Position_t.Y <= 3635.35)
	{
		finish = 7;
	}
	else if(Position_t.X >= -875 && Position_t.X < -275 && Position_t.Y >= 1700 && Position_t.Y <= 3635.35)
	{
		finish = 8;
	}
	else if(Position_t.X > 1700 && Position_t.Y < 4100 && Position_t.Y > 600)
	{
		if(Position_t.angle > -110 && Position_t.angle < -70)
		{
			finish = 11;
		}
		else
		{
			finish = 9;
		}
	}
	else if(Position_t.Y > 4100 && Position_t.X < 1700 && Position_t.X > -1700)
	{
		if(Position_t.angle > -20 && Position_t.angle < 20)
		{
			finish = 12;
		}
		else
		{
			finish = 9;
		}
	}
	else if(Position_t.X < -1700 && Position_t.Y < 4100 && Position_t.Y > 600)
	{
		if(Position_t.angle > 70 && Position_t.angle < 110)
		{
			finish = 13;
		}
		else
		{
			finish = 9;
		}
	}
	else if(Position_t.Y < 600 && Position_t.X > -1700 && Position_t.X < 1700)
	{
		if(Position_t.angle > 160 || Position_t.angle < -160)
		{
			finish = 14;
		}
		else
		{
			finish = 9;
		}
	}
	else
	{
			finish = 9;
	}
	return finish;
}

/*======================================================================================
   函数定义	  ：    比较三个数组谁含有的0多
   函数参数	  ：    a1[10]        第一个数组
										a2[10]        第二个数组
										a3[10]        第三个数组
   函数返回值  ：	  c             含有最多0的是第几个数组
   =======================================================================================*/
int Least_H(int a1[10], int a2[10], int a3[10])
{
	int i, b1 = 0, b2 = 0, b3 = 0, c;

	for (i = 0; i < 10; i++)
	{
		if (!a1[i])
			b1++;
		if (!a2[i])
			b2++;
		if (!a3[i])
			b3++;
	}
	c = Mas(b1, b2, b3);
	return c;
}
/*======================================================================================
   函数定义	  ：    比较四个数组谁含有的0多
   函数参数	  ：    a1[10]        第一个数组
										a2[10]        第二个数组
										a3[10]        第三个数组
										a4[10]        第四个数组
   函数返回值  ：	  c             含有最多0的是第几个数组
   =======================================================================================*/
int Least_S(int a1[10], int a2[10], int a3[10], int a4[10])
{
	int i, b1 = 0, b2 = 0, b3 = 0, b4 = 0, c;

	for (i = 0; i < 10; i++)
	{
		if (!a1[i])
			b1++;
		if (!a2[i])
			b2++;
		if (!a3[i])
			b3++;
		if (!a4[i])
			b4++;
	}
	c = Mas2(b1, b2, b3, b4);
	return c;
}
/*======================================================================================
   函数定义	  ：    更新路线，走之前没怎么走过的(顺时针)
   函数参数	  ：    down          正方形下面三条线中哪条是要走的
                    right         正方形右面四条线中哪条是要走的
                    up            正方形上面三条线中哪条是要走的
                    left          正方形左面四条线中哪条是要走的
   函数返回值  ：	  无
   =======================================================================================*/
void New_Route(int dow, int righ, int u, int lef)
{
	static int side = 1,aim=0,notyet=0,area;
	if(g_plan==1)
	{
		if (Position_t.Y < 1700 )
		{
			if(righ!=4)
			{
				if(Position_t.X > (230 + righ * 480 - AD_CAMERA))
				{
					side = 2;notyet=0;					
					aim=230 + righ * 480;
				}
				else 
				{
					if(Position_t.X<=300)
					{
						area=1;notyet=1;
					}
				}
			}
			else
			{
				if(Position_t.X > (80 + righ * 480 - AD_CAMERA))
				{
					side = 2;notyet=0;
					aim=80 + righ * 480;
				}
				else
				{
					if(Position_t.X<=300)
					{
						area=1;notyet=1;
					}
				}
			}
		}
			
		if (Position_t.X > 300 )
		{
			if(u!=3)
			{
				if(Position_t.Y > (3000 + u * 480 - AD_CAMERA))
				{
					side = 3;notyet=0;
					aim=3000 + u * 480;
				}
				else 
				{
					if(Position_t.Y<=3100)
					{
						area=2;notyet=1;
					}
				}
			}
			else 
			{
				if(Position_t.Y > (2900 + u * 480 - AD_CAMERA))
				{
					side = 3;notyet=0;
					aim=2900 + u * 480;
				}
				else 
				{
					if(Position_t.Y<=3100)
					{
						area=2;notyet=1;
					}
				}
			}
		}
			
		if (Position_t.Y > 3100 )
		{
			if(lef!=1)
			{
				if(Position_t.X < (-2640 + lef * 480 + AD_CAMERA))
				{
					side = 4;notyet=0;
					aim=-2640 + lef * 480;
				}
				else 
				{
					if(Position_t.X>=-300)
					{
						area=3;notyet=1;
					}
				}
			}
			else 
			{
				if(Position_t.X < (-2400 + lef * 480 + AD_CAMERA))
				{
					side = 4;notyet=0;
					aim=-2400 + lef * 480;
				}
				else 
				{
					if(Position_t.X>=-300)
					{
						area=3;notyet=1;
					}
				}				
			}
		}
			
		if (Position_t.X < -300 )
		{
			if(dow==3)
			{
				if(Position_t.Y < (1400 + AD_CAMERA))
				{
					side = 1;notyet=0;
					aim=1400;
				}
				else
				{
					if(Position_t.Y>=1700)
					{
						area=4;notyet=1;
					}
				}				
			}
			else if(dow!=1)
			{
				if(Position_t.Y < (-80 + dow * 480 + AD_CAMERA))
				{
					side = 1;notyet=0;
					aim=-80 + dow * 480;
				}
				else
				{
					if(Position_t.Y>=1700)
					{
						area=4;notyet=1;
					}
				}
			}
			else 
			{
				if(Position_t.Y < ( dow * 480 + AD_CAMERA))
				{
					side = 1;notyet=0;
					aim=dow * 480;
				}
				else
				{
					if(Position_t.Y>=1700)
					{
						area=4;notyet=1;
					}
				}
			}
		}
			
	}
//  if(g_plan==-1)
//	{
//		if (Position_t.Y > 3100 && Position_t.X > (220 + righ * 480 - AD_CAMERA))
//			side = 2;
//		if (Position_t.X < -300 && Position_t.Y > (3100 + u * 480 - AD_CAMERA))
//			side = 3;
//		if (Position_t.Y < 1700 && Position_t.X < (-2520 + lef * 480 + AD_CAMERA))
//			side = 4;
//		if (Position_t.X > 300 && Position_t.Y < (-220 + dow * 480 + AD_CAMERA))
//			side = 1;
//	}
  if(notyet==0)
  {
		switch (side)
		{
			case 1:
			{
				ClLine(0, aim, -90, cameraSpeed);
			} break;

			case 2:
			{
				ClLine(aim, 0, 0, cameraSpeed);
			} break;

			case 3:
			{
				ClLine(0, aim, 90, cameraSpeed);
			} break;

			case 4:
			{
				ClLine(aim, 0, 180, cameraSpeed);
			} break;

			default:
				break;
		}
  }
  else
	{
		switch (area)
		{
			case 1:
			{
				ClLineAngle(-90, cameraSpeed);
			} break;

			case 2:
			{
				ClLineAngle(0, cameraSpeed);
			} break;

			case 3:
			{
				ClLineAngle(90, cameraSpeed);
			} break;

			case 4:
			{
				ClLineAngle(180, cameraSpeed);
			} break;

			default:
				break;
		}
	}
	
	POS_NOTE USART_OUT(UART5,(u8*)"side %d aim %d d%d r%d u%d l%d\r\n",side,aim,dow,righ,u,lef);
}
/*======================================================================================
   函数定义	  ：    扫四条边缘(一圈)
   函数参数	  ：    无

   函数返回值  ：	  1:              已完成
                  0:              未完成
   =======================================================================================*/
int RunEdge(void)
{
	int         finish = 0;
	static int  side = 0,sideTimes=0,avoidBack=0,edgearea=0;
	g_plan = 1;
		if (Position_t.X >= 950&&Position_t.Y<3300)
		{
			edgearea = 2;
		}
		else if (Position_t.Y >= 3300&&Position_t.X>-950)
		{
			edgearea = 3;
		}
		else if (Position_t.X <= -950&&Position_t.Y>1450)
		{
			edgearea = 4;
		}
		else if (Position_t.Y <= 1450&&Position_t.X<950)
		{
			edgearea = 1;
		}	
	  else if(Position_t.Y <= (2400 - Position_t.X) && Position_t.Y < (2400 + Position_t.X))
		{
			edgearea = 11;
		}
		else if(Position_t.Y > (2400 - Position_t.X) && Position_t.Y <= (2400 + Position_t.X))
		{
			edgearea = 12;
		}
		else if(Position_t.Y >= (2400 - Position_t.X) && Position_t.Y > (2400 + Position_t.X))
		{
			edgearea = 13;
		}
		else if(Position_t.Y < (2400 - Position_t.X) && Position_t.Y >= (2400 + Position_t.X))
		{
			edgearea = 14;
		}	
		else 
		{
			USART_OUT(UART5,(u8*)"edgearea erroe\r\n");
		}
  if(!ifEscape)
	{
		if (edgearea==2||edgearea==12)
		{
			if(side!=2)
			{
				sideTimes++;
			}
			side = 2;
		}
		else if (edgearea==3||edgearea==13)
		{
			if(side!=3)
			{
				sideTimes++;
			}
			side = 3;
		}
		else if (edgearea==4||edgearea==14)
		{
			if(side!=4)
			{
				sideTimes++;
			}
			side = 4;
		}
		else if (edgearea==1||edgearea==11)
		{
			if(side!=1)
			{
				sideTimes++;
			}			
			side = 1;
		}
		if(sideTimes>=6)
		{
			side = 0;
			sideTimes = 0;
			avoidBack=0;
			edgearea=1;
			finish = 1;
		}
	}	

//  if(sideTimes==1&&avoidBack==0)
//	{
//		if(g_plan==1)
//		{
//			switch(side)
//			{
//				case 1:
//				{
//					ClLineAngle(-90,1000);
//					if(Position_t.angle>-95&&Position_t.angle<-85)
//					{
//						avoidBack=1;
//					}
//				}break;
//				case 2:
//				{
//					ClLineAngle(0,1000);
//					if(fabs(Position_t.angle)<5)
//						avoidBack=1;
//					}//					{

//				}break;
//				case 3:
//				{
//					ClLineAngle(90,1000);
//					if(Position_t.angle>85&&Position_t.angle<95)
//					{
//						avoidBack=1;
//					}
//				}break;
//				case 4:
//				{
//					ClLineAngle(180,1000);
//					if(fabs(Position_t.angle)>175)
//					{
//						avoidBack=1;
//					}
//				}break;
//				default:
//					break;
//			}	
// 
//		}
//    else
//		{
//			switch(side)
//			{
//				case 1:
//				{
//					TurnAngle(90,1000);
//					if(Position_t.angle<95&&Position_t.angle>85)
//					{
//						avoidBack=1;
//					}
//				}break;
//				case 2:
//				{
//					TurnAngle(0,1000);
//					if(fabs(Position_t.angle)<5)
//					{
//						avoidBack=1;
//					}
//				}break;
//				case 3:
//				{
//					TurnAngle(-90,1000);
//					if(Position_t.angle<-85&&Position_t.angle>-95)
//					{
//						avoidBack=1;
//					}
//				}break;
//				case 4:
//				{
//					TurnAngle(180,1000);
//					if(fabs(Position_t.angle)>175)
//					{
//						avoidBack=1;
//					}
//				}break;
//				default:
//					break;
//			}				
//		}				
//	}
//	if(avoidBack)
	{
		if(g_plan==1)
		{
			switch (side)
			{
				case 1:
				{
					ClLine(0, 245-POSYSTEM_TO_BACK, -90, 1500);
				} break;

				case 2:
				{
					ClLine(2155, 0, 0, 1500);
				} break;

				case 3:
				{
					ClLine(0, 4555-POSYSTEM_TO_BACK, 90, 1500);
				} break;

				case 4:
				{
					ClLine(-2150, 0, 180, 1500);
				} break;

				default:
					break;
			}			
		}
		else
		{
			switch (side)
			{
				case 1:
				{
					ClLine(2155, 0, -4, 1400);
				} break;

				case 2:
				{
					ClLine(0, 4555-POSYSTEM_TO_BACK, 86, 1400);
				} break;

				case 3:
				{
					ClLine(-2355, 0, 176, 1400);
				} break;

				case 4:
				{
					ClLine(0, 100-POSYSTEM_TO_BACK, -94, 1400);
				} break;

				default:
					break;
			}			
		}

	}

	USART_OUT(UART5,(u8*)"edge  %d\t%d\t%d\r\n",side,sideTimes,avoidBack);
	return finish;
}



/*======================================================================================
   函数定义	  ：    计算判断球是否在车身范围内，就可以直接直走
   函数参数	  ：    di      球离摄像头的距离
                  an      球与中线的夹角

   函数返回值  ：	  1:      在车宽内
                  0:      不在车宽内
   =======================================================================================*/
int Vehicle_Width(int di, int an)
{
	float gap;

	gap = di * sin(ANGTORAD(an));
	gap = fabs(gap);
	if (gap < 200)
		return 1;
	else
		return 0;
}
/*======================================================================================
   函数定义	  ：    扫描轨迹是否全已走完，走完全部清零 
   函数参数	  ：    a[10][10]     轨迹二维数组

   函数返回值  ：	  1:            已经都走过了
                  0:            还没走完
   =======================================================================================*/
int ScanTrace(int a[10][10])
{
	int m, n, yes = 0;

	for (m = 0; m < 10; m++)
	{
		for (n = 0; n < 10; n++)
			if (!a[m][n])
				yes++;

	}
	if (yes == 0)
	{
		for (m = 0; m < 10; m++)
		{
			for (n = 0; n < 10; n++)
				a[m][n] = 0;
		}
		return 1;
	}
	else
	{
		return 0;
	}
}
/*======================================================================================
   函数定义	  ：    将摄像头看到的区域划分成四个同角度的扇形
   函数参数	  ：    无

   函数返回值  ：	  numbers.one            从左到右第一个扇形内球数
                  numbers.two            从左到右第二个扇形内球数
                  numbers.there          从左到右第三个扇形内球数
                  numbers.four           从左到右第四个扇形内球数
   =======================================================================================*/
Four_t Apart(void)
{
	int     k;
	Four_t  numbers;

	for (k = 0; k < arr_number; k++)
	{
		if (arr1[k] >= -25 && arr1[k] < -7.5)
			numbers.one++;
		if (arr1[k] >= -15 && arr1[k] < 2.5)
			numbers.two++;
		if (arr1[k] >= -2.5 && arr1[k] < 15)
			numbers.there++;
		if (arr1[k] >= 7.5 && arr1[k] < 25)
			numbers.four++;
	}
	return numbers;
}

/*======================================================================================
   函数定义	  ：    交换顺序
   函数参数	  ：


   函数返回值  ：	  无
   =======================================================================================*/
void ChangeOrder1(int8_t a, int8_t b)
{
	int8_t c;

	c = a;
	a = b;
	b = c;
}
void ChangeOrder2(uint8_t a, uint8_t b)
{
	uint8_t c;

	c = a;
	a = b;
	b = c;
}
/*======================================================================================
   函数定义	  ：    将球的坐标按照从左到右排序
   函数参数	  ：


   函数返回值  ：	  无
   =======================================================================================*/
void Left2Right(void)
{
	int c1, c2;

	for (c1 = 0; c1 < (arr_number - 1); c1++)
	{
		for (c2 = 0; c2 < (arr_number - 1); c2++)
		{
			if (arr1[c2] < arr1[(c2 + 1)])
			{
				ChangeOrder1(arr1[c2], arr1[(c2 + 1)]);
				ChangeOrder2(arr2[c2], arr2[(c2 + 1)]);
			}
		}
	}
}
/*======================================================================================
   函数定义	  ：    将球的坐标按照从下到上排序
   函数参数	  ：


   函数返回值  ：	  无
   =======================================================================================*/
void Down2Up(void)
{
	int c1, c2;

	for (c1 = 0; c1 < (arr_number - 3); c1++)
	{
		for (c2 = 1; c2 < (arr_number - 2); c2++)
		{
			if (arr2[c2] > arr2[(c2 + 1)])
			{
				ChangeOrder1(arr1[c2], arr1[(c2 + 1)]);
				ChangeOrder2(arr2[c2], arr2[(c2 + 1)]);
			}
		}
	}
}
/*======================================================================================
   函数定义	  ：    取较前方的平均角度
   函数参数	  ：


   函数返回值  ：	  最优角度
   =======================================================================================*/
float MostSector(void)
{
	float best, nu = 0, total = 0;
	int   k;

	for (k = 0; k < arr_number; k++)
	{
		if (arr2[k] > 70)
		{
			nu++;
			total = total + arr1[k];
		}
	}
	if (total == 0)
	{
		for (k = 0; k < arr_number; k++)
			total = total + arr1[k];
		best = total / arr_number;
	}
	else
	{
		best = total / nu;
	}
	return best;
}
/*======================================================================================
   函数定义	  ：    两点的距离
   函数参数	  ：


   函数返回值  ：	  两点的距离
   =======================================================================================*/
float P2P(float a1, float a2, float b1, float b2)
{
	float dist;

	dist = sqrt(PF(a1 - b1) + PF(a2 - b2));
	return dist;
}

/*======================================================================================
   函数定义	  ：    走定点
   函数参数	  ：


   函数返回值  ：	  无
   =======================================================================================*/
void GivenPoint(float pointX, float pointY, float givenSpeed)
{
	float o_distant, p_distant, shadow, percent, k, kangle, carangle;

	o_distant = sqrt(pointX * pointX + pointY * pointY);

	shadow  = (pointX * Position_t.X + pointY * Position_t.Y) / o_distant;
	percent = 1 - (shadow / o_distant);
	if (percent >= 0.5 && percent <= 1)
		percent = 1 - sqrt(percent - percent * percent);
	if (percent < 0.5 && percent >= 0)
		percent = sqrt(percent - percent * percent);
	if (pointX == 0)
	{
		if (pointY >= 0)
			ClLine(0, 0, 0, givenSpeed * percent);
		if (pointY < 0)
			ClLine(0, 0, 180, givenSpeed * percent);
	}
	else
	{
		k         = pointY / pointX;
		kangle    = atan2(pointX, pointY);
		carangle  = (kangle * 180) / 3.14 - 90;
		p_distant = (k * Position_t.X - Position_t.Y) / (sqrt(1 + k * k));
		if (p_distant > -50 && p_distant < 50)
			ClLine(0, 0, carangle, givenSpeed * percent);
		else
			ClLine(0, 0, carangle, givenSpeed);
	}
}
/*======================================================================================
   函数定义	  ：    规划最优路线
   函数参数	  ：


   函数返回值  ：	  无
   =======================================================================================*/
float bestTraX[20], bestTraY[20];
int   bestSum=20;

int ballNumber;
/*======================================================================================
函数定义	  ：
函数参数	  ：    
                  
                           
函数返回值  ：	  无
=======================================================================================*/
extern int notcount;
void CountBall(void)
{
	static int ballN=0,sum=0,beginSum=0,pass=0,kunziBall=0;
	ReadActualVel(CAN1, COLLECT_BALL_ID);
	if(pass==2)
	{
		if(g_gather>240)
		{
			pass=0;
			sum=0;
		}
		else 
		{
			pass=1;
		}
	}	
  if(g_gather<=235&&g_gather>100&&sum==0)
	{
		beginSum = 1;
		pass=2;
	}

	if(g_gather>=250)
	{
		beginSum = 0;
		if(sum>=80&&sum<=230)
		{
			ballN=1;
		}
		else if(sum>230&&sum<=1200)
		{
			ballN=2;
		}
		else if(sum>1200&&sum<=2000)
		{
			ballN=3;
		}
		else if(sum>2000)
		{
			ballN=4;
		}
		else 
		{
			ballN=0; 
			sum=0;
		}
	}
	if(beginSum&&pass)
	{
		sum += (250-g_gather);
	}
	if(finishShoot ==1)//射球完毕的标志
	{
		finishShoot=0;
		ballNumber=0;
		kunziBall=0;
		ballN=0;
	}

	if(notcount==0)
	{
		if(ballN)
		{
			kunziBall +=ballN;
			//加上光电
			ballNumber =(kunziBall + photoElectricityCount) / 2;
			ballN=0;
			sum=0;
		}		
	}

	
	//给数球一个上限
	if(ballNumber>50)
	{
		ballNumber=50;
	}
		
	USART_OUT(UART5,(u8*)" \t \t \t \t \t%d\t%d\t%d\t%d\r\n",g_gather,ballNumber,sum,(int)photoElectricityCount);

}
/*======================================================================================
函数定义	  ：
函数参数	  ：    
                  
                           
函数返回值  ：	  无
=======================================================================================*/
int SweepIn(void)
{
	static int inarea1=0,inarea2=0,squareNumber=1,check1=0,check2=0,walkTime=0,areaTime=0;
	int success=0;
	if(squareNumber==1)
	{
		if(Position_t.X>=-50&&Position_t.Y<2335.35)
		{
			if(check1!=1)
			{
				areaTime++;
			}		
      inarea1=1;check1=1;			
		}
		else if(Position_t.X>0&&Position_t.Y>=2335.35)
		{
			if(check1!=2)
			{
				areaTime++;
			}		
			inarea1=2;check1=2;
		}
		else if(Position_t.X<=0&&Position_t.Y>2035.35)
		{
			if(check1!=3)
			{
				areaTime++;
			}
			inarea1=3;check1=3;
		}
		else if(Position_t.X<-50&&Position_t.Y<=2035.35)
		{
			if(check1!=4)
			{
				areaTime++;
			}
			inarea1=4;check1=4;
		}
		else
		{
      success=1;
//			USART_OUT(UART5,(u8*)"sweepin_area_error\r\n");
		}		
	}
	
	if(squareNumber==1)
	{
		switch(inarea1)
		{
			case 1:
			{
				StaightCLose(800,0,0,1500);
			}break;
			case 2:
			{
				StaightCLose(0,3535.35,90,1500);
			}break;
			case 3:
			{
				StaightCLose(-800,0,180,1500);
			}break;
			case 4:
			{
				StaightCLose(0,1135.35,-90,1500);
			}break;
			default:
			 break;
		}		
	}
	if(areaTime>=5&&squareNumber==1)
	{
		areaTime=0;
		squareNumber=3;
	}
//	
//	if(squareNumber==2)
//	{
//		if(Position_t.X>=0&&Position_t.Y<2335.35)
//		{
//			if(check2!=1)
//			{
//			  areaTime++;
//			}
//			inarea2=1;check2=1;
//		}
//		else if(Position_t.X>0&&Position_t.Y>=2335.35)
//		{			
//			if(check2!=2)
//			{
//				areaTime++;
//			}
//			inarea2=2;check2=2;
//		}
//		else if(Position_t.X<=0&&Position_t.Y>2335.35)
//		{			
//			if(check2!=3)
//			{
//			  areaTime++;
//			}
//			inarea2=3;check2=3;
//		}
//		else if(Position_t.X<0&&Position_t.Y<=2335.35)
//		{			
//			if(check2!=4)
//			{
//				areaTime++;
//			}
//			inarea2=4;check2=4;
//		}
//		else
//		{
//      success=1;
////			USART_OUT(UART5,(u8*)"sweepin_area_error\r\n");
//		}			
//	}
//	if(squareNumber==2)
//	{
//		switch(inarea2)
//		{
//			case 1:
//			{
//				StaightCLose(600,0,0,1000);
//			}break;
//			case 2:
//			{
//				StaightCLose(0,3400.35,90,1000);
//			}break;
//			case 3:
//			{
//				StaightCLose(-600,0,180,1000);
//			}break;
//			case 4:
//			{
//				StaightCLose(0,1255.35,-90,1000);
//			}break;
//			default:
//			 break;			
//		}
//	}
//	if(areaTime>=5&&squareNumber==2)
//	{
//		areaTime=0;
//		squareNumber=3;
//	}
	if(squareNumber==3)
	{
		walkTime++;
		VelCrl(CAN2, 1, 15000);
	  VelCrl(CAN2, 2, -15000);		
		if(walkTime>100)
		{
			success=1;
			walkTime=0;
			squareNumber=0;
			inarea1=0;
      inarea2=0;
      check1=0;
      check2=0;
		}
	}
	USART_OUT(UART5,(u8*)"%d\t%d\t%d\r\n",squareNumber,areaTime,check1);
	return success;
}
