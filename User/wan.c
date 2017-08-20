#include "wan.h"
#include "elmo.h"
#include "lyz.h"
#include "math.h"
#include "moveBase.h"
#include "usart.h"
/*==============================================全局变量声明区============================================*/

extern POSITION_T Position_t;
extern int8_t g_cameraFin;
extern int8_t g_cameraNum;
extern int8_t g_cameraAng[50];
extern uint8_t g_cameraDis[50];
static int8_t leftAng[50]={0};
static int8_t midAng[50]={0};
static int8_t rightAng[50]={0};
static float aangle=0;
/*======================================================================================
函数定义	  ：		Send Get函数
函数参数	  ：		
函数返回值  ：		
=======================================================================================*/
void SendAng(float ang)
{
	aangle=ang;
}
float GetAng(void)
{
	return aangle;
}
/*======================================================================================
函数定义	  ：		避免角度溢出
函数参数	  ：		当前角度
函数返回值    ：		修正后的角度
=======================================================================================*/
float AvoidOverAngle(float angle)
{
	if(angle<=-180)
	{
		angle+=360;
	}
	if(angle>180)
	{
		angle-=360;
	}
	return angle;
}
/*======================================================================================
函数定义	  ：		角度闭环
函数参数	  ：		V          小车速度
                  aimAngle   目标角度
                  Kp         P参数
函数返回值  ：	  无
=======================================================================================*/
void angClose(float V,float aimAngle,float Kp)
{
	float angError,angOutput;
	angError=aimAngle-Position_t.angle;
	angError=AvoidOverAngle(angError);
	angOutput=angError*Kp;
	VelCrl(CAN1, 1, V*SP2PULSE+angOutput);
	VelCrl(CAN1, 2, -V*SP2PULSE+angOutput);
}
/*======================================================================================
函数定义	  ：		将小球相对于摄像头的角度转换成相对于陀螺仪的角度
函数参数	  ：		diatance     小球距离摄像头的距离
                  angle        小球相对于摄像头的角度
函数返回值  ：	  aimAngle     小球相对于陀螺仪的角度(单位：度)
=======================================================================================*/
float AngCamera2Gyro(float distance,float angle)
{
	float ThirdSide=0,rad=0,aimAngle=0;
	rad=ANGTORAD(180-angle);
	
	//余弦定理求第三边
	ThirdSide=sqrt(CAMERATOGYRO*CAMERATOGYRO+distance*distance-2*distance*CAMERATOGYRO*cos(rad));
	
	//正弦定理求目标角度(弧度)
	aimAngle=asin(distance*sin(rad)/ThirdSide);
  return RADTOANG(aimAngle);
}
/*======================================================================================
函数定义	  ：		将小球相对于摄像头的距离转换成相对于陀螺仪的距离
函数参数	  ：		diatance     小球相对于摄像头的距离
                  angle        小球相对于摄像头的角度
函数返回值  ：	  ThirdSide    小球相对于陀螺仪的距离(mm)
=======================================================================================*/
float DisBall2Gyro(float distance,float angle)
{
	float ThirdSide=0,rad=0;
	rad=ANGTORAD(180-angle);
	
	//余弦定理求第三边
  ThirdSide=sqrt(CAMERATOGYRO*CAMERATOGYRO+distance*distance-2*distance*CAMERATOGYRO*cos(rad));
	
	//单位换算，cm->mm
	ThirdSide=10*ThirdSide;
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
  BALLNUM_T ballNum={0,0,0};
	int j=0;
	float verDis=0;
	
	//g_cameraFin为1，说明接收数据完成，开始处理
	if(g_cameraFin)
	{
		//消除(摄像头大约60ms更新一次数据，而主函数10ms一个周期，故摄像头更新数据前大约会返回5-7个ballNum的初始化值)
		//BALLNUM_T ballNum={0,0,0};
		for(j=0;j<g_cameraNum;j++)
		{
			//将距离单位cm转换成mm(g_cameraDis[j]*10)
			//将g_cameraAng[j]转换成弧度
			verDis=g_cameraDis[j]*10*sin(g_cameraAng[j]*PI/180);
			USART_OUT(USART1,(u8*)"verDis%d\r\n",(int)verDis);
			
			//判定左边有球
			if(verDis>(WIDTH/2-ADJUSTDIS))
			{
				leftAng[ballNum.leftNum]=g_cameraAng[j];
				ballNum.leftNum++;
			}
			
			//判定右边有球
			else if(verDis<(-WIDTH/2+ADJUSTDIS))
			{
				rightAng[ballNum.rightNum]=g_cameraAng[j];
				ballNum.rightNum++;
			}
			
			//判定中间有球
			else
			{
				midAng[ballNum.midNum]=g_cameraAng[j];
				ballNum.midNum++;
			}
		}
		
		//将摄像头发完数据的瞬间角度发送出去，用于调整接下来60ms车的角度
		SendAng(Position_t.angle);
		
		//清零
		g_cameraNum=0;
		g_cameraFin=0;
	}
	
	USART_OUT(USART1,(u8*)"left%d\tmid%d\tright%d\r\n",ballNum.leftNum,ballNum.midNum,ballNum.rightNum);
	return ballNum;
}
/*======================================================================================
函数定义	  ：		在球最多的区域收集球(精细化调整)
函数参数	  ：		无
             
函数返回值  ：	  
=======================================================================================*/
void CollectMostBall(void)
{
	float aveAngle=0,sumAngle=0,nowAngle=0,aimAngle=0;
	int i=0;
	BALLNUM_T num={0,0,0};
  num=SeekMostBall();
	
	//走中间区域
	if(num.midNum>=num.leftNum&&num.midNum>=num.rightNum)
	{
		for(i=0;i<num.midNum;i++)
		{
			sumAngle+=midAng[i];
		}
		
		//注意midNum可能为0，取平均值
		if(num.midNum!=0)
		{
			aveAngle=sumAngle/num.midNum;
		}
		if(num.midNum==0)
		{
			aveAngle=0;
		}
	}
	
	//走右边区域
	else if(num.rightNum>num.midNum&&num.rightNum>=num.leftNum)
	{
		for(i=0;i<num.rightNum;i++)
		{
			sumAngle+=rightAng[i];
		}
		aveAngle=sumAngle/num.rightNum;
	}
	
	//走左边区域
	else
	{
		for(i=0;i<num.leftNum;i++)
		{
			sumAngle+=leftAng[i];
		}
		aveAngle=sumAngle/num.leftNum;
	}
	nowAngle=GetAng();
	aimAngle=nowAngle+aveAngle;
	aimAngle=AvoidOverAngle(aimAngle);
	USART_OUT(USART1,(u8*)"left%d\tmid%d\tright%d\taveAngle%d\tnowAngle%d\r\n",num.leftNum,num.midNum,num.rightNum,aveAngle,nowAngle);
	angClose(500,aimAngle,100);
}
/*======================================================================================
函数定义	  ：		在球最多的区域收集球(粗略调整角度方案)
函数参数	  ：		无
             
函数返回值  ：	  
=======================================================================================*/
void CollecMostBall(void)
{
	float angleAdjust=0,nowAngle=0,aimAngle=0;
	BALLNUM_T num={0,0,0};
	num=SeekMostBall();
	
	//走中间区域
	if(num.midNum>=num.leftNum&&num.midNum>=num.rightNum)
		angleAdjust=0;
	
	//走右边区域
	else if(num.rightNum>num.midNum&&num.rightNum>=num.leftNum)
		angleAdjust=-12.5;
	
	//走左边区域
	else
		angleAdjust=12.5;
	nowAngle=GetAng();
	aimAngle=nowAngle+angleAdjust;
	USART_OUT(USART1,(u8*)"left%d\tmid%d\tright%d\tangleAdjust%d\tnowAngle%d\r\n",num.leftNum,num.midNum,num.rightNum,angleAdjust,nowAngle);
	angClose(500,aimAngle,100);
}