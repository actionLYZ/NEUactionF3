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
extern int8_t g_cameraAng[10];
extern uint8_t g_cameraDis[10];
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
函数定义	  ：		计算左中右各区域小球的数量
函数参数	  ：		无
                  
函数返回值  ：	  Ballnum.leftNum   左区域的球数
                  Ballnum.midNum    中间区域的球数
                  Ballnum.rightNum  右区域的球数
=======================================================================================*/
BALLNUM_T SeekMaxBall(void)
{
  BALLNUM_T ballNum={0,0,0};
	int j=0;
	float verDis=0;
	
	//g_cameraFin为1，说明接收数据完成，开始处理
	if(g_cameraFin)
	{
		for(j=0;j<g_cameraNum;j++)
		{
			//将距离单位cm转换成mm(g_cameraDis[j]*10)
			//不要忘记将g_cameraAng[j]转换成弧度
			verDis=g_cameraDis[j]*10*sin(g_cameraAng[j]*PI/180);
			USART_OUT(USART1,(u8*)"verDis%d\r\n",(int)verDis);
			
			//判定左边有球
			if(verDis>WIDTH/2)
				ballNum.leftNum++;
			
			//判定右边有球
			else if(verDis<-WIDTH/2)
				ballNum.rightNum++;
			
			//判定中间有球
			else
				ballNum.midNum++;
		}
		
		//清零
		g_cameraNum=0;
		g_cameraFin=0;
	}
	
	//摄像头大约60ms更新一次数据，而主函数10ms一个周期，故摄像头更新数据前大约会返回5-7个ballNum的初始化值
	return ballNum;
//	if(Ballnum.midNum>=Ballnum.rightNum&&Ballnum.midNum>=Ballnum.leftNum)
//		return 0;
//	else if(Ballnum.rightNum>Ballnum.leftNum&&Ballnum.rightNum>Ballnum.midNum)
//		return -12.5;
//	else 
//		return 12.5;
}
