#include "wan.h"
#include "elmo.h"
#include "lyz.h"
#include "math.h"
/*==============================================全局变量声明区============================================*/
extern POSITION_T Position_t;

/*======================================================================================
函数定义	  ：		避免角度溢出
函数参数	  ：		当前角度
函数返回值  ：		修正后的角度
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
函数返回值  ：	  小球相对于陀螺仪的角度
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