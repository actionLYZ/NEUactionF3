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
static uint8_t leftDis[50]={0};
static uint8_t midDis[50]={0};
static uint8_t rightDis[50]={0};
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
函数参数	  ：		diatance     小球距离摄像头的距离(mm)
                        angle        小球相对于摄像头的角度
函数返回值    ：	    aimAngle     小球相对于陀螺仪的角度(单位：度)
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
函数定义	  ：		计算左中右各区域小球的数量(莫名的不好用了)
函数参数	  ：		无
备注        :     超出中间区域边界线(左8cm)(右10cm)左右，小车仍会认为小球还在中间区域             
函数返回值  ：	  Ballnum.leftNum   左区域的球数
                  Ballnum.midNum    中间区域的球数
                  Ballnum.rightNum  右区域的球数
=======================================================================================*/
BALLNUM_T SeekMostBall(void)
{
  static BALLNUM_T ballNum={0,0,0};
	int j=0;
	float verDis=0;
	
	//g_cameraFin为1，说明接收数据完成，开始处理
	if(g_cameraFin)
	{
	  ballNum.leftNum=0;
		ballNum.midNum=0;
		ballNum.rightNum=0;
		
		//消除(摄像头大约60ms更新一次数据，而主函数10ms一个周期，故摄像头更新数据前大约会返回5-7个ballNum的初始化值)
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
				leftDis[ballNum.leftNum]=g_cameraDis[j]*10;
				ballNum.leftNum++;
			}
			
			//判定右边有球
			else if(verDis<(-WIDTH/2+ADJUSTDIS))
			{
				rightAng[ballNum.rightNum]=g_cameraAng[j];
				rightDis[ballNum.rightNum]=g_cameraDis[j]*10;
				ballNum.rightNum++;
			}
			
			//判定中间有球
			else
			{
				midAng[ballNum.midNum]=g_cameraAng[j];
				midDis[ballNum.midNum]=g_cameraDis[j]*10;
				ballNum.midNum++;
			}
		}
		
		//将摄像头发完数据瞬间的角度发送出去，用于调整接下来60ms车的角度
		SendAng(Position_t.angle);
		
		//清零
		g_cameraNum=0;
		g_cameraFin=0;
	}
	
	USART_OUT(USART1,(u8*)"left%d\tmid%d\tright%d\r\n",ballNum.leftNum,ballNum.midNum,ballNum.rightNum);
	return ballNum;
}
/*======================================================================================
函数定义	  ：		求得距离数组中的最大值
函数参数	  ：		无
             
函数返回值  ：	  最大的距离值
=======================================================================================*/
float Max(uint8_t arr[50],int n)
{
	float maxDis=0;
	int i=0;
	maxDis=arr[0];
	if(n==1)
	{
		return maxDis;
	}
	else
	{
		for(i=1;i<n;i++)
		{
			maxDis=(maxDis>=arr[i]?maxDis:arr[i]);
		}
		return maxDis;
	}
}
/*======================================================================================
函数定义	  ：		在球最多的区域收集球(精细化调整)(未验证)
函数参数	  ：		无
             
函数返回值  ：	  
=======================================================================================*/
void CollectMostBall(void)
{
	float aveAngle=0,sumAngle=0,nowAngle=0,aimAngle=0,distance=0;
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
			
			//求出区域中小球的最长距离
			distance=Max(midDis,num.midNum);
			
			//将小球相对于摄像头的距离转换成相对于陀螺仪的距离
			aveAngle=DisBall2Gyro(distance,aveAngle);
		}
		if(num.midNum==0)
		{
			aveAngle=0;
			
			//随便设的
			distance=10;
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
		distance=Max(rightDis,num.rightNum);
		aveAngle=DisBall2Gyro(distance,aveAngle);
	}
	
	//走左边区域
	else
	{
		for(i=0;i<num.leftNum;i++)
		{
			sumAngle+=leftAng[i];
		}
		aveAngle=sumAngle/num.leftNum;
		distance=Max(leftDis,num.leftNum);
		aveAngle=DisBall2Gyro(distance,aveAngle);
	}
	nowAngle=GetAng();
	aimAngle=nowAngle+aveAngle;
	aimAngle=AvoidOverAngle(aimAngle);
	USART_OUT(USART1,(u8*)"left%d\tmid%d\tright%d\taveAngle%d\tnowAngle%d\r\n",num.leftNum,num.midNum,num.rightNum,aveAngle,nowAngle);
	angClose(500,aimAngle,100);
}
/*======================================================================================
函数定义	  ：		在球最多的区域收集球(粗略调整角度方案)(未验证)
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
	aimAngle=AvoidOverAngle(aimAngle);
	USART_OUT(USART1,(u8*)"left%d\tmid%d\tright%d\tangleAdjust%d\tnowAngle%d\r\n",num.leftNum,num.midNum,num.rightNum,angleAdjust,nowAngle);
	angClose(500,aimAngle,100);
}
/*======================================================================================
函数定义		：			利用摄像头收集球最多的区域的小球,基本走形回字形
函数参数		：			无
函数返回值	：			无
=======================================================================================*/
void RunWithCamera(void)
{
	int i=0;
	static int circle=0;
	switch(i)
	{
		//起步先转弯到-90°，然后i++;进入下一个状态
		case 0:
			angClose(500,-90,100);
			if(fabs(Position_t.angle+90)<5)
			{
				i++;
			}
			break;
			
			//在-90°的角度上收集球，当Position_t.X>1800时，转弯，然后下一个角度收集球。以后的步骤类同
		case 1:
			CollecMostBall();
			if(Position_t.X>(1800-circle*SPREAD_DIS))
			{
				angClose(500,0,100);
				if(fabs(Position_t.angle)<5)
				{
					i++;
				}
			}
			break;
		case 2:
			CollecMostBall();
			if(Position_t.Y>(3800-circle*SPREAD_DIS))
			{
				angClose(500,90,100);
				if(fabs(Position_t.angle-90)<5)
				{
					i++;
				}
			}
			break;
		case 3:
			CollecMostBall();
			if(Position_t.X<-(1800-circle*SPREAD_DIS))
			{
				angClose(500,180,100);
				if(fabs(Position_t.angle-180)<5)
				{
					i++;
				}
			}
			break;
		case 4:
			CollecMostBall();
			if(Position_t.Y<(600+circle*SPREAD_DIS))
			{
				angClose(500,-90,100);
				if(fabs(Position_t.angle-90)<5)
				{
					i=1;
					circle++;
				}
			}
			break;
	}
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
				StaightCLose(0,3100 + WIDTH/2 + 100,90,RUN_SPEED);
				if(Position_t.X <= -275 - WIDTH/2-SPREAD_DIS + ADV_TUEN)
					state = 3;
			}break;
			
			//左边，目标角度180度
			case 3:
			{
				StaightCLose((-275 - WIDTH/2 - SPREAD_DIS),0,180,RUN_SPEED);
				if(Position_t.Y <= 1700 - WIDTH/2-SPREAD_DIS + ADV_TUEN )
					state = 4;
			}break;
			
			//下边，目标角度-90度
			case 4:
			{
				StaightCLose(0,1700 - WIDTH/2 -SPREAD_DIS,-90,RUN_SPEED);
				if(Position_t.X >= 275 + WIDTH/2 +SPREAD_DIS- ADV_TUEN)
				{
				  return true;
				}
			}break;
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
bool	RunRectangleW(int length,int wide,float speed)
{
	
	static int state = 1;
	switch(state)
	{
		//长方形右边，目标角度0度
		case 1:
		{
			StaightCLose(275 + wide,0,0,speed);
			if(Position_t.Y >= 3100 + length - ADV_TUEN)
				state = 2;
		}break;
		
		//长方形上边，目标角度90度
		case 2:
		{
			StaightCLose(0,3100 + length,90,speed);
			if(wide	+ SPREAD_DIS	>= 2125 - WIDTH/2 - 100) 
				wide		= 2125 - WIDTH/2 - SPREAD_DIS - 100;
			if(Position_t.X <= -275 - wide-SPREAD_DIS + ADV_TUEN)
				state = 3;
		}break;
			
		//长方形左边，目标角度180度
		case 3:
		{
			StaightCLose(-275 - wide-SPREAD_DIS,0,180,speed);
			if(length + SPREAD_DIS >= 1700 - WIDTH/2 - 100)
				length = 1700 - WIDTH/2 - SPREAD_DIS - 100;
			if(Position_t.Y <= 1700 - length - SPREAD_DIS + ADV_TUEN)
				state = 4;		
		}break;
		
		//长方形下边，目标角度-90度
		case 4:
		{
			StaightCLose(0,1700 - length - SPREAD_DIS,-90,speed);
			if(Position_t.X >= 275 + wide + SPREAD_DIS - ADV_TUEN)
			{
				state = 1;
				return true;
			}
		}break;
	}
	return false;
}
/*======================================================================================
函数定义		：			计算右车头的坐标(顺时针跑场)
函数参数		：		    无

函数返回值	    ：	        右车头的坐标结构体
暂时未加入x的镜像对称
=======================================================================================*/
HEADPOS_T RightHeadPos(void)
{
	float angle=0;
	HEADPOS_T  position;
	
	//计算右车尖与陀螺仪连线在直角坐标系中与X轴的夹角
	angle=AvoidOverAngle(Position_t.angle-ANGRIGHTGYRO+90);
	angle=ANGTORAD(angle);
	position.X=Position_t.X+DISRIGHTGYRO*cos(angle);
	position.Y=Position_t.Y+DISRIGHTGYRO*sin(angle);  
	return position;
}
/*======================================================================================
函数定义		：			继续矫正函数
函数参数		：		    无

函数返回值	    ：	        
=======================================================================================*/
void ContinueCheck()
{
	
}
/*======================================================================================
函数定义		：			给投球电机发送速度（脉冲/s）
函数参数		：		    投球电机速度（脉冲/s）

函数返回值	    ：	        无
=======================================================================================*/

void SendUint8(int32_t pulse)
{
	//定义联合体
	num_t u_Num;
    u_Num.Int32 = pulse;

    //起始位
    USART_SendData(USART1, 'A');
	
    //通过串口1发数
    USART_SendData(USART1, u_Num.Uint8[0]);
    USART_SendData(USART1, u_Num.Uint8[1]);
    USART_SendData(USART1, u_Num.Uint8[2]);
    USART_SendData(USART1, u_Num.Uint8[3]);
	
    //终止位
    USART_SendData(USART1, 'J');
}