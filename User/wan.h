#include <stdbool.h>
#include <stdint.h>
#include "lyz.h"
/*=====================================================结构体定义===================================================*/
//定位系统
typedef struct
{
	float angle;
	float X;
	float Y;
}POSITION_T;

//记录左中右小球个数
typedef struct
{
	int leftNum;
	int midNum;
	int rightNum;
}BALLNUM_T;

//记录某个点的XY坐标
typedef struct
{
	float X;
	float Y;
}POSXY_T;

typedef union
{
    //这个32位整型数是给电机发送的速度（脉冲/s）
    int32_t velInt32;
    //通过串口发送数据每次只能发8位
    uint8_t velUint8[4];

}shootPara_t;
/*=====================================================宏定义区域==================================================*/

//摄像头距离陀螺仪的距离(mm)
#define CAMERATOGYRO     221.32 

//将角度转换成弧度         
#define ANGTORAD(x)      ((x)*PI/180)   

//将弧度转换成角度
#define RADTOANG(x)      ((x)*180/PI) 

//左右区域和中间区域的偏差距离调节量，根据实际情况更改(mm)   
#define ADJUSTDIS        50              

//右车尖与陀螺仪连线和小车中轴线的夹角(度)
#define ANGRIGHTGYRO     29.826      

//右车尖与陀螺仪的距离(mm)    
#define DISRIGHTGYRO     492.598         

//投射点到陀螺仪的距离
#define DISSHOOTTOGYRO   92.47           

//PE4 摄像头拉数据
#define READPE4 	(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)) 

//PE6 摄像头拉数据
#define READPE6     (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)) 

//放置白球的空中储存室的X坐标(mm)
#define WHITEX           -162.5          

//放置白（黑）球的空中储存室的Y坐标(mm)
#define BALLY            2335.35            

//放置黑球的空中储存室的X坐标(mm)
#define BLACKX           162.5        

//储藏室的高度(mm)   
#define BASKETHIGH       600             

//重力加速度(mm/s2)
#define G                9800            

//将球出射速度转换成投球器的脉冲
#define VTOPULSE         12.358          

//宏定义电机转一圈的脉冲数
#define COUNT_PER_ROUND (4096.0f)

//宏定义每度对应脉冲数
#define COUNT_PER_DEGREE  (COUNT_PER_ROUND/360.0f)

//宏定义航向角减速比
#define YAW_REDUCTION_RATIO (4.0f)

//宏定义发射机构航向电机ID
#define GUN_YAW_ID (7)

//宏定义送弹电机ID
#define PUSH_BALL_ID (6)

//宏定义送弹机构送弹时电机位置
#define PUSH_POSITION (4000)

//宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION (5)

//宏定义收球电机ID
#define COLLECT_BALL_ID (8)

//宏定义左轮电机ID
#define LEFT_MOTOR_WHEEL_ID (2)

//宏定义右轮电机ID
#define RIGHT_MOTOR_WHEEL_ID (1)
/*=====================================================函数定义===================================================*/

//将摄像头发完数据瞬间的角度发送出去
void SendAng(float ang);     

                           
float GetAng(void);

//获取小车行驶的距离  
float GetDistance(POSITION_T startPoint);             

//避免角度溢出   
float AvoidOverAngle(float angle);   

//角度闭环                   
void angClose(float V,float aimAngle,float Kp);  

//矫正函数
void CheckError();       

//将小球相对于摄像头的角度转换成相对于陀螺仪的角度(度) 
float AngCamera2Gyro(float distance,float angle);        

//将小球相对于摄像头的距离转换成相对于陀螺仪的距离(mm)
float DisBall2Gyro(float distance,float angle);     

//寻找最多小球的区域     
BALLNUM_T SeekMostBall(void);   

//将摄像机中球的极坐标转换成XY坐标
POSXY_T BallPosXY(uint8_t distance,int8_t angle);                         

//求得距离数组中的最大值
float Max(uint8_t arr[50],int n);                   

//求得距离数组中的最小值     
float Min(uint8_t arr[50],int n);                   

//在球最多的区域收集球(精细)     
void CollectMostBall(void);                              

//2号方案粗略收集小球(粗略)
void CollecMostBall(void);                   

//摄像头返回的数值是球最多区域的角度,直接走那个角度            
void CollecMostBall1(void);                             

//利用摄像头收集球最多的区域的小球,基本走形回字 
void RunWithCamera1(void);                               

//第二套摄像头方案(前进后退，吃完视野中的所有球)
int RunWithCamera2(void);                                

//第一圈(第二套方案，回字形中轴线不在X=0上)
bool FirstRoundW(void);                                  

//长方形扫场(第二套长方形方案,回字形中轴线不在X=0上)
bool RunRectangleW(int length,int wide,float speed);    

//通过陀螺仪的坐标计算右车头的坐标(顺时针跑场)
POSXY_T RightHeadPos(void);                           

//计算投球点的坐标   
POSXY_T ShootPointPos(void);                         

//逃逸函数    
int IfEscape(void);                                      

//给投球电机发送速度（脉冲/s） 
void SendUint8(int32_t pulse);                         

//定点投球方案            
int ShootBall(void);                                    

//发射航向角转换函数 由度转换为脉冲
float YawTransform(float yawAngle);

//发射航向角控制函数 单位：度（枪顺时针转为正，逆时针为负）
void YawAngleCtr(float yawAngle);

//送弹推球函数
void PushBall(void);

//送弹推球收回函数
void PushBallReset(void);

//收球电机速度转换函数 由转每秒转换为脉冲
float CollectBallVelTrans(float round);

//收球电机速度控制函数 单位：转每秒
void CollectBallVelCtr(float round);

//发射电机速度转换函数 由转每秒转换为脉冲
int32_t shootVelTrans(float roundPerS);

//发射电机速度控制函数 单位：转每秒
void ShootCtr(float rps);

// 徐鹏学长任务
void DingDian(float X,float Y,float V);

