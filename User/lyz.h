#include <stdbool.h>
/*=====================================================函数定义===================================================*/

int 	IfStart();																												//通过激光判断是否开始
float Piont2Straight(float aimx,float aimy,float angle);								//计算点到直线距离
void 	StaightCLose(float aimx,float aimy,float angle,float speed);			//点斜式直线闭环
void 	GoGoGo();																													//跑场
int 	FirstRound(float speed);																					//第一圈(放球区附近)
bool 	IfStuck();																												//是否卡住不动，是返回true，不是返回false
bool	RunRectangle(int length,int wide,float speed);										//长方形跑场
void	CheckPosition();																									//坐标校正
void	RunCamera();																											//利用摄像头跑场
/*=====================================================结构体定义===================================================*/

//定位系统
typedef struct position_t 
{
	float angle;
	float X;
	float Y;
}POSITION_T;


/*=====================================================宏定义区域===================================================*/
#define RIGHT_LASER		14						//右侧激光信道
#define LEFT_LASER		15						//左侧激光信道
#define SP2PULSE			12.207				//速度转换为脉冲值
#define FIRST_SPEED 	1000 					//第一圈速度
#define RUN_SPEED			1500					//正常跑场速度
#define FIR_ADV				700						//first_advance:第一圈提前量
#define ADV_TUEN 			1000					//提前变向距离
#define SPREAD_DIS		500						//扩散距离（每圈边长增大距离）
#define STUCK_TIME		0.5						//判断卡住时长(s)
#define PI 						3.1415926