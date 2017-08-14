/*=====================================================函数定义===================================================*/

int IfStart();																												//通过激光判断是否开始
float Piont2Straight(float aimx,float aimy,float angle);							//计算点到直线距离
void StaightCLose(float aimx,float aimy,float angle,float speed);			//点斜式直线闭环
void GoGoGo(int plan);																								//跑场
int FirstRound(int plan,int speed);																		//第一圈(放球区附近)


/*=====================================================结构体定义===================================================*/

//定位系统
typedef struct position_t 
{
	float angle;
	float X;
	float Y;
}POSITION_T;


/*=====================================================宏定义区域===================================================*/
#define RIGHT_LASER		14					//右侧激光信道
#define LEFT_LASER		15					//左侧激光信道
#define SP2PULSE		12.207				//速度转换为脉冲值
#define FIRST_SPEED 	1000 				//第一圈速度
#define RUN_SPEED		1500					//正常跑场速度
#define FIR_ADV			700						//first_advance:第一圈提前量
#define ADV_TUEN 1000							//提前变向距离
#define PI 				3.1415926