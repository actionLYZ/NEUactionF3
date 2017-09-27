#include "moveBase.h"
#include "can.h"
#include "math.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "can.h"
#include "stm32f4xx_usart.h"

/*=====================================================宏定义区域===================================================*/
//车基本尺寸
#define POSYSTEM_TO_GUN         (97.2f)   //球出发位置到定位系统原点的距离（单位：mm）
#define POSYSTEM_TO_BACK        (64.65f)  //定位系统到车最后位置的距离
#define GUN_TO_BACK             (161.85f) //球出发位置到后背的距离
#define INCLINATION_ANGLE       (51)      //发射球与地面的倾向角

//投白，黑球 篮框的坐标位置
#define BASKE_LOCATION_WX       (-275.0f)
#define BASKE_LOCATION_WY       (2335.35f)
#define BASKE_LOCATION_BX       (275.0f)
#define BASKE_LOCATION_BY       (2335.35f)

//转圈时提前量
#define AD_MID_SP               (700)
#define AD_HIGH_SP              (1000)

//高度
#define BASKE_HEIGHT            (600.0f)
#define GUN_HEIGHT              (175.4f)

#define BALL_WEIGHT             (0.046f) //高尔夫球的重量（单位：kg）
#define PF(a)    ((a) * (a))

#define cameraSpeed (1800)

//宏定义电机转一圈的脉冲数
#define COUNT_PER_ROUND         (4096.0f)

//宏定义每度对应脉冲数
#define COUNT_PER_DEGREE        (COUNT_PER_ROUND / 360.0f)

//宏定义航向角减速比
#define YAW_REDUCTION_RATIO     (4.0f)

//宏定义发射机构航向电机ID
#define GUN_YAW_ID              (7)

//宏定义送弹电机ID
#define PUSH_BALL_ID            (6)

//宏定义送弹机构送弹时电机位置
#define PUSH_POSITION           (4000)

//宏定义送弹机构收回时电机位置
#define PUSH_RESET_POSITION     (5)

//宏定义收球电机ID
#define COLLECT_BALL_ID         (8)

//宏定义左轮电机ID
#define LEFT_MOTOR_WHEEL_ID     (2)

//宏定义右轮电机ID
#define RIGHT_MOTOR_WHEEL_ID    (1)


/*=====================================================结构体定义===================================================*/
//********************************************************************************************************************

typedef struct
{
	int8_t  ang;
	uint8_t dis;
}PolarCoo_t;

typedef struct
{
	int hor;
	int ver;
}Coo_t;

typedef struct
{
	int one;
	int two;
	int there;
	int four;
}Four_t;


/*=====================================================函数定义===================================================*/
float PidAngle(float exAngle, float actAngle);
float PidCoordinate(float ex, float act);
void ShunClSquare(int speed, float lineLong, float beginX, float beginY);
void NiClSquare(int speed, float lineLong, float beginX, float beginY);
void ClLineAngle(float lineAngle, int speed);
void ClLine(float aimX, float aimY, float lineAngle, int speed);
void ClLine2(float aimX, float aimY, float lineAngle, int speed);
int Mas2(int number1, int number2, int number3, int number4);
int Mas(int number1, int number2, int number3);
PolarCoo_t Closer_Point(int8_t a[20], uint8_t b[20], int sum);
Coo_t Zoning(float X, float Y);
void First_Scan(void);
int In_Or_Out(void);
int Least_H(int a1[10], int a2[10], int a3[10]);
int Least_S(int a1[10], int a2[10], int a3[10], int a4[10]);
void New_Route(int down, int right, int up, int left);
int RunEdge(void);
int Vehicle_Width(int di, int an);
int ScanTrace(int a[10][10]);
Four_t Apart(void);
void ChangeOrder1(int8_t a, int8_t b);
void ChangeOrder2(uint8_t a, uint8_t b);
void Left2Right(void);
void Down2Up(void);
float MostSector(void);
float P2P(float a1, float a2, float b1, float b2);
void SendUint8(void);


void GivenPoint(float pointX, float pointY, float givenSpeed);
void PathPlan(float camX, float camY);
void CountBall(void);

/*=====================================================函数定义（万典学长的函数）===================================================*/
