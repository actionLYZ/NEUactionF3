#include "moveBase.h"
#include "can.h"
#include "math.h"
#include "elmo.h"
#include "stm32f4xx_it.h"

//车基本尺寸
#define POSYSTEM_TO_GUN (97.2f)        //球出发位置到定位系统原点的距离（单位：mm）
#define POSYSTEM_TO_BACK (64.65f)      //定位系统到车最后位置的距离
#define GUN_TO_BACK (161.85f)          //球出发位置到后背的距离
#define INCLINATION_ANGLE (51)         //发射球与地面的倾向角

//投白，黑球 篮框的坐标位置
#define BASKE_LOCATION_WX (-275.0f)
#define BASKE_LOCATION_WY (2335.35f)
#define BASKE_LOCATION_BX (275.0f)
#define BASKE_LOCATION_BY (2335.35f)

//转圈时提前量
#define AD_MID_SP (700)
#define AD_HIGH_SP (1000)

//高度
#define BASKE_HEIGHT (600.0f)
#define GUN_HEIGHT (175.4f)

#define BALL_WEIGHT (0.046f)           //高尔夫球的重量（单位：kg）
#define ANGTORAD(x) ((x) / 180.0f * PI)//角度值转为弧度制
#define RADTOANG(x) ((x) / PI * 180.0f)//弧度制转为角度值
#define PF(a) ((a)*(a))
 
//万典学长的宏定义
#define CAMERATOGYRO     32              //摄像头距离陀螺仪的距离
#define ADJUSTDIS        50              //左右区域和中间区域的偏差距离调节量，根据实际情况更改(mm)

typedef struct 
{
  int8_t ang;
	uint8_t dis;
}PolarCoo_t;

typedef struct
{
	int hor;
	int ver;	
}Coo_t;

float PidAngle(float exAngle,float actAngle);
float PidCoordinate(float ex,float act);
void ShunClSquare(int speed,float lineLong,float beginX,float beginY);
void NiClSquare(int speed,float lineLong,float beginX,float beginY);
void ClLineAngle(float lineAngle,int speed);
void ClLine(float aimX,float aimY,float lineAngle,int speed);
void ClLine2(float aimX,float aimY,float lineAngle,int speed);
int Mas2(int number1,int number2,int number3,int number4);
int Mas(int number1,int number2,int number3);
PolarCoo_t Closer_Point(int8_t a[20],uint8_t b[20],int sum);
Coo_t Zoning(float X,float Y);
void First_Scan(void);
float AngCamera2Gyro(float distance,float angle);
int In_Or_Out(void);
int Least_H(int a1[10],int a2[10],int a3[10]);
int Least_S(int a1[10],int a2[10],int a3[10],int a4[10]);
void New_Route(int down,int right,int up,int left);
void RunEdge(void);
int Vehicle_Width(int di,int an);
void ScanTrace(int a[10][10]);
