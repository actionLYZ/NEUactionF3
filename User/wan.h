#include <stdbool.h>
#include <stdint.h>
/*=====================================================结构体定义===================================================*/

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

//定义联合体，用来控制电机
typedef union
{
    //这个32位整型数是给电机发送的速度（脉冲/s）
    int32_t Int32 ;
	
    //通过串口发送数据每次只能发8位
    uint8_t Uint8[4];

}num_t;
/*=====================================================宏定义区域==================================================*/
#define CAMERATOGYRO     320             //摄像头距离陀螺仪的距离(mm)
#define ANGTORAD(x)      ((x)*PI/180)    //将角度转换成弧度
#define RADTOANG(x)      ((x)*180/PI)    //将弧度转换成角度
#define ADJUSTDIS        50              //左右区域和中间区域的偏差距离调节量，根据实际情况更改(mm)
#define ANGRIGHTGYRO     29.826          //右车尖与陀螺仪连线和小车中轴线的夹角(度)
#define DISRIGHTGYRO     492.598         //右车尖与陀螺仪的距离(mm)
#define DISSHOOTTOGYRO   92.47           //投射点到陀螺仪的距离
#define READPE4 		      (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4)) //PE4 摄像头拉数据
#define READPE6 		      (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_6)) //PE6 摄像头拉数据
/*=====================================================函数定义===================================================*/
float AvoidOverAngle(float angle);                       //避免角度溢出
void angClose(float V,float aimAngle,float Kp);          //角度闭环
BALLNUM_T SeekMostBall(void);                            //寻找最多小球的区域
float AngCamera2Gyro(float distance,float angle);        //将小球相对于摄像头的角度转换成相对于陀螺仪的角度(度)
float DisBall2Gyro(float distance,float angle);          //将小球相对于摄像头的距离转换成相对于陀螺仪的距离
void CollectMostBall(void);                              //在球最多的区域收集球
void CollecMostBall(void);                               //2号方案粗略收集小球
void RunWithCamera(void);                                //利用摄像头收集球最多的区域的小球
bool	RunRectangleW(int length,int wide,float speed);    //长方形扫场(第二套长方形方案,回字形中轴线不在X=0上)
bool FirstRoundW(void);                                  //第一圈(第二套方案，回字形中轴线不在X=0上)
float Max(uint8_t arr[50],int n);                        //求得距离数组中的最大值
POSXY_T RightHeadPos(void);                            //通过陀螺仪的坐标计算右车头的坐标(顺时针跑场)


