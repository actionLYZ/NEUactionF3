#include <stdbool.h>



/*=====================================================函数定义===================================================*/


int   IfStart(void);                                                  //通过激光判断是否开始
float Piont2Straight(float aimx, float aimy, float angle);            //计算点到直线距离
void  StaightCLose(float aimx, float aimy, float angle, float speed); //点斜式直线闭环
void GoGoGo(float fLine,int stat);                                    //跑场
bool  FirstRound(float firstLine);                                    //第一圈(放球区附近)
bool  IfStuck(void);                                                  //是否卡住不动，是返回true，不是返回false
bool  IfStuck2(void);                                                 //是否靠住墙不动，当行程开关加入后删除此函数
bool  RunRectangle(int length, int wide, float speed);                //长方形跑场
void  TurnAngle(float angel, int speed);                              //原地旋转指定角度
int CheckPosition(void);                                              //坐标校正
int RunCamera(void);                                                  //利用摄像头跑场
int LaserCheck(void);                                                 //激光矫正,矫正成功返回true,不成功返回false
float Angel2PI(float angel);                                          //将角度转换为PI
void InputArr(char[], unsigned char[], int);                          //输出两个数组
void JudgeState(char[]);																							//判断小车当前状态并输出日志

/*=====================================================宏定义区域===================================================*/
#define STUCK_TIME      0.8                                         //判断卡住时长(s)
#define FIRST_SPEED     1000                                        //第一圈速度
#define RUN_SPEED       1500                                        //正常跑场速度
#define FIR_ADV         1500                                         //first_advance:第一圈提前量
#define ADV_TUEN        1200                                        //提前变向距离
#define SPREAD_DIS      400                                         //扩散距离（每圈边长增大距离）
#define CAMERA_DIS      700                                         //摄像头找球时扩散距离
#define RIGHT_LASER     14                                          //右侧激光信道
#define LEFT_LASER      15                                          //左侧激光信道
#define SP2PULSE        10.865                                      //速度转换为脉冲值
#define LEFTSHELTER     -1                                          //左侧激光被挡
#define RIGHTSHELTER    1                                           //右侧激光被挡
#define SWITCHE2        (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2))  //PE2 行程开关
#define SWITCHC0        (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0))  //PC0 行程开关
#define SWITCHC2        (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_2))  //PC2 行程开关
#define LOG_NOTE						if(0)																		//日志注释if(1)为输出日志，if(0)为不输出日志
#define POS_NOTE			   		if(1)																		//坐标注释
#define DOCUMENT_NOTE				if(1)																		//将日志写入文件
/*=====================================================结构体定义===================================================*/
