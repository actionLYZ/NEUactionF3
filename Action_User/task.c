/*=====================================================头文件声明区===================================================*/
#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "math.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "lyz.h"
#include "stm32f4xx_adc.h"
#include "wan.h"
#include "moveBase.h"
#include "c0.h"
#include "MotionCard.h"

/*=====================================================信号量定义===================================================*/

OS_EXT INT8U  OSCPUUsage;
OS_EVENT *    PeriodSem;
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

/*=====================================================全局变量声明===================================================*/

//uint8_t g_camera = 0;					     //摄像头收到的数
int8_t      g_cameraAng[50] = { 0 };  //存储摄像头接受到的角度
uint8_t     g_cameraDis[50] = { 0 };  //存储摄像头接受到的距离
int8_t      g_cameraFin     = 0;      //摄像头接收到0xc9置1
int8_t      g_cameraNum     = 0;      //摄像头接收到的数据的个数
POSITION_T  Position_t;               //矫正的定位
POSITION_T  getPosition_t;            //获得的定位
int         g_plan        = 1;        //跑场方案（顺逆时针）
int8_t      whiteBall     = 1;        //白球信号
int8_t      blackBall     = 0;        //黑球信号
uint8_t     g_cameraPlan  = 0;        //摄像头接球方案
uint8_t     g_ballSignal  = 1;        //判断CCD是否看到球
int32_t     g_shootV      = 0;        //串口接收到的速度
int32_t     g_shootFactV  = 0;        //发射电机的实时转速
int32_t     g_collectSpeed = 0;       //收球电机的实时转速(脉冲每秒)
int32_t     g_shootAngle = 0;
int32_t     btV = 0;                  //蓝牙控制发射台转速
int32_t     g_rightPulse = 0;         //记录右轮的脉冲
int32_t     g_leftPulse = 0;          //记录左轮的脉冲
int32_t     g_collectVel = 0;         //记录收球棍子的速度
int32_t     g_pushPosition = 0;       //推球装置的位置
u16         firstLine = 0;            //记录第一圈的目标直线
uint8_t     circleFlag = 0;           //画圆标志位
uint8_t     shootNum = 0;             //记录射球的个数
extern float             angleError, xError , yError ;
void TwoWheelVelControl(float vel, float rotateVel);
float TwoWheelAngleControl(float targetAng);

int     g_camera = 0;     //摄像头收到的数
int     sweepingScheme = 0, blockTime = 0;
int jiguang1, jiguang2;
extern int32_t g_gather;
extern int finishShoot;

void App_Task()
{
	CPU_INT08U os_err;

	os_err = os_err; /*防止警告...*/

	/*创建信号量*/
	PeriodSem = OSSemCreate(0);

	/*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
	                      (void *)0,
	                      (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
	                      (INT8U)Config_TASK_START_PRIO);

	os_err = OSTaskCreate((void (*)(void *))WalkTask,
	                      (void *)0,
	                      (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
	                      (INT8U)Walk_TASK_PRIO);
}


/*=====================================================初始化任务===================================================*/
void ConfigTask(void)
{
	CPU_INT08U os_err;

	os_err = os_err;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	//1ms定时器用于控制WalkTask周期
	TIM_Init(TIM2, 99, 839, 0, 0);
	AdcInit();            //初始化adc端口
	BeepInit();           //初始化蜂鸣器端口
//	BEEP_Init();
	LimitSwitch();        //行程开关初始化
	NumTypeInit();        //摄像头高低电平拉数据PE4 PE6初始化
	BufferZizeInit(400);  //控制卡初始化

	//CAN初始化
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

	//射球转速
	USART1_Init(115200);

	//树莓派
	USART2_Init(115200);

	//坐标
	USART3_Init(115200);

	//蓝牙串口
	UART5_Init(921600);

	//驱动器初始化
	elmo_Init(CAN2);
	elmo_Enable(CAN2, 1);
	elmo_Enable(CAN2, 2);


	//收球电机初始化
	Vel_cfg(CAN1, COLLECT_BALL_ID, 50000, 50000);

	VelCrl(CAN2, 1, 0);
	VelCrl(CAN2, 2, 0);
	
	//光电门初始化
	PhotoelectricityInit();

	OSTaskSuspend(OS_PRIO_SELF);
}

//看车是在跑，还是在矫正、射球
int carRun = 0, ifEscape = 0, count = 0;

/********************************测试********************/
extern float blindTime;
extern float photoElectricityCount;//球的数量
extern float velocity;
int test;
/*******************************************************/


/*=====================================================执行函数===================================================*/
void WalkTask(void)
{
	CPU_INT08U os_err;

	os_err = os_err;
	
//	//拉低PE6，拉高PE4的电平，接收球最多区域的角度
		GPIO_SetBits(GPIOE, GPIO_Pin_4);
		GPIO_ResetBits(GPIOE, GPIO_Pin_6);
		g_cameraPlan = 2;
	
	//延时，稳定定位系统
	delay_s(12);
	
	//棍子，发射机构的初始速度
	CollectBallVelCtr(60);
	delay_s(3);	
	ShootCtr(60);
	
//	//鸣笛
	GPIO_SetBits(GPIOE,GPIO_Pin_7);
	
	//激光触发
  firstLine = LaserTrigger();
	USART_OUT(UART5,(u8*)"%d\t%d\r\n",(int)g_plan,(int)firstLine);
	
	//关蜂鸣器
	GPIO_ResetBits(GPIOE,GPIO_Pin_7);
	finishShoot=1;
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
//		right = Get_Adc_Average(RIGHT_LASER, 20);
//		left  = Get_Adc_Average(LEFT_LASER, 20);
//    USART_OUT(UART5,(u8*)"r%d\tl%d\r\n",(int)right,(int)left);
//		CountBall();
		//USART_OUT(UART5,"%d\t%d\t%d\r\n",(int)blindTime,(int)velocity,(int)photoElectricityCount);
//		ReadActualVel(CAN2,RIGHT_MOTOR_WHEEL_ID);
//		ReadActualVel(CAN2,LEFT_MOTOR_WHEEL_ID);
//		ShootBallW(); 
//		RunWithCamera1(2);
//		USART_OUT(UART5,(u8*)"%d\t%d\t%d\r\n",(int)Position_t.X,(int)Position_t.Y,(int)Position_t.angle);
		if (ifEscape)
		{
			//逃逸完成后，ifEscape清零
			if(Escape())
			{
				ifEscape = 0;
			}
		}
		else
 		{
			GoGoGo(firstLine);
		}
		if (stuckCar(200,200))
		{
			if (carRun)
				ifEscape = 1;
			else
				ifEscape = 0;
		}
//		finishShoot++;
//		if(finishShoot==100)
//		{
//			PushBall();
//		}
//		if(finishShoot==200)
//		{
//			finishShoot=0;
//			PushBallReset();
//		}
	}
}

void TwoWheelWalk(float x, float y, float vel)
{
	float angle     = 0.0f;
	float rotateVel = 0.0f;

	//当前点到目标点的方向角度
	angle = atan2(y - Position_t.Y, Position_t.X) * 180.0f / 3.1415926f;


	rotateVel = TwoWheelAngleControl(angle);

	TwoWheelVelControl(vel, rotateVel);
//	MultiPinThroughPro(0,0,x,y,vel);
}

void TwoWheelVelControl(float vel, float rotateVel)
{
	rotateVel = rotateVel * 3.1415926f / 180.0f;
	VelCrl(CAN2, 1, SP2PULSE * (vel + rotateVel * WIDTH * 0.5f));
	VelCrl(CAN2, 2, -SP2PULSE * (vel - rotateVel * WIDTH * 0.5f));
}

float angleErr;
float anglePresent;
//角度闭环
float TwoWheelAngleControl(float targetAng)
{
	angleErr = targetAng - GetAngleZ();

	if (angleErr > 180)
		angleErr = angleErr - 360;
	if (angleErr < -180)
		angleErr = angleErr + 360;
	anglePresent = GetAngleZ();
	return angleErr * 10.0f;
}
