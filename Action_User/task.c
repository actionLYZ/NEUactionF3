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
#include "MotionCard.h"
/*=====================================================信号量定义===================================================*/

OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];

/*=====================================================全局变量声明===================================================*/

uint8_t g_camera = 0;					     //摄像头收到的数
int8_t g_cameraAng[50] = {0};        //存储摄像头接受到的角度
uint8_t g_cameraDis[50] = {0};       //存储摄像头接受到的距离
int8_t g_cameraFin = 0;              //摄像头接收到0xc9置1
int8_t g_cameraNum = 0;              //摄像头接收到的数据的个数
POSITION_T Position_t;		         //矫正的定位
POSITION_T getPosition_t;	         //获得的定位
int g_plan = 1;						         //跑场方案（顺逆时针）
int8_t whiteBall = 1;                //白球信号
int8_t blackBall = 0;                //黑球信号
uint8_t g_cameraPlan = 0;            //摄像头接球方案
uint8_t g_ballSignal = 1;            //判断CCD是否看到球
int32_t g_shootV = 0;                //串口接收到的速度
int32_t g_shootFactV = 0;            //发射电机的实时转速

void TwoWheelVelControl(float vel,float rotateVel);
float TwoWheelAngleControl(float targetAng);

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
	AdcInit();				  //初始化adc端口
	BeepInit();               //初始化蜂鸣器端口
//	BEEP_Init();         	
  LimitSwitch();            //行程开关初始化
	NumTypeInit();            //摄像头高低电平拉数据PE4 PE6初始化
  BufferZizeInit(400);      //控制卡初始化      

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
	UART5_Init(115200);

	//驱动器初始化
	elmo_Init(CAN2);
	elmo_Enable(CAN2,1);
	elmo_Enable(CAN2,2);
	
	//配置速度环
	Vel_cfg(CAN2, 1, 50000, 50000);
	Vel_cfg(CAN2, 2, 50000, 50000);

	//收球电机初始化
	Vel_cfg(CAN1, COLLECT_BALL_ID, 50000,50000);
	
	VelCrl(CAN2, 1, 0);
	VelCrl(CAN2, 2, 0);

	OSTaskSuspend(OS_PRIO_SELF);
}

/*=====================================================执行函数===================================================*/
void WalkTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	int count = 0,i = 0;
  //拉低PE4，拉高PE6的电平，接收球最多区域的角度
	GPIO_ResetBits(GPIOE,GPIO_Pin_4);
	GPIO_SetBits(GPIOE,GPIO_Pin_6);
	g_cameraPlan=1;
	CollectBallVelCtr(40);
	delay_s(10);

//	GPIO_SetBits(GPIOE,GPIO_Pin_7);				//蜂鸣器响，示意可以开始跑
	
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
	{
		OSSemPend(PeriodSem, 0, &os_err);
		

	}
}


void TwoWheelWalk(float x,float y,float vel)
{
	
	float angle = 0.0f;
	float rotateVel = 0.0f;
	//当前点到目标点的方向角度
	angle = atan2(y - Position_t.Y,Position_t.X) * 180.0f / 3.1415926f;
	
	
	rotateVel = TwoWheelAngleControl(angle);
	
	TwoWheelVelControl(vel,rotateVel);
//	MultiPinThroughPro(0,0,x,y,vel);
}


void TwoWheelVelControl(float vel, float rotateVel)
{
	rotateVel = rotateVel * 3.1415926f / 180.0f;
	VelCrl(CAN2,1,SP2PULSE * (vel + rotateVel * WIDTH * 0.5f));
	VelCrl(CAN2,2,-SP2PULSE * (vel - rotateVel * WIDTH * 0.5f));  
}

float angleErr ; 
float anglePresent;
//角度闭环
float TwoWheelAngleControl(float targetAng)
{
	angleErr = targetAng - GetAngleZ();
	
	if(angleErr > 180) angleErr = angleErr - 360;
	if(angleErr < -180) angleErr = angleErr + 360;
	anglePresent = GetAngleZ();
	return (angleErr * 10.0f);
}
		 
		
