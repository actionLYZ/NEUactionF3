/**
 ******************************************************************************
 * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
 * @author  MCD Application Team
 * @version V1.0.1
 * @date    13-April-2012
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "elmo.h"
#include "lyz.h"
#include "wan.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
//定位系统
//定位系统


extern POSITION_T Position_t;     //定位系统

extern POSITION_T getPosition_t;  //获得的定位

extern int        g_plan;         //跑场方案（顺逆时针）
extern int8_t     whiteBall;      //白球信号
extern int8_t     blackBall;      //黑球信号
extern float      angleError, xError, yError;
extern uint8_t    g_ballSignal;
extern int32_t    g_shootV;
extern int32_t    g_shootFactV;
int               shootStart = 0, ballColor = 1;

float GetAngleZ(void)
{
	float angle = Position_t.angle + 90;

	if (angle > 180)
		angle = angle - 360;
	return angle;
}
float GetPosx(void)
{
	return Position_t.X;
}
float GetPosy(void)
{
	return Position_t.Y;
}


void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}

#ifdef C0
/**
 * @brief  CAN2 receive FIFO0 interrupt request handler
 * @note
 * @param  None
 * @retval None
 */
void CAN1_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL();   /* Tell uC/OS-II that we are starting an ISR */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	uint32_t  Id;
	uint8_t   re[8];
	CAN_RxMsg(CAN1, &Id, re, 1);
	if (shootStart)
	{
		if (Id == 0x30)
		{
			if (re[0] == 100)
				ballColor = 1;
			else if (re[0] == 1)
				ballColor = 2;
		}
	}

	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}
#else
#ifdef WAN
/**
 * @brief  CAN2 receive FIFO0 interrupt request handler
 * @note
 * @param  None
 * @retval None
 */
void CAN1_RX0_IRQHandler(void)
{
	static uint32_t StdId         = 0;
	static uint8_t  i             = 1;
	static uint8_t  CAN1Buffer[8] = { 0 };

	// g_ballSignal置0，表明球来了
	g_ballSignal = 0;
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN1, &StdId, CAN1Buffer, &i);
	if (CAN_MessagePending(CAN1, CAN_FIFO0) != 0)
	{
		//分球的ID 0x30
		if (StdId == 0x30)
		{
			if (CAN1Buffer[0] == 100)
			{
				//白球信号来临
				whiteBall = 1;
				blackBall = 0;
			}
			if (CAN1Buffer[0] == 1)
			{
				//黑球信号来临
				blackBall = 1;
				whiteBall = 0;
			}
		}
	}
	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}
#endif
#endif
/*************定时器2******start************/
//每1ms调用一次

extern OS_EVENT *PeriodSem;
void TIM2_IRQHandler(void)
{
#define PERIOD_COUNTER    10

	//用来计数10次，产生10ms的定时器
	static uint8_t  periodCounter = PERIOD_COUNTER;

	OS_CPU_SR       cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		//实现10ms 发送1次信号量
		periodCounter--;
		if (periodCounter == 0)
		{
			OSSemPost(PeriodSem);
			periodCounter = PERIOD_COUNTER;
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	OSIntExit();
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	OSIntExit();
}

void TIM3_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	OSIntExit();
}

void TIM4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	OSIntExit();
}

void UART4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	OSIntExit();
}
/***************************试场调参数用蓝牙串口中断*****************************************************/
void USART1_IRQHandler(void)
{
	static uint8_t  ch = 0;
	static uint8_t  count = 0, i = 0;
	static union
	{
		int32_t vel32;
		uint8_t vel[4];
	}         V;
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		ch = USART_ReceiveData(USART1);
		switch (count)
		{
		case 0:

			// 起始位
			if (ch == 'A')
				count++;
			else
				count = 0;
			break;

		case 1:

			//接收发射台蓝牙发送的脉冲数
			V.vel[i] = ch;
			i++;
			if (i >= 4)
			{
				i = 0;
				count++;
			}
			break;

		case 2:

			// 终止符
			if (ch == 'R')
			{
				//主控蓝牙给发射台蓝牙发送的速度（脉冲每秒）
				g_shootV = V.vel32;
			}
			else if (ch == 'V')
			{
				//发射台蓝牙返回的射球机实时转速
				g_shootFactV = V.vel32;
			}
			else
			{
			}
			count = 0;
			break;

		default:
			count = 0;
			break;
		}
	}
	else
	{
		USART_ClearITPendingBit(USART1, USART_IT_PE);
		USART_ClearITPendingBit(USART1, USART_IT_TXE);
		USART_ClearITPendingBit(USART1, USART_IT_TC);
		USART_ClearITPendingBit(USART1, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART1, USART_IT_IDLE);
		USART_ClearITPendingBit(USART1, USART_IT_LBD);
		USART_ClearITPendingBit(USART1, USART_IT_CTS);
		USART_ClearITPendingBit(USART1, USART_IT_ERR);
		USART_ClearITPendingBit(USART1, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART1, USART_IT_NE);
		USART_ClearITPendingBit(USART1, USART_IT_FE);
		USART_ReceiveData(USART1);
	}
	OSIntExit();
}

//定位系统
void USART3_IRQHandler(void) //更新频率200Hz
{
	static uint8_t ch;
	static union
	{
		uint8_t data[24];
		float   ActVal[6];
	}               posture;
	static uint8_t  count = 0;
	static uint8_t  i     = 0;
	OS_CPU_SR       cpu_sr;

	OS_ENTER_CRITICAL(); /*Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		ch = USART_ReceiveData(USART3);
		switch (count)
		{
		case 0:
			if (ch == 0x0d)
				count++;
			else
				count = 0;
			break;

		case 1:
			if (ch == 0x0a)
			{
				i = 0;
				count++;
			}
			else if (ch == 0x0d)
			{
				;
			}
			else
			{
				count = 0;
			}
			break;

		case 2:
			posture.data[i] = ch;
			i++;
			if (i >= 24)
			{
				i = 0;
				count++;
			}
			break;

		case 3:
			if (ch == 0x0a)
				count++;
			else
				count = 0;
			break;

		case 4:
			if (ch == 0x0d)
			{
				//获取当前坐标
				getPosition_t.angle = -posture.ActVal[0];
				getPosition_t.X     = posture.ActVal[3];
				getPosition_t.Y     = posture.ActVal[4];

				//矫正角度
				Position_t.angle = getPosition_t.angle - angleError;
				if (Position_t.angle > 180)
					Position_t.angle -= 360;
				if (Position_t.angle <= -180)
					Position_t.angle += 360;

				//旋转坐标系
				Position_t.X  = getPosition_t.X * cos(Angel2PI(angleError)) + getPosition_t.Y * sin(Angel2PI(angleError));
				Position_t.Y  = getPosition_t.Y * cos(Angel2PI(angleError)) - getPosition_t.X * sin(Angel2PI(angleError));

				//平移坐标系
				Position_t.X  -= xError;
				Position_t.Y  -= yError;

				//计算,角度与x坐标镜像对称
				Position_t.angle  *= g_plan;
				Position_t.X      *= g_plan;
				if (Position_t.angle == -180)
					Position_t.angle = 180;
			}
			count = 0;
			break;

		default:
			count = 0;
			break;
		}
	}
	else
	{
		USART_ClearITPendingBit(USART3, USART_IT_PE);
		USART_ClearITPendingBit(USART3, USART_IT_TXE);
		USART_ClearITPendingBit(USART3, USART_IT_TC);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART3, USART_IT_IDLE);
		USART_ClearITPendingBit(USART3, USART_IT_LBD);
		USART_ClearITPendingBit(USART3, USART_IT_CTS);
		USART_ClearITPendingBit(USART3, USART_IT_ERR);
		USART_ClearITPendingBit(USART3, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART3, USART_IT_NE);
		USART_ClearITPendingBit(USART3, USART_IT_FE);
		USART_ReceiveData(USART3);
	}
//	else if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
//	{
//		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//	}
//	else
//	{
//		USART_OUT(USART3,(u8*)"error");
//	}
	OSIntExit();
}

extern uint8_t  g_camera;
extern int8_t   g_cameraAng[50];
extern uint8_t  g_cameraDis[50];
extern int8_t   g_cameraFin;
extern int8_t   g_cameraNum;
extern uint8_t  g_cameraPlan;
//方案1 发三个区域的球数
int             ballN_L, ballN_M, ballN_R;
//方案2 发球数最多的那个角度
float           bestAngle;
//方案3 最近球的极坐标
float           nearestAngle, nearestDis;
//方案4 所有球的角度和距离
int8_t          arr1[20];
uint8_t         arr2[20];
int             go, arr_number;
void USART2_IRQHandler(void)
{
	uint8_t         camera;
	static uint8_t  i = 0, data = 1, num = 0, best = 0, nearest = 0;
	OS_CPU_SR       cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
#ifdef WAN

	static bool AngOrDis = 0, flag = 0;
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		g_camera = USART_ReceiveData(USART2);

		//E4,E6全为高电平，发送的是所有球的极坐标
		if (g_cameraPlan == 3)
		{
			//接收到终止位，表明接收数据停止
			if (g_camera == 0xD5)
			{
				//g_cameraFin置1表明接收完成，再传参到主函数中
				g_cameraFin = 1;
				flag        = 0;
			}
			if (flag)
			{
				//接受角度数据
				if (AngOrDis == 0)
				{
					g_cameraAng[g_cameraNum]  = g_camera;
					AngOrDis                  = 1;
				}

				//接受距离数据
				else
				{
					g_cameraDis[g_cameraNum]  = g_camera;
					AngOrDis                  = 0;
					g_cameraNum++;
				}
			}

			//接受到起始位，表明下一次数据为可以接收的数据
			if (g_camera == 0xD6)
			{
				flag        = 1;
				g_cameraNum = 0;
			}
		}

		//最近球的极坐标
		else if (g_cameraPlan == 2)
		{
			if (flag)
			{
				if (AngOrDis == 0)
				{
					g_cameraAng[0]  = g_camera;
					AngOrDis        = 1;
				}
				else
				{
					g_cameraDis[0]  = g_camera;
					AngOrDis        = 0;
					flag            = 0;
				}
			}
			if (g_camera == 0xD8)
				flag = 1;
		}

		//球最多的角度
		else if (g_cameraPlan == 1)
		{
			if (flag)
			{
				g_cameraAng[0] = g_camera;

				//将此时的陀螺仪的角度发送出去
				SendAng(Position_t.angle);
				flag = 0;
			}
			if (g_camera == 0xDA)
				flag = 1;
		}

		//三个区域球的数量 三个数
		else if (g_cameraPlan == 0)
		{
			if (flag)
			{
				//暂时用距离数组存储个数
				g_cameraDis[g_cameraNum] = g_camera;
				g_cameraNum++;
			}
			if (g_camera == 0xDC)
			{
				flag        = 1;
				g_cameraNum = 0;
			}
		}
	}
	else
	{
		USART_ClearITPendingBit(USART2, USART_IT_PE);
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		USART_ClearITPendingBit(USART2, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
		USART_ClearITPendingBit(USART2, USART_IT_LBD);
		USART_ClearITPendingBit(USART2, USART_IT_CTS);
		USART_ClearITPendingBit(USART2, USART_IT_ERR);
		USART_ClearITPendingBit(USART2, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART2, USART_IT_NE);
		USART_ClearITPendingBit(USART2, USART_IT_FE);
		USART_ReceiveData(USART2);
	}

#else
#ifdef C0
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		camera = USART_ReceiveData(USART2);
		//GPIO4=LOW,GPIO6=LOW 分三个区域 发三个方位的球数
		if (camera == 0xDC)
			num = 1;
		switch (num)
		{
		case 1:
		{
			ballN_L = camera;
			num     = 2;
		} break;

		case 2:
		{
			ballN_M = camera;
			num     = 3;
		} break;

		case 3:
		{
			ballN_R = camera;
			num     = 0;
			go      = 1;
			if (ballN_L == 0 && ballN_M == 0 && ballN_R == 0)
				arr_number = 0;
			else
				arr_number = ballN_L + ballN_M + ballN_R;
		} break;

		default:
			break;
		}
		//GPIO4=LOW,GPIO6=HIGH 球最多的角度 发一个角度
		if (camera == 0xDA)
			best = 1;
		if (best)
		{
			bestAngle = camera;
			if (bestAngle == 0)
				arr_number = 0;
			else
				arr_number = 1;
			best  = 0;
			go    = 1;
		}
		//GPIO4=HIGH,GPIO6=LOW 最近球的坐标 发一个角度 一个距离
		if (camera == 0xD8)
			nearest = 1;
		switch (nearest)
		{
		case 1:
		{
			nearestAngle = camera;
			if (nearestAngle == 0xD8)
			{
				arr_number  = 0;
				nearest     = 0;
			}
			else
			{
				nearest = 2;
			}
		} break;

		case 2:
		{
			nearestDis  = camera * 10;
			nearest     = 0;
			go          = 1;
		} break;

		default:
			break;
		}
		//GPIO4=HIGH,GPIO6=HIGH 所有极坐标 发每个球的角度和坐标
		if (camera == 0xD5)
		{
			go = 1; i = 0;
		}
		if (i == 1)
		{
			switch (data)
			{
			case 1:
			{
				arr1[arr_number]  = camera;
				data              = 2;
			} break;

			case 2:
			{
				arr2[arr_number]  = camera * 10;
				data              = 1;
				arr_number++;
			} break;

			default:
				break;
			}
		}
		if (camera == 0xD6)
		{
			i = 1; arr_number = 0;
		}
	}
	else
	{
		USART_ClearITPendingBit(USART2, USART_IT_PE);
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		USART_ClearITPendingBit(USART2, USART_IT_TC);
		USART_ClearITPendingBit(USART2, USART_IT_ORE_RX);
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
		USART_ClearITPendingBit(USART2, USART_IT_LBD);
		USART_ClearITPendingBit(USART2, USART_IT_CTS);
		USART_ClearITPendingBit(USART2, USART_IT_ERR);
		USART_ClearITPendingBit(USART2, USART_IT_ORE_ER);
		USART_ClearITPendingBit(USART2, USART_IT_NE);
		USART_ClearITPendingBit(USART2, USART_IT_FE);
		USART_ReceiveData(USART2);
	}
#endif
#endif
	OSIntExit();
}
/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
	while (1)
	{
	}
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}
