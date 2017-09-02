#include "SpeedPlaning.h"
#include <stdlib.h>
#include "MotionCard.h"
#include "ringBuffer.h"
#include "math.h"
#include "calculate.h"


float GetAccMax(void)
{
	return 700;
}

float GetVelMax(void)
{
	return 2000;
}

float aaasdf = 0.0f;
//�ٶȹ滮����
void SpeedPlaning()
{
	float* vell = NULL;
	float* curvature = NULL;



	int n = GetCount();

	vell = (float *)malloc(n * sizeof(float));
	curvature = (float *)malloc(n * sizeof(float));



	for (int i = 0; i < n; i++)
	{
		curvature[i] = GetRingBufferAverCurvature(i + 1);
		aaasdf= curvature[i];
	}
	curvature[n - 1] = curvature[n - 2];
	//	curvature[0] = curvature[1];


	//�������ٶ��ܹ��������С���ʰ뾶
	float curvatureMaxVell = GetVelMax() * GetVelMax() / (2 * GetAccMax());
   

	//����������ʰ뾶ȫ��Ϊ������ٶ����������С���ʰ뾶
	for (int i = 0; i < n; i++)
	{
		if (curvature[i] > curvatureMaxVell)
		{
			curvature[i] = curvatureMaxVell;
		}
	}
	curvature[0] = curvature[1];
	curvature[n - 1] = curvature[n - 2];


	//ͨ�����ʰ뾶����ö������������ٶ�                                         
	for (int i = 0; i < n; i++)
	{
		vell[i] = sqrt((2 * GetAccMax()) * curvature[i]);
	}


	vell[0] = 100;
	vell[n - 1] = 100;

	float tempVell = 0.0f;
	//ͨ��v2^2 - v1^2 = 2*a*s���ٶ��ٴι滮
	for (int i = 0; i < n - 1; i++)
	{
		if (vell[i + 1] > vell[i])
		{
			tempVell = sqrt(2 * (1 * GetAccMax()) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			if (tempVell < vell[i + 1])
			{
				vell[i + 1] = tempVell;
			}
		}
	}

	for (int i = n - 1; i > 0; i--)
	{
		if (vell[i - 1] > vell[i])
		{
			tempVell = sqrt(2 * (1 * GetAccMax()) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			if (tempVell < vell[i - 1])
			{
				vell[i - 1] = tempVell;
			}
		}
	}

	//����ʱ�滮���ٶȷ��뻷��������
	for (int i = 0; i < n; i++)
	{
		aaasdf= vell[i];
		SetRingBufferPointVell(i + 1, vell[i]);
	}

}






