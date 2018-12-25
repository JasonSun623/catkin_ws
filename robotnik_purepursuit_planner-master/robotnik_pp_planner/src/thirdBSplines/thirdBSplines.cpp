// thirdBSplines.cpp : 定义 DLL 应用程序的导出函数。
//

#include <stdio.h>
#include <math.h>

#define PI 3.1412596

bool GetThirdBSplines(int pInData[], int nInLen, float* pOutData, int nOutLen, int nIter, float *pOutRealData, int nOutRealLen)
{
	if (nInLen != 8 || nOutLen != 33 || nOutRealLen != 6)
		return false;

	float fIterBegin = 0.0;
	float fIterEnd = 1.0;

	float dt_iter = 1.0 / (float)nIter;

	float fIter = fIterBegin;

	int i = 0;
	
	int *vectPartControls = pInData;

	while (fIter <= fIterEnd + 0.000001)
	{
		static float fAngleOld = 0.0;
		float fAngle = 0.0;

		float x_begin = 0.0;
		float y_begin = 0.0;
		float x_end = 0.0;
		float y_end = 0.0;

		float f_angle = 0.0;

		float dx_t = 0.0;
		float dy_t = 0.0;
		float dt = 0.0;

		x_begin = vectPartControls[0] * 1.0 / 6.0*pow(1 - fIter, 3)
			+ vectPartControls[2] * 1.0 / 6.0*(3.0*pow(fIter, 3) - 6.0*pow(fIter, 2) + 4.0)
			+ vectPartControls[4] * 1.0 / 6.0*(-3.0*pow(fIter, 3) + 3.0*pow(fIter, 2) + 3.0*fIter + 1)
			+ vectPartControls[6] * 1.0 / 6.0*pow(fIter, 3);
		x_end = vectPartControls[0] * 1.0 / 6.0*pow(1 - fIter - dt_iter, 3)
			+ vectPartControls[2] * 1.0 / 6.0*(3.0*pow(fIter + dt_iter, 3) - 6.0*pow(fIter + dt_iter, 2) + 4.0)
			+ vectPartControls[4] * 1.0 / 6.0*(-3.0*pow(fIter + dt_iter, 3) + 3.0*pow(fIter + dt_iter, 2) + 3.0*(fIter + dt_iter) + 1)
			+ vectPartControls[6] * 1.0 / 6.0*pow(fIter + dt_iter, 3);

		y_begin = vectPartControls[1] * 1.0 / 6.0*pow(1 - fIter, 3)
			+ vectPartControls[3] * 1.0 / 6.0*(3.0*pow(fIter, 3) - 6.0*pow(fIter, 2) + 4.0)
			+ vectPartControls[5] * 1.0 / 6.0*(-3.0*pow(fIter, 3) + 3.0*pow(fIter, 2) + 3.0*fIter + 1)
			+ vectPartControls[7] * 1.0 / 6.0*pow(fIter, 3);
		y_end = vectPartControls[1] * 1.0 / 6.0*pow(1 - fIter - dt_iter, 3)
			+ vectPartControls[3] * 1.0 / 6.0*(3.0*pow(fIter + dt_iter, 3) - 6.0*pow(fIter + dt_iter, 2) + 4.0)
			+ vectPartControls[5] * 1.0 / 6.0*(-3.0*pow(fIter + dt_iter, 3) + 3.0*pow(fIter + dt_iter, 2) + 3.0*(fIter + dt_iter) + 1)
			+ vectPartControls[7] * 1.0 / 6.0*pow(fIter + dt_iter, 3);

		dx_t = vectPartControls[0] * 3.0 / 6.0*pow(1.0 - fIter, 2)*-1.0
			+ vectPartControls[2] * 1.0 / 6.0*(9.0*pow(fIter, 2) - 12.0*fIter)
			+ vectPartControls[4] * 1.0 / 6.0*(-9.0*pow(fIter, 2) + 6.0*fIter + 3.0)
			+ vectPartControls[6] * 3.0 / 6.0*pow(fIter, 2);
		dy_t = vectPartControls[1] * 3.0 / 6.0*pow(1.0 - fIter, 2)*-1.0
			+ vectPartControls[3] * 1.0 / 6.0*(9.0*pow(fIter, 2) - 12.0*fIter)
			+ vectPartControls[5] * 1.0 / 6.0*(-9.0*pow(fIter, 2) + 6.0*fIter + 3.0)
			+ vectPartControls[7] * 3.0 / 6.0*pow(fIter, 2);

		//printf("fAngle Begin\n");

		if (dx_t < 0.000001 && dx_t > -0.000001)
		{
			fAngle = 0.0;
			if (dy_t > 0.000001)
				fAngle = PI / 2.0;
			else if (dy_t < -0.000001)
				fAngle = 3.0 * PI / 2.0;
			else
				fAngle = 6 * PI + 0.5;
		}
		else if (dy_t < 0.000001 && dy_t > -0.000001)
		{
			fAngle = 0.0;
			if (dx_t > 0.000001)
				fAngle = 0.0;
			else if (dx_t < -0.000001)
				fAngle = PI;
			else
				fAngle = 6 * PI + 0.5;
		}
		else
		{
			fAngle = atan2(dy_t, dx_t);
			if (fAngle < 0.000001)
				fAngle += 2 * PI;

			//printf("fAngle22222 = %f\n", fAngle);
		}

		//printf("fAngle end = %f\n", fAngle);

		//if (x_end - x_begin < 0.000001 && x_end - x_begin > -0.000001)
		//{
		//	fAngle = 0.0;
		//	if (y_end - y_begin > 0.000001)
		//		fAngle = PI / 2.0;
		//	else if (y_end - y_begin < -0.000001)
		//		fAngle = 3.0 * PI / 2.0;
		//	else
		//		fAngle = 6 * PI + 0.5;
		//}
		//else if (y_end - y_begin < 0.000001 && y_end - y_begin > -0.000001)
		//{
		//	fAngle = 0.0;
		//	if (x_end - x_begin > 0.000001)
		//		fAngle = 0.0;
		//	else if (x_end - x_begin < -0.000001)
		//		fAngle = PI;
		//	else
		//		fAngle = 6 * PI + 0.5;
		//}
		//else
		//{
		//	fAngle = atan2(y_end - y_begin, x_end - x_begin);
		//	fAngle += 2 * PI;
		//}

		if (fIter >= 0.05)
		{
			if (fabs(fAngle - fAngleOld) > PI / 2.0)
			{
				printf("(x_begin:%f y_begin:%f)  --> (x_end:%f y_end:%f) %f\n", x_begin, y_begin, x_end, y_end, fAngle - fAngleOld);
			}
		}

		fAngleOld = fAngle;

		float x = x_begin;
		float y = y_begin;

		pOutData[i * 3] = x;
		pOutData[i * 3 + 1] = y;
		pOutData[i * 3 + 2] = fAngle;

		fIter += dt_iter;

		i++;
	}

	bool bExist = false;
	i = 0;
	float fAngle = 0.0;
	for (; i < 11; i++)
	{
		if (pOutData[i * 3 + 2] < 6 * PI)
		{
			fAngle = pOutData[i * 3 + 2];
			bExist = true;
			break;
		}
	}
	if (i > 0)
	{
		for (int j = 0; j < i; j++)
		{
			pOutData[j * 3 + 2] = fAngle;
		}
	}

	bExist = false;
	i = 10;
	fAngle = 0.0;
	for (; i >= 0; i--)
	{
		if (pOutData[i * 3 + 2] < 6 * PI)
		{
			fAngle = pOutData[i * 3 + 2];
			bExist = true;
			break;
		}
	}
	if (i > 0)
	{
		for (int j = 10; j > i; j--)
		{
			pOutData[j * 3 + 2] = fAngle;
		}
	}

	pOutRealData[0] = pOutData[0];
	pOutRealData[1] = pOutData[1];
	pOutRealData[2] = pOutData[2];
	pOutRealData[3] = pOutData[30];
	pOutRealData[4] = pOutData[31];
	pOutRealData[5] = pOutData[32];

	return true;
}
