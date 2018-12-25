#pragma once
#include <Windows.h>

extern "C" _declspec(dllexport) bool GetThirdBSplines(int pInData[], int nInLen, float* pOutData, int nOutLen, int nIter, float *pOutRealData, int nOutRealLen);