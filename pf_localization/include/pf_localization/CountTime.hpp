/***************************************************************************
 *                              Supcon XiaoHaiBao Team
 *                     -------------------------------------------------
 * Copyright (c) Supcon , ZheJiang, China.
 * All rights reserved.
 *
 * @author bobshenhui@gmail.com
 *
 ****************************************************************************/

/*! @file CountTime.hpp
 *  �������ʱ�����
 */

#ifndef COUNT_TIME_HPP
#define COUNT_TIME_HPP

#include <string.h>
#include <iostream>
#include <stdio.h>

#include <boost/date_time/posix_time/posix_time.hpp>
/*!
 *   �������㸨���鿴����ִ��ʱ����࣬ʹ�÷�����
 *   @code
 *   #include <CountTime.hpp>
 *   CountTime time;
 *   time.begin();
 *   coding .....
 *   time.end();
 *   printf("use time = %f\n",getTime());
 *   @endcode
 */

#if 0
class CountTime {

	struct timeval tpstart; //<��¼�Ŀ�ʼʱ��
	struct timeval tpend; //<��¼�Ľ���ʱ��
public:
	CountTime() {

	}

	/*!
	 * ��ʼ��ʱ
	 * */
	void begin() {
		gettimeofday(&tpstart, NULL);
	}

	/*!
	 * ������ʱ
	 * */
	void end() {
		gettimeofday(&tpend, NULL);
	}

	/*!
	 * @return �������õ�ʱ��
	 */
	float getTime() const {
		float timeuse;
		timeuse = 1000000 * (tpend.tv_sec - tpstart.tv_sec) + tpend.tv_usec
		- tpstart.tv_usec;
		timeuse /= 1000;
		return timeuse;
	}
};

#endif

#ifdef _WIN32
#include <windows.h>
class CountTime {
private:
	LARGE_INTEGER litmp;
	double freq;
	double startTime;
	double endTime;
public:
	CountTime() {
		QueryPerformanceFrequency(&litmp);
		freq = (double) litmp.QuadPart;
		begin();
	}

	/**
	 * ��ʼ��ʱ
	 */
	void begin() {
		QueryPerformanceCounter(&litmp);
		startTime = (double) litmp.QuadPart;

		//Sleep(20);
	}

	/*!
	 * ������ʱ
	 * */
	void end() {
		QueryPerformanceCounter(&litmp);
		endTime = (double) litmp.QuadPart;
	}

	/**
	 * @return ����begin��end֮��Ļ��ѵ�ʱ�䣬��λ�Ǻ���(ms)
	 */
	float getTime() const {

		float time = (float)((endTime - startTime) / freq * 1000);
		return time;
	}

	/**
	 * ��ӡ���ѵ�ʱ��
	 */
	void printString() {
		printf("@Use Time = %f\n",getTime());
	}

};

#else

class CountTime {
private:
	boost::posix_time::ptime beginTime;
	boost::posix_time::ptime endTime;
public:
	CountTime() {
		begin();
	}

	/**
	 * ��ʼ��ʱ
	 */
	void begin() {
		boost::posix_time::ptime t(
				boost::posix_time::microsec_clock::local_time());
		beginTime = t;
	}

	/*!
	 * ������ʱ
	 * */
	void end() {
		boost::posix_time::ptime t(
				boost::posix_time::microsec_clock::local_time());
		endTime = t;
	}

	/**
	 * @return ����begin��end֮��Ļ��ѵ�ʱ�䣬��λ�Ǻ���(ms)
	 */
	float getTime() const {

		boost::posix_time::time_duration t3 = (endTime - beginTime);

		return t3.total_microseconds() / 1000.0f;
	}

	/**
	 * ��ӡ���ѵ�ʱ��
	 */
	void printString() {
		printf("@Use Time = %f\n", getTime());
	}

};

#endif

#endif
