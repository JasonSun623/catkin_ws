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
 *  计算代码时间的类
 */

#ifndef COUNT_TIME_HPP
#define COUNT_TIME_HPP

#include <string.h>
#include <iostream>
#include <stdio.h>

#include <boost/date_time/posix_time/posix_time.hpp>
/*!
 *   用来计算辅助查看代码执行时间的类，使用方法是
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

	struct timeval tpstart; //<记录的开始时间
	struct timeval tpend; //<记录的结束时间
public:
	CountTime() {

	}

	/*!
	 * 开始计时
	 * */
	void begin() {
		gettimeofday(&tpstart, NULL);
	}

	/*!
	 * 结束计时
	 * */
	void end() {
		gettimeofday(&tpend, NULL);
	}

	/*!
	 * @return 返回所用的时间
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
	 * 开始计时
	 */
	void begin() {
		QueryPerformanceCounter(&litmp);
		startTime = (double) litmp.QuadPart;

		//Sleep(20);
	}

	/*!
	 * 结束计时
	 * */
	void end() {
		QueryPerformanceCounter(&litmp);
		endTime = (double) litmp.QuadPart;
	}

	/**
	 * @return 返回begin和end之间的花费的时间，单位是毫秒(ms)
	 */
	float getTime() const {

		float time = (float)((endTime - startTime) / freq * 1000);
		return time;
	}

	/**
	 * 打印花费的时间
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
	 * 开始计时
	 */
	void begin() {
		boost::posix_time::ptime t(
				boost::posix_time::microsec_clock::local_time());
		beginTime = t;
	}

	/*!
	 * 结束计时
	 * */
	void end() {
		boost::posix_time::ptime t(
				boost::posix_time::microsec_clock::local_time());
		endTime = t;
	}

	/**
	 * @return 返回begin和end之间的花费的时间，单位是毫秒(ms)
	 */
	float getTime() const {

		boost::posix_time::time_duration t3 = (endTime - beginTime);

		return t3.total_microseconds() / 1000.0f;
	}

	/**
	 * 打印花费的时间
	 */
	void printString() {
		printf("@Use Time = %f\n", getTime());
	}

};

#endif

#endif
