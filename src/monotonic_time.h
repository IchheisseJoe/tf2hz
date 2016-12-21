#ifndef __MONOTONIC_TIME_H__
#define __MONOTONIC_TIME_H__

#include <time.h>
#include <stdint.h>

class MonotonicTime
{
	private:
		struct timespec		m_TimeSpec;
		
	public:
		MonotonicTime();
		MonotonicTime(const MonotonicTime&);	//	copy constructer
		MonotonicTime& Now();
		MonotonicTime operator-(MonotonicTime&);
		MonotonicTime operator+(MonotonicTime&);
		int64_t	ToNanoSecond();
		int64_t	ToMicroSecond();
		int64_t	ToMilliSecond();
		int64_t ToSecond();
};

#endif