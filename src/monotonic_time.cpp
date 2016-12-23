#include "monotonic_time.h"
#include <iostream>
#include <string.h>

#define NANOSEC_PERSEC	1000000000ll
#define MICROSEC_PERSEC	1000000ll
#define MILLISEC_PERSEC	1000ll
#define NANO2MICRO		1000
#define NANO2MILLI		1000000

using namespace std;

MonotonicTime::MonotonicTime()
{
	memset((void*)&m_TimeSpec, 0, sizeof(struct timespec));
}

MonotonicTime::MonotonicTime(const MonotonicTime& mt)
{
	m_TimeSpec.tv_sec	= mt.m_TimeSpec.tv_sec;
	m_TimeSpec.tv_nsec	= mt.m_TimeSpec.tv_nsec;
}


MonotonicTime& MonotonicTime::Now()
{
	clock_gettime(CLOCK_MONOTONIC, &m_TimeSpec);
	return *this;
}

MonotonicTime MonotonicTime::operator+(MonotonicTime& mt)
{
	MonotonicTime rc;
	int64_t	ns=ToNanoSecond() + mt.ToNanoSecond();
	
	rc.m_TimeSpec.tv_sec	= ns / NANOSEC_PERSEC;
	rc.m_TimeSpec.tv_nsec	= ns % NANOSEC_PERSEC;
	
	return rc;
}

MonotonicTime MonotonicTime::operator-(MonotonicTime& mt)
{
	MonotonicTime rc;
	int64_t	ns=ToNanoSecond() - mt.ToNanoSecond();
	
	rc.m_TimeSpec.tv_sec	= ns / NANOSEC_PERSEC;
	rc.m_TimeSpec.tv_nsec	= ns % NANOSEC_PERSEC;

	return rc;
}

int64_t MonotonicTime::ToNanoSecond()
{
	int64_t rc;
	rc = m_TimeSpec.tv_sec*NANOSEC_PERSEC + m_TimeSpec.tv_nsec;
	return rc;
}

int64_t MonotonicTime::ToMicroSecond()
{
	int64_t rc;
	rc = ((int64_t)m_TimeSpec.tv_sec)*MICROSEC_PERSEC + m_TimeSpec.tv_nsec/NANO2MICRO;
	return rc;

}

int64_t MonotonicTime::ToMilliSecond()
{
	int64_t rc;
	rc = ((int64_t)m_TimeSpec.tv_sec)*MILLISEC_PERSEC + m_TimeSpec.tv_nsec/NANO2MILLI;
	return rc;

}

int64_t MonotonicTime::ToSecond()
{
	return ((int64_t)m_TimeSpec.tv_sec);

}

