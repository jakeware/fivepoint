#ifndef TIMER_H
#define TIMER_H

#include <sys/time.h>

// Simple timer for profiling code execution times
class Timer
{

private:
    struct timeval time_start;
    struct timeval time_end;
    double dStart;
    double dEnd;

    const int SEC_TO_MICROSEC = 1000000;

public:
    void start()
    {
        gettimeofday(&time_start, NULL);
    }

    void end()
    {
        gettimeofday(&time_end, NULL);
        dStart = time_start.tv_sec * SEC_TO_MICROSEC + time_start.tv_usec;
        dEnd = time_end.tv_sec * SEC_TO_MICROSEC + time_end.tv_usec;
    }

    double get_time_ms()
    {
        return (dEnd - dStart) / 1000;
    }
};

#endif // TIMER_H
