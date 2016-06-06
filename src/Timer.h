// This file is part of FivePoint
// 
// Copyright (C) 2016 Vincent Lui (vincent.lui@monash.edu), Monash University
// 
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
