#ifndef monitor_h
#define monitor_h


#include <sys/time.h>
#include <unistd.h>
#include <iostream>

class TimeChecker
{
private:
    // loop time calc
    bool elapsedTime_toggle_flag;
    bool start_flag;
    struct timeval t1, t2;

protected:
public:

    // Sys loop time checker
    std::string name;
    // loop time calc
    float final_elapsed_time;
    float elapsed_time_accum;
    float elapsed_time;

    // loop time calc
    // return -> elapsed_time
    float LoopTimeCalc();


    // Section time calc
    float DeparturePointTime();
    float ArrivalPointTime();

    TimeChecker();
    virtual ~TimeChecker();
};


#endif /* monitor_h */
