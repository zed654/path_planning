#include "monitor.h"


TimeChecker::TimeChecker()
{
    // loop time calc
    elapsedTime_toggle_flag = false;
    start_flag = false;

    final_elapsed_time = 0;
    elapsed_time_accum = 0;
    elapsed_time = 0;

    gettimeofday(&t1, NULL);
    gettimeofday(&t2, NULL);
}

TimeChecker::~TimeChecker()
{
//    std::cout << "terminate TimeChecker" << std::endl;
}

float TimeChecker::DeparturePointTime()
{
    gettimeofday(&t1, NULL);

    return t1.tv_sec;
}

float TimeChecker::ArrivalPointTime()
{
    float elapsed_time_tmp = 0;
    gettimeofday(&t2, NULL);

    // compute and print the elapsed time in millisec
    elapsed_time_tmp = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsed_time_tmp += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

    // Clipping
    if(elapsed_time_tmp < 0) elapsed_time_tmp *= -1;

    elapsed_time = elapsed_time_tmp;
    return elapsed_time_tmp;
}

float TimeChecker::LoopTimeCalc()
{
    double elapsed_time_tmp = 0;

    if(elapsedTime_toggle_flag == true)
    {
        gettimeofday(&t1, NULL);
        elapsedTime_toggle_flag = false;
    }
    else if(elapsedTime_toggle_flag == false)
    {
        gettimeofday(&t2, NULL);
        elapsedTime_toggle_flag = true;
    }

    // compute and print the elapsed time in millisec
    elapsed_time_tmp = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
    elapsed_time_tmp += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms

    // Clipping
    if(elapsed_time_tmp < 0) elapsed_time_tmp *= -1;
    if(start_flag == false)
    {
        start_flag = true;
        elapsed_time_tmp = 0;
    }

    elapsed_time = elapsed_time_tmp;
    return elapsed_time_tmp;
}
