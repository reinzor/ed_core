#ifndef ED_RATE_H_
#define ED_RATE_H_

#include "time.h"

namespace ed
{

class Rate
{

public:

    Rate(double frequency) :
        start_(Time::now()),
        expected_cycle_time_(Time(1.0 / frequency)),
        actual_cycle_time_(Time(0))
    {}

    bool sleep()
    {
        Time expected_end = start_ + expected_cycle_time_;

        Time actual_end = Time::now();

        // detect backward jumps in time
        if (actual_end < start_)
            expected_end = actual_end + expected_cycle_time_;

        //calculate the time we'll sleep for
        Time sleep_time = expected_end - actual_end;

        //set the actual amount of time the loop took in case the user wants to know
        actual_cycle_time_ = actual_end - start_;

        //make sure to reset our start time
        start_ = expected_end;

        //if we've taken too much time we won't sleep
        if(sleep_time < Time(0))
        {
            // if we've jumped forward in time, or the loop has taken more than a full extra cycle, reset our cycle
            if (actual_end > expected_end + expected_cycle_time_)
                start_ = actual_end;

            return true;
        }

        return sleep_time.sleep();
    }

private:

    Time start_, expected_cycle_time_, actual_cycle_time_;

};

} // end namespace ed

#endif
