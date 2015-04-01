#ifndef ED_TIME_H_
#define ED_TIME_H_

#include <iostream>
#include <sys/time.h>

namespace ed
{

class Time
{

public:

    Time() : secs_(0) {}

    Time(double secs) : secs_(secs) {}

    static Time now()
    {
        struct timeval tp;
        gettimeofday(&tp, NULL);
        return Time(tp.tv_sec + (double) tp.tv_usec / 1000000.0);
    }

    bool operator<(const Time& rhs) const { return secs_ < rhs.secs_; }
    bool operator>(const Time& rhs) const { return secs_ > rhs.secs_; }

    Time operator+(const Time& t) {  return Time(this->seconds() + (+t.seconds())); }
    Time operator-(const Time& t) {  return Time(this->seconds() + (-t.seconds())); }

    friend std::ostream& operator<< (std::ostream& out, const Time& d)
    {
        int isecs = d.secs_;

        int h = isecs / 3600;
        int m = (isecs / 60) % 60;
        int s = isecs % 60;
        int ms = 1000 * (d.secs_ - isecs);

        out << h << ":" << m << ":" << s << ":" << ms;

        return out;
    }

    inline double seconds() const { return secs_; }

    inline bool sleep() const {
        if (secs_ > 0)
            usleep(secs_ * 1000000);
        return secs_ > 0;
    }

private:

    double secs_;

};

} // end namespace ed

#endif
