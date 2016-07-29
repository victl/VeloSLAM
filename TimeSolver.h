#ifndef __TIME_SOLVER_H__
#define __TIME_SOLVER_H__

#include "type_defs.h"
#include <boost/date_time.hpp>
#include <boost/shared_ptr.hpp>

class TimeSolver {
public:
    TimeSolver();
    ~TimeSolver();

    ptime calcTimestamp(InsPVA const* data); // INS version
    ptime calcTimestamp(uint32_t microsecToHour); // HDL version

private:
    date epoch;
    ptime hdlHourTime;
    time_duration hdlOffset;
    time_duration insOffset;
    bool hdlInited;
    bool insInited;
    uint32_t lastHdlReport;
    int hoursPerWeek;
};

#endif //__TIME_SOLVER_H__
