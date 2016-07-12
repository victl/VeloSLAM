#include "TimeSolver.h"

TimeSolver::TimeSolver()
    : epoch(day_clock::local_day().year(),1,1)
    , hdlHourTime(microsec_clock::local_time())
    , hdlInited(false)
    , insInited(false)
    , lastHdlReport(0)
    , hoursPerWeek(168)  // 7 * 24 hours
{
    time_duration tv = hdlHourTime.time_of_day();
    time_duration diff(0, tv.minutes() , tv.seconds() , tv.fractional_seconds());
    hdlHourTime -= diff;
}

TimeSolver::~TimeSolver()
{
}

ptime TimeSolver::calcTimestamp(InsPVA const* data)
{
    ptime p(epoch,
            time_duration(data->week_number * hoursPerWeek, 0, 0, double(data->milliseconds) * 1e3));
    if (! insInited) {
        this->insOffset = ptime(microsec_clock::local_time()) - p;
    }
    ptime insTime(epoch,
            time_duration(data->week_number_pos * hoursPerWeek, 0, 0, double(data->seconds_pos) * 1e6));
    time_duration diff = insTime - p;
    return p + this->insOffset + diff;
}

ptime TimeSolver::calcTimestamp(uint32_t microsecToHour)
{
    if (! hdlInited) { // indicate we have started to receive HDL info
        ptime nowtime(microsec_clock::local_time());
        date today = nowtime.date();
        time_duration hoursToDayBegin(nowtime.time_of_day().hours(),0,0,0);
        this->hdlHourTime = ptime(today, hoursToDayBegin);
        this->hdlOffset = nowtime - this->hdlHourTime - time_duration(0,0,0,microsecToHour);
        hdlInited = true;
    }
    if (this->lastHdlReport > microsecToHour) { // indicate one hour time need to wrap up
        this->hdlHourTime += time_duration(1,0,0,0);
    }
    this->lastHdlReport = microsecToHour;
    ptime timestamp = this->hdlHourTime + this->hdlOffset + time_duration(0,0,0,microsecToHour);
    return timestamp;
}
