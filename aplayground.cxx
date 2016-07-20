#include <boost/date_time.hpp>
#include <iostream>

using namespace boost::posix_time;
using namespace std;

int main(int argc, char* argv[]) {
    ptime now = boost::posix_time::microsec_clock::local_time();
    boost::gregorian::date today = now.date();
    cout << today.week_number() << endl;
    ptime beginOfWeek(boost::gregorian::date(today - boost::gregorian::days(to_tm(today).tm_wday)));
    time_duration millis = now - beginOfWeek;
    cout << "milli: " << millis.total_milliseconds() << endl;
}
