#include "type_defs.h"
#include <iostream>
#include <vector>
#include <boost/random.hpp>
#include <iomanip>
#include <boost/date_time.hpp>
#include <glog/logging.h>
#include <boost/chrono.hpp>

using namespace std;
using namespace boost::posix_time;
using namespace boost::gregorian;

std::vector<boost::shared_ptr<PoseTransform> > transforms;
size_t n = 0;

//std::vector<double>::iterator findNearest(double &t, size_t a = 0, size_t b = 0)
//{
//    ++n;
//    if (b == 0) {
//        b = transforms.size();
//        if (b != 0) --b;
//    }
//    double av = transforms[a];
//    double bv = transforms[b];
//    if (b == a + 1) {
//        return (t - av < bv - t) ?
//                    transforms.begin() + a : transforms.begin() + b;
//    }

//    while (b > a + 1) {
//        double t_avg = (bv - av) / (b - a + 1);
//        int offset = (t - av) / t_avg;
//        size_t c = a + offset;
//        if (c > b - 1) {
//            c = b - 1;
//        }
//        if ( c <= a) {
//            /*debug*/ std::cout << "This could not be possible" << std::endl;
//            c = a + 1;
//        }
//        double cv = transforms[c];
//        if (cv > t) {
//            return findNearest(t, a, c);
//        } else if (transforms[c] < t) {
//            return findNearest(t, c, b);
//        } else {
//            return transforms.begin() + c;
//        }
//    }
//}

std::vector<boost::shared_ptr<PoseTransform> >::iterator findIter(ptime &t, size_t a = 0, size_t b = 0)
{
    ++n;
    if (b == 0) {
        b = transforms.size();
        if (b != 0) --b;
    }
    if (b == a + 1) {
        return (t - transforms[a]->timestamp < transforms[b]->timestamp - t) ?
                    transforms.begin() + a : transforms.begin() + b;
    }
    long av = (transforms[a]->timestamp).time_of_day().total_microseconds();
    long bv = (transforms[b]->timestamp).time_of_day().total_microseconds();
    long tv = t.time_of_day().total_microseconds();
    while (b > a + 1) {
        time_duration t_avg = (transforms[b]->timestamp - transforms[a]->timestamp) / (b - a + 1);
        int offset = (t - transforms[a]->timestamp).total_microseconds() / t_avg.total_microseconds();
        size_t c = a + offset;
        long cv = (transforms[c]->timestamp).time_of_day().total_microseconds();
        if (c > b - 1) {
            c = b - 1;
        }
        if ( c <= a) {
            c = a + 1;
        }
        if (transforms[c]->timestamp > t) {
            return findIter(t, a, c);
        } else if (transforms[c]->timestamp < t) {
            return findIter(t, c, b);
        } else {
            return transforms.begin() + c;
        }
    }
}

int test() {
    n = 0;
    transforms.clear();
    ptime t = microsec_clock::local_time();
    time_duration d = t.time_of_day();
    long seed = long(d.fractional_seconds() * 1e6) % 17;
    boost::random::mt19937 rng(seed);
    boost::random::normal_distribution<double> g(0, 1e3);
    date epoch(2016,7,4);
    double lasttime = 0;
    for (int i = 0; i < 1e6; ++i) {
        boost::shared_ptr<PoseTransform> trans(new PoseTransform);
        double now = lasttime + 1e4 + g(rng);
        trans->timestamp = ptime(epoch, time_duration(0,0,0,now));
        transforms.push_back(trans);
        lasttime = now;
    }
    boost::random::uniform_real_distribution<double> g2(0, 1e6 * 1e4);
    double target = g2(rng);
    ptime tt = ptime(epoch, time_duration(0,0,0,target));
    boost::chrono::thread_clock::time_point start=boost::chrono::thread_clock::now();
    auto iter = findIter(tt);
    boost::chrono::thread_clock::time_point end=boost::chrono::thread_clock::now();
    long ttt = tt.time_of_day().total_microseconds() / 1e3;
    long result = (*iter)->timestamp.time_of_day().total_microseconds() / 1e3;
    long former = (*(iter-1))->timestamp.time_of_day().total_microseconds() / 1e3;
    long follower = (*(iter+1))->timestamp.time_of_day().total_microseconds() / 1e3;
//    LOG(INFO) <<'\n' << setprecision(15) <<"target : " << ttt
//         << "\nsearch result: " << result
//         << "\nFORMER: " << former
//         << "\nFOLLOWER: " << follower
//         << "\nIteration: " << n
//         << "\n Time used: " << end - start << endl;
    return (end - start).count();
}

int main() {
    double sum = 0;
    for (int i = 0; i < 100; ++i) {
        cout << "Running test " << i << " ..." << endl;
        sum += test();
    }
    cout << "average: " << sum / 100 << endl;
}
