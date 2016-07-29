#include "type_defs.h"
#include "TransformManager.h"
#include <iostream>
#include <vector>
#include <boost/random.hpp>
#include <iomanip>
#include <boost/date_time.hpp>
#include <glog/logging.h>
#include <boost/chrono.hpp>
#include <cmath>

int test(TransformManager& mgr, boost::random::mt19937& rng) {

    boost::random::uniform_real_distribution<double> g2(0, 1e6 * 1e4);
    double target = g2(rng);
    date epoch(2016,7,4);
    ptime tt = ptime(epoch, time_duration(0,0,0,target));
    double sum = 0;
    for (int i = 0; i < 100; ++i) {
        std::cout << "Running test " << i << " ..." << std::endl;
        PoseTransform* trans = new PoseTransform;
        tt += time_duration(0,0,0,80);
        boost::chrono::thread_clock::time_point start
                = boost::chrono::thread_clock::now();
        mgr.interpolateTransform(tt, trans);
        boost::chrono::thread_clock::time_point end
                = boost::chrono::thread_clock::now();
        tt = trans->timestamp;
        delete trans;
        sum += (end - start).count();
    }
    //long ttt = tt.time_of_day().total_microseconds() / 1e3;
    //long result = trans->timestamp.time_of_day().total_microseconds() / 1e3;
//    LOG(INFO) <<'\n' << setprecision(15) <<"target : " << ttt
//         << "\nsearch result: " << result
//         << "\n Time used: " << end - start << endl;
    return sum / 100;
}

int main() {
    // initialize data
    TransformManager mgr;
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
        mgr.addTransform(trans);
        lasttime = now;
    }
    // test performance
    std::cout << "average: " << test(mgr, rng) << std::endl;
}
