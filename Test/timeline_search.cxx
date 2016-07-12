#include "type_defs.h"
#include <iostream>
#include <vector>
#include <boost/random.hpp>
#include <iomanip>
#include <boost/date_time.hpp>
#include <glog/logging.h>
#include <boost/chrono.hpp>
#include <cmath>

using namespace std;
using namespace boost::posix_time;
using namespace boost::gregorian;

std::vector<boost::shared_ptr<PoseTransform> > transforms;
vector<vector<boost::shared_ptr<PoseTransform>>> timeline;
size_t n = 0;
ptime startTime;
double interval;

void addTransform(boost::shared_ptr<PoseTransform> trans) {
    if (timeline.size() == 0) {
        startTime = trans->timestamp;
        timeline.push_back(vector<boost::shared_ptr<PoseTransform>>());
        timeline.back().push_back(trans);
    } else if (timeline.size() == 1) {
        interval = (trans->timestamp - timeline[0][0]->timestamp).total_microseconds() * 0.95;
        timeline.push_back(vector<boost::shared_ptr<PoseTransform>>());
        timeline.back().push_back(trans);
    } else {
        int index = (trans->timestamp - startTime).total_microseconds() / interval;
        while (timeline.size() <= index) {
            timeline.push_back(vector<boost::shared_ptr<PoseTransform>>());
        }
        timeline[index].push_back(trans);
    }
}

inline boost::shared_ptr<PoseTransform> searchVec(vector<boost::shared_ptr<PoseTransform>>& vec, ptime& t, long& shortest) {
    boost::shared_ptr<PoseTransform> result = vec[0];
    shortest = abs((t - result->timestamp).total_microseconds());
    for (int i = 1; i < vec.size(); ++i) {
        long distance = abs((t - vec[i]->timestamp).total_microseconds());
        if (distance < shortest) {
            result = vec[i];
            shortest = distance;
        }
    }
    return result;
}

boost::shared_ptr<PoseTransform> searchTimeline(ptime& t) {
    boost::shared_ptr<PoseTransform> result;
    long shortest = 0;
    int index = (t - startTime).total_microseconds() / interval;
    if (timeline[index].size() != 0) {
        result = searchVec(timeline[index], t, shortest);
        if(index > 0 && timeline[index - 1].size() > 0) {
            long distance = 0;
            boost::shared_ptr<PoseTransform> anotherPossible = searchVec(timeline[index - 1], t, distance);
            if (distance < shortest) result = anotherPossible;
        }
        return result;
    }
    int back = (index++) + 1;
    while (timeline[back].size() == 0) ++back;
    result = searchVec(timeline[back], t, shortest);
    while (timeline[index].size() == 0) --index;
    long distance = 0;
    boost::shared_ptr<PoseTransform> anotherPossible = searchVec(timeline[index], t, distance);
    if (distance < shortest) result = anotherPossible;
    return result;
}

int test() {
    timeline.clear();
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
        addTransform(trans);
        lasttime = now;
    }
    boost::random::uniform_real_distribution<double> g2(0, 1e6 * 1e4);
    double target = g2(rng);
    ptime tt = ptime(epoch, time_duration(0,0,0,target));
    boost::chrono::thread_clock::time_point start=boost::chrono::thread_clock::now();
    auto trans = searchTimeline(tt);
    boost::chrono::thread_clock::time_point end=boost::chrono::thread_clock::now();
    long ttt = tt.time_of_day().total_microseconds() / 1e3;
    long result = trans->timestamp.time_of_day().total_microseconds() / 1e3;
//    LOG(INFO) <<'\n' << setprecision(15) <<"target : " << ttt
//         << "\nsearch result: " << result
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
