#include "VeloSLAM.h"
#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>

using namespace std;

int main() {
    HDLManager hdlMgr;
    hdlMgr.start();
    while (true) {
        int num = hdlMgr.getNumberOfFrames();
        cout << "num of transforms: " << num << endl;
        if (num > 95) {
            hdlMgr.stop();
            break;
        }
        boost::this_thread::sleep_for(boost::chrono::seconds(1));
    }
}
