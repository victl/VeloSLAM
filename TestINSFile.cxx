#include "type_defs.h"
#include <iostream>
#include <fstream>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>

int main(int argc, char *argv[])
{
    std::ifstream ifs("/tmp/imu.bin");
    PoseTransform data;
    while(ifs){
        ifs >> data;
        std::cout << data;
        boost::this_thread::sleep(boost::posix_time::milliseconds(500));
    }
    return 0;
}
