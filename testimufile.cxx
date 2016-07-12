#include "type_defs.h"
#include <iostream>
#include <fstream>

int main(int argc, char *argv[])
{
    std::ifstream ifs("/tmp/imu.bin");
    IMUData data;
    while(ifs){
        ifs.read((char*)&data, sizeof(IMUData));
        std::cout << data.timestamp << '\t'
                  << data.T[0] << '\t'
                  << data.T[1] << '\t'
                  << data.T[2] << '\n';
    }
    return 0;
}
