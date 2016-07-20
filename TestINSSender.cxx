// Copyright 2016 Zou Lu <victl@163.com>.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*=========================================================================

  Program:   VeloSLAM
  Module:    TestIMUSender.cxx

  Copyright (c) Zou Lu
  All rights reserved.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME PacketFileSender -
// .SECTION Description
// This program repeatedly form dummy IMUData and sends the packets using UDP.

#include "SmallPacketSender.h"
#include "type_defs.h"
#include "CoordiTran.h"

#include <string>
#include <cstdlib>
#include <iostream>
#include <fstream>

#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <time.h>

using namespace boost::gregorian;
using namespace boost::posix_time;

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cout << "Usage: ./TestINSSender <ins_record_file>.txt" << std::endl;
        exit(-1);
    }
  try
    {
      SmallPacketSender sender("127.0.0.1", 6777);
      std::ifstream ifs(argv[1]);
      InsPVA pva;
      std::memset((void*)&pva, 0, sizeof(pva));
      pva.message_id = 508;
      timeval tv;
      ptime t;
      double originLLH[3] = {39.8569901,116.1736406,89.09288895}; //yby
      double originXYZ[3];
      llh2xyz(originLLH, originXYZ);
      double enu[3] = {0};
      while (ifs) {
          ifs >> enu[0] >> enu[1] >> pva.Eulr[2] >> pva.Eulr[0] >> pva.Eulr[1] >> tv.tv_sec >> tv.tv_usec;
          enu2llh(enu, originXYZ, pva.LLH);
          timevalToPtime(tv, t);
          ptimeToWeekMilli(t, pva.week_number, pva.milliseconds);
          pva.week_number_pos = pva.week_number;
          pva.seconds_pos = pva.milliseconds / 1000.0f;
          sender.pumpPacket<InsPVA>(pva);
          if ((sender.packetCount() % 100) == 0)
            {
            printf("total sent packets: %lu\n", sender.packetCount());
            }
          boost::this_thread::sleep(boost::posix_time::milliseconds(10));
      }
    }
  catch( std::exception & e )
    {
    std::cout << "Caught Exception: " << e.what() << std::endl;
    return 1;
    }

  return 0;
}
