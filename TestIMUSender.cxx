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

#include <string>
#include <cstdlib>
#include <iostream>

#include <boost/thread/thread.hpp>

int main(int argc, char* argv[])
{
  try
    {
      SmallPacketSender sender;
      InsPVA insPVA;
      insPVA.ins_status = 0;
      insPVA.message_id = 508;
      insPVA.milliseconds = 11111;
      insPVA.week_number = 22;
      for (int i = 0; i < 3; ++i) {
          insPVA.Eulr[i] = i;
          insPVA.V[i] = i + 3;
      }
      insPVA.LLH[0] = 31.59971848;
      insPVA.LLH[1] = 120.7682072;
      insPVA.LLH[2] = 18.8910392;

      while (true)
        {
          ++insPVA.milliseconds;
        sender.pumpPacket<InsPVA>(insPVA);
        if ((sender.packetCount() % 500) == 0)
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
