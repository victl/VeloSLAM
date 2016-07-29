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

#ifndef __SmallPacketSender_H__
#define __SmallPacketSender_H__

#include <string>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/static_assert.hpp>

class SmallPacketSender
{
public:
  SmallPacketSender(std::string destinationio = "127.0.0.1",
                 int port = 1777);
  ~SmallPacketSender();

  template <typename T>
  void pumpPacket(T data);
  size_t packetCount() const;

private:
  class vsInternal;
  vsInternal* Internal;

};

//-----------------------------------------------------------------------------
class SmallPacketSender::vsInternal
{
public:
  vsInternal(std::string destinationIp,
             int port) :
    DataSocket(0),
    PacketCount(0),
    DataEndpoint(boost::asio::ip::address_v4::from_string(destinationIp), port)
  {
  }

  boost::asio::ip::udp::socket* DataSocket;

  size_t PacketCount;
  boost::asio::ip::udp::endpoint DataEndpoint;
  boost::asio::io_service IOService;
  };

//-----------------------------------------------------------------------------
// This function does not handle large data (> 1200 bytes)
template <typename T>
void SmallPacketSender::pumpPacket(T data)
{
    //PLEASE make sure sizeof(T) <= 1200, or you'll get a compiler time error
    BOOST_STATIC_ASSERT(sizeof(T) <= 1200);
  const unsigned char* rawdata = reinterpret_cast<const unsigned char *>(&data);
  unsigned int dataLength = sizeof(T);
    ++this->Internal->PacketCount;
    size_t bytesSent = this->Internal->DataSocket->send_to(boost::asio::buffer(rawdata, dataLength),
                                                            this->Internal->DataEndpoint);
}

#endif // __SmallPacketSender_H__
