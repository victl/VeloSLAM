#include "SmallPacketSender.h"

//-----------------------------------------------------------------------------
SmallPacketSender::SmallPacketSender(std::string destinationIp,
                               int port) :
  Internal(new SmallPacketSender::vsInternal(destinationIp, port))
{
  this->Internal->DataSocket = new asio::ip::udp::socket(this->Internal->IOService);
  this->Internal->DataSocket->open(this->Internal->DataEndpoint.protocol());
}

//-----------------------------------------------------------------------------
SmallPacketSender::~SmallPacketSender()
{

  delete this->Internal->DataSocket;
  delete this->Internal;
}

//-----------------------------------------------------------------------------
size_t SmallPacketSender::packetCount() const
{
  return this->Internal->PacketCount;
}
