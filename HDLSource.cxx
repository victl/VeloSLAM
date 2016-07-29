// Copyright 2013 Velodyne Acoustics, Inc.
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

  Program:   Visualization Toolkit
  Module:    vtkVelodyneHDLSource.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "HDLSource.h"
#include "HDLParser.h"
#include "HDLManager.h"
#include "TimeSolver.h"

#include <boost/smart_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

#include <queue>
#include <deque>
#include <limits>
#include <fstream>

//----------------------------------------------------------------------------
namespace
{

  template<typename T>
  class SynchronizedQueue
  {
    public:

      SynchronizedQueue () :
        queue_(), mutex_(), cond_(), request_to_end_(false), enqueue_data_(true) { }

      void
      enqueue (const T& data)
      {
        boost::unique_lock<boost::mutex> lock (mutex_);

        if (enqueue_data_)
        {
          queue_.push (data);
          cond_.notify_one ();
        }
      }

      bool
      dequeue (T& result)
      {
        boost::unique_lock<boost::mutex> lock (mutex_);

        while (queue_.empty () && (!request_to_end_))
        {
          cond_.wait (lock);
        }

        if (request_to_end_)
        {
          doEndActions ();
          return false;
        }

        result = queue_.front ();
        queue_.pop ();

        return true;
      }

      void
      stopQueue ()
      {
        boost::unique_lock<boost::mutex> lock (mutex_);
        request_to_end_ = true;
        cond_.notify_one ();
      }

      unsigned int
      size ()
      {
        boost::unique_lock<boost::mutex> lock (mutex_);
        return static_cast<unsigned int> (queue_.size ());
      }

      bool
      isEmpty () const
      {
        boost::unique_lock<boost::mutex> lock (mutex_);
        return (queue_.empty ());
      }

    private:
      void
      doEndActions ()
      {
        enqueue_data_ = false;

        while (!queue_.empty ())
        {
          queue_.pop ();
        }
      }

      std::queue<T> queue_;              // Use STL queue to store data
      mutable boost::mutex mutex_;       // The mutex to synchronise on
      boost::condition_variable cond_;   // The condition to wait for

      bool request_to_end_;
      bool enqueue_data_;
  };

  //----------------------------------------------------------------------------
  // This class is useless in the new design.
  // Time info is directly coded into pcap file
  // Kept here for reference
  class TimeFileWriter
  {
  public:

    void ThreadLoop()
    {
      boost::posix_time::ptime* newtime = 0;
      while (this->timesQueue->dequeue(newtime))
        {
        ofs << *newtime << std::endl;
        delete newtime;
        }
    }

    void Start(const std::string& filename)
    {
      if (this->Thread)
        {
        return;
        }

      if (ofs.is_open())
        {
          ofs.close();
          ofs.open(filename);
          if(! this->ofs.is_open()) {
              std::cerr << "Failed to open packet file: " << filename << std::endl;
              return;
          }
        }
      this->timesQueue.reset(new SynchronizedQueue<boost::posix_time::ptime*>);
      this->Thread = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&TimeFileWriter::ThreadLoop, this)));
    }

    void Stop()
    {
      if (this->Thread)
        {
        this->timesQueue->stopQueue();
        this->Thread->join();
        this->Thread.reset();
        this->timesQueue.reset();
        }
    }

    void Enqueue(boost::posix_time::ptime* newtime)
    {
      this->timesQueue->enqueue(newtime);
    }

    void Close()
    {
      this->ofs.close();
    }

  private:
    std::ofstream ofs;
    boost::shared_ptr<boost::thread> Thread;
    boost::shared_ptr<SynchronizedQueue<boost::posix_time::ptime*> > timesQueue;
  };

//----------------------------------------------------------------------------
class PacketConsumer
{
public:

  PacketConsumer()
      : hdlParser(new HDLParser)
      , hdlMgr(0)
      , timeSolver(0)
  {

  }

  void handleSensorData(const unsigned char* data, unsigned int length)
  {
      if (length != 1206) {
          return;
      }
    boost::lock_guard<boost::mutex> lock(this->parserMutex);

    uint32_t gpstimestamp = *(reinterpret_cast<const uint32_t*>(data + 1200));
    boost::posix_time::ptime timestamp = timeSolver->calcTimestamp(gpstimestamp);

    this->hdlParser->processHDLPacket(const_cast<unsigned char*>(data), length, timestamp);
    if (this->hdlParser->getAllFrames().size())
      {
      this->hdlMgr->addFrame(this->hdlParser->getAllFrames().back());
      this->hdlParser->clearAllFrames();
      }
  }

  void threadLoop()
  {
    std::string* packet = 0;
    while (this->packetsQueue->dequeue(packet))
      {
      this->handleSensorData(reinterpret_cast<const unsigned char*>(packet->c_str()), packet->length());
      delete packet;
      }
  }

  void start()
  {
    if (this->thread_)
      {
      return;
      }

    this->packetsQueue.reset(new SynchronizedQueue<std::string*>);
    this->thread_ = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketConsumer::threadLoop, this)));
  }

  void stop()
  {
    if (this->thread_)
      {
      this->packetsQueue->stopQueue();
      this->thread_->join();
      this->thread_.reset();
      this->packetsQueue.reset();
      }
  }

  void enqueue(std::string* packet)
  {
    this->packetsQueue->enqueue(packet);
  }

  boost::shared_ptr<HDLParser> getParser()
  {
    return this->hdlParser;
  }

  // Hold this when running parser code code or modifying its internals
  boost::mutex parserMutex;
  HDLManager* hdlMgr;
  boost::shared_ptr<TimeSolver> timeSolver;
  boost::shared_ptr<HDLParser> hdlParser;

protected:


  boost::shared_ptr<SynchronizedQueue<std::string*> > packetsQueue;

  boost::shared_ptr<boost::thread> thread_;
};


//----------------------------------------------------------------------------
// With the new design of the whole library, packets will be written by HDLManager.
// So this class is deprecated. But it's useful in that is records raw (unprocessed)
// pcap file. Might be used some time.
class PacketFileWriter
{
public:

  void threadLoop()
  {
    std::string* packet = 0;
    while (this->Packets->dequeue(packet))
      {
      this->PacketWriter.writePacket(reinterpret_cast<const unsigned char*>(packet->c_str()), packet->length());

      delete packet;
      }
  }

  void Start(const std::string& filename)
  {
    if (this->Thread)
      {
      return;
      }

    if (this->PacketWriter.GetFileName() != filename)
      {
      this->PacketWriter.close();
      }

    if (!this->PacketWriter.isOpen())
      {
      if (!this->PacketWriter.open(filename))
        {
        std::cerr << "Failed to open packet file: " << filename << std::endl;
        return;
        }
      }

    this->Packets.reset(new SynchronizedQueue<std::string*>);
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketFileWriter::threadLoop, this)));
  }

  void Stop()
  {
    if (this->Thread)
      {
      this->Packets->stopQueue();
      this->Thread->join();
      this->Thread.reset();
      this->Packets.reset();
      }
  }

  void Enqueue(std::string* packet)
  {
    this->Packets->enqueue(packet);
  }

  bool IsOpen()
  {
    return this->PacketWriter.isOpen();
  }

  void Close()
  {
    this->PacketWriter.close();
  }

private:
  vtkPacketFileWriter PacketWriter;
  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;
};


class PacketNetworkSource;
//----------------------------------------------------------------------------
class PacketReceiver
{
public:
  PacketReceiver(boost::asio::io_service& io, int _port, PacketNetworkSource* _parent)
  : port(_port)
  , socket_(io, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), _port)),
    parent_(_parent),
    isReceiving(true),
    shouldStop(false)
  {
    this->startReceive();
  }

  ~PacketReceiver()
  {
    this->socket_.cancel();

      {
      boost::unique_lock<boost::mutex> guard(this->isReceivingMtx);
      this->shouldStop = true;
      while(this->isReceiving)
        {
        this->isReceivingCond.wait(guard);
        }
      }
  }

  void startReceive()
  {
      {
      boost::lock_guard<boost::mutex> guard(this->isReceivingMtx);
      this->isReceiving = true;
      }

    // expecting exactly 1206 bytes, using a larger buffer so that if a
    // larger packet arrives unexpectedly we'll notice it.
    this->socket_.async_receive(boost::asio::buffer(this->rxBuffer, 1500),
      boost::bind(&PacketReceiver::socketCallback, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }

  void socketCallback(const boost::system::error_code& error, std::size_t numberOfBytes);

private:
  int port;
  boost::asio::ip::udp::socket socket_;
  PacketNetworkSource* parent_;
  char rxBuffer[1500];

  bool isReceiving;
  bool shouldStop;
  boost::mutex isReceivingMtx;
  boost::condition_variable isReceivingCond;
};

//----------------------------------------------------------------------------
// This class is responsible for the IOService and  two PacketReceiver classes
class PacketNetworkSource
{
public:
  PacketNetworkSource(boost::shared_ptr<PacketConsumer> _consumer)
    : iOService()
    , thread_()
    , lidarPortReceiver()
    , positionPortReceiver()
    , hdlConsumer(_consumer)
//    , packetWriter()
    , dummyWork(new boost::asio::io_service::work(this->iOService))
  {
  }

  ~PacketNetworkSource()
  {
    this->stop();

    delete this->dummyWork;

    if(this->thread_)
      {
      this->thread_->join();
      }
  }

  void queuePackets(std::string* packet)
  {
//    std::string* packet2 = 0;
//    if(this->packetWriter)
//      {
//      packet2 = new std::string(*packet);
//      }

    if( this->hdlConsumer )
      {
      this->hdlConsumer->enqueue(packet);
      }

//    if (this->Writer)
//      {
//      this->Writer->Enqueue(packet2);
//      }
  }

  void start()
  {
      if (this->thread_)
          return;
    if(!this->thread_)
      {
      std::cout << "Start listen for HDL packets on port 2368" << std::endl;
      this->thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &this->iOService)));
      }

    if(this->lidarPortReceiver)
      {
      assert(this->positionPortReceiver);
      return;
      }

    // Create work
    this->lidarPortReceiver = boost::shared_ptr<PacketReceiver>(new PacketReceiver(this->iOService, 2368, this));
    this->positionPortReceiver = boost::shared_ptr<PacketReceiver>(new PacketReceiver(this->iOService, 8308, this));
  }

  void stop()
  {
    // Kill the receivers
    this->positionPortReceiver.reset();
    this->lidarPortReceiver.reset();

  }

  boost::asio::io_service iOService;
  boost::shared_ptr<boost::thread> thread_;

  boost::shared_ptr<PacketReceiver> lidarPortReceiver;
  boost::shared_ptr<PacketReceiver> positionPortReceiver;

  boost::shared_ptr<PacketConsumer> hdlConsumer;
//  boost::shared_ptr<PacketFileWriter> packetWriter;

  boost::asio::io_service::work* dummyWork;
};

void PacketReceiver::socketCallback(const boost::system::error_code& error, std::size_t numberOfBytes)
{
//  std::cout << "CALLBACK " << this->Port << std::endl;
  if(error || this->shouldStop)
    {
    // This is called on cancel
    // TODO: Check other error codes
      {
      boost::lock_guard<boost::mutex> guard(this->isReceivingMtx);
        this->isReceiving = false;
      }
      this->isReceivingCond.notify_one();

    return;
    }

  std::string* packet = new std::string(this->rxBuffer, numberOfBytes);
  this->parent_->queuePackets(packet);

  this->startReceive();
}

} // end namespace

//----------------------------------------------------------------------------
class HDLSource::vsInternal
{
public:

  vsInternal()
      : hdlConsumer(new PacketConsumer)
//                  packetWriter(new PacketFileWriter),
      , networkSource(this->hdlConsumer)
  {
  }

  ~vsInternal()
  {
  }

  boost::shared_ptr<PacketConsumer> hdlConsumer;
//  boost::shared_ptr<PacketFileWriter> packetWriter;
  PacketNetworkSource networkSource;
};

//----------------------------------------------------------------------------
HDLSource::HDLSource(int _port)
    : sensorPort(_port)
{
  this->internal_ = new vsInternal;
}

//----------------------------------------------------------------------------
HDLSource::~HDLSource()
{
  this->stop();
  delete this->internal_;
}

//-----------------------------------------------------------------------------
void HDLSource::setOutputFile(const std::string& filename)
{
  if (filename == this->outputFile)
    {
    return;
    }

//  this->internal_->packetWriter->Close();
  this->outputFile = filename;
}

//-----------------------------------------------------------------------------
int HDLSource::getNumberOfChannels()
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  return this->internal_->hdlConsumer->getParser()->getNumberOfChannels();
}

//-----------------------------------------------------------------------------
const std::string& HDLSource::getCorrectionsFile()
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  return this->internal_->hdlConsumer->getParser()->getCorrectionsFile();
}

//-----------------------------------------------------------------------------
void HDLSource::setCorrectionsFile(const std::string& filename)
{
  if (filename == this->getCorrectionsFile())
    {
    return;
    }

  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  this->internal_->hdlConsumer->getParser()->setCorrectionsFile(filename);
}

//-----------------------------------------------------------------------------
void HDLSource::setLaserSelection(int LaserSelection[64])
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  this->internal_->hdlConsumer->getParser()->setLaserSelection(LaserSelection);
}

//-----------------------------------------------------------------------------
void HDLSource::getLaserSelection(int LaserSelection[64])
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  this->internal_->hdlConsumer->getParser()->getLaserSelection(LaserSelection);
}

//-----------------------------------------------------------------------------
void HDLSource::setCropReturns(int cr)
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  this->internal_->hdlConsumer->getParser()->setCropReturns(cr);
}

//-----------------------------------------------------------------------------
void HDLSource::setCropInside(int ci)
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  this->internal_->hdlConsumer->getParser()->setCropInside(ci);
}

//-----------------------------------------------------------------------------
void HDLSource::setCropRegion(double r[6])
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  this->internal_->hdlConsumer->getParser()->setCropRegion(r);
}

//-----------------------------------------------------------------------------
void HDLSource::setCropRegion(double xmin, double xmax,
                                         double ymin, double ymax,
                                         double zmin, double zmax)
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  this->internal_->hdlConsumer->getParser()->setCropRegion(xmin, xmax,
                                                       ymin, ymax,
                                                       zmin, zmax);
}

//-----------------------------------------------------------------------------
void HDLSource::getVerticalCorrections(double VerticalCorrections[64])
{
  boost::lock_guard<boost::mutex> lock(this->internal_->hdlConsumer->parserMutex);

  this->internal_->hdlConsumer->getParser()->getVerticalCorrections(VerticalCorrections);
}

//-----------------------------------------------------------------------------
unsigned int HDLSource::getDualReturnFilter() const
{
  return this->internal_->hdlConsumer->getParser()->getDualReturnFilter();
}

//-----------------------------------------------------------------------------
void HDLSource::setDualReturnFilter(unsigned int filter)
{
  this->internal_->hdlConsumer->getParser()->setDualReturnFilter(filter);
}

void HDLSource::setHDLManager(HDLManager *hp)
{
    this->internal_->hdlConsumer->hdlMgr = hp;
}

void HDLSource::setTimeSolver(boost::shared_ptr<TimeSolver> solver)
{
    this->internal_->hdlConsumer->timeSolver = solver;
}

void HDLSource::setTransformManager(boost::shared_ptr<TransformManager> mgr)
{
    this->internal_->hdlConsumer->getParser()->setTransformMgr(mgr);
}

boost::shared_ptr<HDLParser> HDLSource::getHDLParser()
{
    return this->internal_->hdlConsumer->getParser();
}

//----------------------------------------------------------------------------
void HDLSource::start()
{
//  if (this->outputFile.length())
//    {
//    this->internal_->packetWriter->Start(this->outputFile);
//    }

//  this->internal_->networkSource.Writer.reset();

//  if (this->internal_->packetWriter->IsOpen())
//    {
//    this->internal_->networkSource.Writer = this->internal_->packetWriter;
//    }

  this->internal_->hdlConsumer->start();

  this->internal_->networkSource.start();
}

//----------------------------------------------------------------------------
void HDLSource::stop()
{
  this->internal_->networkSource.stop();
  this->internal_->hdlConsumer->stop();
//  this->internal_->packetWriter->Stop();
}
