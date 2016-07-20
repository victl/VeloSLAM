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
  Module:    INSSource.cxx

  Copyright (c) Zou Lu
  All rights reserved.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "INSSource.h"

#include <boost/smart_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>

#include <queue>
#include <deque>
#include <limits>
#include <fstream>
#include <cassert>
#include "CoordiTran.h"
#include "TransformManager.h"
#include "TimeSolver.h"

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
  class INSFileWriter
  {
  public:

    void threadLoop()
    {
      boost::shared_ptr<PoseTransform> data = 0;
      while (this->dataQueue->dequeue(data))
        {
        this->insWriter << *data;
        }
    }

    void start(const std::string& filename)
    {
      if (this->thread_)
        {
        return;
        }
      if(this->fileName != filename){
          this->fileName = filename;
          if(this->insWriter.is_open())
              this->insWriter.close();
      }

      if (! this->insWriter.is_open())
        {
          this->insWriter.open(filename);
        if (! this->insWriter)
          {
          std::cerr << "Failed to write to INS file: " << filename << std::endl;
          return;
          }
        }

      this->dataQueue.reset(new SynchronizedQueue<boost::shared_ptr<PoseTransform> >);
      this->thread_ = boost::shared_ptr<boost::thread>(
        new boost::thread(boost::bind(&INSFileWriter::threadLoop, this)));
    }

    void stop()
    {
      if (this->thread_)
        {
        this->dataQueue->stopQueue();
        this->thread_->join();
        this->thread_.reset();
        this->dataQueue.reset();
        }
    }

    void enqueue(boost::shared_ptr<PoseTransform> data)
    {
        if (this->thread_)
            this->dataQueue->enqueue(data);
    }

    void setFileName(const std::string& filename) {
        if (filename == this->fileName) return;
        if (filename.length() != 0) {
            this->fileName = filename;
            stop();
            start(filename);
        } else {
            stop();
        }
    }

  private:
    std::ofstream insWriter;
    std::string fileName;
    boost::shared_ptr<boost::thread> thread_;
    boost::shared_ptr<SynchronizedQueue<boost::shared_ptr<PoseTransform> > > dataQueue;
  };

//----------------------------------------------------------------------------
class PacketConsumer
{
public:

  PacketConsumer()
      : writer(0)
  {
  }

  void handleData(const unsigned char* data, unsigned int length)
  {
    const unsigned short * msg_ptr = reinterpret_cast<const unsigned short *>(data);
    switch (*msg_ptr) {
    case INSPVA:
    {
        insPVA = reinterpret_cast<InsPVA const*>(data);
        boost::shared_ptr<PoseTransform> trans = calcTransform(insPVA);
        if (writer) {
            writer->enqueue(trans);
        }
        if(transformMgr) {
            transformMgr->addTransform(trans);
        }
        break;
    }
    case RAWINS:
        // FIXME: do some useful stuffs
        break;
    case BESTGPSPOS:
        // FIXME: do some useful stuffs
        break;
    default:
        break;
    }
  }

  void threadLoop()
  {
    std::string* packet = 0;
    while (this->packetQueue->dequeue(packet))
      {
      this->handleData(reinterpret_cast<const unsigned char*>(packet->c_str()), packet->length());
      delete packet;
      }
  }

  void start()
  {
    if (this->thread_)
      {
      return;
      }

    this->packetQueue.reset(new SynchronizedQueue<std::string*>);
    this->thread_ = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketConsumer::threadLoop, this)));
  }

  void stop()
  {
      if (writer) {
          writer->stop();
      }
    if (this->thread_)
      {
      this->packetQueue->stopQueue();
      this->thread_->join();
      this->thread_.reset();
      this->packetQueue.reset();
      }
  }

  void enqueue(std::string* packet)
  {
    this->packetQueue->enqueue(packet);
  }

  void setTransformManager(boost::shared_ptr<TransformManager> mgr){
      this->transformMgr = mgr;
  }

  void setTimeSolver(boost::shared_ptr<TimeSolver> solver) {
      this->timeSolver = solver;
  }

  void setWriter(INSFileWriter* w) {
      this->writer = w;
  }

  void setOrigin(double org[3]) {
      this->ORIG_XYZ[0] = org[0];
      this->ORIG_XYZ[1] = org[1];
      this->ORIG_XYZ[2] = org[2];
  }

protected:

  boost::shared_ptr<PoseTransform> calcTransform(InsPVA const* data) {
      double input[3] = {TO_RADIUS(data->LLH[0]), TO_RADIUS(data->LLH[1]), data->LLH[2]};
      double output[3] = {0};
      llh2enu(input, ORIG_XYZ, output);
      boost::shared_ptr<PoseTransform> trans(new PoseTransform);
      trans->T[0] = output[0];
      trans->T[1] = output[1];
      trans->T[2] = output[2];
      trans->R[0] = data->Eulr[0];
      trans->R[1] = data->Eulr[1];
      trans->R[2] = data->Eulr[2];
      trans->V[0] = data->V[0];
      trans->V[1] = data->V[1];
      trans->V[2] = data->V[2];
      trans->week_number = data->week_number;
      trans->milliseconds = data->milliseconds;
      trans->week_number_pos = data->week_number_pos;
      trans->seconds_pos = data->seconds_pos;
      assert(timeSolver);
      trans->timestamp = timeSolver->calcTimestamp(data);
      return trans;
  }

  InsPVA const* insPVA;
  RawINS const* rawINS;
  BestGPSPos const*  bestGPSPos;
  //This is the origin location (in meters) given by Wang Shi Yao.
  //FIXME: this value should be "static const", but due to the library interface
  //in 'CoordiTran.h' dose not support it, hence ommited
  double ORIG_XYZ[3] = {-2781621.9891904, 4672106.75052387, 18.8910392};

  boost::shared_ptr<SynchronizedQueue<std::string*> > packetQueue;

  boost::shared_ptr<boost::thread> thread_;

  boost::shared_ptr<TransformManager> transformMgr;
  boost::shared_ptr<TimeSolver> timeSolver;
  INSFileWriter* writer;
};

class PacketNetworkSource;
//----------------------------------------------------------------------------
class PacketReceiver
{
public:
  PacketReceiver(boost::asio::io_service& io, int _port, PacketNetworkSource* _parent)
  : port_(_port),
    socket_(io, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), _port)),
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
    this->socket_.async_receive(boost::asio::buffer(this->RXBuffer, 1500),
      boost::bind(&PacketReceiver::socketCallback, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }

  void socketCallback(const boost::system::error_code& error, std::size_t numberOfBytes);

private:
  int port_;
  boost::asio::ip::udp::socket socket_;
  PacketNetworkSource* parent_;
  char RXBuffer[1500];

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
  PacketNetworkSource(boost::shared_ptr<PacketConsumer> _consumer, boost::shared_ptr<INSFileWriter> _writer, int port)
    : iOService()
    , thread_()
    , receiver()
    , consumer(_consumer)
    , writer(_writer)
    , imuPort(port)
    , dummyWork(new boost::asio::io_service::work(this->iOService))
  {
      consumer->setWriter(writer.get());
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
    if( this->consumer )
      {
      this->consumer->enqueue(packet);
      }
  }

  void start()
  {
    if(!this->thread_)
      {
        consumer->start();
      std::cout << "Start listen for INS packets on port: " << imuPort << std::endl;
      this->thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &this->iOService)));
      // Create work
      this->receiver = boost::shared_ptr<PacketReceiver>(new PacketReceiver(this->iOService, imuPort, this));
      }
  }

  void stop()
  {
    // Kill the receivers
    this->receiver.reset();
      consumer->stop();

  }

  boost::asio::io_service iOService;
  boost::shared_ptr<boost::thread> thread_;

  boost::shared_ptr<PacketReceiver> receiver;

  boost::shared_ptr<PacketConsumer> consumer;
  boost::shared_ptr<INSFileWriter> writer;
  int imuPort;

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

  std::string* packet = new std::string(this->RXBuffer, numberOfBytes);
  this->parent_->queuePackets(packet);

  this->startReceive();
}

} // end namespace

//----------------------------------------------------------------------------
class INSSource::vsInternal
{
public:

  vsInternal(int port)
      : consumer(new PacketConsumer)
      , writer(new INSFileWriter)
      , networkSource(this->consumer, this->writer, port)
  {
  }

  ~vsInternal()
  {
  }

  boost::shared_ptr<PacketConsumer> consumer;
  boost::shared_ptr<INSFileWriter> writer;
  PacketNetworkSource networkSource;
};

//----------------------------------------------------------------------------
INSSource::INSSource(int port)
    : internal_(new vsInternal(port))
{
}

//----------------------------------------------------------------------------
INSSource::~INSSource()
{
  this->stop();
  delete this->internal_;
}


//-----------------------------------------------------------------------------
void INSSource::setOutputFile(const std::string& filename)
{
  this->internal_->writer->setFileName(filename);
}

void INSSource::setTransformManager(boost::shared_ptr<TransformManager> mgr)
{
    this->internal_->consumer->setTransformManager(mgr);
}

void INSSource::setTimeSolver(boost::shared_ptr<TimeSolver> solver)
{
    this->internal_->consumer->setTimeSolver(solver);
}

void INSSource::setOrigin(double org[])
{
    this->internal_->consumer->setOrigin(org);
}

//----------------------------------------------------------------------------
void INSSource::start()
{
  this->internal_->networkSource.start();
}

//----------------------------------------------------------------------------
void INSSource::stop()
{
  this->internal_->networkSource.stop();
}


