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
  Module:    IMUSource.cxx

  Copyright (c) Zou Lu
  All rights reserved.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
#include "IMUSource.h"

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
class PacketConsumer
{
public:

  PacketConsumer()
      : transformMgr(0)
  {
    this->NewData = false;
    this->MaxNumberOfDatasets = 1;
    this->LastTime = 0.0;
  }

  void handleData(const unsigned char* data, unsigned int length)
  {
    const unsigned short * msg_ptr = reinterpret_cast<const unsigned short *>(data);
    switch (*msg_ptr) {
    case INSPVA:
        insPVA = reinterpret_cast<InsPVA const*>(data);
        this->HandleNewData(calcTransform(insPVA));
        break;
    case RAWIMU:
        // FIXME: do some useful stuffs
        break;
    case BESTGPSPOS:
        // FIXME: do some useful stuffs
        break;
    default:
        break;
    }
  }

  boost::shared_ptr<PoseTransform> GetDatasetForTime(double timeRequest, double& actualTime)
  {
    boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);

    size_t stepIndex = this->GetIndexForTime(timeRequest);
    if (stepIndex < this->Timesteps.size())
      {
      actualTime = this->Timesteps[stepIndex];
      return this->Datasets[stepIndex];
      }
    actualTime = 0;
    return 0;
  }

  std::vector<double> GetTimesteps()
  {
    boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);
    const size_t nTimesteps = this->Timesteps.size();
    std::vector<double> timesteps(nTimesteps, 0);
    for (size_t i = 0; i < nTimesteps; ++i)
      {
      timesteps[i] = this->Timesteps[i];
      }
    return timesteps;
  }


  int GetMaxNumberOfDatasets()
  {
    return this->MaxNumberOfDatasets;
  }

  void SetMaxNumberOfDatasets(int nDatasets)
  {
    boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);
    this->MaxNumberOfDatasets = nDatasets;
    this->UpdateDequeSize();
  }

  bool CheckForNewData()
  {
    boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);
    bool newData = this->NewData;
    this->NewData = false;
    return newData;
  }


  void ThreadLoop()
  {
    std::string* packet = 0;
    while (this->Packets->dequeue(packet))
      {
      this->handleData(reinterpret_cast<const unsigned char*>(packet->c_str()), packet->length());
      delete packet;
      }
  }

  void Start()
  {
    if (this->Thread)
      {
      return;
      }

    this->Packets.reset(new SynchronizedQueue<std::string*>);
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketConsumer::ThreadLoop, this)));
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

  void SetTransformManager(TransformManager* mgr){
      transformMgr = mgr;
  }

protected:

  void UpdateDequeSize()
  {
    if (this->MaxNumberOfDatasets <= 0)
      {
      return;
      }
    while (this->Datasets.size() >= this->MaxNumberOfDatasets)
      {
      this->Datasets.pop_front();
      this->Timesteps.pop_front();
      }
  }

  size_t GetIndexForTime(double time)
  {
    size_t index = 0;
    double minDifference = std::numeric_limits<double>::max();
    const size_t nTimesteps = this->Timesteps.size();
    for (size_t i = 0; i < nTimesteps; ++i)
      {
      double difference = std::abs(this->Timesteps[i] - time);
        if (difference < minDifference)
          {
          minDifference = difference;
          index = i;
          }
      }
    return index;
  }

  boost::shared_ptr<PoseTransform> calcTransform(InsPVA const* data) {
      double input[3] = {data->LLH[0] * M_PI / 180.0, data->LLH[1] * M_PI / 180.0, data->LLH[2]};
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
      // FIXME: the timestamp in microseconds could possibly exceed the significant decimal digits
      // of 'double'. If this case is proven in practice, we could adjust the week_number to start
      // from a year like 2000 or 2016
      trans->timestamp = double(data->week_number) * WEEK_IN_MICROSECONDS
              + double(data->milliseconds) * 1e3;  //timestamps are in microseconds
      return trans;
  }

  void HandleNewData(boost::shared_ptr<PoseTransform> data)
  {
    boost::lock_guard<boost::mutex> lock(this->ConsumerMutex);

    this->UpdateDequeSize();
    this->Timesteps.push_back(this->LastTime);
    if(transformMgr) {
        transformMgr->AddTransform(data);
    }
    this->Datasets.push_back(data);
    this->NewData = true;
    this->LastTime += 1.0;
  }

  bool NewData;
  int MaxNumberOfDatasets;
  double LastTime;
  InsPVA const* insPVA;
  RawIMU const* rawIMU;
  BestGPSPos const*  bestGPSPos;
  //This is the origin location (in meters) given by Wang Shi Yao.
  //FIXME: this value should be "static const", but due to the library interface
  //in 'CoordiTran.h' dose not support it, hence ommited
  double ORIG_XYZ[3] = {-2781621.9891904, 4672106.75052387, 18.8910392};

  // Hold this when modifying internals of reader
  boost::mutex ConsumerMutex;

  std::deque<boost::shared_ptr<PoseTransform> > Datasets;
  std::deque<double> Timesteps;

  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;

  boost::shared_ptr<boost::thread> Thread;

  TransformManager* transformMgr;
};


//----------------------------------------------------------------------------
class PacketFileWriter
{
public:

  void ThreadLoop()
  {
    std::string* packet = 0;
    while (this->Packets->dequeue(packet))
      {
      this->PacketWriter.write(reinterpret_cast<const char*>(packet->c_str()), packet->length());

      delete packet;
      }
  }

  void Start(const std::string& filename)
  {
    if (this->Thread)
      {
      return;
      }
    if(this->fileName != filename){
        this->fileName = filename;
        if(this->PacketWriter.is_open())
            this->PacketWriter.close();
    }

    if (!this->PacketWriter.is_open())
      {
        this->PacketWriter.open(filename);
      if (!this->PacketWriter)
        {
        std::cerr << "Failed to open packet file: " << filename << std::endl;
        return;
        }
      }

    this->Packets.reset(new SynchronizedQueue<std::string*>);
    this->Thread = boost::shared_ptr<boost::thread>(
      new boost::thread(boost::bind(&PacketFileWriter::ThreadLoop, this)));
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
    return this->PacketWriter.is_open();
  }

  void Close()
  {
    this->PacketWriter.close();
  }

private:
  std::ofstream PacketWriter;
  std::string fileName;
  boost::shared_ptr<boost::thread> Thread;
  boost::shared_ptr<SynchronizedQueue<std::string*> > Packets;
};

class PacketNetworkSource;
//----------------------------------------------------------------------------
class PacketReceiver
{
public:
  PacketReceiver(boost::asio::io_service& io, int port, PacketNetworkSource* parent)
  : Port(port),
    PacketCounter(0),
    Socket(io, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), port)),
    Parent(parent),
    IsReceiving(true),
    ShouldStop(false)
  {
    this->StartReceive();
  }

  ~PacketReceiver()
  {
    this->Socket.cancel();

      {
      boost::unique_lock<boost::mutex> guard(this->IsReceivingMtx);
      this->ShouldStop = true;
      while(this->IsReceiving)
        {
        this->IsReceivingCond.wait(guard);
        }
      }
  }

  void StartReceive()
  {
      {
      boost::lock_guard<boost::mutex> guard(this->IsReceivingMtx);
      this->IsReceiving = true;
      }

    // expecting exactly 1206 bytes, using a larger buffer so that if a
    // larger packet arrives unexpectedly we'll notice it.
    this->Socket.async_receive(boost::asio::buffer(this->RXBuffer, 1500),
      boost::bind(&PacketReceiver::SocketCallback, this,
      boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
  }

  void SocketCallback(const boost::system::error_code& error, std::size_t numberOfBytes);

private:
  int Port;
  size_t PacketCounter;
  boost::asio::ip::udp::socket Socket;
  PacketNetworkSource* Parent;
  char RXBuffer[1500];

  bool IsReceiving;
  bool ShouldStop;
  boost::mutex IsReceivingMtx;
  boost::condition_variable IsReceivingCond;
};

//----------------------------------------------------------------------------
// This class is responsible for the IOService and  two PacketReceiver classes
class PacketNetworkSource
{
public:
  PacketNetworkSource(boost::shared_ptr<PacketConsumer> _consumer) :
    IOService(),
    Thread(),
    receiver(),
    Consumer(_consumer),
    Writer(),
    DummyWork(new boost::asio::io_service::work(this->IOService))
  {
  }

  ~PacketNetworkSource()
  {
    this->Stop();

    delete this->DummyWork;

    if(this->Thread)
      {
      this->Thread->join();
      }
  }

  void QueuePackets(std::string* packet)
  {
    std::string* packet2 = 0;
    if(this->Writer)
      {
      packet2 = new std::string(*packet);
      }

    if( this->Consumer )
      {
      this->Consumer->Enqueue(packet);
      }

    if (this->Writer)
      {
      this->Writer->Enqueue(packet2);
      }
  }

  void Start()
  {
    if(!this->Thread)
      {
      std::cout << "Start listen" << std::endl;
      this->Thread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &this->IOService)));
      }

    // Create work
    this->receiver = boost::shared_ptr<PacketReceiver>(new PacketReceiver(this->IOService, 13123, this));
  }

  void Stop()
  {
    // Kill the receivers
    this->receiver.reset();

  }

  boost::asio::io_service IOService;
  boost::shared_ptr<boost::thread> Thread;

  boost::shared_ptr<PacketReceiver> receiver;

  boost::shared_ptr<PacketConsumer> Consumer;
  boost::shared_ptr<PacketFileWriter> Writer;

  boost::asio::io_service::work* DummyWork;
};

void PacketReceiver::SocketCallback(const boost::system::error_code& error, std::size_t numberOfBytes)
{
//  std::cout << "CALLBACK " << this->Port << std::endl;
  if(error || this->ShouldStop)
    {
    // This is called on cancel
    // TODO: Check other error codes
      {
      boost::lock_guard<boost::mutex> guard(this->IsReceivingMtx);
        this->IsReceiving = false;
      }
      this->IsReceivingCond.notify_one();

    return;
    }

  std::string* packet = new std::string(this->RXBuffer, numberOfBytes);
  this->Parent->QueuePackets(packet);

  this->StartReceive();

//  if ((++this->PacketCounter % 500) == 0)
//    {
//    std::cout << "RECV packets: " << this->PacketCounter << " on " << this->Port << std::endl;;
//    }
}

} // end namespace

//----------------------------------------------------------------------------
class IMUSource::vsInternal
{
public:

  vsInternal() : Consumer(new PacketConsumer),
                  Writer(new PacketFileWriter),
                  NetworkSource(this->Consumer)
  {
  }

  ~vsInternal()
  {
  }

  boost::shared_ptr<PacketConsumer> Consumer;
  boost::shared_ptr<PacketFileWriter> Writer;
  PacketNetworkSource NetworkSource;
};

//----------------------------------------------------------------------------
IMUSource::IMUSource()
{
  this->Internal = new vsInternal;
  this->SensorPort = 13123;
}

//----------------------------------------------------------------------------
IMUSource::~IMUSource()
{
  this->Stop();
  delete this->Internal;
}

//-----------------------------------------------------------------------------
const std::string& IMUSource::GetOutputFile()
{
  return this->OutputFile;
}

//-----------------------------------------------------------------------------
void IMUSource::SetOutputFile(const std::string& filename)
{
  if (filename == this->GetOutputFile())
    {
    return;
    }

  this->Internal->Writer->Close();
  this->OutputFile = filename;
}

void IMUSource::SetTransformManager(TransformManager *mgr)
{
    this->Internal->Consumer->SetTransformManager(mgr);
}

//----------------------------------------------------------------------------
void IMUSource::Start()
{
  if (this->OutputFile.length())
    {
    this->Internal->Writer->Start(this->OutputFile);
    }

  this->Internal->NetworkSource.Writer.reset();

  if (this->Internal->Writer->IsOpen())
    {
    this->Internal->NetworkSource.Writer = this->Internal->Writer;
    }

  this->Internal->Consumer->Start();

  this->Internal->NetworkSource.Start();
}

//----------------------------------------------------------------------------
void IMUSource::Stop()
{
  this->Internal->NetworkSource.Stop();
  this->Internal->Consumer->Stop();
  this->Internal->Writer->Stop();
}

//----------------------------------------------------------------------------
void IMUSource::ReadNextFrame()
{
}

//----------------------------------------------------------------------------
int IMUSource::GetCacheSize()
{
  return this->Internal->Consumer->GetMaxNumberOfDatasets();
}

//----------------------------------------------------------------------------
void IMUSource::SetCacheSize(int cacheSize)
{
  if (cacheSize == this->GetCacheSize())
    {
    return;
    }

  this->Internal->Consumer->SetMaxNumberOfDatasets(cacheSize);
}
