/*=========================================================================

  Program:   VeloSLAM
  Module:    TransformManager.cxx

  Copyright (c) Zou Lu <victl@163.com>
  All rights reserved.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

// This file is adapted from the source code of VeloView. Two reasons for
// this adaptation: 1) Original codes involve many not-so-necessary operations
// that would slow down the program, which is unaffordable in online applications.
// 2) Original codes are deeply coupled with the VTK library, which is undesirable
// in my working environment.

/********* FOLLOWING COPYRIGHT INFO FROM VELOVIEW ARE PRESERVED **********/
/*=========================================================================

  Program:   Visualization Toolkit
  Module:    vtkTransformInterpolator.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "HDLProcessor.h"
#include <cmath>
#include <limits>
#include <glog/logging.h>

//----------------------------------------------------------------------------
HDLProcessor::HDLProcessor(int capacity)
    : framesMutex()
    , hasNewData(false)
    , bufferSize(capacity)
    , maxCacheSize(capacity * 1.2)
    , bufferDirName("/tmp/")
    , isUsingBuffer1(true)
    , writerIdle(true)
    , fileBufferMode(true)
    , cacheCounter(0)
    , packetWriter(new vtkPacketFileWriter)
    , transMgr(new TransformManager)
    , insSrc(new INSSource)
    , hdlSrc(new HDLSource)
    , hdlReader(new HDLReader)
    , timeSolver(new TimeSolver)
{
    this->initialize();
}

//----------------------------------------------------------------------------
HDLProcessor::~HDLProcessor()
{
    delete packetWriter;
    this->stopSwaping();
}

int HDLProcessor::getNumberOfFrames() {
    boost::lock_guard<boost::mutex> lock (framesMutex);
    return static_cast<int>(this->frames.size());
}

//----------------------------------------------------------------------------
void HDLProcessor::initialize()
{
    this->hardDriveBuffer = &hardDriveBuffer1;
    this->startSwaping();
}

void HDLProcessor::scanCacheDir()
{
    using namespace boost::filesystem;
    path p(this->bufferDirName);
    for (directory_entry& entry : directory_iterator(p)) {
        if (entry.path().extension() == HDL_META_EXT_NAME) {
            hdlMetaNames.insert(entry.path().filename().string());
        } else if (entry.path().extension() == INS_META_EXT_NAME) {
            insMetaNames.insert(entry.path().filename().string());
        } else if (entry.path().extension() == ".pcap") {
            bufferFileNames.insert(entry.path().filename().string());
        }
    }
}

//----------------------------------------------------------------------------
void HDLProcessor::addFrame(boost::shared_ptr<HDLFrame> frame)
{
    {
        boost::lock_guard<boost::mutex> lock (framesMutex);
        frames.addData(frame);
        hasNewData = true;
        cond_.notify_one();
    }
    if (fileBufferMode && (! frame->isOnHardDrive)) {
        hardDriveBuffer->push_back(frame);
        if (hardDriveBuffer->size() == bufferSize && writerIdle) {
            boost::lock_guard<boost::mutex> lock (writerMutex);
            if (isUsingBuffer1) { // swap the pointer
                hardDriveBuffer = &hardDriveBuffer2;
            } else {
                hardDriveBuffer = &hardDriveBuffer1;
            }
            isUsingBuffer1 = !isUsingBuffer1;
            writerIdle = false; // tell the writing thread "don't be lazy, get busy!"
            writePacketCond_.notify_one();
        }
    } else {
        pushCache(frame);
    }
    frame->printSelf(); //debug
}

boost::intrusive_ptr<HDLFrame> HDLProcessor::prepareFrame(boost::shared_ptr<HDLFrame> frame)
{
    if (frame) return boost::intrusive_ptr<HDLFrame>(frame.get());
    if (frame->isInMemory) {
        return boost::intrusive_ptr<HDLFrame>(frame.get());
    } else if (frame->isOnHardDrive) {
        std::string filename = bufferDirName + to_iso_string(frame->filenameTime) + ".pcap";
        if (hdlReader->getFrame(frame, filename, frame->fileStartPos, frame->skips)) {
            return boost::intrusive_ptr<HDLFrame>(frame.get());
        } else {
            return boost::intrusive_ptr<HDLFrame>();
        }
    } else {
        return boost::intrusive_ptr<HDLFrame>();
    }
}

boost::intrusive_ptr<HDLFrame> HDLProcessor::waitForFrame()
{
    boost::shared_ptr<HDLFrame> result;
    {
        boost::unique_lock<boost::mutex> lock (framesMutex);
        while (!hasNewData) {
            cond_.wait(lock);
        }
        hasNewData = false;
        result = frames.back();
    }
    return prepareFrame(result);
}

boost::intrusive_ptr<HDLFrame> HDLProcessor::getRecentFrame() {
    boost::shared_ptr<HDLFrame> result;
    {
        boost::lock_guard<boost::mutex> lock (framesMutex);
        result = frames.back();
    }
    return prepareFrame(result);
}

boost::intrusive_ptr<HDLFrame> HDLProcessor::getFrameAt(ptime &t)
{
    return prepareFrame(frames.getExactDataAt(t));
}

boost::intrusive_ptr<HDLFrame> HDLProcessor::getFrameNear(ptime &t)
{
    return prepareFrame(frames.getNearestData(t));
}

std::vector<boost::intrusive_ptr<HDLFrame> > HDLProcessor::getRangeBetween(ptime &a, ptime &b)
{
    auto vec = frames.getRangeBetween(a, b);
    std::vector<boost::intrusive_ptr<HDLFrame> > result;
    for (auto& f : vec) {
        result.push_back(prepareFrame(f));
    }
    return std::move(result);
}

void HDLProcessor::setBufferSize(size_t n){
    if (bufferSize == n) return;
    /* only one writer here, lock not needed? */
    bufferSize = n;
}

size_t HDLProcessor::getBufferSize() {
    return bufferSize;
}

void HDLProcessor::setBufferDir(const string &dirname) {
    path p(dirname);
    directory_entry dir(p);
    if (! is_directory(dir.status())){
        std::cerr << "Name of directory invalid" << std::endl;
        return;
    }
    boost::lock_guard<boost::mutex> lock (writerMutex);
    this->bufferDirName = dirname;
    if (this->bufferDirName.back() != '/') this->bufferDirName.append("/");
}

void HDLProcessor::resetBufferDir() {
    boost::lock_guard<boost::mutex> lock (writerMutex);
    this->bufferDirName = "/tmp/";
}

void HDLProcessor::setFileBufferMode(bool m)
{
    if (m == fileBufferMode) return;
    boost::unique_lock<boost::mutex> lock (framesMutex);
    fileBufferMode = m;
}

void HDLProcessor::writePacketsLoop()
{
    Buffer* buff;
    std::string filename;
    ptime filenameTime;
    {
        boost::unique_lock<boost::mutex> lock (writerMutex);
        while(writerIdle) { // keep the ready state
            writePacketCond_.wait(lock);
        }
        if (isUsingBuffer1) {
            buff = &hardDriveBuffer2;
        } else {
            buff = &hardDriveBuffer1;
        }
        filename = bufferDirName + to_iso_string(filenameTime) + ".pcap";
        filenameTime = buff->front()->packets->front().first;
    }

    if (packetWriter->isOpen()) packetWriter->close();
    packetWriter->open(filename);
    for (int i = 0; i < buff->size(); ++i) {
        auto& f = *(buff->at(i)->packets.get());
        for(int j = 0; j < f.size(); ++j) {
            packetWriter->WritePacket(
                        reinterpret_cast<const unsigned char*>(f[j].second.c_str()),
                        f[j].second.length(),
                        f[j].first
                        );
        }
        buff->at(i)->filenameTime = filenameTime;
        // FIXME: need to determine fileStartPos here
        // buff->at(i)->fileStartPos = ...
        buff->at(i)->isOnHardDrive = true;
        cache.push(buff->at(i).get());
        ++cacheCounter;
    } // writing buffer finished, clean it
    bufferFileNames.insert(std::move(filename));
    buff->clear();
    updateCacheSize();
    writerIdle = true;
    writePacketsLoop(); // call itself, thus the 'loop'
}

void HDLProcessor::startSwaping(){
    if (!this->writeThread) {
        this->writeThread = boost::shared_ptr<boost::thread>(
                    new boost::thread(boost::bind(&HDLProcessor::writePacketsLoop, this)));
    }
}

void HDLProcessor::stopSwaping() {
    if (this->writeThread) {
        this->writeThread.reset();
    }
}

void HDLProcessor::updateCacheSize()
{
    int putBackTimes = 10;
    while (cacheCounter > maxCacheSize && putBackTimes) {
        HDLFrame* obj;
        cache.pop(obj);
        if (obj->count) {
            cache.push(obj);
            -- putBackTimes;
            if (!putBackTimes) {
                DLOG(WARNING) << "[CACHE]Consecutively putted back 10 in-using obj, cache size might need increasing.";
            }
        } else {
            obj->clear();
            --cacheCounter;
        }
    }
}

void HDLProcessor::cleanCache()
{
    int tmp = maxCacheSize;
    updateCacheSize();
    maxCacheSize = tmp;
}

void HDLProcessor::saveHDLMeta()
{
    std::string filename = this->bufferDirName + to_iso_string(microsec_clock::local_time()) + HDL_META_EXT_NAME;
    std::ofstream ofs(filename);
    ofs << frames;
}

bool HDLProcessor::loadHDLMeta()
{
    scanCacheDir();
    if (hdlMetaNames.empty()) {
        return false;
    } else {
        for (auto & f : hdlMetaNames) {
            std::ifstream ifs(f);
            ifs >> frames;
        }
        return true;
    }
}

bool HDLProcessor::loadINSMeta()
{
    scanCacheDir();
    if (insMetaNames.empty()) {
        return false;
    } else {
        for (auto & f : insMetaNames) {
            transMgr->loadFromFile(f);
        }
        return true;
    }
}
