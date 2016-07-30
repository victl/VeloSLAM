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

#include "HDLManager.h"
#include "vtkPacketFileReader.h"
#include <cmath>
#include <limits>
#include <glog/logging.h>

//----------------------------------------------------------------------------
HDLManager::HDLManager(int capacity)
    : framesMutex()
    , hasNewData(false)
    , bufferSize(capacity)
    , maxCacheSize(capacity * 1.2)
    , cache(capacity * 1.2)
    , bufferDirName("/tmp/")
    , isUsingBuffer1(true)
    , writerIdle(true)
    , fileBufferMode(false)
    , cacheCounter(0)
    , packetWriter(new vtkPacketFileWriter)
    , transMgr(new TransformManager)
    , insSrc(new INSSource)
    , hdlSrc(new HDLSource)
    , hdlParser(new HDLParser)
    , timeSolver(new TimeSolver)
{
    // wire up connections between key objects
    insSrc->setTimeSolver(timeSolver);
    insSrc->setTransformManager(transMgr);
    hdlSrc->setHDLManager(this);
    hdlSrc->setTimeSolver(timeSolver);
    hdlSrc->setTransformManager(transMgr);
    hdlSrc->setCorrectionsFile(DEFAULT_CALIB_FILENAME);
    hdlParser->setTransformMgr(transMgr);
    hdlParser->setCorrectionsFile(DEFAULT_CALIB_FILENAME);
    assert(setBufferDir(bufferDirName));
    this->initialize();
}

//----------------------------------------------------------------------------
HDLManager::~HDLManager()
{
    delete packetWriter;
}

void HDLManager::startOnline(bool shouldSwap)
{
    this->loadHDLMeta();
    this->loadINSMeta();
    insSrc->start();
    hdlSrc->start();
    if (shouldSwap) {
        startSwaping();
    }
}

void HDLManager::stopOnline()
{
    hdlSrc->stop();
    insSrc->stop();
    if (fileBufferMode) {
        this->flushFileBuffer();
        this->stopSwaping();
    }
    this->saveHDLMeta();
}

void HDLManager::loadOffline(const string &insTxt, const string &pcapfile)
{
    frames.clear();
    transMgr->loadFromTxtFile(insTxt, true);
    std::cout << "Read " << transMgr->getNumberOfTransforms() << " transforms.\n";
    auto frameVec = hdlParser->readFrameInformation(pcapfile);
    /* because readFrameInformation() can't determine carpose for each frame
     * we need to ask transform manager */
    for (auto & f : frameVec) {
        transMgr->interpolateTransform(f->timestamp, f->carpose.get());
        this->addFrame(f);
    }
    std::cout << "Read " << this->getNumberOfFrames() << " frames." << std::endl;
    this->setBufferDir(boost::filesystem::path(pcapfile).parent_path().string(), false);
}

void HDLManager::touchPcap(const string &pcapfile)
{
    hdlParser->readFrameInformation(pcapfile, true);
}

int HDLManager::getNumberOfFrames() {
    boost::lock_guard<boost::mutex> lock (framesMutex);
    return static_cast<int>(this->frames.size());
}

int HDLManager::getNumberOfTransforms()
{
    return transMgr->getNumberOfTransforms();
}

//----------------------------------------------------------------------------
void HDLManager::initialize()
{
    // make sure output dir is prepared
    this->hardDriveBuffer = &hardDriveBuffer1;
}

void HDLManager::scanBufferDir()
{
    boost::filesystem::path p(this->bufferDirName);
    for (boost::filesystem::directory_entry& entry : boost::filesystem::directory_iterator(p)) {
        std::string extname = entry.path().extension().string();
        if (extname == HDL_META_EXT_NAME) {
            hdlMetaNames.insert(entry.path().string());
        } else if (extname == INS_META_EXT_NAME) {
            insMetaNames.insert(entry.path().string());
        } else if (extname == ".pcap") {
            bufferFileNames.insert(entry.path().filename().string());
        }
    }
}

//----------------------------------------------------------------------------
void HDLManager::switchBuffer()
{
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

void HDLManager::setCalibFile(string filename)
{
    hdlSrc->setCorrectionsFile(filename);
    hdlParser->setCorrectionsFile(filename);
}

void HDLManager::addFrame(boost::shared_ptr<HDLFrame> frame)
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
            switchBuffer();
        }
    } else {
        pushCache(frame);
    }
//    frame->printSelf(); //debug
}

boost::intrusive_ptr<HDLFrame> HDLManager::prepareFrame(boost::shared_ptr<HDLFrame> frame)
{
    if (!frame) return boost::intrusive_ptr<HDLFrame>();
    if (frame->isInMemory) {
        return boost::intrusive_ptr<HDLFrame>(frame.get());
    } else if (frame->isOnHardDrive) {
        std::string filename = bufferDirName + to_iso_string(frame->filenameTime) + ".pcap";
        if (hdlParser->getFrame(frame, filename, frame->fileStartPos, frame->skips)) {
            this->pushCache(frame);
            return boost::intrusive_ptr<HDLFrame>(frame.get());
        } else {
            return boost::intrusive_ptr<HDLFrame>();
        }
    } else {
        return boost::intrusive_ptr<HDLFrame>();
    }
}

boost::intrusive_ptr<HDLFrame> HDLManager::waitForFrame(boost::chrono::microseconds micro)
{
    boost::unique_lock<boost::mutex> lock (framesMutex);
    //        while (!hasNewData) {
    cond_.wait_for(lock, micro);
    //        }
    if (hasNewData) {
        hasNewData = false;
        return prepareFrame(frames.back());
    } else {
        return boost::intrusive_ptr<HDLFrame>();
    }
}

boost::intrusive_ptr<HDLFrame> HDLManager::getRecentFrame() {
    boost::shared_ptr<HDLFrame> result;
    {
        boost::lock_guard<boost::mutex> lock (framesMutex);
        result = frames.back();
    }
    return prepareFrame(result);
}

boost::intrusive_ptr<HDLFrame> HDLManager::getFrameAt(ptime &t)
{
    return prepareFrame(frames.getExactDataAt(t));
}

boost::intrusive_ptr<HDLFrame> HDLManager::getFrameNear(ptime &t)
{
    return prepareFrame(frames.getNearestData(t));
}

vector<boost::shared_ptr<HDLFrame> > HDLManager::getAllFrameMeta()
{
    boost::lock_guard<boost::mutex> lock (framesMutex);
    return frames.getAll();
}

std::vector<boost::intrusive_ptr<HDLFrame> > HDLManager::getRangeBetween(ptime &a, ptime &b)
{
    auto vec = frames.getRangeBetween(a, b);
    std::vector<boost::intrusive_ptr<HDLFrame> > result;
    for (auto& f : vec) {
        result.push_back(prepareFrame(f));
    }
    return std::move(result);
}

void HDLManager::setBufferSize(size_t n){
    if (bufferSize == n) return;
    /* only one writer here, lock not needed? */
    bufferSize = n;
}

size_t HDLManager::getBufferSize() {
    return bufferSize;
}

bool HDLManager::setBufferDir(string dirname, bool shouldCreateSubDir) {
//    path p(dirname);
//    directory_entry dir(p);
    if (! boost::filesystem::is_directory(dirname)){
        std::cerr << "Name of directory invalid" << std::endl;
        return false;
    }
    if (dirname.back() != '/') dirname.append("/");
    if (shouldCreateSubDir) {
        std::string subdirname = to_iso_string(microsec_clock::local_time());
        dirname += subdirname + "/";
        boost::filesystem::path p(dirname);
        if(! create_directory(p)) {
            std::cerr << "Failed to create sub directory inside: " << dirname << std::endl;
            return false;
        }
        assert(boost::filesystem::is_directory(dirname));
    }
    {
        boost::lock_guard<boost::mutex> lock (writerMutex);
        this->bufferDirName = dirname;
    }
    // setup output names
    std::string insname = dirname + to_iso_string(microsec_clock::local_time()) + ".insmeta";
    insSrc->setOutputFile(insname);
    return true;
}

void HDLManager::resetBufferDir() {
    boost::lock_guard<boost::mutex> lock (writerMutex);
    this->bufferDirName = "/tmp/";
}

void HDLManager::setFileBufferMode(bool m)
{
    if (m == fileBufferMode) return;
    boost::unique_lock<boost::mutex> lock (framesMutex);
    fileBufferMode = m;
}

void HDLManager::flushFileBuffer()
{
    while (!writerIdle) {
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    switchBuffer();
    return;
}

bool HDLManager::writePackets()
{
    Buffer* buff;
    std::string filename;
    ptime filenameTime;
    {
        boost::unique_lock<boost::mutex> lock (writerMutex);
        while(writerIdle) { // keep the ready state
            writePacketCond_.wait(lock);
        }
//        if (!fileBufferMode) {
//            writerIdle = true;
//            return false;
//        }
        if (isUsingBuffer1) {
            buff = &hardDriveBuffer2;
        } else {
            buff = &hardDriveBuffer1;
        }
        if (buff->empty()) {
            writerIdle = true;
            return false;
        }
        filenameTime = buff->front()->packets.front().first;
        filename = bufferDirName + to_iso_string(filenameTime) + ".pcap";
    }

    if (packetWriter->isOpen()) packetWriter->close();
    packetWriter->open(filename);
    long long posOffset = PCAP_GLOBAL_HEADER_LEN;
    for (int i = 0; i < buff->size(); ++i) {
        int packetsNum = 0;
        auto& f = buff->at(i)->packets;
        for(int j = 0; j < f.size(); ++j) {
            packetWriter->writePacket(
                        reinterpret_cast<const unsigned char*>(f[j].second.c_str()),
                        f[j].second.length(),
                        f[j].first
                        );
            ++packetsNum;
        }
        buff->at(i)->filenameTime = filenameTime;
        // Manually calc fpos_t
        NUM_TO_FPOS_T(buff->at(i)->fileStartPos, posOffset);
        posOffset += packetsNum * PCAP_PACKET_LEN;
        buff->at(i)->isOnHardDrive = true;
        cache.push(buff->at(i).get());
        ++cacheCounter;
    } // writing buffer finished, clean it
    bufferFileNames.insert(std::move(filename));
    buff->clear();
    updateCacheSize();
    writerIdle = true;
    return true;
}

void HDLManager::writePacketsLoop()
{
    while (writePackets()){}
}

void HDLManager::startSwaping(){
    fileBufferMode = true;
    writerIdle = true;
    if (!this->writeThread) {
        this->writeThread = boost::shared_ptr<boost::thread>(
                    new boost::thread(boost::bind(&HDLManager::writePacketsLoop, this)));
    }
}

void HDLManager::stopSwaping() {
//    fileBufferMode = false;
//    writePacketCond_.notify_all(); // tells writePackets() to end the loop
    if (this->writeThread) {
//        this->writeThread->detach();
//        writeThread->join();
        this->writeThread.reset();
    }
}

void HDLManager::updateCacheSize()
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

void HDLManager::cleanCache()
{
    int tmp = maxCacheSize;
    updateCacheSize();
    maxCacheSize = tmp;
}

bool HDLManager::saveHDLMeta()
{
    std::string filename = this->bufferDirName + to_iso_string(microsec_clock::local_time()) + HDL_META_EXT_NAME;
    std::ofstream ofs(filename);
    if (! ofs) return false;
    ofs << frames;
    return true;
}

bool HDLManager::saveINSMeta()
{
    std::string filename = this->bufferDirName + to_iso_string(microsec_clock::local_time()) + INS_META_EXT_NAME;
    return transMgr->writeToMetaFile(filename);
}

bool HDLManager::loadHDLMeta()
{
    scanBufferDir();
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

bool HDLManager::loadINSMeta()
{
    scanBufferDir();
    if (insMetaNames.empty()) {
        return false;
    } else {
        for (auto & f : insMetaNames) {
            transMgr->loadFromMetaFile(f);
        }
        return true;
    }
}
