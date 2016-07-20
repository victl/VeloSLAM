/*=========================================================================

  Program:   VeloSLAM
  Module:    TransformManager.h

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
  Module:    vtkVelodyneTransformInterpolator.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME vtkVelodyneTransformInterpolator - interpolate a series of transformation matrices
// .SECTION Description
// This class is used to interpolate a series of 4x4 transformation
// matrices. Position, scale and orientation (i.e., rotations) are
// interpolated separately, and can be interpolated linearly or with a spline
// function. Note that orientation is interpolated using quaternions via
// SLERP (spherical linear interpolation) or the special vtkQuaternionSpline
// class.
//
// To use this class, specify at least two pairs of (t,transformation matrix)
// with the AddTransform() method.  Then interpolated the transforms with the
// InterpolateTransform(t,transform) method, where "t" must be in the range
// of (min,max) times specified by the AddTransform() method.
//
// By default, spline interpolation is used for the interpolation of the
// transformation matrices. The position, scale and orientation of the
// matrices are interpolated with instances of the classes
// vtkTupleInterpolator (position,scale) and vtkQuaternionInterpolator
// (rotation). The user can override the interpolation behavior by gaining
// access to these separate interpolation classes.  These interpolator
// classes (vtkTupleInterpolator and vtkQuaternionInterpolator) can be
// modified to perform linear versus spline interpolation, and/or different
// spline basis functions can be specified.
//
// .SECTION Caveats
// The interpolator classes are initialized when the InterpolateTransform()
// is called. Any changes to the interpolators, or additions to the list of
// transforms to be interpolated, causes a reinitialization of the
// interpolators the next time InterpolateTransform() is invoked. Thus the
// best performance is obtained by 1) configuring the interpolators, 2) adding
// all the transforms, and 3) finally performing interpolation.

#ifndef __HDLManager_h__
#define __HDLManager_h__

#include "type_defs.h"
#include "HDLFrame.h"
#include "TimeLine.h"
#include "TransformManager.h"
#include "INSSource.h"
#include "HDLSource.h"
#include "HDLParser.h"
#include "TimeSolver.h"
#include "vtkPacketFileWriter.h"
#include <boost/smart_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/date_time.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/atomic.hpp>
#include <boost/lockfree/queue.hpp>
#include <boost/lockfree/spsc_queue.hpp>
#include <string>
#include <set>
#include <boost/filesystem.hpp>

#ifdef ONCAR
#define DEFAULT_CALIB_FILENAME "/home/denggroup/UGV/etc/db64.xml"
#else
#ifdef APPLE
#define DEFAULT_CALIB_FILENAME "/Users/victor/Workspace/ugv/bin/db64.xml"
#else
#define DEFAULT_CALIB_FILENAME "/home/victor/Workspace/ugv/bin/db64.xml"
#endif
#endif

using namespace boost;
using namespace boost::gregorian;
using namespace boost::posix_time;
using namespace boost::filesystem;
namespace {
    typedef std::vector<boost::shared_ptr<HDLFrame>> Buffer;
}
class HDLManager
{
public:
    HDLManager(int capacity = 200); // 600 hdl frames ~= 1 minute
    virtual ~HDLManager();

    void start(bool shouldSwap = false);
    void stop();
  // Description:
  // Return the number of frame in the list of LiDAR frames.
  int getNumberOfFrames();

  // Description:
  void addFrame(boost::shared_ptr<HDLFrame> frame);

  boost::intrusive_ptr<HDLFrame> prepareFrame(boost::shared_ptr<HDLFrame> frame);
  /* Typically you instantiate a dedicated thread to call this function, because
   * it'll block the caller if no new data comes in.
   */
  boost::intrusive_ptr<HDLFrame> waitForFrame(boost::chrono::microseconds micro = boost::chrono::microseconds((int)1e5));
  boost::intrusive_ptr<HDLFrame> getRecentFrame();
  boost::intrusive_ptr<HDLFrame> getFrameAt(ptime& t);
  boost::intrusive_ptr<HDLFrame> getFrameNear(ptime& t);
  /* get range between [a,b], that is inclusive on both end */
  std::vector<boost::intrusive_ptr<HDLFrame>> getRangeBetween(ptime& a, ptime& b);


  void setBufferSize(size_t n);

  size_t getBufferSize();

  bool setBufferDir(string dirname, bool shouldCreateSubDir = true);

  void resetBufferDir();

  void setFileBufferMode(bool m = true);

  void flushFileBuffer();

  bool writePackets();
  void writePacketsLoop();

  void startSwaping();
  void stopSwaping();

  void pushCache(boost::shared_ptr<HDLFrame>& frame) {
      cache.push(frame.get());
      ++cacheCounter;
      updateCacheSize();
  }

  /* updte cache size */
  void updateCacheSize();

  void cleanCache();

  void saveHDLMeta();

  /* All load*** functions are intended to be used during offline, or just before
   * the online process begins. See comments on scanCacheDir()
   */
  bool loadHDLMeta();
  bool loadINSMeta();

  void switchBuffer();

  /* Setting individual inner objests's properties */
  void setCalibFile(std::string filename);

protected:

  // Keep track of inserted data
  TimeLine<HDLFrame> frames;

  /* The distinction between buffer and cache is that buffer only concerns writing
   * data into hard disk, it does not manage memory. While cache is dedicated to
   * memory management. Only when old data been removed from the cache should it's
   * memory get reclaimed.
   * A boost::lockfree::queue sounds reasonable here, as pushing/poping should all be
   * fast operation, and won't happen consecutively and frequently.
   *
   * HOWEVER, a lock free queue requires it's contents T must have a trivial
   * assignment operator, etc. shared_ptr does not meet this requirement. So I've decided
   * to store the raw HDLFrame pointer in it. Because in the current library design,
   * HDLFrames does not go away from memory easily once created (only its member .points,
   * .pointsMeta, .packets get cleared occasionally to free up memory). Most of the time,
   * the deallocation happens on program exit. So it MIGHT BE safe.
   *
   * NOTE: It's quite arguable to using lockfree queue, ref here:
   * http://boost.2283326.n4.nabble.com/LockFree-a-queue-of-std-shared-ptr-td4642662.html
   */
  boost::lockfree::queue<HDLFrame*> cache;
  // if buffer->size() > cacheSize, data will be written into hard disk
  Buffer hardDriveBuffer1, hardDriveBuffer2;
  Buffer* hardDriveBuffer;
  /* boost::lockfree::queue does not provide .size(), so we need an explicit counter */
  boost::atomic<int> cacheCounter;

private:
  HDLManager(const HDLManager&);  // Not implemented.
  void operator=(const HDLManager&);  // Not implemented.
  void initialize();
  boost::mutex framesMutex;
  boost::condition_variable cond_;   // The condition to wait for
  boost::atomic<bool> hasNewData;
  size_t bufferSize; // use it to avoid frequently asks the container
  size_t maxCacheSize;
  std::string bufferDirName;
  // currently filename identifier is a ptime, using an additional cacheFileNames vector
  // is for future switching to using vector[i] as identifier
  std::set<std::string> bufferFileNames;
  std::set<std::string> hdlMetaNames;
  std::set<std::string> insMetaNames;
  boost::atomic<bool> isUsingBuffer1;
  boost::atomic<bool> writerIdle;
  boost::atomic<bool> fileBufferMode;
  vtkPacketFileWriter* packetWriter;
  boost::mutex writerMutex;
  boost::condition_variable writePacketCond_;
  boost::shared_ptr<boost::thread> writeThread;

  // Hold some important objects, and coordinate between them
  boost::shared_ptr<TransformManager> transMgr;
  boost::shared_ptr<INSSource> insSrc;
  boost::shared_ptr<HDLSource> hdlSrc;
  boost::shared_ptr<HDLParser> hdlParser;
  boost::shared_ptr<TimeSolver> timeSolver;

  /* try not call this function when program is running online, because it'll read in
   * some files that are in use by some other threads. This could cause problems.
   */
  void scanCacheDir();
};

#endif  // __HDLManager_h__
