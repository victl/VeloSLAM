/*=========================================================================

  Program:   VeloSLAM
  Module:    HDLParser.h

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
  Module:    vtkVelodyneHDLReader.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME HDLParser - class for processing Velodyne HDL packets
// .Section Description
//

#ifndef _HDLParser_h
#define _HDLParser_h

#include "type_defs.h"
#include "HDLFrame.h"

#include <boost/shared_ptr.hpp>
#include <vector>
#include <deque>
#include <string>
#include <array>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

class TransformManager;
class HDLManager;

class HDLParser
{
public:
  enum DualFlag : unsigned char {
    DUAL_DISTANCE_NEAR = 0x1,   // point with lesser distance
    DUAL_DISTANCE_FAR = 0x2,    // point with greater distance
    DUAL_INTENSITY_HIGH = 0x4,  // point with lesser intensity
    DUAL_INTENSITY_LOW = 0x8,   // point with greater intensity
    DUAL_DOUBLED = 0xf,         // point is single return
    DUAL_DISTANCE_MASK = 0x3,
    DUAL_INTENSITY_MASK = 0xc,
  };

public:

  //Description:
  //
  const std::string& getDirName();
  void setDirName(const std::string& filename);

  //Description:
  //
  const std::string& getCorrectionsFile();
  void setCorrectionsFile(const std::string& correctionsFile);

  // Property functions

  // Description:
  // Number of frames behind current frame to read.  Zero indicates only
  // show the current frame.  Negative numbers are invalid.
  void setNumberOfTrailingFrames(int numberTrailing);

  // Description:
  // TODO: This is not friendly but I dont have a better way to pass 64 values to a filter in
  // paraview
  void setLaserSelection(int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int,
                    int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int,
                    int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int,
                    int, int, int, int, int, int, int, int, int, int, int, int, int, int, int, int);
  void setLaserSelection(int LaserSelection[64]);

  void getLaserSelection(int LaserSelection[64]);

  void getVerticalCorrections(double LaserAngles[64]);

  unsigned int getDualReturnFilter() const;
  void setDualReturnFilter(unsigned int);

  void setPointsSkip(int);

  void setCropReturns(int);
  void setCropInside(int);
  void setCropRegion(double[6]);
  void setCropRegion(double, double, double, double, double, double);

  // I/O and processing functions
  int getNumberOfChannels();

  /* touchOnly means only rename the pcap file, don't use the returned value */
  std::vector<boost::shared_ptr<HDLFrame> > readFrameInformation(const string &name, bool touchOnly = false);
  void setHDLManager(boost::shared_ptr<HDLManager> p);

  bool getFrame(boost::shared_ptr<HDLFrame> &dest, const string &filename, fpos_t &startPos, const int &skip);
  void processHDLPacket(unsigned char *data, unsigned int bytesReceived, boost::posix_time::ptime t);
  std::deque<boost::shared_ptr<HDLFrame>> getAllFrames();
  void clearAllFrames();
  boost::shared_ptr<HDLFrame> createHDLFrame();

  // Transform related functions

  boost::shared_ptr<TransformManager>  getTransformMgr() const;
  void setTransformMgr(boost::shared_ptr<TransformManager> mgr);
  int getApplyTransform();
  void setApplyTransform(int apply);

  HDLParser();
  ~HDLParser();

protected:

  void unloadData();

  std::string correctionsFile;
  std::string dirName;
  std::string timesFileName;

  class vsInternal;
  vsInternal* internal_;

private:

  HDLParser(const HDLParser&);
  void operator = (const HDLParser&);

};
#endif
