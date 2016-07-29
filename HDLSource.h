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
  Module:    vtkVelodyneHDLSource.h

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME HDLSource -
// .SECTION Description
//

#ifndef __HDLSource_h__
#define __HDLSource_h__

#include "type_defs.h"
#include <string>

#include <array>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "vtkPacketFileWriter.h"
#include "TransformManager.h"
#include <boost/shared_ptr.hpp>

class HDLManager;
class TimeSolver;
class TransformManager;
class HDLParser;
class HDLSource
{
public:

  void start();
  void stop();

  const std::string& getCorrectionsFile();
  void setCorrectionsFile(const std::string& correctionsFile);

  void setOutputFile(const std::string& filename);

//  vtkSetMacro(SensorPort, int);
//  vtkGetMacro(SensorPort, int);

  void setLaserSelection(int LaserSelection[64]);
  void getLaserSelection(int LaserSelection[64]);

  void setCropReturns(int);
  void setCropInside(int);
  void setCropRegion(double[6]);
  void setCropRegion(double, double, double, double, double, double);

  void getVerticalCorrections(double LaserAngles[64]);

  unsigned int getDualReturnFilter() const;
  void setDualReturnFilter(unsigned int);

  void setHDLManager(HDLManager* hp);
  void setTimeSolver(boost::shared_ptr<TimeSolver> solver);
  void setTransformManager(boost::shared_ptr<TransformManager> mgr);
  boost::shared_ptr<HDLParser> getHDLParser();

  int getNumberOfChannels();

  HDLSource(int _port = 2368);
  virtual ~HDLSource();

protected:

  int sensorPort;
  std::string PacketFile;
  std::string outputFile;
  std::string timeOutputFile;
  std::string CorrectionsFile;

private:
  HDLSource(const HDLSource&);  // Not implemented.
  void operator=(const HDLSource&);  // Not implemented.

  class vsInternal;
  vsInternal * internal_;
};

#endif  //__HDLSource_h__
