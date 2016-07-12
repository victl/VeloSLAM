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
  Module:    IMUSource.h

  Copyright (c) Zou Lu
  All rights reserved.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME IMUSource -
// .SECTION Description
//

#ifndef __IMUSource_h__
#define __IMUSource_h__

#include "type_defs.h"
#include <string>

class TransformManager;

class IMUSource
{
public:
  void Start();
  void Stop();

  int GetCacheSize();
  void SetCacheSize(int cacheSize);

  void ReadNextFrame();

  const std::string& GetOutputFile();
  void SetOutputFile(const std::string& filename);

  void SetTransformManager(TransformManager* mgr);

  int GetNumberOfChannels();

  IMUSource();
  virtual ~IMUSource();

protected:

  int SensorPort;
  std::string OutputFile;

private:
  IMUSource(const IMUSource&);  // Not implemented.
  void operator=(const IMUSource&);  // Not implemented.

  class vsInternal;
  vsInternal * Internal;
};

#endif  //__IMUSource_h__
