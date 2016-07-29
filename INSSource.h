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
  Module:    INSSource.h

  Copyright (c) Zou Lu
  All rights reserved.

     This software is distributed WITHOUT ANY WARRANTY; without even
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
     PURPOSE.  See the above copyright notice for more information.

=========================================================================*/
// .NAME INSSource -
// .SECTION Description
//

#ifndef __INSSource_h__
#define __INSSource_h__

#include "type_defs.h"
#include <string>

class TransformManager;
class TimeSolver;

class INSSource
{
public:
  void start();
  void stop();

  void setOutputFile(const std::string& filename);

  void setTransformManager(boost::shared_ptr<TransformManager> mgr);
  void setTimeSolver(boost::shared_ptr<TimeSolver> solver);
  void setOrigin(double org[3]);

  INSSource(int port = 6777);
  virtual ~INSSource();

private:
  INSSource(const INSSource&);  // Not implemented.
  void operator=(const INSSource&);  // Not implemented.

  class vsInternal;
  vsInternal * internal_;
};

#endif  //__INSSource_h__
