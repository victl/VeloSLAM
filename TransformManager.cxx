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

#include "TransformManager.h"
#include "CoordiTran.h"
#include <cmath>
#include <limits>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

//----------------------------------------------------------------------------
TransformManager::TransformManager()
    : mutex_()
{
    this->clearTransforms();
}

//----------------------------------------------------------------------------
TransformManager::~TransformManager()
{
}

//----------------------------------------------------------------------------
int TransformManager::getNumberOfTransforms()
{
  return static_cast<int>(this->transforms.size());
}

//----------------------------------------------------------------------------
void TransformManager::clearTransforms()
{
    this->transforms.clear();
}

//----------------------------------------------------------------------------
/* This function is typically called by an INSSource object when it receives an INS
 * data, then it calculate a transform and invoke this function.
 * NOTE: VERY IMPORTANT!!! Currently I can't make this function thread safe
 */
void TransformManager::addTransform(boost::shared_ptr<PoseTransform> trans)
{
    boost::unique_lock<boost::mutex> lock (mutex_);
    transforms.addData(trans);
//    if (transforms.size() % 500 == 0)
//        std::cout << "Received 500 transforms data" << std::endl;
}

bool TransformManager::loadFromMetaFile(std::string filename, bool clearOldData)
{
    if (!boost::filesystem::exists(filename)) {
        return false;
    }
    if (clearOldData) {
        transforms.clear();
    }
    std::ifstream ifs(filename);
    ifs >> transforms;
    ifs.close();
    return true;
}

bool TransformManager::loadFromTxtFile(std::string filename, bool clearOldData)
{
    if (!boost::filesystem::exists(filename)) {
        return false;
    }
    if (clearOldData) {
        transforms.clear();
    }
    std::ifstream ifs(filename);
    timeval tv;
    ptime t;
    double v;
    boost::shared_ptr<PoseTransform> trans(new PoseTransform);
    while (ifs >> trans->T[0] >> trans->T[1] >> trans->R[2] >> trans->R[0] >> trans->R[1] >> v >> tv.tv_sec >> tv.tv_usec) {
        /* turning the angles from radius to degree */
        trans->R[0] = TO_DEGREE(trans->R[0]);
        trans->R[1] = TO_DEGREE(trans->R[1]);
        /* the '-' is here because the definition of our Eulr angle is counter-clockwise,
         * while as the normal definition is clockwise */
        trans->R[2] = - TO_DEGREE(trans->R[2]);
        timevalToPtime(tv, t);
        ptimeToWeekMilli(t, trans->week_number, trans->milliseconds);
        trans->week_number_pos = trans->week_number;
        trans->seconds_pos = trans->milliseconds / 1000.0f;
        trans->timestamp = t;
        this->addTransform(trans);
        trans = boost::shared_ptr<PoseTransform>(new PoseTransform);
    }
    ifs.close();
    return true;
}

bool TransformManager::writeToMetaFile(const string &filename)
{
    std::ofstream ofs(filename);
    if (! ofs) return false;
    ofs << transforms;
    ofs.close();
    return true;
}

//----------------------------------------------------------------------------
/* This function is typically called by a HDLSource object when it receives a HDL packet,
 * it then invokes this function several times during processing that packet.
 * NOTES:
 * 1. The thread safe issue is not addressed very well, should be improved in future.
 * 2. findIter() is considerably slow, so the design criterion is to avoid using it whenever
 *    possible.
 * 3. Performance measurement: According to an offline emulation on a 1e6 transform dataset, random
 *    access (which involve findIter(). Could improve 100%+ by using searchTimeline()) requires
 *    an average of around 23 microsecs, while sequential access gave 3-4 microsecs. Testing
 *    platform is a MacBook Air, early 2015 model. - See "Test/InterpolateTransformMeasure.cxx"
 * 4. Currently I haven't implement a good method to indicate no valid transform could be infered
 */
bool TransformManager::interpolateTransform(ptime& t, PoseTransform *trans)
{
    trans->timestamp = t; // setting the time is the resposibility of interpolator
    pair<boost::shared_ptr<PoseTransform>, boost::shared_ptr<PoseTransform > > bound;
    {
        boost::unique_lock<boost::mutex> lock (mutex_);
        bound = transforms.getBoundaryData(t);
    }
    if ((!bound.first) && (!bound.second)) { // means transform is empty, no valid data
        return false;
    } else if (! bound.second) { // means there is only one transform in the dataset
        PoseTransform& fore = *(bound.first);
        double sec = (t - fore.timestamp).total_microseconds() / 1e6f;
        for (int i = 0; i < 3; ++i) {
            trans->V[i] = fore.V[i];
            trans->R[i] = fore.R[i];
            trans->T[i] = fore.T[i] + fore.V[i] * sec;
        }
        return true;
    } else { // a valid boundary is got
        PoseTransform& fore = *(bound.first);
        PoseTransform& back = *(bound.second);
        time_duration diff = t - fore.timestamp;
        double ratio = double(diff.total_microseconds()) / (back.timestamp - fore.timestamp).total_microseconds();
        *trans = fore + ((back - fore) * ratio);
        trans->seconds_pos = 0; // set from -1 (default) to 0 means a valid data has got
        return true;
    }
}

void TransformManager::setOriginLLH(const double LLH[])
{
    originLLH[0] = TO_RADIUS(LLH[0]);
    originLLH[1] = TO_RADIUS(LLH[1]);
    originLLH[2] = LLH[2];
    llh2xyz(originLLH, originXYZ);
}
