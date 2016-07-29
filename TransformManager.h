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
/* .NAME vtkVelodyneTransformInterpolator - interpolate a series of transformation matrices
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
*/

#ifndef __TransformManager_h__
#define __TransformManager_h__

#include "type_defs.h"
#include "TimeLine.h"
#include <boost/smart_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <vector>

typedef std::vector<boost::shared_ptr<PoseTransform> >::iterator MgrIter;

class TransformManager
{
public:
    TransformManager();
    virtual ~TransformManager();

  // Description:
  // Return the number of transforms in the list of transforms.
  int getNumberOfTransforms();

  // Description:
  // Clear the list of transforms.
  void clearTransforms();

  // Description:
  // Best practice is to add PoseTransforms in time sequence. This is the
  // case in most scenarios. That's why the internal container choosed std::vector
  void addTransform(boost::shared_ptr<PoseTransform> trans);

  bool loadFromMetaFile(std::string filename, bool clearOldData = false);
  bool loadFromTxtFile(std::string filename, bool clearOldData = false);

  bool writeToMetaFile(const std::string& filename);

  // Description:
  // Interpolate the list of transforms and determine a new transform (i.e.,
  // fill in the transformation provided). If t is outside the range of
  // (min,max) values, then t is clamped.
  bool interpolateTransform(ptime &t, PoseTransform *xform);

  void setOriginLLH(const double LLH[3]);

protected:

  // Keep track of inserted data
  TimeLine<PoseTransform> transforms;

private:
  TransformManager(const TransformManager&);  // Not implemented.
  void operator=(const TransformManager&);  // Not implemented.
  boost::mutex mutex_;
  double originLLH[3], originXYZ[3];

};

#endif  // __TransformManager_h__
