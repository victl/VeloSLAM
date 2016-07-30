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
  Module:    vtkVelodyneHDLReader.cxx

  Copyright (c) Ken Martin, Will Schroeder, Bill Lorensen
  All rights reserved.
  See Copyright.txt or http://www.kitware.com/Copyright.htm for details.

  This software is distributed WITHOUT ANY WARRANTY; without even
  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
  PURPOSE.  See the above copyright notice for more information.

=========================================================================*/

#include "HDLParser.h"
#include "TransformManager.h"
#include "HDLManager.h"

#include "vtkPacketFileReader.h"
#include "vtkPacketFileWriter.h"

#include <limits>
#include <stdio.h>

#include <sstream>
#include <fstream>
#include <algorithm>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/filesystem.hpp>

#ifdef _MSC_VER
# include <boost/cstdint.hpp>
typedef boost::uint8_t uint8_t;
#else
# include <stdint.h>
#endif

namespace
{

#define HDL_Grabber_toRadians(x) ((x) * M_PI / 180.0)

enum HDLBlock
{
  BLOCK_0_TO_31 = 0xeeff,
  BLOCK_32_TO_63 = 0xddff
};

#pragma pack(push, 1)
typedef struct HDLLaserReturn
{
  unsigned short distance;
  unsigned char intensity;
} HDLLaserReturn;

struct HDLFiringData
{
  unsigned short blockIdentifier;
  unsigned short rotationalPosition;
  HDLLaserReturn laserReturns[HDL_LASER_PER_FIRING];
};

struct HDLDataPacket
{
  HDLFiringData firingData[HDL_FIRING_PER_PKT];
  uint32_t gpsTimestamp;
  unsigned char blank1;
  unsigned char blank2;
};

struct HDLLaserCorrection
{
  double azimuthCorrection;
  double verticalCorrection;
  double distanceCorrection;
  double verticalOffsetCorrection;
  double horizontalOffsetCorrection;
  double sinVertCorrection;
  double cosVertCorrection;
  double sinVertOffsetCorrection;
  double cosVertOffsetCorrection;
};

struct HDLRGB
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
};
#pragma pack(pop)

//-----------------------------------------------------------------------------
int mapFlags(unsigned int flags, unsigned int low, unsigned int high)
{
  return (flags & low ? -1 : flags & high ? 1 : 0);
}

//-----------------------------------------------------------------------------
int mapDistanceFlag(unsigned int flags)
{
  return mapFlags(flags & HDLParser::DUAL_DISTANCE_MASK,
                  HDLParser::DUAL_DISTANCE_NEAR,
                  HDLParser::DUAL_DISTANCE_FAR);
}

//-----------------------------------------------------------------------------
int mapIntensityFlag(unsigned int flags)
{
  return mapFlags(flags & HDLParser::DUAL_INTENSITY_MASK,
                  HDLParser::DUAL_INTENSITY_LOW,
                  HDLParser::DUAL_INTENSITY_HIGH);
}

//-----------------------------------------------------------------------------
  double HDL32AdjustTimeStamp(int firingblock,
                              int dsr)
{
  return (firingblock * 46.08) + (dsr * 1.152);
}

//-----------------------------------------------------------------------------
double VLP16AdjustTimeStamp(int firingblock,
                            int dsr,
                            int firingwithinblock)
{
  return (firingblock * 110.592) + (dsr * 2.304) + (firingwithinblock * 55.296);
}

}

//-----------------------------------------------------------------------------
class HDLParser::vsInternal
{
public:

    vsInternal()
    {
        this->firingSkip = 0;
        this->lastAzimuth = -1;
        //    this->TimeAdjust = std::numeric_limits<double>::quiet_NaN();
        this->packetReader = 0;
        this->splitCounter = 0;
        this->numberOfTrailingFrames = 0;
        this->applyTransform = 0;
        this->pointsSkip = 0;
        this->shouldCropReturns = false;
        this->shouldCropInside = false;
        this->cropRegion[0] = this->cropRegion[1] = 0.0;
        this->cropRegion[2] = this->cropRegion[3] = 0.0;
        this->cropRegion[4] = this->cropRegion[5] = 0.0;
        this->correctionsInitialized = false;
        this->calibFileReportedNumLasers = 64;

        this->laserSelections.resize(HDL_MAX_NUM_LASERS, true);
        this->dualReturnFilter = 0;
        this->isDualReturnData = false;
        this->isHDL64Data = false;
        frameMetaInited = false;
        /* I have no better way to initialize HDL64BeamLUT */

        int tmparray[64] =
        {38,39,42,43,32,33,36,37,40,41,46,47,50,51,54,55,44,45,48,49,52,53,58,59
         ,62,63,34,35,56,57,60,61,6,7,10,11,0,1,4,5,8,9,14,15,18,19,22,23,12,13
         ,16,17,20,21,26,27,30,31,2,3,24,25,28,29};
//        {36,37,58,59,38,39,32,33,40,41,34,35,48,49,42,43,50,51,44,45,52,53,46
//         ,47,60,61,54,55,62,63,56,57,4,5,26,27,6,7,0,1,8,9,2,3,16,17,10,11,18,19
//         ,12,13,20,21,14,15,28,29,22,23,30,31,24,25};

        std::memcpy(HDL64BeamLUT, tmparray, sizeof(int) * 64);

        this->init();
    }

    ~vsInternal()
    {
    }

    std::deque<boost::shared_ptr<HDLFrame> > frames;
    boost::shared_ptr<HDLFrame> currentFrame;
    bool frameMetaInited;
    boost::shared_ptr<TransformManager> transMgr;
    boost::shared_ptr<HDLManager> hdlMgr;

    bool isDualReturnData;
    bool isHDL64Data;

    /* Because the verticalCorrection of the beams are not sorted, we want the
     * output point clouds be sorted by vertical angle (from lower to upper),
     * hence this Look Up Table (LUT).
     * Ref: The Velodyne HDL-64 S2 Users' Manual
            or
            The correction file '64db.xml' itself */
    int HDL64BeamLUT[64];

    int lastAzimuth;

    vtkPacketFileReader reader;
    int firingSkip;
    vtkPacketFileReader* packetReader;

    int splitCounter;

    // Parameters ready by calibration
    std::vector<double> cos_lookup_table_;
    std::vector<double> sin_lookup_table_;
    HDLLaserCorrection laser_corrections_[HDL_MAX_NUM_LASERS];
    int calibFileReportedNumLasers;
    bool correctionsInitialized;

    // User configurable parameters
    int numberOfTrailingFrames;
    int applyTransform;
    int pointsSkip;

    bool shouldCropReturns;
    bool shouldCropInside;
    double cropRegion[6];

    std::vector<bool> laserSelections;
    unsigned int dualReturnFilter;

    void splitFrame(bool force=false);
    boost::shared_ptr<HDLFrame> createHDLFrame();

    void init();
    void initLookUpTables();
    void loadCorrectionsFile(const std::string& filename);

    void processHDLPacket(unsigned char *data, std::size_t bytesReceived, ptime timestamp);

    /* By re-projecting to the frame beginning, we could deal the whole frame with a
   * coherent coordinate system - the carpose according to the arriving time of the
   *  first packet.
   * NOTE: The re-projection here is not strictly accurate, as no rotation is
   * considered, because only translation relative to carpose is required to
   * restore a global coordinate. */
    void reprojectToFrameBeginning(PoseTransform* trans);

    // Process the laser return from the firing data
    // firingData - one of HDL_FIRING_PER_PKT from the packet
    // hdl64offset - either 0 or 32 to support 64-laser systems
    // firingBlock - block of packet for firing [0-11]
    // azimuthDiff - average azimuth change between firings
    // timestamp - the timestamp of the packet
    // geotransform - georeferencing transform
    void processFiring(HDLFiringData* firingData,
                       int hdl65offset,
                       int firingBlock,
                       int azimuthDiff,
                       ptime timestamp,
                       unsigned int rawtime,
                       Eigen::Affine3d *geotransform);

    void pushFiringData(const unsigned char laserId,
                        const unsigned char rawLaserId,
                        unsigned short azimuth,
                        const ptime timestamp,
                        const unsigned int rawtime,
                        const HDLLaserReturn* laserReturn,
                        const HDLLaserCorrection* correction,
                        Eigen::Affine3d* geotransform,
                        bool dualReturn);
};

//-----------------------------------------------------------------------------
HDLParser::HDLParser()
{
  this->internal_ = new vsInternal;
  this->unloadData();
}

//-----------------------------------------------------------------------------
HDLParser::~HDLParser()
{
  delete this->internal_;
}

//-----------------------------------------------------------------------------
const std::string& HDLParser::getDirName()
{
  return this->dirName;
}

//-----------------------------------------------------------------------------
void HDLParser::setApplyTransform(int apply)
{
  this->internal_->applyTransform = apply;
}

//-----------------------------------------------------------------------------
int HDLParser::getApplyTransform()
{
  return this->internal_->applyTransform;
}

//-----------------------------------------------------------------------------
boost::shared_ptr<TransformManager> HDLParser::getTransformMgr() const
{
  return this->internal_->transMgr;
}

//-----------------------------------------------------------------------------
void HDLParser::setTransformMgr(
  boost::shared_ptr<TransformManager> mgr)
{
  this->internal_->transMgr = mgr;
}

//-----------------------------------------------------------------------------
void HDLParser::setDirName(const std::string& filename)
{
  if (filename == this->dirName)
    {
    return;
    }

  this->dirName = filename;
  this->unloadData();
}

//-----------------------------------------------------------------------------
const std::string& HDLParser::getCorrectionsFile()
{
  return this->correctionsFile;
}

//-----------------------------------------------------------------------------
void HDLParser::setLaserSelection(int x00, int x01, int x02, int x03, int x04, int x05, int x06, int x07,
                                        int x08, int x09, int x10, int x11, int x12, int x13, int x14, int x15,
                                        int x16, int x17, int x18, int x19, int x20, int x21, int x22, int x23,
                                        int x24, int x25, int x26, int x27, int x28, int x29, int x30, int x31,
                                        int x32, int x33, int x34, int x35, int x36, int x37, int x38, int x39,
                                        int x40, int x41, int x42, int x43, int x44, int x45, int x46, int x47,
                                        int x48, int x49, int x50, int x51, int x52, int x53, int x54, int x55,
                                        int x56, int x57, int x58, int x59, int x60, int x61, int x62, int x63)
{
  int mask[64] = {x00, x01, x02, x03, x04, x05, x06, x07,
                  x08, x09, x10, x11, x12, x13, x14, x15,
                  x16, x17, x18, x19, x20, x21, x22, x23,
                  x24, x25, x26, x27, x28, x29, x30, x31,
                  x32, x33, x34, x35, x36, x37, x38, x39,
                  x40, x41, x42, x43, x44, x45, x46, x47,
                  x48, x49, x50, x51, x52, x53, x54, x55,
                  x56, x57, x58, x59, x60, x61, x62, x63};
  this->setLaserSelection(mask);
}

//-----------------------------------------------------------------------------
void HDLParser::setLaserSelection(int laserSelection[64])
{
  for(int i = 0; i < 64; ++i)
    {
    this->internal_->laserSelections[i] = laserSelection[i];
    }
}

//-----------------------------------------------------------------------------
void HDLParser::getLaserSelection(int laserSelection[64])
{
  for(int i = 0; i < 64; ++i)
    {
    laserSelection[i] = this->internal_->laserSelections[i];
    }
}

//-----------------------------------------------------------------------------
unsigned int HDLParser::getDualReturnFilter() const
{
  return this->internal_->dualReturnFilter;
}

//-----------------------------------------------------------------------------
void HDLParser::setDualReturnFilter(unsigned int filter)
{
  if (this->internal_->dualReturnFilter != filter)
    {
    this->internal_->dualReturnFilter = filter;
    }
}

//-----------------------------------------------------------------------------
void HDLParser::getVerticalCorrections(double VerticalCorrections[64])
{
  for(int i = 0; i < 64; ++i)
    {
    VerticalCorrections[i] = this->internal_->laser_corrections_[i].verticalCorrection;
    }
}

//-----------------------------------------------------------------------------
void HDLParser::setPointsSkip(int pr)
{
  this->internal_->pointsSkip = pr;
}

//-----------------------------------------------------------------------------
void HDLParser::setNumberOfTrailingFrames(int numTrailing)
{
  assert(numTrailing >= 0);
  this->internal_->numberOfTrailingFrames = numTrailing;
}

//-----------------------------------------------------------------------------
void HDLParser::setCropReturns(int crop)
{
  if (!this->internal_->shouldCropReturns == !!crop)
    {
    this->internal_->shouldCropReturns = !!crop;
    }
}

//-----------------------------------------------------------------------------
void HDLParser::setCropInside(int crop)
{
  if (!this->internal_->shouldCropInside == !!crop)
    {
    this->internal_->shouldCropInside = !!crop;
    }
}

//-----------------------------------------------------------------------------
void HDLParser::setCropRegion(double region[6])
{
  std::copy(region, region + 6, this->internal_->cropRegion);
}

//-----------------------------------------------------------------------------
void HDLParser::setCropRegion(
  double xl, double xu, double yl, double yu, double zl, double zu)
{
  this->internal_->cropRegion[0] = xl;
  this->internal_->cropRegion[1] = xu;
  this->internal_->cropRegion[2] = yl;
  this->internal_->cropRegion[3] = yu;
  this->internal_->cropRegion[4] = zl;
  this->internal_->cropRegion[5] = zu;
}

//-----------------------------------------------------------------------------
void HDLParser::setCorrectionsFile(const std::string& correctionsFile)
{
  if (correctionsFile == this->correctionsFile)
    {
    return;
    }

  if (!boost::filesystem::exists(correctionsFile) ||
      boost::filesystem::is_directory(correctionsFile))
    {
    std::cerr << "Invalid sensor configuration file" << correctionsFile << std::endl;
    return;
    }
  this->internal_->loadCorrectionsFile(correctionsFile);

  this->correctionsFile = correctionsFile;
  this->unloadData();
}

//-----------------------------------------------------------------------------
void HDLParser::unloadData()
{
  this->internal_->lastAzimuth = -1;

  this->internal_->isDualReturnData = false;
  this->internal_->isHDL64Data = false;
  this->internal_->frames.clear();
  this->internal_->currentFrame = this->internal_->createHDLFrame();
}

//-----------------------------------------------------------------------------
void HDLParser::processHDLPacket(unsigned char *data, unsigned int bytesReceived, ptime t)
{
  this->internal_->processHDLPacket(data, bytesReceived, t);
}

//-----------------------------------------------------------------------------
std::deque<boost::shared_ptr<HDLFrame>> HDLParser::getAllFrames()
{
    return this->internal_->frames;
}

void HDLParser::clearAllFrames()
{
    this->internal_->frames.clear();
}

bool HDLParser::getFrame(boost::shared_ptr<HDLFrame> &dest, const string &filename, fpos_t &startPos, const int &skip)
{
    this->unloadData();
    if (! this->internal_->reader.open(filename)) {
        std::cerr << "failed to open packets file: " << filename << std::endl;
        return false;
    }
    if (!this->internal_->correctionsInitialized)
      {
      std::cerr << "Corrections have not been set" << std::endl;
      return false;
      }

    const unsigned char* data = 0;
    unsigned int dataLength = 0;
    ptime t;

    this->internal_->reader.setFilePosition(&startPos);
    this->internal_->firingSkip = skip;

    while (this->internal_->reader.nextPacket(data, dataLength, t))
      {
      this->processHDLPacket(const_cast<unsigned char*>(data), dataLength, t);

      if (this->internal_->frames.size())
        {
          dest->points = std::move(this->internal_->frames.back()->points);
          dest->pointsMeta
                  = std::move(this->internal_->frames.back()->pointsMeta);
//        dest.swap(this->internal_->frames.back());
        this->internal_->frames.clear();
        return true;
        }
      }

    this->internal_->splitFrame();
    dest.swap(this->internal_->frames.back());
    this->internal_->frames.clear();
    return true;
}

//-----------------------------------------------------------------------------
int HDLParser::getNumberOfChannels()
{
  return this->internal_->calibFileReportedNumLasers;
}

//-----------------------------------------------------------------------------
boost::shared_ptr<HDLFrame> HDLParser::vsInternal::createHDLFrame()
{
//    boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> >
//            data(new std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>(calibFileReportedNumLasers));
//      for(int i = 0; i < calibFileReportedNumLasers; ++i){
//          data->at(i) = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
//          data->at(i)->points.reserve(HDL_MAX_PTS_PER_LASER);
//      }
//      boost::shared_ptr<std::vector<boost::shared_ptr<std::vector<PointMeta> > > >
//              metadata(new std::vector<boost::shared_ptr<std::vector<PointMeta> > >(calibFileReportedNumLasers));
//        for(int i = 0; i < calibFileReportedNumLasers; ++i){
//            metadata->at(i) = boost::shared_ptr<std::vector<PointMeta> >(new std::vector<PointMeta>);
//            metadata->at(i)->reserve(HDL_MAX_PTS_PER_LASER);
//        }
//        boost::shared_ptr<std::vector<std::pair<boost::posix_time::ptime, std::string>>>
//                packetsdata(new std::vector<std::pair<boost::posix_time::ptime, std::string>>());
      boost::shared_ptr<HDLFrame> f(new HDLFrame);
      f->points.resize(calibFileReportedNumLasers);
      for(int i = 0; i < calibFileReportedNumLasers; ++i){
          f->points[i] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
          f->points[i]->points.reserve(HDL_MAX_PTS_PER_LASER);
      }
      f->pointsMeta.resize(calibFileReportedNumLasers);
      for(int i = 0; i < calibFileReportedNumLasers; ++i){
          f->pointsMeta[i] = boost::shared_ptr<std::vector<PointMeta> >(new std::vector<PointMeta>);
          f->pointsMeta[i]->reserve(HDL_MAX_PTS_PER_LASER);
      }
//      f->packets = packetsdata;
      f->isInMemory = true;
      frameMetaInited = false;
    return f;
}

//-----------------------------------------------------------------------------
void HDLParser::vsInternal::pushFiringData(const unsigned char laserId,
                                                       const unsigned char rawLaserId,
                                                       unsigned short azimuth,
                                                       const ptime timestamp,
                                                       const unsigned int rawtime,
                                                       const HDLLaserReturn* laserReturn,
                                                       const HDLLaserCorrection* correction,
                                                       Eigen::Affine3d* geotransform,
                                                       const bool dualReturn)
{
  azimuth %= 36000;
  //const int thisPointId = this->Points->GetNumberOfPoints();
  const short intensity = laserReturn->intensity;

  double cosAzimuth, sinAzimuth;
  if (correction->azimuthCorrection == 0)
  {
    cosAzimuth = this->cos_lookup_table_[azimuth];
    sinAzimuth = this->sin_lookup_table_[azimuth];
  }
  else
  {
    double azimuthInRadians = HDL_Grabber_toRadians((static_cast<double> (azimuth) / 100.0) - correction->azimuthCorrection);
    cosAzimuth = std::cos (azimuthInRadians);
    sinAzimuth = std::sin (azimuthInRadians);
  }

  double distanceM = laserReturn->distance * 0.002 + correction->distanceCorrection;
  double xyDistance = distanceM * correction->cosVertCorrection;

  // Compute raw position
  double pos[3] =
    {
    xyDistance * sinAzimuth - correction->horizontalOffsetCorrection * cosAzimuth,
    xyDistance * cosAzimuth + correction->horizontalOffsetCorrection * sinAzimuth,
    distanceM * correction->sinVertCorrection + correction->verticalOffsetCorrection
    };

  // Apply sensor transform
  //this->SensorTransform->InternalTransformPoint(pos, pos);

  // Test if point is cropped
  if (this->shouldCropReturns)
    {
    bool pointOutsideOfBox = pos[0] >= this->cropRegion[0] && pos[0] <= this->cropRegion[1] &&
      pos[1] >= this->cropRegion[2] && pos[1] <= this->cropRegion[3] &&
      pos[2] >= this->cropRegion[4] && pos[2] <= this->cropRegion[5];
    if ((pointOutsideOfBox && !this->shouldCropInside) ||
        (!pointOutsideOfBox && this->shouldCropInside))
      {
      return;
      }
    }

//  // Do not add any data before here as this might short-circuit
//  if (dualReturn)
//    {
//    const vtkIdType dualPointId = this->LastPointId[rawLaserId];
//    if (dualPointId < this->FirstPointIdThisReturn)
//      {
//      // No matching point from first set (skipped?)
//      this->Flags->InsertNextValue(DUAL_DOUBLED);
//      this->DistanceFlag->InsertNextValue(0);
//      this->IntensityFlag->InsertNextValue(0);
//      }
//    else
//      {
//      const short dualIntensity = this->Intensity->GetValue(dualPointId);
//      const double dualDistance = this->Distance->GetValue(dualPointId);
//      unsigned int firstFlags = this->Flags->GetValue(dualPointId);
//      unsigned int secondFlags = 0;

//      if (dualDistance == distanceM && intensity == dualIntensity)
//      {
//        // ignore duplicate point and leave first with original flags
//        return;
//      }

//      if (dualIntensity < intensity)
//        {
//        firstFlags &= ~DUAL_INTENSITY_HIGH;
//        secondFlags |= DUAL_INTENSITY_HIGH;
//        }
//      else
//        {
//        firstFlags &= ~DUAL_INTENSITY_LOW;
//        secondFlags |= DUAL_INTENSITY_LOW;
//        }

//      if (dualDistance < distanceM)
//        {
//        firstFlags &= ~DUAL_DISTANCE_FAR;
//        secondFlags |= DUAL_DISTANCE_FAR;
//        }
//      else
//        {
//        firstFlags &= ~DUAL_DISTANCE_NEAR;
//        secondFlags |= DUAL_DISTANCE_NEAR;
//        }

//      // We will output only one point so return out of this
//      if (this->DualReturnFilter)
//        {
//        if (!(secondFlags & this->DualReturnFilter))
//          {
//          // second return does not match filter; skip
//          this->Flags->SetValue(dualPointId, firstFlags);
//          this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(firstFlags));
//          this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(firstFlags));
//          return;
//          }
//        if (!(firstFlags & this->DualReturnFilter))
//          {
//          // first return does not match filter; replace with second return
//          this->Points->SetPoint(dualPointId, pos);
//          this->Distance->SetValue(dualPointId, distanceM);
//          this->Intensity->SetValue(dualPointId, laserReturn->intensity);
//          this->Timestamp->SetValue(dualPointId, timestamp);
//          this->RawTime->SetValue(dualPointId, rawtime);
//          this->Flags->SetValue(dualPointId, secondFlags);
//          this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(secondFlags));
//          this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(secondFlags));
//          return;
//          }
//        }

//      this->Flags->SetValue(dualPointId, firstFlags);
//      this->DistanceFlag->SetValue(dualPointId, MapDistanceFlag(firstFlags));
//      this->IntensityFlag->SetValue(dualPointId, MapIntensityFlag(firstFlags));
//      this->Flags->InsertNextValue(secondFlags);
//      this->DistanceFlag->InsertNextValue(MapDistanceFlag(secondFlags));
//      this->IntensityFlag->InsertNextValue(MapIntensityFlag(secondFlags));
//      }
//    }
//  else
//    {
//    this->Flags->InsertNextValue(DUAL_DOUBLED);
//    this->DistanceFlag->InsertNextValue(0);
//    this->IntensityFlag->InsertNextValue(0);
//    }

  // Apply geoposition transform
  //geotransform->InternalTransformPoint(pos, pos);
//  this->Points->InsertNextPoint(pos);
  if (geotransform)
    transformPoint(pos, *geotransform);
  pcl::PointXYZI p;
  p.x = pos[0];
  p.y = pos[1];
  p.z = pos[2];
  p.intensity = intensity;
//  if (isHDL64Data)
//    this->currentFrame->points[HDL64BeamLUT[laserId]]->points.push_back(p);
//  else
      this->currentFrame->points[laserId]->points.push_back(p);

  PointMeta m;
  m.azimuth = azimuth;
//  m.Timestamp = timestamp;
//  m.RawTime = rawtime;
  m.distance = distanceM;
//  if (isHDL64Data)
//    this->currentFrame->pointsMeta[HDL64BeamLUT[laserId]]->push_back(m);
//  else
      this->currentFrame->pointsMeta[laserId]->push_back(m);
}

//-----------------------------------------------------------------------------
void HDLParser::vsInternal::initLookUpTables()
{
  if (cos_lookup_table_.size() == 0 || sin_lookup_table_.size() == 0)
    {
    cos_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    sin_lookup_table_.resize(HDL_NUM_ROT_ANGLES);
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++)
      {
      double rad = HDL_Grabber_toRadians(i / 100.0);
      cos_lookup_table_[i] = std::cos(rad);
      sin_lookup_table_[i] = std::sin(rad);
      }
    }
}

//-----------------------------------------------------------------------------
void HDLParser::vsInternal::loadCorrectionsFile(const std::string& correctionsFile)
{
  boost::property_tree::ptree pt;
  try
    {
    read_xml(correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
    }
  catch (boost::exception const&)
    {
    std::cerr << "LoadCorrectionsFile: error reading calibration file: " << correctionsFile << std::endl;
    return;
    }

  int enabledCount = 0;
  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.enabled_"))
    {
    std::stringstream ss;
    if(v.first == "item")
      {
      ss << v.second.data();
      int test = 0;
      ss >> test;
      if(!ss.fail() && test == 1)
        {
        enabledCount++;
        }
      }
    }
  this->calibFileReportedNumLasers = enabledCount;

  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.points_"))
    {
    if (v.first == "item")
      {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH (boost::property_tree::ptree::value_type &px, points)
        {
        if (px.first == "px")
          {
          boost::property_tree::ptree calibrationData = px.second;
          int index = -1;
          double azimuth = 0;
          double vertCorrection = 0;
          double distCorrection = 0;
          double vertOffsetCorrection = 0;
          double horizOffsetCorrection = 0;

          BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData)
            {
            if (item.first == "id_")
              index = atoi(item.second.data().c_str());
            if (item.first == "rotCorrection_")
              azimuth = atof(item.second.data().c_str());
            if (item.first == "vertCorrection_")
              vertCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrection_")
              distCorrection = atof(item.second.data().c_str());
            if (item.first == "vertOffsetCorrection_")
              vertOffsetCorrection = atof(item.second.data().c_str());
            if (item.first == "horizOffsetCorrection_")
              horizOffsetCorrection = atof(item.second.data().c_str());
            }
          if (index != -1)
            {
            laser_corrections_[index].azimuthCorrection = azimuth;
            laser_corrections_[index].verticalCorrection = vertCorrection;
            laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
            laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
            laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;

            laser_corrections_[index].cosVertCorrection = std::cos (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
            laser_corrections_[index].sinVertCorrection = std::sin (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
            }
          }
        }
      }
    }

  for (int i = 0; i < 64; i++)
    {
    HDLLaserCorrection correction = laser_corrections_[i];
    laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.sinVertCorrection;
    laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.cosVertCorrection;
    }
  this->correctionsInitialized = true;
}

//-----------------------------------------------------------------------------
void HDLParser::vsInternal::init()
{
  this->initLookUpTables();
}

//-----------------------------------------------------------------------------
void HDLParser::vsInternal::splitFrame(bool force)
{
  if(this->splitCounter > 0 && !force)
    {
    this->splitCounter--;
    return;
    }
    /* update the width & height for each point cloud */
  for (auto& cloud : currentFrame->points) {
      cloud->width = cloud->points.size();
      cloud->height = 1;
  }
  /* begin debug */
  if (isHDL64Data)
  {
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> re_arranged_pts;
      std::vector<boost::shared_ptr<std::vector<PointMeta> > > re_arranged_ptm;
      re_arranged_pts.resize(64);
      re_arranged_ptm.resize(64);
      for (int i = 0; i < 64; ++i)
      {
          re_arranged_pts[i] = currentFrame->points[HDL64BeamLUT[i]];
          re_arranged_ptm[i] = currentFrame->pointsMeta[HDL64BeamLUT[i]];
      }
      currentFrame->points = std::move(re_arranged_pts);
      currentFrame->pointsMeta = std::move(re_arranged_ptm);
  }
  /* end debug */
  this->frames.push_back(this->currentFrame);
  this->currentFrame = this->createHDLFrame();
}

//-----------------------------------------------------------------------------
void HDLParser::vsInternal::processFiring(HDLFiringData* firingData,
                                                      int hdl64offset,
                                                      int firingBlock,
                                                      int azimuthDiff,
                                                      ptime timestamp,
                                                      unsigned int rawtime,
                                                      Eigen::Affine3d* geotransform)
{
  const bool dual = (this->lastAzimuth == firingData->rotationalPosition) &&
    (!this->isHDL64Data);

  if (!dual)
    {
    //this->FirstPointIdThisReturn = this->Points->GetNumberOfPoints();
    }

  if (dual && !this->isDualReturnData)
    {
    this->isDualReturnData = true;
    //this->CurrentDataset->GetPointData()->AddArray(this->DistanceFlag.GetPointer());
    //this->CurrentDataset->GetPointData()->AddArray(this->IntensityFlag.GetPointer());
    }

  for (int dsr = 0; dsr < HDL_LASER_PER_FIRING; dsr++)
    {
    unsigned char rawLaserId = static_cast<unsigned char>(dsr + hdl64offset);
    unsigned char laserId = rawLaserId;
//    unsigned short azimuth = firingData->rotationalPosition;
    /* debug */
    unsigned short azimuth = firingData->rotationalPosition;
    /* end debug */

    // Detect VLP-16 data and adjust laser id if necessary
    int firingWithinBlock = 0;

    if(this->calibFileReportedNumLasers == 16)
      {
      assert(hdl64offset == 0);
      if(laserId >= 16)
        {
        laserId -= 16;
        firingWithinBlock = 1;
        }
      }

    // Interpolate azimuth
    double timestampadjustment = 0.0;
    double blockdsr0 = 0.0;
    double nextblockdsr0 = 1.0;
    if(this->calibFileReportedNumLasers == 32)
      {
      timestampadjustment = HDL32AdjustTimeStamp(firingBlock, dsr);
      nextblockdsr0 = HDL32AdjustTimeStamp(firingBlock+1,0);
      blockdsr0 = HDL32AdjustTimeStamp(firingBlock,0);
      }
    else if(this->calibFileReportedNumLasers == 16)
      {
      timestampadjustment = VLP16AdjustTimeStamp(firingBlock, laserId, firingWithinBlock);
      nextblockdsr0 = VLP16AdjustTimeStamp(firingBlock+1,0,0);
      blockdsr0 = VLP16AdjustTimeStamp(firingBlock,0,0);
      }
    int azimuthadjustment = std::round(azimuthDiff * ((timestampadjustment - blockdsr0) / (nextblockdsr0 - blockdsr0)));
    timestampadjustment = std::round(timestampadjustment);

    if (firingData->laserReturns[dsr].distance != 0.0 && this->laserSelections[laserId])
      {
      this->pushFiringData(laserId,
                           rawLaserId,
                           azimuth + azimuthadjustment,
                           timestamp + time_duration(0,0,0,timestampadjustment),
                           rawtime + static_cast<unsigned int>(timestampadjustment),
                           &(firingData->laserReturns[dsr]),
                           &(laser_corrections_[dsr + hdl64offset]),
                           geotransform,
                           dual);
      }
    }
}

//-----------------------------------------------------------------------------
void HDLParser::vsInternal::processHDLPacket(unsigned char *data, std::size_t bytesReceived, ptime timestamp)
{
  if (bytesReceived != 1206)
    {
    return;
    }

  HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket *>(data);

  boost::shared_ptr<PoseTransform> transform(new PoseTransform);
  const uint32_t rawtime = dataPacket->gpsTimestamp;

  this->transMgr->interpolateTransform(timestamp, transform.get());
  if (!frameMetaInited) {
      // set the new currentFrame's meta data
      std::memcpy((void*) currentFrame->carpose.get(), (void*) transform.get(), sizeof(PoseTransform));
//      currentFrame->carpose = transform;
      currentFrame->timestamp = timestamp;
      currentFrame->skips = firingSkip;
      currentFrame->packets.push_back(std::make_pair(timestamp, std::string(reinterpret_cast<char *>(data), bytesReceived)));
      frameMetaInited = true;
  }
  transform->timestamp = timestamp;
  boost::shared_ptr<Eigen::Affine3d> geotransform;
  if (transform->seconds_pos != -1) { // FIXME: once a graceful invalid transform check is devised, modify this code
      reprojectToFrameBeginning(transform.get());
      geotransform = boost::shared_ptr<Eigen::Affine3d>(new Eigen::Affine3d(transform->getMatrix()));
  }
  // push the raw packet contents into current frame to make it offline capable
  currentFrame->packets.push_back(std::make_pair(timestamp, std::string(reinterpret_cast<char *>(data), bytesReceived)));

//  int firingBlock = this->firingSkip;
//  this->firingSkip = 0;
  int firingBlock = firingSkip;
  firingSkip = 0;

  std::vector<int> diffs (HDL_FIRING_PER_PKT - 1);
  for(int i = 0; i < HDL_FIRING_PER_PKT - 1; ++i)
    {
    int localDiff = (36000 + dataPacket->firingData[i+1].rotationalPosition -
                     dataPacket->firingData[i].rotationalPosition) % 36000;
    diffs[i] = localDiff;
    }
  std::nth_element(diffs.begin(),
                   diffs.begin() + HDL_FIRING_PER_PKT/2,
                   diffs.end());
  int azimuthDiff = diffs[HDL_FIRING_PER_PKT/2];
  assert(azimuthDiff >= 0);

  for ( ; firingBlock < HDL_FIRING_PER_PKT; ++firingBlock)
    {
    HDLFiringData* firingData = &(dataPacket->firingData[firingBlock]);
    int hdl64offset = (firingData->blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;
    this->isHDL64Data |= (hdl64offset > 0);

    if (firingData->rotationalPosition < this->lastAzimuth){
        firingSkip = firingBlock;
        // push current frame to frames pool, and create a new currentFrame
        this->splitFrame();
    }

    // Skip this firing every PointSkip
    if(this->pointsSkip == 0 || firingBlock % (this->pointsSkip + 1) == 0)
      {
      this->processFiring(firingData,
                          hdl64offset,
                          firingBlock,
                          azimuthDiff,
                          timestamp,
                          rawtime,
                          geotransform.get());
      }

    this->lastAzimuth = firingData->rotationalPosition;
  }
}

void HDLParser::vsInternal::reprojectToFrameBeginning(PoseTransform *trans)
{
    for (int i = 0; i < 3; ++i) {
        trans->T[i] -= currentFrame->carpose->T[i];
    }
}

//-----------------------------------------------------------------------------
std::vector<boost::shared_ptr<HDLFrame>> HDLParser::readFrameInformation(const std::string& name, bool touchOnly)
{
    std::vector<boost::shared_ptr<HDLFrame>> result;
    vtkPacketFileReader reader;
    if (!reader.open(name))
    {
        std::cerr << "Failed to open packet file: " << name
                  << std::endl << reader.getLastError() << std::endl;
        return std::move(result);
    }
    /* check that filename is a valid ptime string. If not, the filename will be
     * converted to a string corresponding to the first packet's ptime */
    boost::filesystem::path p(name);
    std::string s = p.filename().stem().string();
    ptime t;
    try {
        t = boost::posix_time::from_iso_string(s);
    } catch (boost::exception& e) {
        std::cout << "Filename is not a valid ptime string, "
                     "I'll convert it into a ptime string.\n";
    }

    const unsigned char* data = 0;
    unsigned int dataLength = 0;
    ptime packetTime, filenameTime;

    unsigned int lastAzimuth = 0;
    unsigned int lastTimestamp = 0;
    uint64_t timeCursor = 0, currentFrameBegin = 0;

    fpos_t lastFilePosition;
    result.push_back(boost::shared_ptr<HDLFrame>(new HDLFrame));
    reader.getFilePosition(&lastFilePosition);

    result.back()->fileStartPos = lastFilePosition;
    result.back()->skips = 0;
    result.back()->isOnHardDrive = true;
    result.back()->carpose = boost::shared_ptr<PoseTransform>(new PoseTransform);

    while (reader.nextPacket(data, dataLength, packetTime))
    {

        if (dataLength != 1206) continue;

        /* if the first packet (filenameTime.is_special() == true), set filenameTime accordingly */
        if (filenameTime.is_special()) {
            filenameTime = packetTime;
            result.back()->timestamp = packetTime;
            if (touchOnly) break;
        }

        const HDLDataPacket* dataPacket = reinterpret_cast<const HDLDataPacket *>(data);

        //    unsigned int timeDiff = dataPacket->gpsTimestamp - lastTimestamp;
        //    if (timeDiff > 600 && lastTimestamp != 0)
        //      {
        //      printf("missed %d packets\n",  static_cast<int>(floor((timeDiff/553.0) + 0.5)));
        //      }

        for (int i = 0; i < HDL_FIRING_PER_PKT; ++i)
        {
            HDLFiringData firingData = dataPacket->firingData[i];

            if (firingData.rotationalPosition < lastAzimuth)
            {
                result.push_back(boost::shared_ptr<HDLFrame>(new HDLFrame));
                result.back()->fileStartPos = lastFilePosition;
                result.back()->skips = i;
                result.back()->isOnHardDrive = true;
                result.back()->timestamp = packetTime;
                result.back()->carpose = boost::shared_ptr<PoseTransform>(new PoseTransform);
            }

            lastAzimuth = firingData.rotationalPosition;
        }

        lastTimestamp = dataPacket->gpsTimestamp;
        currentFrameBegin = timeCursor++;
        reader.getFilePosition(&lastFilePosition);
    }

    /* set all frames' filenameTime */
    for (auto& f : result) {
        f->filenameTime = filenameTime;
    }

    /* rename the file if necessary */
    if (t.is_special()) {
        std::string newname = p.parent_path().string() + "/" + to_iso_string(filenameTime) + ".pcap";
        boost::filesystem::path newp(newname);
        boost::filesystem::rename(p, newp);
        std::cout << "The original filename: '" << name
                  << "' was converted to: '" << newname << '\'' << std::endl;
    }
    return std::move(result);
}


void HDLParser::setHDLManager(boost::shared_ptr<HDLManager> p)
{
    this->internal_->hdlMgr = p;
}
