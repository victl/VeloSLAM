#ifndef __type_defs_h__
#define __type_defs_h__

#include <inttypes.h>
#include <boost/date_time.hpp>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>

using namespace boost::posix_time;

#define HDL_NUM_ROT_ANGLES 36001
#define HDL_LASER_PER_FIRING 32
#define HDL_MAX_NUM_LASERS 64
#define HDL_FIRING_PER_PKT 12
#define HDL_MAX_PTS_PER_LASER 2000
#define WEEK_IN_MICROSECONDS 604800000000  // 1e6 * 3600 * 24 * 7
#define HDL_META_EXT_NAME ".hdlmeta"
#define INS_META_EXT_NAME ".insmeta"

#define TO_RADIUS(degree) ((degree) * M_PI / 180)

struct INSData {
    double T[3];
    double R[3];
    double timestamp;
};

enum PackageType {INSPVA = 508,
      RAWINS = 325,
      BESTGPSPOS = 423};

//normal 10ms id 508
typedef struct InsPVA
{
    uint16_t message_id;
    uint16_t week_number;

    /* Because the definition of ULong on the INS system is 32bits long,
     * so we choose uint32_t from standard system header "inttypes.h" to
     * represent ULong instead of "uint32_t", which could cause
     * problems on 64bit systems. The header file "inttypes.h" is portable
     * at least on most Linux systems and Mac OS X, Windows platforms sould
     * also be ok, but have not been tested at present. - 7/1/2016
     */
    uint32_t milliseconds;
    uint32_t week_number_pos;
    double seconds_pos;
    double LLH[3];
    double V[3];
    double Eulr[3];
    int32_t ins_status;
}InsPVA;

//unknown id 325
typedef struct RawINS
{
    uint16_t message_id;
    uint16_t week_number;
    uint32_t milliseconds;
    int32_t imu_status;
    int32_t Accel[3];
}RawINS;

//1s id 423
typedef struct BestGPSPos
{
    uint16_t message_id;
    uint16_t week_number;
    uint32_t milliseconds;
    int32_t solution;
    int32_t gps_status;
    float diff_age;
}BestGPSPos;


struct PoseMatrix{
    //TODO: define a 4 by 4 transform matrix
};

struct PoseTransform {
    //TODO: change design of this struct
    double T[3];
    double R[3];
    double V[3];
    boost::posix_time::ptime timestamp;
    uint16_t week_number;
    uint32_t milliseconds;
    uint32_t week_number_pos;
    double seconds_pos;

    PoseTransform();

    /* Note: the operators defined here didn't deal with this->timestamp.
     * This member variable should be assigned a valid value explicitly.
     */
    PoseTransform operator+ (PoseTransform delta) {
        PoseTransform result;
        for (int i = 0; i < 3; ++i) {
            result.T[i] = this->T[i] + delta.T[i];
            result.R[i] = this->R[i] + delta.R[i];
            result.V[i] = this->V[i] + delta.V[i];
        }
        return result;
    }
    PoseTransform operator* (double ratio) {
        PoseTransform result;
        for (int i = 0; i < 3; ++i) {
            result.T[i] = this->T[i] * ratio;
            result.R[i] = this->R[i] * ratio;
            result.V[i] = this->V[i] * ratio;
        }
        return result;
    }
    PoseTransform operator- (PoseTransform delta) {
        PoseTransform result;
        for (int i = 0; i < 3; ++i) {
            result.T[i] = this->T[i] - delta.T[i];
            result.R[i] = this->R[i] - delta.R[i];
            result.V[i] = this->V[i] - delta.V[i];
        }
        return result;
    }

    // Turn PoseTransform into Eigen's transform matrix
    Eigen::Affine3d getMatrix() {
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.rotate (Eigen::AngleAxisd (TO_RADIUS(R[0]), Eigen::Vector3d::UnitY()));
        transform.rotate (Eigen::AngleAxisd (TO_RADIUS(R[1]), Eigen::Vector3d::UnitX()));
        transform.rotate (Eigen::AngleAxisd (TO_RADIUS(R[2]), Eigen::Vector3d::UnitZ()));
        transform.translation() << T[0], T[1], T[2];
        return transform;
    }
};

// Write PoseTransform to file
std::ofstream& operator<<(std::ofstream& os, const PoseTransform& obj);
// Read PoseTransform from file
std::ifstream& operator>>(std::ifstream& is, PoseTransform& obj);

// Write PoseTransform to std::cout for DEBUG
std::ostream& operator<<(std::ostream& os, const PoseTransform& obj);

// this input function is automatically dealt with the template
//std::ifstream &operator>>(std::ifstream &is, TimeLine<PoseTransform> &obj);

template <typename Scalar> void
transformPoint (double pt0[3], const Eigen::Transform<Scalar, 3, Eigen::Affine> &transform) {
    Eigen::Matrix<Scalar, 3, 1> pt (pt0[0], pt0[1], pt0[2]);
    pt0[0] = static_cast<double> (transform (0, 0) * pt.coeffRef (0) + transform (0, 1) * pt.coeffRef (1) + transform (0, 2) * pt.coeffRef (2) + transform (0, 3));
    pt0[1] = static_cast<double> (transform (1, 0) * pt.coeffRef (0) + transform (1, 1) * pt.coeffRef (1) + transform (1, 2) * pt.coeffRef (2) + transform (1, 3));
    pt0[2] = static_cast<double> (transform (2, 0) * pt.coeffRef (0) + transform (2, 1) * pt.coeffRef (1) + transform (2, 2) * pt.coeffRef (2) + transform (2, 3));
}

struct PointMeta {
    unsigned short azimuth;
    float distance;
//    boost::posix_time::ptime Timestamp;
//    unsigned int RawTime;
    unsigned char intensityFlag;
    unsigned char distanceFlag;
    unsigned char flags;
};

void timevalToPtime(struct timeval& tv, ptime& t);
void ptimeToWeekMilli(ptime& t, uint16_t& week, uint32_t &milli);
void ptimeToTimeval(ptime& t, struct timeval& tv);

#endif   //__type_defs_h__
