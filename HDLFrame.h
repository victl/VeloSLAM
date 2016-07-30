#ifndef __HDLFRMAE_H__
#define __HDLFRMAE_H__
#include "TimeLine.h"
#include <boost/shared_ptr.hpp>
#include <boost/date_time.hpp>
#include <pcap.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <array>
#include <vector>
#include "type_defs.h"

struct HDLFrame{
    HDLFrame();
    ~HDLFrame();
    ptime timestamp;
    /* if swaped out of memory, reset the following 3 variables; */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> points;
    std::vector<boost::shared_ptr<std::vector<PointMeta> > > pointsMeta;
    /* the original packets are stored here, the purpose is to store them into
     * hard drive when memory is insufficient. */
    std::vector<std::pair<boost::posix_time::ptime, std::string>> packets;
    /* mainly used for visualizing */
    boost::shared_ptr<PoseTransform> carpose;
    /* in memory indicator. (could check the validity of this->points for the
     * same purpose, but that will be less intuitive/readable) */
    bool isInMemory;
    bool isOnHardDrive;

    /* the 'count' is used by boost::intrusive_ptr(), which the interface to
     * End Users. When all end users have finished using certain frame, its
     * count becomes 0, which tells memory manager (that is, the
     * HDLManager.updateCacheSize() method) its points could be safely removed
     * from memory via HDLFrame.clear() */
    unsigned char count;
    /* The file name in which this frame is stored. The type 'ptime' could
     * easily convertible to string, which is exactly the name of file. The
     * directory in which this file reside should be specified elsewhere. */
    boost::posix_time::ptime filenameTime;
    /* offset in that file */
    fpos_t fileStartPos;
    /* because files are stored in a unit of packets, which contains 12 block
     * firing each. The  begining of current frame may not be the first firing
     * block. Hence the need for a 'skips' variable. This variable is set by
     * a HDLParser */
    uint8_t skips;

    /* methods primarily used for debuging */
    void printSelf();
    void dumpToFiles(std::string dirname);
    /* beamId = -1 means dump all points, reguardless of which beam they came
     * from */
    void dumpToImage(std::string dirname, double gridSize, int beamId = -1);
    void dumpToPCD(std::string dirname, int beamId = -1);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getPointsAsOneCloud(int startBeam = 0
            , int endBeam = 64);

    /* This function should explicitly RELEASE all resources hold by points,
     * pointsMeta, and packets. Leaving only a 'stub' directing which file to
     * restore all the infos. MEMORY MUST BE CLEANED UP. This is a very error
     * prone task, as shared_ptrs are used. Other ojects elsewhere might also
     * hold this ptr. I've tested that because only the outer most share_ptr is
     * actually shared between different objects, so reseting the inner
     * shared_ptr WILL clean up the memory, try this code, note the memory
     * consumption before and after the program outputed "reseted": */
    /*
    typedef vector<boost::shared_ptr<vector<long long>>> HdlPoints;

    int main() {
        boost::shared_ptr<HdlPoints> points(new HdlPoints());
        {
            boost::shared_ptr<std::vector<long long>> vec(new std::vector<long long>(4e8));
            points->push_back(vec);
        }
        boost::shared_ptr<HdlPoints> anotherHolder = points;
        boost::this_thread::sleep_for(boost::chrono::seconds(10));
        points->front().reset();
        cout << "reseted" << endl;
        boost::this_thread::sleep_for(boost::chrono::seconds(100));
    }
     */
    void clear();
};

/* The following two functions are required by boost::intrusive_ptr() */
void intrusive_ptr_add_ref(HDLFrame* p);
void intrusive_ptr_release(HDLFrame* p);

std::ofstream& operator<<(std::ofstream& os, const HDLFrame& obj);

std::ifstream& operator>>(std::ifstream& is, HDLFrame& obj);

/* these I/O function is automatically dealt with the template, so they are
 * nolonger necessary */
//std::ofstream& operator<<(std::ofstream& os, TimeLine<HDLFrame>& obj);

//ifstream &operator>>(ifstream &is, TimeLine<HDLFrame> &obj);

// Instantiate the template struct
//typedef TimeData<HDLFrame> HDLFrame;
//struct HDLFrame : public TimeData<HDLFrame_t> {

//};

#endif  //__HDLFRMAE_H__
