#ifndef __MapManager_H__
#define __MapManager_H__

#include "MapPatch.h"
#include <vector>
#include <set>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

/* ROI_RANGE is not a good name, but I couldn't think of a better name for it
 * right now. It's purpose is to indicate the detecting range of the sensors
 * like LiDAR or LUX */
#define ROI_RANGE 100

class MapManager
{
    MapManager();
    ~MapManager();
    /* The difference between getPatch() and findPatch() is that getPatch() will
     * create a patch if it does not exist, while findPatch() only returns a
     * void boost::shared_ptr */
    boost::shared_ptr<MapPatch> getPatch(double x, double y);
    inline boost::shared_ptr<MapPatch> findPatch(double x, double y);
    /* region of interest is defined as such: the sufficient map patch(es) that
     * could cover the current detecting range */
    std::set<boost::shared_ptr<MapPatch> > getROI(double x, double y);
private:
    float range, patchRange;
    double centerX, centerY;
    std::vector<boost::shared_ptr<MapPatch>> mapPatches;
    /* 'indices' will be used to index into the vector of 'mapPatches'. Each
     * time someone query a specific map patch, we first find the corresponding
     * index in 'indices'.
     * The 'indices' is interpreted as such: the first byte is the boolean value
     * to indicate whether the patch is actually present in 'mapPatches' or not,
     * '0' for 'not present', '255' for 'present'. If it does present, then the
     * last two bytes is interpreted as an unsigned short revealing the offset
     *  position inside 'mapPatches' */
    cv::Mat indices;
    /* Given a coordinate (x,y) find the corresponding map center it should
     * belong */
    std::pair<double, double> getMapCenter(double x, double y);
    /* Given a coordinate (x,y) find the index number of the map patch */
    inline std::pair<int, int> getPatchIdx(double x, double y);
    boost::shared_ptr<MapPatch> createPatch(double x, double y);
    friend std::ofstream& operator<<(std::ofstream& os, const MapManager& obj);
    friend std::ifstream& operator>>(std::ifstream& is, MapManager& obj);
};

std::ofstream& operator<<(std::ofstream& os, const MapManager& obj);
std::ifstream& operator>>(std::ifstream& is, MapManager& obj);

#endif // __MapManager_H__
