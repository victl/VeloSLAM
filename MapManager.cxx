#include "MapManager.h"
#include <cmath>

MapManager::MapManager()
    : range(0)
    , patchRange(0)
    , centerX(0)
    , centerY(0)
{

}

MapManager::~MapManager()
{

}

boost::shared_ptr<MapPatch> MapManager::getPatch(double x, double y)
{
    auto p = findPatch(x, y);
    if (p) return p;
    else return createPatch(x, y);
}

boost::shared_ptr<MapPatch> MapManager::findPatch(double x, double y)
{
    auto idx = getPatchIdx(x, y);
    cv::Vec3b val = indices.at<cv::Vec3b>(idx.second, idx.first);
    if (val[0] == 0) return boost::shared_ptr<MapPatch>();
    unsigned short* ptr = reinterpret_cast<unsigned short*>(&val[1]);
    return mapPatches[*ptr];
}

std::set<boost::shared_ptr<MapPatch> > MapManager::getROI(double x, double y)
{
    std::set<boost::shared_ptr<MapPatch> > result;
    for (double cornerX : {x + ROI_RANGE, x - ROI_RANGE})
    {
        for (double cornerY : {y + ROI_RANGE, y- ROI_RANGE})
        {
            auto p = findPatch(cornerX, cornerY);
            if (p) result.insert(p);
        }
    }
    return std::move(result);
}

std::pair<double, double> MapManager::getMapCenter(double x, double y)
{
    int x_idx = std::floor((x + range / 2) / range);
    int y_idx = std::floor((y + range / 2) / range);
    return std::make_pair<double, double>(x_idx * range, y_idx * range);
}

std::pair<int, int> MapManager::getPatchIdx(double x, double y)
{
    int x_idx = std::floor(x - (centerX - range / 2)) / patchRange;
    int y_idx = std::floor(y - (centerY - range / 2)) / patchRange;
    return std::make_pair(x_idx, y_idx);
}

boost::shared_ptr<MapPatch> MapManager::createPatch(double x, double y)
{
    /* In principle, this function is designed to be a private method. Only be
     * called when no valid patch for current (x,y) exists, so checking before
     * creating is not necessary. But for safty (and the possibility of making
     * it a public method), validity is checked again here */
    auto idx = getPatchIdx(x, y);
    cv::Vec3b val = indices.at<cv::Vec3b>(idx.second, idx.first);
    unsigned short* ptr = reinterpret_cast<unsigned short*>(&val[1]);
    if (val[0] == 0) /* means no map patch currently exist for that idx */
    {
        boost::shared_ptr<MapPatch> mp(new MapPatch);
        mapPatches.push_back(mp);
        val[0] = 255;
        *ptr = mapPatches.size() - 1;
    }
    return mapPatches[*ptr];
}

std::ofstream &operator<<(std::ofstream &os, const MapManager &obj)
{
    os.write(reinterpret_cast<const char*>(&obj.centerX), sizeof(double));
    os.write(reinterpret_cast<const char*>(&obj.centerY), sizeof(double));
    os.write(reinterpret_cast<const char*>(&obj.range), sizeof(float));
    os.write(reinterpret_cast<const char*>(&obj.patchRange), sizeof(float));
    unsigned short sz = (unsigned short) obj.mapPatches.size();
    os.write(reinterpret_cast<const char*>(&sz), sizeof(unsigned short));
    for (unsigned short i = 0; i < sz; ++i)
    {
        os << *obj.mapPatches[i];
    }
    return os;
}

std::ifstream &operator>>(std::ifstream &is, MapManager &obj)
{
    is.read(reinterpret_cast<char*>(&obj.centerX), sizeof(double));
    is.read(reinterpret_cast<char*>(&obj.centerY), sizeof(double));
    is.read(reinterpret_cast<char*>(&obj.range), sizeof(float));
    is.read(reinterpret_cast<char*>(&obj.patchRange), sizeof(float));
    unsigned short sz = (unsigned short) obj.mapPatches.size();
    is.read(reinterpret_cast<char*>(&sz), sizeof(unsigned short));
    obj.mapPatches.resize(sz);
    for (unsigned short i = 0; i < sz; ++i)
    {
        is >> *obj.mapPatches[i];
    }
    return is;
}
