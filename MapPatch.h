#ifndef __MapPatch_H__
#define __MapPatch_H__

#include "MapObjects.h"
#include <vector>

struct MapPatch
{
    MapPatch(double x = 0, double y = 0, float r = 0);
    ~MapPatch();
    float range;
    double centerX, centerY;
    std::vector<UprightPost> posts;
    std::vector<Plane> planes;
    std::vector<GroundLineMark> marks;
    std::vector<Complex> cplxes;
};

std::ofstream& operator<<(std::ofstream& os, const MapPatch& obj);
std::ifstream& operator>>(std::ifstream& is, MapPatch& obj);

#endif // __MapPatch_H__
