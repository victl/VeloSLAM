#include "MapPatch.h"

std::ofstream &operator<<(std::ofstream &os, const MapPatch &obj)
{
    os.write(reinterpret_cast<const char*>(&obj.centerX), sizeof(double));
    os.write(reinterpret_cast<const char*>(&obj.centerY), sizeof(double));
    os.write(reinterpret_cast<const char*>(&obj.range), sizeof(float));
    unsigned short sz = (unsigned short) obj.posts.size();
    os.write(reinterpret_cast<const char*>(&sz), sizeof(unsigned short));
    for (unsigned short i = 0; i < sz; ++i)
    {
        os << obj.posts[i];
    }
    sz = (unsigned short) obj.planes.size();
    os.write(reinterpret_cast<const char*>(&sz), sizeof(unsigned short));
    for (unsigned short i = 0; i < sz; ++i)
    {
        os << obj.planes[i];
    }
    sz = (unsigned short) obj.marks.size();
    os.write(reinterpret_cast<const char*>(&sz), sizeof(unsigned short));
    for (unsigned short i = 0; i < sz; ++i)
    {
        os << obj.marks[i];
    }
    sz = (unsigned short) obj.cplxes.size();
    os.write(reinterpret_cast<const char*>(&sz), sizeof(unsigned short));
    for (unsigned short i = 0; i < sz; ++i)
    {
        os << obj.cplxes[i];
    }
    return os;
}

std::ifstream &operator>>(std::ifstream &is, MapPatch &obj)
{
    is.read(reinterpret_cast<char*>(&obj.centerX), sizeof(double));
    is.read(reinterpret_cast<char*>(&obj.centerY), sizeof(double));
    is.read(reinterpret_cast<char*>(&obj.range), sizeof(float));
    unsigned short sz = (unsigned short) obj.posts.size();
    is.read(reinterpret_cast<char*>(&sz), sizeof(unsigned short));
    obj.posts.resize(sz);
    for (unsigned short i = 0; i < sz; ++i)
    {
        is >> obj.posts[i];
    }
    sz = (unsigned short) obj.planes.size();
    is.read(reinterpret_cast<char*>(&sz), sizeof(unsigned short));
    obj.planes.resize(sz);
    for (unsigned short i = 0; i < sz; ++i)
    {
        is >> obj.planes[i];
    }
    sz = (unsigned short) obj.marks.size();
    is.read(reinterpret_cast<char*>(&sz), sizeof(unsigned short));
    obj.marks.resize(sz);
    for (unsigned short i = 0; i < sz; ++i)
    {
        is >> obj.marks[i];
    }
    sz = (unsigned short) obj.cplxes.size();
    is.read(reinterpret_cast<char*>(&sz), sizeof(unsigned short));
    obj.cplxes.resize(sz);
    for (unsigned short i = 0; i < sz; ++i)
    {
        is >> obj.cplxes[i];
    }
    return is;
}

MapPatch::MapPatch(double x, double y, float r)
    : centerX(x)
    , centerY(y)
    , range(r)
{

}

MapPatch::~MapPatch()
{

}
