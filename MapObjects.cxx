#include "MapObjects.h"

std::ofstream &operator<<(std::ofstream &os, const UprightPost &obj)
{
    os.write(reinterpret_cast<const char*>(&obj.x), sizeof(double));
    os.write(reinterpret_cast<const char*>(&obj.y), sizeof(double));
    os.write(reinterpret_cast<const char*>(&obj.z), sizeof(double));
    os.write(reinterpret_cast<const char*>(&obj.height), sizeof(float));
    os.write(reinterpret_cast<const char*>(&obj.radius), sizeof(float));
    return os;
}

std::ifstream &operator>>(std::ifstream &is, UprightPost &obj)
{
    is.read(reinterpret_cast<char*>(&obj.x), sizeof(double));
    is.read(reinterpret_cast<char*>(&obj.y), sizeof(double));
    is.read(reinterpret_cast<char*>(&obj.z), sizeof(double));
    is.read(reinterpret_cast<char*>(&obj.height), sizeof(float));
    is.read(reinterpret_cast<char*>(&obj.radius), sizeof(float));
    return is;
}

std::ofstream &operator<<(std::ofstream &os, const Plane &obj)
{
    os.write(reinterpret_cast<const char*>(obj.coeffi), sizeof(float) * 4);
    os.write(reinterpret_cast<const char*>(obj.x), sizeof(double) * 4);
    os.write(reinterpret_cast<const char*>(obj.y), sizeof(double) * 4);
    os.write(reinterpret_cast<const char*>(obj.z), sizeof(double) * 4);
    return os;
}

std::ifstream &operator>>(std::ifstream &is, Plane &obj)
{
    is.read(reinterpret_cast<char*>(obj.coeffi), sizeof(float) * 4);
    is.read(reinterpret_cast<char*>(obj.x), sizeof(double) * 4);
    is.read(reinterpret_cast<char*>(obj.y), sizeof(double) * 4);
    is.read(reinterpret_cast<char*>(obj.z), sizeof(double) * 4);
    return is;
}

std::ofstream &operator<<(std::ofstream &os, const Complex &obj)
{
    os.write(reinterpret_cast<const char*>(obj.center), sizeof(double) * 3);
    os.write(reinterpret_cast<const char*>(obj.bounding), sizeof(float) * 3);
    unsigned short sz = (unsigned short) obj.points.size();
    os.write(reinterpret_cast<const char*>(&sz), sizeof(unsigned short));
    for (unsigned short i = 0; i < sz; ++i)
    {
        os.write(reinterpret_cast<const char*>(&obj.points[i]), sizeof(float));
    }
    return os;
}

std::ifstream &operator>>(std::ifstream &is, Complex &obj)
{
    is.read(reinterpret_cast<char*>(obj.center), sizeof(double) * 3);
    is.read(reinterpret_cast<char*>(obj.bounding), sizeof(float) * 3);
    unsigned short sz;
    is.read(reinterpret_cast<char*>(&sz), sizeof(unsigned short));
    obj.points.resize(sz);
    for (int i = 0; i < sz; ++i)
    {
        is.read(reinterpret_cast<char*>(&obj.points[i]), sizeof(float));
    }
    return is;
}

std::ofstream &operator<<(std::ofstream &os, const GroundLineMark &obj)
{
    os.write(reinterpret_cast<const char*>(obj.x), sizeof(double) * 2);
    os.write(reinterpret_cast<const char*>(obj.y), sizeof(double) * 2);
    os.write(reinterpret_cast<const char*>(obj.z), sizeof(double) * 2);
    os.write(reinterpret_cast<const char*>(&obj.width), sizeof(float));
    return os;
}

std::ifstream &operator>>(std::ifstream &is, GroundLineMark &obj)
{
    is.read(reinterpret_cast<char*>(obj.x), sizeof(double) * 2);
    is.read(reinterpret_cast<char*>(obj.y), sizeof(double) * 2);
    is.read(reinterpret_cast<char*>(obj.z), sizeof(double) * 2);
    is.read(reinterpret_cast<char*>(&obj.width), sizeof(float));
    return is;
}
