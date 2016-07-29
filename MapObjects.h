#ifndef __MapObjects_H__
#define __MapObjects_H__

#include <vector>
#include <fstream>

/* The map objects defined here are all for storing HDL detected features. There
 * are also specific vectorized maps that were obtained from map companys, which
 * should be dealt separately */

struct UprightPost
{
    /* 'z' is the lower end point's height */
    double x, y, z;
    float height, radius;
};

struct Plane
{
    /* coefficients A, B, C, D in 'Ax + By + Cz +D = 0' */
    float coeffi[4];
    /* coordinates of the four corner points */
    double x[4];
    double y[4];
    double z[4];
};

struct Complex
{
    /* 3 coords correspond to (x,y,z) */
    double center[3];
    /* 3 value correspond to (width, length, height) */
    float bounding[3];
    /* points in (x1,y1,z1,x2,y2,z2,...) sequence */
    std::vector<float> points;
};

/* This struct is mainly used as HDL detected lanelines, stoplines, etc. */
struct GroundLineMark
{
    /* 2 end points */
    double x[2];
    double y[2];
    double z[2];
    float width;
};

/* I/O operators for various map objects */
std::ofstream& operator<<(std::ofstream& os, const UprightPost& obj);
std::ifstream& operator>>(std::ifstream& is, UprightPost& obj);

std::ofstream& operator<<(std::ofstream& os, const Plane& obj);
std::ifstream& operator>>(std::ifstream& is, Plane& obj);

std::ofstream& operator<<(std::ofstream& os, const Complex& obj);
std::ifstream& operator>>(std::ifstream& is, Complex& obj);

std::ofstream& operator<<(std::ofstream& os, const GroundLineMark& obj);
std::ifstream& operator>>(std::ifstream& is, GroundLineMark& obj);


#endif // __MapObjects_H__
