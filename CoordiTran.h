#ifndef __COORDITRAN_H_
#define __COORDITRAN_H_

#include <math.h>
#include <iostream>

using namespace std;

void eulr2dcm(double eul_vect[3],double DCMbn[3][3]);
void llh2xyz(double llh[3],double xyz[3]);
void xyz2llh(double xyz [3],double llh [3]);
void xyz2enu(double xyz[3],double orgxyz[3],double enu[3]);
void enu2xyz(double enu[3],double orgxyz[3],double xyz[3]);
void HDL2enu(double Target_HDL[3],double eulr[3],double Car_Ref[3],double enu[3]);
void enu2llh(double enu[3],double orgxyz[3],double llh[3]);
void llh2enu(double llh[3],double orgxyz[3],double enu[3]);
double MappingAngle(double angle);

#endif
