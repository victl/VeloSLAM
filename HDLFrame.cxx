#include "HDLFrame.h"
#include <cstdlib>
#include <glog/logging.h>

HDLFrame::HDLFrame()
    : isInMemory(false)
    , isOnHardDrive(false)
    , count(0)
{
    carpose = boost::shared_ptr<PoseTransform>(new PoseTransform);
    //carpose->timestamp = std::rand();
}

HDLFrame::~HDLFrame()
{
}

void HDLFrame::printSelf()
{
    int pn = 0;
    for (int i = 0; i < points->size(); ++i) {
        pn += points->at(i)->points.size();
    }
    DLOG(INFO) << "Total points: " << pn << "\n\t"
               << "typical (xyzi): " << points->at(10)->points[0].x << ", "
               << points->at(10)->points[0].y << ", " << points->at(10)->points[0].z << ", "
               << points->at(10)->points[0].intensity;

}

void HDLFrame::clear()
{
    for (int i = 0; i < points->size(); ++i) {
        points->at(i).reset();
    }
    for (int i = 0; i < points->size(); ++i) {
        pointsMeta->at(i).reset();
    }
    std::vector<std::pair<boost::posix_time::ptime, std::string>> vec; // an empty one
    packets->swap(vec);
} // vec goes out of scope, memory should be freed

ofstream &operator<<(ofstream &os, const HDLFrame &obj)
{
    os.write(reinterpret_cast<const char*>(&obj.timestamp), sizeof(obj.timestamp));
    os.write(reinterpret_cast<const char*>(&obj.filenameTime), sizeof(obj.filenameTime));
    os.write(reinterpret_cast<const char*>(&obj.fileStartPos), sizeof(obj.fileStartPos));
    os.write(reinterpret_cast<const char*>(&obj.skips), sizeof(obj.skips));
    os << *(obj.carpose.get());
    return os;
}

ifstream &operator>>(ifstream &is, HDLFrame &obj)
{
    is.read(reinterpret_cast<char*>(&obj.timestamp), sizeof(obj.timestamp));
    is.read(reinterpret_cast<char*>(&obj.filenameTime), sizeof(obj.filenameTime));
    is.read(reinterpret_cast<char*>(&obj.fileStartPos), sizeof(obj.fileStartPos));
    is.read(reinterpret_cast<char*>(&obj.skips), sizeof(obj.skips));
    is >> *(obj.carpose.get());
    return is;
}

//ifstream &operator>>(ifstream &is, TimeLine<HDLFrame> &obj)
//{
//    boost::shared_ptr<HDLFrame> item(new HDLFrame());
//    while (is >> *(item.get())) {
//        obj.addData(item);
//        item = boost::shared_ptr<HDLFrame>(new HDLFrame());
//    }
//    return is;
//}

//ofstream &operator<<(ofstream &os, TimeLine<HDLFrame>& obj)
//{
//    obj.resetCursor();
//    do {
//        os << *(obj.nextData().get());
//    } while (obj.hasMore());
//    return os;
//}

void intrusive_ptr_add_ref(HDLFrame *p)
{
    ++p->count;
}

void intrusive_ptr_release(HDLFrame *p)
{
    if (p->count != 0) --p->count;
}
