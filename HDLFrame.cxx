#include "HDLFrame.h"
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

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
    for (int i = 0; i < points.size(); ++i) {
        pn += points[i]->points.size();
    }
    DLOG(INFO) << "Total points: " << pn << "\n\t"
               << "typical (xyzi): " << points[10]->points[0].x << ", "
               << points[10]->points[0].y << ", "
               << points[10]->points[0].z << ", "
               << points[10]->points[0].intensity;

}

void HDLFrame::dumpToFiles(string dirname)
{
    dirname += "/" + boost::posix_time::to_iso_string(timestamp);
    std::ofstream ofs(dirname + "-points.txt");
    ofs << std::setprecision(9);
    for (auto& cloud : points) {
        for (auto& point : cloud->points) {
            ofs << point.x << '\t' << point.y << '\t' << point.z
                << '\t' << point.intensity << std::endl;
        }
    }
    ofs.close();
    ofs.open(dirname + "-pointsMeta.txt");
    for (auto& cloud : pointsMeta) {
        for (auto& meta : *cloud) {
            ofs << meta.azimuth << '\t' << meta.distance
                << '\t' << (int)meta.intensityFlag
                << '\t' << (int)meta.distanceFlag
                << '\t' << (int)meta.flags << std::endl;
        }
    }
    ofs.close();
    ofs.open(dirname + "-others.txt");
    std::string s, l;
    if (carpose) {
        std::stringstream ss;
        ss << *carpose;
        while(std::getline(ss, l)) {
            s.append(l).append("\n");
        }
    }
    ofs << s << std::endl
        << isInMemory << '\t' << isOnHardDrive << '\t' << (int)count << '\n'
        << filenameTime << '\n'
        << *(reinterpret_cast<long long*>(&fileStartPos)) << '\n'
        << (int)skips << std::endl;
    ofs.close();
}

void HDLFrame::dumpToImage(string dirname, double gridSize, int beamId)
{
    if (!points.size()) return;
    dirname += "/" + boost::posix_time::to_iso_string(timestamp)
            + std::to_string(beamId) + ".png";
    /* only +/- 100m range is considered */
    int range = 200 / gridSize;
    cv::Mat img(range, range, CV_8UC1, cv::Scalar(255));
    int row, col;
    auto coordsToRowCol = [&] (float x, float y) mutable->bool {
        row = (100 + x) / gridSize;
        col = (100 - y) / gridSize;
        if (row >= 0 && row < range && col >=0 && col < range) return true;
        else return false;
    };
    int startBeam, endBeam;
    if (beamId < 0 || beamId > 63) {
        startBeam = 0;
        endBeam = 63;
    } else {
        startBeam = beamId;
        endBeam = beamId + 1;
    }
    for (int i = startBeam; i < endBeam; ++i) {
        auto& currCloud = points[i]->points;
        for (auto& point : currCloud) {
            if (coordsToRowCol(point.x, point.y)) {
                img.at<unsigned char>(col, row) = point.intensity;
            }
        }
    }
    cv::imwrite(dirname, img);
}

void HDLFrame::dumpToPCD(string dirname, int beamId)
{
    if (!points.size()) return;
    int startBeam, endBeam;
    if (beamId < 0 || beamId > 63) {
        startBeam = 0;
        endBeam = 63;
    } else {
        startBeam = beamId;
        endBeam = beamId + 1;
    }
    auto cloud = getPointsAsOneCloud(startBeam, endBeam);
    pcl::io::savePCDFileASCII(dirname + "/"
                              + boost::posix_time::to_iso_string(timestamp)
                              + "-" + std::to_string(beamId) + ".pcd"
                              , *cloud);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr
HDLFrame::getPointsAsOneCloud(int startBeam , int endBeam)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr result;
    if (!points.size()) return result;
    if (startBeam < 0) startBeam = 0;
    if (endBeam <= startBeam + 1)
    {
        return points[startBeam];
    }
    if (endBeam > points.size()) endBeam = points.size();
    result = pcl::PointCloud<pcl::PointXYZI>::Ptr
            (new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = startBeam; i < endBeam; ++i) {
        *result += *(points[i]);
    }
    return result;
}

void HDLFrame::clear()
{
    /* swap with an empty vector to free up memory */
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> pt;
    std::vector<boost::shared_ptr<std::vector<PointMeta> > > ptm;
    std::vector<std::pair<boost::posix_time::ptime, std::string>> vec;
    points.swap(pt);
    pointsMeta.swap(ptm);
    packets.swap(vec);
    /* debug */
    DLOG(INFO) << "frame: " << this->timestamp << " cleared.";
    /* end debug */
} // vec goes out of scope, memory should be freed

ofstream &operator<<(ofstream &os, const HDLFrame &obj)
{
    os.write(reinterpret_cast<const char*>(&obj.timestamp)
             , sizeof(obj.timestamp));
    os.write(reinterpret_cast<const char*>(&obj.filenameTime)
             , sizeof(obj.filenameTime));
    os.write(reinterpret_cast<const char*>(&obj.fileStartPos)
             , sizeof(obj.fileStartPos));
    os.write(reinterpret_cast<const char*>(&obj.skips)
             , sizeof(obj.skips));
    os.write(reinterpret_cast<const char*>(&obj.isOnHardDrive)
             , sizeof(obj.isOnHardDrive));
    os << *(obj.carpose.get());
    return os;
}

std::ifstream &operator>>(std::ifstream &is, HDLFrame &obj)
{
    is.read(reinterpret_cast<char*>(&obj.timestamp)
            , sizeof(obj.timestamp));
    is.read(reinterpret_cast<char*>(&obj.filenameTime)
            , sizeof(obj.filenameTime));
    is.read(reinterpret_cast<char*>(&obj.fileStartPos)
            , sizeof(obj.fileStartPos));
    is.read(reinterpret_cast<char*>(&obj.skips)
            , sizeof(obj.skips));
    is.read(reinterpret_cast<char*>(&obj.isOnHardDrive)
            , sizeof(obj.isOnHardDrive));
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
