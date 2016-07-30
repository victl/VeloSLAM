#include "HDLManager.h"
#include <iostream>
#include <fstream>
#include <pcl/visualization/cloud_viewer.h>

#define tic(label) boost::chrono::thread_clock::time_point (label) =boost::chrono::thread_clock::now()
#define toc(label) \
    std::cout << "processing used: " \
    << boost::chrono::nanoseconds(boost::chrono::thread_clock::now() - (label)).count() / 1000 \
    << " microseconds" << std::endl;

int main() {
    HDLManager hdlMgr(5);
    hdlMgr.setBufferDir("/Volumes/Moto/718/meta", false);
    assert(hdlMgr.loadHDLMeta());
    assert(hdlMgr.loadINSMeta());
    auto frames = hdlMgr.getAllFrameMeta();
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Point Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    viewer->addPointCloud<pcl::PointXYZI> (cloud, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    int cnt = 0;
    for (auto & f : frames) {
        cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        tic(startGetFrame);
        std::cout << f->timestamp << std::endl;
        auto frame = hdlMgr.getFrameAt(f->timestamp);
        if (!frame) {
            ++cnt;
            continue;
        }
        auto& cloudVec = frame->points;
        for (auto & c : cloudVec) {
            cloud->insert(cloud->end(), c->begin(), c->end());
        }
        std::cout << "Num of points in current frame: " << cloud->size() << std::endl;
        viewer->updatePointCloud<pcl::PointXYZI>(cloud, "cloud");
        toc(startGetFrame);
        viewer->spinOnce (100);
    }
    std::cout << cnt << " invalid" << std::endl;
}
