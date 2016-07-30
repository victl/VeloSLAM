#ifndef SIMPLEVIEWER_H
#define SIMPLEVIEWER_H

#include <iostream>
#include <deque>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/intrusive_ptr.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
class HDLManager;
class HDLFrame;

namespace Ui
{
  class SimpleViewer;
}

class SimpleViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit SimpleViewer (QWidget *parent = 0);
  ~SimpleViewer ();

public slots:

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr dispCloud;
  PointCloudT::Ptr inputCloud;
  PointCloudT::Ptr groundlessCloud;

private slots:
  void on_btnPrev_clicked();

  void on_btnNext_clicked();

  void updateViewer();

  void on_btnHold_clicked();

  void on_btnDiscard_clicked();

  void on_btnMerge_clicked();

  void on_btnGo_clicked();

private:
    Ui::SimpleViewer *ui;
    HDLManager* hdlMgr;
    std::vector<boost::shared_ptr<HDLFrame> > frames;
    boost::intrusive_ptr<HDLFrame> f;
    std::deque<int> seq;
    int id;
};

#endif // SIMPLEVIEWER_H
