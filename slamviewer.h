#ifndef SLAMVIEWER_H
#define SLAMVIEWER_H

#include <iostream>

// Qt
#include <QtWidgets>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "HDLManager.h"
#include <boost/shared_ptr.hpp>
#include <boost/thread/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/atomic.hpp>
#include <iostream>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class SLAMViewer;
}

class SLAMViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit SLAMViewer (QWidget *parent = 0);
  ~SLAMViewer ();

protected:
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  PointCloudT::Ptr cloud;

private slots:
  void on_btnChooseCalibFile_clicked();

  void on_rbnOnline_toggled(bool checked);

  void on_btnChoosePcapFile_clicked();

  void on_btnStop_clicked();

  void on_btnStart_clicked();

private:
  Ui::SLAMViewer *ui;
  boost::shared_ptr<HDLManager> hdlMgr;
  boost::shared_ptr<boost::thread> thread_;
  boost::atomic<bool> inRecv;
  QString currentPath;
  QString open(const QString &path, const QString &title, const QString &filter);
  QString save(const QString &path, const QString &title, const QString &filter);
  void toggleHide(bool hide);
  void threadLoop();

};

#endif // SLAMVIEWER_H
