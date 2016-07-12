#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <QLineEdit>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <fstream>
#include <glog/logging.h>
#include <boost/chrono.hpp>

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer),
  hdlMgr(new HDLManager),
  inRecv(false)
{
  ui->setupUi (this);
  this->setWindowTitle ("SLAM viewer");

  // Setup the cloud pointer
  cloud.reset (new PointCloudT);

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  viewer->resetCamera ();
  ui->qvtkWidget->update ();

  // Set up the viewer
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->addPointCloud<PointT>(cloud);
  viewer->initCameraParameters ();
//  hdlMgr->start();
//  this->thread_ = boost::shared_ptr<boost::thread>(
//    new boost::thread(boost::bind(&PCLViewer::threadLoop, this)));
}

PCLViewer::~PCLViewer ()
{
    delete ui;
    hdlMgr->stop();
    on_btnStop_clicked();
}

QString PCLViewer::open(const QString &path, const QString &title, const QString &filter)
{
    QString fileName;
    if (path.isNull())
        fileName = QFileDialog::getOpenFileName(this, title,
                currentPath, filter);
    else
        fileName = path;

    if (!fileName.isEmpty()) {
        QFile file(fileName);
        if (!file.exists()) {
            QMessageBox::critical(this, title,
                           QString("Could not open file '%1'.").arg(fileName));
            return "";
        }
        return fileName;

        if (!fileName.startsWith(":/")) {
            currentPath = fileName;
//            setWindowTitle(tr("%1 - UgvMap").arg(currentPath));
        }
    }
    return "";
}

QString PCLViewer::save(const QString &path, const QString &title, const QString &filter)
{
    QString fileName;
    if (path.isNull())
        fileName = QFileDialog::getSaveFileName(this, title,
                currentPath, filter);
    else
        fileName = path;

    if (!fileName.isEmpty()) {
        QFile file(fileName);
        if (file.exists()) {
            QMessageBox::StandardButton a = QMessageBox::question(this, title,
                           QString("File '%1' already exists, do you really want to OVERWRITE it?").arg(fileName)
                                  , QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Yes);
            if (a == QMessageBox::No || a == QMessageBox::Escape)
                return "";
        }
        return fileName;

        if (!fileName.startsWith(":/")) {
            currentPath = fileName;
//            setWindowTitle(tr("%1 - UgvMap").arg(currentPath));
        }
    }
    return "";
}

void PCLViewer::toggleHide(bool hide)
{
    if (hide) {
        ui->edtINSFile->hide();
        ui->btnChooseINSFile->hide();
        ui->edtPcapFile->hide();
        ui->btnChoosePcapFile->hide();
    } else {
        ui->edtINSFile->show();
        ui->btnChooseINSFile->show();
        ui->edtPcapFile->show();
        ui->btnChoosePcapFile->show();
    }
}

void PCLViewer::threadLoop()
{
    while (inRecv) {
        auto frame = hdlMgr->getRecentFrame();
        if (frame) {
            cloud.reset(new PointCloudT);
            for (int i = 0; i < frame->points->size(); ++i) {
                *cloud += *(frame->points->at(i).get());
            }
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> intensity_distribution(cloud, 0,255,0);
            viewer->updatePointCloud<pcl::PointXYZI>(cloud, intensity_distribution, "cloud");
            ui->qvtkWidget->update();
        }
        boost::this_thread::sleep_for(boost::chrono::microseconds((int)1e5));
    }
}

void PCLViewer::on_btnChooseCalibFile_clicked()
{
    QString fileName = open(QString(), "Open Calib file", "HDL Calibration files (*.xml)");
    if(fileName.isEmpty())
        return;
    ui->edtCalibFile->setText(fileName);
    hdlMgr->setCalibFile(fileName.toStdString());
}

void PCLViewer::on_rbnOnline_toggled(bool checked)
{
    if (checked) {
        this->toggleHide(true);
    } else {
        this->toggleHide(false);
    }
}

void PCLViewer::on_btnChoosePcapFile_clicked()
{
    QString fileName = open(QString(), "Open HDL .pcap file", "HDL Calibration files (*.pcap)");
    if(fileName.isEmpty())
        return;
    ui->edtPcapFile->setText(fileName);
}

void PCLViewer::on_btnStop_clicked()
{
    this->inRecv = false;
    if (thread_)
        thread_->join();
    thread_.reset();
    hdlMgr->stop();
}

void PCLViewer::on_btnStart_clicked()
{
    hdlMgr->start();
    this->inRecv = true;
    if (!this->thread_) {
        this->thread_ = boost::shared_ptr<boost::thread>(
          new boost::thread(boost::bind(&PCLViewer::threadLoop, this)));
    }
}
