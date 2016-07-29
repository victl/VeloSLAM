#include "simpleviewer.h"
#include "ui_simpleviewer.h"
#include "HDLFrame.h"
#include "HDLManager.h"
#include <cmath>

SimpleViewer::SimpleViewer (QWidget *parent)
    : QMainWindow (parent)
    , ui (new Ui::SimpleViewer)
    , hdlMgr(0)
    , id(0)
{
  ui->setupUi (this);
  this->setWindowTitle ("Simple viewer");

  // Setup the cloud pointer
  dispCloud.reset (new PointCloudT);
  holdedCloud.reset (new PointCloudT);
  mergedCloud.reset (new PointCloudT);

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  viewer->addPointCloud<pcl::PointXYZI> (dispCloud, "cloud");
  viewer->resetCamera ();
  ui->qvtkWidget->update ();

  hdlMgr = new HDLManager(5);
  hdlMgr->setBufferDir("/Users/victor/Repo/HDL_Data/718/meta", false);
  assert(hdlMgr->loadHDLMeta());
  assert(hdlMgr->loadINSMeta());
  auto frame = hdlMgr->getAllFrameMeta().at(5);
  f = hdlMgr->getFrameAt(frame->timestamp);
  dispCloud = f->getPointsAsOneCloud(0, 63);
  viewer->updatePointCloud<pcl::PointXYZI>(dispCloud, "cloud");
  ui->qvtkWidget->update ();
}

SimpleViewer::~SimpleViewer ()
{
  delete ui;
    delete hdlMgr;
}

void SimpleViewer::on_btnPrev_clicked()
{
    if (id == 0) return;
    --id;
    updateViewer();
}

void SimpleViewer::on_btnNext_clicked()
{
    if (id >= f->points->size() - 1) return;
    ++id;
    updateViewer();
}

void SimpleViewer::updateViewer()
{
    ui->lcdBeamId->display(id);
    *dispCloud = *holdedCloud + *f->points->at(id);
    dispCloud->width = dispCloud->points.size();
    dispCloud->height = 1;
    viewer->updatePointCloud<pcl::PointXYZI>(dispCloud, "cloud");
    ui->qvtkWidget->update ();
    double xx = f->points->at(id)->front().x - f->points->at(id)->back().x;
    double yy = f->points->at(id)->front().y - f->points->at(id)->back().y;
    double zz = f->points->at(id)->front().z - f->points->at(id)->back().z;
    std::cout << "(xx,yy,zz): (" << xx << ',' << yy << ',' << zz << ")\n";
    std::cout << "front/back distance: "
              << f->pointsMeta->at(id)->front().distance
              << ',' << f->pointsMeta->at(id)->back().distance << std::endl;
    double displacement = std::sqrt(
                std::pow(xx, 2) + std::pow(yy, 2) + std::pow(zz, 2)
                );
    std::cout << "Displacement in current frame: " << displacement << std::endl;
}

void SimpleViewer::on_btnHold_clicked()
{
    *holdedCloud = *mergedCloud + *f->points->at(id);
}

void SimpleViewer::on_btnDiscard_clicked()
{
    *holdedCloud = *mergedCloud;
    updateViewer();
}

void SimpleViewer::on_btnMerge_clicked()
{
    mergedCloud.reset(new PointCloudT);
    seq.push_back(id);
    std::string dispTxt;
    for (auto& val : seq) {
        *mergedCloud += *f->points->at(val);
        dispTxt += std::to_string(val) + '\t';
    }
    ui->textBrowser->setText(QString::fromStdString(dispTxt));
}

void SimpleViewer::on_btnGo_clicked()
{
    id = ui->spinId->value();
    updateViewer();
}
