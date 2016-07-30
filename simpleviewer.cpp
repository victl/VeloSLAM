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
  inputCloud.reset (new PointCloudT);
  groundlessCloud.reset (new PointCloudT);

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
  frames = hdlMgr->getAllFrameMeta();
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
    if (id >= f->points.size() - 1) return;
    ++id;
    updateViewer();
}

void SimpleViewer::updateViewer()
{
    ui->lcdBeamId->display(id);
    *dispCloud = *inputCloud + *f->points[id];
    dispCloud->width = dispCloud->points.size();
    dispCloud->height = 1;
    viewer->updatePointCloud<pcl::PointXYZI>(dispCloud, "cloud");
    ui->qvtkWidget->update ();
    double xx = f->points[id]->front().x - f->points[id]->back().x;
    double yy = f->points[id]->front().y - f->points[id]->back().y;
    double zz = f->points[id]->front().z - f->points[id]->back().z;
    std::cout << "(xx,yy,zz): (" << xx << ',' << yy << ',' << zz << ")\n";
    std::cout << "front/back distance: "
              << f->pointsMeta[id]->front().distance
              << ',' << f->pointsMeta[id]->back().distance << std::endl;
    double displacement = std::sqrt(
                std::pow(xx, 2) + std::pow(yy, 2) + std::pow(zz, 2)
                );
    std::cout << "Displacement in current frame: " << displacement << std::endl;
}

void SimpleViewer::on_btnHold_clicked()
{
    *inputCloud = *groundlessCloud + *f->points[id];
}

void SimpleViewer::on_btnDiscard_clicked()
{
    *inputCloud = *groundlessCloud;
    updateViewer();
}

void SimpleViewer::on_btnMerge_clicked()
{
    groundlessCloud.reset(new PointCloudT);
    seq.push_back(id);
    std::string dispTxt;
    for (auto& val : seq) {
        *groundlessCloud += *f->points[val];
        dispTxt += std::to_string(val) + '\t';
    }
    ui->textBrowser->setText(QString::fromStdString(dispTxt));
}

void SimpleViewer::on_btnGo_clicked()
{
    id = ui->spinId->value();
    updateViewer();
}
