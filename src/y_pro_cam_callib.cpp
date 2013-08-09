#include <y_pro_cam_callib.h>
#include <QtGui>
#include <QDebug>


#include "ui_main_window.h"
#include "ui_display.h"

ProCamCal::ProCamCal(QMainWindow *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  camera_(0)
{
  ui->setupUi(this);

  int screen_count = QApplication::desktop()->screenCount();
  qDebug() << "Total screen:" << screen_count;
  ui->spinBoxProjectorID->setMaximum(screen_count-1);
  on_spinBoxProjectorID_valueChanged(0);

  timer_ = new QTimer(this);
  connect(timer_, SIGNAL(timeout()), this, SLOT(onTimerUpdate()));

  display = new QDialog(this);
  display_ui = new Ui::Display;
  display_ui->setupUi(display);
}

ProCamCal::~ProCamCal()
{  
  if(camera_) delete camera_;
  delete ui;
  QApplication::closeAllWindows();
}

void ProCamCal::on_spinBoxProjectorID_valueChanged(int value)
{
  QRect screen = QApplication::desktop()->screenGeometry(value);
  qDebug() << screen;
  ui->spinBoxProjectorWidth->setValue(screen.width());
  ui->spinBoxProjectorHeight->setValue(screen.height());

  //QDialog* dialog = new QDialog(this);
  //Ui::Display* display = new Ui::Display;
  //display->setupUi(dialog);
  //dialog->show();
  //dialog->move(QPoint(screen.x(), screen.y()));
  //dialog->resize(screen.width(), screen.height());
  //dialog->showFullScreen();
}


void ProCamCal::on_pushButtonStart_clicked()
{
  camera_ = new cv::VideoCapture(ui->spinBoxCameraID->value());
  camera_->set(CV_CAP_PROP_FRAME_WIDTH, ui->spinBoxCameraWidth->value());
  camera_->set(CV_CAP_PROP_FRAME_HEIGHT, ui->spinBoxCameraHeight->value());
  if(camera_->isOpened())
  {
    QRect screen = QApplication::desktop()->screenGeometry(ui->spinBoxProjectorID->value());
    display->show();
    display->move(QPoint(screen.x(), screen.y()));
    display->resize(screen.width(), screen.height());
    display->showFullScreen();
    ui->pushButtonStart->setEnabled(false);
    ui->pushButtonStop->setEnabled(true);
    timer_->start(10);
  }
  else
  {
    delete camera_;
    camera_ = 0;
    QMessageBox::warning(this, "Cannot connect to camera", QString("Cannot connect to camera %1").arg(ui->spinBoxCameraID->value()));
  }
}

void ProCamCal::on_pushButtonStop_clicked()
{
  if(camera_)
  {
    delete camera_;
    camera_ = 0;
  }

  if(display->isVisible())
  {
    display->showNormal();
    display->hide();
  }

  timer_->stop();
  ui->pushButtonStart->setEnabled(true);
  ui->pushButtonStop->setEnabled(false);
}

void ProCamCal::onTimerUpdate()
{
  cv::Mat frame;
  (*camera_) >> frame;
  cv::imshow("camera", frame);
  cv::waitKey(1);
}

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  ProCamCal app;
  app.show();
  int ret = a.exec();
  return ret;
}