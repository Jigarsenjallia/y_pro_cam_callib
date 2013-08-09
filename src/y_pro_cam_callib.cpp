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

  display_ = new QDialog(this);
  display_ui = new Ui::Display;
  display_ui->setupUi(display_);
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
}


void ProCamCal::on_pushButtonStart_clicked()
{
  camera_ = new cv::VideoCapture(ui->spinBoxCameraID->value());
  camera_->set(CV_CAP_PROP_FRAME_WIDTH, ui->spinBoxCameraWidth->value());
  camera_->set(CV_CAP_PROP_FRAME_HEIGHT, ui->spinBoxCameraHeight->value());
  if(camera_->isOpened())
  {
    QRect screen = QApplication::desktop()->screenGeometry(ui->spinBoxProjectorID->value());

    updatePattern();    
    display_->show();    
    display_->resize(screen.width(), screen.height());
    display_->move(QPoint(screen.x(), screen.y()));
    display_->showFullScreen();      
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

  display_->showNormal();  
  display_->hide();        
  
  timer_->stop();
  ui->pushButtonStart->setEnabled(true);
  ui->pushButtonStop->setEnabled(false);
}


void ProCamCal::on_spinBoxPatternOffset_valueChanged(int value)
{
  updatePattern();
}

void ProCamCal::on_spinBoxPatternGap_valueChanged(int value)
{
  updatePattern();
}

void ProCamCal::on_spinBoxPatternSize_valueChanged(int value)
{
  updatePattern();
}

void ProCamCal::onTimerUpdate()
{
  cv::Mat frame;
  (*camera_) >> frame;
  cv::imshow("camera", frame);
  cv::waitKey(1);
}

void ProCamCal::updatePattern()
{
  QRect screen = QApplication::desktop()->screenGeometry(ui->spinBoxProjectorID->value());

  display_pattern_ = getPattern(
    screen.width(), screen.height(),    
    ui->spinBoxPatternOffset->value(),
    ui->spinBoxPatternGap->value(),
    ui->spinBoxPatternSize->value());
  display_ui->label->setPixmap(QPixmap::fromImage(display_pattern_));
}

QImage ProCamCal::getPattern(int w, int h, int offset, int gap, int radius)
{
  QPixmap pixmap(w, h);
  pixmap.fill();
  QPainter painter(&pixmap);
  painter.setBrush(QBrush(Qt::black, Qt::SolidPattern));

  for(int i = ui->spinBoxPatternOffset->value(); i < (w - ui->spinBoxPatternSize->value()); i+=ui->spinBoxPatternGap->value())
  {
    for(int j = ui->spinBoxPatternOffset->value(); j < (h - ui->spinBoxPatternSize->value()); j+=ui->spinBoxPatternGap->value())
    {
      painter.drawEllipse(QPoint(i, j), ui->spinBoxPatternSize->value(), ui->spinBoxPatternSize->value());
    }
  }
  return pixmap.toImage();
}


int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  ProCamCal app;
  app.show();
  int ret = a.exec();
  return ret;
}