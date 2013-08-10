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

  display_ = new QDialog(this);
  display_ui = new Ui::Display;
  display_ui->setupUi(display_);
}

ProCamCal::~ProCamCal()
{  
  if(!pipe_line_stop_)
  {
    pipe_line_stop_ = true;
    pipe_line_future_.waitForFinished();
  }


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
  //camera_->set(CV_CAP_PROP_SETTINGS, 1.0);
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
    pipe_line_stop_ = false;
    pipe_line_future_ = QtConcurrent::run(this, &ProCamCal::pipeLine);
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
  pipe_line_stop_ = true;
  pipe_line_future_.waitForFinished();
    
  if(camera_)
  {
    delete camera_;
    camera_ = 0;
  }

  display_->showNormal();  
  display_->hide();        
    
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

void ProCamCal::pipeLine()
{
  if(camera_ == 0)  
    return;

  while(!pipe_line_stop_)
  {
    cv::Mat frame, frame_gray, frame_gray_not;
    (*camera_) >> frame;
    
    //switch()
    
    
    cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
    cv::bitwise_not(frame_gray, frame_gray_not);
    //cv::imshow("debug", frame_gray_not);
    
   
    bool projector_found = false;
    std::vector<cv::Point2f> projector_points;
    cv::Size projector_pattern_size = cv::Size(step_x_, step_y_);
    cv::SimpleBlobDetector::Params param;
    //param.minConvexity = 0.8;
    //param.thresholdStep = 5;
    //param.minThreshold = 10;
    //param.maxThreshold = 250;
    //param.minArea = 15;
    cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector(param);
    projector_found = cv::findCirclesGrid( frame_gray_not, cv::Size(step_x_, step_y_), projector_points, cv::CALIB_CB_SYMMETRIC_GRID);
    

#if 0
    cv::Ptr<cv::FeatureDetector> blobDetector = new cv::SimpleBlobDetector();
    QFuture<bool> projector_furture = QtConcurrent::run(
      cv::findCirclesGrid,
      frame, 
      projector_pattern_size, 
      projector_points, 
      cv::CALIB_CB_SYMMETRIC_GRID, 
      blobDetector);
    projector_found = projector_furture.result();
#endif
    
       
    bool camera_found = false;   
    std::vector<cv::Point2f> camera_found_points;
    cv::Size camera_pattern_size = cv::Size(ui->spinBoxCameraPatternWidth->value(), ui->spinBoxCameraPatternHeight->value());    
    camera_found = cv::findChessboardCorners( frame, camera_pattern_size, camera_found_points,
      CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
    

    if(projector_found)
    {
      cv::drawChessboardCorners( frame, projector_pattern_size, cv::Mat(projector_points), projector_found);
    }

    if(camera_found)
    {
      cv::drawChessboardCorners( frame, camera_pattern_size, cv::Mat(camera_found_points), camera_found);
    }
        
    
    cv::imshow("camera", frame);
    
    cv::waitKey(1);
  }
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

QImage ProCamCal::getPattern(int w, int h, int offset, int gap, int size)
{
  QPixmap pixmap(w, h);
  pixmap.fill(Qt::black);
  //pixmap.fill(Qt::white);
  QPainter painter(&pixmap);
  painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
  //painter.setBrush(QBrush(Qt::black, Qt::SolidPattern));
  step_x_ = (w - (offset + size)) / gap; 
  step_y_ = (h - (offset + size)) / gap;
  step_x_++;
  step_y_++;

  qDebug() << step_x_ << step_y_;

  for(int i = offset; i < (w - size); i+=gap)
  {
    for(int j = offset; j < (h - size); j+=gap)
    {
      painter.drawEllipse(QPoint(i, j), size, size);
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