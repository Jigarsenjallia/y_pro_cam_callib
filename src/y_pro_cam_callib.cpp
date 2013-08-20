#include <y_pro_cam_callib.h>
#include <QtGui>
#include <QDebug>
#include <time.h>

#include "ui_main_window.h"
#include "ui_display.h"

ProCamCal::ProCamCal(QMainWindow *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow),
  camera_(0),
  camera_parameter_loaded_(false),
  plan_detected_(false)
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

void ProCamCal::on_pushButtonConnectCamera_clicked()
{
  QMutexLocker lock(&piep_line_mutex_);
  if(camera_)
  {
    pipe_line_stop_ = true;
    pipe_line_future_.waitForFinished();    
    delete camera_;
    camera_ = 0;
    ui->pushButtonConnectCamera->setText("Start");
  }
  else
  {
    camera_ = new cv::VideoCapture(ui->spinBoxCameraID->value());
    //camera_->set(CV_CAP_PROP_SETTINGS, 1.0);
    camera_->set(CV_CAP_PROP_FRAME_WIDTH, ui->spinBoxCameraWidth->value());
    camera_->set(CV_CAP_PROP_FRAME_HEIGHT, ui->spinBoxCameraHeight->value());
    if(camera_->isOpened())
    {
      pipe_line_stop_ = false;
      pipe_line_future_ = QtConcurrent::run(this, &ProCamCal::pipeLine);
      ui->pushButtonConnectCamera->setText("Stop");
    }
    else
    {
      delete camera_;
      camera_ = 0;
      QMessageBox::warning(this, "Cannot connect to camera", QString("Cannot connect to camera %1").arg(ui->spinBoxCameraID->value()));
    }
  }
}

void ProCamCal::on_pushButtonLoadCameraParams_clicked()
{
  camera_parameter_filename_ = QFileDialog::getOpenFileName(this);
  if(camera_parameter_filename_.isNull())
    return;
  qDebug() << "Load camera parameters from: " << camera_parameter_filename_;

  cv::FileStorage file(camera_parameter_filename_.toStdString(), cv::FileStorage::READ);

  // first method: use (type) operator on FileNode.
  file["calibration_time"] >> calibration_time_;
  file["image_width"] >> image_width_;
  file["image_height"] >> image_height_;
  file["board_width"] >> board_width_;
  file["board_height"] >> board_height_;
  file["square_size"] >> square_size_;
  file["flags"] >> flags_;
  file["camera_matrix"] >> camera_matrix_;
  file["distortion_coefficients"] >> camera_dist_coeff_;
  file["avg_reprojection_error"] >> avg_reprojection_error_;

  camera_rvec_ = cv::Mat::zeros(3, 1, CV_64F);
  camera_tvec_ = cv::Mat::zeros(3, 1, CV_64F);

  if(!file["rvec"].isNone()) file["rvec"] >> camera_rvec_; 
  if(!file["tvec"].isNone()) file["tvec"] >> camera_tvec_;

  std::cout 
    << "calibration_time: " << calibration_time_ << std::endl
    << "image_width: " << image_width_ << std::endl
    << "image_height: " << image_height_ << std::endl
    << "board_width: " << board_width_ << std::endl
    << "board_height: " << board_height_ << std::endl
    << "square_size: " << square_size_ << std::endl
    << "flags:" << flags_ << std::endl
    << "camera_matrix: " << camera_matrix_ << std::endl
    << "distortion_coefficients: " << camera_dist_coeff_ << std::endl
    << "avg_reprojection_error: " << avg_reprojection_error_ << std::endl
    << "rvec" << camera_rvec_ << std::endl
    << "tvec" << camera_tvec_ << std::endl;
  file.release();

  camera_parameter_loaded_ = true;
}

void ProCamCal::on_pushButtonUpdateCameraParameter_clicked()
{
  QString filename = QFileDialog::getSaveFileName(this);
  if(filename.isNull())
    return;
  camera_parameter_filename_ = filename;
  cv::FileStorage fs(camera_parameter_filename_.toStdString(), cv::FileStorage::WRITE);

  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );
  
  fs << "calibration_time" << buf;
  fs << "image_width" << image_width_;
  fs << "image_height" << image_height_;
  fs << "board_width" << board_width_;
  fs << "board_height" << board_height_;
  fs << "square_size" << square_size_;
  fs << "flags" << flags_;
  fs << "camera_matrix" << camera_matrix_;
  fs << "distortion_coefficients" << camera_dist_coeff_;
  fs << "avg_reprojection_error" << avg_reprojection_error_;
  fs <<  "rvec" << camera_rvec_; 
  fs <<  "tvec" << camera_tvec_; 

  fs.release();  
}

void ProCamCal::on_pushButtonDetectPlane_clicked()
{
  if(!camera_)
    return;

  QMutexLocker lock(&piep_line_mutex_);

  cv::Mat frame, frame_gray;
  (*camera_) >> frame;
  cvtColor(frame, frame_gray, CV_BGR2GRAY);

  bool camera_found = false;   
  std::vector<cv::Point2f> camera_found_points;
  cv::Size camera_pattern_size = cv::Size(ui->spinBoxCameraPatternWidth->value(), ui->spinBoxCameraPatternHeight->value());    
  camera_found = cv::findChessboardCorners( frame, camera_pattern_size, camera_found_points,
    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

  if(!camera_found) 
  {
    ui->pushButtonDetectPlane->setText("Detect plane - Failed!");
    return; 
  }

  // improve the found corners' coordinate accuracy
  cv::cornerSubPix( frame_gray, camera_found_points, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
  //cv::drawChessboardCorners( frame, camera_pattern_size, cv::Mat(camera_found_points), camera_found);
  std::vector<cv::Point3f> object_points;
  for(int j = 0; j < ui->spinBoxCameraPatternHeight->value(); j++)
  {
    for(int i = 0; i < ui->spinBoxCameraPatternWidth->value(); i++)
    {
      object_points.push_back(cv::Point3f(i*ui->spinBoxCameraPatternSize->value(), j*ui->spinBoxCameraPatternSize->value(), 0));
      //std::cout << object_points.back() << std::endl;
    }
  }

  cv::solvePnP(object_points, camera_found_points, camera_matrix_, camera_dist_coeff_, camera_rvec_, camera_tvec_);
  
  std::cout << "camera rvec:" << camera_rvec_ << std::endl;
  std::cout << "camera tvec:" << camera_tvec_ << std::endl;

  plan_detected_ = true;



  std::vector<cv::Point3f> target_points;
  std::vector<cv::Point2f> image_points;
  target_points.push_back(cv::Point3f(0, 0, 0));
  target_points.push_back(cv::Point3f(0.3, 0, 0));
  target_points.push_back(cv::Point3f(0, 0.3, 0));
  target_points.push_back(cv::Point3f(0, 0, 0.3));

  target_points.push_back(cv::Point3f(0.5, 0.2, 0));
  
  
  cv::projectPoints(
    target_points, 
    camera_rvec_, 
    camera_tvec_, 
    camera_matrix_,
    camera_dist_coeff_,
    image_points);

  for(int i = 4; i < target_points.size(); i++)
  {
    std::cout << target_points[i] << image_points[i] << std::endl;
    cv::circle(frame, image_points[i], 5, CV_RGB(255, 0, 0), 2);
  }

  cv::line(frame, image_points[0], image_points[1], CV_RGB(255, 0, 0), 2);
  cv::line(frame, image_points[0], image_points[2], CV_RGB(0, 255, 0), 2);
  cv::line(frame, image_points[0], image_points[3], CV_RGB(0, 0, 255), 2);

  ui->pushButtonDetectPlane->setText("Detect plane - Done!");
  cv::imshow("detect_plane", frame);
 
}

void ProCamCal::on_spinBoxProjectorID_valueChanged(int value)
{
  QRect screen = QApplication::desktop()->screenGeometry(value);
  qDebug() << screen;
  ui->spinBoxProjectorWidth->setValue(screen.width());
  ui->spinBoxProjectorHeight->setValue(screen.height());
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

void ProCamCal::on_pushButtonShowPattern_clicked(bool checked)
{
  if(checked)
  {
    QRect screen = QApplication::desktop()->screenGeometry(ui->spinBoxProjectorID->value());
    updatePattern();    
    display_->show();    
    display_->resize(screen.width(), screen.height());
    display_->move(QPoint(screen.x(), screen.y()));
    display_->showFullScreen();      
  }
  else
  {    
    display_->showNormal();  
    display_->hide();        
  }
}

void ProCamCal::on_pushButtonCaptureFrame_clicked()
{
  QMutexLocker lock(&piep_line_mutex_);

  cv::Mat frame, frame_undistorted, frame_gray, frame_gray_neg;
  (*camera_) >> frame;
  cv::undistort(frame, frame_undistorted, camera_matrix_, camera_dist_coeff_);
  cvtColor(frame_undistorted, frame_gray, CV_BGR2GRAY);
  cv::bitwise_not(frame_gray, frame_gray_neg);

  bool projector_found = false;
  std::vector<cv::Point2f> projector_points;
  cv::Size projector_pattern_size = cv::Size(step_x_, step_y_);
  projector_found = cv::findCirclesGrid( frame_gray_neg, projector_pattern_size, projector_points, cv::CALIB_CB_SYMMETRIC_GRID);
    



  if(!projector_found)
  {
    return;
  }

  cv::drawChessboardCorners( frame_undistorted, projector_pattern_size, cv::Mat(projector_points), projector_found);
  cv::circle(frame_undistorted, projector_points[0], 8, CV_RGB(0, 255, 0), 2);
  //compute position on plane
  cv::Mat R;
  cv::Rodrigues(camera_rvec_, R);
  std::vector<cv::Point3f> plane_points;
  imagePoints2plane(projector_points, plane_points, R, camera_tvec_, camera_matrix_, 0.0);
#if 1
  cv::Point3f start_point = plane_points[0];
  for(int i = 0; i < plane_points.size(); i++)
  {
    std::cout << projector_points[i] << plane_points[i];
    plane_points[i].x -=  start_point.x;
    plane_points[i].y -=  start_point.y;
    plane_points[i].z -=  start_point.z;
    std::cout << plane_points[i] << std::endl;
  } 
#endif
  projector_object_points_.push_back(plane_points);
  projector_image_points_.push_back(pattern_points_);

  ui->pushButtonCaptureFrame->setText(QString("Capture (%1)").arg(projector_object_points_.size()));

  cv::imshow("pattern", frame_undistorted);
 
}

double computeReprojectionErrors(
    const std::vector<std::vector<cv::Point3f> >& objectPoints,
    const std::vector<std::vector<cv::Point2f> >& imagePoints,
    const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
    const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    std::vector<float>& perViewErrors )
{
  std::vector<cv::Point2f> imagePoints2;
  int i, totalPoints = 0;
  double totalErr = 0, err;
  perViewErrors.resize(objectPoints.size());

  for( i = 0; i < (int)objectPoints.size(); i++ )
  {
    projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i],
                  cameraMatrix, distCoeffs, imagePoints2);
    err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);
    int n = (int)objectPoints[i].size();
    perViewErrors[i] = (float)std::sqrt(err*err/n);
    totalErr += err*err;
    totalPoints += n;
  }

  return std::sqrt(totalErr/totalPoints);
}

void ProCamCal::on_pushButtonCalibrateProjector_clicked()
{

  cv::Mat projector_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  projector_camera_matrix.at<double>(0,0) = 400;
  projector_camera_matrix.at<double>(0,2) = 400;
  projector_camera_matrix.at<double>(1,1) = 400;
  projector_camera_matrix.at<double>(1,2) = 300;
  std::cout << projector_camera_matrix << std::endl;

  cv::Mat projector_camera_dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

  std::vector<cv::Mat> rvecs;
  std::vector<cv::Mat> tvecs;
  double rms = cv::calibrateCamera(
    projector_object_points_, 
    projector_image_points_, 
    cv::Size(ui->spinBoxProjectorWidth->value(), ui->spinBoxProjectorHeight->value()),
    projector_camera_matrix,
    projector_camera_dist_coeffs,
    rvecs, 
    tvecs, 
    CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

  printf("RMS error reported by calibrateCamera: %g\n", rms);
  std::cout << projector_camera_matrix << std::endl;
  std::cout << projector_camera_dist_coeffs << std::endl;
}

void ProCamCal::on_pushButtonLoadProjectorParemeter_clicked()
{
  QString filename = QFileDialog::getOpenFileName(this);
  if(filename.isNull())
    return;
  qDebug() << filename;

  cv::FileStorage file(filename.toStdString(), cv::FileStorage::READ);

  file["camera_matrix"] >> projector_matrix_;
  file["distortion_coefficients"] >> projector_dist_coeff_;

  std::cout << projector_matrix_ << std::endl;    
  std::cout << projector_dist_coeff_ << std::endl;
#if 0
  projector_rvec_ = cv::Mat::zeros(3, 1, CV_32F);
  projector_tvec_ = cv::Mat::zeros(3, 1, CV_32F);
  std::cout << projector_rvec_ << projector_tvec_ << std::endl;
  std::vector<cv::Point3f> object_points;
  std::vector<cv::Point2f> image_points;
  object_points.push_back(cv::Point3f(0, 0, 0));
  object_points.push_back(cv::Point3f(0, -0.1, 0));
  object_points.push_back(cv::Point3f(0.1, -0.1, 0));
  object_points.push_back(cv::Point3f(-0.1, -0.1, 0));
  object_points.push_back(cv::Point3f(0, -0.2, 0));
  object_points.push_back(cv::Point3f(0, -0.3, 0));
  object_points.push_back(cv::Point3f(0, -0.4, 0));
 
  
  cv::projectPoints(
    object_points, 
    projector_rvec_, 
    projector_tvec_, 
    projector_matrix_,
    projector_dist_coeff_,
    image_points);

  cv::Mat img = cv::Mat::zeros(600, 800, CV_8UC3);

  for(int i = 0; i < object_points.size(); i++)
  {
    std::cout << object_points[i] << image_points[i] << std::endl;
    cv::circle(img, image_points[i], 5, CV_RGB(255, 0, 0), 2);
  }
  cv::imshow("test", img);
#endif
  
}

void ProCamCal::on_pushButtonProjectorDetectPlane_clicked()
{
  QMutexLocker lock(&piep_line_mutex_);

  if(projected_corner_points_.size() != 4)
    return;

  cv::Mat frame, frame_undistorted, frame_gray, frame_gray_neg;
  (*camera_) >> frame;
  cv::undistort(frame, frame_undistorted, camera_matrix_, camera_dist_coeff_);
  cvtColor(frame_undistorted, frame_gray, CV_BGR2GRAY);
  
  // improve the found corners' coordinate accuracy
  cv::cornerSubPix( frame_gray, projected_corner_points_, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
  
  for(size_t i = 0; i < projected_corner_points_.size(); i++)
    std::cout << projected_corner_points_[i] << std::endl;

  //compute position on plane
  cv::Mat R;
  cv::Rodrigues(camera_rvec_, R);
  std::vector<cv::Point3f> plane_points;
  imagePoints2plane(projected_corner_points_, plane_points, R, camera_tvec_, camera_matrix_, 0.0);

  for(int i = 0; i < plane_points.size(); i++)
  {
    std::cout << plane_points[i];
    plane_points[i].x -=  plane_points[0].x;
    plane_points[i].y -=  plane_points[0].y;
    plane_points[i].z -=  plane_points[0].z;
    std::cout << plane_points[i] << std::endl;
  }

  cv::solvePnP(
    plane_points, 
    pattern_points_, 
    projector_matrix_, 
    projector_dist_coeff_, 
    projector_rvec_, 
    projector_tvec_);

  std::cout << "projector_rvec:" << projector_rvec_ << std::endl;
  std::cout << "projector_tvec:" << projector_tvec_ << std::endl;



  std::vector<cv::Point3f> target_points;
  std::vector<cv::Point2f> image_points;  
  target_points.push_back(cv::Point3f(0.5, 0.2, 0));
  target_points.push_back(cv::Point3f(0.4, 0.2, 0));
  target_points.push_back(cv::Point3f(0.5, 0.1, 0));

  //target_points.push_back(cv::Point3f(0.5, 0.2, 0.2));
  
  
  
  cv::projectPoints(
    target_points, 
    projector_rvec_, 
    projector_tvec_,
    projector_matrix_,
    projector_dist_coeff_,
    image_points);

  
  QPixmap pixmap(ui->spinBoxProjectorWidth->value(), ui->spinBoxProjectorHeight->value());
  pixmap.fill(Qt::black);
  QPainter painter(&pixmap);
  painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));


  for(int i = 0; i < image_points.size(); i++)
  {
    std::cout << target_points[i] << image_points[i] << std::endl;
    painter.drawEllipse(QPoint(image_points[i].x, image_points[i].y), 20, 20);
  }

  display_ui->label->setPixmap(pixmap);

}

void ProCamCal::on_pushButtonProjectorDetectPlane_clicked2()
{
  QMutexLocker lock(&piep_line_mutex_);

  cv::Mat frame, frame_undistorted, frame_gray, frame_gray_neg;
  (*camera_) >> frame;
  cv::undistort(frame, frame_undistorted, camera_matrix_, camera_dist_coeff_);
  cvtColor(frame_undistorted, frame_gray, CV_BGR2GRAY);
  cv::bitwise_not(frame_gray, frame_gray_neg);

  bool projector_found = false;
  std::vector<cv::Point2f> projector_points;
  cv::Size projector_pattern_size = cv::Size(step_x_, step_y_);
  projector_found = cv::findCirclesGrid( frame_gray_neg, projector_pattern_size, projector_points, cv::CALIB_CB_SYMMETRIC_GRID);
 
  if(projector_found)
  { 
    cv::drawChessboardCorners( frame_undistorted, projector_pattern_size, cv::Mat(projector_points), projector_found);
  
    //compute position on plane
    cv::Mat R;
    cv::Rodrigues(camera_rvec_, R);
    std::vector<cv::Point3f> plane_points;
    imagePoints2plane(projector_points, plane_points, R, camera_tvec_, camera_matrix_, 0.0);

    cv::solvePnP(
      plane_points, 
      pattern_points_, 
      projector_matrix_, 
      projector_dist_coeff_, 
      projector_rvec_, 
      projector_tvec_);

    std::cout << "projector_rvec:" << projector_rvec_ << std::endl;
    std::cout << "projector_tvec:" << projector_tvec_ << std::endl;
    cv::imshow("pattern", frame_undistorted);
  }
}

void ProCamCal::on_pushButtonSaveProjectorParameters_clicked()
{
  QString filename = QFileDialog::getSaveFileName(this);
  if(filename.isNull())
    return;
  projector_parameter_filename_ = filename;
  cv::FileStorage fs(projector_parameter_filename_.toStdString(), cv::FileStorage::WRITE);

  time_t tt;
  time( &tt );
  struct tm *t2 = localtime( &tt );
  char buf[1024];
  strftime( buf, sizeof(buf)-1, "%c", t2 );
  
  fs << "calibration_time" << buf;
  fs << "projector_width" << ui->spinBoxProjectorWidth->value();
  fs << "projector_height" << ui->spinBoxProjectorHeight->value();
  fs << "pattern_offset" << ui->spinBoxPatternOffset->value();
  fs << "pattern_gap" << ui->spinBoxPatternGap->value();
  fs << "pattern_size" << ui->spinBoxPatternSize->value();
  fs << "projector_matrix" << projector_matrix_;
  fs << "distortion_coefficients" << projector_dist_coeff_;
  fs <<  "rvec" << projector_rvec_; 
  fs <<  "tvec" << projector_tvec_; 

  fs.release();  
}

void ProCamCal::on_pushButtonStart_clicked()
{
  ui->pushButtonStart->setEnabled(false);
  ui->pushButtonStop->setEnabled(true);
}

void ProCamCal::on_pushButtonStop_clicked()
{  
  ui->pushButtonStart->setEnabled(true);
  ui->pushButtonStop->setEnabled(false);
}

void cvMouseCallBack(int event, int x, int y, int flags, void* userdata)
{
  if((event == cv::EVENT_LBUTTONDOWN) && (flags & cv::EVENT_FLAG_CTRLKEY))
  {
    qDebug() << event << flags << x << y;

    std::vector<cv::Point2f>* ptr = (std::vector<cv::Point2f>*)userdata;

    if(ptr->size() == 4)
    {
      ptr->clear();
    }
    ptr->push_back(cv::Point2f(x, y));
  }
}

void ProCamCal::pipeLine()
{
  cv::Mat frame, frame_undistroted;
  piep_line_mutex_.lock();
  (*camera_) >> frame;
  piep_line_mutex_.unlock();

  cv::imshow("camera", frame);
  cv::setMouseCallback("camera", cvMouseCallBack, &(this->projected_corner_points_));
  

  while(!pipe_line_stop_)
  {
    cv::waitKey(1);

    if(camera_ == 0)  
      continue;

    piep_line_mutex_.lock();
    (*camera_) >> frame;
    piep_line_mutex_.unlock();

    if(camera_parameter_loaded_)
    {
      cv::undistort(frame, frame_undistroted, camera_matrix_, camera_dist_coeff_);
      frame = frame_undistroted;
    }


    for(size_t i = 0; i < projected_corner_points_.size(); i++)
      cv::circle(frame, projected_corner_points_[i], 5, projected_corner_points_.size() == 4 ? CV_RGB(0, 255, 0) : CV_RGB(255, 0, 0));
 
    cv::imshow("camera", frame);
  }
}

void ProCamCal::updatePattern()
{
  QRect screen = QApplication::desktop()->screenGeometry(ui->spinBoxProjectorID->value());

  if(ui->comboBoxPatternType->currentText() == "Circles") 
  {
  
    display_pattern_ = getPattern(
      screen.width(), screen.height(),
      ui->spinBoxPatternOffset->value(),
      ui->spinBoxPatternGap->value(),
      ui->spinBoxPatternSize->value());
  }
  else
  {
    display_pattern_ = getPattern(
      screen.width(), screen.height(),
      ui->spinBoxPatternOffset->value());
  }
  
  display_ui->label->setPixmap(QPixmap::fromImage(display_pattern_));
}

QImage ProCamCal::getPattern(int w, int h, int offset, int gap, int size)
{
  QPixmap pixmap(w, h);
  pixmap.fill(Qt::black);
  QPainter painter(&pixmap);
  painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));
  step_x_ = (w - (offset + size)) / gap; 
  step_y_ = (h - (offset + size)) / gap;
  step_x_++;
  step_y_++;

  pattern_points_.clear(); 
  for(int j = offset; j < (h - size); j+=gap)  
  {
    for(int i = offset; i < (w - size); i+=gap)
    {
      painter.drawEllipse(QPoint(i, j), size, size);
      pattern_points_.push_back(cv::Point2f(i, j));     
    }
  }
  return pixmap.toImage();
}


QImage ProCamCal::getPattern(int w, int h, int offset)
{
  QPixmap pixmap(w, h);
  pixmap.fill(Qt::black);
  QPainter painter(&pixmap);
  painter.setBrush(QBrush(Qt::white, Qt::SolidPattern));

  pattern_points_.clear(); 
  pattern_points_.push_back(cv::Point2f(offset, offset));
  pattern_points_.push_back(cv::Point2f(w - offset - 1, offset));
  pattern_points_.push_back(cv::Point2f(offset,  h - (offset) - 1));
  pattern_points_.push_back(cv::Point2f(w - offset - 1,  h - (offset) - 1));
  
  //upper left
  painter.drawRect(offset, 0, offset, offset);
  painter.drawRect(0, offset, offset, offset);

  //upper right
  painter.drawRect(w - offset - 1, 0, offset, offset);
  painter.drawRect(w - (offset*2) - 1, offset, offset, offset);

  //lower left
  painter.drawRect(offset, h - (offset*2) - 1, offset, offset);
  painter.drawRect(0, h - (offset) - 1, offset, offset);

  //lower right
  painter.drawRect(w - offset - 1, h - (offset*2) - 1, offset, offset);
  painter.drawRect(w - (offset*2) - 1,  h - (offset) - 1, offset, offset);

  return pixmap.toImage();
}

cv::Point3f ProCamCal::image2plane(cv::Point2f imgpt, const cv::Mat& R, const cv::Mat& tvec, const cv::Mat& cameraMatrix, double Z)
{
  cv::Mat R1 = R.clone();
  R1.col(2) = R1.col(2)*Z + tvec;
  cv::Mat_<double> v = (cameraMatrix*R1).inv()*(cv::Mat_<double>(3,1) << imgpt.x, imgpt.y, 1);
  double iw = fabs(v(2,0)) > DBL_EPSILON ? 1./v(2,0) : 0;
  return cv::Point3f((float)(v(0,0)*iw), (float)(v(1,0)*iw), (float)Z);
}

void ProCamCal::imagePoints2plane(const std::vector<cv::Point2f>& image_points, std::vector<cv::Point3f>& plane_points, 
                                  const cv::Mat& R, const cv::Mat& tvec, const cv::Mat& cameraMatrix, double Z)
{
  cv::Mat R1 = R.clone();
  R1.col(2) = R1.col(2)*Z + tvec;
  plane_points.clear();
  cv::Mat mat = (cameraMatrix*R1).inv();
  for(int i = 0; i < image_points.size(); i++)
  {
    cv::Mat_<double> v = mat*(cv::Mat_<double>(3,1) << image_points[i].x, image_points[i].y, 1);
    double iw = fabs(v(2,0)) > DBL_EPSILON ? 1./v(2,0) : 0;
    plane_points.push_back(cv::Point3f((float)(v(0,0)*iw), (float)(v(1,0)*iw), (float)Z));
  }
}

int main(int argc, char *argv[])
{
  QApplication a(argc, argv);
  ProCamCal app;
  app.show();
  int ret = a.exec();
  return ret;
}