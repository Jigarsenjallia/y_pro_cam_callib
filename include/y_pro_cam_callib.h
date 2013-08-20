#ifndef Y_PRO_CAM_CALLIB_H
#define Y_PRO_CAM_CALLIB_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>
#include <QFuture>
#include <QMutex>

namespace Ui {
  class MainWindow;
  class Display;
}

class ProCamCal : public QMainWindow
{
  Q_OBJECT

public:
  explicit ProCamCal(QMainWindow *parent = 0);
  ~ProCamCal();

protected slots:
  void on_pushButtonConnectCamera_clicked(); 
  void on_pushButtonLoadCameraParams_clicked();
  void on_pushButtonDetectPlane_clicked();
  void on_pushButtonUpdateCameraParameter_clicked();


  void on_spinBoxProjectorID_valueChanged(int value);
  void on_spinBoxPatternOffset_valueChanged(int value);
  void on_spinBoxPatternGap_valueChanged(int value);
  void on_spinBoxPatternSize_valueChanged(int value);
  void on_pushButtonShowPattern_clicked(bool checked);
  void on_pushButtonCaptureFrame_clicked();
  void on_pushButtonCalibrateProjector_clicked();
  void on_pushButtonLoadProjectorParemeter_clicked();
  void on_pushButtonProjectorDetectPlane_clicked();
  void on_pushButtonProjectorDetectPlane_clicked2();
  void on_pushButtonSaveProjectorParameters_clicked();

  void on_pushButtonStart_clicked();
  void on_pushButtonStop_clicked();

 
 

protected:
  //thread function
  void pipeLine();
    

  void detectProjectorCornerPattern();
  
  //pattern
  void updatePattern();
  QImage getPattern(int w, int h, int offset, int gap, int radius);
  QImage getPattern(int w, int h, int offset);

  
  //from select3dobj.cpp
  cv::Point3f image2plane(cv::Point2f imgpt, const cv::Mat& R, const cv::Mat& tvec, const cv::Mat& cameraMatrix, double Z);
  void imagePoints2plane(const std::vector<cv::Point2f>& image_points, std::vector<cv::Point3f>& plane_points, 
                         const cv::Mat& R, const cv::Mat& tvec, const cv::Mat& cameraMatrix, double Z);


private:
  Ui::MainWindow *ui;
  
  cv::VideoCapture* camera_;
  QDialog* display_;
  Ui::Display* display_ui;
  QImage display_pattern_;
  int step_x_;
  int step_y_;
  std::vector<cv::Point2f> pattern_points_;
  std::string calibration_time_;
  int image_width_;
  int image_height_;
  int board_width_;
  int board_height_;
  double square_size_;
  int flags_;
  double avg_reprojection_error_;
  QString camera_parameter_filename_;
  cv::Mat camera_matrix_;
  cv::Mat camera_dist_coeff_;
  cv::Mat camera_rvec_;
  cv::Mat camera_tvec_;
  bool camera_parameter_loaded_;
  bool plan_detected_;
  std::vector<std::vector<cv::Point3f> > projector_object_points_;
  std::vector<std::vector<cv::Point2f> > projector_image_points_;

  QString projector_parameter_filename_;
  cv::Mat projector_matrix_;
  cv::Mat projector_dist_coeff_;
  cv::Mat projector_rvec_;
  cv::Mat projector_tvec_;
  
  std::vector<cv::Point2f> projected_corner_points_;




  QFuture<void> pipe_line_future_;
  QMutex piep_line_mutex_;
  bool pipe_line_stop_;
};

#endif