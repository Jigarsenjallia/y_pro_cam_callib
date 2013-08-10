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
  void on_spinBoxProjectorID_valueChanged(int value);
  void on_pushButtonStart_clicked();
  void on_pushButtonStop_clicked();

  void on_spinBoxPatternOffset_valueChanged(int value);
  void on_spinBoxPatternGap_valueChanged(int value);
  void on_spinBoxPatternSize_valueChanged(int value);
  
  //thread function
  void pipeLine();

protected:
  void updatePattern();
  QImage getPattern(int w, int h, int offset, int gap, int radius);

private:
  Ui::MainWindow *ui;
  
  cv::VideoCapture* camera_;
  QDialog* display_;
  Ui::Display* display_ui;
  QImage display_pattern_;
  int step_x_;
  int step_y_;


  QFuture<void> pipe_line_future_;
  QMutex piep_line_mutex_;
  bool pipe_line_stop_;
};

#endif