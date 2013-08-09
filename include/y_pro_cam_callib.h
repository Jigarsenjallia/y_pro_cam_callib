#ifndef Y_PRO_CAM_CALLIB_H
#define Y_PRO_CAM_CALLIB_H

#include <QMainWindow>
#include <opencv2/opencv.hpp>

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

  void onTimerUpdate();

private:
  Ui::MainWindow *ui;
  QTimer* timer_;

  cv::VideoCapture* camera_;
  QDialog* display;
  Ui::Display* display_ui;
};

#endif