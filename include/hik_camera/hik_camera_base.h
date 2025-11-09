#ifndef HIK_CAM_BASE
#define HIK_CAM_BASE

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <iconv.h>
#include <vector>

#include "MvCodeReaderParams.h"
#include "MvCodeReaderErrorDefine.h"
#include "MvCodeReaderCtrl.h"

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define MAX_BCR_LEN 512

class HikCamBase
{
public:
  virtual ~HikCamBase() = default;

  virtual void start_camera() = 0;
  virtual void stop_camera() = 0;
  virtual void restart_camera() = 0;
  virtual void yaml_init(const boost::array<double, 9>& K, const int& width, const int& height) {};
  virtual bool isHandleNull() = 0;
  virtual bool isCameraReady() = 0;
  virtual bool read_info() = 0;

  // image
  virtual bool isImageBufferEmpty() = 0;
  virtual unsigned char* getImagePtr() = 0;
  virtual MV_CODEREADER_IMAGE_OUT_INFO_EX2 getImageInfo() = 0;

  // tag info
  virtual int get_tagID() = 0;
  virtual int get_tagCenter_pixelX() = 0;
  virtual int get_tagCenter_pixelY() = 0;
  virtual Eigen::Vector3d get_realWorld_coordinate(const double& distance_z) = 0;
  virtual double get_yawRadian() = 0;
  virtual double get_yawDegree() = 0;
  virtual std::string get_last_info_time() = 0;
};

#endif  // HIK_CAM_BASE
