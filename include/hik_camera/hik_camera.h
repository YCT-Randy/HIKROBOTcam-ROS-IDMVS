#ifndef HIK_CAMERA
#define HIK_CAMERA

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <iconv.h>

#include "MvCodeReaderParams.h"
#include "MvCodeReaderErrorDefine.h"
#include "MvCodeReaderCtrl.h"

#include <opencv2/opencv.hpp>

#include <hik_camera/hik_imgbuffer.h>
#include <hik_camera/hik_infodecoder.h>
#include <hik_camera/hik_tag.h>
#include <hik_camera/hik_sdk_mode.h>
#include <hik_camera/hik_udp_mode.h>
#include <hik_camera/hik_camera_enum.h>
#include <hik_camera/hik_camera_base.h>

#define MAX_BCR_LEN 512

class HikCam : public HikCamBase
{
public:
  HikCam();
  ~HikCam();
  void switch_mode(const int& mode);

  // Base override
  void start_camera() override;
  void stop_camera() override;
  void restart_camera() override;
  void yaml_init(const boost::array<double, 9>& K, const int& width, const int& height) override;
  bool isHandleNull() override;
  bool isCameraReady() override;
  bool read_info() override;

  // image
  bool isImageBufferEmpty() override;
  unsigned char* getImagePtr() override;
  MV_CODEREADER_IMAGE_OUT_INFO_EX2 getImageInfo() override;

  // tag info
  int get_tagID() override;
  int get_tagCenter_pixelX() override;
  int get_tagCenter_pixelY() override;
  Eigen::Vector3d get_realWorld_coordinate(const double& distance_z) override;
  double get_yawRadian() override;
  double get_yawDegree() override;
  std::string get_last_info_time() override;

private:
  HikCamSDK sdk_mode_;
  HikUDP udp_mode_;

  HikCamBase* main_mode_ptr_ = nullptr;

  int camera_mode_;
};

#endif  // HIK_CAMERA
