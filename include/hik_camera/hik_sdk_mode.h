#ifndef HIK_SDK_MODE_H
#define HIK_SDK_MODE_H

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
#include <hik_camera/hik_camera_base.h>

#define MAX_BCR_LEN 512

class HikCamSDK : public HikCamBase
{
public:
  HikCamSDK();
  ~HikCamSDK();

  HikImageBuffer img_buffer_;
  HikInfoDecoder info_decoder_;
  HikTag detect_information_;

  void start_camera() override;
  void stop_camera() override;
  void restart_camera() override;
  void yaml_init(const boost::array<double, 9>& K, const int& width, const int& height) override;
  bool isHandleNull() override;
  bool isCameraReady() override;
  bool read_info() override;

  // HikImageBuffer
  bool isImageBufferEmpty() override;
  unsigned char* getImagePtr() override;
  MV_CODEREADER_IMAGE_OUT_INFO_EX2 getImageInfo() override;

  // HikTag
  int get_tagID() override;
  int get_tagCenter_pixelX() override;
  int get_tagCenter_pixelY() override;
  Eigen::Vector3d get_realWorld_coordinate(const double& distance_z) override;
  double get_yawRadian() override;
  double get_yawDegree() override;
  std::string get_last_info_time() override;

private:
  static constexpr size_t IMAGE_WIDTH = 800;
  static constexpr size_t IMAGE_HEIGHT = 600;
  static constexpr size_t IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT;  // 影像大小
  static int GB2312ToUTF8(char* szSrc, size_t iSrcLen, char* szDst, size_t iDstLen);
  static void __stdcall ImageCallBack(unsigned char* pData, MV_CODEREADER_IMAGE_OUT_INFO_EX2* pFrameInfo, void* pUser);

  void* handle_ = NULL;

  std::mutex image_lock_;
  bool camera_isready_;

  void processImage(unsigned char* pData, MV_CODEREADER_IMAGE_OUT_INFO_EX2* pFrameInfo);
  void PressEnterToExit(void);
  bool PrintDeviceInfo(MV_CODEREADER_DEVICE_INFO* pstMVDevInfo);
};

#endif  // HIK_SDK_MODE_H
