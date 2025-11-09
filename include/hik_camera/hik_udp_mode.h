#ifndef HIK_UDP_H  // 確保這個標頭檔只會被包含一次
#define HIK_UDP_H

// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include <stdlib.h>
// #include <string.h>
// #include <termios.h>
// #include <iconv.h>
// #include <vector>

// #include "MvCodeReaderParams.h"
// #include "MvCodeReaderErrorDefine.h"
// #include "MvCodeReaderCtrl.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <hik_camera/hik_camera_base.h>
#include <thread>

#define MAX_BCR_LEN 512

struct decode_unit
{
  double realworld_x = 1e-6;
  double realworld_y = 1e-6;
  double angle = 1e-3;
};

class HikUDP : public HikCamBase
{
public:
  HikUDP();
  ~HikUDP();
  void run();
  void show_packet_hex(const unsigned char*& p, size_t& n);
  std::vector<double> decode_udp_msg(const unsigned char*& p, size_t& n);
  void decode_packet(const unsigned char* p, size_t n);
  std::vector<std::string> decode_sdk_msg(const unsigned char*& buf, size_t& len);

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
  HikTag sdk_information_;

  bool camera_isready_;

  DetectState detect_buffer_;
  DetectState detect_result_;

  std::thread worker_;
  std::mutex mtx_;
};
#endif  // HIK_UDP_H
