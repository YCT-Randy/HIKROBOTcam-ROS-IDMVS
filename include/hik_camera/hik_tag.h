#ifndef HIKTAG_H
#define HIKTAG_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <iconv.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <camera_info_manager/camera_info_manager.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct DetectState
{
  int tag_id;
  double tagCenter_pixelX, tagCenter_pixelY, tag_yawDegree, tag_yawRadian;
  int pixelH, pixelW;
  Eigen::Vector3d realWorld_pose;
  std::string info_time;

  // 清空函式
  void clear()
  {
    tag_id = 0;
    tagCenter_pixelX = tagCenter_pixelY = tag_yawDegree = tag_yawRadian = 0.0;
    pixelH = pixelW = 0;
    realWorld_pose.setZero();
    info_time.clear();  // std::string 自帶 clear()
  }

  // 判斷是否為空
  bool empty() const
  {
    return info_time.empty();
  }
};
class HikTag
{
public:
  HikTag();
  virtual ~HikTag();

  // Set paramter from camera calibration result yaml
  void yaml_init(const boost::array<double, 9>& K, const int& width, const int& height);

  void set_tag_info(const std::vector<std::string>& info_pkg, const int& width, const int& height);

  int get_tagID();
  int get_tagCenter_pixelX();
  int get_tagCenter_pixelY();
  Eigen::Vector3d get_realWorld_coordinate(const double& distance_z);

  double get_yawDegree();
  double get_yawRadian();
  std::string get_hik_info_time();
  int get_pixelH();
  int get_pixelW();

private:
  // yaml
  Eigen::Matrix3d K_matrix_;
  Eigen::Matrix3d Precomputed_matrix_;

  int pixelH_;
  int pixelW_;

  // info[0]
  int id_;

  // info[1]
  int centerX_;
  int centerY_;
  double yawDegree_;

  // info[3]
  std::string hik_info_time_;

  // other
  bool can_not_to_real_ = false;

  void set_tag_id(const std::string& info);
  void set_XYandDegree(const std::string& info);
  void set_hik_time(const std::string& info);
  void set_pixelHW(const int& width, const int& height);
  void set_K_matrix(const boost::array<double, 9>& K);
  void set_Precomputed_matrix();
  void check_info_error(const int& width, const int& height);

  // void clear_all_info();
  // void clear_info(std::string& time);
};

#endif  // HIKTAG_H
