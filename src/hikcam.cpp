#include <hik_camera/hik_camera.h>
#include <hik_camera/hik_camera_ros.h>
#include <ros/ros.h>

HikCam::HikCam()
{
  camera_mode_ = static_cast<int>(HikCamera_Mode::SDK);
  main_mode_ptr_ = static_cast<HikCamBase*>(&sdk_mode_);
}

HikCam::~HikCam()
{
  printf("HikCam() closed \r\n");
}

void HikCam::switch_mode(const int& mode)
{
  if (mode <= static_cast<int>(HikCamera_Mode::OutRange_Min) || mode >= static_cast<int>(HikCamera_Mode::OutRange_Max))
  {
    camera_mode_ = static_cast<int>(HikCamera_Mode::SDK);
  }
  else
  {
    camera_mode_ = mode;
  }
  ROS_WARN_STREAM("Camera Mode is " << camera_mode_);

  if (camera_mode_ == static_cast<int>(HikCamera_Mode::SDK))
  {
    main_mode_ptr_ = static_cast<HikCamBase*>(&sdk_mode_);
  }
  else if (camera_mode_ == static_cast<int>(HikCamera_Mode::UDP))
  {
    main_mode_ptr_ = static_cast<HikCamBase*>(&udp_mode_);
  }
}

void HikCam::start_camera()
{
  main_mode_ptr_->start_camera();
}

void HikCam::stop_camera()
{
  main_mode_ptr_->stop_camera();
}
void HikCam::restart_camera()
{
  main_mode_ptr_->restart_camera();
}

void HikCam::yaml_init(const boost::array<double, 9>& K, const int& width, const int& height)
{
  main_mode_ptr_->yaml_init(K, width, height);
}

bool HikCam::isHandleNull()
{
  return main_mode_ptr_->isHandleNull();
}

bool HikCam::isCameraReady()
{
  return main_mode_ptr_->isCameraReady();
}

bool HikCam::isImageBufferEmpty()
{
  return main_mode_ptr_->isImageBufferEmpty();
}

MV_CODEREADER_IMAGE_OUT_INFO_EX2 HikCam::getImageInfo()
{
  return main_mode_ptr_->getImageInfo();
}

double HikCam::get_yawRadian()
{
  return main_mode_ptr_->get_yawRadian();
}

double HikCam::get_yawDegree()
{
  return main_mode_ptr_->get_yawDegree();
}

int HikCam::get_tagID()
{
  return main_mode_ptr_->get_tagID();
}

Eigen::Vector3d HikCam::get_realWorld_coordinate(const double& distance_z)
{
  return main_mode_ptr_->get_realWorld_coordinate(distance_z);
}

unsigned char* HikCam::getImagePtr()
{
  return main_mode_ptr_->getImagePtr();
}

int HikCam::get_tagCenter_pixelX()
{
  return main_mode_ptr_->get_tagCenter_pixelX();
}

int HikCam::get_tagCenter_pixelY()
{
  return main_mode_ptr_->get_tagCenter_pixelY();
}

bool HikCam::read_info()
{
  return main_mode_ptr_->read_info();
}

std::string HikCam::get_last_info_time()
{
  return main_mode_ptr_->get_last_info_time();
}