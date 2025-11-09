#include <hik_camera/hik_camera.h>
#include <hik_camera/hik_camera_ros.h>
#include <hik_camera/hik_tag.h>
#include <hik_camera/hik_udp_mode.h>

#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hikirobot_camera");

  HikCamROS camera;
  camera.open_camera();

  // ros::spin();
  // ros::Rate loop_rate(30);  // 10Hz 迴圈

  return 0;
}