#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <camera_info_manager/camera_info_manager.h>
#include <Eigen/Geometry>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

class HikCamROS
{
public:
  HikCamROS();
  virtual ~HikCamROS();

  void open_camera();

private:
  ros::NodeHandle nh_;  // ROS节点句柄
  ros::NodeHandle nh_private_;

  HikCam hikcam_;

  image_transport::Publisher image_pub_;
  ros::Publisher fiducial_tfs_pub_;

  ros::Subscriber switch_mode_sub_;
  ros::Subscriber restart_sub_;

  void switch_mode_Callback(const std_msgs::Int32::ConstPtr& msg);
  void restart_Callback(const std_msgs::Bool::ConstPtr& msg);

  cv::Mat plot_message(const cv::Mat& img_gray, const bool& block_info);
  sensor_msgs::ImagePtr convertToROSImage(const cv::Mat& img);
  void publishTransform(const int& id, const Eigen::Vector3d& xy, const Eigen::Quaterniond& quaternion);
  void fiducial_tfs_publish(const int& id, const Eigen::Vector3d& xy, const Eigen::Quaterniond& quaternion);

  // tf2 set
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Show the info on image
  bool image_show_info_ = true;
  int mode_;
  bool need_restart_ = false;
  bool show_debug_info_;
  std::string image_save_path_;
  std::string yaml_path_;
};