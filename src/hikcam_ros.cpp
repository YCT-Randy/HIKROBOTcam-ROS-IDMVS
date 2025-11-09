#include <hik_camera/hik_camera.h>
#include <hik_camera/hik_camera_ros.h>
#include <hik_camera/hik_tag.h>
#include <vector>

HikCamROS::HikCamROS() : nh_(), nh_private_("~"), tf_buffer_(), tf_listener_(tf_buffer_)
{
  image_transport::ImageTransport it(nh_);
  image_pub_ = it.advertise("/hikcam_SC2005AM/image", 1);
  fiducial_tfs_pub_ = nh_.advertise<fiducial_msgs::FiducialTransformArray>("/hikcam_SC2005AM_fiducial_tfs", 1);

  switch_mode_sub_ = nh_.subscribe("/switch_hikcam_mode", 1, &HikCamROS::switch_mode_Callback, this);
  restart_sub_ = nh_.subscribe("/restart_camera", 1, &HikCamROS::restart_Callback, this);

  nh_private_.param<bool>("show_debug_info", show_debug_info_, false);
  nh_private_.param<bool>("image_show_info", image_show_info_, false);
  nh_private_.param<int>("mode", mode_, static_cast<int>(HikCamera_Mode::UDP));

  std::string file_path;
  nh_private_.param<std::string>("file_path", file_path, "/systems");
  image_save_path_ = file_path + "/camera_down.png";
  yaml_path_ = "file://" + file_path +
               "/config/camera_down.yaml";
}

HikCamROS::~HikCamROS()
{
  ROS_INFO("Destroying HikCamROS");
}

// 回呼函式
void HikCamROS::switch_mode_Callback(const std_msgs::Int32::ConstPtr &msg)
{
  mode_ = msg->data;

  if (mode_ == static_cast<int>(HikCamera_Mode::UDP))
  {
    ROS_ERROR_STREAM("Switch to UDP");
  }
  else
  {
    ROS_ERROR_STREAM("Switch to SDK");
  }
}

void HikCamROS::restart_Callback(const std_msgs::Bool::ConstPtr &msg)
{
  if (msg->data)
  {
    ROS_INFO("Received restart signal: true");
    need_restart_ = true;
  }
}

void HikCamROS::open_camera()
{
  ros::Rate loop_rate(30);

  camera_info_manager::CameraInfoManager cinfo(nh_, "camera", yaml_path_);
  sensor_msgs::CameraInfo cam_info = cinfo.getCameraInfo();
  const boost::array<double, 9> &K_data = cam_info.K;

  std::string last_info_time = "";

  // It means the tag info have error(most of time is no information)
  bool info_empty;

  // The Current_Mode is copy from member mode_, it useful in AsyncSpinner
  // If Last_Mode differs from Current_Mode, switch the camera state.
  int Current_Mode = mode_;
  int Last_Mode = mode_;

  ROS_WARN_STREAM("Start while!");

  while (ros::ok())
  {
    Current_Mode = mode_;

    // Last_Mode != Current_Mode: The mode switch need change
    // need_restart_: Get topic to restart camera
    // hikcam_.isHandleNull(): Camera not start yet
    if ((Last_Mode != Current_Mode || need_restart_) || hikcam_.isHandleNull())
    {
      hikcam_.stop_camera();
      hikcam_.switch_mode(Current_Mode);
      hikcam_.start_camera();
      hikcam_.yaml_init(cam_info.K, static_cast<int>(cam_info.width), static_cast<int>(cam_info.height));
      while (!hikcam_.isCameraReady())
      {
        ROS_WARN_STREAM("Wait for Camera Ready!");
        loop_rate.sleep();
      }
      need_restart_ = false;
      Last_Mode = Current_Mode;
    }

    if (Current_Mode == static_cast<int>(HikCamera_Mode::UDP))
    {
      if (show_debug_info_)
        ROS_WARN_STREAM("Now mode is UDP");
    }
    else
    {
      if (show_debug_info_)
        ROS_WARN_STREAM("Now mode is SDK");
    }

    //  (!hikcam_.isHandleNull() && !hikcam_.isImageBufferEmpty())-->SDK normal is have image
    //  UDP Mode don't have image but have info
    if ((!hikcam_.isHandleNull() && !hikcam_.isImageBufferEmpty()) ||
        Current_Mode == static_cast<int>(HikCamera_Mode::UDP))
    {
      if (hikcam_.read_info()) // have info
      {
        if (last_info_time != hikcam_.get_last_info_time())
        {
          // Hik camera sends duplicate data
          // Use last_time to prevent publishing duplicate data.

          Eigen::AngleAxisd yaw_angle(hikcam_.get_yawRadian(), Eigen::Vector3d::UnitZ());
          Eigen::Quaterniond quaternion(yaw_angle);
          publishTransform(hikcam_.get_tagID(), hikcam_.get_realWorld_coordinate(0.15), quaternion);
          fiducial_tfs_publish(hikcam_.get_tagID(), hikcam_.get_realWorld_coordinate(0.15), quaternion);

          info_empty = false;
          last_info_time = hikcam_.get_last_info_time();
        }
        else
        {
          info_empty = false;
        }
      }
      else
      {
        Eigen::Quaterniond quaternion;
        Eigen::Vector3d temp(-100, -100, -100);
        publishTransform(-1, temp, quaternion);
        fiducial_tfs_publish(-1, temp, quaternion);
        info_empty = true;
      }

      if (Current_Mode == static_cast<int>(HikCamera_Mode::SDK))
      {
        // Image process
        sensor_msgs::ImagePtr img_ptr;
        cv::Mat img_gray(hikcam_.getImageInfo().nHeight, hikcam_.getImageInfo().nWidth, CV_8UC1, hikcam_.getImagePtr());
        cv::imwrite(image_save_path_, img_gray);
        if (image_show_info_)
          img_ptr = convertToROSImage(plot_message(img_gray, info_empty));
        else
          img_ptr = convertToROSImage(img_gray);

        img_ptr->header.stamp = ros::Time::now();
        image_pub_.publish(img_ptr);
      }
    }
    else
    {
      if (show_debug_info_)
        ROS_ERROR_STREAM("No Image");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void HikCamROS::publishTransform(const int &id, const Eigen::Vector3d &geometry, const Eigen::Quaterniond &quaternion)
{
  if (show_debug_info_)
    ROS_INFO("Publishing TF: x=%.4f y=%.4f yaw=%.4f", geometry.x(), geometry.y(), hikcam_.get_yawDegree());
  if (geometry.x() == -100 || geometry.y() == -100)
  {
    if (id == -1)
    {
      if (show_debug_info_)
        ROS_ERROR_STREAM("Real World transform Failed");
    }
    else
    {
      if (show_debug_info_)
        ROS_ERROR_STREAM("Image Width or Height not match with yaml!");
    }
  }
  else
  {
    // tf2 transfrom msg
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "camera_down_link";
    transformStamped.child_frame_id = "tag_camera_down_node_" + std::to_string(id);
    transformStamped.transform.translation.x = geometry.x();
    transformStamped.transform.translation.y = geometry.y();
    transformStamped.transform.translation.z = geometry.z();
    transformStamped.transform.rotation.x = quaternion.x();
    transformStamped.transform.rotation.y = quaternion.y();
    transformStamped.transform.rotation.z = -1.0 * quaternion.z();
    transformStamped.transform.rotation.w = quaternion.w();
    tf_broadcaster_.sendTransform(transformStamped);
  }
}

void HikCamROS::fiducial_tfs_publish(const int &id, const Eigen::Vector3d &geometry,
                                     const Eigen::Quaterniond &quaternion)
{
  if (geometry.x() == -100 || geometry.y() == -100)
  {
    fiducial_msgs::FiducialTransformArray tfs;
    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "camera_down"; // 相機frame ID
    fiducial_tfs_pub_.publish(tfs);
  }
  else
  {
    fiducial_msgs::FiducialTransformArray tfs;
    fiducial_msgs::FiducialTransform tf;

    tfs.header.stamp = ros::Time::now();
    tfs.header.frame_id = "camera_down"; // 相機frame ID
    // tfs.image_seq = idName;           // 圖像序列號

    tf.fiducial_id = std::to_string(id);
    tf.transform.translation.x = geometry.x();
    tf.transform.translation.y = geometry.y();
    tf.transform.translation.z = geometry.z();
    tf.transform.rotation.x = quaternion.x();
    tf.transform.rotation.y = quaternion.y();
    tf.transform.rotation.z = -1.0 * quaternion.z();
    tf.transform.rotation.w = quaternion.w();

    tf.image_error = 0.0;
    tf.object_error = 0.0;
    tf.fiducial_area = 100.0;
    tfs.transforms.push_back(tf);
    fiducial_tfs_pub_.publish(tfs);
  }
}

sensor_msgs::ImagePtr HikCamROS::convertToROSImage(const cv::Mat &img)
{
  if (img.empty())
  {
    std::cerr << "Error: Input image is empty, cannot convert to ROS Image." << std::endl;
    return nullptr;
  }

  try
  {
    if (img.channels() == 1)
      return cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
    else
      return cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
  }
  catch (const cv_bridge::Exception &e)
  {
    std::cerr << "cv_bridge Exception: " << e.what() << std::endl;
    return nullptr;
  }
  catch (const std::exception &e)
  {
    std::cerr << "Standard Exception: " << e.what() << std::endl;
    return nullptr;
  }
  catch (...)
  {
    std::cerr << "Unknown error occurred during image conversion." << std::endl;
    return nullptr;
  }
}

cv::Mat HikCamROS::plot_message(const cv::Mat &img_gray, const bool &info_empty)
{
  int W = img_gray.cols;
  int H = img_gray.rows;

  //  GRAY to BGR
  cv::Mat annotated;
  cv::cvtColor(img_gray, annotated, cv::COLOR_GRAY2BGR);

  std::string label;
  if (info_empty)
  {
    // Because not have tag info and not clear the past info, so block
    label = cv::format("No information");
  }
  else
  {
    label = cv::format("[x:%d,y:%d,yaw:%f]", hikcam_.get_tagCenter_pixelX(), hikcam_.get_tagCenter_pixelY(),
                       hikcam_.get_yawRadian());
    // ---- 畫 Pixel 座標點 ----
    cv::Point pixel_point(hikcam_.get_tagCenter_pixelX(), hikcam_.get_tagCenter_pixelY());
    cv::circle(annotated, pixel_point, 5, cv::Scalar(0, 0, 255), 2);
  }

  // 右上角對齊：先拿字寬高，再算起點
  int fontFace = cv::FONT_HERSHEY_SIMPLEX;
  double scale = 0.6;
  int thickness = 1;
  int baseline = 0;
  int margin = 8;

  cv::Size sz = cv::getTextSize(label, fontFace, scale, thickness, &baseline);
  // org 是「基線左下角」；右上角要往左 (sz.width) 與往下 (sz.height) 偏移
  cv::Point org(W - sz.width - margin, margin + sz.height);

  // 灰階上做「實心黑底 + 白字」提高可讀性
  cv::Rect bg(org.x - 4, org.y - sz.height - 4, sz.width + 8, sz.height + baseline + 8);
  bg &= cv::Rect(0, 0, W, H); // 防越界
  cv::rectangle(annotated, bg, cv::Scalar(0), cv::FILLED);
  cv::putText(annotated, label, org, fontFace, scale, cv::Scalar(0, 255, 0), thickness, cv::LINE_AA);
  // ---- 標註結束 ----

  return annotated;
}