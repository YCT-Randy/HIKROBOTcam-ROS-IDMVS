#include <hik_camera/hik_tag.h>

HikTag::HikTag()
{
}

HikTag::~HikTag()
{
  printf("HikTag() closed \r\n");
}

void HikTag::yaml_init(const boost::array<double, 9>& K, const int& width, const int& height)
{
  set_K_matrix(K);
  set_Precomputed_matrix();
  set_pixelHW(width, height);
}

void HikTag::set_K_matrix(const boost::array<double, 9>& K)
{
  K_matrix_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(K.data());
  std::cout << "K matrix:\n" << K_matrix_ << std::endl;
}

void HikTag::set_Precomputed_matrix()
{
  // Matrix3d[0, -1, 0, -1, 0, 0, 0, 0, 1]*K.inverse
  Eigen::Matrix3d R_cam2ros;
  R_cam2ros << 0, -1, 0, -1, 0, 0, 0, 0, 1;
  Precomputed_matrix_ = R_cam2ros * K_matrix_.inverse();
}

void HikTag::set_pixelHW(const int& width, const int& height)
{
  pixelW_ = width;
  pixelH_ = height;
}

int HikTag::get_pixelH()
{
  return pixelH_;
}

int HikTag::get_pixelW()
{
  return pixelW_;
}

void HikTag::set_tag_info(const std::vector<std::string>& info_pkg, const int& width, const int& height)
{
  check_info_error(width, height);
  set_tag_id(info_pkg[0]);
  set_XYandDegree(info_pkg[1]);
  set_hik_time(info_pkg[3]);
}

void HikTag::check_info_error(const int& width, const int& height)
{
  if (width != pixelW_ || height != pixelH_)
  {
    can_not_to_real_ = true;
  }
  else
  {
    can_not_to_real_ = false;
  }
}

void HikTag::set_tag_id(const std::string& info)
{
  try
  {
    id_ = std::stoi(info);
  }
  catch (const std::invalid_argument& e)
  {
    std::cerr << "stoi invalid_argument: " << e.what() << " for string: [" << info << "]\n";
  }
  catch (const std::out_of_range& e)
  {
    std::cerr << "stoi out_of_range: " << e.what() << " for string: [" << info << "]\n";
  }
}

int HikTag::get_tagID()
{
  return id_;
}

void HikTag::set_XYandDegree(const std::string& info)
{
  size_t start = info.find('(') + 1;  // 找到 '(' 之後的位置
  size_t middle = info.find('x');     // 找到 'x' 來分割 x 和 y
  size_t end = info.find(')');        // 找到 ')'

  if (start == std::string::npos || middle == std::string::npos || end == std::string::npos)
  {
    std::cerr << "格式錯誤" << std::endl;
    return;
  }

  // 解析 x 和 y
  centerX_ = static_cast<int>(std::stod(info.substr(start, middle - start)));         // x
  centerY_ = static_cast<int>(std::stod(info.substr(middle + 1, end - middle - 1)));  // y
  yawDegree_ = static_cast<double>(std::stod(info.substr(end + 1)));                  // 轉換為 double
}

int HikTag::get_tagCenter_pixelX()
{
  return centerX_;
}

int HikTag::get_tagCenter_pixelY()
{
  return centerY_;
}

Eigen::Vector3d HikTag::get_realWorld_coordinate(const double& distance_z)
{
  // pixel coords=K*camera coords (normalized)
  // normalized camera coords=K.inverse*p
  // camera coords in real world=K.inverse*p*distance
  // ROS coords = Matrix3d[0, -1, 0, -1, 0, 0, 0, 0, 1] * normalized camera coords
  // ROS coords in real world = Matrix3d[0, -1, 0, -1, 0, 0, 0, 0, 1]*K.inverse*p*distance
  if (can_not_to_real_)
  {
    Eigen::Vector3d p(-100, -100, -100);
    return p;
  }
  else
  {
    Eigen::Vector3d p(centerX_, centerY_, 1);
    Eigen::Vector3d world_p = Precomputed_matrix_ * p * distance_z;
    return world_p;
  }
}

double HikTag::get_yawDegree()
{
  return yawDegree_;
}

double HikTag::get_yawRadian()
{
  return yawDegree_ * M_PI / 180.0;
}

void HikTag::set_hik_time(const std::string& info)
{
  hik_info_time_ = info;
}

std::string HikTag::get_hik_info_time()
{
  return hik_info_time_;
}
