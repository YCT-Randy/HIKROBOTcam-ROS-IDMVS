#include <hik_camera/hik_camera.h>
#include <hik_camera/hik_camera_ros.h>

#include <hik_camera/hik_udp_mode.h>
#include <cstdint>  // 放在檔頭 includes 區

HikUDP::HikUDP()
{
}

HikUDP::~HikUDP()
{
  printf("HikUDP() closed \r\n");
}

// 4 bytes Big-Endian → int32
static int32_t be_to_i32(const unsigned char* p)
{
  uint32_t u = (uint32_t(p[0]) << 24) | (uint32_t(p[1]) << 16) | (uint32_t(p[2]) << 8) | (uint32_t(p[3]));
  return static_cast<int32_t>(u);
}

// 取倒數 20 bytes，從這 20 的開頭起，4 bytes 一組印成 signed int
std::vector<double> HikUDP::decode_udp_msg(const unsigned char*& buf, size_t& n)
{
  size_t start = (n > 20) ? (n - 20) : 0;  // 不足 20 則從 0 開始
  size_t len = n - start;                  // 實際可用長度（<=20）
  size_t groups = len / 4;                 // 可湊成幾組 int32

  // std::cout << "[last " << len << " bytes → " << groups << " x int32 BE] ";

  std::vector<double> temp;

  for (size_t i = 0; i < groups; ++i)
  {
    int32_t raw = be_to_i32(buf + start + i * 4);  // 4 bytes → 有正負號的 int32
    double val;
    if (i == 2)
      val = static_cast<double>(raw) * 1e-3;  // 乘 0.001 用 double
    else
      val = static_cast<double>(raw) * 1e-6;  // 乘 0.000001 用 double
    temp.push_back(val);
  }
  return temp;
}

void HikUDP::show_packet_hex(const unsigned char*& p, size_t& n)
{
  for (size_t i = 0; i < n; ++i)
  {
    if (i && i % 16 == 0)
      std::cout << "\n";
    std::cout << std::hex << std::setw(2) << std::setfill('0') << (unsigned)p[i] << ' ';
  }
  std::cout << std::dec << "\n";
}

std::vector<std::string> HikUDP::decode_sdk_msg(const unsigned char*& buf, size_t& len)
{
  bool start = false;
  std::string sdk_msg;
  std::vector<std::string> sdk_msg_vector;
  for (size_t i = 0; i < len; ++i)
  {
    // 遇到 '<' 開始記錄
    // 有遇過連續兩個'<'
    if (buf[i] == '<')
    {
      start = true;
      sdk_msg.clear();
    }
    else if (start)
    {
      if (buf[i] == '@')
      {
        if (sdk_msg.empty())
        {
          // Defend->stoi invalid_argument: stoi for string: []
          // decode_packet will not set info if vector empty
          sdk_msg_vector.clear();
          break;
        }
        else
        {
          sdk_msg_vector.push_back(sdk_msg);
          sdk_msg.clear();
        }
      }
      else if (buf[i] == '>')
      {
        // 遇到第一個 '>' 就結束
        if (sdk_msg.empty())
        {
          // Defend->stoi invalid_argument: stoi for string: []
          sdk_msg_vector.clear();
        }
        else
          sdk_msg_vector.push_back(sdk_msg);
        break;
      }
      else
      {
        if (!std::isprint(buf[i]))
        {
          // Defend->stoi invalid_argument: stoi for string: can not print
          // decode_packet will not set info if vector empty
          sdk_msg_vector.clear();
          // ROS_ERROR_STREAM("error buf[i]:" << buf[i]);
          break;
        }
        sdk_msg.push_back(static_cast<char>(buf[i]));
      }
    }
  }
  // for (const auto& s : sdk_msg_vector)
  // {
  //   std::cout << s << '\n';
  // }

  return sdk_msg_vector;
}

void HikUDP::run()
{
  int port = 1024;               // 例：./udp_recv_min 8888 enp4s0
  std::string iface = "enp4s0";  // 可選：enp4s0

  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0)
  {
    perror("socket");
  }

  // 可重複綁定（重啟程式不會卡）
  int yes = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  // 只收指定介面的封包（需要 root；失敗不致命）
#ifdef SO_BINDTODEVICE
  if (!iface.empty())
  {
    if (setsockopt(fd, SOL_SOCKET, SO_BINDTODEVICE, iface.c_str(), iface.size() + 1) < 0)
    {
      std::cerr << "warn: SO_BINDTODEVICE(" << iface << "): " << strerror(errno) << "\n";
    }
  }
#endif

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);  // 若要只收該介面 IP，可改成該 IP
  addr.sin_port = htons(port);
  if (bind(fd, (sockaddr*)&addr, sizeof(addr)) < 0)
  {
    perror("bind");
  }

  std::cout << "Listening UDP on port " << port;
  if (!iface.empty())
    std::cout << " (iface=" << iface << ")";
  std::cout << " ... Ctrl+C to stop\n";

  std::vector<unsigned char> buf(65536);
  while (camera_isready_)
  {
    sockaddr_in peer{};
    socklen_t sl = sizeof(peer);
    ssize_t n = recvfrom(fd, buf.data(), buf.size(), 0, (sockaddr*)&peer, &sl);
    if (n < 0)
    {
      camera_isready_ = false;
      if (errno == EINTR)
        continue;
      perror("recvfrom");
      break;
    }
    else if (n == 268)
    {
      char ip[INET_ADDRSTRLEN];
      inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));
      // std::cout << "from " << ip << ":" << ntohs(peer.sin_port) << "  len=" << n << "\n";
      decode_packet(buf.data(), (size_t)n);
    }
  }

  close(fd);
  ROS_ERROR_STREAM("fd closed");
}

void HikUDP::decode_packet(const unsigned char* p, size_t n)
{
  // show_packet_hex(p, n);  // 以 16 進位列印 payload
  std::vector<std::string> sdk_msg_vector = decode_sdk_msg(p, n);
  if (!sdk_msg_vector.empty())
  {
    sdk_information_.set_tag_info(sdk_msg_vector, 800, 600);
    std::vector<double> udp_msg_vector = decode_udp_msg(p, n);

    mtx_.lock();
    Eigen::Vector3d pose(-1.0 * udp_msg_vector[1], -1.0 * udp_msg_vector[0], 1);
    detect_buffer_.clear();
    detect_buffer_.tag_id = sdk_information_.get_tagID();
    detect_buffer_.tagCenter_pixelX = sdk_information_.get_tagCenter_pixelX();
    detect_buffer_.tagCenter_pixelY = sdk_information_.get_tagCenter_pixelY();
    detect_buffer_.tag_yawDegree = sdk_information_.get_yawDegree();
    detect_buffer_.tag_yawRadian = sdk_information_.get_yawRadian();
    detect_buffer_.pixelH = sdk_information_.get_pixelH();
    detect_buffer_.pixelW = sdk_information_.get_pixelW();
    detect_buffer_.info_time = sdk_information_.get_hik_info_time();
    detect_buffer_.realWorld_pose = pose;
    mtx_.unlock();
  }
}

void HikUDP::start_camera()
{
  camera_isready_ = true;

  worker_ = std::thread(&HikUDP::run, this);
}

void HikUDP::stop_camera()
{
  camera_isready_ = false;
  if (worker_.joinable())
  {
    worker_.join();
  }
}

void HikUDP::restart_camera()
{
  stop_camera();
  start_camera();
}

void HikUDP::yaml_init(const boost::array<double, 9>& K, const int& width, const int& height)
{
}

bool HikUDP::isHandleNull()
{
  return !camera_isready_;
}

bool HikUDP::isCameraReady()
{
  return camera_isready_;
}

bool HikUDP::isImageBufferEmpty()
{
  return true;
}

MV_CODEREADER_IMAGE_OUT_INFO_EX2 HikUDP::getImageInfo()
{
  MV_CODEREADER_IMAGE_OUT_INFO_EX2 n;
  return n;
}

double HikUDP::get_yawRadian()
{
  return detect_result_.tag_yawRadian;
}

double HikUDP::get_yawDegree()
{
  return detect_result_.tag_yawDegree;
}

int HikUDP::get_tagID()
{
  return detect_result_.tag_id;
}

Eigen::Vector3d HikUDP::get_realWorld_coordinate(const double& distance_z)
{
  return detect_result_.realWorld_pose;
}

unsigned char* HikUDP::getImagePtr()
{
  return nullptr;
}

int HikUDP::get_tagCenter_pixelX()
{
  return detect_result_.tagCenter_pixelX;
}

int HikUDP::get_tagCenter_pixelY()
{
  return detect_result_.tagCenter_pixelY;
}

bool HikUDP::read_info()
{
  mtx_.lock();
  detect_result_.clear();
  detect_result_ = std::move(detect_buffer_);
  mtx_.unlock();

  if (!detect_result_.empty())
    return true;
  else
    return false;
}

std::string HikUDP::get_last_info_time()
{
  return detect_result_.info_time;
}