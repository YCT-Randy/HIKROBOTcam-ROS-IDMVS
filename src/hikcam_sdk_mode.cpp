#include <hik_camera/hik_camera.h>
#include <hik_camera/hik_camera_ros.h>
#include <ros/ros.h>
#include <hik_camera/hik_sdk_mode.h>

HikCamSDK::HikCamSDK()
{
}

HikCamSDK::~HikCamSDK()
{
  printf("HikCamSDK() closed \r\n");
}

// ch:中文编码GB2312格式转换UTF_8 | en: Chinese coding format GB2312 to utf_8
int HikCamSDK::GB2312ToUTF8(char* szSrc, size_t iSrcLen, char* szDst, size_t iDstLen)
{
  iconv_t cd = iconv_open("utf-8//IGNORE", "gb2312//IGNORE");
  if (0 == cd)
  {
    return -2;
  }

  memset(szDst, 0, iDstLen);
  char** src = &szSrc;
  char** dst = &szDst;
  if (-1 == (int)iconv(cd, src, &iSrcLen, dst, &iDstLen))
  {
    return -1;
  }

  iconv_close(cd);
  return 0;
}

bool HikCamSDK::isHandleNull()
{
  if (handle_)
    return false;
  else
    return true;
}

bool HikCamSDK::isCameraReady()
{
  return camera_isready_;
}

// ch:等待用户输入enter键来结束取流或结束程序
// en:wait for user to input enter to stop grabbing or end the sample program
void HikCamSDK::PressEnterToExit(void)
{
  int c;
  while ((c = getchar()) != '\n' && c != EOF)
    ;
  fprintf(stderr, "\nPress Enter to exit.\n");
  while (getchar() != '\n')
    ;
}

// ch:打印设备信息 | en:Print device Info
bool HikCamSDK::PrintDeviceInfo(MV_CODEREADER_DEVICE_INFO* pstMVDevInfo)
{
  if (NULL == pstMVDevInfo)
  {
    printf("The Pointer of pstMVDevInfo is NULL!\r\n");
    return false;
  }
  if (MV_CODEREADER_GIGE_DEVICE == pstMVDevInfo->nTLayerType)
  {
    int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
    printf("CurrentIp: %d.%d.%d.%d\r\n", nIp1, nIp2, nIp3, nIp4);
    printf("UserDefinedName: %s\r\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
  }
  else if (MV_CODEREADER_USB_DEVICE == pstMVDevInfo->nTLayerType)
  {
    printf("UserDefinedName: %s\r\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
  }
  else
  {
    printf("Not support.\r\n");
  }

  return true;
}

void HikCamSDK::processImage(unsigned char* pData, MV_CODEREADER_IMAGE_OUT_INFO_EX2* pFrameInfo)
{
  std::unique_lock<std::mutex> lock(image_lock_);
  size_t dataSize = pFrameInfo->nWidth * pFrameInfo->nHeight;
  img_buffer_.CreateImageBuffer(dataSize);
  img_buffer_.WriteImageData(pData);
  img_buffer_.WriteImageInfo(pFrameInfo);
}

// ch: 回调图像信息 | en:Register Image Call Back // This is a static member function
void __stdcall HikCamSDK::ImageCallBack(unsigned char* pData, MV_CODEREADER_IMAGE_OUT_INFO_EX2* pFrameInfo, void* pUser)
{
  int nRet = MV_CODEREADER_OK;
  if (!pData || !pUser)
  {
    std::cerr << "Error: Null pointer received in callback!\n";
    return;
  }
  else
  {
    HikCamSDK* self = static_cast<HikCamSDK*>(pUser);
    self->processImage(pData, pFrameInfo);
  }
}

void HikCamSDK::start_camera()
{
  int nRet = MV_CODEREADER_OK;
  // void* handle_ = NULL;
  bool bIsNormalRun = true;

  // ch:枚举设备 | Enum device
  MV_CODEREADER_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CODEREADER_DEVICE_INFO_LIST));
  MV_CODEREADER_EnumDevices(&stDeviceList, MV_CODEREADER_GIGE_DEVICE);
  while (stDeviceList.nDeviceNum <= 0)
  {
    printf("Enum Devices fail! nRet [0x%x]\r\n", nRet);
    memset(&stDeviceList, 0, sizeof(MV_CODEREADER_DEVICE_INFO_LIST));
    MV_CODEREADER_EnumDevices(&stDeviceList, MV_CODEREADER_GIGE_DEVICE);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  printf("Enum Devices succeed!\r\n");

  if (stDeviceList.nDeviceNum > 0)
  {
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
      printf("[device %d]:\r\n", i);
      MV_CODEREADER_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (NULL == pDeviceInfo)
      {
        // break;
      }
      PrintDeviceInfo(pDeviceInfo);
    }
  }
  else
  {
    printf("Find No Devices!\r\n");
    // break;
  }

  printf("Please Intput camera index:");
  unsigned int nIndex = 0;
  // scanf("%d", &nIndex);

  if (nIndex >= stDeviceList.nDeviceNum)
  {
    printf("Intput error!\r\n");
    // break;
  }

  // ch:选择设备并创建句柄 | Select device and create handle
  while (MV_CODEREADER_OK != MV_CODEREADER_CreateHandle(&handle_, stDeviceList.pDeviceInfo[nIndex]))
  {
    printf("Create Handle fail! nRet [0x%x]\r\n", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  printf("Create Handle succeed!\r\n");

  // ch:打开设备 | Open device
  while (MV_CODEREADER_OK != MV_CODEREADER_OpenDevice(handle_))
  {
    printf("Open Device fail! nRet [0x%x]\r\n", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  printf("Open Device succeed!\r\n");

  // ch:设置触发模式为off | eb:Set trigger mode as off
  while (MV_CODEREADER_OK != MV_CODEREADER_SetEnumValue(handle_, "TriggerMode", MV_CODEREADER_TRIGGER_MODE_OFF))
  {
    printf("Set Trigger Mode fail! nRet [0x%x]\r\n", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  printf("Set Trigger Mode succeed!\r\n");

  // ch:注册图像回调 | en:Register image callback
  while (MV_CODEREADER_OK != MV_CODEREADER_RegisterImageCallBackEx2(handle_, HikCamSDK::ImageCallBack, this))
  {
    printf("Register Image CallBack fail! nRet [0x%x]\r\n", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  printf("Register Image CallBack succeed!\r\n");

  // ch:开始取流 | en:Start grab image
  while (MV_CODEREADER_OK != MV_CODEREADER_StartGrabbing(handle_))
  {
    printf("Start Grabbing fail! nRet [0x%x]\r\n", nRet);
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  printf("Start Grabbing succeed!\r\n");
  camera_isready_ = true;
  printf("camera_isready_ True!\r\n");

  printf("Press Enter to stop grabbing.\r\n");
}

void HikCamSDK::stop_camera()
{
  if (handle_ != NULL)
  {
    // ch:停止取流 | en:Stop grab image
    int nRet = MV_CODEREADER_StopGrabbing(handle_);
    if (MV_CODEREADER_OK != nRet)
    {
      printf("Stop Grabbing fail! nRet [0x%x]\r\n", nRet);
    }
    else
    {
      printf("Stop Grabbing succeed!\r\n");
    }
    // ch:关闭设备 | en:Close device
    // ch:销毁句柄 | en:Destroy handle_
    MV_CODEREADER_CloseDevice(handle_);
    MV_CODEREADER_DestroyHandle(handle_);
    handle_ = NULL;
  }
  camera_isready_ = false;
}

void HikCamSDK::restart_camera()
{
  stop_camera();
  start_camera();
}

void HikCamSDK::yaml_init(const boost::array<double, 9>& K, const int& width, const int& height)
{
  detect_information_.yaml_init(K, width, height);
}

bool HikCamSDK::isImageBufferEmpty()
{
  return img_buffer_.isImageBufferEmpty();
}

MV_CODEREADER_IMAGE_OUT_INFO_EX2 HikCamSDK::getImageInfo()
{
  return img_buffer_.getImageInfo();
}

double HikCamSDK::get_yawRadian()
{
  return detect_information_.get_yawRadian();
}

double HikCamSDK::get_yawDegree()
{
  return detect_information_.get_yawDegree();
}

int HikCamSDK::get_tagID()
{
  return detect_information_.get_tagID();
}

Eigen::Vector3d HikCamSDK::get_realWorld_coordinate(const double& distance_z)
{
  return detect_information_.get_realWorld_coordinate(distance_z);
}

unsigned char* HikCamSDK::getImagePtr()
{
  return img_buffer_.getImagePtr();
}

int HikCamSDK::get_tagCenter_pixelX()
{
  return detect_information_.get_tagCenter_pixelX();
}

int HikCamSDK::get_tagCenter_pixelY()
{
  return detect_information_.get_tagCenter_pixelY();
}

bool HikCamSDK::read_info()
{
  if (!isHandleNull() && !img_buffer_.isImageBufferEmpty())
  {
    MV_CODEREADER_IMAGE_OUT_INFO_EX2 info = img_buffer_.getImageInfo();
    std::vector<std::string> tokens = info_decoder_.decodeInfo(&info, "@");
    if (tokens.size() < 5)
    {
      // Camera not detect any tag
      // Some time init camera or switch mode will send strange information
      // If this strange information set into tag info, will let process dead
      return false;
    }
    else if (tokens[3] == detect_information_.get_hik_info_time())
    {
      // Same info data, not need to set
      return true;
    }
    else
    {
      detect_information_.set_tag_info(tokens, static_cast<int>(img_buffer_.getImageInfo().nWidth),
                                       static_cast<int>(img_buffer_.getImageInfo().nHeight));
      return true;
    }
  }
  else
    return false;
}

std::string HikCamSDK::get_last_info_time()
{
  return detect_information_.get_hik_info_time();
}