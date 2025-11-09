#include <hik_camera/hik_camera.h>
#include <hik_camera/hik_camera_ros.h>

#include <hik_camera/hik_imgbuffer.h>

HikImageBuffer::HikImageBuffer()
{
}

HikImageBuffer::~HikImageBuffer()
{
  delete[] image_ptr_;
  printf("HikImageBuffer() closed \r\n");
}

void HikImageBuffer::CreateImageBuffer(const size_t& buffer_size)
{
  if (image_ptr_ == nullptr)
  {
    image_ptr_ = new unsigned char[buffer_size];
    image_buffer_size_ = buffer_size;
  }
  else if (image_buffer_size_ != buffer_size)
  {
    delete[] image_ptr_;
    image_ptr_ = new unsigned char[buffer_size];
    image_buffer_size_ = buffer_size;
  }
}

void HikImageBuffer::WriteImageData(unsigned char* pData)
{
  std::memcpy(image_ptr_, pData, image_buffer_size_);
}

void HikImageBuffer::WriteImageInfo(MV_CODEREADER_IMAGE_OUT_INFO_EX2* pFrameInfo)
{
  image_info_ = *pFrameInfo;
}

bool HikImageBuffer::isImageBufferEmpty()
{
  return (image_ptr_ == nullptr);
}

unsigned char* HikImageBuffer::getImagePtr()
{
  return image_ptr_;
}
MV_CODEREADER_IMAGE_OUT_INFO_EX2 HikImageBuffer::getImageInfo()
{
  return image_info_;
}
