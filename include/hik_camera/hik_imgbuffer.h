#ifndef HIK_IMAGE_BUFFER_H
#define HIK_IMAGE_BUFFER_H

#include "MvCodeReaderParams.h"
#include "MvCodeReaderErrorDefine.h"
#include "MvCodeReaderCtrl.h"

#include <opencv2/opencv.hpp>

#define MAX_BCR_LEN 512

class HikImageBuffer
{
public:
  HikImageBuffer();
  ~HikImageBuffer();

  void CreateImageBuffer(const size_t& buffer_size);
  void WriteImageData(unsigned char* pData);
  void WriteImageInfo(MV_CODEREADER_IMAGE_OUT_INFO_EX2* pFrameInfo);
  bool isImageBufferEmpty();
  unsigned char* getImagePtr();
  MV_CODEREADER_IMAGE_OUT_INFO_EX2 getImageInfo();

private:
  MV_CODEREADER_IMAGE_OUT_INFO_EX2 image_info_;
  unsigned char* image_ptr_ = nullptr;

  size_t image_buffer_size_;
};
#endif  // HIK_IMAGE_BUFFER_H
