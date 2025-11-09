#ifndef HIK_INFO_DECODER_H  // 確保這個標頭檔只會被包含一次
#define HIK_INFO_DECODER_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <iconv.h>
#include <vector>

#include "MvCodeReaderParams.h"
#include "MvCodeReaderErrorDefine.h"
#include "MvCodeReaderCtrl.h"

#define MAX_BCR_LEN 512

class HikInfoDecoder
{
public:
  HikInfoDecoder();
  ~HikInfoDecoder();
  std::vector<std::string> decodeInfo(MV_CODEREADER_IMAGE_OUT_INFO_EX2* pFrameInfo, const char* delim);

private:
  char strChar_[MAX_BCR_LEN] = { 0 };

  int GB2312ToUTF8(char* szSrc, size_t iSrcLen, char* szDst, size_t iDstLen);
  void createInfoToken(char* message);

  char* token_ptr_ = nullptr;
};
#endif  // HIK_INFO_DECODER_H
