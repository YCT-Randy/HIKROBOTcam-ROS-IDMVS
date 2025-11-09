#include <hik_camera/hik_camera.h>
#include <hik_camera/hik_camera_ros.h>

#include <hik_camera/hik_infodecoder.h>

HikInfoDecoder::HikInfoDecoder()
{
}

HikInfoDecoder::~HikInfoDecoder()
{
  printf("HikInfoDecoder() closed \r\n");
}

int HikInfoDecoder::GB2312ToUTF8(char* szSrc, size_t iSrcLen, char* szDst, size_t iDstLen)
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

std::vector<std::string> HikInfoDecoder::decodeInfo(MV_CODEREADER_IMAGE_OUT_INFO_EX2* pFrameInfo, const char* delim)
{
  MV_CODEREADER_RESULT_BCR_EX* stBcrResult = (MV_CODEREADER_RESULT_BCR_EX*)pFrameInfo->pstCodeListEx;

  memset(strChar_, 0, MAX_BCR_LEN);
  int nRet = GB2312ToUTF8(stBcrResult->stBcrInfoEx[0].chCode, strlen(stBcrResult->stBcrInfoEx[0].chCode), strChar_,
                          MAX_BCR_LEN);
  if (nRet == MV_CODEREADER_OK)
  {
    // printf("%s \r\n", strChar_);
  }
  else
  {
    // printf("CodeNum[%d] Code[%s]\r\n", i, stBcrResult->stBcrInfoEx[i].chCode);
  }

  std::vector<std::string> tokens;        // 存放切割後的字串
  char* token = strtok(strChar_, delim);  // 以 `delim` 切割
  while (token != NULL)
  {
    tokens.push_back(token);  // 存入 vector
    token = strtok(NULL, delim);
    if (tokens[0].front() == '<')
      tokens[0].erase(0, 1);
    if (tokens.back().back() == '>')
      tokens.back().pop_back();
  }
  return tokens;
}
