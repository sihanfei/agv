
#pragma once
#ifndef COMMON_GLOG_HELPER_H
#define COMMON_GLOG_HELPER_H

#include "glog/logging.h"
#include "glog/raw_logging.h"

#include <stdlib.h>
using namespace std;

#define SUPERG_DEBUG LOG(DEBUG)

#define SUPERG_INFO LOG(INFO)
#define SUPERG_WARN LOG(WARNING)
#define SUPERG_ERROR LOG(ERROR)
#define SUPERG_FATAL LOG(FATAL)

// LOG_IF
#define SUPERG_INFO_IF(cond) LOG_IF(INFO, cond)
#define SUPERG_WARN_IF(cond) LOG_IF(WARN, cond)
#define SUPERG_ERROR_IF(cond) LOG_IF(ERROR, cond)

// LOG_EVERY_N
#define SUPERG_INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define SUPERG_WARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define SUPERG_ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

//配置输出日志的文件夹：
#define LOGDIR "work/log/superglog/"
#define MKDIR "mkdir -p " LOGDIR

class GLogHelper
{
public:
  // GLOG配置：
  GLogHelper(char *program);
  // GLOG内存清理：
  ~GLogHelper();
  void setLogDirectory(const char *logDir);
};

#endif // COMMON_GLOG_HELPER_H_