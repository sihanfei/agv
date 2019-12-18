#include "glog_helper.h"

//将信息输出到单独的文件和 LOG(ERROR)
void SignalHandle(const char *data, int size)
{
  std::string str = std::string(data, size);
  /*
  std::ofstream fs("glog_dump.log",std::ios::app);
  fs<<str;
  fs.close();
  */
  LOG(ERROR) << str;
  //也能够直接在这里发送邮件或短信通知。只是这种方法是被回调多次的（每次回调仅仅输出一行错误信息，所以如上面的记录到文件，也须要>以追加模式方可）。
  //所以这里发邮件或短信不是非常适合。只是倒是能够调用一个
  // SHELL 或 PYTHON 脚本，而此脚本会先 sleep 3秒左右，然后将错
  //误信息通过邮件或短信发送出去，这样就不须要监控脚本定时高频率运行，浪费效率了。
}
// GLOG配置：
GLogHelper::GLogHelper(char *program)
{
  system(MKDIR);
  google::InitGoogleLogging(program);

  google::SetStderrLogging(google::INFO); //设置级别高于 google::INFO 的日志同一时候输出到屏幕
  FLAGS_colorlogtostderr = true;          //设置输出到屏幕的日志显示对应颜色
  // google::SetLogDestination(google::ERROR,"log/error_");    //设置 google::ERROR 级别的日志存储路径和文件名称前缀
  google::SetLogDestination(google::INFO, LOGDIR "/INFO_"); //设置 google::INFO 级别的日志存储路径和文件名称前缀
  google::SetLogDestination(google::WARNING,
                            LOGDIR "/WARNING_"); //设置 google::WARNING 级别的日志存储路径和文件名称前缀
  google::SetLogDestination(google::ERROR, LOGDIR "/ERROR_"); //设置 google::ERROR 级别的日志存储路径和文件名称前缀
  FLAGS_logbufsecs                = 0;    //缓冲日志输出，默觉得30秒。此处改为马上输出
  FLAGS_max_log_size              = 100;  //最大日志大小为 100MB
  FLAGS_stop_logging_if_full_disk = true; //当磁盘被写满时，停止日志输出
  google::SetLogFilenameExtension("M6_"); //设置文件名称扩展。如平台？或其他须要区分的信息
  google::InstallFailureSignalHandler();  //捕捉 core dumped
  google::InstallFailureWriter(
      &SignalHandle); //默认捕捉 SIGSEGV 信号信息输出会输出到 stderr，能够通过以下的方法自己定义输出>方式：
}
// GLOG内存清理：
GLogHelper::~GLogHelper()
{
  google::ShutdownGoogleLogging();
}

void GLogHelper::setLogDirectory(const char *logDir)
{

  char log_file_path[1024] = {0};
  sprintf(log_file_path, "%s%s", LOGDIR, logDir);
  system((string("mkdir -p ") + string(log_file_path)).c_str());
  // google::SetLogDestination(google::ERROR,"log/error_");    //设置 google::ERROR 级别的日志存储路径和文件名称前缀
  google::SetLogDestination(google::INFO, (string(log_file_path) + string("/INFO_")).c_str());
  //设置 google::INFO 级别的日志存储路径和文件名称前缀
  google::SetLogDestination(google::WARNING, (string(log_file_path) + string("/WARNING_")).c_str());
  //设置 google::WARNING 级别的日志存储路径和文件名称前缀
  google::SetLogDestination(google::ERROR, (string(log_file_path) + string("/ERROR_")).c_str());
  //设置 google::ERROR 级别的日志存储路径和文件名称前缀
}