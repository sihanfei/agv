# common

## 介绍
包含通用功能模块。

## glog 日志模块使用说明

### 1、源码中添加h文件
在的节点源码文件中加入 `#include "glog_helper.h"`

### 2、修改CMakeLists.txt
- 在`CMakeLists.txt`文件的 `include_directories` 中加入  
```
    include_directories(
    ~/work/superg_agv/src/third_party/glog/include  
    ~/work/superg_agv/src/common/include  
    ~/superg_agv/src/third_party/glog/include  
    ~/superg_agv/src/common/include  
    )
```

- 在 link_directories 块中加入
```
    link_directories(
    ~/work/superg_agv/src/third_party/glog/lib
    ~/superg_agv/src/third_party/glog/lib
  )
```
- 在 catkin_package 块中加入
```
    catkin_package(
    CATKIN_DEPENDS
    LIBRARIES glog_helper
  )
  ```
- 在生成可执行文件的 target_link_libraries 中加入
```
  target_link_libraries(tanway_m16_lidar_node
   glog_helper
  ${catkin_LIBRARIES}
)
```
  >注意：
  此处路径应按照各自工作区目录修改。

### 3、初始化：
- 使用为当前可执行文件的名称为日志文件名称  
  `GLogHelper gh(argv[0]);`
  
- 使用自定义日志名称  
`GLogHelper gh(( char * )"xxx_log");`

### 4、设置日志文件输出目录

  `gh.setLogDirectory("/work/log/xxx");`  
  依照数据落盘规范，使用“～/work/log/”作为所有日志文件的根目录，“xxx”为节点名称，也可自定义，名称尽量清晰明确。
  举例：  
  `gh.setLogDirectory("/work/log/control");`  
  为control节点使用“～/work/log/control/”为日志存储目录。

### 5、日志输出：
  ```
  SUPERG_INFO <<"info test";//输出一个Info日志  
  SUPERG_WARN <<"warning test";//输出一个Warning日志  
  SUPERG_ERROR <<"error test";//输出一个Error日志  
  SUPERG_FATAL <<"fatal test";//输出一个Fatal日志  
  ```

>特别注意：  
>使用`SUPERG_FATAL`输出之后会【中止程序】。

### 6、条件输出：
```
  SUPERG_LOG_IF(条件) <<"日志";//当条件满足时输出日志
  SUPERG_WARN_IF(条件) <<"日志";
  SUPERG_ERROR_IF(条件) <<"日志";
```
-  每10次再输出一次日志信息，可以自定义间隔  
  `SUPERG_INFO_EVERY(10) <<"Got the "<< iCOUNTER <<"th cookie";`

### 7、其他：
  以上说明未尽之处，请查看https://github.com/google/glog

### 8、demo
  ```sh
  GLogHelper gh(( char * )"xxx_log");
  gh.setLogDirectory("/work/log/xxx"); // xxx=节点名

  for (int i = 0; i < 100; i++)
  {
    SUPERG_INFO << "Hello,info! ";
    SUPERG_ERROR << "Hello erroe! " << i;
    SUPERG_WARN << "Hello,waring! " << i << "+" << i << "=" << i + i;
    //LOG(FATAL) << "Hello,fatal! ";

    SUPERG_INFO_IF((i % 10) == 0) << "Hello,info_if " << i;
    SUPERG_INFO_EVERY(20) << "Hello,info_every " << i;
  }
  ```

### 参与贡献
  林海