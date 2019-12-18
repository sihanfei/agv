# drivers/conti_radar_driver

介绍
===
>Conti 毫米波雷达驱动

- <sup>1</sup>该驱动位于数翔工程目录的`superg_agv/src/drivers/`下，驱动包名称`conti_radar_driver`

- <sup>2</sup>编译后的启动节点名称为`cannet_radar_node`

- <sup>3</sup>在启动节点前，需要确认配置文件中各项参数是否正确
- <sup>4</sup>配置文件在该驱动包的`/cfg` 目录下，名称为`config.yaml`

```
log_dir: "/work/log/"                              #upd原始数据日志记录存放目录  
sensor_name: Conti                                 #传感器产品名称  
device_list:                                       #要启动的设备列表，当前配置文件中预制了6个设备  
device_name: radar1                                #设备名称 用于区别安装于车辆上的多个设备  
  device_enable: true                              #设备是否启用 true=启用 false=不启用  
  device_ip: "192.168.2.121"                       #设备配置的IP地址 毫米波雷达是通过CAN转NET接入的，这里是CANNET的接入IP  
  intput_port: 8001                                #设备上传数据到工控机的UDP端口，指的是工控机上的UDP接收端口  
  topic_name_1: "/drivers/radar1/radar_obstacle"   #设备数据接收后转换的 障碍物TOPIC名称  
  topic_name_2: "/drivers/radar1/radar_cloud"      #设备数据接收后转换的 点云TOPIC名称  
```
- <sup>5</sup>配置文件无误后运行以下命令启动  
`rosrun conti_radar_driver cannet_radar_node`  



