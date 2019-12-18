1、编译整个工程

2、在SUPERG_AGV/src/drivers/obu_driver/cfg/目录下找到config.yaml文件，配置OBU的参数。

3、新开一个终端窗口执行 rosrun obu_driver fixed_lidar_info_node

4、在另一个终端窗口中执行 rostopic echo /drivers/localization/fixed_lidar_msg
