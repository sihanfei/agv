Update Date: 2019/03/25


Function:
1)	运行　roslaunch ml16_view TensorLite05.launch 
可正常显示TensorLite05套点云

2)	运行　roslaunch ml16_view TensorLite06.launch 
可正常显示TensorLite06套点云

3)	运行　roslaunch ml16_view display_nodes.launch 
可显示TensorLite05&06套点云

4)	运行　roslaunch ml16_view dynamic_nodes.launch 
可显示TensorLite05&06套点云, 并动态调节参数


配置文件TensorLite0x.launch在ml16_view中launch文件夹下，其中可配置参数说明如下：

1)	PC端IP地址 
参数名为“host”，默认值为“192.168.111.204”。
2)	PC端端口号
参数名为“port”，默认值为“5600”。
3)	雷达IP地址 
参数名为“LiDARhost”，默认值为“192.168.111.31”。
4)	雷达端口号
参数名为“LiDARport”，默认值为“5050”。
5)	坐标系名称
参数名为“frame_id”，默认值为“TanwayML16”。
说明激光雷达上传ROS的数据所属坐标系，在应用中各个传感器坐标转换时使用
6)	话题名称
参数名为“topic”，默认值为“/ml16_cloud”，定义ROS topic名称。
7)	激光雷达显示的可用角度，包括开始角度与结束角度
参数名分别为“StartAngle”和“EndAngle”，默认值分别为2和350。
8)	通道静态量（x: 1-16）
参数名为“StaticQuantityFirst_x”，默认值为“0”，表示各通道在系统内部的固有误差修正值。如“StaticQuantity_1”表示第一个通道的静态量。
9)	颜色
参数名为“Color”，默认值为“Indoor”。可设置为 None/Indoor/Outdoor，分别显示固定颜色/室内场/室外场景颜色。
10)	点云的刚性变化状态
参数名为“transformCloud_Status”，布尔型变量，默认值为“False”。值为“True”时，可分别设置trans_x/ trans_y/ trans_z/ rotate_theta_xy/ rotate_theta_xz/ rotate_theta_yz进行点云的平移、旋转变化。
11)	GPS时间戳
参数名为“GPS_Status”，布尔型变量，默认值为“False”。值为“True”时，时间戳为GPS同步后的时间，否则为ROS系统时间。
