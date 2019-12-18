## ContiRadar驱动

### Build

新建一个ROS的工作空间，将conti_radar 和 conti_radar_msgs放到该工作空间中，然后执行下面的命令

```shell
cd catkin_ws
catkin_make
```

### Run

```shell
roslaunch conti_radar conti_radar.launch
```

conti_radar.launch的具体内容如下：

```xml
<launch>
    <node name="conti_radar" pkg="conti_radar" type = "conti_radar" output="screen">
	<param name="config_file"  type="string" value="$(find conti_radar)/config/conti_radar_config.ini" />
	<param name="frame_id"     type="string" value="conti_radar" />
    </node>
</launch>

```

其中：

- config_file 表示参数文件的地址
- frame_id 表示毫米波雷达消息的frame_id



ROS消息定义在conti_radar_msgs中

