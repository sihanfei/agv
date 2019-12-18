# Release 1.0

## 首次安装AGV开机启动服务

### 1、将`/monitor/rivz_monitor/scripts/agvservice.service` 文件复制到海高的`/etc/systemd/system/`下

### 2、并给agvservice.service修改权限

`sudo chmod 777 agvservice.service`

### 3、执行使能命令

 `sudo systemctl enable agvservice.service`

## 重新加载配置文件

$ `sudo systemctl daemon-reload`

## 使能AGV开机启动服务

`systemctl enable agvservice.service`

## 注销AGV开机启动服务

`systemctl disable agvservice.service`

## 显示所有已启动的服务

`systemctl list-units --type=service`

## 启动AGV开机启动服务

`systemctl start agvservice.service`

## 停止AGV开机启动服务

`systemctl stop agvservice.service`

## 重启AGV开机启动服务

`systemctl restart agvservice.service`

## 检查AGV开机启动服务状态

`systemctl status agvservice.service` （服务详细信息）

## 卸载ROS包

### 1、首先卸载包

`sudo apt-get purge ros-indigo-**`

### 2、然后卸载依赖包

`sudo apt-get autoremove`

### rosbridge 部署中的问题

`sudo apt-get install ros-kinetic-rosauth`

`sudo apt-get install ros-kinetic-rosbridge-suite`

`sudo apt-get install ros-kinetic-tf2-web-republisher`

`sudo apt-get install python-pip`

`pip install tornado==4.2.1`

ImportError: No module named bson

`pip install pymongo==3.8.0`

## 地图服务启动错误解决

ImportError: No module named 'networkx'
sudo apt install python3-pip
安装networkx 命令：`sudo pip3 install networkx`

ImportError: No module named 'matplotlib'
安装matplotlib 命令： `sudo pip3 install matplotlib`

## screen 使用说明

### 1、使用ssh远程连接到海高工控机

`ssh higo@192.168.2.200`

### 2、海高安装screen,在ssh终端中执行

`sudo apt-get install screen`

### 3、常用命令

screen的说明相当复杂，但事实上，我们只需要掌握下面五个命令就足够我们使用了：

screen -S yourname      #新建一个叫yourname的session

screen -ls              #列出当前所有的session

screen -r yourname      # 回到yourname这个session

screen -d yourname      # 远程detach某个session

exit                    #退出当前窗口

### 4、建议使用快捷键

Ctrl+a c ：在当前screen会话中创建窗口

Ctrl+a d  : 效果与screen -d相同，卸载当前会话

Ctrl+a w ：显示当前会话中的窗口列表，显示在标题栏中

Ctrl+a n ：切换到下一个窗口

Ctrl+a p ：切换到上一个窗口

Ctrl+a 0-9 ：在第0个窗口和第9个窗口之间切换

说明：这里的快捷键由三个键组成，如`Ctrl+a c`，你可以按住Ctrl键，再依次按下a和c。也可以先按一次`Ctrl+a`，再按一次`Ctrl+c`。两种方法都是可行的。

## 示例

1、在使用ssh连接到海高的情况下，在ssh终端中执行：`screen -S linh-roscore` #创建一个名为 linh-roscore 的会话，采用的是创建者姓名+要执行的命令名称这种方式，方便在之后的检索中知道每个会话是谁建立的，用来执行什么命令。

2、在新创建的screen会话中，执行`roscore`。

3、先按一次`Ctrl+a`，再按一次`Ctrl+d`。分离当前会话。

4、执行`screen -ls`会显示所有的会话列表，如下：

```bash
higo@higo-WiseADCU-M6:~$ screen -ls
There are screens on:
 29995.linh-roscore (2019年07月22日 13时51分55秒) (Detached)
 3039.linh3 (2019年07月18日 16时32分22秒) (Detached)
 2826.linh2 (2019年07月18日 16时31分47秒) (Detached)
 2410.linh1 (2019年07月18日 16时29分00秒) (Detached)
4 Sockets in /var/run/screen/S-higo.
higo@higo-WiseADCU-M6:~$
```

5、执行`screen -r linh-roscore`或者`screen -r 29995`就可以恢复到指定的会话窗口。

## 如何用bash脚本创建screen并向其发送命令

```bash
如果我们直接在脚本里写 screen -S my_screen, 会导致脚本无法继续执行。为了使脚本执行下去，创建screen的具体代码如下：

screen_name=$"my_screen"
screen -dmS $screen_name
现在，我们就已经创建了一个名为 my_screen 的窗口。然后，我们需要向其发送具体的命令。我们用如下命令：
cmd=$"java Test";
screen -x -S $screen_name -p 0 -X stuff "$cmd"
screen -x -S $screen_name -p 0 -X stuff $'\n'
这样，我们就向screen发送了一条java Test命令了。
如果要退出窗口，则再向其发送一条 exit 命令即可
```

打开终端

ubuntu 在终端下载glog包
`git clone https://github.com/google/glog`

安装所依赖的包
`sudo apt-get install autoconf automake libtool`

进入到下载的glog包里面
`cd glog`

之后创建build
`mkdir build && cd build`

执行cmake
`cmake ..`
`sudo make install`

安装完成了

## 8080

`superg.qicp.vip:56484?url=ws://superg.qicp.vip:37482`

## 9090

`superg.qicp.vip:37482`

```bash
ssh -oPort=6001 higo@127.0.0.1

ssh  ubuntu@106.54.1.81

agv.nvzo.com:8080/?url=ws://106.54.1.81:9091
```

## 关于激光雷达驱动的处理

当前所用的两个品牌激光雷达，rmp都是600，一分钟转600圈，1秒转10圈
velodyne激光雷达 1秒钟发出754个数据包，那么激光雷达转一圈，相当于发出大约76个数据包
速腾雷达 1秒钟发出840个数据包，转一圈发出84个数据包。

是否可以考虑 不用等一帧点云（360度）（76包或84包）完全接收后再通过定位数据转换全局坐标
而是每接收到一个数据包后就通过订阅当前的定位数据，进行全局坐标转换。
解决点云数据和定位信息时间同步的问题。

## 落盘数据硬盘挂载

### 1、查看Linux硬盘信息

`$ sudo fdisk -l`

### 2、临时挂载分区

`sudo mount /dev/sda /home/higo/work/log`

### 3、查看磁盘分区的UUID

`$ sudo blkid`

### 4、配置开机自动挂载

mount命令重启后会失效，将分区信息写入 /etc/fstab 文件启动自动挂载：

`sudo vim /etc/fstab`
加入：

`UUID=xxxxxxxx /home/higo/work/log ntfs defaults 0 0`

### 5、重启系统

修改完/etc/fstab文件后，运行

`sudo mount -a`
验证配置是否正确，配置不正确可能会导致系统无法正常启动。

## 查找空目录并删除

`find -type d -empty | xargs -n 1 rm -fr`

## 查看磁盘

`lsblk`

## 查看共享内存

`ipcs -m`
