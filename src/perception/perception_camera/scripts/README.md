# 目标物追踪（V1.1）

## Repo 结构：

    |
    |----README.md
    |
    |----sort.py: SORT tracker的具体定义
    |
    |----utils.py: 函数用于进行非极大值抑制
    |
    |----viz.py:类用于绘制追踪框
    |
    |----tracker.py:类用于定义tracking pipeline
    |
    |----yolo_trt.py:用于生成所需的Tensorflow frozen graph 文件
    |
    |----video_demo.py:用于生存demo视频
    |
    |------detection_models/
    |------|------yolov3/:包含tensorflow yolov3 checkpoint files和生成的frozen graph



## 如何使用
### Step I
首先需从钉盘的detection_models文件夹中下载yolov3 ckpt 文件,并将所有文件放入detection_models 文件夹中
### Step II
将ckpt文件转化为 frozen graph pb 文件，可直接使用如下命令在v_tracker上一级文件夹中运行

```bashrc
$ python -m python -m v_tracker.yolo_trt
```
在detection_models 文件夹中的trt_*.pb文件为tensorrt graph而detection_model.pb为 raw model graph

### Step III
使用上一步中生成的pb文件，对目标图像进行推理，可直接使用如下命令在v_tracker上一级文件夹中运行
```bashrc
$ python -m v_tracker.video_demo
```
具体的tracking 设置可参考tracker.py

