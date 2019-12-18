Header header                                #按照约定取收到第一帧时的IPC时间
location_msgs/FusionDataInfo location_start  #本次点云第一帧定位信息
location_msgs/FusionDataInfo location_end    #本次点云最后帧定位信息
sensor_msgs/PointCloud2 agv_cloud_object      #被切的车体点云，点云中时间与header相同
sensor_msgs/PointCloud2 point_cloud_ground    #切车体后被切的地面点云，点云中时间与header相同
sensor_msgs/PointCloud2 point_cloud_object    #切车体和切地面后剩余的点云，点云中时间与header相同
