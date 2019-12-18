#!/usr/bin/env python
import os
import cv2
import sys
import time
import rospy
import roslib
roslib.load_manifest('perception_camera')
import argparse
from viz import Draw
from tracker import Tracker
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from perception_camera.msg import CameraObstacle

publish_topic_name = rospy.get_param("image_obstacle_publish_topic")
pb_path = rospy.get_param("pb_path")
cam_cfg_path = rospy.get_param("cam_cfg_path")
sub_image_topic = rospy.get_param("sub_image_topic")

bridge = CvBridge()
pub = rospy.Publisher(publish_topic_name, CameraObstacle, queue_size=10)
track = Tracker(
    pb_path = pb_path, 
    cam_cfg_path=cam_cfg_path, 
    score_threshold = 0.5)

def callBack(data):
    start_time = time.time()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    tracking_results, frame = track.begins_tracking(cv_image)

    camera_obstacle = CameraObstacle()
    if tracking_results is not None and tracking_results.shape[0] > 0:
        for tracking_result in tracking_results:
            camera_obstacle.xmin.append(tracking_result[0])
            camera_obstacle.ymin.append(tracking_result[1])
            camera_obstacle.xmax.append(tracking_result[2])
            camera_obstacle.ymax.append(tracking_result[3])
            camera_obstacle.confidence.append(tracking_result[4])
            camera_obstacle.object_class.append(int(tracking_result[5]))
            camera_obstacle.x.append(tracking_result[6])
            camera_obstacle.y.append(tracking_result[7])
            camera_obstacle.w.append(tracking_result[8])
            camera_obstacle.h.append(tracking_result[9])
    camera_obstacle.header.stamp = data.header.stamp
    pub.publish(camera_obstacle)
    prev_time=time.time()
    print("============================= camera python =================================")
    print(prev_time - start_time)
    print("============================= camera python =================================")

    fps = 1.0
    result_image = frame
    if tracking_results is not None and tracking_results.shape[0] > 0:
        drawer = Draw(frame, tracking_results, fps)
        result_image = drawer.bboxes()
    cv2.imshow("img",result_image)
    cv2.waitKey(1)

def main():
    
    rospy.init_node('perception_camera_image', anonymous=True)
    rospy.Subscriber(sub_image_topic, Image, callBack)
    rospy.spin()

    pass

if __name__ == '__main__':
    main()
    

        