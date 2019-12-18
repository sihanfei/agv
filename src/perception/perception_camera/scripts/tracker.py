import cv2
import time
import math
import numpy as np
import tensorflow as tf
from sort import Sort 
from viz import Draw

class Tracker:
    def __init__(self, 
                 pb_path, 
                 cam_cfg_path,
                 score_threshold=0.5, 
                 per_process_gpu_memory_fraction=0.5):
        self.pb_file = pb_path
        self.score_thresold = score_threshold
        self._getCameraInfo(cam_cfg_path)

        self.detection_graph = self._load_model()
        self.detection_graph.as_default()

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=per_process_gpu_memory_fraction)
        self.sess = tf.Session(graph=self.detection_graph, config=tf.ConfigProto(gpu_options=gpu_options))
        
        self.tracker = Sort()

    def _getCameraInfo(self, camera_cfg_path):
        fs = cv2.FileStorage(camera_cfg_path, cv2.FileStorage_READ)
        self.focus = fs.getNode('focus').real()
        self.img_h = fs.getNode('height').real()
        self.img_w = fs.getNode('width').real()
        self.cameraMatrix = fs.getNode('cameraMatrix').mat()
        self.distCoeffs = fs.getNode('distCoeffs').mat()
        self.dist_each_pix_x = fs.getNode('dist_each_pix_x').real()
        self.dist_each_pix_y = fs.getNode('dist_each_pix_y').real()
        self.H = fs.getNode('H').mat()
        self.each_pix_in_sensor = fs.getNode('each_pix_in_sensor').real()
        self.fdx = self.cameraMatrix[0, 0]
        self.fdy = self.cameraMatrix[1, 1]

    def _load_model(self):
        detection_graph = tf.Graph()
        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(self.pb_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        return detection_graph
    
    def _getIPMPoint(self, point):
        dist_x = (self.H[0][0]*point[0]+self.H[0][1]*point[1]+self.H[0][2]) / (self.H[2][0]*point[0]+self.H[2][1]*point[1]+self.H[2][2])
        dist_y = (self.H[1][0]*point[0]+self.H[1][1]*point[1]+self.H[1][2]) / (self.H[2][0]*point[0]+self.H[2][1]*point[1]+self.H[2][2])
        
        return dist_x, dist_y

    def _getDistance(self, tracking_results):
        dist_arr = []

        for tracking_result in tracking_results:
            point_min = self._getIPMPoint([tracking_result[0], tracking_result[1]])
            point_max = self._getIPMPoint([tracking_result[2], tracking_result[3]])
            # distance_x = point_max[1] * self.dist_each_pix_y
            distance_x = (self.focus * 860) / (math.fabs(360-tracking_result[3]) / (self.fdy / self.focus))

            center = (tracking_result[0] + tracking_result[2]) / 2 
            center_dist = self.img_w / 2 - center
            distance_y = (distance_x / self.focus) * (center_dist / (self.fdx / self.focus))

            width = (distance_x / self.focus) * (math.fabs(tracking_result[0] - tracking_result[2]) / (self.fdx / self.focus))
            hight = (distance_x / self.focus) * (math.fabs(tracking_result[1] - tracking_result[3]) / (self.fdy / self.focus))
            
            dist_arr.append([distance_x/1000, distance_y/1000, width/1000, hight/1000])

        dist_arr = np.array(dist_arr, np.float32)

        tracking_results = np.concatenate((tracking_results, dist_arr), axis=1)

        return tracking_results
        
    def begins_tracking(self, img):
        """
        Method to be called to perform tracking
        """

        dets = None
        tracking_results = None
        
        undistort_frame = cv2.undistort(img, self.cameraMatrix, self.distCoeffs)
        # undistort_frame = img
        image_resize = cv2.resize(undistort_frame, (300, 300))
        image_np_expanded = np.expand_dims(image_resize, axis=0)

        # Actual detection.
        start_time = time.time()
        boxes, scores, classes = self.sess.run([self.boxes, self.scores, self.classes], feed_dict={self.image_tensor: image_np_expanded})
        
        mask = scores > self.score_thresold
        boxes = boxes[mask, :]
        scores = scores[mask]
        classes = classes[mask]
        
        if boxes is None:
            return tracking_results, undistort_frame
            # tracking_results = self.tracker.update(dets)
        else:
            for idx, i in enumerate(classes):
                class_label = i 
                xmin = 0 if boxes[idx, 1]*self.img_w < 0 else boxes[idx, 1]*self.img_w-1
                ymin = 0 if boxes[idx, 0]*self.img_h < 0 else boxes[idx, 0]*self.img_h-1
                xmax = self.img_w-1 if boxes[idx, 3]*self.img_w > self.img_w-1 else boxes[idx, 3]*self.img_w-1
                ymax = self.img_h-1 if boxes[idx, 2]*self.img_h > self.img_h-1 else boxes[idx, 2]*self.img_h-1
                score = scores[idx]
                if not isinstance(dets, np.ndarray):
                    dets = np.array([[xmin, ymin, xmax, ymax, score, class_label+1]])
                else:
                    dets = np.concatenate([dets, np.array([[xmin, ymin, xmax, ymax, score, class_label+1]])])

            # if dets is None:
            #     dets = np.array([[]])
            tracking_results = dets
            # tracking_results = self.tracker.update(dets)

        if tracking_results is not None and tracking_results.shape[0] > 0:
            tracking_results = self._getDistance(tracking_results)

        return tracking_results, undistort_frame

