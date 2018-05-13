#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import sys
from multiprocessing import Queue, Pool
import tensorflow as tf
#from utils.app_utils import FPS
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util


class RosTensorFlow():
    def __init__(self):
        self._cv_bridge = CvBridge()
        self._sub = rospy.Subscriber('usb_cam/image_raw', Image, self.callback, queue_size=1, buff_size=2**24)
        print('Image converter constructor ')
        self._pub = rospy.Publisher("/result_ripe", Image, queue_size=0)
        #self._pub = rospy.Publisher('result', Int16, queue_size=1)

    def detect_objects(image_np, sess, detection_graph):
        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)
        image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = detection_graph.get_tensor_by_name('detection_scores:0')
        classes = detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = detection_graph.get_tensor_by_name('num_detections:0')

        # Actual detection.
        (boxes, scores, classes, num_detections) = sess.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

        # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            np.squeeze(boxes),
            np.squeeze(classes).astype(np.int32),
            np.squeeze(scores),
            category_index,
            use_normalized_coordinates=True,
            line_thickness=8)
        return image_np
    
    def callback(self, image_msg):
            
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        with detection_graph.as_default():
            with tf.Session(graph=detection_graph) as sess:
                image_np = cv_image
                # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
                image_np_expanded = np.expand_dims(image_np, axis=0)
                image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
                # Each box represents a part of the image where a particular object was detected.
                boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
                # Each score represent how level of confidence for each of the objects.
                # Score is shown on the result image, together with the class label.
                scores = detection_graph.get_tensor_by_name('detection_scores:0')
                classes = detection_graph.get_tensor_by_name('detection_classes:0')
                num_detections = detection_graph.get_tensor_by_name('num_detections:0')
                # Actual detection.
                (boxes, scores, classes, num_detections) = sess.run(
                    [boxes, scores, classes, num_detections],
                    feed_dict={image_tensor: image_np_expanded})
                # Visualization of the results of a detection.
                vis_util.visualize_boxes_and_labels_on_image_array(
                    image_np,
                    np.squeeze(boxes),
                    np.squeeze(classes).astype(np.int32),
                    np.squeeze(scores),
                    category_index,
                    use_normalized_coordinates=True,
                    line_thickness=8)
        
        try:
            self._pub.publish(self._cv_bridge.cv2_to_imgmsg(image_np, "bgr8"))
        except CvBridgeError as e:
            print(e)    
    
    
    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ros_tensorflow_detect')

    #ROOT_PATH = os.path.abspath(os.path.join(sys.path[0], os.pardir))
    ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))

    # Path to frozen detection graph. This is the actual model that is used for the object detection.
    #MODEL_NAME = 'ssd_mobilenet_v1_coco_2017_11_17'
    PATH_TO_CKPT = ROOT_PATH + '/include/object_detector/frozen_inference_graph.pb'

    # List of the strings that is used to add correct label for each box.
    PATH_TO_LABELS = ROOT_PATH + '/include/object_detector/mscoco_label_map.pbtxt'

    NUM_CLASSES = 1

    detection_graph = tf.Graph()
    with detection_graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='')

    # Loading label map
    label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
    categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
    category_index = label_map_util.create_category_index(categories)

    tensor = RosTensorFlow()
    tensor.main()


