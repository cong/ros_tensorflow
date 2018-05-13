#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import tensorflow as tf
import os
import re


class RosTensorFlow():
    def __init__(self):
       
        self._session = tf.Session()
        self._cv_bridge = CvBridge()

        self._sub = rospy.Subscriber('usb_cam/image_raw', Image, self.callback, queue_size=1)
        self._pub = rospy.Publisher('/result_ripe', String, queue_size=1)
        self.score_threshold = rospy.get_param('~score_threshold', 0.1)
        self.use_top_k = rospy.get_param('~use_top_k', 5)

    def load(self, label_lookup_path, uid_lookup_path):

        if not tf.gfile.Exists(uid_lookup_path):
          tf.logging.fatal('File does not exist %s', uid_lookup_path)
        if not tf.gfile.Exists(label_lookup_path):
          tf.logging.fatal('File does not exist %s', label_lookup_path)

        # Loads mapping from string UID to human-readable string
        proto_as_ascii_lines = tf.gfile.GFile(uid_lookup_path).readlines()
        uid_to_human = {}
        p = re.compile(r'[n\d]*[ \S,]*')
        for line in proto_as_ascii_lines:
            parsed_items = p.findall(line)
            uid = parsed_items[0]
            human_string = parsed_items[2]
            uid_to_human[uid] = human_string

        # Loads mapping from string UID to integer node ID.
        node_id_to_uid = {}
        proto_as_ascii = tf.gfile.GFile(label_lookup_path).readlines()
        for line in proto_as_ascii:
            if line.startswith('  target_class:'):
                target_class = int(line.split(': ')[1])
            if line.startswith('  target_class_string:'):
                target_class_string = line.split(': ')[1]
                node_id_to_uid[target_class] = target_class_string[1:-2]

        # Loads the final mapping of integer node ID to human-readable string
        node_id_to_name = {}
        for key, val in node_id_to_uid.items():
            if val not in uid_to_human:
                tf.logging.fatal('Failed to locate: %s', val)
            name = uid_to_human[val]
            node_id_to_name[key] = name

        return node_id_to_name
    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image_data = cv2.imencode('.jpg', cv_image)[1].tostring()

        # Creates graph from saved GraphDef.
        softmax_tensor = self._session.graph.get_tensor_by_name('softmax:0')
        predictions = self._session.run(
            softmax_tensor, {'DecodeJpeg/contents:0': image_data})
        predictions = np.squeeze(predictions)

        # Creates node ID --> English string lookup.
        node_lookup = self.load(PATH_TO_LABELS, PATH_TO_UID)
        top_k = predictions.argsort()[-self.use_top_k:][::-1]
        for node_id in top_k:
            
            if node_id not in node_lookup:
                human_string = ''
            else:
                human_string = node_lookup[node_id]

            score = predictions[node_id]
            if score > self.score_threshold:
                rospy.loginfo('%s (score = %.5f)' % (human_string, score))
                self._pub.publish(human_string)

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    
    ROOT_PATH = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
    PATH_TO_CKPT = ROOT_PATH + '/include/classifier/classify_image_graph_def.pb'
    PATH_TO_LABELS = ROOT_PATH + '/include/classifier/imagenet_2012_challenge_label_map_proto.pbtxt'
    PATH_TO_UID = ROOT_PATH + '/include/classifier/imagenet_synset_to_human_label_map.txt'

    with tf.gfile.FastGFile(PATH_TO_CKPT, 'rb') as f:
    	graph_def = tf.GraphDef()
    	graph_def.ParseFromString(f.read())
    	_ = tf.import_graph_def(graph_def, name='')    
    
    
    rospy.init_node('ros_tensorflow_classify')
    tensor = RosTensorFlow()
    tensor.main()
