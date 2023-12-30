import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class BaseRosCamera:
    def __init__(self, image_topic="/camera/rgb/image_raw", encoding="bgr8"):
        '''
        :param image_topic: string, image topic such as /camera/rgb/image_raw
        :param encoding: string, image encoding bgr8 for rgb, passthrough for depth
        '''
        self.frame: np.array = None
        self.frame_cnt = 0
        self.encoding = encoding
        self.image_topic = image_topic
        self.subscriber = rospy.Subscriber(image_topic, Image, self.callback)
        self.bridge = CvBridge()

        rospy.loginfo("[@camera.py] Initializing camera...")

        image = rospy.wait_for_message(image_topic, Image, timeout=5)
        self.callback(image)

        rospy.logdebug("[@camera.py] Topic: " + image_topic)
        rospy.logdebug("[@camera.py] Encoding: " + encoding)
        rospy.logdebug("[@camera.py] Image Data: " + str(self.frame.shape))

    def callback(self, data):
        self.frame = self.bridge.imgmsg_to_cv2(data, desired_encoding=self.encoding)
        self.frame_cnt += 1

    def read(self):
        return self.frame

    def safe_read(self):
        return self.frame.copy()
