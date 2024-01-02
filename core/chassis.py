import rospy
from loguru import logger
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from typing import *

class Chassis:
    def __init__(self, topic_name:str = "/cmd_vel", odom_topic: Optional[str]="/odom", imu_topic: Optional[str]="/imu"):
        self.odom_data = None
        self.imu_data = None
        self.isready = False

        self.topic_name = topic_name
        self.odom_topic = odom_topic
        self.use_odom = True
        self.imu_topic = imu_topic
        self.use_imu = True

        logger.info("Initializing Chassis")
        self.publisher = rospy.Publisher(self.topic_name, Twist, queue_size=10)

        logger.info("Initializing Odometry...")
        if self.odom_topic:
            logger.debug("Subscribing to odometry topic...")
            self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
            try:
                rospy.wait_for_message(self.odom_topic, Odometry, timeout=3)
            except rospy.ROSException:
                logger.warning("Timeout occurred while waiting for odometry message.")
                self.use_odom = False
        else:
            self.use_odom = False

        if not self.use_odom:
            logger.warning(f"odom ({self.odom_topic}) is not available")

        logger.info("Initializing Imu...")
        if self.imu_topic:
            logger.debug("Subscribing to imu topic...")
            self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)
            try:
                rospy.wait_for_message(self.imu_topic, Imu, timeout=3)
            except rospy.ROSException:
                logger.warning("Timeout occurred while waiting for imu message.")
                self.use_imu = False
        else:
            self.use_imu = False

        if not self.use_imu:
            logger.warning(f"imu ({self.imu_topic}) is not available")

        self.isready = True
        logger.success("Chassis is ready!")

    def odom_callback(self, msg: Odometry):
        self.odom_data = msg

    def imu_callback(self, msg: Imu):
        self.imu_data = msg
