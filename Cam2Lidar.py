import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import LaserScan
import numba as nb

from core.camera import BaseRosCamera

NODE_NAME = "camera_node"


@nb.njit
def ray_casting(image, point, angle_range=(-180, 180), num_rays=360):
    height, width = image.shape[:2]
    rays = np.linspace(angle_range[0], angle_range[1], num_rays)

    result = []
    distance = []
    for angle in rays:
        theta = np.radians(angle)
        direction = np.array([np.cos(theta), np.sin(theta)])

        x, y = point
        step = 0
        flag = False
        while 0 <= x < width and 0 <= y < height:
            step += 1
            pixel = image[int(y), int(x)]
            if pixel != 0:
                result.append((int(x), int(y)))
                distance.append(step)
                flag = True
                break

            x += direction[0]
            y += direction[1]

        if not flag:
            result.append((int(x), int(y)))
            distance.append(900)

    return result, distance


def convert_lidar_to_laserscan(lidar_data, intersity):
    scan_msg = LaserScan()
    for i in range(len(lidar_data)):
        lidar_data[i] *= intersity
    # Set the necessary parameters
    scan_msg.header.stamp = rospy.Time.now()
    scan_msg.header.frame_id = 'camera_depth_frame'
    scan_msg.angle_min = 0.0
    scan_msg.angle_max = 2.0 * math.pi
    scan_msg.angle_increment = (2.0 * math.pi) / len(lidar_data)
    scan_msg.time_increment = 0.0
    scan_msg.range_min = 0.1
    scan_msg.range_max = 2.1875

    # Fill in the range data
    scan_msg.ranges = lidar_data
    scan_msg.intensities = [1.0] * len(lidar_data)

    return scan_msg


'''
def get_bg_noise(fovh, img, theta, height):
    near = img.shape[0]/(2*np.tan(fovh/2))
    th = np.arange(img.shape[0]/2, -img.shape[0]/2, -1) / near
    th = np.tan(np.arctan(th)+theta/2)
    bg_noise = np.zeros_like(img)
    bg_noise_col = height * (np.cos(theta/2) + np.sin(theta/2) * th)
    bg_noise[:, :] = bg_noise_col[:, np.newaxis]
    return bg_noise
'''


def get_bg_noise(fovh, img, theta, height):
    return np.loadtxt("depth.txt")


def main():
    rospy.init_node(NODE_NAME)
    rospy.loginfo(f"{NODE_NAME} started!")
    cam = BaseRosCamera("/camera/depth/image_raw", "passthrough")
    rospy.sleep(1)
    rospy.loginfo("Camera is ready!")

    pub = rospy.Publisher('/my_scan', LaserScan, queue_size=10)

    frame_cnt = 0
    t = time.time()
    print(np.max(cam.read()))
    bg_noise = np.zeros_like(cam.read())
    bg_noise = get_bg_noise(np.deg2rad(cam.fov[1]), bg_noise, np.deg2rad(25), 900)
    print(bg_noise)
    print(cam.read())

    max_diff = 25
    diff = None

    intersity = 105 / 480 / 100
    while not rospy.is_shutdown():
        frame_cnt += 1
        rospy.Rate(10).sleep()

        diff = np.zeros_like(cam.read())
        diff = diff.astype(np.uint8)
        diff[np.where(np.abs(cam.read() - bg_noise) > max_diff)] = 255

        diff[:, :10] = 0
        diff = cv2.medianBlur(diff, 7)

        rgb = cv2.cvtColor(diff, cv2.COLOR_GRAY2RGB)
        x, y = 320, 479

        result, distance = ray_casting(diff, (x, y))
        for i in range(len(result) - 1):
            x2, y2 = result[i]
            if distance[i] < 900:
                cv2.line(rgb, (x, y), (x2, y2), (0, 0, 255), 1)
            else:
                cv2.line(rgb, (x, y), (x2, y2), (255, 0, 0), 1)

        scan_msg = convert_lidar_to_laserscan(distance, intersity)
        pub.publish(scan_msg)

        cv2.imshow("image", rgb)
        cv2.imshow("diff", diff)
        kc = cv2.waitKey(1)

        if kc == 27:
            # np.savetxt("diff.txt", diff.astype(int), fmt="%d")
            break

        if frame_cnt == 50:
            rospy.loginfo(f"{frame_cnt} frames in {t - time.time():.3f}, FPS: {frame_cnt / (t - time.time()):.3f}")
            frame_cnt = 0
            t = time.time()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
