#!/usr/bin/env python3

import numpy as np
# from cv_bridge import CvBridge
import cv2
# from numpy import angle
# from tool.darknet2pytorch import Darknet
# from tool.torch_utils import *
import rospy
from sensor_msgs.msg import Image, PointCloud2
from rospy.numpy_msg import numpy_msg
from geometry_msgs.msg import Point, PoseStamped
from tf.transformations import euler_from_quaternion
import tf
import ros_numpy
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PointStamped
from solution.msg import car_msg
import time
import math
from math import atan2

def translateRotation(rotation, width, height):
    if (width < height):
        rotation = -1 * (rotation - 90)
    if (rotation > 90):
        rotation = -1 * (rotation - 180)
    rotation *= -1
    return round(rotation)

class angle_detection():

    def __init__(self):


        # Position position and velocity variabless

        self.vel = Point()
        self.base_vel = Point()

        self.ugv_global_coordinate = Point()
        self.ugv_base_coordinate = Point()

        # Defining subscribers and publishers

        self.get_depth_image = rospy.Subscriber('/depth_camera/rgb/image_raw', Image, self.get_depth)
        rospy.Subscriber("/depth_camera/depth/points", PointCloud2, self.callback)
        rospy.Subscriber("/clock",Clock,self.time_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.get_drone_yaw)
        self.car_info_publisher = rospy.Publisher('/car_feedback', car_msg, queue_size = 10)

        self.rate = rospy.Rate(20)
        rospy.loginfo('INIT')
        # Initializing YOLO Model and running transforms

        self.listener = tf.TransformListener()
        self.listener.waitForTransform("/map", "/tilt_link", rospy.Time(), rospy.Duration(8.0))

        # Variables to store old setpoints
        self.old_x = 0
        self.old_y = 0
        self.old_z = 0

        self.old_base_x = 0
        self.old_base_y = 0
        self.old_base_z = 0
        time.sleep(5)


    def get_drone_yaw(self, data):
        q = []
        q.append(data.pose.orientation.x)
        q.append(data.pose.orientation.y)
        q.append(data.pose.orientation.z)
        q.append(data.pose.orientation.w)

        rpy = euler_from_quaternion(q)
        self.drone_yaw = rpy[2]*180 / (math.pi)
        #print(self.drone_yaw)


    def highpass(img, sigma):
        return img - cv2.GaussianBlur(img, (0,0), sigma) + 127

    def depth_data_to_edge_pass(self):
        # img = img-np.min(img)
        # img = img/np.max(img)
        # img = 255*img
        # img = np.floor(img)

        aperture_size = 5
        img = self.img_depth
        img = np.uint8(img)
        edges = cv2.Canny(img, 1, 150, apertureSize=aperture_size)

        img = angle_detection.highpass(edges, 3)
        img = img.copy()

        gray = img.copy()

        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                cv2.THRESH_BINARY_INV, 51, 2)

        cntrs, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        if np.size(cntrs):
            cv2.drawContours(img, cntrs, -1, (0, 0, 255), thickness=3)
            cntrs = sorted(cntrs, key=lambda x: cv2.contourArea(x), reverse=True)
            rotrect = cv2.minAreaRect(cntrs[0])

            box = cv2.boxPoints(rotrect)
            box = np.int0(box)
            result = img.copy()

            cv2.drawContours(result,[box],0,(0,0,255),2)
            cX = (box[0][0]+box[2][0])/2
            cY = (box[0][1]+box[2][1])/2
            cX = math.floor(cX) #centre dimensions
            cY = math.floor(cY)
            # cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1) #centre


            # angle = rotrect[-1]
            # width = rotrect[1][0]
            # height = rotrect[1][1]
            # angle = translateRotation(angle, width, height)

            # angle = atan2((-box[0][1] + box[3][1]) , box[0][0] - box[3][0])
            # angle = 90 - angle
            # d = 100
            # cX2 = cX - d*math.cos(angle)
            # cY2 = cY - d*math.sin(angle)
            # cX2 = math.floor(cX2)
            # cY2 = math.floor(cY2)
            # cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1) #centre of the car displayed as white dot
            # cv2.arrowedLine(img, (cX, cY), (cX2, cY2), (255, 0, 0), 3)

            d = 100
            # cX2 = cX - d*math.cos(angle)
            # cY2 = cY - d*math.sin(angle)
            # cX2 = math.floor(cX2)
            # cY2 = math.floor(cY2)

            point0 = np.array(box[0])
            point1 = np.array(box[1])
            point3 = np.array(box[3])

            if(np.linalg.norm(point0-point1)>np.linalg.norm(point0-point3)):
                cX2 = (box[1][0]+box[2][0])/2
                cY2 = (box[1][1]+box[2][1])/2
            else:
                cX2 = (box[2][0]+box[3][0])/2
                cY2 = (box[2][1]+box[3][1])/2

            cX2 = math.floor(cX2)
            cY2 = math.floor(cY2)
            cv2.circle(img, (cX, cY), 7, (255, 0, 0), -1) #centre of the car displayed as white dot
            cv2.arrowedLine(img, (cX, cY), (cX2, cY2), (255, 0, 0), 3)
            # cv2_imshow(result)
            angle = math.atan2(cY2-cY, cX2-cX)
            angle = angle*180/math.pi
            if(angle<-90):
                angle = -(angle+90)
            else:
                angle = -(angle+90)
            # print(angle)

            # cv2.imshow("Result", img)
            # cv2.waitKey(1)
                        # Update drone here
            # angle += self.drone_yaw
            # start_time = self.time_now


            self.localcoordinate = self.get_xyz(int(cY),int(cX))
            self.base_coordinate = self.get_base(self.localcoordinate[0], self.localcoordinate[1], self.localcoordinate[2])
            self.globalcoordinate = self.get_global(self.localcoordinate[0], self.localcoordinate[1], self.localcoordinate[2])

            # end_time = self.time_now

            # self.dt = end_time - start_time
            self.dt = 1/20

            self.ugv_global_coordinate.x = self.globalcoordinate[0]
            self.ugv_global_coordinate.y = self.globalcoordinate[1]
            self.ugv_global_coordinate.z = self.globalcoordinate[2]

            self.ugv_base_coordinate.x = self.base_coordinate[0]
            self.ugv_base_coordinate.y = self.base_coordinate[1]
            self.ugv_base_coordinate.z = self.base_coordinate[2]

            self.vel.x = (self.ugv_global_coordinate.x-self.old_x)/((self.dt))
            self.vel.y = (self.ugv_global_coordinate.y-self.old_y)/((self.dt))
            self.vel.z = (self.ugv_global_coordinate.z-self.old_z)/((self.dt))

            self.base_vel.x = (self.ugv_base_coordinate.x-self.old_base_x)/((self.dt))
            self.base_vel.y = (self.ugv_base_coordinate.y-self.old_base_y)/((self.dt))
            self.base_vel.z = (self.ugv_base_coordinate.z-self.old_base_z)/((self.dt))

            self.old_x = self.ugv_global_coordinate.x
            self.old_y = self.ugv_global_coordinate.y
            self.old_z = self.ugv_global_coordinate.z

            self.old_base_x = self.ugv_base_coordinate.x
            self.old_base_y = self.ugv_base_coordinate.y
            self.old_base_z = self.ugv_base_coordinate.z

            return angle
        else:
            print("No car")
            return None

    def time_callback(self,val):
        self.time_now = val.clock.nsecs/1000000000



    def get_depth(self, depth_data):

        # img_depth = np.frombuffer(depth_data.data, dtype=np.float32).reshape(depth_data.height, depth_data.width, -1)
        # img = np.array(img_depth).copy()
        # img = np.nan_to_num(img, nan = -100)
        # img = np.reshape(img, (img.shape[0], img.shape[1]))

        # bridge = CvBridge()
        #cv_image = bridge.imgmsg_to_cv2(depth_data, "bgr8")
        self.img_rgb = np.frombuffer(depth_data.data, dtype=np.uint8).reshape(depth_data.height, depth_data.width, -1)[:,:,0:3]
        self.img_bgr = self.img_rgb[...,::-1]
        #print(img)
        self.img_depth = self.img_bgr.copy()
        # np.save("array", self.img_depth)
        # cv2.imshow("Depth data",self.img_depth)
        # cv2.waitKey(1)
        # self.rate.sleep()

    def callback(self, value):
        self.pc_arr = ros_numpy.numpify(value)


    def get_xyz(self, w, h):
        x = self.pc_arr['x'][w, h]
        y = self.pc_arr['y'][w, h]
        z = self.pc_arr['z'][w, h]

        return np.array([x, y, z])

    def get_global(self, x, y, z):
        ps = PointStamped()
        ps.header.frame_id = "tilt_link"
        ps.header.stamp = rospy.Time(0)
        ps.point.x = x
        ps.point.y = y
        ps.point.z = z
        mat = self.listener.transformPoint("/map", ps)

        return [mat.point.x, mat.point.y, mat.point.z]


    def get_base(self, x, y, z):
        ps = PointStamped()
        ps.header.frame_id = "tilt_link"
        ps.header.stamp = rospy.Time(0)
        ps.point.x = x
        ps.point.y = y
        ps.point.z = z
        mat = self.listener.transformPoint("/base_link", ps)

        return [mat.point.x, mat.point.y, mat.point.z]




# if __name__ == "__main__":
#     rospy.init_node('iris_drone', anonymous = True)
#     time.sleep(2)
#     detector = angle_detection()
#     rate = rospy.Rate(20)

#     while True:
#         angle = detector.depth_data_to_edge_pass()
#         detector.car_info_publisher.publish(angle, detector.globalcoordinate[0], detector.globalcoordinate[1], detector.globalcoordinate[2], detector.vel.x, detector.vel.y, detector.vel.z)
#         rate.sleep()
