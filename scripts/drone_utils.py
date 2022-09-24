#!/usr/bin/env python

# third-party imports
import math
from math import sqrt
import numpy as np
import rospy
from mavros_msgs.srv import (
    SetMode,
    SetModeRequest,
    CommandBool,
    CommandBoolRequest,
    CommandTOL,
    CommandTOLRequest,
)
from mavros_msgs.msg import PositionTarget
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Quaternion, Pose, Twist
from geographic_msgs.msg import GeoPoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelStates
from scipy.spatial.transform import Rotation as R


def eular_quad(roll, pitch, yaw):
    r = R.from_euler('xyz', [roll, pitch, math.radians(yaw)])
    x,y,z,w = r.as_quat()
    return x,y,z,w

def quad_eular(x, y, z, w):
    r = R.from_quat([x ,y,z ,w])
    curr_roll ,curr_pitch ,curr_yaw=r.as_euler('xyz', degrees=True)
    return curr_roll, curr_pitch, curr_yaw



def transform_points(pt):
    # world 3 specific
    ## transpose of roll pitch yaw of iris pose
    # rot = np.array([
    #     [0.0078247, 0.9999477, 0.00658],
    #     [-0.999, 0.0081, -0.0439],
    #     [-0.0439, -0.00623, 0.999]
    # ])
    rot = np.array([
        [0.0078247, -0.999, -0.0439],
        [0.9999477, 0.0081, -0.00623],
        [0.00658, -0.0439, 0.999]
    ])
    # get from iris pose
    npt = np.matmul(rot, (pt - np.array([108.857122, -265.657738, 49.057766])))
    return npt



class Drone:
    def __init__(self):
        rospy.loginfo("Initializing Drone class")
        self.global_position = None
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.feedback_global_position)

        self.transformation_matrix = np.array([[0, -1, 0], [1,0,0], [0,0,1]])

        self.pose = Pose()
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.feedback_local_position)

        self.drone_pose = Pose()
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.feedback_get_gazebo_pose)

        self.w = Point()
        self.publish_vel = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        rospy.Subscriber('/mavros/imu/data', Imu, self.get_angular_velocity)

        self.local_position_publisher = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
        self.global_position_publisher = rospy.Publisher('/mavros/setpoint_position/global', GeoPoseStamped, queue_size=10)
        self.local_position_velocity_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    def feedback_get_gazebo_pose(self,data):
        for i in range (len(data.name)):
            if data.name[i] == 'iris':
                idx = i
        # print ("index of drone in model states is ", idx)
        self.drone_pose.position.x = data.pose[idx].position.x
        self.drone_pose.position.y = data.pose[idx].position.y
        self.drone_pose.position.z = data.pose[idx].position.z
        self.drone_pose.orientation.x = data.pose[idx].orientation.x
        self.drone_pose.orientation.y = data.pose[idx].orientation.y
        self.drone_pose.orientation.z = data.pose[idx].orientation.z
        self.drone_pose.orientation.w = data.pose[idx].orientation.w
        self.v_x = data.twist[idx].linear.x
        self.v_y = data.twist[idx].linear.y
        self.v_z = data.twist[idx].linear.z
        self.velocity = math.sqrt(self.v_x**2 + self.v_y**2 + self.v_z**2)
        self.yaw = self.get_yaw()

    def normalize(self,angle):
        if angle > math.pi:
            angle -= 2*math.pi
        if angle < -math.pi:
            angle += 2*math.pi
        return angle

    def get_yaw(self):
        x,y,z,w = self.drone_pose.orientation.x , self.drone_pose.orientation.y, self.drone_pose.orientation.z, self.drone_pose.orientation.w
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return self.normalize(yaw_z)

    def getvelBody(self, u, v, w, gamma):
        rate = rospy.Rate(10)
        msg = Twist()
        msg.linear.x = u
        msg.linear.y = v
        msg.linear.z = w
        msg.angular.z = gamma
        msg.angular.x = self.w.x
        msg.angular.y = self.w.y
        self.publish_vel.publish(msg)
        rate.sleep()

    def get_angular_velocity(self, data):
        self.w.x = data.angular_velocity.x
        self.w.y = data.angular_velocity.y
        self.w.z = data.angular_velocity.z


    def feedback_local_position(self, data):
        self.pose.position.x = data.pose.position.x
        self.pose.position.y = data.pose.position.y
        self.pose.position.z = data.pose.position.z
        self.pose.orientation.x = data.pose.orientation.x
        self.pose.orientation.y = data.pose.orientation.y
        self.pose.orientation.z = data.pose.orientation.z
        self.pose.orientation.w = data.pose.orientation.w
        self.curr_yaw = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])[2]

    def feedback_global_position(self, msg):
        self.global_position = msg

    ###### 5.22 AM, added Saad's Rotation fix. By Harshil ##############################

    def corrected_pose(self, current_pos):
        current_pos = np.array(current_pos)
        new_pos = np.matmul(self.transformation_matrix,current_pos)
        return list(new_pos)

    # def rotate(self, point):
    #     angle = self.yaw - math.pi/2
    #     px, py = point[0], point[1]
    #     nx =  math.cos(angle) * px  - math.sin(angle) * py
    #     ny =  math.sin(angle) * px  + math.cos(angle) * py
    #     return [nx, ny, point[2]]

    ###################################################################################3

    def arm(self):
        rospy.loginfo("Arming the drone")
        call_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        data = CommandBoolRequest()
        data.value = True
        return call_srv(data)

    def takeoff(self, latitude, longitude, altitude):
        rospy.loginfo("Initiating takeoff")
        call_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        data = CommandTOLRequest()
        data.latitude = latitude
        data.longitude = longitude
        data.altitude = altitude
        return call_srv(data)

    def land(self, latitude, longitude, altitude):
        rospy.loginfo("Initiating landing")
        call_srv = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        data = CommandTOLRequest()
        data.latitude = latitude
        data.longitude = longitude
        data.altitude = altitude
        return call_srv(data)

    def mavros_set_mode(self, mode):
        rospy.loginfo("Switching to mode: " + mode)
        call_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        data = SetModeRequest()
        data.custom_mode = mode
        return call_srv(data)

    def switch_to_guided_mode(self):
        mode = "GUIDED"
        return self.mavros_set_mode(mode)

    def switch_to_stabilize_mode(self):
        mode = "STABILIZE"
        return self.mavros_set_mode(mode)

    def wait_till_local_position_pose_reached(self, position, publish=True):
        rospy.loginfo("Moving to setpoint using wait_till_local_position_pose_reached " + str(position))
        rate = rospy.Rate(20)
        sp = PoseStamped()
        sp.pose.position.x = position[0]
        sp.pose.position.y = position[1]
        sp.pose.position.z = position[2]
        dist = sqrt(((self.pose.position.x-position[0])**2) + ((self.pose.position.y-position[1])**2) + ((self.pose.position.z-position[2])**2))
        while dist > 1:
            # rospy.loginfo("Currently at setpoint (" + str(self.pose.position.x) + ", " + str(self.pose.position.y) + ", " + str(self.pose.position.z) + ")")
            # rospy.loginfo("Dist: " + str(dist))
            if publish:
                self.local_position_publisher.publish(sp)
            dist = sqrt(((self.pose.position.x-position[0])**2) + ((self.pose.position.y-position[1])**2) + ((self.pose.position.z-position[2])**2))
            rate.sleep()
        rospy.loginfo("Reached setpoint " + str(position))

    def wait_till_local_position_via_position_velocity(self, position, velocity_magnitude, yaw, publish=True):
        rospy.loginfo("Moving to setpoint using wait_till_local_position_via_position_velocity with position " + str(position) + " and velocity " + str(velocity_magnitude))
        rospy.loginfo(yaw)
        FRAME_LOCAL_NED = 1
        FRAME_LOCAL_OFFSET_NED = 7
        FRAME_BODY_NED = 8
        FRAME_BODY_OFFSET_NED = 9
        rate = rospy.Rate(20)
        sp = PositionTarget()
        sp.coordinate_frame = FRAME_LOCAL_NED
        # sp.type_mask = 3520  # pos+vel
        # sp.type_mask = 3527  # vel
        sp.type_mask = 1991
        # sp.yaw = yaw
        # sp.position.x = position[0]
        # sp.position.y = position[1]
        # sp.position.z = position[2]
        x = position[0] - self.pose.position.x
        y = position[1] - self.pose.position.y
        z = position[2] - self.pose.position.z
        normalization_factor = sqrt(x**2 + y**2 + z**2)
        sp.velocity.x = x * velocity_magnitude / normalization_factor
        sp.velocity.y = y * velocity_magnitude / normalization_factor
        sp.velocity.z = z * velocity_magnitude / normalization_factor
        dist = sqrt(((self.pose.position.x-position[0])**2) + ((self.pose.position.y-position[1])**2) + ((self.pose.position.z-position[2])**2))
        while dist > 2:
            if publish:
                self.local_position_velocity_publisher.publish(sp)
            dist = sqrt(((self.pose.position.x-position[0])**2) + ((self.pose.position.y-position[1])**2) + ((self.pose.position.z-position[2])**2))
            rate.sleep()
        rospy.loginfo("Reached setpoint with position " + str(position) + " and velocity " + str(velocity_magnitude))

    def gotopose_yaw_1(self,x,y,z,yaw):
        # rate = rospy.Rate(20)
        sp = PoseStamped()
        orien = eular_quad(0,0,yaw)
        sp.pose.position.x = x
        sp.pose.position.y = y
        sp.pose.position.z = z
        sp.pose.orientation.x = orien[0]
        sp.pose.orientation.y = orien[1]
        sp.pose.orientation.z = orien[2]
        sp.pose.orientation.w = orien[3]
        self.local_position_publisher.publish(sp)

