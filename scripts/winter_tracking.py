#!/usr/bin/env python3

# standard imports
import sys
import time

# third-party imports
import rospy
import numpy as np
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Pose, Point, Quaternion

# app imports
from drone_utils import Drone
from mapping import Mapping
from data_processing import process_data
from final_contour import angle_detection
from prius_controller import CONTROLLER, load


class WinterTracking:
    def __init__(self):
        rospy.init_node("winter_tracking")
        rospy.set_param('WPNAV_SPEED', 10)
        self.iris = Drone()
        self.car_pose_local = Pose()
        self.car_vel = Point()
        self.state = None
        self.flight_height = 19
        self.pub2 = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size = 10)

    def takeoff_iris(self):
        gp = self.iris.global_position
        while gp == None:
            if rospy.is_shutdown():
                sys.exit(1)
            time.sleep(1)
            rospy.loginfo("Waiting for global position to be updated...")
            gp = self.iris.global_position
        rospy.loginfo("Got new global position of the drone: ")
        rospy.loginfo(gp)
        srv_call = self.iris.switch_to_guided_mode()
        if not srv_call.mode_sent:
            rospy.logerr("Unable to switch to GUIDED mode")
            sys.exit(1)
        srv_call = self.iris.arm()
        if not srv_call.success:
            rospy.logerr("Unable to ARM drone")
            sys.exit(1)
        srv_call = self.iris.takeoff(gp.latitude, gp.longitude, self.flight_height)
        if not srv_call.success:
            rospy.logerr("Unable to Takeoff drone")
            sys.exit(1)
        self.iris.wait_till_local_position_pose_reached([0, 0, self.flight_height], publish=False)

    def getvelLocal(self, u, v, w, alpha):
        rospy.loginfo("getvelLocal published")
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.coordinate_frame = 8
        msg.type_mask = 448
        msg.velocity.x = 0
        msg.velocity.y = 0
        msg.velocity.z = 0

        msg.position.x = u
        msg.position.y = v
        msg.position.z = w

        msg.yaw_rate = 0.0
        # msg.acceleration_or_force.x = u
        # msg.acceleration_or_force.y = v
        # msg.acceleration_or_force.z = w
        msg.yaw = alpha
        # r = rospy.Rate(20) # 10hz
        self.pub2.publish(msg)


    def map_road(self):
        self.takeoff_iris()
        map = Mapping()
        x_way_points, y_way_points = load('final.npy')
        prius = CONTROLLER(x_way_points, y_way_points)
        detector = angle_detection()
        self.angle = detector.depth_data_to_edge_pass()
        time.sleep(2)
        rate = rospy.Rate(20)
        prius_not_started = True

        while True:
            self.angle = detector.depth_data_to_edge_pass()
            # detector.car_info_publisher.publish(self.angle, detector.globalcoordinate[0], detector.globalcoordinate[1], detector.globalcoordinate[2], detector.vel.x, detector.vel.y, detector.vel.z)
            self.car_pose_local.position.x = detector.globalcoordinate[0]
            self.car_pose_local.position.y = detector.globalcoordinate[1]
            self.car_pose_local.position.z = detector.globalcoordinate[2]
            self.car_vel.x = detector.vel.x
            self.car_vel.y = detector.vel.y
            self.car_vel.z = detector.vel.z

            print("car")
            print(self.car_pose_local.position.x, self.car_pose_local.position.y, self.car_pose_local.position.z)
            print("drone")
            # print(self.car_pose_local)
            # print(detector.vel.x, detector.vel.y, detector.vel.z)
            # print(detector.base_coordinate)
            # print(self.angle)

            error_pos = np.array([ detector.ugv_base_coordinate.x, detector.ugv_base_coordinate.y, detector.ugv_base_coordinate.z + self.flight_height ])
            error_yaw = self.angle
            print(f"yaw: {error_yaw}")
            # error_yaw = 0

            self.getvelLocal(error_pos[0], error_pos[1], error_pos[2], error_yaw)
            prius.load_pose(self.car_pose_local, self.car_vel, error_yaw-90)
            if prius_not_started:
                prius.start()
                prius_not_started = False

            rate.sleep()



if __name__ == "__main__":
    node = WinterTracking()
    node.map_road()

