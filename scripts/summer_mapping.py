#!/usr/bin/env python3

# standard imports
import sys
import time

# third-party imports
import rospy
import numpy as np

# app imports
from drone_utils import Drone
from mapping import Mapping


class SummerMapping:
    def __init__(self):
        rospy.init_node("summer_mapping")
        rospy.set_param('WPNAV_SPEED', 10)
        self.iris = Drone()
        self.state = None
        self.flight_height = 19

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

    def map_road(self):
        self.takeoff_iris()
        map = Mapping()
        while True:
            map.main()
            map.return_stuff[0] =  np.nan_to_num(map.return_stuff[0], nan = 0)
            self.iris.gotopose_yaw_1(map.return_stuff[0][0], map.return_stuff[0][1] ,map.return_stuff[0][2] + self.flight_height, self.iris.curr_yaw + map.road.wted_yaw)



if __name__ == "__main__":
    node = SummerMapping()
    node.map_road()

