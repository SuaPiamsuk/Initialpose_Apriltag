#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, String

from initial_pose_apriltag.srv import auto_initial_pose
from apriltag_ros.msg import AprilTagDetectionArray

import rospkg
import yaml
import json
import requests
import math
import numpy as np

from enum import Enum

class MainState(Enum):
    INITIAL  = 0
    WAITTING = 1
    RUNNING  = 2
    SUCCESS  = 3
    FAILURE  = 4

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class InitialPosePublisher:
    def __init__(self):
        rospy.init_node('initial_pose_publisher2', anonymous=True)
    
        # Subscriber 
        self.status_sub = rospy.Subscriber('/status', String, self.status_callback)
        self.status = None
        self.current_time = None
   

        self.initial_pose_finished_sub = rospy.Subscriber('/initial_pose_finished', Bool, self.initial_pose_status_callback)
        self.initial_pose_finished = False
        
        # service
        self.service_initial_pose_apriltag = rospy.ServiceProxy('initial_pose_apriltag_srv', auto_initial_pose)

        # Timer
        self.duration_time = 0.3
        self.timer = rospy.Timer(rospy.Duration(self.duration_time), self.initial_pose_callback)

        self.MainState = MainState.INITIAL
        self.count = 0

    def status_callback(self, msg):
        self.status = msg.data
    
    def initial_pose_status_callback(self,msg):
        self.initial_pose_finished = msg.data
      
                
    def initial_pose_srv(self, msg):
        print("hello world")
        self.srv_called = True

        # reset
        self.amcl_pose_received = False
        self.cal_tranform_finished = False
        self.already_get_info = False

        return "send srv succeed"

    def initial_pose_callback(self, timer):

        # IF USE STATE
        if self.MainState == MainState.INITIAL:
            print(self.MainState)
            self.current_time = rospy.Time.now()
            self.MainState = MainState.WAITTING

        elif self.MainState == MainState.WAITTING:
            print(self.MainState)
            try:
                if (rospy.Time.now() - self.current_time).to_sec() >= 5: # if server not response
                    self.MainState = MainState.FAILURE
                    print("server not response")

                self.service_initial_pose_apriltag()
            except:
                pass
            else:
                self.MainState = MainState.RUNNING

        elif self.MainState == MainState.RUNNING:
            print(self.MainState)
            print(self.count)
            self.count = self.count + 1
            
            # if self.initial_pose_finished == True:
            #     self.MainState = MainState.SUCCESS

            if (rospy.Time.now() - self.current_time).to_sec() >= 5: # if server not response
                    self.MainState = MainState.FAILURE
                    print("server not response")

            if self.status == 'SUCCESS':
                print("finished")
                self.MainState = MainState.SUCCESS
                print(self.MainState)

            elif self.status == 'FAILURE':
                print("tag_not_found")
                self.MainState = MainState.FAILURE
                print(self.MainState)


        elif self.MainState == MainState.SUCCESS:
            pass
        
        elif self.MainState == MainState.FAILURE:
            pass

        # print(self.MainState)
        # print(self.initial_pose_finished)
        # print(self.status)


if __name__ == '__main__':
    try:
        InitialPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
