#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool, String

from initial_pose.srv import auto_initial_pose
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
        rospy.init_node('initial_pose_publisher', anonymous=True)
        
        # Publisher for /initialpose topic
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.start_stop_undistort_pub = rospy.Publisher('/start_stop_undistort', Bool, queue_size=10)
        self.initial_pose_finished_pub = rospy.Publisher('/initial_pose_finished', Bool, queue_size=10)
        self.status_pub = rospy.Publisher('/status', String, queue_size=10)
        
        # Subscriber for /amcl_pose topic
        self.amcl_pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.tag_subscription = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.apriltag_callback)
        self.tag_subscription
        self.apriltag_msg = None
        
        # service
        srv = rospy.Service('initial_pose_apriltag_srv', auto_initial_pose, self.initial_pose_srv)

        # Timer
        self.duration_time = 0.2
        self.timer = rospy.Timer(rospy.Duration(self.duration_time), self.initial_pose_callback)
        
        self.amcl_pose_received = False

        self.count = 0

         # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_apriltag_to_robot = None


        self.force_initail = False

        self.MainState = MainState.INITIAL
        self.force_initial_on_process = None

        self.srv_called = False

        self.Tranform = None
        self.cal_tranform_finished = False

        self.already_get_info = False
        self.tag_name = None
        self.trans_of_map_to_tag= None
        self.rot_of_map_to_tag = None
       
        self.tag_bundles = rospy.get_param("/apriltag_ros_continuous_node/tag_bundles")

       
        # Get path to the YAML file
        rospack = rospkg.RosPack()
        yaml_file_path = rospack.get_path('initial_pose_apriltag') + '/config/apriltag_to_map_pose.yaml'

        # Load parameters from the YAML file
        with open(yaml_file_path, 'r') as stream:
            try:
                self.apriltag_pose = yaml.safe_load(stream)
                # rospy.loginfo("Loaded parameters: %s", params)
                rospy.loginfo("Loaded parameters")
            except yaml.YAMLError as e:
                rospy.logerr("Failed to load parameters from YAML file: %s", str(e))
                
    def initial_pose_srv(self, msg):
        print("hello world")
        self.srv_called = True
        # self.MainState = MainState.INITIAL

        # reset
        self.amcl_pose_received = False
        self.cal_tranform_finished = False
        self.already_get_info = False

        return "send srv succeed"

    def apriltag_callback(self, msg):
        self.apriltag_msg = msg

    def cal_homogeneous_matrix(self, angles, translation): # radians, meters
        """Convert Euler angles and translation to a homogeneous transformation matrix."""
        roll, pitch, yaw = angles

        # Conversion to radians
        # roll = np.radians(roll)
        # pitch = np.radians(pitch)
        # yaw = np.radians(yaw)

        # Homogeneous transformation matrix components
        c1 = np.cos(roll)
        s1 = np.sin(roll)
        c2 = np.cos(pitch)
        s2 = np.sin(pitch)
        c3 = np.cos(yaw)
        s3 = np.sin(yaw)

        # Rotation matrices
        R_x = np.array([[1, 0, 0],
                        [0, c1, -s1],
                        [0, s1, c1]])

        R_y = np.array([[c2, 0, s2],
                        [0, 1, 0],
                        [-s2, 0, c2]])

        R_z = np.array([[c3, -s3, 0],
                        [s3, c3, 0],
                        [0, 0, 1]])

        # Combined rotation matrix
        R = np.dot(R_z, np.dot(R_y, R_x))

        # Homogeneous transformation matrix
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = translation

        return H

    def homogeneous_matrix_to_quaternion_and_translation(self, matrix):
        """Extract Euler angles (roll, pitch, yaw) and translation (xyz) from a homogeneous transformation matrix."""
        R = matrix[:3, :3]
        d = matrix[:3, 3]

        x = matrix[0, 3]
        y = matrix[1, 3]
        z = matrix[2, 3]

        # Extract Euler angles from rotation matrix
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
        yaw = np.arctan2(R[1, 0], R[0, 0])

        # Convert Euler angles to degrees
        # roll = np.degrees(roll)
        # pitch = np.degrees(pitch)
        # yaw = np.degrees(yaw)

        q = quaternion_from_euler(roll, pitch, yaw)

        return x, y, z, q[0], q[1], q[2], q[3] #x, y, z, quaternion_x, quaternion_y, quaternion_z, quaternion_w

    # def initialpose_auto_callback(self, msg):
    #     self.force_initial = msg.data
    
    def initial_pose(self, tag_name_, trans_of_map_to_tag_, rot_of_map_to_tag_):
    
        try:
            if self.cal_tranform_finished == False:
                try:
                    self.tf_apriltag_to_robot = self.tfBuffer.lookup_transform(tag_name_, 'base_footprint', rospy.Time()) 
                except:
                    pass
                
                # create new transform map -> tag
                self.Tranform = self.cal_homogeneous_matrix([0, 0, 0],trans_of_map_to_tag_)
                self.Tranform = np.dot(self.Tranform, self.cal_homogeneous_matrix(rot_of_map_to_tag_,[0,0,0]))

                # get tf from tag to robot
                trans_tag_to_robot = [self.tf_apriltag_to_robot.transform.translation.x, self.tf_apriltag_to_robot.transform.translation.y, self.tf_apriltag_to_robot.transform.translation.z]
                rot_tag_to_robot = euler_from_quaternion(self.tf_apriltag_to_robot.transform.rotation.x, self.tf_apriltag_to_robot.transform.rotation.y, self.tf_apriltag_to_robot.transform.rotation.z, self.tf_apriltag_to_robot.transform.rotation.w)
                
                # transform map -> robot
                self.Tranform = np.dot(self.Tranform, self.cal_homogeneous_matrix([0, 0, 0],trans_tag_to_robot))
                self.Tranform = np.dot(self.Tranform, self.cal_homogeneous_matrix(rot_tag_to_robot,[0, 0, 0]))
                
                self.Tranform = self.homogeneous_matrix_to_quaternion_and_translation(self.Tranform)

                self.cal_tranform_finished = True
            
            else:
                initial_pose_msg = PoseWithCovarianceStamped()
                # initial_pose_msg.header.stamp = rospy.Time.now()
                initial_pose_msg.header.frame_id = "map"
                initial_pose_msg.pose.pose.position = Point(self.Tranform[0], self.Tranform[1], self.Tranform[2])
                initial_pose_msg.pose.pose.orientation = Quaternion(self.Tranform[3], self.Tranform[4], self.Tranform[5], self.Tranform[6]) 
                initial_pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
                self.initialpose_pub.publish(initial_pose_msg)
              
        except:
            pass
    
    def get_info_from_web(self, tag_name):
        try:
            trans_map_to_tag = [4.015, 0.066, 0.262] #xyz
            rot_map_to_tag = [1.625, -0.001, -1.558] #rpy 
            # rot_map_to_tag = euler_from_quaternion()

            trans_map_to_tag = [self.apriltag_pose['translation'][0], self.apriltag_pose['translation'][1],self.apriltag_pose['translation'][2]] #xyz
            # rot_map_to_tag = [1.625, -0.001, -1.558] #rpy 
            rot_map_to_tag = euler_from_quaternion(self.apriltag_pose['rotation'][0], self.apriltag_pose['rotation'][1], self.apriltag_pose['rotation'][2], self.apriltag_pose['rotation'][3])

            return trans_map_to_tag, rot_map_to_tag

        except:
            print("error")
    
    def get_tag_name(self):
        dict = {}
        try:
            # 1. put info to dictionary
            for i in self.apriltag_msg.detections:              
                distance = i.pose.pose.pose.position.z
                tag_id = i.id
                # add to dictionary {0.x = {id_x}}
                dict[distance] = tag_id

            # 2. find tag that lowest distance
            lowest_distance = min(dict)
            nearest_bundle = dict[lowest_distance]

            # 3. find bundle_name
            for bundle in self.tag_bundles:
                layout = bundle.get('layout', [])
                bundle_ids = {tag['id'] for tag in layout}  # เก็บ IDs ของ bundle เป็นเซ็ตสำหรับการเปรียบเทียบ
                
                # เช็คว่า bundle_ids เป็น subset ของ id_pattern
                if set(nearest_bundle).issubset(bundle_ids):
                    # print(bundle.get('name')) 
                    update_tag_name = bundle.get('name')
                    return update_tag_name
            
            # print(update_tag_name)
        except:
            pass

        # print(len(self.apriltag_msg.detections) == 0)

    def amcl_pose_callback(self, msg):
        try:
            if (self.Tranform[0] - 0.05 < round(msg.pose.pose.position.x,2) < self.Tranform[0] + 0.05) and self.Tranform[1] - 0.05 < (round(msg.pose.pose.position.y,2) < self.Tranform[1] + 0.05):
                self.amcl_pose_received = True
                # rospy.signal_shutdown("Finished")
        except:
            pass

    def initial_pose_callback(self, timer):
        
        #     ###################################################################
        #     # 1. open undistort to start apriltag node
        #     # 2. get bundle name
        #     # 3. get pose from map tp tag 
        #     # 4. intial pose
        #     # 5. close undistort to stop apriltag node
        #     ###################################################################
        if self.MainState == MainState.INITIAL:
           # reset
            self.amcl_pose_received = False
            self.cal_tranform_finished = False
            self.already_get_info = False

            self.MainState = MainState.WAITTING

        elif self.MainState == MainState.WAITTING:
            self.status_pub.publish(str(self.MainState.name))
            if self.srv_called == True :
                self.MainState = MainState.RUNNING
        
        elif self.MainState == MainState.RUNNING:
            self.initial_pose_finished_pub.publish(False)
            self.start_stop_undistort_pub.publish(True)
        
            if self.already_get_info == False:
                try:
                    self.tag_name = None
                    self.trans_of_map_to_tag = None
                    self.rot_of_map_to_tag = None

                    self.tag_name = self.get_tag_name()
                    self.trans_of_map_to_tag, self.rot_of_map_to_tag = self.get_info_from_web(self.tag_name)
                    if (self.tag_name != None) and (self.trans_of_map_to_tag !=  None) and (self.rot_of_map_to_tag != None):
                        self.already_get_info = True
                except:
                    pass
            else:
                self.initial_pose(self.tag_name, self.trans_of_map_to_tag, self.rot_of_map_to_tag)

            if self.amcl_pose_received == True :
                self.start_stop_undistort_pub.publish(False)
                self.initial_pose_finished_pub.publish(True)
                self.apriltag_msg = None
                self.srv_called = False
                self.MainState = MainState.SUCCESS
            
            if (self.apriltag_msg != None) and (len(self.apriltag_msg.detections) == 0) : # tag not found
                self.start_stop_undistort_pub.publish(False)
                self.initial_pose_finished_pub.publish(False)
                self.apriltag_msg = None
                self.srv_called = False
                self.MainState = MainState.FAILURE
        
        elif self.MainState == MainState.SUCCESS:           
            self.status_pub.publish(str(self.MainState.name))
            self.MainState = MainState.INITIAL
           

        elif self.MainState == MainState.FAILURE:
            self.status_pub.publish(str(self.MainState.name))
            print("Apriltag_bundle_not_found")
            self.MainState = MainState.INITIAL
           
        # print((str(self.MainState.name)))

        # self.status_pub.publish(str(self.MainState.name))

    

if __name__ == '__main__':
    try:
        InitialPosePublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass







     # def initial_pose(self):
    #     # Update via srv? or else
    #     # - map_name
    #     # - tag_name
    #     update_map_name = 'UTL1_map_1_name'
    #     update_tag_name = 'bundle_1'

        
        
    #     try:
    #         self.tf_apriltag_to_robot = self.tfBuffer.lookup_transform(update_tag_name, 'base_footprint', rospy.Time()) 
    #     except:
    #         pass

    #     try:
    #         if self.cal_tranform_finished == False:
    #             # get tf from map to tag
    #             trans_map_to_tag = self.params['map_name'][update_map_name]['tag_name'][update_tag_name]['translation']
    #             rot_map_to_tag = self.params['map_name'][update_map_name]['tag_name'][update_tag_name]['rotation']

    #             # get tf from tag to robot
    #             trans_tag_to_robot = [self.tf_apriltag_to_robot.transform.translation.x, self.tf_apriltag_to_robot.transform.translation.y, self.tf_apriltag_to_robot.transform.translation.z]
    #             rot_tag_to_robot = euler_from_quaternion(self.tf_apriltag_to_robot.transform.rotation.x, self.tf_apriltag_to_robot.transform.rotation.y, self.tf_apriltag_to_robot.transform.rotation.z, self.tf_apriltag_to_robot.transform.rotation.w)
                
    #             # create new transform map -> tag
    #             self.Tranform = self.cal_homogeneous_matrix([0, 0, 0],trans_map_to_tag)
    #             self.Tranform = np.dot(self.Tranform, self.cal_homogeneous_matrix(rot_map_to_tag,[0,0,0]))

    #             # transform map -> robot
    #             self.Tranform = np.dot(self.Tranform, self.cal_homogeneous_matrix([0, 0, 0],trans_tag_to_robot))
    #             self.Tranform = np.dot(self.Tranform, self.cal_homogeneous_matrix(rot_tag_to_robot,[0, 0, 0]))
                
    #             self.Tranform = self.homogeneous_matrix_to_quaternion_and_translation(self.Tranform)

    #             self.cal_tranform_finished = True

    #         initial_pose_msg = PoseWithCovarianceStamped()
    #         # initial_pose_msg.header.stamp = rospy.Time.now()
    #         initial_pose_msg.header.frame_id = "map"
    #         initial_pose_msg.pose.pose.position = Point(self.Tranform[0], self.Tranform[1], self.Tranform[2])
    #         initial_pose_msg.pose.pose.orientation = Quaternion(self.Tranform[3], self.Tranform[4], self.Tranform[5], self.Tranform[6])  # No rotation
    #         initial_pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                                         0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    #                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
    #         self.initialpose_pub.publish(initial_pose_msg)
    #         print('waitting')
    #     except:
    #         pass