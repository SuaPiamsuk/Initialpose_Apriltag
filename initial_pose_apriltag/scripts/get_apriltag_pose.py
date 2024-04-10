#!/usr/bin/env python

import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import Bool
from initial_pose_apriltag.msg import apriltag_pose
import yaml
import rospkg

class MyServiceServer:
    def __init__(self):
        rospy.init_node('initial_pose_apriltag')

        # Publisher
        self.apriltag_pose_pub = rospy.Publisher('/pose_map_to_apriltag', apriltag_pose, queue_size=10)
        self.start_stop_undistort_pub = rospy.Publisher('/start_stop_undistort_get_apriltag_pose', Bool, queue_size=10)

        # subscriber
        self.tf_tag_subscription = rospy.Subscriber('/get_tf_map_to_apriltag', Bool, self.req_apriltag_pose_callback)
        self.tf_tag_subscription = rospy.Subscriber('/save_apriltag_pose', Bool, self.save_apriltag_pose_callback)
        self.tag_subscription = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.apriltag_callback) 
        self.apriltag_msg = None
        self.req = None

        # Timer
        self.duration_time = 0.2
        self.timer = rospy.Timer(rospy.Duration(self.duration_time), self.initial_pose_callback)

        # TF
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_apriltag_to_robot = None
        
        #get param
        self.tag_bundles = rospy.get_param("/apriltag_ros_continuous_node/tag_bundles")

        # Declare variable
        self.current_time = rospy.Time.now().to_sec()

        rospy.loginfo("Service server is ready.")
    
    def save_apriltag_pose_callback(self, msg):
        
        if msg.data == True:
            
            try:

                # dump to yaml file
                translation = [self.tf_apriltag_to_robot.transform.translation.x, self.tf_apriltag_to_robot.transform.translation.y, self.tf_apriltag_to_robot.transform.translation.z]
                rotation = [self.tf_apriltag_to_robot.transform.rotation.x, self.tf_apriltag_to_robot.transform.rotation.y, self.tf_apriltag_to_robot.transform.rotation.z, self.tf_apriltag_to_robot.transform.rotation.w]
                print("1")
                # Create a dictionary to hold translation and rotation data
                data_dict = {
                    'rotation': rotation,
                    'translation': translation
                }
            

                rospack = rospkg.RosPack()
                yaml_file_path = rospack.get_path('initial_pose_apriltag') + '/config/apriltag_to_map_pose.yaml'
                # Dump the dictionary to a YAML file
                with open(yaml_file_path, 'w') as yaml_file:
                    yaml.dump(data_dict, yaml_file, default_flow_style=False)
                rospy.loginfo("Transform data dumped to transform_data.yaml")
            except:
                pass

    def req_apriltag_pose_callback(self, msg):
        self.req = msg.data
        self.current_time = rospy.Time.now().to_sec()

    def apriltag_callback(self,msg):
        self.apriltag_msg = msg
        # print(msg)
    
    def get_tag_name(self):
        dict = {}
        try:
            if (len(self.apriltag_msg.detections) > 1):
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
                    
            elif (len(self.apriltag_msg.detections) == 1):
                bundle_found = self.apriltag_msg.detections[0].id
                for bundle in self.tag_bundles:
                    layout = bundle.get('layout', [])
                    bundle_ids = {tag['id'] for tag in layout}  # เก็บ IDs ของ bundle เป็นเซ็ตสำหรับการเปรียบเทียบ
                    
                    # เช็คว่า bundle_ids เป็น subset ของ id_pattern
                    if set(bundle_found).issubset(bundle_ids):
                        # print(bundle.get('name')) 
                        update_tag_name = bundle.get('name')
                        return update_tag_name
        except:
            pass

    def initial_pose_callback(self, timer):
        try:
            if (rospy.Time.now().to_sec() - self.current_time <= 300) : #60*5 sec = 5 mins

                if self.req == True:
                    # print("start")
                    #1
                    self.start_stop_undistort_pub.publish(True)

                    #2
                    try:
                        tag_id = self.get_tag_name()
                       
                        self.tf_apriltag_to_robot = self.tfBuffer.lookup_transform('map', tag_id, rospy.Time()) 

                        

                        msg = apriltag_pose()
                        msg.tag_id = int(tag_id)
                        msg.position.x = self.tf_apriltag_to_robot.transform.translation.x
                        msg.position.y = self.tf_apriltag_to_robot.transform.translation.y
                        msg.position.z = self.tf_apriltag_to_robot.transform.translation.z
                        msg.orientation.x = self.tf_apriltag_to_robot.transform.rotation.x 
                        msg.orientation.y = self.tf_apriltag_to_robot.transform.rotation.y 
                        msg.orientation.z = self.tf_apriltag_to_robot.transform.rotation.z 
                        msg.orientation.w = self.tf_apriltag_to_robot.transform.rotation.w 

                        self.apriltag_pose_pub.publish(msg)

                        

                    except:
                        pass
                    
                else:
                    self.start_stop_undistort_pub.publish(False)
        
            else:
                self.start_stop_undistort_pub.publish(False)
        except:
            print("error")

if __name__ == "__main__":
    try:
        my_service_server = MyServiceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("An error occurred.")

