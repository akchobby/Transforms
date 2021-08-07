#!/usr/bin/env python
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import rospy



FRAME_ID = "base_link" # base_link_kimera for experimental, base_link for sim

class Fusion():

    def __init__(self):
        rospy.init_node('ros_sample', anonymous=True, log_level=rospy.INFO)
        rospy.loginfo("Starting publish node...")

        self._pub = rospy.Publisher('pose', PoseStamped, queue_size=100)
        self.pub_debug = rospy.Publisher("pose_debug", PoseStamped, queue_size=100 )
        self.cnt = 0


    def spin(self):
        rate = rospy.Rate(4)
        while True:
            self.publish()
            self.publish_debug()
            if rospy.is_shutdown():
                rospy.loginfo("Shutting down...")
                self._pub.unregister()
                break
            rate.sleep()
      


    def publish(self):
        #time.sleep(0.02)
        self._pub.publish(self.pose_creator())
    
    def publish_debug(self):
        #time.sleep(0.02)
        self.pub_debug.publish(self.pose_creator())



    
    def pose_creator(self):
        obj = PoseStamped()
        obj_pose = Pose()
        obj_pose.position.x = 1.0
        obj_pose.position.y = 1.0
        obj_pose.position.z = 1.0
        obj_pose.orientation.x = 0.0 # name
        obj_pose.orientation.y = 0.0 # width
        obj_pose.orientation.z = 20.0 # height
        obj.header.stamp = rospy.Time.now() + rospy.Duration(self.cnt) 
        obj.pose = obj_pose
        self.cnt += 1
        return obj




                    



if __name__ == '__main__':
    CameraLidarFusion = Fusion()
    CameraLidarFusion.spin()

