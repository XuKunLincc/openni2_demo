#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose

right_hand_pose = PoseStamped()
right_hand_pose.header.seq = 1
right_hand_pose.header.frame_id = "kinect2_rgb_optical_frame"

get_pose = False

def callback(data):
    global right_hand_pose
    global get_pose
    right_hand_pose.pose = data.poses[0]
    get_pose = True

if __name__ == '__main__':
    rospy.init_node('transform_to_base')

    rospy.Subscriber("right_hand_pose", PoseArray, callback)
    pub = rospy.Publisher("right_hand_pose_base", Pose, queue_size=20)
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    global get_pose

    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        if get_pose:
            get_pose = False
            try:
                base_link_pose = listener.transformPose('base_link', right_hand_pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            pos = base_link_pose.pose.position
            orien = base_link_pose.pose.orientation
            br.sendTransform((pos.x, pos.y, pos.z), (orien.x, orien.y, orien.z, orien.w), rospy.Time.now(), "right_hand", "base_link")
            pub.publish(base_link_pose.pose)
            rospy.loginfo(base_link_pose)

        rate.sleep()
