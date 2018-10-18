#include <ros/ros.h>
#include <iostream>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

int main(int argv, char ** argc){

    ros::init(argv, argc, "tf_transform");
    ros::NodeHandle mNodeHandle;

    tf::TransformListener listener;

    ros::Rate rate(10.0);
    ros::Time now = ros::Time.now();

    geometry_msgs::PoseStamped right_hand_pose;
    geometry_msgs::PoseStamped base_link_pose;

    while(ros::ok()){

        try {
            listener.transformPose("base_link", now, right_hand_pose, "kinect2_rgb_optical_frame",  base_link_pose);
        } catch (tf::TransformException &ex) {

        }

    }
}
