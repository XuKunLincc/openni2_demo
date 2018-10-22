#include <NiTE.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#define VISI_DEBUG 1

using namespace std;
using namespace nite;
using namespace cv;

#define N 4

ros::NodeHandle *mNodeHandle;
ros::Publisher posePub;

int data_index = 0;

double target_x = 0;
double target_y = 0;
double target_z = 0;

typedef struct {
    double x[N];
    double y[N];
    double z[N];
    double target_x;
    double target_y;
    double target_z;
} hand_data;

map<int,hand_data> hand_data_map;
map<int,hand_data>::iterator it;

// init ros env
int rosInit(int argc, char **argv){
    ros::init(argc, argv, "right_handle");
    mNodeHandle = new ros::NodeHandle();
    posePub = mNodeHandle->advertise<geometry_msgs::PoseArray>("right_hand_pose", 1000);
}

void getData(hand_data &data){
    data_index = 0;
    double max_x = data.x[0], max_y = data.y[0], max_z = data.z[0];
    double min_x = max_x, min_y = max_y, min_z = max_z;
    double sum_x = 0, sum_y = 0, sum_z = 0;

    for(int i = 0; i < N; i ++){
        if(data.x[i] > max_x)
            max_x = data.x[i];
        if(data.x[i] < min_x)
            min_x = data.x[i];
        if(data.y[i] > max_y)
            max_y = data.y[i];
        if(data.y[i] < min_y)
            min_y = data.y[i];
        if(data.z[i] > max_z)
            max_z = data.z[i];
        if(data.z[i] < min_z)
            min_z = data.z[i];

        sum_x += data.x[i];
        sum_y += data.y[i];
        sum_z += data.z[i];
    }

    sum_x -= (max_x + min_x);
    sum_y -= (max_y + min_y);
    sum_z -= (max_z + min_z);

    data.target_x =  sum_x / (N - 2);
    data.target_y =  sum_y / (N - 2);
    data.target_z =  sum_z / (N - 2);

}

int main(int argc, char **argv){

    // init ros
    rosInit(argc, argv);

    // init NiTE
    NiTE::initialize();

    // ADD UserTracker
    UserTracker mUserTracker;
    mUserTracker.create();

    // include user frame
    UserTrackerFrameRef mUserFrame;
    namedWindow( "SkeletonImage",  CV_WINDOW_AUTOSIZE );

    // 10Hz
    ros::Rate loop_rate(60);

    while(ros::ok()){

        // get the user body frame
        mUserTracker.readFrame(&mUserFrame);

        // get user list
        const nite::Array<nite::UserData>& aUsers = mUserFrame.getUsers();

        std::vector<geometry_msgs::Pose>  poses;

        ROS_INFO("user num =%d ", aUsers.getSize());

        for(int i = 0; i <  aUsers.getSize(); i ++){

            const nite::UserData& rUser = aUsers[i];

            if(rUser.isNew()){
                hand_data data;
                //data.target_x = 10;
                data.x[0] = 101;
                hand_data_map[(int)rUser.getId()] = data;
            }

            mUserTracker.startSkeletonTracking( rUser.getId() );

            const nite::Skeleton& rSkeleton = rUser.getSkeleton();

            if( rSkeleton.getState() == nite::SKELETON_TRACKED )
            {
                // 得到右手坐标
                const nite::SkeletonJoint& righthand
                        = rSkeleton.getJoint( nite::JOINT_RIGHT_HAND );
                const nite::Point3f& position = righthand.getPosition();
                const nite::Quaternion& quat = righthand.getOrientation();

                it = hand_data_map.find(rUser.getId());
                it->second.x[data_index] = position.x;
                it->second.y[data_index] = position.y;
                it->second.z[data_index] = position.z;

                data_index ++;
                if(data_index >= 3){
                    getData( it->second);      // will set index = 0
                    ROS_INFO("%s %f | %f | %f",  "right_hand_pos: ", position.x, position.y, position.z);
                    //cout << "右手坐标： " << position.x << "/" << position.y << "/" << position.z << "|||||||||||||||" <<   it->second.x[0] << endl;;
                    geometry_msgs::Pose pose;
                    pose.position.x =  (- it->second.target_x / 1000 ) - 0.1;  pose.position.y = - it->second.target_y  / 1000; pose.position.z = it->second.target_z  / 1000;
                    pose.orientation.x = quat.x; pose.orientation.y = quat.y; pose.orientation.z = quat.z; pose.orientation.w = quat.w;
                    poses.push_back(pose);
                }
            }
        }   // for

        // send any people right hand pos
        if( !poses.empty() ){
            geometry_msgs::PoseArray poseArray;
            poseArray.poses = poses;
            posePub.publish(poseArray);
        }

        // get user body pixels
        const nite::UserMap& userLabels = mUserFrame.getUserMap();
        const nite::UserId* pLabels = userLabels.getPixels();

#ifdef VISI_DEBUG
        cv::Mat mUserBody(mUserFrame.getDepthFrame().getHeight(),  mUserFrame.getDepthFrame().getWidth(), CV_16UC1,
                          (void *) pLabels);

        //  convert CV_16UC1 to CV_8U
        cv::Mat mScaledHandDepth;
        mUserBody.convertTo( mScaledHandDepth, CV_8U, 255.0);

        // show img
        for( int h = 0; h < mScaledHandDepth.size().height; h++){
            for(int w = 0; w < mScaledHandDepth.size().width; w++){
                uchar *ptr = mScaledHandDepth.ptr<uchar>(h , w) ;
                if( *ptr == 0)
                    *ptr = 123;
                else
                    *ptr = 0;
            }
        }

        // show the image
        cv::imshow( "SkeletonImage", mScaledHandDepth );
        if (waitKey(1) >= 0)
            break;
#endif

        //ros::spinOnce();
        loop_rate.sleep();

    } // while(true)

    mUserFrame.release();
    mUserTracker.destroy();
    nite::NiTE::shutdown();

    return 0;
}
