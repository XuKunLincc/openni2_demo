#include <openni2/OpenNI.h>
#include "ros/ros.h"
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace openni;

int main(int argc, char **argv){
    ros::init(argc, argv, "openni2_demo");
    ros::NodeHandle m_nodeHandle;

    VideoFrameRef mDepthFrRef;
    cv::Mat cvDepthImg;

    // 初始化openni接口
    OpenNI::initialize();
    Array< DeviceInfo > deviceInfoLis;

    cout << "deviceInfoLis.getSize()" << endl;

    // connect kienct
    Device m_device;
   // open all device
    m_device.open(ANY_DEVICE);

    // get depth data
    VideoStream mDepthStream;
    mDepthStream.create( m_device, SENSOR_DEPTH );

    VideoMode mDepthMode;
    mDepthMode.setResolution( 640, 480 );
    mDepthMode.setFps( 30 );
    mDepthMode.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
    mDepthStream.setVideoMode(mDepthMode);

    // read depth image
    mDepthStream.readFrame(&mDepthFrRef);

    cv::Mat cvRawImg16U( mDepthFrRef.getHeight(), mDepthFrRef.getWidth(), CV_16UC1, (void*)mDepthFrRef.getData() );
    cvRawImg16U.convertTo( cvDepthImg, CV_8U, 255.0/(mDepthStream.getMaxPixelValue()));

    cv::imshow("depth_img", cvDepthImg);

    return 0;

}
