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
    VideoFrameRef mColorFrRef;
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
    VideoStream mDepthStream ;
    VideoStream mColorStream;

    mDepthStream.create( m_device, SENSOR_DEPTH );
    mColorStream.create( m_device, SENSOR_COLOR );
/*
    VideoMode mDepthMode;
    mDepthMode.setResolution( 640, 480 );
    mDepthMode.setFps( 30 );
    mDepthMode.setPixelFormat( PIXEL_FORMAT_DEPTH_1_MM );
    mDepthStream.setVideoMode(mDepthMode);
*/

    VideoMode mColorMode;
    mColorMode.setResolution( 320, 240 );
    mColorMode.setFps( 30 );
    mColorMode.setPixelFormat( PIXEL_FORMAT_RGB888 );
    mColorStream.setVideoMode(mColorMode);


    m_device.setImageRegistrationMode( IMAGE_REGISTRATION_DEPTH_TO_COLOR );
    mDepthStream.start();
    mColorStream.start();
    // read depth image
  //  mDepthStream.readFrame(&mDepthFrRef);

    // set the depth <==> color


    while(true){
        mColorStream.readFrame(&mColorFrRef);

       // cv::Mat cvRawImg16U( mDepthFrRef.getHeight(), mDepthFrRef.getWidth(), CV_16UC1, (void*)mDepthFrRef.getData() );
      //  cvRawImg16U.convertTo( cvDepthImg, CV_8U, 255.0/(mDepthStream.getMaxPixelValue()));

        cv::Mat cvRgbImg(mColorFrRef.getHeight(), mColorFrRef.getWidth(), CV_8UC3, (void*)mColorFrRef.getData() );
        cv::Mat cImageBGR;
        cv::cvtColor(cvRgbImg, cImageBGR, CV_RGB2BGR);

     //   cv::imshow("depth_img", cvDepthImg);
        cv::imshow("rgb_img", cImageBGR);

        if( cv::waitKey(1) == 27)
            break;
    }
    return 0;

}
