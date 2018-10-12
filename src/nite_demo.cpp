#include <NiTE.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace nite;

int main(){

    // init NiTE
    NiTE::initialize();

    // ADD UserTracker
    UserTracker mUserTracker;
    mUserTracker.create();


    UserTrackerFrameRef mUserFrame;

    while(true){

        // get the user body frame
        mUserTracker.readFrame(&mUserFrame);
        // get user number
        const nite::Array<nite::UserData>& aUsers = mUserFrame.getUsers();
        cout << "in this frame have:" << aUsers.getSize() <<  "peopel" << endl;

        for(int i = 0; i <  aUsers.getSize(); i ++){
            const nite::UserData& rUser = aUsers[i];

            // new user come in
            if(rUser.isNew()){
                 cout << "new user [" << rUser.getId() << "] come in." << endl;
                 mUserTracker.startSkeletonTracking( rUser.getId() );

                 const nite::Skeleton& rSkeleton = rUser.getSkeleton();
                 if( rSkeleton.getState() == nite::SKELETON_TRACKED )
                 {
                     // 得到右手坐标
                     const nite::SkeletonJoint& righthand
                         = rSkeleton.getJoint( nite::JOINT_RIGHT_HAND );
                     const nite::Point3f& position = righthand.getPosition();
                     cout << "右手坐标： " << position.x << "/" << position.y << "/" << position.z << endl;
                 }
            } // if user new
        }   // for
    } // while(true)

    mUserFrame.release();
    mUserTracker.destroy();
    nite::NiTE::shutdown();
}
