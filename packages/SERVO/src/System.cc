/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>

#include<tf/transform_broadcaster.h>



namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer):mSensor(sensor),mbReset(false),mbActivateLocalizationMode(false),
        mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    cout << endl <<
    "ORB-SLAM2 Copyright (C) 2014-2016 Raul Mur-Artal, University of Zaragoza." << endl <<
    "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    "This is free software, and you are welcome to redistribute it" << endl <<
    "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       exit(-1);
    }


    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();
    bool bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    //Initialize the Local Mapping thread and launch
    mpLocalMapper = new LocalMapping(mpMap, mSensor==MONOCULAR);
    mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run,mpLocalMapper);

    //Initialize the Loop Closing thread and launch
    mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
    mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);

    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer,mpMapDrawer,mpTracker,strSettingsFile);
    if(bUseViewer)
        mptViewer = new thread(&Viewer::Run, mpViewer);

    mpTracker->SetViewer(mpViewer);


    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    mpLoopCloser->SetTracker(mpTracker);
    mpLoopCloser->SetLocalMapper(mpLocalMapper);
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }   

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }
    return mpTracker->GrabImageStereo(imLeft,imRight,timestamp);
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageRGBD(im,depthmap,timestamp);
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
    {
        unique_lock<mutex> lock(mMutexMode);
        if(mbActivateLocalizationMode)
        {
            mpLocalMapper->RequestStop();

            // Wait until Local Mapping has effectively stopped
            while(!mpLocalMapper->isStopped())
            {
                usleep(1000);
            }

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    return mpTracker->GrabImageMonocular(im,timestamp);
}

void System::SetPublisherHandle(const ros::Publisher pubHandle)
{
    publisherHandle = pubHandle;
}

void System::PublishOdometry()
{
	if(!((mpTracker->mCurrentFrame).mTcw.empty()) & !(mpTracker->GetVelocity().empty()))
	{
        double dt = mpTracker->mCurrentFrame.mTimeStamp - mpTracker->GetLastTimeStamp();

        nav_msgs::Odometry odometryMsg;

		cv::Mat Tcw = (mpTracker->mCurrentFrame).mTcw.clone();
		cv::MatExpr Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
		cv::Mat twc = -(Rwc*Tcw.rowRange(0,3).col(3));
	    vector<float> q = Converter::toQuaternion(Rwc);

        //cv::Mat twistAngularSkew = Tcw.rowRange(0,2).colRange(0,2)*mpTracker->GetVelocity().rowRange(0,2).colRange(0,2).t();
        cv::Mat twistAngularSkew = cv::Mat::eye(4,4,CV_32F) - mpTracker->GetVelocity();
        cv::Mat twistLinear = -1.0d*mpTracker->GetVelocity().rowRange(0,3).col(3).clone();

		odometryMsg.header.seq = 1;
		odometryMsg.header.stamp = ros::Time(mpTracker->mCurrentFrame.mTimeStamp);
        odometryMsg.header.frame_id = "world";
        odometryMsg.child_frame_id = "ORB_odometry";
		odometryMsg.pose.pose.position.x = twc.at<float>(0);
		odometryMsg.pose.pose.position.y = twc.at<float>(1);
		odometryMsg.pose.pose.position.z = twc.at<float>(2);

		odometryMsg.pose.pose.orientation.x = q[0];
		odometryMsg.pose.pose.orientation.y = q[1];
		odometryMsg.pose.pose.orientation.z = q[2];
		odometryMsg.pose.pose.orientation.w = q[3];

		for(unsigned int i=0;i<6;i++){
		  for(unsigned int j=0;j<6;j++){
			odometryMsg.pose.covariance[j+6*i] = -1;
		  }
		}

		odometryMsg.twist.twist.linear.x = twistLinear.at<float>(0)/dt;
		odometryMsg.twist.twist.linear.y = twistLinear.at<float>(1)/dt;
		odometryMsg.twist.twist.linear.z = twistLinear.at<float>(2)/dt;
		odometryMsg.twist.twist.angular.x = (twistAngularSkew.at<float>(2,1)-twistAngularSkew.at<float>(1,2))/(2*dt);
		odometryMsg.twist.twist.angular.y = (twistAngularSkew.at<float>(0,2)-twistAngularSkew.at<float>(2,0))/(2*dt);
		odometryMsg.twist.twist.angular.z = (twistAngularSkew.at<float>(1,0)-twistAngularSkew.at<float>(0,1))/(2*dt);

		for(unsigned int i=0;i<6;i++){
		  for(unsigned int j=0;j<6;j++){
			odometryMsg.twist.covariance[j+6*i] = -1;
		  }
		}
		publisherHandle.publish(odometryMsg);
	}
}

void System::PublishPoseTransform(ros::Time t, const cv::Mat &T21, std::string frame1, std::string frame2)
{
    if(T21.empty())
    {
        cout << "Non-valid transform for transformation, tf message aborted" << endl;
        return;
    }

    tf::StampedTransform tfMsg;

    tfMsg.stamp_ = t;
    tfMsg.frame_id_ = frame1;
    tfMsg.child_frame_id_ = frame2;

    // R12 = T21(0:2,0:2).transposed()
    cv::Mat R12 = (T21.rowRange(0, 3).colRange(0, 3).t());
    // t12 = (-1)*R12*T21(0:2,3)
    cv::Mat t12 = R12*T21.rowRange(0, 3).col(3)*(-1);

    vector<float> q12 = Converter::toQuaternion(R12);

    tfMsg.setOrigin(tf::Vector3(t12.at<float>(0), t12.at<float>(1), t12.at<float>(2)));
    tfMsg.setRotation(tf::Quaternion(q12[0], q12[1], q12[2], q12[3]));

    tfBroadcaster.sendTransform(tfMsg);
}

void System::PublishInertialTransform(ros::Time t, std::string mapFrame, std::string worldFrame, std::string initialFrame)
{
    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.

    //vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    //sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
    //cv::Mat Tow = vpKFs[0]->GetPose();

    cv::Mat Tow = mpMap->GetFirstKeyframe()->GetPose();
    PublishPoseTransform(t, Tow, worldFrame, mapFrame);
    PublishPoseTransform(t, mpTracker->mInitialPosition, worldFrame, initialFrame);
}

// Check whether the tracker initial pose was initialized
bool System::CheckTrackerInitialization()
{
    return mpTracker->mInitialPosition.at<float>(3,3) > 0;
}

// Set the initial position using external pose measurements, initial frame at camera frame of external system
void System::SetTrackerInitialPose(const nav_msgs::OdometryConstPtr& msgOdometry,
                                   const geometry_msgs::PoseWithCovarianceStampedConstPtr& msgPose)
{
  cv::Mat Tcw = Converter::toCvMat(Converter::toEigenTf(msgOdometry, msgPose).inverse(Eigen::TransformTraits::Isometry));

  if(mpTracker->mbExtInit)
      mpTracker->mInitialPosition = Tcw;
  else
      mpTracker->mInitialPosition = cv::Mat::eye(4, 4, CV_32F);

      //Eigen::Transform<double,3,0> Twc = Converter::toEigenTf(msgOdometry, msgPose);
      //Eigen::Transform Tcw = Twc.inverse();

  if(mpTracker->mbExtOdo)
  {
      mpTracker->mExternalPoseMeas = Tcw;
      mpTracker->mLastExternalPoseMeas = mpTracker->mExternalPoseMeas;
  }
}

// Feed motion model using external odometry
void System::SetMotionModel(const nav_msgs::OdometryConstPtr& msgOdometry,
                            const geometry_msgs::PoseWithCovarianceStampedConstPtr& msgPose)
{
    if(mpTracker->mbExtOdo)
    {
        Eigen::Transform<double,3,0> Twc = Converter::toEigenTf(msgOdometry, msgPose);
        mpTracker->mLastExternalPoseMeas = mpTracker->mExternalPoseMeas;
        mpTracker->mExternalPoseMeas = Converter::toCvMat(Twc.inverse(Eigen::TransformTraits::Isometry));
    }
}


void System::ActivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode()
{
    unique_lock<mutex> lock(mMutexMode);
    mbDeactivateLocalizationMode = true;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    mpLoopCloser->RequestFinish();
    mpViewer->RequestFinish();

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished()  ||
          !mpViewer->isFinished()      || mpLoopCloser->isRunningGBA())
    {
        usleep(5000);
    }

    pangolin::BindToContext("ORB-SLAM2: Map Viewer");
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    cout << "origin was found at " << Two << endl;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    list<bool>::iterator lbL = mpTracker->mlbLost.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
        while(pKF->isBad())
        {
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        vector<float> q = Converter::toQuaternion(Rwc);

        f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R);
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

} //namespace ORB_SLAM
