#include "./../include/publisher.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <geometry_msgs/Transform.h>
#include <tf/transform_broadcaster.h>
#include <vector>


using namespace cv;
using namespace std;
using namespace tf;


Publisher::Publisher()
    : it_(nh_)
{
    intrinsics.at<double>(0,0) = 557.406874834532;
    intrinsics.at<double>(0,1) = 0.0;
    intrinsics.at<double>(0,2) = 287.2472655023014;
    intrinsics.at<double>(1,0) = 0.0;
    intrinsics.at<double>(1,1) = 561.5082308935905;
    intrinsics.at<double>(1,2) = 167.2745685278068;
    intrinsics.at<double>(2,0) = 0.0;
    intrinsics.at<double>(2,1) = 0.0;
    intrinsics.at<double>(2,2) = 1.0;

    distortion.at<double>(0,0) = 0;
    distortion.at<double>(0,1) = 0;
    distortion.at<double>(0,2) = 0;
    distortion.at<double>(0,3) = 0;
    distortion.at<double>(0,4) = 0;
    distortion.at<double>(0,5) = 0;

    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 10, &Publisher::imageCb, this);
    image_pub_ = it_.advertise("/calibration/output", 10);

    camera_object_publisher_ = nh_.advertise<geometry_msgs::Transform>("/camera_object", 10);
    world_effector_publisher_ = nh_.advertise<geometry_msgs::Transform>("/world_effector", 10);

    drone_pose_subscriber_ = nh_.subscribe("/optitrack/rgbd_sensor/pose", 10, &Publisher::PoseCb, this);

    cv::namedWindow(OPENCV_WINDOW);
}


void Publisher::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  vector<Point2f> ptvec;
  vector<Point3f> boardPoints;
  cv_bridge::CvImagePtr cv_ptr;

  tf::TransformBroadcaster tfbr;

  for (int i=0; i<8; i++)
  {
    for (int j=0; j<6; j++)
    {
      boardPoints.push_back( Point3f( double(i), double(j), 0.0) );
    }
  }

  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  bool found = findChessboardCorners( cv_ptr->image, cvSize(8,6), ptvec, CV_CALIB_CB_ADAPTIVE_THRESH);

  if (found){

    //cout << "Intrinsics" << intrinsics << endl;
    //cout << "Dist" << distortion << endl;
    solvePnP (boardPoints, ptvec, intrinsics, distortion, rvec, tvec, false);


    cout << fixed << setprecision(2) << "rvec = ["
            << rvec.at<double>(0,0) << ", "
            << rvec.at<double>(1,0) << ", "
            << rvec.at<double>(2,0) << "] \t" << "tvec = ["
            << tvec.at<double>(0,0) << ", "
            << tvec.at<double>(1,0) << ", "
            << tvec.at<double>(2,0) << "]" << endl;

    }

  c2o.translation.x = tvec.at<double>(0,0);
  c2o.translation.y = tvec.at<double>(0,1);
  c2o.translation.z = tvec.at<double>(0,2);

  q.setRPY(rvec.at<double>(0,2), rvec.at<double>(0,1), rvec.at<double>(0,0));

  c2o.rotation.w = q.getW();
  c2o.rotation.x = q.getX();
  c2o.rotation.y = q.getY();
  c2o.rotation.z = q.getZ();

  tmptf.setOrigin(tf::Vector3(c2o.translation.x, c2o.translation.y, c2o.translation.z));
  tmptf.setRotation(q);
  tmptf.inverse();


  invc2o.translation.x = tmptf.getOrigin().getX();
  invc2o.translation.y = tmptf.getOrigin().getY();
  invc2o.translation.z = tmptf.getOrigin().getZ();

  invc2o.rotation.w = tmptf.getRotation().getW();
  invc2o.rotation.x = tmptf.getRotation().getX();
  invc2o.rotation.y = tmptf.getRotation().getY();
  invc2o.rotation.z = tmptf.getRotation().getZ();


/*
  c2otf.setOrigin(tf::Vector3(c2o.translation.x, c2o.translation.y, c2o.translation.z));
  c2otf.setRotation(q);
  c2otf.frame_id_ = "camera";
  c2otf.child_frame_id_ = "object";

  tfbr.sendTransform(c2otf);
*/
  camera_object_publisher_.publish(invc2o);

  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);

  image_pub_.publish(cv_ptr->toImageMsg());
}


void Publisher::PoseCb(const geometry_msgs::PoseStampedConstPtr &msg){

  w2d.translation.x = msg->pose.position.x;
  w2d.translation.y = msg->pose.position.y;
  w2d.translation.z = msg->pose.position.z;

  w2d.rotation.w = msg->pose.orientation.w;
  w2d.rotation.x = msg->pose.orientation.x;
  w2d.rotation.y = msg->pose.orientation.y;
  w2d.rotation.z = msg->pose.orientation.z;

  world_effector_publisher_.publish(w2d);


}
