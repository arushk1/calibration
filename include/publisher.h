#ifndef PUBLISHER_H
#define PUBLISHER_H


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



class Publisher
{
public:
  Publisher();

  tf::StampedTransform c2otf;

  Mat intrinsics = Mat(Size(3,3), CV_64F);
  Mat distortion = Mat(Size(1,5), CV_64F);

  Mat rvec = Mat(Size(3,1), CV_64F);
  Mat tvec = Mat(Size(3,1), CV_64F);

  geometry_msgs::Transform c2o;
  tf::Quaternion q;
  geometry_msgs::Transform w2d;

  geometry_msgs::Transform invc2o;
  tf::Transform tmptf;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher camera_object_publisher_;
  ros::Publisher world_effector_publisher_;

  const std::string OPENCV_WINDOW = "Image window";

  ros::Subscriber drone_pose_subscriber_;

  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  void PoseCb(const geometry_msgs::PoseStampedConstPtr &msg);

};

#endif
