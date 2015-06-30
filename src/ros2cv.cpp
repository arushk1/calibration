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

static const std::string OPENCV_WINDOW = "Image window";


using namespace cv;
using namespace std;

Mat intrinsics = Mat(Size(3,3), CV_64F);
Mat distortion = Mat(Size(1,5), CV_64F);
geometry_msgs::Transform c2o;
tf::Quaternion q;
geometry_msgs::Quaternion qc2o;
geometry_msgs::Transform w2d;
geometry_msgs::Transform qw2d;


class ImageConverter
{

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Publisher camera_object_publisher_;
  ros::Publisher world_effector_publisher_;

  ros::Subscriber drone_pose_subscriber_;

public:
  ImageConverter()
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

    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 10, &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/calibration/output", 10);

    camera_object_publisher_ = nh_.advertise<geometry_msgs::Transform>("/camera_object", 10);
    world_effector_publisher_ = nh_.advertise<geometry_msgs::Transform>("/world_effector", 10);

    drone_pose_subscriber_ = nh_.subscribe("/optitrack/rgbd_sensor/pose", 10, &ImageConverter::PoseCb, this);
    cv::namedWindow(OPENCV_WINDOW);
    }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {

    vector<Point2f> ptvec;
    vector<Point3f> boardPoints;
    Mat rvec = Mat(Size(3,1), CV_64F);
    Mat tvec = Mat(Size(3,1), CV_64F);
    cv_bridge::CvImagePtr cv_ptr;

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

    qc2o.w = q.getW();
    qc2o.x = q.getX();
    qc2o.y = q.getY();
    qc2o.z = q.getZ();
    c2o.rotation = qc2o;

    camera_object_publisher_.publish(c2o);

    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    image_pub_.publish(cv_ptr->toImageMsg());
  }


  void PoseCb(const geometry_msgs::PoseStampedConstPtr &msg){

    w2d.translation.x = msg->pose.position.x;
    w2d.translation.y = msg->pose.position.y;
    w2d.translation.z = msg->pose.position.z;

    w2d.rotation.w = msg->pose.orientation.w;
    w2d.rotation.x = msg->pose.orientation.x;
    w2d.rotation.y = msg->pose.orientation.y;
    w2d.rotation.z = msg->pose.orientation.z;

    world_effector_publisher_.publish(w2d);


  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera2object");
  ImageConverter ic;
  ros::spin();
  return 0;
}
