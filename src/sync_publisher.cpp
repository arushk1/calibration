#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Pose.h>
#include "./../include/client.h"
#include <iostream>
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>

using namespace message_filters;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace std;
char input;


geometry_msgs::Transform cam2obj;
geometry_msgs::Transform wld2eff;

ros::Publisher camera_object_publisher_;
ros::Publisher world_effector_publisher_;

void callback(const PoseStampedConstPtr& posemsg, const PoseStampedConstPtr& trackermsg)
{

    tf::Transform tf_pub;
    static tf::TransformBroadcaster br;


    cam2obj.translation.x = posemsg->pose.position.z;
    cam2obj.translation.y = -(posemsg->pose.position.x);
    cam2obj.translation.z = -(posemsg->pose.position.y);
    cam2obj.rotation = posemsg->pose.orientation;

    wld2eff.translation.x = trackermsg->pose.position.x*100;
    wld2eff.translation.y = trackermsg->pose.position.y*100;
    wld2eff.translation.z = trackermsg->pose.position.z*100;
    wld2eff.rotation = trackermsg->pose.orientation;

    ROS_INFO_STREAM("TS CheckerPose: " << posemsg->header.stamp);
    ROS_INFO_STREAM("TS Optitrack: " << trackermsg->header.stamp);
    tf::Quaternion q;
    q.setW(cam2obj.rotation.w);
    q.setX(cam2obj.rotation.x);
    q.setY(cam2obj.rotation.y);
    q.setZ(cam2obj.rotation.z);

    tf_pub.setOrigin(tf::Vector3(cam2obj.translation.x/100, cam2obj.translation.y/100, cam2obj.translation.z/100));
    tf_pub.setRotation(q);
    br.sendTransform(tf::StampedTransform(tf_pub, ros::Time::now(), "/camera_link", "/checker_frame"));

    camera_object_publisher_.publish(cam2obj);
    world_effector_publisher_.publish(wld2eff);

    cout << "K.Thnx" << endl;
    //cin >> input;



}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "publisher_node");

  ros::NodeHandle nh;
  message_filters::Subscriber<geometry_msgs::PoseStamped> checker_sub(nh, "/marker_extractor/pose", 100);
  message_filters::Subscriber<geometry_msgs::PoseStamped> tracker_sub(nh, "/optitrack/pixhawk/pose", 200);

  camera_object_publisher_ = nh.advertise<geometry_msgs::Transform> ("camera_object", 1000);
  world_effector_publisher_ = nh.advertise<geometry_msgs::Transform> ("world_effector", 1000);

  typedef sync_policies::ApproximateTime<PoseStamped, PoseStamped> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), checker_sub, tracker_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
