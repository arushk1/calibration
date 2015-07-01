#ifndef __visp_hand2eye_calibration_CLIENT_H__
#define __visp_hand2eye_calibration_CLIENT_H__
#include "ros/ros.h"
#include "geometry_msgs/Transform.h"
#include "visp_hand2eye_calibration/compute_effector_camera.h"
#include "visp_hand2eye_calibration/compute_effector_camera_quick.h"
#include "visp_hand2eye_calibration/reset.h"
#include "sensor_msgs/Image.h"
#include "visp/vpHomogeneousMatrix.h"
#include "image_transport/image_transport.h"

namespace visp_hand2eye_calibration{
  class Client{
  private:
    ros::NodeHandle n_;
    ros::Publisher camera_object_publisher_;
    ros::Publisher world_effector_publisher_;
    ros::Publisher pub_camera_object_;

    ros::Subscriber drone_pose_subscriber_;
    ros::Subscriber sub_cMo_;

    ros::ServiceClient reset_service_;
    ros::ServiceClient compute_effector_camera_service_;
    ros::ServiceClient compute_effector_camera_quick_service_;

    visp_hand2eye_calibration::reset reset_comm;
    visp_hand2eye_calibration::compute_effector_camera emc_comm;
    visp_hand2eye_calibration::compute_effector_camera_quick emc_quick_comm;

    vpHomogeneousMatrix cMo; // eye (camera) to object transformation. The object frame is attached to the calibrartion grid
    vpHomogeneousMatrix wMe; // world to hand (end-effector) transformation
    vpHomogeneousMatrix eMc; // hand (end-effector) to eye (camera) transformation (given for simulation)
    vpHomogeneousMatrix wMo; // world to object (target) transformation (given for simulation)

    void cMoCb(const geometry_msgs::Transform& msg);
    void simulateCameraMeasurements();
    int count_;


  public:
    Client();

    void initAndSimulate();
    void initAndSimulateUsingCam();
    void computeFromTopicStream();
    void computeUsingQuickService();
    void Simulator();
    void Publisher();
 };
}
#endif
