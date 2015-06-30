#include "client.h"
#include <geometry_msgs/Transform.h>
#include "visp_hand2eye_calibration/TransformArray.h"
#include <visp_bridge/3dpose.h>
#include "names.h"

#include <visp/vpCalibration.h>
#include <visp/vpExponentialMap.h>


double camera_object[6][6];
double world_effector[6][6];
int counter;
using namespace std;

namespace visp_hand2eye_calibration
{
Client::Client()
{
  camera_object_publisher_ = n_.advertise<geometry_msgs::Transform> (visp_hand2eye_calibration::camera_object_topic, 1000);

  world_effector_publisher_ = n_.advertise<geometry_msgs::Transform> (visp_hand2eye_calibration::world_effector_topic, 1000);

  reset_service_ = n_.serviceClient<visp_hand2eye_calibration::reset> (visp_hand2eye_calibration::reset_service);

  compute_effector_camera_service_
          = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera> (visp_hand2eye_calibration::compute_effector_camera_service);

  compute_effector_camera_quick_service_
          = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick> (visp_hand2eye_calibration::compute_effector_camera_quick_service);

  camera_object[0][0] = 4.32;
  camera_object[0][1] = 4.24;
  camera_object[0][2] = 28.46;
  camera_object[1][0] = 5.22;
  camera_object[1][1] = 2.95;
  camera_object[1][2] = 27.68;
  camera_object[2][0] = 5.08;
  camera_object[2][1] = 2.53;
  camera_object[2][2] = 27.67;
  camera_object[3][0] = -4.36;
  camera_object[3][1] = 3.12;
  camera_object[3][2] = 25.49;
  camera_object[4][0] = -8.06;
  camera_object[4][1] = 2.89;
  camera_object[4][2] = 30.5;
  camera_object[5][0] = -8.03;
  camera_object[5][1] = 2.84;
  camera_object[5][2] = 3.41;

  world_effector[0][0] = -2.32;
  world_effector[0][1] = -3.24;
  world_effector[0][2] = -28.46;
  world_effector[1][0] = -3.22;
  world_effector[1][1] = -4.95;
  world_effector[1][2] = -27.68;
  world_effector[2][0] = -3.08;
  world_effector[2][1] = -1.53;
  world_effector[2][2] = -27.67;
  world_effector[3][0] = 6.36;
  world_effector[3][1] = -3.12;
  world_effector[3][2] = -25.49;
  world_effector[4][0] = 10.06;
  world_effector[4][1] = -2.89;
  world_effector[4][2] = -30.5;
  world_effector[5][0] = 10.03;
  world_effector[5][1] = -2.84;
  world_effector[5][2] = -3.41;
  counter = 6;
}

void Client::initAndSimulate()
{
  ROS_INFO("Waiting for topics...");
  ros::Duration(1.).sleep();
  while(!reset_service_.call(reset_comm)){
    if(!ros::ok()) return;
    ros::Duration(1).sleep();
  }


  // We want to calibrate the hand to eye extrinsic camera parameters from 6 couple of poses: cMo and wMe
  const int N = 6;
  // Input: six couple of poses used as input in the calibration proces
  vpHomogeneousMatrix cMo; // eye (camera) to object transformation. The object frame is attached to the calibrartion grid
  vpHomogeneousMatrix wMe; // world to hand (end-effector) transformation
  vpHomogeneousMatrix eMc; // hand (end-effector) to eye (camera) transformation

  // Initialize an eMc transformation used to produce the simulated input transformations cMo and wMe
  vpTranslationVector etc(0.1, 0.2, 0.3);
  vpThetaUVector erc;
  erc[0] = vpMath::rad(10); // 10 deg
  erc[1] = vpMath::rad(-10); // -10 deg
  erc[2] = vpMath::rad(25); // 25 deg

  eMc.buildFrom(etc, erc);
  ROS_INFO("1) GROUND TRUTH:");

  ROS_INFO_STREAM("hand to eye transformation: " <<std::endl<<visp_bridge::toGeometryMsgsTransform(eMc)<<std::endl);

  vpColVector v_c(6); // camera velocity used to produce 6 simulated poses
  for (int i = 0; i < N; i++)
  {
    v_c = 0;
    if (i == 0)
    {
      // Initialize first poses
      cMo.buildFrom(0, 0, 0.5, 0, 0, 0); // z=0.5 m
      wMe.buildFrom(0, 0, 0, 0, 0, 0); // Id
    }
    else if (i == 1)
      v_c[3] = M_PI / 8;
    else if (i == 2)
      v_c[4] = M_PI / 8;
    else if (i == 3)
      v_c[5] = M_PI / 10;
    else if (i == 4)
      v_c[0] = 0.5;
    else if (i == 5)
      v_c[1] = 0.8;

    vpHomogeneousMatrix cMc; // camera displacement
    cMc = vpExponentialMap::direct(v_c); // Compute the camera displacement due to the velocity applied to the camera
    if (i > 0)
    {
      // From the camera displacement cMc, compute the wMe and cMo matrixes
      cMo = cMc.inverse() * cMo;
      wMe = wMe * eMc * cMc * eMc.inverse();

    }

    geometry_msgs::Transform pose_c_o;
    pose_c_o = visp_bridge::toGeometryMsgsTransform(cMo);
    geometry_msgs::Transform pose_w_e;
    pose_w_e = visp_bridge::toGeometryMsgsTransform(wMe);
    camera_object_publisher_.publish(pose_c_o);
    world_effector_publisher_.publish(pose_w_e);
    emc_quick_comm.request.camera_object.transforms.push_back(pose_c_o);
    emc_quick_comm.request.world_effector.transforms.push_back(pose_w_e);

  }
  ros::Duration(1.).sleep();

}


void Client::Simulator(){

  ROS_INFO("Waiting for topics...");
  ros::Duration(1.).sleep();
  while(!reset_service_.call(reset_comm)){
    if(!ros::ok()) return;
    ros::Duration(1).sleep();
  }

  for (int i = 0; i < counter; i++){


      geometry_msgs::Transform pose_c_o;
      geometry_msgs::Transform pose_w_e;

      pose_c_o.translation.x = camera_object[counter][0];
      pose_c_o.translation.y = camera_object[counter][1];
      pose_c_o.translation.z = camera_object[counter][2];

      pose_w_e.translation.x = world_effector[counter][0];
      pose_w_e.translation.y = world_effector[counter][1];
      pose_w_e.translation.z = world_effector[counter][2];

      camera_object_publisher_.publish(pose_c_o);
      world_effector_publisher_.publish(pose_w_e);

      cout << "SIMOUT" << endl;

  }


}


void Client::Publisher(){

    ROS_INFO("Waiting for topics...");
    ros::Duration(1.).sleep();
    while(!reset_service_.call(reset_comm)){
      if(!ros::ok()) return;
      ros::Duration(1).sleep();
    }


    ros::Duration(30).sleep();

}

void Client::computeUsingQuickService()
{
  vpHomogeneousMatrix eMc;
  vpThetaUVector erc;
  ROS_INFO("2) QUICK SERVICE:");
  if (compute_effector_camera_quick_service_.call(emc_quick_comm))
  {
    ROS_INFO_STREAM("hand_camera: "<< std::endl << emc_quick_comm.response.effector_camera);
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }
}

void Client::computeFromTopicStream()
{
  vpHomogeneousMatrix eMc;
  vpThetaUVector erc;
  ROS_INFO("3) TOPIC STREAM:");
  if (compute_effector_camera_service_.call(emc_comm))
  {
    ROS_INFO_STREAM("hand_camera: " << std::endl << emc_comm.response.effector_camera);
  }
  else
  {
    ROS_ERROR("Failed to call service");
  }

}
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
