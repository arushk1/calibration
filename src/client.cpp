#include "./../include/client.h"
#include <geometry_msgs/Transform.h>
#include "visp_hand2eye_calibration/TransformArray.h"
#include <visp_bridge/3dpose.h>
#include "./../include/names.h"

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
  camera_object_publisher_ = n_.advertise<geometry_msgs::Transform> (visp_hand2eye_calibration::camera_object_topic, 1);

  world_effector_publisher_ = n_.advertise<geometry_msgs::Transform> (visp_hand2eye_calibration::world_effector_topic, 1);

  reset_service_ = n_.serviceClient<visp_hand2eye_calibration::reset> (visp_hand2eye_calibration::reset_service);

  compute_effector_camera_service_
          = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera> (visp_hand2eye_calibration::compute_effector_camera_service);

  compute_effector_camera_quick_service_
          = n_.serviceClient<visp_hand2eye_calibration::compute_effector_camera_quick> (visp_hand2eye_calibration::compute_effector_camera_quick_service);
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
  vpTranslationVector etc(0.15, 0.2, 0.3);
  vpThetaUVector erc;
  erc[0] = vpMath::rad(10); // 10 deg
  erc[1] = vpMath::rad(-50); // -10 deg
  erc[2] = vpMath::rad(25); // 25 deg

  eMc.buildFrom(etc, erc);
  ROS_INFO("1) GROUND TRUTH:");

  ROS_INFO_STREAM("hand to eye transformation: " <<std::endl<<visp_bridge::toGeometryMsgsTransform(eMc)<<std::endl);

  vpColVector v_c(6); // camera velocity used to produce 6 simulated poses
  for (int i = 0; i < N; i++) // MM: replace for with subscription to your extracted camera_object pose (cMo)
                              //     add counter and trigger 'computeFromTopicStream()' after x received poses
  {
    v_c = 0;
    if (i == 0)
    {
      // Initialize first poses
      cMo.buildFrom(0, 0, 0.4, 0, 0, 0); // z=0.5 m
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
      ROS_INFO_STREAM("old cMo: " <<  std::endl<<visp_bridge::toGeometryMsgsTransform(cMo)<<std::endl);
      // From the camera displacement cMc, compute the wMe and cMo matrixes
      cMo = cMc.inverse() * cMo;
      ROS_INFO_STREAM("new cMo: " <<  std::endl<<visp_bridge::toGeometryMsgsTransform(cMo)<<std::endl);
      ROS_INFO_STREAM("cMc:     " <<  std::endl<<visp_bridge::toGeometryMsgsTransform(cMc)<<std::endl);
      wMe = wMe * eMc * cMc * eMc.inverse();

    }

    geometry_msgs::Transform pose_c_o;
    pose_c_o = visp_bridge::toGeometryMsgsTransform(cMo);
    geometry_msgs::Transform pose_w_e;
    pose_w_e = visp_bridge::toGeometryMsgsTransform(wMe);
    camera_object_publisher_.publish(pose_c_o);
    world_effector_publisher_.publish(pose_w_e);
    //emc_quick_comm.request.camera_object.transforms.push_back(pose_c_o);
    //emc_quick_comm.request.world_effector.transforms.push_back(pose_w_e);

  }
  ros::Duration(1.).sleep();

}

// { MM
void Client::cMoCb(const geometry_msgs::Transform& msg)
{

    ROS_INFO("cMoCb");
    cMo = visp_bridge::toVispHomogeneousMatrix(msg);
    wMe = wMo * cMo.inverse() * eMc.inverse();

    geometry_msgs::Transform pose_c_o;
    pose_c_o = visp_bridge::toGeometryMsgsTransform(cMo);
    geometry_msgs::Transform pose_w_e;
    pose_w_e = visp_bridge::toGeometryMsgsTransform(wMe);
    camera_object_publisher_.publish(pose_c_o);
    world_effector_publisher_.publish(pose_w_e);
    //emc_quick_comm.request.camera_object.transforms.push_back(pose_c_o);
    //emc_quick_comm.request.world_effector.transforms.push_back(pose_w_e);

    count_++;
    if (count_ == 6)
    {
        // unsubscribe cb
        sub_cMo_.shutdown();
        // start calibration
        this->computeFromTopicStream();
    }
}

void Client::simulateCameraMeasurements()
{
    pub_camera_object_ = n_.advertise<geometry_msgs::Transform> (visp_hand2eye_calibration::camera_object_sim_topic, 1);
    vpHomogeneousMatrix cMo_sim;
    vpColVector v_c(6); // camera velocity used to produce 6 simulated poses
    for (int i = 0; i < 6; i++)
    {
      v_c = 0;
      if (i == 0)
      {
        // Initialize first poses
        cMo_sim.buildFrom(0, 0, 0.5, 0, 0, 0); // z=0.5 m
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
        // From the camera displacement cMc, compute the cMo matrixes
        cMo_sim = cMc.inverse() * cMo_sim;
      }

      geometry_msgs::Transform pose_c_o;
      pose_c_o = visp_bridge::toGeometryMsgsTransform(cMo_sim);
      pub_camera_object_.publish(pose_c_o);
      ros::Duration(1.0).sleep();
      ros::spinOnce();
    }

}

void Client::initAndSimulateUsingCam()
{
  ROS_INFO("Waiting for topics...");
  ros::Duration(1.).sleep();
  while(!reset_service_.call(reset_comm)){
    if(!ros::ok()) return;
    ros::Duration(1).sleep();
  }


  // We want to calibrate the hand to eye extrinsic camera parameters from 6 couple of poses: cMo and wMe
  count_ = 0;
  // Input: six couple of poses used as input in the calibration proces

  // Initialize an eMc transformation used to produce the simulated input transformations cMo and wMe
  vpTranslationVector etc(0.15, 0.2, 0.3);
  vpThetaUVector erc;
  erc[0] = vpMath::rad(10); // 10 deg
  erc[1] = vpMath::rad(-20); // -10 deg
  erc[2] = vpMath::rad(25); // 25 deg

  eMc.buildFrom(etc, erc);
  ROS_INFO("1) GROUND TRUTH:");

  ROS_INFO_STREAM("hand to eye transformation: " <<std::endl<<visp_bridge::toGeometryMsgsTransform(eMc)<<std::endl);

  // Initialize an wMo transformation used to produce the simulated input transformations cMo and wMe
  vpTranslationVector wto(1.0, 2.0, 3.0);
  vpThetaUVector wro;
  wro[0] = vpMath::rad(0); // 10 deg
  wro[1] = vpMath::rad(0); // -10 deg
  wro[2] = vpMath::rad(0); // 25 deg

  wMo.buildFrom(wto, wro);

  ROS_INFO_STREAM("world to object transformation: " <<std::endl<<visp_bridge::toGeometryMsgsTransform(wMo)<<std::endl);

  // subscribe to camera image
  sub_cMo_ = n_.subscribe(visp_hand2eye_calibration::camera_object_sim_topic,1,&Client::cMoCb,this);

  // simulate camera transforms
  simulateCameraMeasurements();

}

void Client::Publisher(){

    ROS_INFO("Waiting for topics...");
    ros::Duration(1.).sleep();
    while(!reset_service_.call(reset_comm)){
      if(!ros::ok()) return;
      ros::Duration(1).sleep();
    }


    ros::Duration(60).sleep();

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
