#include "./../include/client.h"
#include "ros/ros.h"

using namespace std;

int main(int argc,char**argv){
  ros::init(argc, argv, "client");

  visp_hand2eye_calibration::Client ct;

  //ct.initAndSimulateUsingCam();
  // ct.Simulator();

  ct.Publisher();

  //ct.computeUsingQuickService();
  ct.computeFromTopicStream();

  return 0 ;
}
