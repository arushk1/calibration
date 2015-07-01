#include "./../include/publisher.h"
#include "ros/ros.h"

using namespace std;

int main(int argc,char**argv){
    ros::init(argc, argv, "camera2object");
    Publisher pb;
    ros::spin();
    return 0;
}
