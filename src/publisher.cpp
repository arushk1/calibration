#include "./../include/publisher.h"


using namespace cv;
using namespace std;
using namespace tf;
using namespace message_filters;
using namespace sensor_msgs;
using namespace geometry_msgs;

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

    for (int i=0; i<8; i++)
        {
            for (int j=0; j<6; j++)
            {
                boardPoints.push_back( Point3d( double(i), double(j), 0.0) );
            }
        }

    //generate points in the reference frame
    framePoints.push_back( Point3d( 0.0, 0.0, 0.0 ) );
    framePoints.push_back( Point3d( 5.0, 0.0, 0.0 ) );
    framePoints.push_back( Point3d( 0.0, 5.0, 0.0 ) );
    framePoints.push_back( Point3d( 0.0, 0.0, 5.0 ) );


    image_sub_ = it_.subscribe("/camera/rgb/image_rect_color", 10, &Publisher::imageCb, this);
    image_pub_ = it_.advertise("/calibration/output", 10);

    camera_object_publisher_ = nh_.advertise<geometry_msgs::Transform>("/camera_object", 10);
    world_effector_publisher_ = nh_.advertise<geometry_msgs::Transform>("/world_effector", 10);

    drone_pose_subscriber_ = nh_.subscribe("/optitrack/rgbd_sensor/pose", 10, &Publisher::PoseCb, this);

    message_filters::Subscriber<Image> image_sub(nh_, "image", 1);
    message_filters::Subscriber<PoseStamped> pose_sub(nh_, "pose", 1);

//    TimeSynchronizer<sensor_msgs::Image, PoseStamped> sync(image_sub, pose_sub, 10);
//    sync.registerCallback(boost::bind(&Publisher::image_pose_callback, this, _1, _2));


    cv::namedWindow(OPENCV_WINDOW);
}


void Publisher::imageCb(const sensor_msgs::ImageConstPtr& msg)
{



    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    webcamImage = cv_ptr->image;

    cvtColor(webcamImage,gray,COLOR_BGR2GRAY);

    bool found = findChessboardCorners( gray, cvSize(8,6), imagePoints, CV_CALIB_CB_ADAPTIVE_THRESH);

    if (found){

        // TODO: AK Error in the format of one of the arguments
        // Double Free when node starts and then you show the checkerboard
        solvePnP (Mat(boardPoints), Mat(imagePoints), intrinsics, distortion, rvec, tvec, false);

        projectPoints(framePoints, rvec, tvec, intrinsics, distortion, imageFramePoints );

        //DRAWING
        //draw the reference frame on the image
        circle(webcamImage, (cv::Point2d) imagePoints[0], 4 ,CV_RGB(255,0,0) );

        cv::Point one, two, three;
        one.x=10; one.y=10;
        two.x = 60; two.y = 10;
        three.x = 10; three.y = 60;

        line(webcamImage, one, two, CV_RGB(255,0,0) );
        line(webcamImage, one, three, CV_RGB(0,255,0) );


        line(webcamImage, imageFramePoints[0], imageFramePoints[1], CV_RGB(255,0,0), 2 );
        line(webcamImage, imageFramePoints[0], imageFramePoints[2], CV_RGB(0,255,0), 2 );
        line(webcamImage, imageFramePoints[0], imageFramePoints[3], CV_RGB(0,0,255), 2 );


        cout << fixed << setprecision(2) << "rvec = ["
             << rvec.at<double>(0,0) << ", "
             << rvec.at<double>(1,0) << ", "
             << rvec.at<double>(2,0) << "] \t" << "tvec = ["
             << tvec.at<double>(0,0) << ", "
             << tvec.at<double>(1,0) << ", "
             << tvec.at<double>(2,0) << "]" << endl;


    }
/*
    c2o.translation.x = tvec.at<double>(0,2);
    c2o.translation.y = -tvec.at<double>(0,0);
    c2o.translation.z = -tvec.at<double>(0,1);

    q.setEuler(rvec.at<double>(0,2), rvec.at<double>(0,1), rvec.at<double>(0,0));

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

    camera_object_publisher_.publish(invc2o);
*/
    cv::imshow(OPENCV_WINDOW, webcamImage);
    cv::waitKey(3);

    //image_pub_.publish(cv_ptr->toImageMsg());
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


void Publisher::image_pose_callback(const sensor_msgs::ImageConstPtr& img_msg, const geometry_msgs::PoseStampedConstPtr &pose_msg)
{/*

    vector<Point2f> imagePoints;
    vector<Point3f> boardPoints;
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
        cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    bool found = findChessboardCorners( cv_ptr->image, cvSize(8,6), imagePoints, CV_CALIB_CB_ADAPTIVE_THRESH);

    if (found){

        solvePnP (boardPoints, imagePoints, intrinsics, distortion, rvec, tvec, false);

        cout << fixed << setprecision(2) << "rvec = ["
             << rvec.at<double>(0,0)*180/3.14 << ", "
             << rvec.at<double>(1,0)*180/3.14 << ", "
             << rvec.at<double>(2,0)*180/3.14 << "] \t" << "tvec = ["
             << tvec.at<double>(0,0) << ", "
             << tvec.at<double>(1,0) << ", "
             << tvec.at<double>(2,0) << "]" << endl;

    }

    c2o.translation.x = tvec.at<double>(0,2);
    c2o.translation.y = -tvec.at<double>(0,0);
    c2o.translation.z = -tvec.at<double>(0,1);

    q.setEuler(rvec.at<double>(0,2), rvec.at<double>(0,1), rvec.at<double>(0,0));

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

    w2d.translation.x = pose_msg->pose.position.x;
    w2d.translation.y = pose_msg->pose.position.y;
    w2d.translation.z = pose_msg->pose.position.z;

    w2d.rotation.w = pose_msg->pose.orientation.w;
    w2d.rotation.x = pose_msg->pose.orientation.x;
    w2d.rotation.y = pose_msg->pose.orientation.y;
    w2d.rotation.z = pose_msg->pose.orientation.z;

    camera_object_publisher_.publish(invc2o);
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    world_effector_publisher_.publish(w2d);
    image_pub_.publish(cv_ptr->toImageMsg());
*/
}
