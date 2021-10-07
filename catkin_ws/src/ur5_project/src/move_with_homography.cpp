#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "opencv2/imgcodecs.hpp"

#include <iostream>

#include "opencv2/opencv.hpp"

#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace std;

// TODO: define your initial z, roll, pitch, and yaw values 
double initial_Z = .4203;
double initial_roll = -3.127;
double initial_pitch = 0;
double initial_yaw = 0;

int main(int argc, char ** argv) {
    
    ros::init(argc, argv, "joint");
    ros::NodeHandle nh_;
    ros::NodeHandle home("~");

    // setting up the loop frequency
      int loop_freq = 10;
      float dt = (float) 1 / loop_freq;
      ros::Rate loop_rate(loop_freq);

    // TODO: add a publisher 'point_real_pub' to publish the geometry_msgs::Twist and you will add subscriber to subscribe the this message in your ik.cpp
    // You can define the names of the topic with your choise, as long as they are consistent in ik.cpp 
    ros::Publisher point_real_pub = nh_.advertise <geometry_msgs::Twist> ("real_points", 1);

    // TODO: copy and paste your homography
    // Four corners in the real coordinates
    vector < Point2f > world_corner;
    world_corner.push_back(Point2f(.0306,-.40284));
    world_corner.push_back(Point2f(0.0993,-.44593 ));
    world_corner.push_back(Point2f(0.05231,-.48525  ));
    world_corner.push_back(Point2f( .12669,-.49073  ));
    world_corner.push_back(Point2f(  .05948,-.444  ));
    
    // Four corners in image coordinates
    vector < Point2f > camera_corner;
    camera_corner.push_back(Point2f(924,537));
    camera_corner.push_back(Point2f(686,443));
    camera_corner.push_back(Point2f(841,335 ));
    camera_corner.push_back(Point2f(612,318));
    camera_corner.push_back(Point2f(809,444 ));
  
    // Calculate Homography
    Mat h = findHomography(camera_corner, world_corner, cv::RANSAC);


    // TODO: define pixel coordinates of the marked point
    std::vector < Point2f > point_camera;
    point_camera.push_back(Point2f(663, 385));

    std::vector < Point2f > point_real;    
    perspectiveTransform(point_camera, point_real, h);

    geometry_msgs::Twist robot_coor;
    //TODO: construct robot_coor with point_real and init_z, roll, pitch and yaw
    robot_coor.linear.x = point_real[0].x;
    robot_coor.linear.y = point_real[0].y;
    robot_coor.linear.z = initial_Z;
    robot_coor.angular.x = initial_roll;
    robot_coor.angular.y = initial_pitch;
    robot_coor.angular.z = initial_yaw;

    // cout << "linear x: " << robot_coor.linear.x << endl;
    // cout << "linear y: " << robot_coor.linear.x << endl;

    while (ros::ok()) {
            
            point_real_pub.publish(robot_coor); // TODO uncomment this line after you created point_real_pub

            loop_rate.sleep();
            ros::spinOnce();
    }
    return 0;

}









