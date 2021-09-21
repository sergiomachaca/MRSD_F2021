#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "opencv2/imgcodecs.hpp"

#include <iostream>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;


int main(int argc, char ** argv) {
    

    // Four corners in the real coordinates
    vector < Point2f > world_corner;
    //TODO create world_corner (actual coordinates)
    world_corner.push_back(Point2f(-1.865579907094137,-1.6436007658587855));
    world_corner.push_back(Point2f(-1.8925378958331507,-1.4148615042315882));
    world_corner.push_back(Point2f(-1.7978585402118128,-1.3320182005511683));
    world_corner.push_back(Point2f(-1.8549884001361292,-1.6208670775042933));

 

    // Four corners in image coordinates
    vector < Point2f > camera_corner;
    // TODO create camera_corner
    camera_corner.push_back(Point2f(573,299));
    camera_corner.push_back(Point2f(802,476));
    camera_corner.push_back(Point2f(1029,427));
    camera_corner.push_back(Point2f(804,268));


    // Calculate Homography
    Mat h = findHomography(camera_corner, world_corner, RANSAC);


    // use the homography to convert the camera location to the actual locations
    vector < Point2f > point_camera;
    vector < Point2f > point_real;
    //point_camera.push_back(Point2f(751, 458));
    point_camera.push_back(Point2f(727, 369));
    perspectiveTransform(point_camera, point_real, h);

  
    // open file 
    cv::FileStorage file("homography.yml", cv::FileStorage::WRITE);
    // Write to file!
    file << "homography" << h;
    file << "point" << point_real;

    
  
    return 0;

}

