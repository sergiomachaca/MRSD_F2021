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
    world_corner.push_back(Point2f(0.08707,-0.53616));
    world_corner.push_back(Point2f(0.01431,-0.48015));
    world_corner.push_back(Point2f(0.02569,-0.42653));
    world_corner.push_back(Point2f(0.08691,-0.47915));
    world_corner.push_back(Point2f(0.08514,-0.50344));


 

    // Four corners in image coordinates
    vector < Point2f > camera_corner;
    // TODO create camera_corner
    camera_corner.push_back(Point2f(755,132));
    camera_corner.push_back(Point2f(1031,297));
    camera_corner.push_back(Point2f(995,438));
    camera_corner.push_back(Point2f(782,322));
    camera_corner.push_back(Point2f(776,243));

    // Calculate Homography
    Mat h = findHomography(camera_corner, world_corner, RANSAC);


    // use the homography to convert the camera location to the actual locations
    vector < Point2f > point_camera;
    vector < Point2f > point_real;
    point_camera.push_back(Point2f(863, 276));
    perspectiveTransform(point_camera, point_real, h);

  
    // open file 
    cv::FileStorage file("homography.yml", cv::FileStorage::WRITE);
    // Write to file!
    file << "homography" << h;
    file << "point" << point_real;

    
  
    return 0;

}

