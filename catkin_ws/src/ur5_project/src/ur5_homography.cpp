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
    world_corner.push_back(Point2f(0.08651,-0.53585));
    world_corner.push_back(Point2f(0.0145,-0.47061));
    world_corner.push_back(Point2f(0.02481,-0.42305));
    world_corner.push_back(Point2f(0.09136,-0.48382));
   

 

    // Four corners in image coordinates
    vector < Point2f > camera_corner;
    // TODO create camera_corner
    camera_corner.push_back(Point2f(744,130));
    camera_corner.push_back(Point2f(1018,294));
    camera_corner.push_back(Point2f(985,437));
    camera_corner.push_back(Point2f(761,301));


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

