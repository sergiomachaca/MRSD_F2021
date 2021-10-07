#include <ros/ros.h>

#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>

#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "opencv2/imgcodecs.hpp"

#include <iostream>

#include "opencv2/opencv.hpp"

#include <trajectory_msgs/JointTrajectory.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

#include <ros/package.h>

#include <math.h>

using namespace cv;
using namespace std;
using namespace cv_bridge;



Mat h;


CvImagePtr cv_ptr;


int thresh = 100;
Mat src_gray;


Mat img_raw;
int width, height;
bool initialized;
bool track_detected = false;
int track_ctr = 0;

bool plan_created = false;
bool plan_write = false;

vector < Point2f > final_way_points;

// TODO: define your initial z, roll, pitch, and yaw values 

double initial_Z = .47965;
double initial_roll = -3.1;
double initial_pitch = 0.0;
double initial_yaw = 0.0;



void get_img(const sensor_msgs::Image & _data) {
    //read the rectified rgb input image from camera
    cv_ptr = toCvCopy(_data, "rgb8");
    // take the image component and put it in a Mat opencv variable
    img_raw = cv_ptr -> image;
    // get the width and height
    width = _data.width;
    height = _data.height;

    initialized = true;
}

// find the distance between to XY points
double distance(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return (sqrt(dx * dx + dy * dy));
}

// TODO: define ROI values here pixel 
int rect_x = 700;
int rect_y = 328;
int rect_width = 250;
int rect_height = 200;

//TODO: determine if pt is inside rect and return boolean. 
bool ptInRect(cv::Point pt, cv::Rect rect) {
    
    return rect.contains(pt);
}


int main(int argc, char * argv[]) {

    ros::init(argc, argv, "ur5_homography");
    ros::NodeHandle nh_;
    ros::NodeHandle home("~");
    // cout<< "running 1"<<endl;

    //TODO: the contour_detection and laserFollow are all false right now, change these values 
    // while you are at the corresponding step
    bool contour_detection = true;
    bool laserFollow = true;

    home.getParam("contour_detection", contour_detection);
    home.getParam("laserFollow", laserFollow);
    cout << "contour detection = " << contour_detection << endl;

    int loop_freq = 5;
    ros::Rate loop_rate(loop_freq);

    ros::Publisher img_pub = nh_.advertise < sensor_msgs::Image > ("homography", 1);

    ros::Subscriber cam_sub = nh_.subscribe("camera/color/image_raw", 1, get_img);

    // publisher for sending the point plans to the trajectory generator of the robot
    ros::Publisher plan_pub = nh_.advertise < trajectory_msgs::JointTrajectory > ("/plan", 1);

    // TODO: copy and paste your homography here
    //Four corners in the real coordinates
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
    h = findHomography(camera_corner, world_corner, cv::RANSAC);

    int npt = 30; //  number of points sent to the robot 

    Mat img_crop;

    vector < vector < Point > > contours;
    // contours inside the region of interest
    vector < vector < Point > > contour_path;
    // variable used for contour detection
    vector < Vec4i > hierarchy;

    //Define region of interest
    Rect roi;
    // cout << "running 2" << endl;

    while (ros::ok) {
        if (initialized && !plan_created) {

            img_raw = cv_ptr -> image;

            // TODO: define region of interest with rect_x, y, width, and heigh defined above, in this case, we will use a rectangle. You can choose how large your roi is
            // It should be larger than the shape on the cardboard
            
            roi.x = rect_x;
            roi.y = rect_y;
            roi.width = rect_width;
            roi.height = rect_height;

            Mat img_gray;
            Mat hsv1, g0, g1;
            // convert the input image to grayscale
            cvtColor(img_raw, img_gray, CV_RGB2GRAY);
            int scalar_low = 25;
            int scalar_high = 65;
            inRange(img_gray, cv::Scalar(scalar_low), cv::Scalar(scalar_high), img_gray);
            
            // namedWindow("image window", CV_WINDOW_AUTOSIZE); //create a window called "Control"
            // imshow("image window", img_gray);
            // waitKey(0);
            // destroyAllWindows();

            // cout << "running 3" << endl;

            // previous waypoints sent to the robot (used for filtering purposes)
            vector < Point2f > prev_way_points;
            // waypoints sent to the robot
            vector < Point2f > way_points;

            // detect the first contour that generated 
            if (contour_detection) {
                Mat img_canny;
                if (!track_detected) {
                    track_detected = true;
                    // TODO: define thresh_low and thresh_high so that the contour is detected (0-255)
                    int thresh_low = 55;
                    int thresh_high = 3*thresh_low;

                    //  Find edges 
                    Canny(img_gray, img_canny, thresh_low, thresh_high);
                    dilate(img_canny, img_canny, getStructuringElement(MORPH_ELLIPSE, Size(12, 12)));
                    erode(img_canny, img_canny, getStructuringElement(MORPH_ELLIPSE, Size(8, 8)));

                    // namedWindow("image window", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                    // imshow("image window", img_canny);
                    // waitKey(0);
                    // destroyAllWindows();
                    // // //  Find contours
                    findContours(img_canny, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));
                    // cout << "num countours = " << contours.size() << endl;
                    
                    

                } else {
                    plan_created = true;
                }
                contour_path.clear();
                bool waypoint_found = false;
                
                for (int i = 0; i < contours.size(); i++) {
                    bool in_region = true;
                    
                    // TODO: change contour_detection in your launch file to be true and determine if all the points on the countour is inside the roi 
                    // the variable contours contains numerous of individual contours and each of the contour have numerous points. 
                    // You need to determine if each individual points is inside the roi
                    
                    // double max_len = 0;
                    vector <Point> contour = contours[i];
                    // if (cv::arcLength(contour, true) > max_len){
                    //     max_len = cv::arcLength(contour, true);
                    // }        
                    for (Point point : contour){
                        if (!ptInRect(point, roi)){
                            in_region = false;
                        }
                    }
                    // cout << "in region" << in_region << endl;
                    // if the path is in the region of interest 
                    if (in_region) {
                        // // add the contours in the region of interest to the final contour 
                        // drawContours(img_raw, contours, i, cv::Scalar(0,255,0), 2);
                        // namedWindow("image window", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                        // imshow("image window", img_raw);
                        // waitKey(0);
                        // destroyAllWindows();
                        contour_path.push_back(contours[i]);
                        // use the first contour path 
                        
                    }
                }
                vector <Point> longest_contour;
                double max_cont_len = 0;
                for (vector <Point> c : contour_path){
                    if (arcLength(c, true) > max_cont_len){
                        max_cont_len = arcLength(c, true);
                        longest_contour = c;
                    }
                }
                
                // drawContours(img_raw, vector<vector<Point> >(1,longest_contour), -1, cv::Scalar(0,255,0), 2);
                // namedWindow("image window", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                // imshow("image window", img_raw);
                // waitKey(0);
                // destroyAllWindows();
                // int index = 0;
                // if (contour_path.size() == index+1) {
                    // TODO: calculate the length of the contour in pixels, don't forget the distance between the endpoint and the start point
                    // The total length will be restore in cont_len 
                double cont_len = 0;
                int cont_size = longest_contour.size();
                // cout << "cont size = " << cont_size << endl;
                for (int k = 0; k < cont_size; k++){
                    if (k == cont_size-1){
                        cont_len += distance(longest_contour[k].x, longest_contour[k].y, longest_contour[0].x, longest_contour[0].y);
                    }
                    cont_len += distance(longest_contour[k+1].x, longest_contour[k+1].y, longest_contour[k].x, longest_contour[k].y);
                }
                
                    // cout << "cont len = " << cont_len << endl;

                Point pt;
                double seg_len = 0; //segment distance
                double seg_len_prev = 0; //previous segment distance
                way_points.clear();

                pt = longest_contour[0];
                // add the first point to the waypoint list 
                way_points.push_back(pt);
                // go through the points in the contour and generate waypoints that are equal distance to each other
                for (int k = 0; k < cont_size - 1; k++) {
                    // calculate the length of a segment on the contour
                    seg_len += distance(longest_contour[k].x, longest_contour[k].y, longest_contour[k + 1].x, longest_contour[k + 1].y);
                    // check if it is close to the average length (total_length/number_of_waypoints)
                    if ((seg_len_prev < cont_len / npt) && (seg_len > cont_len / npt)) {
                        // add this point to the waypoint list
                        pt = longest_contour[k];
                        way_points.push_back(pt);
                        // reset the length			
                        seg_len = 0;

                    }
                    seg_len_prev = seg_len;
                }
                // cout << "waypoint size = " << way_points.size() << endl;
                    
                prev_way_points = way_points;
            
                if (plan_created) {
                    final_way_points = way_points;
                }

            }
            // cout << "running 4" << endl;

            if (plan_created) { // once the plan is created, publish /plan 
                // cout << "plan created 1" << endl;
                // way points in the robot coordinates 
                std::vector < Point2f > real_way_points(way_points.size());
                // cout << way_points << endl;
                // cout << real_way_points << endl;
                // cout << h << endl;

                perspectiveTransform(way_points, real_way_points, h);
                // cout << "post perspective" << endl;
                trajectory_msgs::JointTrajectory plan;
                cout << way_points.size()  << endl;
                for (int k = 0; k < way_points.size(); k++) {
                    trajectory_msgs::JointTrajectoryPoint plan_pt;
                    plan_pt.positions.push_back(real_way_points[k].x);
                    plan_pt.positions.push_back(real_way_points[k].y);
                    plan_pt.positions.push_back(initial_Z);
                    plan_pt.positions.push_back(initial_roll);
                    plan_pt.positions.push_back(initial_pitch);
                    plan_pt.positions.push_back(initial_yaw);

                    for (int ind = 0; ind < 6; ind++) {
                        plan_pt.velocities.push_back(0);
                        plan_pt.accelerations.push_back(0);
                    }
                    plan.points.push_back(plan_pt);

                }
                cout << "plan created = " << plan_created << endl;
                cout << "plan size = " << plan.points[0] << endl;
                plan_pub.publish(plan);

            }
            // cout << "running 5" << endl;

            // draw the roi on the image 
            rectangle(img_raw, roi, cv::Scalar(255, 255, 0), 1, 8, 0);

            // image publisher
            cv_ptr -> image = img_raw;
            img_pub.publish(cv_ptr -> toImageMsg());
            // cout << "published" << endl;

        }
        if (plan_created) {
                // cout << "plan created" << endl;

            if (laserFollow) {
                // ******************************************Laser Pointer Follower****************************************************
                // cout << "laser follower" << endl;
                // TODO: change laserFollow in your launch file to true and define the following thresholds to finish the laser pointer follower
                // You can make contour_detection to be false for now
                int iLowH = 0;
                int iHighH = 179;

                int iLowS = 0;
                int iHighS = 255;

                int iLowV = 245;
                int iHighV = 255;

                int iH = 1; 
                int iS = 10; 

                Mat img_canny;
                Mat imgHSV;
                cvtColor(cv_ptr -> image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

                Mat imgThresholded;
                inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

                //morphological opening (removes small objects from the foreground)
                erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(iH, iH)));
                dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(iH, iH)));

                // //morphological closing (removes small holes from the foreground)
                dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(iS, iS)));
                erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(iS, iS)));

                // namedWindow("image window", CV_WINDOW_AUTOSIZE); //create a window called "Control"
                // rectangle(imgThresholded, roi, cv::Scalar(255, 255, 0), 1, 8, 0);
                // imshow("image window", imgThresholded);
                // waitKey(0);
                // destroyAllWindows();

                // find moments of the image, in order to find the center of the laser pointer 
                Moments m = moments(imgThresholded, true);
                Point p(m.m10 / m.m00, m.m01 / m.m00);

                // if the center of the laser pointer is inside roi, then draw a circle on the graph and calculate the error between 
                // the closest point on the contour
                if (ptInRect(p, roi)) {
                    // draw the center of the laser pointer 
                    // cout << p << endl;
                    circle(img_raw, p, 5, Scalar(0, 0, 255), FILLED, 8, 0);
                    int ind_min = -1; // index of the point on the contour that is closest to the laser pointer 
                    double x_error, y_error, dist_min = 999.9; // value of the distance between the laser point and the closest point the contour in x, and y direction
                    for (int i = 0; i < contour_path[0].size(); i++) {
                        double current_dist = distance(p.x, p.y, contour_path[0][i].x, contour_path[0][i].y);
                        if (current_dist < dist_min) {
                            ind_min = i;
                            dist_min = current_dist;
                        }
                    }

                }
            }
            // cout << "running 6" << endl;

            //*****************************************End of Laser pointer follower**********************************************

            // draw the way points of the image 
            int thickness;
            // cout << final_way_points.size() << endl;
            for (int i = 0; i < npt; i++) {
                if (i == 0) {
                    thickness = FILLED;

                } else {
                    thickness = 1;
                }
                circle(img_raw, final_way_points[i], 3, Scalar(255, 255, 0), thickness, 8, 0);
                
            }
            rectangle(img_raw, roi, cv::Scalar(255, 255, 0), 1, 8, 0);
            cv_ptr -> image = img_raw;
            img_pub.publish(cv_ptr -> toImageMsg());

        }
        // cout << "end of code" << endl;
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}