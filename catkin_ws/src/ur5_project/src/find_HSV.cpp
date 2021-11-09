#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Image window";

int thresh = 100;
Mat src_gray;
int countour_detection = 1; // 1 if countour detection; 0 otherwise
int laserFollow = 1;        // 1 if laser follow; 0 otherwise

int iLowH = 75;
int iHighH = 96;

int iLowS = 75;
int iHighS = 157;

int iLowV = 114;
int iHighV = 240;

int iLastX = 1;
int iLastY = 1;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
      : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/color/image_raw", 1,
                               &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), 0.5, 0.5); // resize image

    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    //Create trackbars in "Control" window to find the best hsw for laser following
    createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    createTrackbar("HighH", "Control", &iHighH, 179);

    createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    createTrackbar("HighS", "Control", &iHighS, 255);

    createTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    createTrackbar("HighV", "Control", &iHighV, 255);

    Mat imgHSV;
    cvtColor(cv_ptr->image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    Mat imgThresholded;
    Mat img_gray;
    Mat img_canny;
    // inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    cvtColor(cv_ptr->image, img_gray, CV_RGB2GRAY);
    int scalar_low = 25;
    int scalar_high = 75;
    inRange(img_gray, cv::Scalar(scalar_low), cv::Scalar(scalar_high), img_gray);
    Canny(img_gray, img_canny, iLowS, iHighS);

    //morphological opening (removes small objects from the foreground)
    // erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    // dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    // //morphological closing (removes small holes from the foreground)
    // dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    // erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));


    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, img_canny);
    cv::waitKey(3);

    // // Output modified video stream
    // image_pub_.publish(cv_ptr->toImageMsg());

    //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    if (waitKey(30) == 27)
    {
      cout << "esc key is pressed by user" << endl;
      ros::shutdown();
      cv::destroyWindow(OPENCV_WINDOW);
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cv_node");
  ImageConverter ic;
  ros::spin();
  return 0;
}
