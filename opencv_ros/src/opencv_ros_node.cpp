#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/opencv.hpp"

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

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
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1,
&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
 
    cv::namedWindow(OPENCV_WINDOW);
  }
 
  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }
 
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //Convert the ros message to openCV image
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
 
    // Read image
    Mat gray_image;
    
    // Convert the coloured image to gray image
    cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
 
    // Setup SimpleBlobDetector parameters
    SimpleBlobDetector::Params params;
    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 255;
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0;  
    params.maxCircularity = 1;  
 
    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
 
    // Detect the keypoints from the gray_image
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(gray_image, keypoints);
 
    // Draw detected blobs as red circles on the coloured image
    drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image, cv::Scalar(255, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
 
    // Show blobs
    imshow(OPENCV_WINDOW, cv_ptr->image);
    waitKey(3);
 
    // Publish image
    image_pub_.publish(cv_ptr->toImageMsg());

    if(keypoints.size()){
      std::cout<<"\nCircle detected";
    }
  }
};
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}