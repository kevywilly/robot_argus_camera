#include <cstdio>
#include <iostream>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <signal.h>

/***
# oST version 5.0 parameters
[image]

width
1280

height
720

[camera]

camera matrix
647.876881 0.000000 639.902919
0.000000 485.764373 352.857511
0.000000 0.000000 1.000000

distortion
-0.281166 0.057563 0.003871 0.003425 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
452.509735 0.000000 648.408514 0.000000
0.000000 389.708099 355.531485 0.000000
0.000000 0.000000 1.000000 0.000000
 * 
*/

#define UNDISTORT 0

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        647.876881, 0, 639.902919,
        0, 485.764373, 352.857511,
        0, 0, 1.00);

cv::Mat distortionCoefficients = (cv::Mat_<double>(1, 5) << -0.281166, 0.057563, 0.003871, 0.003425, 0.000000);

cv::Mat rectificationMatrix = cv::Mat::eye(3, 3, CV_64F); // Assuming no rectification

cv::Mat projectionMatrix = (cv::Mat_<double>(3, 4) <<
    452.509735, 0, 648.408514, 0,
    0, 389.708099, 355.531485, 0,
    0, 0, 1.0, 0);

class CSICameraPublisher : public rclcpp::Node {
public:
    CSICameraPublisher()
        : Node("camera") {
        
        declare_parameter("num_cameras", 1);

        get_parameter("num_cameras", num_cameras_);

        // Create an image publisher
        left_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("left/image_raw", 10);

        if(num_cameras_ > 1) {
          right_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("right/image_raw", 10);
        }

        // Initialize the camera
        if (!initCamera()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize the camera.");
            rclcpp::shutdown();
        }

        // Start capturing and publishing
        captureAndPublish();
    }

  void release() {
    if(camera_0_.isOpened()) {
      camera_0_.release();
    }
    if(num_cameras_ > 1){
      if(camera_0_.isOpened()) {
        camera_1_.release();
      }
    }

  }

private:
    bool initCamera() {
        // Open the camera using OpenCV
        camera_0_ = cv::VideoCapture("nvarguscamerasrc sensor_id=0 sensor_mode=3 ! video/x-raw(memory:NVMM), width=(int)1640, height=(int)1232,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)I420 ! appsink max-buffers=1 drop=true");
        /*
                                      nvarguscamerasrc sensor_id=ID ! video/x-raw(memory:NVMM), width=X, height=Y, format=(string)NV12 ! nvvidconv flip-method=M ! video/x-raw, format=I420, appsink max-buffers=1 drop=true
        */
        if (!camera_0_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
            return false;
        }

        if(num_cameras_ > 1) {
          camera_1_ = cv::VideoCapture("nvarguscamerasrc sensor_id=1 sensor_mode=3 ! video/x-raw(memory:NVMM), width=(int)1640, height=(int)1232,format=(string)NV12, framerate=(fraction)30/1 ! nvvidconv ! video/x-raw, format=(string)I420 ! appsink max-buffers=1 drop=true");

          if (!camera_1_.isOpened()) {
              RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
              return false;
          }
        }
        return true;
    }

    void undistortImage(cv::Mat img, cv::Mat &undistortedImage) {
      cv::undistort(img, undistortedImage, cameraMatrix, distortionCoefficients, rectificationMatrix);
      
    }
    void captureAndPublish() {
        // Main loop to capture and publish images
        while (rclcpp::ok()) {
            cv::Mat frame_0;
            camera_0_ >> frame_0;
            if (!frame_0.empty()) {
                // Convert OpenCV image to ROS2 sensor message
                cv::Mat dst_0;
                cv::cvtColor(frame_0, dst_0, cv::COLOR_YUV2BGR_I420);

                std::shared_ptr<sensor_msgs::msg::Image> img_msg_0;

                if(UNDISTORT) {
                  cv::Mat undistorted;
                  cv::undistort(dst_0, undistorted, cameraMatrix, distortionCoefficients, cv::noArray());
                  img_msg_0 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", undistorted).toImageMsg();
                } else {
                  img_msg_0 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dst_0).toImageMsg();
                }

                // Publish the image
                if(num_cameras_ > 1) {
                  right_image_publisher_->publish(*img_msg_0);
                } else {
                  left_image_publisher_->publish(*img_msg_0);
                }
            }

            if(num_cameras_ > 1) {
              cv::Mat frame_1;
              camera_1_ >> frame_1;
              if (!frame_1.empty()) {
                  // Convert OpenCV image to ROS2 sensor message
                  cv::Mat dst_1;
                  cv::cvtColor(frame_1, dst_1, cv::COLOR_YUV2BGR_I420);
                  std::shared_ptr<sensor_msgs::msg::Image> img_msg_1;
                  
                  if(UNDISTORT) {
                    cv::Mat undistorted_1;
                    cv::undistort(dst_1, undistorted_1, cameraMatrix, distortionCoefficients, cv::noArray());
                    img_msg_1 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", undistorted_1).toImageMsg();
                  } else {
                    img_msg_1 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", dst_1).toImageMsg();
                  }
                    // Publish the image
                    left_image_publisher_->publish(*img_msg_1);
                  
              }
            }

            if(!rclcpp::ok()) {
              release();
            }
            // Sleep for a while to control the publishing rate
            rclcpp::sleep_for(std::chrono::milliseconds(33));
        }
        release();
        
    }

    int num_cameras_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_image_publisher_;
    
    rclcpp::Clock clock_;
    cv::VideoCapture camera_0_;
    cv::VideoCapture camera_1_;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CSICameraPublisher>();
    rclcpp::spin(node);
    node->release();
    rclcpp::shutdown();

    return 0;
}
