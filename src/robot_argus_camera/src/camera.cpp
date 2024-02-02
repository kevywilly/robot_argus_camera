#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <signal.h>

struct SensorMode {
  int id;
  int width;
  int height;
  int framerate;
};

static const SensorMode sensor_modes[6] = {
  {0,3264,2464,21},
  {1,3264,1848,28},
  {2,1928,1080,29},
  {3,1640,1232,29},
  {4,1280,720,59},
  {5,1280,720,120}
};

static std::string sensor_mode_to_nvargus_string(int sensor_id, int mode_id) {
  SensorMode mode = sensor_modes[mode_id];
  std::stringstream ss;
  ss << "nvarguscamerasrc sensor_id=" << sensor_id << " sensor_mode=" << mode.id << " ";
  ss << "! video/x-raw(memory:NVMM), width=(int)" << mode.width << ", ";
  ss << "height=(int)" << mode.height << ", ";
  ss << "format=(string)NV12, framerate=(fraction)" << mode.framerate << "/1 ";
  ss << "! nvvidconv ! video/x-raw, format=(string)I420 ! appsink max-buffers=1 drop=true";
  return ss.str();
}

// "nvarguscamerasrc sensor_id=0 sensor_mode=0 ! video/x-raw(memory:NVMM), width=(int)3264, height=(int)2464, format=(string)NV12, framerate=(fraction)21/1 ! nvvidconv ! video/x-raw, format=(string)I420 ! appsink max-buffers=1 drop=true",


// Camera Matrix
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
  848.721379, 0.000000, 939.509142, 0.000000, 848.967602, 596.153547, 0.000000, 0.000000, 1.000000
);

cv::Mat distortionCoefficients = (cv::Mat_<double>(1, 5) << 
  -0.296850, 0.061372, 0.002562, -0.002645, 0.000000
);

cv::Mat rectificationMatrix = cv::Mat::eye(3, 3, CV_64F); // Assuming no rectification

cv::Mat projectionMatrix = (cv::Mat_<double>(3, 4) <<
  452.509735, 0.000000, 648.408514, 0.000000,
  0.000000, 389.708099, 355.531485, 0.000000,
  0.000000, 0.000000, 1.000000, 0.000000
);

class CSICameraPublisher : public rclcpp::Node {

public:
    CSICameraPublisher(): Node("camera") {
        
        declare_parameter("stereo", false);
	      declare_parameter("undistort",false);
        declare_parameter("sensor_mode",0);
        get_parameter("stereo", stereo_);
        get_parameter("undistort", undistort_);
        get_parameter("sensor_mode", sensor_mode_);

        capture_delay_millis_ = (int)1000/sensor_modes[sensor_mode_].framerate;

        auto topic0 = stereo_ ? "right/image_raw" : "left/image_raw";
        auto topic1 = "right_image_raw";

        // Create an image publisher
        
        camera_0_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic0, 10);

        if(stereo_){
          camera_1_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic1, 10);
        }

        // Initialize the camera

        if (!_initCamera()) {
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
    if(stereo_){
      if(camera_1_.isOpened()) {
        camera_1_.release();
      }
    }

  }

private:

    bool _initCamera() {
        // Open the camera using OpenCV
        camera_0_ = cv::VideoCapture(sensor_mode_to_nvargus_string(0,sensor_mode_));
        if (!camera_0_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
            return false;
        }

        if(stereo_) {
          camera_1_ = cv::VideoCapture(sensor_mode_to_nvargus_string(1,sensor_mode_));

          if (!camera_1_.isOpened()) {
              RCLCPP_ERROR(this->get_logger(), "Failed to open the camera.");
              return false;
          }
        }
        return true;
    }

    bool _capture(cv::VideoCapture cap, cv::Mat &mat) {
      cv::Mat frame;
      cv::Mat tmp;
      cap >> frame;
      if(!frame.empty()) {
        if(undistort_) {
          cv::Mat dst;
          cv::cvtColor(frame, dst, cv::COLOR_YUV2BGR_I420);
          cv::undistort(dst, mat, cameraMatrix, distortionCoefficients, cv::noArray());
        } else {
          cv::cvtColor(frame, mat, cv::COLOR_YUV2BGR_I420);
        }
        
        //cv::resize(tmp,mat, cv::Size(224,244),cv::INTER_AREA);

        return true;
      }
      return false;
    }

    void captureAndPublish() {
        // Main loop to capture and publish images
        while (rclcpp::ok()) {
            cv::Mat img_0;
            cv::Mat img_1;

            bool got0 = _capture(camera_0_, img_0);
            bool got1 = stereo_ ? _capture(camera_1_, img_1) : false;

            if(got0) {
              auto img_msg_0 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_0).toImageMsg();
              camera_0_publisher_->publish(*img_msg_0);
            }
            if(got1) {
              auto img_msg_1 = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_1).toImageMsg();
              camera_1_publisher_->publish(*img_msg_1);
            }

            if(!rclcpp::ok()) {
              release();
            }
            // Sleep for a while to control the publishing rate
            rclcpp::sleep_for(std::chrono::milliseconds(capture_delay_millis_));
        }
        release();
        
    }

    bool stereo_;
    bool undistort_;
    int sensor_mode_;
    int capture_delay_millis_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_0_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_1_publisher_;
    
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


/***

[image]

width
1640

height
1232

[narrow_stereo]

camera matrix
848.721379 0.000000 939.509142
0.000000 848.967602 596.153547
0.000000 0.000000 1.000000

distortion
-0.296850 0.061372 0.002562 -0.002645 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
592.232422 0.000000 942.258069 0.000000
0.000000 670.806702 591.348538 0.000000
0.000000 0.000000 1.000000 0.000000

***/

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