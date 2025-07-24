#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

bool camera_info_received = false;
bool image_received = false;
bool depth_received = false;
cv::Mat K;
cv::Mat image;
cv::Mat depth;
cv::Mat bgr;
cv::Mat hsv;
cv::Mat dst;
cv::Point cpoint;
//色相
int hmin = 0;
int hmin_Max = 360;
int hmax = 180;
int hmax_Max = 180;
//饱和度
int smin = 0;
int smin_Max = 255;
int smax = 255;
int smax_Max = 255;
//亮度
int vmin = 106;
int vmin_Max = 255;
int vmax = 255;
int vmax_Max = 255;

// Forward declaration of callBack function
void callBack(int, void*);

// Mouse callback function
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && !hsv.empty())
    {
        // Get the HSV value at the clicked position
        cv::Vec3b hsv_pixel = hsv.at<cv::Vec3b>(y, x);
        int h = hsv_pixel[0];
        int s = hsv_pixel[1];
        int v = hsv_pixel[2];
        
        ROS_INFO("Clicked at (%d, %d) - HSV: (%d, %d, %d)", x, y, h, s, v);
        
        // Update trackbar positions with a range around the clicked value
        int range = 20; // You can adjust this range as needed
        
        hmin = std::max(0, h - range);
        hmax = std::min(hmax_Max, h + range);
        smin = std::max(0, s - range);
        smax = std::min(smax_Max, s + range);
        vmin = std::max(0, v - range);
        vmax = std::min(vmax_Max, v + range);
        
        // Update trackbars
        cv::setTrackbarPos("hmin", "hsv_image", hmin);
        cv::setTrackbarPos("hmax", "hsv_image", hmax);
        cv::setTrackbarPos("smin", "hsv_image", smin);
        cv::setTrackbarPos("smax", "hsv_image", smax);
        cv::setTrackbarPos("vmin", "hsv_image", vmin);
        cv::setTrackbarPos("vmax", "hsv_image", vmax);
        
        // Call the processing function
        callBack(0, 0);
    }
}

void callBack(int, void*)
{
    if (hsv.empty()) return;
    
    //输出图像分配内存
    dst = cv::Mat::zeros(image.size(), image.type());
    //掩码
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax), mask);
    //掩模到原图的转换
    for (int r = 0; r < bgr.rows; r++)
    {
        for (int c = 0; c < bgr.cols; c++)
        {
            if (mask.at<uchar>(r, c) == 255)
            {
                dst.at<cv::Vec3b>(r, c) = bgr.at<cv::Vec3b>(r, c);
            }
        }
    }
    cv::Mat gray;
    cv::cvtColor(dst, gray, cv::COLOR_BGR2GRAY);
    cv::Mat binary;
    cv::threshold(gray, binary, 127, 255, cv::THRESH_OTSU);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
    // 遍历所有轮廓
    int max_matches = 0;
    cv::Rect best_rect;
    cv::Point center_point;

    for (size_t i = 0; i < contours.size(); i++) {
        // Get bounding rectangle
        cv::Rect rect = cv::boundingRect(contours[i]);
        
        // Create a mask for just this rectangle
        // 创建矩形框
        cv::Mat rect_mask = cv::Mat::zeros(image.size(), CV_8UC1);
        cv::drawContours(rect_mask, contours, i, cv::Scalar(255), cv::FILLED);
        
        // Count matching pixels within this rectangle
        //计算框内颜色匹配的像素数量
        cv::Mat rect_hsv;
        hsv.copyTo(rect_hsv, rect_mask);
        
        cv::Mat rect_color_mask;
        cv::inRange(rect_hsv, cv::Scalar(hmin, smin, vmin), cv::Scalar(hmax, smax, vmax), rect_color_mask);
        
        int matches = cv::countNonZero(rect_color_mask(rect));
        
        // Update best rectangle if this one has more matches
        if (matches > max_matches) {
            max_matches = matches;
            best_rect = rect;
            center_point = cv::Point(rect.x + rect.width/2, rect.y + rect.height/2);
        }
        
        // Draw all rectangles (optional)
        cv::rectangle(dst, rect, cv::Scalar(0, 0, 255), 2);
    }

    // Draw the best rectangle with a different color and mark its center
    if (max_matches > 1000) {
        cv::rectangle(dst, best_rect, cv::Scalar(0, 255, 0), 3);
        cv::circle(dst, center_point, 5, cv::Scalar(255, 0, 0), -1);
        
        ROS_INFO("Best rectangle center at (%d, %d) with %d matching pixels", 
                center_point.x, center_point.y, max_matches);
        cpoint = center_point;
    }

    if(!dst.empty()) {
        imshow("hsv_image", dst);
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr &msg)
{
    int r = 10;
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "16UC1");
        depth = cv_ptr->image.clone();
        if (!depth.empty()) {
            depth_received = true;
            
            // if (cpoint.x >= 0 && cpoint.y >= 0 && 
            //     cpoint.x < depth.cols && cpoint.y < depth.rows) {
                
                // Create circular mask
                cv::Mat mask = cv::Mat::zeros(depth.size(), CV_8UC1);
                cv::circle(mask, cpoint, r, cv::Scalar(255), -1);
                
                // Calculate average depth in the region
                cv::Mat depth_roi;
                depth.copyTo(depth_roi, mask);
                
                // Convert to float for calculation (depth is typically uint16)
                cv::Mat depth_float;
                depth_roi.convertTo(depth_float, CV_32F);
                
                // Mask out zero values (invalid depth)
                depth_float.setTo(std::numeric_limits<float>::quiet_NaN(), depth_roi == 0);
                
                // Calculate mean depth, ignoring NaN values
                cv::Scalar mean_depth = cv::mean(depth_float, mask);
                
                if (!std::isnan(mean_depth[0])) {
                    ROS_INFO("Average depth at (%d,%d): %f mm", 
                            cpoint.x, cpoint.y, mean_depth[0]);
                    
                    // Optional: Visualize the region
                    cv::Mat depth_vis;
                    cv::normalize(depth, depth_vis, 0, 255, cv::NORM_MINMAX, CV_8U);
                    cv::cvtColor(depth_vis, depth_vis, cv::COLOR_GRAY2BGR);
                    cv::circle(depth_vis, cpoint, r, cv::Scalar(0, 0, 255), 2);
                    cv::imshow("Depth ROI", depth_vis);
                    cv::waitKey(1);
                }
            // }
        }
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in depth callback: %s", e.what());
    }
}


void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        image = cv_ptr->image.clone();
        if (!image.empty()) {
            image_received = true;
            bgr = image.clone();
            cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
        }
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("Failed to convert ROS image to OpenCV image: %s", e.what());
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception in image callback: %s", e.what());
    }
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
    if (!camera_info_received) {
        // 初始化相机内参矩阵
        K = cv::Mat(3, 3, CV_32F);
        for (int i = 0; i < 9; ++i)
            K.at<float>(i/3, i%3) = msg->K[i];

        camera_info_received = true;
        ROS_INFO_STREAM("相机内参矩阵:\n" << K);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cube_detection_node");
    ros::NodeHandle nh;
    std::string image_topic, camera_info_topic, depth_topic;
    nh.param<std::string>("image_topic_name", image_topic, "/camera/color/image_raw");
    nh.param<std::string>("camera_info_topic_name", camera_info_topic, "/camera/color/camera_info");
    nh.param<std::string>("depth_topic_name", depth_topic, "/camera/depth/image_raw");
    
    ros::Subscriber image_sub = nh.subscribe(image_topic, 1, imageCallback);
    ros::Subscriber info_sub = nh.subscribe(camera_info_topic, 1, cameraInfoCallback);
    ros::Subscriber depth_sub = nh.subscribe(depth_topic, 1, depthCallback);
    
    // Create windows once
    cv::namedWindow("origin_image", cv::WINDOW_GUI_EXPANDED);
    cv::namedWindow("hsv_image", cv::WINDOW_GUI_EXPANDED);
    
    // Set mouse callback for hsv_image window
    cv::setMouseCallback("origin_image", onMouse, NULL);
    
    // Create trackbars once
    cv::createTrackbar("hmin", "hsv_image", &hmin, hmin_Max, callBack);
    cv::createTrackbar("hmax", "hsv_image", &hmax, hmax_Max, callBack);
    cv::createTrackbar("smin", "hsv_image", &smin, smin_Max, callBack);
    cv::createTrackbar("smax", "hsv_image", &smax, smax_Max, callBack);
    cv::createTrackbar("vmin", "hsv_image", &vmin, vmin_Max, callBack);
    cv::createTrackbar("vmax", "hsv_image", &vmax, vmax_Max, callBack);

    ros::Rate r(30);

    while(ros::ok()) {
        if (image_received && !image.empty()) {
            cv::imshow("origin_image", image);
            callBack(0, 0);
        }
        
        int key = cv::waitKey(1);
        if (key == 27) {  // ESC key
            break;
        }
        
        ros::spinOnce();
        r.sleep();
    }
    
    cv::destroyAllWindows();
    return 0;
}