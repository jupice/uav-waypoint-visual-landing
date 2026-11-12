#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <std_msgs/Bool.h> 
class ImageDetector {
public:
ImageDetector() : nh_("~"), it_(nh_) {
ROS_INFO("Initializing ImageDetector node...");
 detection_pub_ = nh_.advertise<std_msgs::Bool>("/detection_result", 1);
// 加载目标图片
std::string image_path;
nh_.param<std::string>("target_image_path", image_path,
ros::package::getPath("rotors_gazebo") + "/resource/1.png");

ROS_INFO("Loading target image from: %s", image_path.c_str());
target_image_ = cv::imread(image_path, cv::IMREAD_COLOR);

if(target_image_.empty()) {
ROS_ERROR("Failed to load target image!");
ros::shutdown();
}
else {
ROS_INFO("Target image loaded successfully. Size: %dx%d", target_image_.cols, target_image_.rows);
}
// 调整尺寸（示例：缩小到 200x200）
cv::Mat resized_image;
cv::resize(target_image_, resized_image, cv::Size(200, 200));

// 更新目标图片
target_image_ = resized_image;


// 初始化订阅者和窗口
image_sub_ = it_.subscribe("/hummingbird/vi_sensor/camera_3/image_raw", 1, &ImageDetector::imageCallback, this);
ROS_INFO("Subscribed to topic: %s", image_sub_.getTopic().c_str());
cv::namedWindow("Detection Result", cv::WINDOW_AUTOSIZE);
ROS_INFO("OpenCV window initialized.");
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
std_msgs::Bool detection_msg;
try {
cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

// 执行模板匹配
cv::Mat result;
cv::matchTemplate(frame, target_image_, result, cv::TM_CCOEFF_NORMED);

double min_val, max_val;
cv::Point min_loc, max_loc;
cv::minMaxLoc(result, &min_val, &max_val, &min_loc, &max_loc);

// 如果匹配度高于阈值
if(max_val > 0.3) {
detection_msg.data = true;
// 绘制矩形框
cv::rectangle(frame, max_loc,
cv::Point(max_loc.x + target_image_.cols, max_loc.y + target_image_.rows),
cv::Scalar(0, 255, 0), 2);

// 打印检测结果
ROS_INFO("Target detected! Image name: H");
}
else {
detection_msg.data = false;
// 未检测到目标时打印日志
ROS_INFO("No target detected.");
}

detection_pub_.publish(detection_msg);
// 显示结果
cv::imshow("Detection Result", frame);
cv::waitKey(1);

} catch (cv_bridge::Exception& e) {
ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
}
}

private:
ros::NodeHandle nh_;
image_transport::ImageTransport it_;
image_transport::Subscriber image_sub_;
cv::Mat target_image_;
ros::Publisher detection_pub_;
};

int main(int argc, char** argv) {
ros::init(argc, argv, "image_detector");
ImageDetector detector;
ros::AsyncSpinner spinner(1);
spinner.start();
ros::waitForShutdown();
return 0;
}