#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include "example_ros2_interfaces/msg/rect_array.hpp"
#include <opencv2/opencv.hpp>
//#include <opencv2/ml.hpp>
#include <vector>
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::ml;
using RectArray= example_ros2_interfaces::msg::RectArray;
class Image : public rclcpp::Node
{
public:
    ~Image()
    {
        image_sub1_.reset();
        image_sub2_.reset();
        timer_->cancel();
        cv::destroyAllWindows();
    }
    Image() : Node("image")
    {
        image_sub1_ = this->create_subscription<sensor_msgs::msg::Image>(
            "processed_image1", 10,bind(&Image::image_callback1, this,placeholders::_1));
        image_sub2_ = this->create_subscription<RectArray>(
            "processed_image2", 10,bind(&Image::image_callback2, this,placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33),
         std::bind(&Image::publish_result, this));
        
    }
private:
    Mat img1;
    mutex img1_mutex;
    void image_callback1(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        lock_guard<mutex> lock(img1_mutex);
        img1= cv_bridge::toCvShare(msg, "bgr8")->image;
    }
    void image_callback2(const RectArray::SharedPtr msg)
    {
        lock_guard<mutex> lock(img1_mutex);
        for(auto rect : msg->rects)
        {
            cv::Rect cv_rect(
                rect.center_x,         // 假设消息包含x,y,width,height字段
                rect.center_y,
                rect.width,
                rect.height
            );
            rectangle(img1, cv_rect, Scalar(0,255,0), 2);
        }
    }
    void publish_result()
    {
        lock_guard<mutex> lock(img1_mutex);
        if(img1.empty())
        {
            RCLCPP_INFO(this->get_logger(), "视频结束，关闭节点");
            //rclcpp::shutdown();
        }
        else{

            imshow("result",img1);
            if(cv::waitKey(33)==27)
            {
                cv::destroyAllWindows();
                rclcpp::shutdown();
            }
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub1_;
    rclcpp::Subscription<RectArray>::SharedPtr image_sub2_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr result_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<Image>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}