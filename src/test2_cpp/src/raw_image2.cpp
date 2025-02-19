#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class VideoFramePublisher2 : public rclcpp::Node
{
    public:
        VideoFramePublisher2()
            : Node("video_frame_publisher")
            {
                // 创建一个发布者，发布到"/video_frames"话题
                publisher_ = this->create_publisher<sensor_msgs::msg::Image>("raw_image2", 10);
                
                capture_ = VideoCapture("/home/yuan/Desktop/output.avi");
                if (!capture_.isOpened())
                {
                    RCLCPP_ERROR(this->get_logger(), "无法打开");
                    rclcpp::shutdown();
                }
                timer_ = this->create_wall_timer(std::chrono::milliseconds(33), 
                bind(&VideoFramePublisher2::publish_frame, this));
            }
    private:
        void publish_frame()
        {
            Mat frame;
            if (capture_.read(frame))
            {
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                publisher_->publish(*msg);
                
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "视频结束，关闭节点");
                rclcpp::shutdown();
            }
        }
        
        

        VideoCapture capture_;
        rclcpp::TimerBase::SharedPtr timer_;
    
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    
};

int main(int argc, char** argv)
{
rclcpp::init(argc, argv);
auto publisher = make_shared<VideoFramePublisher2>();

rclcpp::spin(publisher);
rclcpp::shutdown();
return 0;
}
