    #include <rclcpp/rclcpp.hpp>
    #include <sensor_msgs/msg/image.hpp>
    #include <cv_bridge/cv_bridge.h>
    #include <opencv2/opencv.hpp>

    class VideoFramePublisher : public rclcpp::Node
    {
        public:
            VideoFramePublisher()
                : Node("video_frame_publisher")
                {
                    // 创建一个发布者，发布到"/video_frames"话题
                    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("raw_image", 10);
                    
                    capture_ = cv::VideoCapture("/home/yuan/Desktop/test0/test2.mp4");
                    if (!capture_.isOpened())
                    {
                        RCLCPP_ERROR(this->get_logger(), "无法打开视频文件！");
                        rclcpp::shutdown();
                    }

                    // 创建定时器，每秒发布一帧
                    timer_ = this->create_wall_timer(std::chrono::milliseconds(33), // 假设帧率为30fps
                    std::bind(&VideoFramePublisher::publish_frame, this));
                }

        private:
            void publish_frame()
            {
                cv::Mat frame;
                if (capture_.read(frame))
                {
                    // 使用cv_bridge将OpenCV图像转换为ROS 2消息
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
                    publisher_->publish(*msg);
                    
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "视频结束，关闭节点");
                    rclcpp::shutdown();
                }
            }
            
           

            cv::VideoCapture capture_;
            rclcpp::TimerBase::SharedPtr timer_;
        
            rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        
    };

    int main(int argc, char** argv)
    {
    rclcpp::init(argc, argv);
    auto publisher = std::make_shared<VideoFramePublisher>();

    rclcpp::spin(publisher);
    rclcpp::shutdown();
    return 0;
    }

