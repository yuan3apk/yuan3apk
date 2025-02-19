#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

class ImageProcess : public rclcpp::Node
{
public:
    ImageProcess() : Node("image_process")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "raw_image2", 10, std::bind(&ImageProcess::image_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            100ms, std::bind(&ImageProcess::timer_callback, this));

        // 初始化相机矩阵和畸变系数
        camera_matrix = (Mat_<double>(3,3) << 1795.48075, 0., 719.15967,
                                        0., 1788.97397, 554.12545,
                                        0., 0., 1.);
        distortion_coefficients = (Mat_<double>(5,1) << -0.073464, 0.128799, 0.001334, 0.001541, 0.000000);
        lengthpix = 100;
        lengthmeter = 1.35;
        focalpix = 1795.48075;
        distance = focalpix * lengthmeter / lengthpix;
        
        // 初始化其他变量
        prev_points.clear();
        prev_gray = Mat();
        curr_gray = Mat();
        curr_points.clear();
        status.clear();
        err.clear();
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;
            Mat frame;
            undistort(img, frame, camera_matrix, distortion_coefficients);
            cvtColor(frame, curr_gray, COLOR_BGR2GRAY);

            // 初始化光流跟踪点
            if (prev_points.empty() || prev_points.size() < 10)
            {
                goodFeaturesToTrack(curr_gray, prev_points, 100, 0.3, 7);
            }

            // 确保 prev_gray 已初始化
            if (prev_gray.empty())
            {
                prev_gray = curr_gray.clone();
                return;  // 如果没有前一帧灰度图，直接返回
            }

            // 计算光流
            vector<Point2f> valid_prev_points;
            vector<Point2f> valid_curr_points;
            Point2f center(0,0);
            int count = 0;

            try {
                calcOpticalFlowPyrLK(prev_gray, curr_gray, prev_points, curr_points, status, err);
            } catch (const cv::Exception& e) {
                RCLCPP_ERROR(this->get_logger(), "OpenCV Exception in calcOpticalFlowPyrLK: %s", e.what());
                return;
            }

            for (size_t i = 0; i < prev_points.size(); ++i)
            {
                if (status[i])
                {
                    valid_prev_points.push_back(prev_points[i]);
                    valid_curr_points.push_back(curr_points[i]);
                    center += curr_points[i];
                    count++;
                }
            }

            prev_points = valid_prev_points;
            curr_points = valid_curr_points;

            double angle = 0;

            for (size_t i = 0; i < prev_points.size(); i++)
            {
                if (status[i])
                {
                    Point2f diff = curr_points[i] - prev_points[i];
                    angle += atan2(diff.y, diff.x);
                }
            }

            int a;
            // 判断旋转方向
            if (abs(angle) < 0.01)
            {
                a=0;
                RCLCPP_INFO(this->get_logger(), "Object is stationary");
            }
            else if (angle > 0)
            {
                a=1;
                RCLCPP_INFO(this->get_logger(), "Object is rotating clockwise");
            }
            else
            {   
                a=2;
                RCLCPP_INFO(this->get_logger(), "Object is rotating counter-clockwise");
            }

            // 计算角速度
            double angular_velocity = angle / 100;
            RCLCPP_INFO(this->get_logger(), "Angular velocity: %f rad/s", angular_velocity);
            double time=distance/15;
            double turn_point=get_point(time,angular_velocity,lengthpix,lengthmeter);
            RCLCPP_INFO(this->get_logger(), "turn_point: %f", turn_point);
            Point3d point3D=Point3d(0,0,5);
            if(a==0)
            {
                circle(frame, get_2D(camera_matrix,point3D), 5, Scalar(0, 0, 255), -1);
                cout<<get_2D(camera_matrix,point3D);
            }
            else if(a==1)
            {
                
                circle(frame, get_2D(camera_matrix,Point3d(-turn_point,0,5)), 5, Scalar(0, 0, 255), -1);
                cout<<get_2D(camera_matrix,Point3d(-turn_point,0,5));
            }
            else 
            {
                
                circle(frame, get_2D(camera_matrix,Point3d(turn_point,0,5)), 5, Scalar(0, 0, 255), -1);
                cout<<get_2D(camera_matrix,Point3d(turn_point,0,5));
            }

            // 更新上一帧
            prev_gray = curr_gray.clone();
            this->frame = frame.clone();  // 更新 frame 以便显示
        } catch (const cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV Exception: %s", e.what());
        }
    }

    void timer_callback()
    {
        if (!this->frame.empty()) {
            cv::imshow("Image window", this->frame);
            if(cv::waitKey(30)==27)
                {
                    cv::destroyAllWindows();
                    rclcpp::shutdown();
                }
        }
    }

    Point2d get_2D(const Mat &camera_matrix, const Point3d& point3D)
    {
        Mat point3D_homogeneous=(Mat_<double>(3,1)<<point3D.x,point3D.y,point3D.z);
        Mat point2D_homogeneous=camera_matrix*point3D_homogeneous;
        double u=point2D_homogeneous.at<double>(0)/point2D_homogeneous.at<double>(2);
        double v=point2D_homogeneous.at<double>(1)/point2D_homogeneous.at<double>(2);
        return Point2d(u,v);
    }

    double get_point(const double &time, const double& angular_velocity, const double& lengthpix, const double &lengthmeter)
    {
        double _angle=angular_velocity*time;
        double sin_angle=sin(_angle);
        double turn_length=0.67*sin_angle;
        double turn_pix=turn_length*lengthpix/lengthmeter;
        double turn_point=turn_pix/200;
        return turn_point;
    }

    Mat camera_matrix;
    Mat distortion_coefficients;
    double lengthpix;
    double lengthmeter;
    double focalpix;
    double distance;
    Mat frame, img, prev_frame, prev_gray, curr_gray;
    vector<Point2f> prev_points, curr_points;
    vector<uchar> status;
    vector<float> err;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = make_shared<ImageProcess>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}