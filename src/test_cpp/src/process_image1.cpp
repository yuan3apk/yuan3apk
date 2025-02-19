// image_processing_node.cpp
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
//#include <opencv2/ml.hpp>
#include <vector>
#include <iostream>
using namespace std;
using namespace cv;
using namespace cv::ml;

class ImageProcessingNode1 : public rclcpp::Node {
public:
    ImageProcessingNode1() : Node("image_processing_node1") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("processed_image1", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "raw_image", 10,std::bind(&ImageProcessingNode1::imageCallback, this, std::placeholders::_1));
        
        
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        // 调用你的图像处理函数
        Mat hsv, mask_red, mask_blue;
        vector<RotatedRect> minRects_red, minRects_blue;
        vector<array<Point2f, 4>> points_red, points_blue;
        vector<Point2f> center_red, center_blue;
        vector<float> aspectRatios_red, aspectRatios_blue;

        // 预处理一帧图像以获取HSV图像和颜色掩码
        handel_frame(frame, hsv, mask_red, mask_blue);

        // 从红色和蓝色掩码中选择矩形区域
        select_rect(mask_red, minRects_red, points_red, center_red);
        select_rect(mask_blue, minRects_blue, points_blue, center_blue);

        // 计算所有检测到的矩形的宽高比
        aspectRatio(minRects_red, aspectRatios_red);
        aspectRatio(minRects_blue, aspectRatios_blue);
        for (size_t i = 0; i < minRects_red.size(); i++)
        {
            for(size_t j = 1; j < i; j++)
            {   // 获取当前矩形的宽高比
                float aspectRatio_red1 = aspectRatios_red[i];
                float aspectRatio_red2 = aspectRatios_red[j];
                // 判断是否为目标对象
                bool iftarget_red1= iftarget(aspectRatio_red1);
                bool iftarget_red2 = iftarget(aspectRatio_red2);
                float calculation;
                if (iftarget_red1&&iftarget_red2)
                {
                    calculate(minRects_red[i], minRects_red[j], center_red[i], center_red[j],calculation);
                    bool ifPair=ifpair(calculation);
                    if(ifPair)
                    {
                        
                        draw_rect(frame, points_red[i].data(), Scalar(255,0,0));
                        draw_rect(frame, points_red[j].data(), Scalar(255,0,0));
                        
                    }
                }
            }
        }
        for (size_t i = 0; i < minRects_blue.size(); i++)
        {
            for(size_t j = 1; j < i; j++)
            {   // 获取当前矩形的宽高比
                float aspectRatio_blue1 = aspectRatios_blue[i];
                float aspectRatio_blue2 = aspectRatios_blue[j];
                // 判断是否为目标对象
                bool iftarget_blue1= iftarget(aspectRatio_blue1);
                bool iftarget_blue2 = iftarget(aspectRatio_blue2);
                float calculation;
                if (iftarget_blue1&&iftarget_blue2)
                {
                    calculate(minRects_blue[i], minRects_blue[j], center_blue[i], center_blue[j],calculation);
                    bool ifPair=ifpair(calculation);
                    if(ifPair)
                    {
                        
                        draw_rect(frame, points_blue[i].data(), Scalar(0,0,255));
                        draw_rect(frame, points_blue[j].data(), Scalar(0,0,255));
                         
                    }
                }
            }
        }

        // 发布处理后的图像
        auto processed_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        publisher_->publish(*processed_msg);
    }

    // 处理帧图像，将其转换为HSV色彩空间，并根据指定的颜色范围创建红色和蓝色的掩码
    void handel_frame(Mat &frame, Mat &hsv, Mat &mask_red, Mat &mask_blue)
    {
        // 将输入的BGR图像转换为HSV色彩空间
        cvtColor(frame, hsv, COLOR_BGR2HSV);
    
        // 定义红色和蓝色的HSV颜色范围
        Scalar lower_red1(8, 0, 200), upper_red1(23, 255, 255);
        Scalar lower_blue(85, 0, 180), upper_blue(105, 150, 255);
    
        // 根据红色和蓝色的HSV颜色范围创建掩码
        inRange(hsv, lower_red1, upper_red1, mask_red);
        inRange(hsv, lower_blue, upper_blue, mask_blue);
    }

    // 从掩码中选择矩形区域，并计算其相关属性
    void select_rect(Mat &mask, vector<RotatedRect> &minRects, vector<array<Point2f, 4>> &points, vector<Point2f> &center)
    {
        // 存储找到的轮廓
        vector<vector<Point>> contours;

        // 将输入图像转换为二值图像
        Mat binaryMask;
        threshold(mask, binaryMask, 0, 255, THRESH_BINARY);

        // 查找轮廓
        findContours(binaryMask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // 遍历所有轮廓，计算最小面积矩形
        for (size_t i = 0; i < contours.size(); i++)
        {
            RotatedRect minRect = minAreaRect(contours[i]);
            minRects.push_back(minRect);

            // 获取最小面积矩形的四个顶点
            Point2f rect_points[4];
            minRect.points(rect_points);
            points.push_back({rect_points[0], rect_points[1], rect_points[2], rect_points[3]});

            // 存储矩形中心点
            center.push_back(minRect.center);
        }
    }

    // 计算所有最小面积矩形的宽高比
    void aspectRatio(vector<RotatedRect> &minRects, vector<float> &aspectRatios)
    {
        // 遍历所有最小面积矩形，计算并存储宽高比
        for (size_t i = 0; i < minRects.size(); i++)
        {   
            RotatedRect rect = minRects[i];
            float width = max(rect.size.width, rect.size.height);
            float height = min(rect.size.width, rect.size.height);
            
            aspectRatios.push_back(width / height);
        }
    }
    void calculate(RotatedRect &minRects1,RotatedRect& minRects2,Point2f &center1,Point2f &center2,float &calculateRatios)
    {
        float distance = sqrt(pow(center1.x - center2.x, 2) + pow(center1.y - center2.y, 2));
        float width = max(minRects1.size.width, minRects2.size.height);
        calculateRatios = distance / width;
    }
   

    // 判断给定的宽高比是否满足目标条件
    bool iftarget(float aspectRatio)
    {
        // 检查宽高比是否大于等于3.2，以确定是否为目标对象
        if (aspectRatio >= 3.2)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    bool ifpair(float calculateRatios)
    {
        if ((calculateRatios >=2.0&&calculateRatios<=2.6)||(calculateRatios >= 3.7&&calculateRatios <= 4.3))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    void draw_rect(Mat &frame, Point2f point[4], Scalar color)
    {
        // 绘制矩形的四个边
        for (size_t i = 0; i < 4; i++)
        {
            line(frame, point[i], point[(i + 1) % 4], color, 4);
        }
    }


        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};
    
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<ImageProcessingNode1>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

