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

class ImageProcessingNode2 : public rclcpp::Node {
public:
    ImageProcessingNode2() : Node("image_processing_node2") {
        subscription2_ = this->create_subscription<sensor_msgs::msg::Image>(
            "raw_image", 10,
            std::bind(&ImageProcessingNode2::imageCallback, this, std::placeholders::_1));
        publisher2_ = this->create_publisher<RectArray>("processed_image2", 10);
        
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
        vector<Mat> img_roi;
        vector<Rect> rects;
        vector<vector<Point>> contours;
        getroi(frame, img_roi,rects,contours);
        vector<Rect> rects_res;


        for(size_t i = 0; i < contours.size(); i++)
        {
            Mat sample1;
            sample1 = preprocessImage(img_roi[i], 32);
    
            
            
            for (auto& contour : contours)
            {
                
                
            
                vector<double> features;
                if(contour.size()<3)
                {
                
                    continue;
                }
    
                double perimeter = arcLength(contour, true);
            
                Rect bounding_rect = boundingRect(contour);
                 
                double aspect_ratio = static_cast<double>(bounding_rect.width) / bounding_rect.height;
                double compactness = contourArea(contour) / (perimeter * perimeter);
                
                features.push_back(aspect_ratio);
                features.push_back(compactness);
                
                double prediction_fea ;
                if ((features[0] > 0.5 && features[0] < 2.5) && (features[1] > 0.01 && features[1] < 0.1))
                {
                    prediction_fea = 1;
                }
                else
                {   
                    prediction_fea = 0;
                }

                if(prediction_fea>0)
                {
                   
                    rects_res.push_back(rects[i]);
                    
            
                }
            } 
            if(rects_res.size()>0)
            {
                RectArray rectArray;
                for (size_t i = 0; i < rects_res.size(); i++)
                {
                    example_ros2_interfaces::msg::Rect ros_rect;  // 创建消息类型对象
                    ros_rect.center_x = rects_res[i].x;                  // 坐标转换
                    ros_rect.center_y = rects_res[i].y;
                    ros_rect.width = rects_res[i].width;
                    ros_rect.height = rects_res[i].height;
                    rectArray.rects.push_back(ros_rect);          // 推送转换后的对象
                }
                publisher2_->publish(rectArray);
            }
        }
    }
    Mat preprocessImage(const Mat &image, int inputSize) 
    {
        Mat img, resizedImage;
        // 将图像调整为模型输入大小
        resize(image, img, Size(inputSize, inputSize));
        img.convertTo(resizedImage, CV_32F);
        Mat grayImage;
        // 转换为灰度图像
        cvtColor(resizedImage, grayImage, COLOR_BGR2GRAY);
        Mat normalizedImage = grayImage / 255.0;
        return normalizedImage;
    }  
    Mat preprocess(const Mat &image, int inputSize) 
    {
        Mat img, resizedImage;
        // 将图像调整为模型输入大小
        resize(image, img, Size(inputSize, inputSize));
        img.convertTo(resizedImage, CV_32F);
        Mat normalizedImage = img / 255.0;
        return normalizedImage;
    }   
    void ifempty(Mat &img)
    {
        if (img.empty())
        {
            cout << "img is empty" << endl;
        }
    }
    void draw_prediction(Mat &frame, int &prediction)
    {
        putText(frame, "prediction: " + to_string(prediction), Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    }

    Mat preprocess_image(Mat& img) 
    {
        Mat gray, blur, thresh;
        cvtColor(img, gray, COLOR_BGR2GRAY);
        GaussianBlur(gray, blur, Size(5, 5), 0);
        adaptiveThreshold(blur, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2);
        return thresh;
    }
    vector<Rect> find_digit_contours(Mat& thresh) 
    {
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(thresh, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<Rect> digit_rects;
        for (const auto& cnt : contours) 
        {
            Rect rect = boundingRect(cnt);
            double aspect_ratio = (double)rect.width / rect.height;
            double area = contourArea(cnt);

        // 过滤条件：宽高比和面积
        if (aspect_ratio > 0.2 && aspect_ratio < 1.2 && area > 100) {
        digit_rects.push_back(rect);
        }
        }
        return digit_rects;
    }


    void getroi(Mat &img, vector<Mat> &img_roi,vector<Rect> &rects,vector<vector<Point>> &contours_roi)
    {
        
        
        
        // 用于二值化的图像
        Mat gray,binary;
        cvtColor(img,gray,COLOR_BGR2GRAY);
        GaussianBlur(gray,gray,Size(13,13),4,4);
        threshold(gray,binary,170,255,THRESH_BINARY);
    
        vector<vector<Point>> contours;
        findContours(binary,contours,RETR_TREE,CHAIN_APPROX_SIMPLE);
        
        for (const auto &contour : contours)
        {
            // 计算当前轮廓的边界矩形
            Rect bounding_rect = boundingRect(contour);
            if(contour.size()<=3)
            {
                
                continue;
            }
            if(bounding_rect.width==0)
            {
                
                continue;
            }
            // 检查矩形的高度与宽度之比，以确定是否提取该区域
            if (bounding_rect.height / bounding_rect.width > 0.5 && bounding_rect.height / bounding_rect.width < 2.5)
            {
                // 提取感兴趣区域
                Mat sample = img(bounding_rect);
                // 将样本添加到存储向量中
                img_roi.push_back(sample);
                // 将边界矩形添加到存储向量中
                rects.push_back(bounding_rect);
                contours_roi.push_back(contour);
            }
        }
    }
    void extract_features1(const vector<vector<Point>> &contours,vector<vector<double>>& train_features,vector<double>& train_labels)
    {   
        for(auto &contour:contours)
        {
            vector<double> features;
            if(contour.size()>3)
            {
            
                continue;
                    
            }
        
            double perimeter = arcLength(contour, true);
            Rect bounding_rect = boundingRect(contour);
            double aspect_ratio = static_cast<double>(bounding_rect.width) / bounding_rect.height;
            double compactness = contourArea(contour) / (perimeter * perimeter);
            
            features.push_back(aspect_ratio);
            features.push_back(compactness);
            train_features.push_back(features);
            if ((features[0] > 0.5 && features[0] < 3.0) && (features[1] > 0.05 && features[1] < 0.5))
                {
                    train_labels.push_back(1);
                }
                else
                {
                    train_labels.push_back(0);
                }
        }
    }

        

    vector<double> extract_features2(const vector<Point> &contour)
    {
        vector<double> features;
        
        
            double perimeter = arcLength(contour, true);
            Rect bounding_rect = boundingRect(contour);
            double aspect_ratio = static_cast<double>(bounding_rect.width) / bounding_rect.height;
            double compactness = contourArea(contour) / (perimeter * perimeter);
            
            features.push_back(aspect_ratio);
            features.push_back(compactness);
        
        return features;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription2_;
    rclcpp::Publisher<RectArray>::SharedPtr publisher2_;
    
};
int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    auto node=std::make_shared<ImageProcessingNode2>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}