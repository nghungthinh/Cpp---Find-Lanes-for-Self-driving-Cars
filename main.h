
#include <iostream>
#include<opencv2/opencv.hpp>

class Lanes
{
    public:
        std::string _path_img;
        cv::Mat _img_lanes;
        
        cv::Mat Canny();
        cv::Mat RoI(cv::Mat canny);

        Lanes(std::string path_img, cv::Mat img_lanes);
        Lanes(std::string path_img);
        // ~Lanes();
        
        void setPath_img(std::string path_img);
        std::string getPath__img();

        cv::Mat getImglanes();
        cv::Mat getlinesImg(cv::Mat img, std::vector<cv::Vec4i> lines);
        void average_slope_intercept(cv::Mat img, std::vector<cv::Vec4i> lines);
        void make_coordinates(cv::Mat img, cv::Point point);

        void display(cv::Mat img);
};