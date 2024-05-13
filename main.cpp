#include <opencv2/opencv.hpp> 
#include <stdio.h> 
#include "main.h"
#include <vector>
#include <cmath>
#include <numeric>

int main(int argc, char** argv) 
{ 
	if (argc != 2) { 
		printf("usage: img.out <Image_Path>\n"); 
		return -1; 
	} 
	
	std::string path = argv[1];
	Lanes lane(path);
	cv::Mat img = cv::imread(path, 1);
	cv::Mat canny = lane.Canny();
	cv::Mat crop_img = lane.RoI(canny);
	
	std::vector<cv::Vec4i> lines;
	cv::HoughLinesP(crop_img, lines, 2, M_PI/180, 100, 40, 5);
	std::vector<cv::Vec4i> average_lines = lane.average_slope_intercept(img, lines);
	
	cv::Mat lane_lines = lane.getlinesImg(img, average_lines);

	cv::addWeighted(img, 0.8, lane_lines, 1, 1, img);
	lane.display(img);

	cv::waitKey(0);
	return 0; 
}

Lanes::Lanes(std::string path_img)
{
	this->_path_img = path_img;
}

Lanes::Lanes(std::string path_img, cv::Mat img_lanes)
{
	this->_path_img = path_img;
	this->_img_lanes = img_lanes;
}

void Lanes::setPath_img(std::string path_img)
{
	this->_path_img = path_img;
}

std::string Lanes::getPath__img()
{
	return this->_path_img;
}

cv::Mat Lanes::getImglanes()
{
	return this->_img_lanes;
}

void Lanes::display(cv::Mat img)
{
	cv::imshow("Display Image: }", img);
}

cv::Mat Lanes::Canny()
{
	cv::Mat img;
	std::string path_img = this->_path_img;
	img = cv::imread(path_img,1);

	// Chage to Grayscale
	cv::Mat GrayScale;
	cv::cvtColor(img, GrayScale, cv::COLOR_RGB2GRAY);

	// Reduce noise ( Using GassianBlur -> Smooth)
	cv::Mat GaussianBlur;
	cv::Size ksize = {5,5};
	cv::GaussianBlur(GrayScale, GaussianBlur, ksize, 0);
	
	// Detect edges
	cv::Canny(GaussianBlur, img, 50, 150);
	return img;	
}

cv::Mat Lanes::RoI(cv::Mat canny)
{
	int height = canny.rows;
	std::vector<cv::Point> triangle;
	triangle.push_back(cv::Point(200, height));
	triangle.push_back(cv::Point(1100, height));
	triangle.push_back(cv::Point(550, 250));
	
	// Mask and fill Triangle point
	cv::Mat mask = cv::Mat::zeros(canny.size(), canny.type());
	cv::fillPoly(mask, triangle, 255);

	// Bitwise for && mask point in real img
	cv::bitwise_and(canny, mask, mask);
	return mask;
}

cv::Mat Lanes::getlinesImg(cv::Mat img, std::vector<cv::Vec4i> lines)
{
	cv::Mat lane_lines = cv::Mat::zeros(img.size(), img.type());

    for (size_t i = 0; i < lines.size(); ++i) {
        cv::Vec4i line = lines[i];
        cv::line(lane_lines, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(255, 0, 0), 5);
		// std::cout << line[0] << " - " << line[1];
		// std::cout << "\n";
		// std::cout << line[2] << " - " << line[3];
		// std::cout << "\n";
    }
	// cv::line(lane_lines, cv::Point(998, 704), cv::Point(617, 422), cv::Scalar(255, 0, 0), 5);

	return lane_lines;
}

std::vector<cv::Vec4i> Lanes::average_slope_intercept(cv::Mat img, std::vector<cv::Vec4i> lines)
{
	cv::Vec4i left_fit, right_fit;
	std::vector<cv::Point> left_average, right_average;
	std::vector<float> left_slopes, right_slopes, left_intercepts, right_intercepts;
	cv::Vec4f fitLine;
	int x1, y1, x2, y2;

	for (int i = 0; i < lines.size(); i++)
	{
		cv::Vec4i line = lines[i];
		x1 = line[0];
		y1 = line[1];
		x2 = line[2];
		y2 = line[3];
		// std::cout << line[0] << " - " << line[1];
		// std::cout << "\n";
		// std::cout << line[2] << " - " << line[3];
		// std::cout << "\n";

		std::vector<cv::Point2f> points;
		cv::Point2f x1y1Point(x1, y1);
		cv::Point2f x2y2Point(x2, y2);
		points.push_back(x1y1Point);
		points.push_back(x2y2Point);

		cv::fitLine(points, fitLine, cv::DIST_L2, 1, 0.01, 0.01);
		
		float slope = fitLine[1] / fitLine[0];
		float intercept = fitLine[3] - slope * fitLine[2];;
		// std::cout << slope << "-" << intercept << "\n";
		if (slope < 0)
		{
			left_slopes.push_back(slope);
			left_intercepts.push_back(intercept);
		}
		else
		{

			right_slopes.push_back(slope);
			right_intercepts.push_back(intercept);
		}
	}
	
    // Calculate AVG Left ( Slope, Intercept)
	float left_avg_slope = std::accumulate(left_slopes.begin(), left_slopes.end(), 0.0) / left_slopes.size();
    float left_avg_intercept = std::accumulate(left_intercepts.begin(), left_intercepts.end(), 0.0) / left_intercepts.size();

    // Calculate AVG Right ( Slope, Intercept)
    float right_avg_slope = std::accumulate(right_slopes.begin(), right_slopes.end(), 0.0) / right_slopes.size();
    float right_avg_intercept = std::accumulate(right_intercepts.begin(), right_intercepts.end(), 0.0) / right_intercepts.size();

    // Print or use the average slopes and intercepts 
    std::cout << "Left Lane: Avg Slope=" << left_avg_slope << ", Avg Intercept=" << left_avg_intercept << std::endl;
    std::cout << "Right Lane: Avg Slope=" << right_avg_slope << ", Avg Intercept=" << right_avg_intercept << std::endl;
	
	left_fit = make_coordinates(img, cv::Point2f(left_avg_slope, left_avg_intercept));
	std::cout << left_fit[0] << "-" << left_fit[1] << " "<< left_fit[2] << "-" << left_fit[3] << "\n";
	right_fit = make_coordinates(img, cv::Point2f(right_avg_slope, right_avg_intercept));
	std::cout << right_fit[0] << "-" << right_fit[1] << " "<< right_fit[2] << "-" << right_fit[3] << "\n";

	std::vector<cv::Vec4i> results;
	results.push_back(left_fit);
	results.push_back(right_fit);
	return results;
}

cv::Vec4i Lanes::make_coordinates(cv::Mat img, cv::Point2f point)
{
	int y1 = img.rows;
	int y2 = int(y1*3/5);
	int x1 = int((y1 - point.y) / point.x);
	int x2 = int((y2 - point.y) / point.x);
	cv::Vec4i results;

	cv::Mat coor = (cv::Mat_<int>(2, 2) << x1, y1, x2, y2);
	// std::cout << x1 << "-" << y1 << " "<< x2 << "-" << y2 << "\n";

	results = {x1, y1, x2, y2};
	return results;
}