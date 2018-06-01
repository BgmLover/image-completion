#pragma once
#include<vector>
#include<opencv2\opencv.hpp>
#include"param.h"
using namespace cv;
using namespace std;

class Image {

public:
	Mat srcImage;
	Mat mask;						//record the position that is unknown in the image
	Mat image_masked;				//the image with the unknown region
	Mat image_inpainted;			//the image after inpainting
	vector<vector<Point2i>>curve_points;	//the user marks the curves to show the structure

	Image() = default;
	Image(Mat src);
	void getMask();
	void getCurves();

private:
	string path = "test_data/result/";
};