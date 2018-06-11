#pragma once
#include<opencv2\opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;

class Photometric_Correction {
private:
	Mat mask;
	int **B;
	float **Jx, **Jy;
	float **J_x, **J_y;
	Mat src;
	Mat patch;
public:
	Photometric_Correction() = default;
	Photometric_Correction(Mat &mask); 
	void updateMask(Rect rec);
	bool inMask(Point2i p);
	Mat correct(Mat &patch, Mat &resImg,Rect &rec);
};