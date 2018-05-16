#pragma once
#include<opencv2\opencv.hpp>
#include<iostream>
#include<vector>
using namespace cv;
using namespace std;

float calcuSSD(Mat m1, Mat m2);
float calcuDistance(vector<Point2i>ci, vector<Point2i>cxi);

