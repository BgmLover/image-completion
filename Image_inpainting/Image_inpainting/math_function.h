#pragma once
#include<opencv2\opencv.hpp>
#include<iostream>
#include"param.h"
#include<vector>
using namespace cv;
using namespace std;

float calcuSSD(Mat m1, Mat m2);
float calcuDistance(vector<Point2i>ci, vector<Point2i>cxi);
void initArray(float*a, int num);
void initArray(int *a, int num);
void initArray(bool*a, int num);
void addArray(float*a, float*b, float*c,int num);
void minusArray(float*a, float*b, float *c,int num);
bool isEqualArray(float *a, float*b, int num);
void moveArray(float*a, float*b, int num);
bool contain(Rect &rec, Point2i &p);