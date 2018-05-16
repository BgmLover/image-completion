#pragma once
#include"Image.h"
#include"AnchorPoint.h"
#include<opencv2\opencv.hpp>
#include<vector>
using namespace cv;
using namespace std;

class Structure_propagation {

private:
	Image image;
	vector<vector<AnchorPoint>> unknown_anchors;
	vector<vector<AnchorPoint>> sample_anchors;

	/*
	the way to find the anchor point is that,from the first point on the curve,each turn we get the half number
	of points of the patch.
	*/
	int getOneAnchorPos(int lastanchor_index,PointType &t,int curve_index,bool flag, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample);
	void getOneCurveAnchors(int curve_index,vector<AnchorPoint>&unknown,vector<AnchorPoint>&sample);

	Point2i getLeftTopPoint (int point_index, int curve_index);
	Point2i getLeftTopPoint(Point2i p);
	Mat getOnePatch(Point2i p);

	double calcuEi(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	double calcuEs(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	double calcuE1(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	double calcuE2(AnchorPoint unknown1, AnchorPoint unknown2, AnchorPoint sample1, AnchorPoint sample2, int curve_index);

	vector<int> DP(vector<AnchorPoint>unknown, vector<AnchorPoint>sample, int curve_index);
	//BP algorithm
	//copy the patch
	//get_a_new_curve
	
public:
	Structure_propagation() = default;
	Structure_propagation(Mat src) { image = *(new Image(src));}
	void getMask() { image.getMask(); }
	void getCurves() { image.getCurves(); }
	void getAnchors();
	void drawAnchors();//for debug
	void getNewStructure();

};