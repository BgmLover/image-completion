#pragma once
#include"Image.h"
#include"AnchorPoint.h"
#include<opencv2\opencv.hpp>
#include<vector>
using namespace cv;
using namespace std;

class Structure_propagation {

private:
	
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
	Point2i getAnchorPoint(AnchorPoint ap, int curve_index);
	Rect getRect(AnchorPoint ap, int curve_index);
	Mat getOnePatch(Point2i p,Mat &img);
	Mat getOnePatch(AnchorPoint ap, Mat &img, int curve_index);
	void copyPatchToImg(AnchorPoint unknown, Mat &patch, Mat &img, int curve_index);

	float calcuEi(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	float calcuEs(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	float calcuE1(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	float calcuE2(AnchorPoint unknown1, AnchorPoint unknown2, AnchorPoint sample1, AnchorPoint sample2, int curve_index);
	
	vector<int> DP(vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample, int curve_index);

	//need to be correct ,not done
	
	void getOneNewCurve(vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample, int curve_index, bool flag);
	//BP algorithm
	
	
public:
	Image image;
	Structure_propagation() = default;
	Structure_propagation(Mat src) { image = *(new Image(src));}
	void getMask() { image.getMask(); }
	void getCurves() { image.getCurves(); }
	void getAnchors();
	void drawAnchors();//for debug
	void getNewStructure();
	void testOneCurve();
};