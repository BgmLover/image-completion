#pragma once
#include"Image.h"
#include"AnchorPoint.h"
#include<opencv2\opencv.hpp>
#include<vector>
using namespace cv;
using namespace std;

class Structure_propagation {

private:
	string path = "test_data/result/";
	/*
	the way to find the anchor point is that,from the first point on the curve,each turn we get the half number
	of points of the patch.
	*/
	int getOneAnchorPos(int lastanchor_index,PointType &t,int curve_index,bool flag, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample);
	void getOneCurveAnchors(int curve_index,vector<AnchorPoint>&unknown,vector<AnchorPoint>&sample);

	int getOneAnchorFront(int lastanchor_index, PointType &t, int curve_index, bool flag, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample);
	int getOneAnchorBack(int lastanchor_index, PointType &t, int curve_index, bool flag, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample);


	Point2i getAnchorPoint(AnchorPoint ap, int curve_index);
	Rect getRect(AnchorPoint ap, int curve_index);
	Rect getRect(Point2i p);
	Mat getOnePatch(Point2i p,Mat &img);
	Mat getOnePatch(AnchorPoint ap, Mat &img, int curve_index);
	void copyPatchToImg(AnchorPoint unknown, Mat &patch, Mat &img, int curve_index);

	float calcuEi(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	float calcuEs(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	float calcuE1(AnchorPoint unknown, AnchorPoint sample, int curve_index);
	float calcuE2(AnchorPoint unknown1, AnchorPoint unknown2, AnchorPoint sample1, AnchorPoint sample2, int curve_index);
	
	vector<int> DP(vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample, int curve_index);
	vector<int> BP(vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample, int curve_index);
	//to judge if two points are neighbor
	bool isNeighbor(Point2i point1, Point2i point2);
	bool isIntersect(int curve1, int curve2);
	//add the front and the behind anchor point as the neighbor
	void addNeighborFB(int curve_index);
	//to find the intersecting curves and merge them into the first curve
	void mergeCurves(vector<bool>&isSingle);

	//need to be correct ,not done
	void getOneNewCurve(vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample, int curve_index, bool flag);
	
	
public:
	Image image;
	vector<vector<AnchorPoint>> unknown_anchors;
	vector<vector<AnchorPoint>> sample_anchors;

	Structure_propagation() = default;
	Structure_propagation(Mat src) { image = *(new Image(src));}
	void getMask() { image.getMask(); }
	void getCurves() { image.getCurves(); }
	void getAnchors();
	void drawAnchors();
	void getNewStructure();
	//just for test
	void testOneCurve();

	//tool function
	Point2i getLeftTopPoint(int point_index, int curve_index);
	Point2i getLeftTopPoint(Point2i p);
};