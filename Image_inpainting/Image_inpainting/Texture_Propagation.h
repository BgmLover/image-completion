#pragma once
#include<opencv2\opencv.hpp>
#include<iostream>
#include<vector>
#include<unordered_map>
#include"Structure_propagation.h"
#include"param.h"
#include<random>
#include<set>
using namespace std;
using namespace cv;


extern float **level_map;
class Texture_Propagation {
private:
	Structure_propagation * sp;
	int area_num = 0;
	int **area;				//to show which area does the pixel belong to after the partition of the image after the sturcture propagation
	vector<int>area_size;
	vector<int>area_unknown_size;
	float **confidence_map;	//to show the how much information does the position contain

	float **Gaussian_kernel;//to calculate gaussian falloff in calculating confidence map
	Point2i **original_pixel_map;//to record the original pixel's position

	bool ifpartition = false;
	Mat mask;
	int mask_left, mask_top, mask_right, mask_bottom;	//describe the mask 
	Mat srcImg;
	Mat resImg;
	Mat region;
	Mat to_show;
	int sizeof_neighborhood = SIZEOFNEIGHBORHOOD;
	int Gaussian_kernel_size = KERNEL_SIZE;
	double level_set_radio = LEVEL_SET_RADIO;
	string path = "test_data/result/";
	/* 
	the following functions are used to determine the priority of the position to be compelete
	*/
	void init_confidence_map();
	void update_confidence_map(int area_index,vector<Point2i>&points);
	void init_gaussian_kernel(int Gaussian_kernel_size);
	float gaussion_x_y(float x, float y, float x0, float y0, float sigma = 1.0);
	float gaussion_x(float x, float sigma = 1.0);
	void cal_level_map(int area_index);

	/*
	the following functions are mainly about synthesizing textures from known regions
	*/
	void extend_curve();
	//to test if a point is in the picture
	bool inBoundary(Point2i p);
	//to test if a point is in the mask
	bool inMask(Point2i p);
	//get the points unknown in the area
	vector<Point2i>get_unknown_points(int area_index);
	//to find the original pixel of an unknown point
	bool fill_one_pixel(Point2i unknown_point, int area_index);
	//get L-shaped neighbors of one point 
	void get_candidates(Point2i unknown_point,vector<Point2i>&neighbors,vector<Point2i>&candidates,int area_index,int size);
	Point2i get_best_candidate(Point2i unknown_point,vector<Point2i>&candidates);
	void synthesize_area_texture(int area_index);
	void init_original_pixel_map();
	void show_patch(Point2i unknown,Point2i candidate);
	void show_mask();
public:
	Texture_Propagation(Structure_propagation* p);
	/*
	divide the whole image into areas to synthesize the textures independently
	*/
	void partition();
	void show_partition();
	void show_partition_image();
	void synthesize_texture();
	
};


class l_point {
public:
	int x;
	int y;
	l_point(Point2i &p) { x = p.x; y = p.y; }
	bool operator<(l_point &p2) { 
		return level_map[this->y][this->x] > level_map[p2.y][p2.x];
	}
};