#pragma once
#include<opencv2\opencv.hpp>
#include<iostream>
#include<vector>

#include"Structure_propagation.h"
#include"param.h"

using namespace std;
using namespace cv;

class Texture_Propagation {
private:
	Structure_propagation * sp;
	int **area;				//to show which area does the pixel belong to after the partition of the image after the sturcture propagation
	float **confidence_map;	//to show the how much information does the position contain
	float **level_map;		//to determine the set of candidate positions
	Mat mask;
	Mat srcImg;
	int sizeof_neighborhood = SIZEOFNEIGHBORHOOD;


	/* 
	the following functions are used to determine the priority of the position to be compelete
	*/
	void init_confidence_map();
	void update_confidence_map();
	void cal_level_map();

	/*
	the following functions are mainly about synthesizing textures from known regions
	*/
	int get_best_candidate();

public:
	Texture_Propagation(Structure_propagation* p);
	/*
	divide the whole image into areas to synthesize the textures independently
	*/
	void partition();

	void synthesize_texture();
	
};