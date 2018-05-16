#pragma once
 
#include"param.h"
#include<vector>
class AnchorPoint {

public:
	int begin_point;
	int end_point;
	int anchor_point;
	PointType type;
	std::vector<int> neighbors;

	AnchorPoint(int begin,int end,int anchor,PointType t):begin_point(begin),end_point(end),anchor_point(anchor),type(t){}
};
