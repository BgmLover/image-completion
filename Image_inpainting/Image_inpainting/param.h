#pragma once
#include<string>
using namespace std;

#define LINE_CURVE 1
#define PENCIL_CURVE 2
#define MASK_SIZE 40
#define CURVE_SIZE 6

//in Structure_Propagation
#define KS 50
#define KI 2
#define PatchSizeRow 13
#define PatchSizeCol 13


//in Texture_Propagation

#define SIZEOFNEIGHBORHOOD 5
#define KERNEL_SIZE 5
#define PI 3.14
#define LEVEL_SET_RADIO 0.1

typedef enum {
	INNER,BORDER,OUTER
}PointType;

