#pragma once
#include<string>
using namespace std;

#define LINE_CURVE 1
#define PENCIL_CURVE 2
#define MASK_SIZE 20
#define CURVE_SIZE 3

//in Structure_Propagation
#define KS 50
#define KI 2
#define PatchSizeRow 12
#define PatchSizeCol 12


//in Texture_Propagation

#define SIZEOFNEIGHBORHOOD 5
#define KERNEL_SIZE 5
#define PI 3.14
#define LEVEL_SET_RADIO 0.1

typedef enum {
	INNER,BORDER,OUTER
}PointType;

