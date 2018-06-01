#pragma once
#include<string>
using namespace std;

//in Structure_Propagation
#define KS 50
#define KI 2
#define PatchSizeRow 12
#define PatchSizeCol 12


//in Texture_Propagation

#define SIZEOFNEIGHBORHOOD 5
#define KERNEL_SIZE 5
#define PI 3.14


typedef enum {
	INNER,BORDER,OUTER
}PointType;

