#include"math_function.h"
#include"param.h"
#include<limits>
float calcuSSD(Mat m1, Mat m2) {
	if (m1.empty() || m2.empty()) {
		cout << "In calcuSSD: The mat is empty" << endl;
		throw exception();
	}
	Mat result(1, 1, CV_32F);
	matchTemplate(m1, m2, result, CV_TM_SQDIFF_NORMED);
	return result.at<float>(0, 0);
}

float calcuDistance(vector<Point2i>ci, vector<Point2i>cxi) {
	float result = 0;
	float shortest, sq;
	
	float normalized = norm(Point2i(PatchSizeCol, PatchSizeRow));

	for (int i = 0; i < ci.size(); i++) {
		shortest = FLT_MAX;
		for (int j = 0; j < cxi.size(); j++) {
			sq = norm(ci[i] - cxi[j]) / normalized;
			sq *= sq;
			if (sq < shortest) {
				shortest = sq;
			}
		}
		result += sq;
	}
	return result;
}

void initArray(float*a, int num) {
	for (int i = 0; i < num; i++) {
		a[i] = 0;
	}
}
void initArray(int *a, int num) {
	for (int i = 0; i < num; i++) {
		a[i] = 0;
	}
}
void initArray(bool*a, int num) {
	for (int i = 0; i < num; i++) {
		a[i] = false;
	}
}

void minusArray(float*a, float*b, float *c, int num);
void addArray(float*a, float*b, float*c, int num) {
	for (int i = 0; i < num; i++) {
		a[i] = a[i] + b[i];
	}
}
void minusArray(float*a, float*b, float *c, int num) {
	for (int i = 0; i < num; i++) {
		a[i] = a[i] - b[i];
	}
}
bool isEqualArray(float *a, float*b, int num) {
	for (int i = 0; i < num; i++) {
		if (a[i] != b[i]) {
			return false;
		}
	}
	return true;
}
void moveArray(float*a, float*b, int num) {
	for (int i = 0; i < num; i++) {
		a[i] = b[i];
	}
}