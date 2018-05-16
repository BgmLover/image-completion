#include<opencv2\opencv.hpp>
#include<iostream>
#include"Structure_propagation.h"
using namespace cv;
using namespace std;
void test(){
	Mat a = imread("2.png");
	Structure_propagation s(a);
	s.getMask();
	s.getCurves();
	s.getAnchors();
	s.drawAnchors();

}
int main() {
	test();
}

