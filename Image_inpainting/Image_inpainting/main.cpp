#include<opencv2\opencv.hpp>
#include<iostream>
#include"Image.h"
using namespace cv;
using namespace std;
void test(){
	Mat a = imread("2.png");
	Image img(a);
	img.getMask();

}
int main() {
	test();
}

