#include<opencv2\opencv.hpp>
#include<iostream>
using namespace cv;
using namespace std;
void test(){
	Mat a = imread("2.png");
	if (!a.empty()) {
		imshow("test", a);
		waitKey();
	}
}
int main() {
	test();
}

