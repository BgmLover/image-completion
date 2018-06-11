#include"Photometric_Correction.h"

Photometric_Correction::Photometric_Correction(Mat & mask)
{
	this->mask = mask.clone();
	int rows = mask.rows;
	int cols = mask.cols;

}

void Photometric_Correction::updateMask(Rect rec)
{
	mask(rec).setTo(255);
}


bool Photometric_Correction::inMask(Point2i p)
{
	return mask.at<uchar>(p.y, p.x) == 0;
}

Mat  Photometric_Correction::correct(Mat &patch, Mat &resImg, Rect &rec)
{
	Point2i left_top = Point2i(rec.x, rec.y);
	Point2i right_down = left_top + Point2i(rec.width, rec.height);
	Mat dst = resImg(rec).clone();
	Mat src = patch.clone();
	Mat _mask = mask(rec).clone();

	for (int row = 0; row < _mask.rows; row++) {
		for (int col = 0; col < _mask.cols; col++) {
			if (_mask.at<uchar>(row, col) == 255) {
				_mask.at<uchar>(row, col) = 0;
			}
			else {
				_mask.at<uchar>(row, col) = 255;
			}
		}
	}

	//imshow("src", src);
	//imshow("dst", dst);
	//imshow("_mask", _mask);
	//imshow("mask", mask);
	//waitKey();
	for (int i = 0; i < dst.rows; i++) {
		for (int j = 0; j < dst.cols; j++) {
			if (_mask.at<uchar>(i, j) == 255) {
				dst.at<Vec3b>(i, j) = patch.at<Vec3b>(i, j);
			}
		}
	}
	/*for (int i = 0; i < dst.cols; i++) {
		if (_mask.at<uchar>(0, i) == 255) {
			dst.at<Vec3b>(0, i) = patch.at<Vec3b>(0, i);
		}
		if (_mask.at<uchar>(dst.rows-1, i) == 255) {
 			dst.at<Vec3b>(dst.rows - 1, i) = patch.at<Vec3b>(dst.rows - 1, i);
		}
		if (_mask.at<uchar>(1, i) == 255) {
			dst.at<Vec3b>(1, i) = patch.at<Vec3b>(1, i);
		}
		if (_mask.at<uchar>(dst.rows - 2, i) == 255) {
			dst.at<Vec3b>(dst.rows - 2, i) = patch.at<Vec3b>(dst.rows - 2, i);
		}
	}
	for (int j = 0; j < dst.rows; j++) {
		if (_mask.at<uchar>(j, 1) == 255) {
			dst.at<Vec3b>(j, 1) = patch.at<Vec3b>(j, 1);
		}
		if (_mask.at<uchar>(j, dst.cols - 2) == 255) {
			dst.at<Vec3b>(j, dst.cols - 2) = patch.at<Vec3b>(j, dst.cols - 2);
		}
		if (_mask.at<uchar>(j, 0) == 255) {
			dst.at<Vec3b>(j, 0) = patch.at<Vec3b>(j, 0);
		}
		if (_mask.at<uchar>(j, dst.cols - 1) == 255) {
			dst.at<Vec3b>(j, dst.cols - 1) = patch.at<Vec3b>(j, dst.cols - 1);
		}
	}*/
	Rect re;
	re.x = 1; re.y = 1;
	re.width = rec.width - 1;
	re.height = rec.height - 1;
	_mask.setTo(255);
	//waitKey();
	Mat blend;
	src = src(re).clone();
	seamlessClone(src, dst, _mask, Point(patch.cols / 2, patch.rows / 2), blend, NORMAL_CLONE);
	blend.copyTo(patch);
	//imshow("dst_n", dst);
	//imshow("blend", blend);
	//imshow("res", resImg);
	//waitKey();
	updateMask(rec);
	return blend;
}
