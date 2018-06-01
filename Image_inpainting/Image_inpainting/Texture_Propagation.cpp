#include"Texture_Propagation.h"
#include"debug.h"
/*
纹理生成部分，记得在候选人部分去除重复的候选人
5*5 neighborhood
*/

Texture_Propagation::Texture_Propagation(Structure_propagation * p)
{
	sp = p;
	srcImg = p->image.srcImage.clone();
	mask = p->image.mask.clone();
	int curve_size = p->unknown_anchors.size();
	for (int curve_index = 0; curve_index < curve_size; curve_index++) {
		for (int anchor_index = 0; anchor_index < p->unknown_anchors[curve_index].size(); anchor_index++) {
			int  point_index = p->unknown_anchors[curve_index][anchor_index].anchor_point;
			Point2i left_top = p->getLeftTopPoint(point_index, curve_index);
			Point2i right_down = left_top + Point2i(PatchSizeCol, PatchSizeRow);
			Mat tmp = mask(Rect(left_top, right_down));
			tmp.setTo(255);
			//imshow("mask_t", mask);
			//waitKey();
		}
	}
	if (ifsavemask_t) {
		imwrite(path + "t_mask.png", mask);
	}
	init_gaussian_kernel(Gaussian_kernel_size);
	init_confidence_map();
}


void Texture_Propagation::update_confidence_map(int area_index)
{
	int rows = mask.rows;
	int cols = mask.cols;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			if (area[i][j] != area_index) {
				continue;
			}
			if (mask.at<uchar>(i, j) == 255) {
				confidence_map[i][j] = 1;
			}
			else {
				int distance = Gaussian_kernel_size / 2;
				confidence_map[i][j] = 0;	//initialize it to be 0
				for (int row = i - distance; row < i + distance; row++) {
					if (row<0 || row>rows - 1) continue;
					for (int col = j - distance; col < j + distance; col++) {
						if (col<0 || col>cols - 1) continue;
						confidence_map[i][j] += Gaussian_kernel[row-i+distance][col-j+distance] * (mask.at<uchar>(row, col)/255);
					}
				}
			}
		}
	}
}

void Texture_Propagation::init_confidence_map()
{
	int rows = mask.rows;
	int cols = mask.cols;
	confidence_map = new float*[rows];
	level_map = new float*[rows];
	for (int i = 0; i < rows; i++) {
		confidence_map[i] = new float[cols];
		level_map[i] = new float[cols];
	}
}

void Texture_Propagation::init_gaussian_kernel(int size)
{
	int center = size / 2;		//size=3,5,7,9......
	Gaussian_kernel = new float*[size];
	for (int i = 0; i < size; i++) {
		Gaussian_kernel[i] = new float[size];
	}
	double sum = 0;
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			Gaussian_kernel[i][j] = gaussion_x_y(i, j, center, center);
			sum += Gaussian_kernel[i][j];
		}
	}
	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			Gaussian_kernel[i][j] /= sum;
		}
	}
}

float Texture_Propagation::gaussion_x_y(float x, float y, float x0, float y0, float sigma)
{
	return (1 / (2 * PI*sigma*sigma))*exp(-((x - x0)*(x - x0) + (y - y0)*(y - y0)) / (2 * sigma*sigma));
}




void Texture_Propagation::cal_level_map(int area)
{
	
}






