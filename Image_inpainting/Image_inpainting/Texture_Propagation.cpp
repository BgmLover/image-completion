#include"Texture_Propagation.h"
#include"debug.h"
#include"math_function.h"
#include<set>
#include<limits>
/*
纹理生成部分，记得在候选人部分去除重复的候选人
5*5 neighborhood
*/


Texture_Propagation::Texture_Propagation(Structure_propagation * p)
{
	sp = p;
	srcImg = p->image.image_inpainted.clone();
	mask = p->image.mask.clone();
	resImg = srcImg.clone();

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
	//calculate the boundary of the mask area
	mask_bottom = 0;
	mask_top = mask.rows;
	mask_left = mask.cols;
	mask_right = 0;
	for (int row_index = 0; row_index < mask.rows; row_index++) {
		for (int col_index = 0; col_index < mask.cols; col_index++) {
			if (inMask(Point2i(row_index,col_index))) {
				mask_bottom = row_index > mask_bottom ? row_index : mask_bottom;
				mask_top = row_index < mask_top ? row_index : mask_top;
				mask_left = col_index < mask_left ? col_index : mask_left;
				mask_right = col_index > mask_right ? col_index : mask_right;
			}
		}
	}
	if (ifsavemask_t) {
		imwrite(path + "t_mask.png", mask);
	}
}


void Texture_Propagation::update_confidence_map(int area_index, vector<Point2i>&points)
{
	int rows = mask.rows;
	int cols = mask.cols;
	int num_points = points.size();
	for (int point_index = 0; point_index < num_points; point_index++) {
		int row_index = points[point_index].y;
		int col_index = points[point_index].x;
		if (inMask(points[point_index])) {
			confidence_map[row_index][col_index] = 1;
		}
		else {
			int distance = Gaussian_kernel_size / 2;
			confidence_map[row_index][col_index] = 0;	//initialize it to be 0
			for (int row = row_index - distance; row < row_index + distance; row++) {
				if (row<0 || row>rows - 1) continue;
				for (int col = col_index - distance; col < col_index + distance; col++) {
					if (col<0 || col>cols - 1) continue;
					int tmp = mask.at<uchar>(row, col) == 255;
					confidence_map[row_index][col_index] += Gaussian_kernel[row - row_index + distance][col - col_index + distance] * tmp;
				}
			}
		}
	}


	
	if (ifshowConfidencemap) {
		cout << endl;
		double sum = 0;
		for (int i = 0; i < mask.rows; i++) {
			for (int j = 0; j < mask.cols; j++) {
				cout << confidence_map[i][j] << "  ";
				sum += confidence_map[i][j];
			}
			cout << endl;
		}
		cout << "average: " << sum / mask.rows / mask.cols << endl;
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

bool Texture_Propagation::compare(Point2i i, Point2i j)
{
	return level_map[i.y][i.x] > level_map[j.y][j.x];
}

void Texture_Propagation::cal_level_map(int area_index, vector<Point2i>&points)
{
	int rows = mask.rows;
	int cols = mask.cols;
	int num_points = points.size();
	//calculate the sigma 
	double sum = 0;
	vector<float> tmp;
	for (int point_index = 0; point_index < num_points; point_index++) {
		int row_index = points[point_index].y;
		int col_index = points[point_index].x;
		sum += confidence_map[row_index][col_index];
		tmp.push_back(confidence_map[row_index][col_index]);
	}

	double average = sum / tmp.size();
	double sigma = 0;
	for (int i = 0; i < tmp.size(); i++) {
		sigma += (tmp[i] - average)*(tmp[i] - average);
	}
	sigma /= tmp.size();
	sigma = sqrt(sigma);

	
	random_device rd;
	mt19937 gen(rd());
	uniform_real_distribution<> rand(0, sigma);
	//we select 20% of the max value points as the candidates
	
	for (int point_index = 0; point_index < num_points; point_index++) {
		int row_index = points[point_index].y;
		int col_index = points[point_index].x;
		if (confidence_map[row_index][col_index] > average) {
			level_map[row_index][col_index] = 0;
		}
		else {
			level_map[row_index][col_index] = confidence_map[row_index][col_index] + rand(gen);
		}
	}

	if (ifshowlevelmap) {
		cout << endl;
		cout << "sigma=" << sigma << endl;
		double sum = 0;
		for (int i = 0; i < mask.rows; i++) {
			for (int j = 0; j < mask.cols; j++) {
				cout << level_map[i][j] << "  ";
				sum += level_map[i][j];
			}
			cout << endl;
		}
		cout << "average level_map value in the whole picture: " << sum / mask.rows / mask.cols << endl;
	}
}

void Texture_Propagation::extend_curve()
{
	vector<vector<Point2i>>& curves = sp->image.curve_points_copy;
	int curve_num = curves.size();
	for (int i = 0; i < curve_num; i++) {
		if (curves[i].size() > 4) {
			Point2i first_point = curves[i][0];
			Point2i second_point = curves[i][1];
			Point2i tmp1 = second_point - first_point;
			while (inBoundary(first_point - tmp1)) {
				first_point = first_point - tmp1;
				area[first_point.y][first_point.x] = -1;
			}
			Point2i last_point = curves[i][curves[i].size() - 1];
			Point2i se_last_point= curves[i][curves[i].size() - 2];
			Point2i tmp2 = se_last_point - last_point;
			while (inBoundary(last_point - tmp2)) {
				last_point = last_point - tmp2;
				area[last_point.y][last_point.x] = -1;
			}
		}
	}
}

bool Texture_Propagation::inBoundary(Point2i p)
{
	if (p.x >= 0 && p.x <= mask.cols - 1 && p.y >= 0 && p.y <= mask.rows - 1) {
		return true;
	}
	return false;
}

bool Texture_Propagation::inMask(Point2i p)
{
	int col = p.x;
	int row = p.y;
	if (mask.at<uchar>(row, col) == 0) {
		return true;
	}
	return false;
}



void Texture_Propagation::partition()
{	
	//init the area
	area = new int*[mask.rows];
	for (int i = 0; i < mask.rows; i++) {
		area[i] = new int[mask.cols];
		initArray(area[i], mask.cols);
	}
	//partition the area by the user
	if (ifUserPartition) {
		
	}
	//partition the area by the user specified curves
	else {
		//mark the curves to belong to area -1
		vector<vector<Point2i>>& curves = sp->image.curve_points_copy;
		int curve_num = curves.size();
		for (int i = 0; i < curve_num; i++) {
			int curve_i_num = curves[i].size();
			for (int j = 0; j < curve_i_num; j++) {
				area[curves[i][j].y][curves[i][j].x] = -1;
			}
		}
		//extend the curves to partition the area
		extend_curve();
		//show_partition();

		vector<set<int>>diff_areas;
		for (int row_index = 0; row_index < mask.rows; row_index++) {
			for (int col_index = 0; col_index < mask.cols; col_index++) {
				if (area[row_index][col_index] == -1) {
					continue;
				}
				if (area[row_index][col_index] == 0) {
					int up_area = row_index > 0 ? area[row_index - 1][col_index] : 0;
					int left_area = col_index > 0 ? area[row_index][col_index - 1] : 0;
					if (up_area <= 0 && left_area <= 0) {
						area_num++;
						area[row_index][col_index] = area_num;
					}
					else if (up_area <= 0 && left_area > 0) {
						area[row_index][col_index] = left_area;
					}
					else if (up_area > 0 && left_area <= 0) {
						area[row_index][col_index] = up_area;
					}
					else if (up_area == left_area && up_area > 0) {
						area[row_index][col_index] = up_area;
					}
					else if (up_area > 0 && left_area > 0 && up_area != left_area) {
						int i = 0;
						for (; i < diff_areas.size(); i++) {
							if (diff_areas[i].find(up_area) != diff_areas[i].end()) {
								diff_areas[i].insert(left_area);
								break;
							}
							else if (diff_areas[i].find(left_area) != diff_areas[i].end()) {
								diff_areas[i].insert(up_area);
								break;
							}
						}
						if (i == diff_areas.size()) {
							set<int>new_area;
							new_area.insert(up_area);
							new_area.insert(left_area);
							diff_areas.push_back(new_area);
						}
						area[row_index][col_index] = up_area;
					}

				}
			}
		}
		//show_partition();
		int *final_area = new int[area_num+1];
		final_area[0] = 0;
		for (int area_index = 1; area_index <= area_num; area_index++) {
			//init the area_array to be the corresponding area index
			final_area[area_index] = area_index;
			//find the area index in all sets
			for (int set_index = 0; set_index < diff_areas.size(); set_index++) {
				if (diff_areas[set_index].find(area_index) != diff_areas[set_index].end()) {
					set<int>::iterator it;
					//find the smallest value in the set
					for (it = diff_areas[set_index].begin(); it != diff_areas[set_index].end(); it++) {
						if ((int)final_area[area_index] >(int) *it) {
							final_area[area_index] = *it;
						}
					}
					break;
				}
			}
		}

		for (int row_index = 0; row_index < mask.rows; row_index++) {
			for (int col_index = 0; col_index < mask.cols; col_index++) {
				if (area[row_index][col_index] == -1) {
					continue;
				}
				else {
					int area_index=area[row_index][col_index];
					area[row_index][col_index] = final_area[area_index];
				}
			}
		}
		//show_partition();
		int count = 0;
		for (int i = 1; i <= area_num; i++) {
			if (final_area[i] > count) {
				int last_index = final_area[i];				
				final_area[i]=++count;
				int j = i + 1;
				while (j <= area_num) {
					if (final_area[j] == last_index) {
						final_area[j] = count;
					}
					j++;
				}
			}
		}
		this->area_num = count;
		for (int row_index = 0; row_index < mask.rows; row_index++) {
			for (int col_index = 0; col_index < mask.cols; col_index++) {
				if (area[row_index][col_index] == -1) {
					continue;
				}
				else {
					int area_index = area[row_index][col_index];
					area[row_index][col_index] = final_area[area_index];
				}
			}
		}
		//show_partition();
		//cout << endl;
	}
	ifpartition = true;
	//statistic the number of each area
	area_size = vector<int>(area_num + 1,0);
	for (int row_index = 0; row_index < mask.rows; row_index++) {
		for (int col_index = 0; col_index < mask.cols; col_index++) {
			int area_index = area[row_index][col_index];
			area_size[area_index]++;
		}
	}
	area_unknown_size.assign(area_size.begin(), area_size.end());
}


void Texture_Propagation::show_partition()
{
	for (int i = 0; i < mask.rows; i++) {
		cout << endl;
		for (int j = 0; j < mask.cols; j++) {
			cout << area[i][j] << " ";
		}
	}
	cout << endl;
}

void Texture_Propagation::show_partition_image()
{
	Mat partition(mask.size(), CV_8UC3);
	int *R, *G, *B;
	R = new int[area_num];
	G = new int[area_num];
	B = new int[area_num];
	for (int i = 0; i < area_num; i++) {
		R[i] = rand() % 255;
		G[i] = rand() % 255;
		B[i] = rand() % 255;
	}
	for (int i = 0; i < partition.rows; i++) {
		for (int j = 0; j < partition.cols; j++) {
			int index = area[i][j];
			partition.at<Vec3b>(i, j) = Vec3b(B[index],G[index],R[index]);
		}
	}
	imwrite(path + "partition.png", partition);
	delete[] R, G, B;
}



void Texture_Propagation::init_original_pixel_map()
{
	original_pixel_map = new Point2i*[mask.rows];
	for (int i = 0; i < mask.rows; i++) {
		original_pixel_map[i] = new Point2i[mask.cols];
	}
	for (int row = 0; row < mask.rows; row++) {
		for (int col = 0; col < mask.cols; col++) {
			if (inMask(Point2i(row, col))) {
				original_pixel_map[row][col] = Point2i(-1, -1);
			}
			else {
				original_pixel_map[row][col] = Point2i(row, col);
			}
		}
	}
}

vector<Point2i> Texture_Propagation::get_unknown_points(int area_index)
{
	vector<Point2i>points;
	for (int row_index = mask_top; row_index <= mask_bottom; row_index++) {
		for (int col_index = mask_left; col_index <= mask_right; col_index++) {
			if (area[row_index][col_index] == area_index) {
				if (inMask(Point2i(row_index, col_index))) {
					points.push_back(Point2i(row_index, col_index));
				}
			}
		}
	}
	return points;
}

void Texture_Propagation::synthesize_area_texture(int area_index)
{	
	while (area_unknown_size[area_index] > 0) {
		//select one pixel to find its original pixel
		vector<Point2i>unknown_points;
		update_confidence_map(area_index,unknown_points);
		cal_level_map(area_index, unknown_points);
		int candidate_size = area_size[area_index] * level_set_radio;
		candidate_size = candidate_size > unknown_points.size() ? unknown_points.size() : candidate_size;
		//the 0~candidate-1 th point in the unknown_points will be tht candidate to be fill
		sort(unknown_points.begin(), unknown_points.end(), compare);
		
	}


}

void Texture_Propagation::synthesize_texture()
{
	init_gaussian_kernel(Gaussian_kernel_size);
	init_confidence_map();
	partition();				//partition the whole picture
	init_original_pixel_map();	//init the original pixel map

	for (int i = 1; i <= area_num; i++) {
		synthesize_area_texture(i);
	}

}
