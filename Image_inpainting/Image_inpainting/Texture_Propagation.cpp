#include"Texture_Propagation.h"
#include"debug.h"
#include"math_function.h"
#include<limits>
#include<ctime>

float **level_map;		//to determine the set of candidate positions

Texture_Propagation::Texture_Propagation(Structure_propagation * p)
{
	sp = p;
	srcImg = p->image.image_inpainted.clone();
	mask = p->image.mask.clone();
	resImg = srcImg.clone();
	region = resImg.clone();
	to_show = resImg.clone();
	int curve_size = p->unknown_anchors.size();
	for (int curve_index = 0; curve_index < curve_size; curve_index++) {
		for (int anchor_index = 0; anchor_index < p->unknown_anchors[curve_index].size(); anchor_index++) {
			int  point_index = p->unknown_anchors[curve_index][anchor_index].anchor_point;
			Point2i left_top = p->getLeftTopPoint(point_index, curve_index);
			Point2i right_down = left_top + Point2i(PatchSizeCol, PatchSizeRow);
			Mat tmp = mask(Rect(left_top, right_down));
			tmp.setTo(255);
		}
	}
	//calculate the boundary of the mask area
	mask_bottom = 0;
	mask_top = mask.rows;
	mask_left = mask.cols;
	mask_right = 0;
	for (int row_index = 0; row_index < mask.rows; row_index++) {
		for (int col_index = 0; col_index < mask.cols; col_index++) {
			if (inMask(Point2i(col_index, row_index))) {
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
		if (!inMask(points[point_index])) {
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
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
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
						int tmp = mask.at<uchar>(row, col) == 255;
						confidence_map[i][j] += Gaussian_kernel[row - i + distance][col - j + distance] * tmp;
					}
				}
			}
		}
	}
	if (ifshowConfidencemap) {
		cout <<"confidence_map:" <<endl;
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
	cal_level_map(0);
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

float Texture_Propagation::gaussion_x(float x, float sigma)
{
	return exp(-x*x  / (2 * sigma*sigma)) / (sigma*sqrt(2 * PI));
}

void Texture_Propagation::cal_level_map(int area_index)
{
	int rows = mask.rows;
	int cols = mask.cols;
	double sum = 0;
	vector<float> tmp;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			if (ifpartition) {
				if (area[i][j] != area_index)
					continue;
			}
			sum += confidence_map[i][j];
			tmp.push_back(confidence_map[i][j]);
		}
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
	uniform_real_distribution<> rand(0, sigma/4);//这里偏差大了，改小一些
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			if (ifpartition) {
				if (area[i][j] != area_index)
					continue;
			}
			if (confidence_map[i][j] > average) {
				level_map[i][j] = 0;
			}
			else {
				float biais = rand(gen);
				level_map[i][j] = confidence_map[i][j] + biais;
			}
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
		cout << "average: " << sum / mask.rows / mask.cols << endl;
	}
}

void Texture_Propagation::extend_curve()
{
	vector<vector<Point2i>>& curves = sp->image.curve_points_copy;
	Mat curve = srcImg.clone();
	int curve_num = curves.size();
	for (int i = 0; i < curve_num; i++) {
		if (curves[i].size() > 4) {
			Point2i first_point = curves[i][0];
			Point2i second_point = curves[i][1];
			Point2i tmp1 = second_point - first_point;
			while (inBoundary(first_point - tmp1)) {
				first_point = first_point - tmp1;
				if (area[first_point.y][first_point.x] == -1) {
					break;
				}
				area[first_point.y][first_point.x] = -1;
			}
			Point2i last_point = curves[i][curves[i].size() - 1];
			Point2i se_last_point = curves[i][curves[i].size() - 2];
			Point2i tmp2 = se_last_point - last_point;
			while (inBoundary(last_point - tmp2)) {
				last_point = last_point - tmp2;
				if (area[last_point.y][last_point.x] == -1) {
					break;
				}
				area[last_point.y][last_point.x] = -1;
			//	curve.at<Vec3b>(last_point.y, last_point.x) == Vec3b(0, 255, 0);
			//	curve.at<Vec3b>(last_point.y, last_point.x-1) == Vec3b(0, 255, 0);
			//	curve.at<Vec3b>(last_point.y, last_point.x + 1) == Vec3b(0, 255, 0);
			//	imshow("curve", curve);
			//	waitKey();
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
		int curve_size = sp->unknown_anchors.size();
		for (int curve_index = 0; curve_index < curve_size; curve_index++) {
			for (int anchor_index = 0; anchor_index < sp->unknown_anchors[curve_index].size(); anchor_index++) {
				int  point_index = sp->unknown_anchors[curve_index][anchor_index].anchor_point;
				Point2i left_top = sp->getLeftTopPoint(point_index, curve_index);
				Point2i right_down = left_top + Point2i(PatchSizeCol, PatchSizeRow);
				for (int i = left_top.y+1; i < right_down.y; i++) {
					for (int j = left_top.x+1; j < right_down.x; j++) {
						area[i][j] = -1;
					}
				}
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
		int *final_area = new int[area_num + 1];
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
						if ((int)final_area[area_index] > (int) *it) {
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
					int area_index = area[row_index][col_index];
					area[row_index][col_index] = final_area[area_index];
				}
			}
		}
		//show_partition();
		int count = 0;
		for (int i = 1; i <= area_num; i++) {
			if (final_area[i] > count) {
				int last_index = final_area[i];
				final_area[i] = ++count;
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
	area_size = vector<int>(area_num + 1, 0);
	area_unknown_size.assign(area_size.begin(), area_size.end());
	for (int row_index = 0; row_index < mask.rows; row_index++) {
		for (int col_index = 0; col_index < mask.cols; col_index++) {
			int area_index = area[row_index][col_index];
			if (area_index > 0) {
				area_size[area_index]++;
				if (inMask(Point2i(col_index, row_index))) {
					area_unknown_size[area_index]++;
				}
			}
				
		}
	}
	//show_partition();
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
			if (index == -1) {
				partition.at<Vec3b>(i, j) = Vec3b(0, 0, 0);
			}
			else {
				partition.at<Vec3b>(i, j) = Vec3b(B[index], G[index], R[index]);
			}
			
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
			////固定分配
			Point p(col, row);
			if (inMask(p)) {
				original_pixel_map[row][col] = Point2i(-1,-1);
			}
			else {
				original_pixel_map[row][col] = p;
			}
			//下面是随机分配策略
			//int x, y;
			//do {
			//	x = rand() % mask.cols;
			//	y = rand() % mask.rows;
			//} while (area[y][x] != area[row][col] || inMask(Point2i(x, y)));
			//original_pixel_map[row][col] = Point2i(x, y);
		}
	}
	if (ifshowOriginal_map) {
		for (int row = 0; row < mask.rows; row++) {
			cout << endl;
			for (int col = 0; col < mask.cols; col++) {
				cout << original_pixel_map[row][col];			
			}
		}
	}
	cout << endl;
}

void Texture_Propagation::show_patch(Point2i unknown,Point2i candidate)
{
	//int radius = 10;
	//Point2i left_top = p - Point2i(radius, radius);
	//Point2i right_down = left_top + Point2i(radius*2, radius*2);
	//Rect rec(left_top, right_down);
	Mat show = resImg.clone();
	show.at<Vec3b>(unknown.y, unknown.x) = Vec3b(0, 255, 0);
	show.at<Vec3b>(candidate.y, candidate.x) = Vec3b(0, 255, 0);
	
	//show.at<Vec3b>(candidate.y-1, candidate.x-1) = Vec3b(0, 0, 255);
	//show.at<Vec3b>(candidate.y, candidate.x - 1) = Vec3b(0, 0, 255);
	//show.at<Vec3b>(candidate.y - 1, candidate.x) = Vec3b(0, 0, 255);
	//show.at<Vec3b>(candidate.y + 1, candidate.x + 1) = Vec3b(0, 0, 255);
	//show.at<Vec3b>(candidate.y  , candidate.x + 1) = Vec3b(0, 0, 255);
	//show.at<Vec3b>(candidate.y + 1, candidate.x ) = Vec3b(0, 0, 255);
	//show.at<Vec3b>(candidate.y + 1, candidate.x - 1) = Vec3b(0, 0, 255);
	//show.at<Vec3b>(candidate.y - 1, candidate.x + 1) = Vec3b(0, 0, 255);

	imshow("patch", show);
	waitKey();
}

void Texture_Propagation::show_mask()
{
	for (int i = 0; i < mask.rows; i++) {
		for (int j = 0; j < mask.cols; j++) {
			int  c = mask.at<uchar>(i, j);
			cout << c << " " << " ";
		}
		cout << endl;
	}
}

vector<Point2i> Texture_Propagation::get_unknown_points(int area_index)
{
	//show_partition();
	vector<Point2i>points;
	for (int row_index = mask_top; row_index <= mask_bottom; row_index++) {
		for (int col_index = mask_left; col_index <= mask_right; col_index++) {
			if (area[row_index][col_index] != -1 && area[row_index][col_index] != area_index) {
				continue;
			}
			if (inMask(Point2i(col_index, row_index))) {
				if (area[row_index][col_index] == area_index) {
					points.push_back(Point2i(col_index, row_index));
				}
				else if (area[row_index][col_index] == -1) {
					if (area[row_index + 1][col_index] != -1) {
						area[row_index][col_index] = area[row_index + 1][col_index];
						points.push_back(Point2i(col_index, row_index));
					}
					else if (area[row_index - 1][col_index] != -1) {
						area[row_index][col_index] = area[row_index - 1][col_index];
						points.push_back(Point2i(col_index, row_index));
					}
					else if (area[row_index][col_index + 1] != -1) {
						area[row_index][col_index] = area[row_index][col_index + 1];
						points.push_back(Point2i(col_index, row_index));
					}
					else if (area[row_index][col_index - 1] != -1) {
						area[row_index][col_index] = area[row_index][col_index - 1];
						points.push_back(Point2i(col_index, row_index));
					}
				}
			}
		}
	}
	return points;
}

bool Texture_Propagation::fill_one_pixel(Point2i unknown_point, int area_index)
{
	vector<Point2i>neighbors;
	vector<Point2i>candidates;
	int size_neighbors = sizeof_neighborhood/2+1;
	int i = 0;		//the times that find the candidate of the unknown point
	Point2i best_candidate(-1, -1);
	do {
		i++;
		size_neighbors = size_neighbors * 2 - 1;
		candidates.clear();
		neighbors.clear();
		get_candidates(unknown_point, neighbors, candidates, area_index, size_neighbors);
		best_candidate = get_best_candidate(unknown_point, candidates);
		cout << "find the unknown_point" << unknown_point << "for" << int_to_string(i) << "times" << endl;
		if (i > 4)
			return false;
	} while (best_candidate == Point2i(-1, -1) || candidates.size() ==0);

	if (resImg.at<Vec3b>(best_candidate.y, best_candidate.x) == Vec3b(0, 0, 255)) {
		cout << unknown_point << "fuck it" << best_candidate << endl;
	}
	//copy the pixel

	resImg.at<Vec3b>(unknown_point.y, unknown_point.x) = resImg.at<Vec3b>(best_candidate.y, best_candidate.x);

	//show_patch(unknown_point,best_candidate);
	//update the original_pixel_map and the mask
	original_pixel_map[unknown_point.y][unknown_point.x] = best_candidate;
	mask.at<uchar>(unknown_point.y, unknown_point.x) = 255;
	//show_mask();
	if (i > 1) {
		if(i<3)
			region.at<Vec3b>(unknown_point.y, unknown_point.x) = Vec3b(255, 0, 0);
		else
			region.at<Vec3b>(unknown_point.y, unknown_point.x) = Vec3b(255, 255, 67);
	}
	else{
		region.at<Vec3b>(unknown_point.y, unknown_point.x) = Vec3b(0, 255, 0);
	}

	imshow("region", region);
	imshow("to_show", resImg);
	waitKey(1);
	return true;
}

void Texture_Propagation::get_candidates(Point2i unknown_point, vector<Point2i> &neighbors,vector<Point2i>&candidates,int area_index,int size)
{
	//L-shaped neighbors
	int point_row = unknown_point.y;
	int point_col = unknown_point.x;
	int distance = size / 2;

	for (int row_index = point_row- distance; row_index <= point_row + distance; row_index++) {
		for (int col_index = point_col - distance; col_index <= point_col + distance; col_index++) {
			if (row_index == point_row&&col_index==point_col) {
				continue;
			}
			Point2i neighbor_point(col_index, row_index);
			//the neighbor must be valid 
			if (inBoundary(neighbor_point) && !inMask(neighbor_point)) {
				if (area[row_index][col_index] == area_index) {
					neighbors.push_back(neighbor_point);
					Point2i original_point = original_pixel_map[row_index][col_index];
					Point2i candidate(-1, -1);
					if (size != sizeof_neighborhood)
						candidate = original_point;
					else
						candidate = original_point + unknown_point - neighbor_point;
					//judge if the candidate is in the right area and in boundary
					if (inBoundary(candidate) && !inMask(candidate)) {
						if (area[candidate.y][candidate.x] == area_index) {
							if (find(candidates.begin(), candidates.end(), candidate) == candidates.end()) {
								candidates.push_back(candidate);
							}
						}
					}
				}
			}

		}
	}
}

Point2i Texture_Propagation::get_best_candidate(Point2i unknown_point,vector<Point2i>& candidates)
{
	float min_L2_difference = FLT_MAX;
	Point2i best_candidate(-1, -1);
	vector<Point2i>::iterator it;
	int point_row = unknown_point.y;
	int point_col = unknown_point.x;

	for (it = candidates.begin(); it != candidates.end(); it++) {
		int count = 0;
		float ratio_sum = 0;
		Point2i candidate_point = *it;
		int distance = sizeof_neighborhood / 2;
		float L2_difference = 0;
		bool has_common_original = false;
		int row_r = 0;
		int col_r = 0;
		bool is_equal = false;
		for (int row_index =  - distance; row_index <= distance; row_index++) {
			for (int col_index =  - distance; col_index <= distance; col_index++) {
				if (row_index == 0 && col_index==0) {
					continue;
				}				
				Point2i unknown_neighbor_point(col_index + unknown_point.x, row_index + unknown_point.y);
				Point2i candidate_neighbor_point(col_index + candidate_point.x, row_index + candidate_point.y);
				if (!inBoundary(unknown_neighbor_point) || !inBoundary(candidate_neighbor_point) || inMask(unknown_neighbor_point) || inMask(candidate_neighbor_point)) {
					break;
				}
				//for debug
				Point2i original_point = original_pixel_map[unknown_neighbor_point.y][unknown_neighbor_point.x];				
				if (original_point == candidate_neighbor_point) {
					has_common_original = true;
					row_r = row_index;
					col_r = col_index;
					if (resImg.at<Vec3b>(unknown_neighbor_point.y, unknown_neighbor_point.x) == resImg.at<Vec3b>(candidate_neighbor_point.y, candidate_neighbor_point.x)) {
						is_equal = true;
					}
				}				
				//the two corresponding points both exist
				count++;

				uchar u_R = srcImg.at<Vec3b>(unknown_neighbor_point.y, unknown_neighbor_point.x)[2];
				uchar u_G = srcImg.at<Vec3b>(unknown_neighbor_point.y, unknown_neighbor_point.x)[1];
				uchar u_B = srcImg.at<Vec3b>(unknown_neighbor_point.y, unknown_neighbor_point.x)[0];

				uchar c_R = srcImg.at<Vec3b>(candidate_neighbor_point.y, candidate_neighbor_point.x)[2];
				uchar c_G = srcImg.at<Vec3b>(candidate_neighbor_point.y, candidate_neighbor_point.x)[1];
				uchar c_B = srcImg.at<Vec3b>(candidate_neighbor_point.y, candidate_neighbor_point.x)[0];
				//normalized distance
				float gaussian_d = sqrt((row_index*row_index + col_index * col_index) / (distance*distance));
				float ratio = gaussion_x(gaussian_d);
				ratio_sum += ratio;
				L2_difference += sqrt((u_R - c_R)*(u_R - c_R) + (u_B - c_B)*(u_B - c_B) + (u_G - c_G)*(u_G - c_G))*ratio;
			}
		}
		//if (count < SIZEOFNEIGHBORHOOD*SIZEOFNEIGHBORHOOD/4) {
		//	return Point2i(-1, -1);
		//}

		//error
		if (L2_difference == 0 && candidates.size()!=0) {
			//cout << "error: the two points have no neighbor in common   " << "unknown_point:" << unknown_point << " candidate_point:" << candidate_point << endl;
			//cout << "unknown:" << inMask(unknown_point) << " candidate:" << inMask(candidate_point) << endl;
			//cout << "unknown:" << resImg.at<Vec3b>(unknown_point.y, unknown_point.x) << "candidate:" << resImg.at<Vec3b>(candidate_point.y, candidate_point.x) << endl;
			//show_patch(unknown_point, candidate_point);
			//for (int row_index = -distance; row_index <= distance; row_index++) {
			//	for (int col_index = -distance; col_index <= distance; col_index++) {
			//		if (row_index == 0 && col_index == 0) {
			//			continue;
			//		}
			//		Point2i unknown_neighbor_point(col_index + unknown_point.x, row_index + unknown_point.y);
			//		Point2i candidate_neighbor_point(col_index + candidate_point.x, row_index + candidate_point.y);
			//		if (inBoundary(unknown_neighbor_point) && inBoundary(candidate_neighbor_point)) {
			//			Point2i original_point = original_pixel_map[unknown_neighbor_point.y][unknown_neighbor_point.x];
			//			bool equal = original_point == candidate_neighbor_point;
			//			cout <<unknown_neighbor_point<<inMask(unknown_neighbor_point)<< original_point << inMask(unknown_neighbor_point) << candidate_neighbor_point << inMask(candidate_neighbor_point)<<"is equal:"<<equal << endl;

			//		}
			//	}
			//}
			//cout << endl;
		}
		else {
			//nomalized
			L2_difference /= ratio_sum;
			if (L2_difference < min_L2_difference) {
				min_L2_difference = L2_difference;
				best_candidate = candidate_point;
			}
		}
	}

	if (best_candidate == Point2i(-1, -1) ) {
		cout << "Warnning: no best candidate" << endl;	
	}
	return best_candidate;
}

void Texture_Propagation::synthesize_area_texture(int area_index)
{
	int radio_size=area_unknown_size[area_index] * level_set_radio;
	vector<Point2i>unknown_points = get_unknown_points(area_index);
	while (unknown_points.size() > 0) {
		//select one pixel to find its original pixel
		bool filled_it = true;
		int count = 0;
		unknown_points = get_unknown_points(area_index);
		cout << "rest of unknown points:" << unknown_points.size()<< endl;
		if (unknown_points.size() <= 0) {
			break;
		}
		cal_level_map(area_index);
		//the 0~candidate-1 th point in the unknown_points will be tht candidate to be fill
		//int candidate_size = radio_size > unknown_points.size() ? unknown_points.size() : radio_size;
		vector<l_point>temp;
		for (int i = 0; i < unknown_points.size(); i++) {
			l_point lp(unknown_points[i]);
			temp.push_back(lp);
		}
		sort(temp.begin(), temp.end());
		unknown_points.clear();
		for (int i = 0; i < temp.size(); i++) {
			Point2i point(temp[i].x, temp[i].y);
			unknown_points.push_back(point);
		}
		int candidate_size = unknown_points.size();
		//search the original pixel of the unknown points
		for (int point_index = 0; point_index < candidate_size; point_index++) {
			Point2i unknown_point = unknown_points[point_index];
			//to judge if the pixel has been filled successfullly
  			filled_it=fill_one_pixel(unknown_point, area_index);
			if(filled_it==false)
				count++;
		}
		update_confidence_map(area_index, unknown_points);
		if (count > 0) {
			cout << "failed to fill " << int_to_string(count) << " points" << endl;
		}
	}
}

void Texture_Propagation::synthesize_texture()
{
	init_gaussian_kernel(Gaussian_kernel_size);
	init_confidence_map();
	partition();				//partition the whole picture
	show_partition_image();
	init_original_pixel_map();	//init the original pixel map
	
	for (int area_index = 1; area_index <= area_num; area_index++) {
		synthesize_area_texture(area_index);
		imwrite(path+"syn_texture"+int_to_string(area_index)+".png", resImg);
	}
	//resImg = natGenerate(resImg, mask, 3);
	imshow("syn_texture", resImg);
	imwrite(path+"syn_texture.png", resImg);
}
