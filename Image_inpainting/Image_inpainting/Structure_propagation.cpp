#include"Structure_propagation.h"
#include"math_function.h"
#include"debug.h"
//for debug
void Structure_propagation::testOneCurve() {
	imwrite("mask.png", image.mask);
	imwrite("img_mask.png", image.image_masked);
	imwrite("img_inpaint.png", image.image_inpainted);
	getOneNewCurve(unknown_anchors[0], sample_anchors[0], 0, true);
	imshow("Get one curve", image.image_inpainted);
	imshow("srcImg", image.srcImage);
	waitKey();
}

float Structure_propagation::calcuEs(AnchorPoint unknown, AnchorPoint sample, int curve_index) {
	vector<Point2i>ci, cxi;
	//the left_top point of the patches
	Point2i origin_unknown = getLeftTopPoint(unknown.anchor_point, curve_index);
	Point2i origin_sample = getLeftTopPoint(sample.anchor_point, curve_index);

	//to calculate the relative coordinate of the point in the patch
	for (int i = unknown.begin_point; i <= unknown.end_point; i++) {
		Point2i p = image.curve_points[curve_index][i];
		ci.push_back(p - origin_unknown);
	}
	for (int i = sample.begin_point; i <= sample.end_point; i++) {
		Point2i p = image.curve_points[curve_index][i];
		cxi.push_back(p - origin_sample);
	}

	
	int num_ci = unknown.end_point - unknown.begin_point + 1;
	int num_cxi = sample.end_point - sample.begin_point + 1;
	float result = calcuDistance(ci, cxi)/num_ci + calcuDistance(cxi, ci)/num_ci;
	//normalized the sum of the shortest distance
	//result /= (unknown.end_point - unknown.begin_point + 1);

	return result;
}

float Structure_propagation::calcuEi(AnchorPoint unknown, AnchorPoint sample, int curve_index) {
	if (unknown.type != BORDER)
		return 0;
	Mat patch_image, patch_mask;
	Point2i p = image.curve_points[curve_index][unknown.anchor_point];
	getOnePatch(p, image.srcImage).copyTo(patch_image);
	getOnePatch(p, image.mask).copyTo(patch_mask);

	Mat unknown_mask(PatchSizeCol, PatchSizeRow, CV_8U);
	unknown_mask.setTo(0);
	Mat sample_mask(PatchSizeCol, PatchSizeRow, CV_8U);
	sample_mask.setTo(0);
	patch_image.copyTo(unknown_mask, patch_mask);
	Point2i sp = image.curve_points[curve_index][sample.anchor_point];
	getOnePatch(sp, image.srcImage).copyTo(sample_mask, patch_mask);

	return calcuSSD(unknown_mask, sample_mask);
}

float Structure_propagation::calcuE1(AnchorPoint unknown, AnchorPoint sample, int curve_index) {
	//return KS * calcuEs(unknown, sample, curve_index) + KI * calcuEi(unknown, sample, curve_index);
	float  Es = calcuEs(unknown, sample, curve_index);
	float  Ei = calcuEi(unknown, sample, curve_index);
	//cout << "Es=" << KS*Es << "  " << "Ei=" << KI*Ei << endl;
	return KS * Es + KI * Ei;
}


float Structure_propagation::calcuE2(AnchorPoint unknown1, AnchorPoint unknown2, AnchorPoint sample1, AnchorPoint sample2, int curve_index) {
	//get the four vertexes of the two patches
	Point2i ult = getLeftTopPoint(unknown1.anchor_point, curve_index);
	Point2i urb = ult + Point2i(PatchSizeCol, PatchSizeRow);

	Point2i slt = getLeftTopPoint(unknown2.anchor_point, curve_index);
	Point2i srb = slt + Point2i(PatchSizeCol, PatchSizeRow);

	Rect rec1(ult, srb);
	Rect rec2(urb, slt);
	Rect intersect = rec1 & rec2;

	Mat patch1 = getOnePatch(getAnchorPoint(sample1, curve_index), image.srcImage);
	Mat patch2 = getOnePatch(getAnchorPoint(sample2, curve_index), image.srcImage);

	Mat copy1 = image.srcImage.clone();
	Mat copy2 = image.srcImage.clone();
	//overlap the srcimage with the corresponding sample patch
	patch1.copyTo(copy1(Rect(ult, urb)));
	patch2.copyTo(copy2(Rect(slt, srb)));

	float result;
	//calculate the SSD of the overlap parts of two sample patches 
	result = calcuSSD(copy1(intersect), copy2(intersect));
	//for debug
	//cout << "E2=" << result << endl;
	return result;
}

vector<int> Structure_propagation::DP(vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample, int curve_index) {

	int unknown_size = unknown.size();
	int sample_size = sample.size();

	if (unknown_size == 0 || sample_size == 0) {
		cout << "In DP: the size of vector AnchorPoint is 0" << endl;
		throw exception();
	}

	float **M = new float*[unknown_size];
	int **last_point = new int*[unknown_size];
	float **E1 = new float*[unknown_size];//unknonw_size*sample_size

	for (int i = 0; i < unknown_size; i++) {
		M[i] = new float[sample_size];
		last_point[i] = new int[sample_size];
		E1[i] = new float[sample_size];
	}

	
	for (int i = 0; i < unknown_size; i++) {
		for (int j = 0; j < sample_size; j++) {
			E1[i][j] = calcuE1(unknown[i], sample[j], curve_index);
		}
	}
	//initialize M[0]
	for (int i = 0; i < sample_size; i++) {
		M[0][i] = E1[0][i];
	}
	//calculate the M[i][j]
	for (int i = 1; i < unknown_size; i++) {
		for (int j = 0; j < sample_size; j++) {
			float min = FLT_MAX;
			int min_index = 0;
			float E_1 = E1[i][j];
			// find the sample anchor t to make the Mi to be mininum
			for (int t = 0; t < sample_size; t++) {
				float tmp = calcuE2(unknown[i - 1], unknown[i], sample[t], sample[j], curve_index) + M[i - 1][t];
				if (tmp < min) {
					min = tmp;
					min_index = t;
				}
			}
			M[i][j] = E_1 + min;
			last_point[i][j] = min_index;
		}
	}
	//for debug
	if (ifshowDP_E1) {
		cout << "E1[i][j]:" << endl;
		for (int i = 0; i < unknown_size; i++) {
			for (int j = 0; j < sample_size; j++) {
				cout << "(" << i << ',' << j << ")=" << E1[i][j] << "  ";
			}
			cout << endl;
		}
	}
	if (ifshowDP_M) {
		cout << "M[i][j]:" << endl;
		for (int i = 0; i < unknown_size; i++) {
			for (int j = 0; j < sample_size; j++) {
				cout << "(" << i << ',' << j << ")=" << M[i][j] << "  ";
			}
			cout << endl;
		}
	}
	vector<int>label;
	// find the best patch for the last unknown anchor point
	int last_patch = 0;
	float tmp_min = M[unknown_size - 1][0];
	for (int i = 0; i < sample_size; i++) {
		if (M[unknown_size - 1][i] < tmp_min) {
			last_patch = i;
			tmp_min = M[unknown_size - 1][i];
		}
	}
	label.push_back(last_patch);
	//back tracing
	if (unknown_size > 1) {
		for (int i = unknown_size - 1; i > 0; i--) {
			last_patch = last_point[i][last_patch];
			label.push_back(last_patch);
		}
	}

	reverse(label.begin(), label.end());
	for (int i = 0; i < unknown_size; i++) {
		delete[] M[i];
		delete[] last_point[i];
		delete[] E1[i];
	}
	delete[] M;
	delete[] E1;
	delete[] last_point;
	//for debug
	if (ifshowDPlabel) {
		cout << "The min energy of curve " << curve_index << " is " << tmp_min << endl;
		cout << "The size of the sample patch: " << label.size() << endl;
		for (int i = 0; i < label.size(); i++) {
			cout << label[i] << " ";
		}
		cout << endl;
	}

	cout << "DP is done" << endl;

	return label;
}


bool Structure_propagation::isNeighbor(Point2i point1, Point2i point2) {
	return norm(point1 - point2) < norm(Point2i(PatchSizeCol / 2, PatchSizeRow / 2));
}

bool Structure_propagation::isIntersect(int curve1, int curve2) {
	if (unknown_anchors[curve1].empty() || unknown_anchors[curve2].empty()) {
		return false;
	}
	int num_curve1 = unknown_anchors[curve1].size();
	int num_curve2 = unknown_anchors[curve2].size();

	for (int i = 0; i < num_curve1; i++) {
		int p1 = unknown_anchors[curve1][i].anchor_point;
		Point2i point1 = image.curve_points[curve1][p1];
		for (int j = 0; j < num_curve2; j++) {
			int p2 = unknown_anchors[curve2][j].anchor_point;
			Point2i point2 = image.curve_points[curve2][p2];
			if (isNeighbor(point1, point2)) {
				unknown_anchors[curve1][i].neighbors.push_back(j + num_curve1);
				unknown_anchors[curve1][i].neighbors.push_back(j + 1 + num_curve1);
				/*this means that the anchor point i will become the milddle point between j and j+1, so there will no path from j
				to j+1 directly*/
				if (j + 1 < unknown_anchors[curve2].size()) {
					unknown_anchors[curve2][j].neighbors[1] = i - num_curve1;
					unknown_anchors[curve2][j + 1].neighbors[0] = i - num_curve1;
				}
				else {
					unknown_anchors[curve2][j].neighbors.push_back(i - num_curve1);
				}

				return true;
			}
		}
	}
	return false;
}

vector<int> Structure_propagation::BP(vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample, int curve_index) {
	cout << "begin to BP ... ..." << endl;

	vector<int>label;
	int unknown_size = unknown.size();
	int sample_size = sample.size();

	float ***M = new float**[unknown_size];//unknown_times*unknown_size*sample_size
	float **E1 = new float*[unknown_size];//unknonw_size*sample_size

	//initilize the array
	for (int i = 0; i < unknown_size; i++) {
		E1[i] = new float[sample_size];
		M[i] = new float*[unknown_size];
		for (int j = 0; j < unknown_size; j++) {
			M[i][j] = new float[sample_size];
			initArray(M[i][j], sample_size);
		}
	}

	//to calculate the matrix E1 for the convenience of the next calculation
	for (int i = 0; i < unknown_size; i++) {
		for (int j = 0; j < sample_size; j++) {
			E1[i][j] = calcuE1(unknown[i], sample[j], curve_index);
		}
	}

	if (ifshowBP_E1) {
		cout << "E1[i][j]:" << endl;
		for (int i = 0; i < unknown_size; i++) {
			for (int j = 0; j < sample_size; j++) {
				cout << "(" << i << ',' << j << ")=" << E1[i][j] << "  ";
			}
			cout << endl;
		}
	}

	//to judge if the node has been converged
	bool **isConverge = new bool*[unknown_size];
	for (int i = 0; i < unknown_size; i++) {
		isConverge[i] = new bool[unknown_size];
		initArray(isConverge[i], unknown_size);
	}


	float *sum_vec = new float[sample_size];//the sum of vectors from neighbors
	float *E_M_sum = new float[sample_size];//sum_vec-M[j][i]+E[i]
	float *new_vec = new float[sample_size];//the final vector calculated for M[i][j]

	//begin to iterate
	for (int t = 0; t < unknown_size; t++) {
		for (int node = 0; node < unknown_size; node++) {
			//calcaulate the sum of M[t-1][i][j]
			initArray(sum_vec, sample_size);
			for (int neighbor_index = 0; neighbor_index < unknown[node].neighbors.size(); neighbor_index++) {
				//neighbors to node
				addArray(sum_vec, M[neighbor_index][node],sum_vec,sample_size);
			}
			//node to neighbors
			for (int times = 0; times < unknown[node].neighbors.size(); times++) {
				int neighbor_index = unknown[node].neighbors[times];
				if (isConverge[node][neighbor_index] == true) {
					continue;
				}
				minusArray(sum_vec, M[neighbor_index][node], E_M_sum, sample_size);
				addArray(E_M_sum, E1[node], E_M_sum,sample_size);
				
				for (int xj = 0; xj < sample_size; xj++) {
					float min = FLT_MAX;
					for (int xi = 0; xi < sample_size; xi++) {
						float E2 = calcuE2(unknown[node], unknown[neighbor_index], sample[xi], sample[xj], curve_index);
						float sum = E2 + E_M_sum[xi];
						if (sum < min) {
							min = sum;
						}
					}
					new_vec[xj] = min;
				}
				//to judge if the vector has been converged
				bool flag = isEqualArray(M[node][neighbor_index], new_vec,sample_size);
				if (flag) {
					isConverge[node][neighbor_index] = true;
				}
				else {
					moveArray(M[node][neighbor_index], new_vec, sample_size);
				}
			}

		}
	}

	//after iteration,we need to find the optimum label for every node
	for (int i = 0; i < unknown_size; i++) {
		initArray(sum_vec, sample_size);
		addArray(sum_vec, E1[i], sum_vec,sample_size);
		for (int j = 0; j < unknown[i].neighbors.size(); j++) {
			//neighbor to node
			addArray(sum_vec, M[j][i], sum_vec, sample_size);
		}
		//find the min label
		float min = FLT_MAX;
		int label_index = 0;
		for (int i = 0; i < sample_size; i++) {
			if (sum_vec[i] < min) {
				min = sum_vec[i];
				label_index = i;
			}
		}
		label.push_back(label_index);
	}
	delete[] new_vec;
	delete[] sum_vec;
	delete[] E_M_sum;
	for (int i = 0; i < unknown_size; i++) {
		delete[] isConverge[i];
		delete[] E1[i];
		for (int j = 0; j < unknown_size; j++) {
			delete[]M[i][j];
		}
	}
	delete[] isConverge;
	delete[] E1;
	for (int i = 0; i < unknown_size; i++) {
		delete[]M[i];
	}
	delete[] M;
	if (ifshowBPlabel) {
		for (int i = 0; i < label.size(); i++) {
			cout << label[i] << " ";
		}
		cout << endl;
	}
	return label;

}

void Structure_propagation::addNeighborFB(int curve_index) {
	for (int i = 0; i < unknown_anchors[curve_index].size(); i++) {
		if (i - 1 >= 0) {
			unknown_anchors[curve_index][i].neighbors.push_back(i - 1);
		}
		if (i + 1 < unknown_anchors[curve_index].size()) {
			unknown_anchors[curve_index][i].neighbors.push_back(i + 1);
		}
	}
}

void Structure_propagation::mergeCurves(vector<bool>&isSingle) {
	
	int num_curves = image.curve_points.size();
	//initialize the neighbors
	for (int i = 0; i < num_curves; i++) {
		addNeighborFB(i);
	}
	
	//begin to merge
	for (int i = 0; i < num_curves; i++) {
		if (unknown_anchors[i].size() == 0) {
			continue;
		}
		for (int j = i + 1; j < num_curves; j++) {
			if (unknown_anchors[j].size() == 0) {
				continue;
			}
			if (isIntersect(i, j)) {
				isSingle[i] = false;
				isSingle[j] = false;
				//transfer the unknown anchor points
				int num_points = image.curve_points[i].size();
				int num_unknown_size = unknown_anchors[i].size();
				for (int anchor_index = 0; anchor_index < unknown_anchors[j].size(); anchor_index++) {
					unknown_anchors[j][anchor_index].anchor_point += num_points;
					unknown_anchors[j][anchor_index].begin_point += num_points;
					unknown_anchors[j][anchor_index].end_point += num_points;
					//add the anchor point to the end of the first curve
					for (int t = 0; t < unknown_anchors[j][anchor_index].neighbors.size(); t++) {
						unknown_anchors[j][anchor_index].neighbors[t] += num_unknown_size;
					}
					unknown_anchors[i].push_back(unknown_anchors[j][anchor_index]);
				}
				//transfer the sample anchor points
				for (int sample_index = 0; sample_index < sample_anchors[j].size(); sample_index++) {
					sample_anchors[j][sample_index].anchor_point += num_points;
					sample_anchors[j][sample_index].begin_point += num_points;
					sample_anchors[j][sample_index].end_point += num_points;

					sample_anchors[i].push_back(sample_anchors[j][sample_index]);
				}
				//transfer the real points 
				for (int point_index = 0; point_index < image.curve_points[j].size(); point_index++) {
					image.curve_points[i].push_back(image.curve_points[j][point_index]);
				}

				unknown_anchors[j].clear();
				sample_anchors[j].clear();
				image.curve_points[j].clear();
			}
		}
	}
	
}

void Structure_propagation::getOneNewCurve(vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample, int curve_index, bool flag) {
	vector<int>label;
	if (sample.size() == 0) {
		return;
	}
	if (flag) {
		label = DP(unknown, sample, curve_index);
	}
	else {
		label = BP(unknown, sample, curve_index);
	}
	if (unknown.size() != label.size()) {
		cout << endl << "In getOneNewCurve() : The sizes of unknown and label are different" << endl;
		throw exception();
	}
	for (int i = 0; i < unknown.size(); i++) {
		Mat patch = getOnePatch(sample[label[i]], image.srcImage, curve_index);
		copyPatchToImg(unknown[i], patch, image.image_inpainted, curve_index);	
	}
}



//for debug
void Structure_propagation::drawAnchors() {
	Mat showAnchors = image.image_masked.clone();
	for (int i = 0; i < sample_anchors.size(); i++) {
		for (int j = 0; j < sample_anchors[i].size(); j++) {
			Point2i p = image.curve_points[i][sample_anchors[i][j].anchor_point];
			//circle(showAnchors, p, 5, Scalar(255, 255, 0));
			Point2i tmp = getLeftTopPoint(p);
			Rect rec(tmp.x,tmp.y, PatchSizeCol, PatchSizeRow);
			rectangle(showAnchors, rec, Scalar(255, 255, 0));
		}
	}
	for (int i = 0; i < unknown_anchors.size(); i++) {
		for (int j = 0; j < unknown_anchors[i].size(); j++) {
			Point2i p = image.curve_points[i][unknown_anchors[i][j].anchor_point];
			//circle(showAnchors, p, 5, Scalar(255, 255, 0));
			Point2i tmp = getLeftTopPoint(p);
			Rect rec(tmp.x, tmp.y, PatchSizeCol, PatchSizeRow);
			rectangle(showAnchors, rec, Scalar(255, 255, 0));
		}
	}
	imshow("show anchors", showAnchors);
	waitKey();
}

void Structure_propagation::getNewStructure() {
	int curve_size = image.curve_points.size();
	vector<bool>isSingle(curve_size, true);
	//for debug
	if(ifshowMerge){
		for (int i = 0; i < curve_size; i++) {
			cout << "curve_index: " << i << " size:" << unknown_anchors[i].size() << " ," << sample_anchors[i].size() << " ," << image.curve_points[i].size() << endl;
		}
	}
	
	mergeCurves(isSingle);
	if (ifshowMerge) {
		cout << endl << "after merge:" << endl;
		for (int i = 0; i < curve_size; i++) {
			cout << "curve_index: " << i << " size:" << unknown_anchors[i].size() << " ," << sample_anchors[i].size() << " ," << image.curve_points[i].size() << endl;
		}
	}
	
	for (int i = 0; i < curve_size; i++) {
		if (isSingle[i]) {
			if (!unknown_anchors[i].empty())
				getOneNewCurve(unknown_anchors[i], sample_anchors[i], i, true);//DP
		}
		else {
			if(!unknown_anchors[i].empty())
				getOneNewCurve(unknown_anchors[i], sample_anchors[i], i, false);//BP
		}
	}
	imshow("Get one curve", image.image_inpainted);
	if(ifshowsrcImg)
		imshow("srcImg", image.srcImage);
	if (ifsaveinpainted) {
		imwrite(path + "inpainted.png", this->image.image_inpainted);
	}
	waitKey();
}

//get all the anchor points in the image and save them
void Structure_propagation::getAnchors() {
	vector<AnchorPoint>unknown, sample;
	for (int i = 0; i < image.curve_points.size(); i++) {
		getOneCurveAnchors(i, unknown, sample);
		this->unknown_anchors.push_back(unknown);
		this->sample_anchors.push_back(sample);

		sample.clear();
		unknown.clear();
	}
	cout << endl;
	//cout <<"The size of unknown_anchors[0]="<< unknown_anchors[0].size() << endl;
	//for (int i = 0; i < unknown_anchors[0].size(); i++) {
	//	cout << getAnchorPoint(unknown_anchors[0][i],0) << endl;
	//}

}

//get all the anchor points on the one curve
int Structure_propagation::getOneAnchorFront(int lastanchor_index, PointType &t, int curve_index, bool flag, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample) {
	Point2i p = image.curve_points[curve_index][lastanchor_index];
	Rect rec = getRect(p);
	int i = lastanchor_index + 1;
	if (i >= image.curve_points[curve_index].size() - 1) {
		return image.curve_points[curve_index].size() - 1;
	}
	if (image.mask.at<uchar>(image.curve_points[curve_index][i]) == 0) {
		t = INNER;
	}
	else {
		t = OUTER;
	}
	while (i < image.curve_points[curve_index].size() && contain(rec,image.curve_points[curve_index][i])) {
		uchar tmp = image.mask.at<uchar>(image.curve_points[curve_index][i]);
		if (tmp == 0 && t == OUTER || tmp == 255 && t == INNER) {
			t = BORDER;
			if (flag) {
				int count = sample.size();
				if (count>0)
					sample[count - 1].type = BORDER;
			}
			else {
				int count = unknown.size();
				if (count>0)
					unknown[count - 1].type = BORDER;
			}
		}
		i++;
	}
	return i;
}
int Structure_propagation::getOneAnchorBack(int lastanchor_index, PointType &t, int curve_index, bool flag, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample) {
	Point2i p = image.curve_points[curve_index][lastanchor_index];
	Rect rec = getRect(p);
	int i = lastanchor_index - 1;
	t = OUTER;
	while (i >= 0 && contain(rec, image.curve_points[curve_index][i])) {
		i--;
	}
	if (i < 0) {
		return -1;
	}
	return i;
}
void Structure_propagation::getOneCurveAnchors(int curve_index, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample){
	//Point2i unknown_begin;
	int unknown_begin;
	int num_points = image.curve_points[curve_index].size();
	for (int i = 0; i < num_points; i++) {
		if (image.mask.at<uchar>(image.curve_points[curve_index][i]) == 0) {
			unknown_begin = i;
			break;
		}
	}
	PointType type;
	bool flag = true;
	//find all the unknown anchor points
	int now_index, last_index;

	last_index = unknown_begin;
	while (true) {
		if (last_index < 0) {
			break;
		}
		now_index = getOneAnchorBack(last_index, type, curve_index, flag, unknown, sample);
		if (now_index < 0) {
			break;
		}
		if (last_index != unknown_begin) {
				sample[sample.size() - 1].begin_point = now_index + 1;
		}
		AnchorPoint anchor(now_index, last_index-1, now_index, type);
		sample.push_back(anchor);
		last_index = now_index;
	}
	//this point doesn't have enough points in the patch
	if(sample.size()>0){
		sample.pop_back();
		reverse(sample.begin(), sample.end());
	}


	int first_unknown_begin = getOneAnchorBack(unknown_begin, type, curve_index, flag, unknown, sample) + 1;
	now_index = unknown_begin;
	last_index = unknown_begin;
	AnchorPoint anchor(first_unknown_begin, now_index, now_index, BORDER);
	unknown.push_back(anchor);

	while (true) {
		now_index = getOneAnchorFront(now_index, type, curve_index, flag, unknown, sample);
		if (now_index >= image.curve_points[curve_index].size() - 1)
			break;
		if (flag) {
			unknown[unknown.size() - 1].end_point = now_index - 1;
		}
		else {
			sample[sample.size() - 1].end_point = now_index - 1;
		}
		AnchorPoint anchor(last_index + 1, now_index, now_index, type);
		if (anchor.type == OUTER) {
			sample.push_back(anchor);
			flag = false;
		}
		else {
			unknown.push_back(anchor);
			flag = true;
		}
		last_index = now_index;
	}

	if (flag) {
		unknown.pop_back();
	}
	else {
		sample.pop_back();
	}
}

int Structure_propagation::getOneAnchorPos(int lastanchor_index, PointType &t, int curve_index,bool flag, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample){

	Point2i vertex = getLeftTopPoint(lastanchor_index, curve_index);
	Rect rec(vertex.x, vertex.y, PatchSizeCol, PatchSizeRow);

	int i = lastanchor_index+1;//this point will be the begin point in the next patch,so we should judge its type
	if (i >= image.curve_points[curve_index].size() - 1) {
		return image.curve_points[curve_index].size() - 1;
	}
	if (image.mask.at<uchar>(image.curve_points[curve_index][i]) == 0) {
		t = INNER;
	}
	else {
		t = OUTER;
	}

	while (i < image.curve_points[curve_index].size() && rec.contains(image.curve_points[curve_index][i])) {
		uchar tmp = image.mask.at<uchar>(image.curve_points[curve_index][i]);
		if (tmp == 0 && t == OUTER || tmp == 255 && t == INNER) {
			t = BORDER;
			if (flag) {
				int count = sample.size();
				if(count>0)
					sample[count-1].type = BORDER;
			}
			else {
				int count = unknown.size();
				if (count>0)
					unknown[count - 1].type = BORDER;
			}
		}
		i++;
	}
	return i;
}

Mat Structure_propagation::getOnePatch(Point2i p,Mat &img) {
	Mat patch;
	Point2i left_top = getLeftTopPoint(p);
	Point2i right_buttom = left_top + Point2i(PatchSizeCol, PatchSizeRow);
	Rect rec(left_top, right_buttom);

	if (left_top.x<0 || left_top.y<0 || right_buttom.x>img.cols || right_buttom.y>img.rows) {
		cout << "exception:" << left_top << "   " << right_buttom << "when getting one patch" << endl;
		throw exception();
	}
	img(rec).copyTo(patch);
	return patch;
}

Mat Structure_propagation::getOnePatch(AnchorPoint ap, Mat &img, int curve_index) {
	Mat patch;
	Rect rec = getRect(ap, curve_index);
	if (rec.x<0 || rec.y<0 || rec.x + rec.width>img.cols || rec.y + rec.height>img.rows) {
		cout << "exception:" << rec.x << "   " << rec.y << "when getting one patch" << endl;
		throw exception();
	}
	img(rec).copyTo(patch);
	return patch;
}

void Structure_propagation::copyPatchToImg(AnchorPoint unknown, Mat &patch, Mat &img, int curve_index) {
	Rect rec = getRect(unknown, curve_index);
	//need to be correct ,to be done
	patch.copyTo(img(rec));
}

Point2i Structure_propagation::getLeftTopPoint(int point_index, int curve_index) {
	Point2i p = image.curve_points[curve_index][point_index];
	int x = (p.x - PatchSizeCol / 2) > 0 ? p.x - PatchSizeCol / 2 : 0;
	int y = (p.y - PatchSizeRow / 2) > 0 ? p.y - PatchSizeRow / 2 : 0;
	return Point2i(x, y);
}

Point2i Structure_propagation::getLeftTopPoint(Point2i p) {

	int x = (p.x - PatchSizeCol / 2) > 0 ? p.x - PatchSizeCol / 2 : 0;
	int y = (p.y - PatchSizeRow / 2) > 0 ? p.y - PatchSizeRow / 2 : 0;
	return Point2i(x, y);
}

Point2i Structure_propagation::getAnchorPoint(AnchorPoint ap, int curve_index) {
	return image.curve_points[curve_index][ap.anchor_point];
}

Rect Structure_propagation::getRect(AnchorPoint ap, int curve_index) {
	Point2i p = image.curve_points[curve_index][ap.anchor_point];
	Point2i left_top = p - Point2i(PatchSizeCol/2, PatchSizeRow/2);
	Point2i right_down = left_top + Point2i(PatchSizeCol , PatchSizeRow);
	return Rect(left_top, right_down);
}

Rect Structure_propagation::getRect(Point2i p) {
	Point2i left_top = p - Point2i(PatchSizeCol / 2, PatchSizeRow / 2);
	Point2i right_down = left_top + Point2i(PatchSizeCol, PatchSizeRow);
	return Rect(left_top, right_down);
}