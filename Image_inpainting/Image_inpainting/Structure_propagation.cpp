#include"Structure_propagation.h"
#include"math_function.h"


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

	float result = calcuDistance(ci, cxi) + calcuDistance(cxi, ci);
	//normalized the sum of the shortest distance
	result /= (unknown.end_point - unknown.begin_point + 1);

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

	return KS * calcuEs(unknown, sample, curve_index) + KI * calcuEi(unknown, sample, curve_index);
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

	return result;
}

vector<int> Structure_propagation::DP(vector<AnchorPoint>unknown, vector<AnchorPoint>sample, int curve_index) {

	int unknown_size = unknown.size();
	int sample_size = sample.size();

	if (unknown_size == 0 || sample_size == 0) {
		cout << "In DP: the size of vector AnchorPoint is 0" << endl;
		throw exception();
	}

	float **M = new float*[unknown_size];
	int **last_point = new int*[unknown_size];
	for (int i = 0; i < unknown_size; i++) {
		M[i] = new float[sample_size];
		last_point[i] = new int[sample_size];
	}
	//initialize M[0]
	for (int i = 0; i < sample_size; i++) {
		M[0][i] = calcuE1(unknown[0], sample[i], curve_index);
	}
	//calculate the M[i][j]
	for (int i = 1; i < unknown_size; i++) {
		for (int j = 0; j < sample_size; j++) {
			float min = FLT_MAX;
			int min_index = 0;
			float E1 = calcuE1(unknown[i], sample[j], curve_index);
			// find the sample anchor t to make the Mi to be mininum
			for (int t = 0; t < sample_size; t++) {
				float tmp = calcuE2(unknown[i - 1], unknown[i], sample[t], sample[j], curve_index) + M[i - 1][t];
				if (tmp < min) {
					min = tmp;
					min_index = t;
				}
			}
			M[i][j] = E1 + min;
			last_point[i][j] = min_index;
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
		for (int i = unknown_size - 1; i > 0; i) {
			last_patch = last_point[i][last_patch];
			label.push_back(last_patch);
		}
	}

	reverse(label.begin(), label.end());

	//for debug
	cout << "The min energy of curve " << curve_index << " is " << tmp_min << endl;
	cout << "The size of the sample patch: " << label.size() << endl;
	cout << "DP is done" << endl;
	return label;
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

}
//get all the anchor points on the one curve
void Structure_propagation::getOneCurveAnchors(int curve_index, vector<AnchorPoint>&unknown, vector<AnchorPoint>&sample){
	PointType type;
	int last_index = 0;
	int now_index = 0;
	
	bool flag = true;
	while (true) {

		now_index = getOneAnchorPos(last_index, type, curve_index,flag,unknown,sample);

		if (now_index >= image.curve_points[curve_index].size() - 1)
			break;

		if (last_index != 0) {
			if (flag)
				sample[sample.size() - 1].end_point = now_index-1;
			else
				unknown[unknown.size() - 1].end_point = now_index-1;
		}
		AnchorPoint anchor(last_index+1, now_index, now_index, type);
		if (anchor.type == OUTER){
			sample.push_back(anchor);
			flag = true;
		}
		else {
			unknown.push_back(anchor);
			flag = false;
		}

		last_index = now_index;
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
				sample[count-1].type = BORDER;
			}
			else {
				int count = unknown.size();
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