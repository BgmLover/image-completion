#include"Structure_propagation.h"


double Structure_propagation::calcuE1() {

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

Mat Structure_propagation::getOnePatch(Point2i p) {
	Mat patch;
	Point2i left_top = getLeftTopPoint(p);
	Point2i right_buttom = left_top + Point2i(PatchSizeCol, PatchSizeRow);
	Rect rec(left_top, right_buttom);

	if (left_top.x<0 || left_top.y<0 || right_buttom.x>image.srcImage.cols || right_buttom.y>image.srcImage.rows) {
		cout << "exception:" << left_top << "   " << right_buttom << "when getting one patch" << endl;
		return;
	}
	image.srcImage(rec).copyTo(patch);
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