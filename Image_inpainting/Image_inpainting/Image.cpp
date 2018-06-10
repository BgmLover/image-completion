#include"Image.h"
#include<string>
#include"debug.h"

static int mouse_event;
static Point2i mouse_pos;
static int mouse_flags;

void on_MouseHandle(int event, int x, int y, int flags, void *param) {
	mouse_event = event;
	mouse_pos = Point2i(x, y);
	mouse_flags = flags;
}
Image::Image(Mat src)
{
	srcImage = src.clone();
	image_masked = src.clone();
	image_inpainted = src.clone();
	mask = Mat::zeros(src.size(), CV_8U);
	mask.setTo(255);
}
void Image::getMask() {
	Mat display = this->srcImage.clone();
	const  string name = "Get the mask";
	imshow(name, display);
	setMouseCallback(name, on_MouseHandle);

	int size = MASK_SIZE;

	Point2i last_pos(-1, -1);
	bool first = true;
	while (1) {
		Mat temp = display.clone();
		char key = waitKey(30);
		if (key == '=') { size++; }
		else if (key == '-') { size = size - 1 > 0 ? size - 1 : 1; }
		else if (key == 'q') { break; }
		if ((mouse_event == CV_EVENT_MOUSEMOVE && (mouse_flags == CV_EVENT_FLAG_LBUTTON)) || (mouse_event == CV_EVENT_LBUTTONDOWN)) {
			if (first) {
				last_pos = mouse_pos;
				first = false;
			}
			else {
				line(this->mask, last_pos, mouse_pos, Scalar(0), size);
				line(display, last_pos, mouse_pos, Scalar(0, 0, 255), size);
				line(this->image_masked, last_pos, mouse_pos, Scalar(0, 0, 255), size);
				last_pos = mouse_pos;
			}
		}
		//circle(display, mouse_pos, size, Scalar(0, 0, 255));
		imshow(name, display);
	}
	if (ifsavemask) {
		imwrite(path + "mask.png", mask);
	}
	this->image_inpainted = image_masked.clone();
	destroyWindow(name);
	//imshow("mask", this->mask);
	//imshow("wm", this->image_masked);
	//imshow("src",this->srcImage);
	//waitKey();
	return;
}

void Image::getCurves() {
	Mat display = this->image_masked.clone();
	int size = CURVE_SIZE;
	const string namewindow = "Get the curves";
	namedWindow(namewindow);
	imshow(namewindow, display);
	setMouseCallback(namewindow, on_MouseHandle);
	int click_times = 0;
	vector<Point2i>one_curve;
	Point2i last_pos(-1, -1);
	Point2i linepoint(-1, -1);
	bool first = true;
	while (true) {
		char key = waitKey(20);
		if (key == 'q') break;
		//finish drawing one curve and save it 
		else if (key == 'f') {
			first = true;
			cout << one_curve.size() << endl;
			if (one_curve.size() > 0)
				this->curve_points.push_back(one_curve);
			one_curve.clear();
		}
		else if ((mouse_event == CV_EVENT_MOUSEMOVE && (mouse_flags == CV_EVENT_FLAG_LBUTTON)) ||(mouse_event == CV_EVENT_LBUTTONDOWN)) {
			if (first == true) {
				first = false;
				last_pos = mouse_pos;
				continue;
			}
			line(display, last_pos, mouse_pos, Scalar(0, 255, 0), size);
			LineIterator it(display, last_pos, mouse_pos);
			for (int i = 0; i < it.count; i++, it++) {
				if (linepoint != it.pos()) {
					one_curve.push_back(it.pos());
					linepoint = it.pos();
				}					
			last_pos = mouse_pos;
			}						
		}
		
		imshow(namewindow, display);
	}
	curve_points_copy.assign(curve_points.begin(), curve_points.end());
	if (!ifshowCurves) {
		destroyWindow(namewindow);
	}
	return;
}