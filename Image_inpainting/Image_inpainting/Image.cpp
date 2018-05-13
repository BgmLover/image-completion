#include"Image.h"

static int mouse_event;
static Point2i mouse_pos;
static int mouse_flags;

void on_MouseHandle(int event, int x, int y, int flags, void *param) {
	mouse_event = event;
	mouse_pos = Point2i(x, y);
	mouse_flags = flags;
}
void Image::getMask() {
	Mat display = this->srcImage.clone();
	
	imshow("Get the mask", display);
	setMouseCallback("Get the mask", on_MouseHandle);

	int size = 40;

	Point2i last_pos(-1,-1);
	bool first = true;
	while (1) {
		Mat temp = display.clone();
		char key = waitKey(30);
		if (key == '=') { size++; }
		else if (key == '-') { size = size - 1 > 0 ? size - 1 : 1; }
		else if (key == 'q') { break; }
		if ((mouse_event == CV_EVENT_MOUSEMOVE && (mouse_flags & CV_EVENT_FLAG_LBUTTON)) || (mouse_event == CV_EVENT_LBUTTONDOWN)) {
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
		imshow("Get the mask", display);
	}
	
	destroyWindow("Get the mask");
	//imshow("mask", this->mask);
	//imshow("wm", this->image_masked);
	//waitKey();
	return;
}

void Image::getCurves() {

}