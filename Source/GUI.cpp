#include "GUI.h"
using namespace Visualisation;
void GUI::StartSetup() {

}

void GUI::StartMainMode() {

}

void GUI::Update(Mat &frame1, Mat &frame2, Mat &disp) {
	Mat working_area(Size(frame1.cols*2+200,frame2.rows*2+200), CV_8UC1);
	Mat picture1 = working_area(Rect(0, 0, frame1.cols, frame1.rows));
	Mat picture2 = working_area(Rect(frame1.cols+20, 0, frame2.cols, frame2.rows));
	Mat picture3 = working_area(Rect(0, frame1.rows+20, disp.cols, disp.rows));

	frame1.copyTo(picture1);
	frame2.copyTo(picture2);
	disp.copyTo(picture3);

	namedWindow("WorkingArea", WINDOW_AUTOSIZE);
	imshow("WorkingArea", working_area);
}

void GUI::EndMainMode() {
	destroyAllWindows();
}