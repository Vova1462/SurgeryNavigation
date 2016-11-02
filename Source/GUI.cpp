#include "GUI.h"
using namespace Visualisation;
void GUI::StartSetup(Mat &frame1, Mat &frame2, Mat &disp) {
	
	
	Mat working_area(frame1.size()*2, CV_8UC1);
	Mat picture1=working_area(Rect(0, 0,640, 480));
	Mat picture2=working_area(Rect(640, 0, 640, 480));
	Mat picture3 = working_area(Rect(0, 480, 640, 480));

	frame1.copyTo(picture1);
	frame2.copyTo(picture2);
	disp.copyTo(picture3);
	



	namedWindow("WorkingArea", WINDOW_AUTOSIZE);
	imshow("WorkingArea", working_area);
	waitKey(6000);
}

void GUI::StartMainMode() {

}

void GUI::Update() {

}

void GUI::EndMainMode() {
	destroyAllWindows();
}