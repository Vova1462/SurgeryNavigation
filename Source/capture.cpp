#include<cv.hpp>
using namespace std;
using namespace cv;

int main()
{


	Mat frame1, frame2, frame3;
	int capture_number=1;
	frame1 = imread("C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\Images\\1_1.png", 0);
	Canny(frame1, frame2, 10, 100);
	Sobel(frame2, frame3, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT);
	imshow("capture2", frame1);
	imshow("capture1", frame2);
	imshow("capture1", frame3);
	char c = waitKey(6000);
	if (c==17)
		return 0;
}