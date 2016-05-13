#include<cv.hpp>
#include<opencv2\highgui.hpp>
#include<opencv2\calib3d.hpp>
#include<opencv2\opencv.hpp>
#include<opencv2\videoio.hpp>

using namespace std;
using namespace cv;

int main()
{
	VideoCapture cap1(-1);
	if (!cap1.isOpened())
		return -1;
	VideoCapture cap2(-1);
	if (!cap2.isOpened())
		return -2;
	
	namedWindow("capture1", 1);
	namedWindow("capture2", 1);
	int capture_number=0;

	for (;;)
	{
		Mat frame1, frame2;
		cap1 >> frame1;
		cap2 >> frame2;
		
		imshow("capture1", frame1);
		imshow("capture2", frame2);
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		char c = waitKey(33);
		const string prefix_left("C:\\dev\\MyProjects\\SurgeryNavigation\\left");
		const string prefix_right("C:\\dev\\MyProjects\\SurgeryNavigation\\right");
		const string postfix(".png");

		if (c == 13)
		{
			imwrite(prefix_left + to_string(capture_number) + postfix, frame1, compression_params);
			imwrite(prefix_right + to_string(capture_number) + postfix, frame2, compression_params);
			++capture_number;
		}
		if (c==27) break;
	}
	return 0;
}