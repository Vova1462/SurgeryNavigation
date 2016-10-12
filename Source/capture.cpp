#include<cv.hpp>
using namespace std;
using namespace cv;

int main()
{
	VideoCapture camera1(2);
	if (!camera1.isOpened())
		return -1;
	VideoCapture camera2(0);
	if (!camera2.isOpened())
		return -2;

	Mat frame1, frame2;
	int capture_number=1;
	for (;;)
	{

		camera1 >> frame1;
		camera2 >> frame2;

		imshow("capture1", frame1);
		imshow("capture2", frame2);
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		char c = waitKey(33);
		const string prefix_left("C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\Images\\1_");
		const string prefix_right("C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\Images\\2_");
		const string postfix(".png");
		if (c == 13)
		{
			imwrite(prefix_left + to_string(capture_number) + postfix, frame1, compression_params);
			imwrite(prefix_right + to_string(capture_number) + postfix, frame2, compression_params);
			++capture_number;
		}
		if (c == 27) break;
	}
}