#pragma once
#include<cv.hpp>
#include<highgui.h>

using namespace cv;
namespace Visualisation {
	class GUI {

	public:
		void StartSetup();
		void StartMainMode();
		void Update(Mat &frame1, Mat &frame2, Mat &disp);
		void EndMainMode();

	};
}