#pragma once
#include<cv.hpp>
#include<highgui.h>

using namespace cv;
namespace Visualisation {
	class GUI {

	public:
		void StartSetup(Mat &frame1, Mat &frame2, Mat &disp);
		void StartMainMode();
		void Update();
		void EndMainMode();

	};
}