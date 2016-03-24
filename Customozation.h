#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include <opencv2\calib3d\calib3d_c.h>
#include<opencv2\videoio.hpp>
#include <iostream>


#define WIDTH 800
#define HEIGHT 600
struct Customozation{

	void stereoGrabberInitFrames();
	void stereoGrabberGrabFrames();
	void stereoGrabberStopCam();
	IplImage* imageLeft;
	IplImage* imageRight;
};
