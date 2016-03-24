#include<stdlib.h>
#include<Customozation.h>
#include <cv.h>

CvCapture *camera1 = NULL, *camera2 = NULL;
void  Customozation::stereoGrabberInitFrames()
{	
	camera1 = cvCaptureFromCAM(2);
	assert(camera1 != NULL);
	cvWaitKey(100);

	camera2 = cvCaptureFromCAM(1);
	assert(camera2 != NULL); // Inicializaciya kamer
}

void Customozation::stereoGrabberGrabFrames()
{ 
	imageLeft = cvQueryFrame(camera1);
	imageRight = cvQueryFrame(camera2);//Posledovatelnoe schitovanie kadrov c kameri 1 i 2
}

void Customozation::stereoGrabberStopCam()
{
	cvReleaseCapture(&camera1);
	cvReleaseCapture(&camera2);//Osvobowdenie pamyati
}