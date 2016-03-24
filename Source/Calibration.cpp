#include<Calibration.h>

void Calibration::SCalibration(Customozation *cust)
{
	int nx = 10, ny = 7, n_boards = 40; //Kollichestvo yglov po osyam x i y, 
	int n = nx*ny; //kollichestvo uglov na shahmatnoj doske
	CvSize board_size = cvSize(nx, ny); //razmer doski
	vector<CvPoint2D32f> temp1(n);
	vector<CvPoint2D32f> temp2(n);
	
	//Start web-cam
	cust->stereoGrabberGrabFrames();
	cust->stereoGrabberInitFrames();
	IplImage *frame1 = cust->imageLeft;
	IplImage *frame2 = cust->imageRight; //Zahvat izobraweniya s 1 i 2 kameri
	IplImage *gr_frame1,*gr_frame2;
	
	//Funkcia dlya poiska yglov na doske(s tochnostu do pikselya), so vstroennoj normalizaciej i poiska chernih kvadratov dlya yvelichenia tochnosti calibrovki
	cvFindChessboardCorners(frame1,
		board_size,
		&temp1[0],
		NULL,
		CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);
	cvFindChessboardCorners(frame2,
		board_size,
		&temp2[0],
		NULL,
		CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);

	//Convertaciya izobrawenij v gradacii serogo
	cvConvert

	
	//Funkcia dlya poiska yglov na doske(s tochnostu do subpikselya), so vstroennoj normalizaciej i poiska chernih kvadratov dlya yvelichenia tochnosti calibrovki
	cvFindCornerSubPix(



		);
	//Zavershenie raboti kamer
	cust->stereoGrabberStopCam();
}