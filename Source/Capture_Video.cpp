#include <cv.h>
#include <highgui.h>
#include <stdlib.h>
#include <opencv2\calib3d\calib3d_c.h>
#include <stdio.h>

int main(int argc, char **argv) {
	IplImage *Limage = 0, *Rimage = 0, *Lgray = 0, *Rgray = 0, *dst=0;
	dst = cvLoadImage("IMG_2746.JPG", 1);
	if (dst == NULL)
	{
		fprintf(stderr, "ERROR: Could not load left image %s\n");
		
		return 1;
	}
	Limage = cvCreateImage(cvSize(dst->width /7, dst->height / 7), IPL_DEPTH_8U, dst->nChannels);
	cvResize(dst, Limage, 1);
	dst = cvLoadImage("IMG_2747.JPG", 1);
	if (dst == NULL)
	{
		fprintf(stderr, "ERROR: Could not load right image %s\n");
		return 1;
	}
	Rimage = cvCreateImage(cvSize(dst->width / 7, dst->height / 7), IPL_DEPTH_8U, dst->nChannels);
	cvResize(dst, Rimage, 1);
	Lgray = cvCreateImage(cvSize(Limage->width, Limage->height), IPL_DEPTH_8U, 1);
	Rgray = cvCreateImage(cvSize(Rimage->width , Rimage->height), IPL_DEPTH_8U, 1);
	cvConvertImage(Limage, Lgray, CV_8S);
	cvConvertImage(Rimage, Rgray, CV_8S);
	cvFlip(Lgray, 0);
	cvFlip(Rgray, 0);

	CvMat *Stereo = cvCreateMat(Rgray->height, Rgray->width, CV_16S);
	CvMat *VStereo = cvCreateMat(Rgray->height, Rgray->width, CV_8U);
	
	if (Stereo == NULL)
	{
		fprintf(stderr, "ERROR: Could not create depth map %s\n");
		return 1;
	}
	
	CvStereoBMState* state = cvCreateStereoBMState(CV_STEREO_BM_BASIC,0);
	if (state == NULL)
	{
		fprintf(stderr, "ERROR: Could not create CvStereoBMState\n");
		return 1;
	}
	cvNamedWindow("cvHoughCircles", CV_WINDOW_AUTOSIZE);



		state->preFilterSize =21;
		state->preFilterCap = 63;
		state->SADWindowSize =41;
		state->minDisparity = 0;
		state->numberOfDisparities =80;
		state->textureThreshold =20;
		state->uniquenessRatio =8;
		state->speckleWindowSize = 20;
		state->speckleRange = 15;
		cvFindStereoCorrespondenceBM(Lgray, Rgray, Stereo, state);
		cvNormalize(Stereo, VStereo, 0, 256, CV_MINMAX, NULL);
	CvMemStorage* storage = cvCreateMemStorage(0); //хранилище кругов
	cvSmooth(VStereo, VStereo, CV_GAUSSIAN, 1, 1);//сглаживание изображения
	cvNamedWindow("left1", CV_WINDOW_AUTOSIZE);
	cvShowImage("left1", Lgray);
	CvSeq* results = cvHoughCircles(
		Lgray,
		storage,
		CV_HOUGH_GRADIENT,
		5,
		Lgray->width / 2,
		100,
		100,
	    20,
		60
		);//поиск окружностей
	CvScalar s;
	s.val[0] = 0;
	for (int i = 0; i < results->total; i++) {
		float* p = (float*)cvGetSeqElem(results, i);
		printf("%f%f%f\n", p[0], p[1], p[2]);
		CvPoint pt = cvPoint(cvRound(p[0]), cvRound(p[1]));
		cvCircle(Lgray, pt, cvRound(p[2]), CV_RGB(0xff,0, 0xff),4);
		s = cvGet2D(VStereo, p[1], p[0]);
		printf("%f\n", s.val[0]);
	}//прорисовка окружностей
	cvNamedWindow("cvHoughCircles", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("left", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("right", CV_WINDOW_AUTOSIZE);
	cvShowImage("left", Lgray);
	cvShowImage("right", Rgray);
	cvShowImage("cvHoughCircles", VStereo);
	cvWaitKey(0);
	cvReleaseImage(&Limage);
	cvReleaseImage(&Rimage);
	cvReleaseImage(&Lgray);
	cvReleaseImage(&Rgray);
	cvReleaseMat(&VStereo);
	cvReleaseMat(&Stereo);
	cvDestroyAllWindows();
	return 0;
}
