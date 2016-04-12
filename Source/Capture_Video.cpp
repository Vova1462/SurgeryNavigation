#include <iostream>
#include "cv.h"
#include "highgui.h"
#include "cvaux.h"
#include <opencv2\calib3d\calib3d_c.h>
#include<opencv2\videoio.hpp>
#include <iostream>
#include <vector>
#include<cxcore.h>


using namespace std;
#define WIDTH 800
#define HEIGHT 600

//void stereoGrabberInitFrames();
//void stereoGrabberGrabFrames();
//void stereoGrabberStopCam();
//void SCalibration();

IplImage* imageLeft;
IplImage* imageRight;

CvCapture *camera1, *camera2, *capture1, *capture2;


int main(int argc, char **argv)
{
	// получаем любую подключённую камеру
	//SCalibration();
	capture1 = cvCreateCameraCapture(0); // или Ваш cvCaptureFromCAM( 0 );
	capture2 = cvCreateCameraCapture(0); // или Ваш cvCaptureFromCAM( 0 );
	IplImage* frame1 = 0;
	IplImage* frame2 = 0;

	cvNamedWindow("TestWinD1", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("TestWinD2", CV_WINDOW_AUTOSIZE);
	while (true)
	{
	/*	 получаем кадр*/
		frame1 = cvQueryFrame(capture1);
		frame2 = cvQueryFrame(capture2);
		/* показываем*/
		cvShowImage("TestWinD1", frame1);
		cvShowImage("TestWinD2", frame2);
		char c = cvWaitKey(33);
		if (c == 27) { // нажата ESC
			break;
		}
	}
	 //освобождаем ресурсы
	cvReleaseCapture(&capture1);
	cvDestroyWindow("capture1");
	cvReleaseCapture(&capture2);
	cvDestroyWindow("capture2");

	return 0;
}


//void  stereoGrabberInitFrames()
//{
//	camera1 = cvCreateCameraCapture(3); 
//	cvWaitKey(100);
//	camera2 = cvCreateCameraCapture(4);
// // Inicializaciya kamer
//}
//
//void stereoGrabberGrabFrames()
//{
//	imageLeft = cvQueryFrame(camera1);
//	imageRight = cvQueryFrame(camera2);//Posledovatelnoe schitovanie kadrov c kameri 1 i 2
//}
//
//void stereoGrabberStopCam()
//{
//	cvReleaseCapture(&camera1);
//	cvReleaseCapture(&camera2);//Osvobowdenie pamyati
//}
//
//void SCalibration()
//{
//	int nx = 10, ny = 7, boards_needed = 40, result1 = 0, result2 = 0; //Колличество углов по осям х и у, колличество циклов, результаты поиска углов
//	int n = nx*ny; //Колличество углов на шахмтной доске
//	int successes1 = 0, successes2 = 0;
//	int frame = 0, corners_count1 = 0, corners_count2 = 0, step = 0;
//	CvSize board_size = cvSize(nx, ny); //Размер доски
//	vector<CvPoint2D32f> found_points_coords1(n);
//	vector<CvPoint2D32f> found_points_coords2(n);//Массив координат точек найденных на шахматной доске
//
//	//Запуск веб-камер
//	
//	stereoGrabberInitFrames();
//	stereoGrabberGrabFrames();
//	IplImage *frame1 = imageLeft;
//	IplImage *frame2 = imageRight; //Захват изображения с 1 и 2 камеры
//	IplImage *gr_frame1, *gr_frame2;
//
//	//Выделение памяти под хранилища
//
//	CvMat* image_points1 = cvCreateMat(boards_needed*n, 2, CV_32FC1);
//	CvMat* image_points2 = cvCreateMat(boards_needed*n, 2, CV_32FC1);
//	CvMat* object_points1 = cvCreateMat(boards_needed*n, 3, CV_32FC1);
//	CvMat* object_points2 = cvCreateMat(boards_needed*n, 3, CV_32FC1);
//	CvMat* point_counts1 = cvCreateMat(boards_needed, 1, CV_32SC1);
//	CvMat* point_counts2 = cvCreateMat(boards_needed, 1, CV_32SC1);
//	//Создание окон для отображения изображения получаемого с камер
//	cvNamedWindow("Camera 1", 1);
//	cvNamedWindow("Camera 2", 1);
//
//	while ((successes1<boards_needed) || (successes2<boards_needed))
//	{
//		//Ожидание 20 кадров доски с разных ракурсов для захвата изображения
//		if (frame++ % 20 == 0)
//		{
//			//Функция для поиска углов на доске(с точностью пикселя), со встроенной нормализацией и поиском черных квадратов для увеличения точности калибровки
//			result1 = cvFindChessboardCorners(frame1,
//				board_size,
//				&found_points_coords1[0],
//				NULL,
//				CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);
//			result2 = cvFindChessboardCorners(frame2,
//				board_size,
//				&found_points_coords2[0],
//				NULL,
//				CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);
//
//			//Конвертация изображений в градации серого
//			cvConvertImage(frame1, gr_frame1, CV_8S);
//			cvConvertImage(frame1, gr_frame1, CV_8S);
//
//			//Функция для поиска углов на доске(с точностью субпикселя), со встроенной нормализацией и поиском черных квадратов для увеличения точности калибровки
//			cvFindCornerSubPix(gr_frame1,
//				&found_points_coords1[0],
//				corners_count1,
//				cvSize(11, 11),
//				cvSize(-1, -1),
//				cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 0.01));
//
//			cvFindCornerSubPix(gr_frame2,
//				&found_points_coords2[0],
//				corners_count2,
//				cvSize(11, 11),
//				cvSize(-1, -1),
//				cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 0.01));
//
//			//Отрисовка найденых углов
//			cvDrawChessboardCorners(frame1,
//				board_size,
//				&found_points_coords1[0],
//				corners_count1,
//				result1);
//			cvDrawChessboardCorners(frame2,
//				board_size,
//				&found_points_coords2[0],
//				corners_count2,
//				result2);
//
//			//Отображение изображения
//			cvShowImage("Camera1", frame1);
//			cvShowImage("Camera2", frame2);
//
//
//			//Добавление хорошего(у котрого задетектированы все углы) представления доски в коллекцию
//			if (corners_count1 == n)
//			{
//				step = successes1*n;
//				for (int i = step, j = 0; j < n; ++i, ++j)
//				{
//					CV_MAT_ELEM(*image_points1, float, i, 0) = found_points_coords1[j].x;
//					CV_MAT_ELEM(*image_points1, float, i, 1) = found_points_coords1[j].y;
//					CV_MAT_ELEM(*object_points1, float, i, 0) = j / nx;
//					CV_MAT_ELEM(*object_points1, float, i, 1) = j%nx;
//					CV_MAT_ELEM(*object_points1, float, i, 1) = 0.0f;
//				}
//			}
//			if (corners_count2 == n)
//			{
//				step = successes2*n;
//				for (int i = step, j = 0; j < n; ++i, ++j)
//				{
//					CV_MAT_ELEM(*image_points2, float, i, 0) = found_points_coords2[j].x;
//					CV_MAT_ELEM(*image_points2, float, i, 1) = found_points_coords2[j].y;
//					CV_MAT_ELEM(*object_points2, float, i, 0) = j / nx;
//					CV_MAT_ELEM(*object_points2, float, i, 1) = j%nx;
//					CV_MAT_ELEM(*object_points2, float, i, 1) = 0.0f;
//				}
//				CV_MAT_ELEM(*point_counts1, int, successes1, 0) = n;
//				CV_MAT_ELEM(*point_counts2, int, successes2, 0) = n;
//				successes1++;
//				successes2++;
//			}
//		}
//
//		//Съемка следущей фотографии
//		stereoGrabberGrabFrames();
//		frame1 = imageLeft;
//		cvShowImage("Camera 1", frame1);
//		frame2 = imageRight;
//		cvShowImage("Camera 1", frame2);
//
//		//Ожидание нажатия клавишы для завершения
//		if (cvWaitKey(15) == 27) break;
//	}
//
//	//Калибровка камеры
//	//cvCalibrateCamera2(
//	//	object_points1,
//	//	image_points1,
//	//	point_counts1,
//	//	cvGetSize(frame1),
//
//	//	);
//
//	//Завершение работы камер
//	stereoGrabberStopCam();
//
//}