#include<cv.hpp>
#include <string>
#include <iostream>
#include <iterator> 



using namespace std;
using namespace cv;

static bool ReadStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}

static void StereoCallibration(VideoCapture camera1, VideoCapture camera2)
{
	char a;
	string str="";
	int w, h;
	float squareSize = 1.0;
	Size boardSize;
	bool displayCorners = true, useCalibrated = false, showRectified = true;

	cout << "Do you have image list?Y/N\n";
	cin >> a;

	if (a == 'Y')
	{
		cout << "Please, write number of corners in the horizontal, in the vertical and path to image list\n";
		cin>>w>>h>>str;
	}
	else
	{
		namedWindow("capture1", 1);
		namedWindow("capture2", 1);
		int capture_number = 0;
		Mat frame1, frame2;
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
			const string prefix_left("C:\\dev\\MyProjects\\SurgeryNavigation\\Calibration\\left");
			const string prefix_right("C:\\dev\\MyProjects\\SurgeryNavigation\\Calibration\\right");
			const string postfix(".png");
			if (c == 13)
			{
				imwrite(prefix_left + to_string(capture_number) + postfix, frame1, compression_params);
				imwrite(prefix_right + to_string(capture_number) + postfix, frame2, compression_params);
				++capture_number;
			}
			if (c == 27) break;
		}
		cout << "Please, write number of corners in the horizontal, in the vertical and path to image list\n";
		cin >> w>>h;
		str = "C:\\dev\\MyProjects\\SurgeryNavigation\\Calibration\\image_list.xml";
	}
	vector<string> imagelist;
	bool ok = ReadStringList(str, imagelist);
	if (!ok || imagelist.empty())
	{
		cout << "can not open " << str << " or the string list is empty" << endl;
		return;
	}

	
	boardSize.width = w;
	boardSize.height = h;

	if (imagelist.size() % 2 != 0)
	{
		cout << "Error: the image list contains odd (non-even) number of elements\n";
		return;
	}

	const int maxScale = 2;
	// ARRAY AND VECTOR STORAGE:

	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	Size imageSize;

	int i, j, k, nimages = (int)imagelist.size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			const string& filename = imagelist[i * 2 + k];
			Mat img = imread(filename, 0);
			if (img.empty())
				break;
			if (imageSize == Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				cout << "The image " << filename << " has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			vector<Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners,
					CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			if (displayCorners)
			{
				cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS,
					30, 0.01));
		}
		if (k == 2)
		{
			goodImageList.push_back(imagelist[i * 2]);
			goodImageList.push_back(imagelist[i * 2 + 1]);
			j++;
		}
	}
	cout << j << " pairs have been successfully detected.\n";
	nimages = j;
	if (nimages < 2)
	{
		cout << "Error: too little pairs to run the calibration\n";
		return;
	}

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 0);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 0);
	Mat R, T, E, F;

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO +
		CALIB_ZERO_TANGENT_DIST +
		CALIB_USE_INTRINSIC_GUESS +
		CALIB_SAME_FOCAL_LENGTH +
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "done with RMS error=" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average epipolar err = " << err / npoints << endl;

	// save intrinsic parameters
	FileStorage fs("C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		0, 1, imageSize, &validRoi[0], &validRoi[1]);

	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	Mat _M1;
	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\intrinsics.yml", FileStorage::READ);
	

	// OpenCV can handle left-right
	// or up-down camera arrangements
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	// COMPUTE AND DISPLAY RECTIFICATION
	if (!showRectified)
		return;

	Mat rmap[2][2];
	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}
	
	//Precompute maps for cv::remap()

	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_32F, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_32F, rmap[1][0], rmap[1][1]);

	Mat canvas;
	double sf;
	if (!isVerticalStereo)
	{
		sf = 600. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h, w * 2, CV_8UC3);
	}
	else
	{
		sf = 300. / MAX(imageSize.width, imageSize.height);
		w = cvRound(imageSize.width*sf);
		h = cvRound(imageSize.height*sf);
		canvas.create(h * 2, w, CV_8UC3);
	}

	for (i = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			Mat img = imread(goodImageList[i * 2 + k], 0), rimg, cimg;
			remap(img, rimg, rmap[k][0], rmap[k][1], INTER_LINEAR);
			cvtColor(rimg, cimg, COLOR_GRAY2BGR);
			Mat canvasPart = !isVerticalStereo ? canvas(Rect(w*k, 0, w, h)) : canvas(Rect(0, h*k, w, h));
			resize(cimg, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);
			if (useCalibrated)
			{
				Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
					cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
				rectangle(canvasPart, vroi, Scalar(0, 0, 255), 3, 8);
			}
		}

		if (!isVerticalStereo)
			for (j = 0; j < canvas.rows; j += 16)
				line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);
		else
			for (j = 0; j < canvas.cols; j += 16)
				line(canvas, Point(j, 0), Point(j, canvas.rows), Scalar(0, 255, 0), 1, 8);
		imshow("rectified", canvas);
		char c = (char)waitKey();
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}
}

static void CreatROI(Mat *capture1, Mat *capture2, Rect *roi1, Rect *roi2, Size imgsize)
{
	//»нициализаци€ матриц камеры, векторов вращени€, перемещени€, координат смещени€ дл€ изображений
	Mat R1, R2, R, T, P1, P2, M1, M2, D1, D2, Q, rect_map[2][2],img1rect, img2rect;
	//«агрузка из YAML фалов внутренних и внешних параметров камеры, и координат дл€ исправлени€ изображений, полученных на этапе калибровки 
	FileStorage fs;
	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\extrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["P1"] >> R1;
		fs["R2"] >> R2;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
	}
	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"] >> M1;
		fs["M2"] >> M2;
		fs["D1"] >> D1;
		fs["D2"] >> D2;
	}

	//¬ычисление областей дл€ работы с изображением
	stereoRectify(M1, D1, M2, D2, imgsize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imgsize, roi1, roi2);

	//¬ычисление координат дл€ исправлени€ искажений на изображни€х
	initUndistortRectifyMap(M1, D1, R1, P1, imgsize, CV_16SC2, rect_map[0][0], rect_map[0][1]);
	initUndistortRectifyMap(M2, D2, R2, P2, imgsize, CV_16SC2, rect_map[1][0], rect_map[0][1]);

	//ѕерестраивает изображени€ дл€ исправлени€ оптических искажений
	remap(*capture1, img1rect, rect_map[0][0], rect_map[0][1], INTER_LINEAR);
	remap(*capture2, img2rect, rect_map[1][0], rect_map[0][1], INTER_LINEAR);
	
	Mat croped_img1, croped_img2;
	if (roi1->area() <= roi2->area())
	{
		croped_img1 = img1rect(*roi1);
		croped_img2 = img2rect(*roi1);
		*roi2 = *roi1;
	}
	else
	{
		croped_img1 = img1rect(*roi2);
		croped_img2 = img2rect(*roi2);
		*roi1 = *roi2;
	}
	*capture1 = croped_img1;
	*capture2 = croped_img2;

}

static void CreateDisparityMap(Ptr<StereoBM> bm,Mat capture1, Mat capture2,Mat *dispmap, Rect roi1, Rect roi2, int *param_for_stereomatch)
{
	//«адание параметров дл€ вычислени€ карты несоответстви€
	bm->setROI1(roi1);
	bm->setROI2(roi2);
	if (param_for_stereomatch[0]>0)
		bm->setPreFilterCap(param_for_stereomatch[0]);
	if (param_for_stereomatch[1] % 2 != 0 && param_for_stereomatch[1] > 3)
		bm->setBlockSize(param_for_stereomatch[1]);
	else
		bm->setBlockSize(8);

		
	bm->setMinDisparity(0);
	if (param_for_stereomatch[2] >= 0)
		bm->setNumDisparities(param_for_stereomatch[2] * 16);
	bm->setTextureThreshold(param_for_stereomatch[3]);
	bm->setUniquenessRatio(param_for_stereomatch[4]);
	bm->setSpeckleWindowSize(50);
	bm->setSpeckleRange(6);
	bm->setDisp12MaxDiff(1);

	//¬ычисление карты несоответстви€
	bm->compute(capture1, capture2, *dispmap);
}

static void StereoMatch(int iteration, Mat capture1,Mat capture2, Rect roi1, Rect roi2)
{
	Mat disp(capture1.rows, capture1.cols, CV_16S), vdisp(capture1.rows, capture1.cols, CV_8U), img1rect, img2rect;
	
	//Ќачальна€ инициализаци€ параметров дл€ стереосоответстви€
	int param_for_stereomatch[5];
	param_for_stereomatch[0] = 31, param_for_stereomatch[1] = 11,param_for_stereomatch[2] = 16,	param_for_stereomatch[3] = 16,param_for_stereomatch[4] = 0;
	
	//—оздание окон дл€ отображени€ и изменений параметров стереосоответстви€
	namedWindow("Parametars", 1);
	createTrackbar("setPreFilterCap", "Parametars", &param_for_stereomatch[0], 63);
	createTrackbar("setBlockSize", "Parametars", &param_for_stereomatch[1], 30);
	createTrackbar("setTextureThreshold", "Parametars", &param_for_stereomatch[2], 70);
	createTrackbar("setNumDisparities", "Parametars", &param_for_stereomatch[3], 20);
	createTrackbar("setUniquenessRatio", "Parametars", &param_for_stereomatch[4], 100);


	Ptr<StereoBM> bm = StereoBM::create(16, 9);
	if (iteration == 0)
	for (;;)
	{
		param_for_stereomatch[0] = getTrackbarPos("setPreFilterCap", "Parametars");
		param_for_stereomatch[1] = getTrackbarPos("setBlockSize", "Parametars");
		param_for_stereomatch[2] = getTrackbarPos("setBlockSize", "Parametars");
		param_for_stereomatch[3] = getTrackbarPos("setNumDisparities", "Parametars");
		param_for_stereomatch[4] = getTrackbarPos("setUniquenessRatio", "Parametars");
		//ѕостроение карты несоответстви€ методом BM с нормализацией изображени€ дл€ дальнейшего отображени€
		CreateDisparityMap(bm, capture1, capture2, &disp, roi1, roi2, param_for_stereomatch);
		disp.convertTo(vdisp, CV_8U, 1/16.);
		
		imshow("DisparityMap", vdisp);

		char c = waitKey(100);
		if (c == 27) 
		{
			break;
		}
			
	}
	else
	{
		CreateDisparityMap(bm, capture1, capture2, &disp, roi1, roi2, param_for_stereomatch);
		normalize(disp, vdisp, 0, 1, CV_MINMAX);
		disp.convertTo(vdisp, CV_8U);
		imshow("DisparityMap", vdisp);
	}
}

static void FindCircles(Mat capture1, Mat templ, double minval,double maxval, Point minloc, Point maxloc, int search_metod, Mat *gray_result)
{
	
		Mat result;
	
		//ѕоиск шаблона
		matchTemplate(capture1, templ, result, search_metod);
		
		Mat treshold_result;
		/*result.convertTo(uint_result, CV_32F);*/
		threshold(result, treshold_result,8.0e+6, 1, THRESH_BINARY_INV);
		imshow("result", treshold_result);
		treshold_result.convertTo(*gray_result, CV_8UC1);

		//for (;;)
		//{
		//	//ѕоиск пикселей с максимальной и минимальной €ркостью
		//	minMaxLoc(result, &minval, &maxval, &minloc, &maxloc);
		//	//ќтображение пр€моугольника вокруг пикселей минимальной €ркости
		//	rectangle(capture1, Point(minloc.x, minloc.y), Point(minloc.x + templ.cols - 1, minloc.y + templ.rows - 1), CV_RGB(255, 0, 0), 1, 8);
		//	
		//	for (int i = minloc.x;i < minloc.x;i++)
		//		for (int j = minloc.y;j < minloc.y;j++)
		//			result.at<float>(i, j) != result.at<float>(i, j);

		//	imshow("result", result);
		//	imshow("capture1", capture1);
		//	char c = waitKey(100);
		//	if (c == 27) break;
		//}
	
}

int main()
{
	char a;
	Mat frame1, frame2, gframe1, gframe2, templ, match_img1, match_img2;
	string filename;
	const string templ_prefix = "C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\Template\\", img_prefix= "C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\Images\\";
	const string postfix = ".png";
	double minval=0, maxval=0;
	Point minloc, maxloc;
	Rect roi1, roi2;
	//ѕараметр показывающий откуда брать изображение. ≈сли 0-то с камер, если 1-то из файла.
	int source_of_image = 0;
	//ѕараметр устанавливающий метод поиска шаблона. ¬арьируетс€ от 0 до 5
	int search_metod=0;

	VideoCapture cap1(2);
	if (!cap1.isOpened())
		return -1;
	VideoCapture cap2(3);
	if (!cap2.isOpened())
		return -2;


	cout << "Do you need calibration?Y/N\n";
	cin >> a;
	if (a == 'Y')
	{
		StereoCallibration(cap1,cap2);
	}
	
	namedWindow("templ");
	namedWindow("capture1", 1);
	namedWindow("capture2", 1);
	
	

	for (;;)
	{
		if (source_of_image == 0)
		{
			cap1 >> frame1;
			cap2 >> frame2;
		}
		else if (source_of_image == 1)
		{
			frame1 = imread(img_prefix+"1_2"+postfix,1);
			frame2 = imread(img_prefix + "2_2" + postfix, 1);
		}
		
		imshow("capture1", frame1);
		imshow("capture2", frame2);
		//ќбрезка и исправление искажений на изображени€х
		CreatROI(&frame1,&frame2,&roi1,&roi2,frame1.size());

		imshow("capture1", frame1);
		imshow("capture2", frame2);
		gframe1.create(frame1.rows, frame1.cols, CV_8UC1);
		gframe2.create(frame2.rows, frame2.cols, CV_8UC1);

		cvtColor(frame1, gframe1, CV_BGR2GRAY);
		cvtColor(frame2, gframe2, CV_BGR2GRAY);
		//«агрузка шаблона
		templ = imread(templ_prefix + to_string(4) + postfix, 0);
		//ѕоиск кругов и создание изображений дл€ поиска соответстви€
		FindCircles(gframe1, templ,minval,maxval,minloc,maxloc, search_metod,&match_img1);
		FindCircles(gframe2, templ, minval, maxval, minloc, maxloc, search_metod, &match_img2);
		
		//ѕоиск соответстви€ между изображени€ми
		StereoMatch(0, gframe1, gframe2, roi1, roi2);
		
		

		

		char c = waitKey(100);
		if (c == 27) break;
	}
	return 0;
}