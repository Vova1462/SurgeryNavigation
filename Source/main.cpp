#include<cv.hpp>
#include<opencv2\videoio.hpp>
#include <string>
#include <iostream>
#include <iterator> 



using namespace std;
using namespace cv;

static bool readStringList(const string& filename, vector<string>& l)
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

		for (;;)
		{
			Mat frame1, frame2;
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
	bool ok = readStringList(str, imagelist);
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
		{
			destroyWindow("rectified");
			break;
		}
	}
}

static void CreatROI(Mat *capture1, Mat *capture2, Rect *roi1, Rect *roi2, Size imgsize)
{
	//Инициализация матриц камеры, векторов вращения, перемещения, координат смещения для изображений
	Mat R1, R2, R, T, P1, P2, M1, M2, D1, D2, Q, rect_map[2][2], img1rect, img2rect;
	//Загрузка из YAML фалов внутренних и внешних параметров камеры, и координат для исправления изображений, полученных на этапе калибровки
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

	//Вычисление областей для работы с изображением
	stereoRectify(M1, D1, M2, D2, imgsize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imgsize, roi1, roi2);
	
	//Вычисление координат длz исправления искажений на изображниях
	initUndistortRectifyMap(M1, D1, R1, P1, imgsize, CV_16SC2, rect_map[0][0], rect_map[0][1]);
	initUndistortRectifyMap(M2, D2, R2, P2, imgsize, CV_16SC2, rect_map[1][0], rect_map[0][1]);

	//Перестраивает изображения для исправления оптических искажений
	remap(*capture1, img1rect, rect_map[0][0], rect_map[0][1], INTER_LINEAR);
	remap(*capture2, img2rect, rect_map[1][0], rect_map[0][1], INTER_LINEAR);

	//Обрезка изображений
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

static void StereoMatch(Mat capture1,Mat capture2, Mat *disp, int *parametrs)
{


	Size imgsize = capture1.size();
	Mat vdisp(capture1.rows, capture1.cols, CV_16U);

	Ptr<StereoBM> bm = StereoBM::create(64, 21);

			if (*(parametrs+0) > 0)
				bm->setPreFilterCap(*(parametrs+0));
			if (*(parametrs+1) > 1)
				bm->setBlockSize(*(parametrs+1) * 2 + 1);
			bm->setMinDisparity(0);
			if (*(parametrs+2) > 0)
				bm->setNumDisparities(*(parametrs+2) * 16);
			bm->setTextureThreshold(*(parametrs+3));
			bm->setUniquenessRatio(*(parametrs+4));
			bm->setSpeckleWindowSize(100);
			bm->setSpeckleRange(32);
			bm->setDisp12MaxDiff(1);
			//Вычисление карты глубины по предыдущем параметрам
			bm->compute(capture1, capture2, vdisp);

			vdisp.convertTo(*disp, CV_8U, 1 / 16.);
}

static void FindCircles(Mat capture1, int *params,	vector<Vec3f> *circles)
{
	Mat blured;
	//Сглаживание изображения, для уменьшения шума на изображении
	medianBlur(capture1, blured, *(params+5)*2+1);
	//Canny(blured, capture1, 100, 100);
	//Поиск окружностей на изображении
	HoughCircles(blured, *circles, HOUGH_GRADIENT,
		1, *(params + 0), *(params+1)+100, *(params + 2), *(params + 3), *(params + 4));
	
	Vec3f coords_and_radius;//Вектор для хранения координат окружности и радиуса, при отрисовке
	
	int vec_nums[4]; //Массив для хранения номеров векторов, храняobq радиус окружности

	//Отрисовка окружностей на изображении
	for (size_t i = 0; i < circles->size(); i++)
	{
		coords_and_radius = *(circles->begin()+i);
		circle(capture1, Point(coords_and_radius[0], coords_and_radius[1]), coords_and_radius[2], Scalar(0, 0, 255), 3, LINE_AA);
		/*circle(capture1, Point(c[0], c[1]), 2, Scalar(0, 255, 0), 3, LINE_AA);*/
	}
}



int main(int argc, char** argv)
{
	char a;
	Mat frame1, frame2, gframe1, gframe2, disp;
	const string img_prefix = "C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\Images\\";
	const string postfix = ".png";
	Vec3f coords_and_radius;
	Rect roi1, roi2;
	double depth=0, baseline=62, focal_length=4.1 , sencorsize=0.005;
	int source_of_image = 0;
	double fps = atof(argv[1]);
	
	
	//Инициализация камер
	VideoCapture cap1(2);
	if (!cap1.isOpened())
		return -1;
	VideoCapture cap2(3);
	if (!cap2.isOpened())
		return -2;



	//Калибровка камер
	cout << "Do you need calibration?Y/N\n";
	cin >> a;
	if (a == 'Y')
	{
		StereoCallibration(cap1,cap2);
	}


	//Настройка частоты захвата изображения
	cap1.set(CV_CAP_PROP_FPS, fps);
	cap2.set(CV_CAP_PROP_FPS, fps);
	//fps=cap1.get(CV_CAP_PROP_FPS);


	vector<Vec3f> circles1, circles2;

	int parametrs_for_matching[5],parametrs_for_hough[6];

	//Инициализация начальных параметров для поиска неравенства
	parametrs_for_matching[0] = 31;
	parametrs_for_matching[1] = 9;
	parametrs_for_matching[2] = 9;
	parametrs_for_matching[3] = 16;
	parametrs_for_matching[4] = 0;

	//Инициализация начальных параметров для поиска окружностей
	parametrs_for_hough[0] = 23;
	parametrs_for_hough[1] = 21;
	parametrs_for_hough[2] = 40;
	parametrs_for_hough[3] = 9;
	parametrs_for_hough[4] = 47;
	parametrs_for_hough[5] = 7;


	//Создание окон для отображения информации
	namedWindow("capture1", 1);
	namedWindow("capture2", 1);
	namedWindow("Parameters for Hough", 1);
	namedWindow("Parameters for StereoMatching", 1);
	
	//Создание ползунков для поиска окружностей
	createTrackbar("Threshold", "Parameters for Hough", &parametrs_for_hough[0], 100, 0);
	createTrackbar("Thresholdofstoradge", "Parameters for Hough", &parametrs_for_hough[1], 25, 0);
	createTrackbar("Mindist", "Parameters for Hough", &parametrs_for_hough[2], 50, 0);
	createTrackbar("MinRad", "Parameters for Hough", &parametrs_for_hough[3], 30, 0);
	createTrackbar("MaxRad", "Parameters for Hough", &parametrs_for_hough[4], 80, 0);
	createTrackbar("Blured", "Parameters for Hough", &parametrs_for_hough[5], 15, 0);

	//Создание ползунков для поиска неравенства
	createTrackbar("setPreFilterCap", "Parameters for StereoMatching", &parametrs_for_matching[0], 63);
	createTrackbar("setBlockSize", "Parameters for StereoMatching", &parametrs_for_matching[1], 50);
	createTrackbar("setTextureThreshold", "Parameters for StereoMatching", &parametrs_for_matching[2], 20);
	createTrackbar("setNumDisparities", "Parameters for StereoMatching", &parametrs_for_matching[3], 20);
	createTrackbar("setUniquenessRatio", "Parameters for StereoMatching", &parametrs_for_matching[4], 20);


	bool first_time_matching = true;
	for (;;)
	{
		//Метод для получения фотографий
		if (source_of_image == 0)
		{
			cap1 >> frame1;
			cap2 >> frame2;
		}
		else if (source_of_image == 1)
		{
			frame1 = imread(img_prefix + "1_2" + postfix, 1);
			frame2 = imread(img_prefix + "2_2" + postfix, 1);
		}

		//Исправление изображений
		CreatROI(&frame1, &frame2, &roi1, &roi2, frame1.size());

		//Создания изображений
		gframe1.create(480, 640, CV_8UC1);
		gframe2.create(480, 640, CV_8UC1);

		//Конвертация изображений в градации серого
		cvtColor(frame1, gframe1, CV_BGR2GRAY);
		cvtColor(frame2, gframe2, CV_BGR2GRAY);
		
		//Поиск маркеров
		FindCircles(gframe1,parametrs_for_hough, &circles1);
		FindCircles(gframe2, parametrs_for_hough, &circles2);

		//Задание параметров для поиска окружностей
		parametrs_for_hough[0] = getTrackbarPos("Threshold", "Parameters for Hough");
		parametrs_for_hough[1] = getTrackbarPos("Thresholdofstoradge", "Parameters for Hough");
		parametrs_for_hough[2] = getTrackbarPos("Mindist", "Parameters for Hough");
		parametrs_for_hough[3] = getTrackbarPos("MinRad", "Parameters for Hough");
		parametrs_for_hough[4] = getTrackbarPos("MaxRad", "Parameters for Hough");
		parametrs_for_hough[5] = getTrackbarPos("Blured", "Parameters for Hough");
		
		//Поиск карты неравенства
		if (first_time_matching)
			//Подбор параметров на одном изображении для наилучшего отображения карты глубины
			for (;;)
			{
				imshow("capture1", gframe1);
				imshow("capture2", gframe2);
				StereoMatch(gframe1, gframe2, &disp, parametrs_for_matching);
				imshow("disparity", disp);

				parametrs_for_matching[0] = getTrackbarPos("setPreFilterCap", "Parameters for StereoMatching");
				parametrs_for_matching[1] = getTrackbarPos("setBlockSize", "Parameters for StereoMatching");
				parametrs_for_matching[2] = getTrackbarPos("setTextureThreshold", "Parameters for StereoMatching");
				parametrs_for_matching[3] = getTrackbarPos("setNumDisparities", "Parameters for StereoMatching");
				parametrs_for_matching[4] = getTrackbarPos("setUniquenessRatio", "Parameters for StereoMatching");

				char c = (char)waitKey(60);
				if (c == 27)
				{
					first_time_matching = false;
					destroyWindow("Parameters for StereoMatching");
					break;
				}
			}
		else
		{
				StereoMatch(gframe1, gframe2, &disp, parametrs_for_matching);
		}


		//Отображение окружностей на карте глубины, подсчет расстояния и отображение на изображении
		for (size_t i = 0; i < circles1.size(); i++)
		{
			coords_and_radius = circles1[i];
			circle(disp, Point(coords_and_radius[0], coords_and_radius[1]), coords_and_radius[2], Scalar(0, 0, 255), 3, LINE_AA);
			circle(disp, Point(coords_and_radius[0], coords_and_radius[1]), coords_and_radius[2], Scalar(0, 255, 0), 3, LINE_AA);
			depth = ((baseline*focal_length) / (sencorsize*disp.at<uchar>(coords_and_radius[0], coords_and_radius[1])))*0.01;
			putText(frame1, to_string(depth), Point(coords_and_radius[0], coords_and_radius[1]), 1, 1, Scalar(0, 0, 255));
		}


		//Отображение полученных изображений
		imshow("disparity", disp);
		imshow("capture1", frame1);
		imshow("capture2", frame2);


		//Ожидание нажатия клавиши
		char c = waitKey(100);
		if (c == 27) break;
	}


	destroyAllWindows();
	return 0;
}