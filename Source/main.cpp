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
	FileStorage fs("C:\\dev\\MyProjects\\SurgeryNavigation\\intrinsics.yml", FileStorage::WRITE);
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

	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";

	Mat _M1;
	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\intrinsics.yml", FileStorage::READ);
	

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
	
	
	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\mx1.yml",FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "mx1" << rmap[0][0] << "my1" << rmap[0][1];
		fs << "mx2" << rmap[1][0]<<"my2"<<rmap[1][1];
		fs.release();
	}
	

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


static void StereoMatch(int iteration, Mat capture1,Mat capture2/*, Rect roi1, Rect roi2*/)
{
	Mat R1, R2, R, T, P1, P2, M1, M2, D1, D2, Q, mx1, mx2, my1, my2;
	Rect roi1, roi2;
	int prefiltercap = 31, blocksize = 9, texturethreshold = 16, numdisparity = 16, uniquenessratio = 15;

	Size imgsize = capture1.size();
	Mat disp(capture1.rows, capture1.cols,CV_16S), vdisp(capture1.rows, capture1.cols, CV_8U),img1rect,img2rect;

	FileStorage fs;
	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\extrinsics.yml",FileStorage::READ);
	if (fs.isOpened())
	{
		fs["R"] >> R;
		fs["T"]>> T;
		fs["R1"] >> R1;
		fs["P1"] >> R1;
		fs["R2"] >> R2;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
	}
	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\intrinsics.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["M1"]>>M1;
		fs["M2"] >> M2;
		fs["D1"] >> D1;
		fs["D2"] >> D2;
	}
	//поставить коментарий
	fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\mx1.yml", FileStorage::READ);
	if (fs.isOpened())
	{
		fs["mx1"] >> mx1;
		fs["my1"] >> my1;
		fs["mx2"] >> mx2;
		fs["my2"] >> my2;
	}


	namedWindow("Parametars", 1);
	createTrackbar("setPreFilterCap", "Parametars", &prefiltercap, 63);
	createTrackbar("setBlockSize", "Parametars", &blocksize, 100);
	createTrackbar("setTextureThreshold", "Parametars", &texturethreshold, 20);
	createTrackbar("setNumDisparities", "Parametars", &numdisparity, 20);
	createTrackbar("setUniquenessRatio", "Parametars", &uniquenessratio, 20);

	//¬ычисление характеристик дл€ дальнейшего изображений
	stereoRectify(M1, D1, M2, D2, imgsize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imgsize, &roi1,&roi2);
	
	//ѕерестраивает изображени€ дл€ исправлени€ оптических искажений 
	remap(capture1, img1rect, mx1, my1, INTER_LINEAR);
	remap(capture2, img2rect, mx2, my2, INTER_LINEAR);

	capture1 = img1rect;
	capture2 = img2rect;

	Ptr<StereoBM> bm = StereoBM::create(64, 21);
	//Ћогические отступы и добавить коментарии
	if (iteration == 0)
	{
		imshow("capture1", capture1);
		imshow("capture2", capture2);
		for (;;)
		{

			bm->setROI1(roi1);
			bm->setROI2(roi2);
			if (getTrackbarPos("setPreFilterCap", "Parametars") > 0)
				bm->setPreFilterCap(getTrackbarPos("setPreFilterCap", "Parametars"));
			if (getTrackbarPos("setBlockSize", "Parametars") % 2 != 0 && getTrackbarPos("setBlockSize", "Parametars") > 3)
				bm->setBlockSize(getTrackbarPos("setBlockSize", "Parametars"));
			bm->setMinDisparity(0);
			if (getTrackbarPos("setTextureThreshold", "Parametars") > 0)
				bm->setNumDisparities(getTrackbarPos("setTextureThreshold", "Parametars") * 16);
			bm->setTextureThreshold(getTrackbarPos("setNumDisparities", "Parametars"));
			bm->setUniquenessRatio(getTrackbarPos("setUniquenessRatio", "Parametars"));
			bm->setSpeckleWindowSize(100);
			bm->setSpeckleRange(32);
			bm->setDisp12MaxDiff(1);

			bm->compute(capture1, capture2, disp);
			normalize(disp, vdisp, 0, 1, CV_MINMAX);


			disp.convertTo(vdisp, CV_8U);
			imshow("DisparityMap", vdisp);
			char c = waitKey(100);
			if (c == 27)
			{
				//сделать функцию дл€ изменени€ параметров и устранить copypaste
				fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\params.yml", FileStorage::WRITE);
				if (fs.isOpened())
				{
					fs << "PreFilterCap" << getTrackbarPos("setPreFilterCap", "Parametars") << "BlockSize" << getTrackbarPos("setBlockSize", "Parametars");
					fs << "TextureThreshold" << getTrackbarPos("setTextureThreshold", "Parametars") << "NumDisparities" << getTrackbarPos("setNumDisparities", "Parametars");
					fs << "UniquenessRatio" << getTrackbarPos("setUniquenessRatio", "Parametars");
				}
				break;
			}

		}
	}
	else
	{
		fs.open("C:\\dev\\MyProjects\\SurgeryNavigation\\params.yml", FileStorage::READ);
		if (fs.isOpened())
		{
			fs["PreFilterCap"] >> prefiltercap;
			fs["BlockSize"] >> blocksize;
			fs["TextureThreshold"] >> texturethreshold;
			fs["NumDisparities"] >> numdisparity;
			fs["UniquenessRatio"] >> uniquenessratio;
		}
		bm->setROI1(roi1);
		bm->setROI2(roi2);
		if (prefiltercap>0)
			bm->setPreFilterCap(prefiltercap);
		if (blocksize % 2 != 0 && blocksize > 3)
			bm->setBlockSize(blocksize);
		bm->setMinDisparity(0);
		if (numdisparity > 0)
			bm->setNumDisparities(numdisparity * 16);
		bm->setTextureThreshold(texturethreshold);
		bm->setUniquenessRatio(uniquenessratio);
		bm->setSpeckleWindowSize(100);
		bm->setSpeckleRange(32);
		bm->setDisp12MaxDiff(1);
		bm->compute(capture1, capture2, disp);
		normalize(disp, vdisp, 0, 1, CV_MINMAX);
		disp.convertTo(vdisp, CV_8U);
		imshow("DisparityMap", vdisp);
	}
}
static void FindCircles(Mat capture1, int thresholdofCanny, int thresholdofstoradge, int dp,int mindist, int minrad,int maxrad,
	vector<Vec3f> circles, int *x,int *y, int *heigth, int *width)
{
	int xmin=0, ymin=0, xmax=0, ymax=0;
	HoughCircles(capture1, circles, HOUGH_GRADIENT,
		dp, mindist, thresholdofCanny+500, thresholdofstoradge, minrad, maxrad);
	
	Vec3i coords_and_radius;//¬ектор дл€ чтени€ координат окружности и радиуса, при отрисовке
	
	int vec_nums[4]; //ћассив дл€ хранени€ номеров векторов, которые хран€т радиус окружности

	for (size_t i = 0; i < circles.size(); i++)
	{
		coords_and_radius = circles[i];
		circle(capture1, Point(coords_and_radius[0], coords_and_radius[1]), coords_and_radius[2], Scalar(0, 0, 255), 3, LINE_AA);
		/*circle(capture1, Point(c[0], c[1]), 2, Scalar(0, 255, 0), 3, LINE_AA);*/

		//ѕоиск максимальных и минимальных координат центров окружностей
		if (xmax <= coords_and_radius[0])
		{xmax = coords_and_radius[0];vec_nums[0] = i;}
		if (ymax <= coords_and_radius[1])
		{ymax = coords_and_radius[1];vec_nums[1] = i;}
		if (xmin >= coords_and_radius[0])
		{xmin = coords_and_radius[0];vec_nums[2] = i;}
		if (ymin >= coords_and_radius[1])
		{ymin = coords_and_radius[1];vec_nums[3] = i;}
	}
		/*x = floor(xmin -2*circles[t[2]][2]);
		y = floor(ymin - 2 * circles[t[3]][2]);
		heigth = ceil(xmax + 2 * circles[t[0]][2] - xmin + 2 * circles[t[2]][2]);
		width = ceil(ymax + 2 * circles[t[1]][2] - ymin + 2 * circles[t[3]][2]);*/
}



int main()
{
	char a;
	Mat frame1, frame2,gframe1,gframe2;
	/*Rect roi1, roi2;*/
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
	

	namedWindow("capture1", 1);
	namedWindow("capture2", 1);
	namedWindow("Options", 1);
	
	vector<Vec3f> circles1, circles2;
	int thresholdofCanny=23, thresholdofstoradge=21,t=0,dp=1, minrad=9,maxrad=40, mindist=47,x,y,heigth, width;

	createTrackbar("Threshold", "Options", &thresholdofCanny, 100, 0);
	createTrackbar("Thresholdofstoradge", "Options", &thresholdofstoradge, 25, 0);
	createTrackbar("dp", "Options", &dp, 50, 0);
	createTrackbar("Mindist", "Options", &mindist, 50, 0);
	createTrackbar("MinRad", "Options", &minrad, 30, 0);
	createTrackbar("MaxRad", "Options", &maxrad, 80, 0);
	for (;;)
	{
		cap1 >> frame1;
		cap2 >> frame2;

		gframe1.create(480, 640, CV_8UC1);
		gframe2.create(480, 640, CV_8UC1);

		cvtColor(frame1, gframe1, CV_BGR2GRAY);
		cvtColor(frame2, gframe2, CV_BGR2GRAY);

		//исправить на Find в имени, сделать указатель на массив
		FindCircles(gframe1,
			getTrackbarPos("Threshold","Options"), 
			getTrackbarPos("Thresholdofstoradge", "Options"), 
			getTrackbarPos("dp", "Options"),
			getTrackbarPos("Mindist", "Options"),
			getTrackbarPos("MinRad", "Options"),
			getTrackbarPos("MaxRad", "Options"),
			circles1, &x, &y, &heigth, &width);
		Rect roi1(x, y, heigth, width);
		
		FindCircles(gframe2, 
			getTrackbarPos("Threshold", "Options"), 
			getTrackbarPos("Thresholdofstoradge", "Options"), 
			getTrackbarPos("dp", "Options"), 
			getTrackbarPos("Mindist", "Options"),
			getTrackbarPos("MinRad", "Options"),
			getTrackbarPos("MaxRad", "Options"),
			circles2, &x, &y, &heigth, &width);
		//Rect roi2(x, y, heigth, width);
		
		StereoMatch(t,gframe1,gframe2/*,roi1,roi2*/);

		imshow("capture1", gframe1);
		imshow("capture2", gframe2);
		
		t++;
		char c = waitKey(100);
		if (c == 27) break;
	}
	return 0;
}