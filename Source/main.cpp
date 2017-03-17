#include<cv.hpp>
#include<opencv2\videoio.hpp>
#include <string>
#include <iostream>
#include <iterator> 
#include <opencv2\opencv.hpp>
#include"GUI.h"

using namespace std;
using namespace cv;

//static bool readStringList(const string& filename, vector<string>& l)
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

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

void SaveParams(Mat &M1, Mat &M2, Mat &D1, Mat &D2, Mat &R, Mat &T, Mat &R1, Mat &R2,Mat &P1, Mat &P2, Mat &Q) {
	FileStorage ifs("intrinsics.yml", FileStorage::WRITE),efs("extrinsics.yml",FileStorage::WRITE);
	if (ifs.isOpened()) {
		ifs << "M1" << M1 << "D1" << D1 << "M2" << M2 << "D2" << D2;
		ifs.release();
	}
	else
	{
		cout << "Error: can not save the intrinsics parametrs\n";
	}
	if (efs.isOpened()) {
		efs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 
			<< "P1" << P1 << "P2" << P2 << "Q" << Q;
	}
	else {
		cout << "Error: can not save the extrinsics parametrs\n";
	}
}


static void Calibration(VideoCapture camera_left, VideoCapture camera_right, int number_of_good_boards) {

	String WINDOW_NAME = "Window calibration";

	double square_size = 0.1;
	Size board_size, img_size;
	//Массивы для хранения изображения
	Mat img_left, img_right, display_area;
	
	//Массив для перестроения изображения
	Mat rmap[2][2];

	//
	Rect validRoi[2];
	
	//Режим работы калибровки
	int mode = DETECTION;

	//Получение колличества углов по ширине и высоте
	cout << "Type number of corners width and heigth\n";
	cin >> board_size.width >> board_size.height;

	//Массив для хранения углов шахматной доски
	vector<vector<Point2f>> imagePoints[2];


	//создание окна отображения
	namedWindow(WINDOW_NAME, 1);

	//Флаг, показывающий, нашлись ли углы
	bool found_left = false, found_right = false;

	for (int i = 0;;i++) {

		//Захват изображений с камеры
		camera_left >> img_left;
		camera_right >> img_right;
		img_size = img_left.size();
		//Создание области отображения двух изображений
		display_area.create(Size(img_left.cols + img_right.cols,img_left.rows), img_left.type());

		//Присваивание области отображения левому и правому снимку
		Mat left_area = (display_area)(Rect(0, 0, img_left.cols, img_left.rows));
		Mat right_area = (display_area)(Rect(img_left.cols, 0, img_right.cols, img_right.rows));

		//Создание массивов для хранения изображений в градациях серого
		Mat gray_left, gray_right;

		//Создание массива для хранения параметров каждой камеры
		Mat camera_matrix_left,camera_matrix_right, dist_coeffs[2];

		//Конвертация в градации серого
		cvtColor(img_left, gray_left, COLOR_BGR2GRAY);
		cvtColor(img_right, gray_right, COLOR_BGR2GRAY);

		//Хранилище координат углов
		vector<Point2f> point_buf_left;
		vector<Point2f> point_buf_right;

		//Поиск углов на шахматной доске
		found_left = findChessboardCorners(gray_left, board_size, point_buf_left,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
		found_right = findChessboardCorners(gray_right, board_size, point_buf_right,
			CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);

		//Поиск углов на шахматной доске на уровне суб-пикселей
		if (found_left&&found_right) {
			cornerSubPix(gray_left, point_buf_left, Size(11, 11), Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
			cornerSubPix(gray_right, point_buf_right, Size(11, 11), Size(-1, -1),
				TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
		}

		//
		if (mode == CAPTURING&&found_left&&found_right) {
			imagePoints[0].push_back(point_buf_left);
			imagePoints[1].push_back(point_buf_right);
		}

		//Отображение углов на изображении
		if (found_left&&found_right) {
			drawChessboardCorners(img_left, board_size, point_buf_left, found_left);
			drawChessboardCorners(img_right, board_size, point_buf_right, found_right);
		}

		//Вывод сообщения на экран
		string msg = mode == CAPTURING ? "100/100" :
			mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
		if (mode == CAPTURING)
		{
			msg = format("%d/%d", (int)imagePoints[0].size(), number_of_good_boards);
		}
		putText(img_left, msg, Point(150, 400), 1, 1, Scalar(255, 0, 0), 2);
		putText(img_right, msg, Point(150, 400), 1, 1, Scalar(255, 0, 0), 2);



		if (mode == CAPTURING&&imagePoints[0].size() == number_of_good_boards) {
			vector<vector<Point3f>> objectPoints(number_of_good_boards);

			//
			for (int i = 0;i < number_of_good_boards;++i)
				for (int j = 0;j < board_size.height;++j)
					for (int k = 0;k < board_size.width;++k)
						objectPoints[i].push_back(Point3f(k*square_size, j*square_size, 0));

			//
			imagePoints[0].resize(number_of_good_boards);
			imagePoints[1].resize(number_of_good_boards);

			//
			camera_matrix_left = initCameraMatrix2D(objectPoints, imagePoints[0], img_size, 0);
			camera_matrix_right = initCameraMatrix2D(objectPoints, imagePoints[1], img_size, 0);
			//Инициализация матриц вращения, сдвига систем координат, существенная и фундаментальная
			Mat R, T, E, F;

			//Минимизация и расчет среднеквадратической ошибки перепроецирования точек
			double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
				camera_matrix_left, dist_coeffs[0],
				camera_matrix_right, dist_coeffs[1],
				img_size, R, T, E, F,
				CALIB_FIX_ASPECT_RATIO +
				CALIB_ZERO_TANGENT_DIST +
				CALIB_USE_INTRINSIC_GUESS +
				CALIB_SAME_FOCAL_LENGTH +
				CALIB_RATIONAL_MODEL +
				CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
				TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));

			//Вывод минимальной ошибки
			cout << "Done with RMS error =" << rms << endl;


			//
			double err = 0;
			int npoints = 0;
			vector<Vec3f> lines[2];
			for (int i = 0; i < number_of_good_boards; i++)
			{
				int npt = (int)imagePoints[0][i].size();
				Mat imgpt_left,imgpt_right;

					imgpt_left = Mat(imagePoints[0][i]);
					undistortPoints(imgpt_left, imgpt_left, camera_matrix_left, dist_coeffs[0], Mat(), camera_matrix_left);
					computeCorrespondEpilines(imgpt_left, 1, F, lines[0]);
					imgpt_right = Mat(imagePoints[1][i]);
					undistortPoints(imgpt_right, imgpt_right, camera_matrix_right, dist_coeffs[1], Mat(), camera_matrix_right);
					computeCorrespondEpilines(imgpt_right, 2, F, lines[1]);

				for (int j = 0; j < npt; j++)
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

			//
			Mat R1, R2, P1, P2, Q;
			stereoRectify(camera_matrix_left, dist_coeffs[0],
				camera_matrix_right, dist_coeffs[1],
				img_size, R, T, R1, R2, P1, P2, Q,
				0, 1, img_size, &validRoi[0], &validRoi[1]);
			
			//
			initUndistortRectifyMap(camera_matrix_left, dist_coeffs[0], R1, P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);
			initUndistortRectifyMap(camera_matrix_right, dist_coeffs[1], R2, P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);

			//Функция сохранения параметров
			SaveParams(camera_matrix_left,camera_matrix_right,dist_coeffs[0],dist_coeffs[1],R,T,R1,R2,P1,P2,Q);
			
			//Переключение режима
			mode = CALIBRATED;
		}

		char c = waitKey(50);
		if (c == 27) {
			destroyWindow(WINDOW_NAME);
			break;
		}

		if (c == 'g')
		{
			mode = CAPTURING;
		}

		if (c == 'u'&&mode == CALIBRATED) {
			remap(img_left, img_left, rmap[0][0], rmap[0][1],INTER_LINEAR);
			remap(img_right, img_right, rmap[1][0], rmap[1][1], INTER_LINEAR);
			for (int i = 0;i < img_left.rows;i += 16) {
				line(img_left, Point(0, i), Point(img_left.cols, i), Scalar(0, 255, 0), 1, 8);
				line(img_right, Point(0, i), Point(img_right.cols, i), Scalar(0, 255, 0), 1, 8);
			}
			rectangle(img_left, validRoi[0], Scalar(255, 0, 0), 2, 8);
			rectangle(img_right, validRoi[1], Scalar(255, 0, 0), 2, 8);
		}

		img_left.copyTo(left_area);
		img_right.copyTo(right_area);

		imshow(WINDOW_NAME, display_area);
	}
}

static void GetCropedImage(Mat *capture1, Mat *capture2, Rect *roi1, Rect *roi2, Size imgsize, double *cx, double *cy)
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

	//Получение координаты оптической оси
	*cx = M1.at<double>(0, 2);
	*cy = M1.at<double>(1, 2);

	//Вычисление областей для работы с изображением
	stereoRectify(M1, D1, M2, D2, imgsize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, imgsize, roi1, roi2);
	
	//Вычисление координат длz исправления искажений на изображниях
	initUndistortRectifyMap(M1, D1, R1, P1, imgsize, CV_16SC2, rect_map[0][0], rect_map[0][1]);
	initUndistortRectifyMap(M2, D2, R2, P2, imgsize, CV_16SC2, rect_map[1][0], rect_map[1][1]);

	//Перестраивает изображения для исправления оптических искажений
	remap(*capture1, img1rect, rect_map[0][0], rect_map[0][1], INTER_LINEAR);
	remap(*capture2, img2rect, rect_map[1][0], rect_map[1][1], INTER_LINEAR);

	////Обрезка изображений
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

static void StereoMatch(Mat capture1,Mat capture2, Mat *disp, int prefilter_cap,int block_size,int texture_threshold, int num_disparities, int uniquness_ratio)
{


	Size imgsize = capture1.size();
	Mat vdisp(capture1.rows, capture1.cols, CV_16U);

	Ptr<StereoBM> bm = StereoBM::create(64, 21);

			if (prefilter_cap > 0)
				bm->setPreFilterCap(prefilter_cap);
			if (block_size > 1)
				bm->setBlockSize(block_size * 2 + 1);
			bm->setMinDisparity(0);
			if (texture_threshold > 0)
				bm->setNumDisparities(texture_threshold * 16);
			bm->setTextureThreshold(num_disparities);
			bm->setUniquenessRatio(uniquness_ratio);
			bm->setSpeckleWindowSize(100);
			bm->setSpeckleRange(32);
			bm->setDisp12MaxDiff(1);
			//Вычисление карты глубины по предыдущем параметрам
			bm->compute(capture1, capture2, vdisp);

			vdisp.convertTo(*disp, CV_8U, 1 / 16.);
}

static void FindCircles(Mat capture, Mat *canny_pic, vector<Vec3f> *circles, int threshold, 
						int threshold_of_storadge, int min_dist, int min_radius, 
						int max_radius, int blur)
{
	Mat blured;
	
	//Сглаживание изображения, для уменьшения шума на изображении
	medianBlur(capture, blured, blur*2+1);
	Canny(blured, *canny_pic, threshold+50, threshold_of_storadge+100);

	//Поиск окружностей на изображении
	HoughCircles(blured, *circles, HOUGH_GRADIENT,
		1, 
		threshold+50, 
		threshold_of_storadge+100, 
		min_dist, 
		max_radius, 
		min_radius);
}

int main(int argc, char** argv)
{

	char a;
	const string img_prefix = "C:\\dev\\MyProjects\\SurgeryNavigation\\Data\\Images\\";
	const string postfix = ".png";
	Vec3f coords_and_radius;
	Rect roi1, roi2;
	double Z=0, baseline=atof(argv[2]), focal_length=5.5, sencor_size_horizontal=5.5, sencor_size_vertical =2.75, X=0, Y=0,cx=0,cy=0;
	double fps = atof(argv[1]);
	GUI gui;
	
	//Инициализация камер
	VideoCapture cap1(2);
	if (!cap1.isOpened())
		return -1;

	VideoCapture cap2(0);
	if (!cap2.isOpened())
		return -2;

	//Калибровка камер
	cout << "Do you need calibration?Y/N\n";
	cin >> a;
	if (a == 'Y')
	{
		Calibration(cap1,cap2,12);
	}

	//Настройка частоты захвата изображения
	cap1.set(CV_CAP_PROP_FPS, fps);
	cap2.set(CV_CAP_PROP_FPS, fps);
	//fps=cap1.get(CV_CAP_PROP_FPS);


	int parametrs_for_matching[5],parametrs_for_hough[6];

	vector<Vec3f> circles1, circles2;

	bool setup_needed = true;
	const int image_heigth = 480;
	const int image_width = 640;
	Mat frame_left{ image_heigth,image_width,CV_16SC3 }, 
				frame_right{ frame_left }, gframe_left{ frame_left }, 
				gframe_right{ frame_left }, disp{ frame_left }, canny_pic{ frame_left };

	gui.StartSetup(&gframe_left, &gframe_right, &disp, &canny_pic);

	// Основной цикл
	for (;;)
	{
		cap1 >> frame_left;
		cap2 >> frame_right;

		//frame_left = imread(img_prefix + "1_2" + postfix, 1);
		//frame_right = imread(img_prefix + "2_2" + postfix, 1);

		//Создания изображений
		gframe_left.create(480, 640, CV_8U);
		gframe_right.create(480, 640, CV_8U);

		//Конвертация изображений в градации серого
		cvtColor(frame_left, gframe_left, CV_BGR2GRAY);
		cvtColor(frame_right, gframe_right, CV_BGR2GRAY);

		//Нормализация изображений
		//Получение гистограм правого и левого изображения
		/*MatND ghist_left,ghist_right;
		int hist_size = 256;
		float range[] = { 0, 256 };
		const float* hist_range = { range };
		bool uniform = true;
		bool accumulate = false;
		int hist_width = 512, hist_heigth = 400, bin_w = round((double)hist_width / hist_size);

*/
		//calcHist(&gframe_left, 1, 0, Mat(), ghist_left, 1, &hist_size, &hist_range,true,false);
		//calcHist(&gframe_right, 1, 0, Mat(), ghist_right, 1, &hist_size, &hist_range, true, false);
		//
		//		
		//Mat histogram_left(hist_heigth, hist_width, CV_8U),histogram_right(hist_heigth, hist_width, CV_8U);
		//
		//normalize(ghist_left, ghist_left, 0, histogram_left.rows, NORM_MINMAX, -1, Mat());
		//normalize(ghist_right, ghist_right, 0, histogram_right.rows, NORM_MINMAX, -1, Mat());
		//
		//for (int i = 1;i < hist_size;i++) {
		//	line(histogram_left, Point(bin_w*(i - 1), hist_heigth - round(ghist_left.at<float>(i - 1))),
		//		Point(bin_w*(i), hist_heigth - round(ghist_left.at<float>(i))),
		//		Scalar(0, 255, 255), 2, 8, 0);
		//	line(histogram_right, Point(bin_w*(i - 1), hist_heigth - round(ghist_right.at<float>(i - 1))),
		//						Point(bin_w*(i), hist_heigth - round(ghist_right.at<float>(i))),
		//						Scalar(0, 255, 255), 2, 8, 0);
		//}
		
		//Нормализация гистограм
		equalizeHist(gframe_left, gframe_left);
		equalizeHist(gframe_right, gframe_right);
		

		
		//Исправление изображений
		GetCropedImage(&gframe_left, &gframe_right, &roi1, &roi2, gframe_left.size(),&cx,&cy);
		
		//Уменьшение шума на изображениях
		medianBlur(gframe_left, gframe_left, 3);
		medianBlur(gframe_right, gframe_right, 3);

		//Поиск маркеров
		FindCircles(gframe_left, &canny_pic, &gui.circles, 
					gui.threshold, 
					gui.threshold_of_storadge, 
					gui.mindist, 
					gui.minrad, 
					gui.maxrad, 
					gui.blur);
		/*FindCircles(gframe_right, &circles2, 
					gui.threshold,
					gui.threshold_of_storadge,
					gui.mindist,
					gui.minrad,
					gui.maxrad,
					gui.blur);*/
				


		//Поиск карты неравенства
		if (setup_needed)
			//Подбор параметров на одном изображении для наилучшего отображения карты глубины
			for (;;) {

				FindCircles(gframe_left, &canny_pic, &gui.circles,
					gui.threshold,
					gui.threshold_of_storadge,
					gui.mindist,
					gui.minrad,
					gui.maxrad,
					gui.blur);

				StereoMatch(gframe_left, gframe_right, &disp, gui.prefilter_cap,
													 gui.block_size,
													 gui.texture_threshold,
													 gui.num_disparities,
													 gui.uniqueness_ratio);
				
				gui.UpdateSetup();
				
				// Закрытие настроечных окон и настройка основного режима
				if (waitKey(60) == 27) {
					setup_needed = false;
					gui.EndSetup();
					gui.StartMainMode();
					break;
				}
			}

		// Вычисление карты глубины для основного режима
		StereoMatch(gframe_left, gframe_right, &disp, gui.prefilter_cap,
											 gui.block_size,
											 gui.texture_threshold,
											 gui.num_disparities,
											 gui.uniqueness_ratio);


		//Вычисление поля зрения камеры
		double fov_horizontal = 2 * atan(sencor_size_horizontal / 2 * focal_length);
		double fov_vertical = 2 * atan(sencor_size_vertical / 2 * focal_length);
		double average_value_of_dispar=0;
		// Подсчет координат на изображении
		for (size_t i = 0; i < gui.circles.size(); i++)
		{
			for (int j = 3;j > -2;j--)
				for (int k = 3;k > -2;k--)
					average_value_of_dispar += disp.at<uchar>(gui.circles[i][1]-k, gui.circles[i][0] - j);
			average_value_of_dispar = average_value_of_dispar / 25;

			//Вычисление расстояний
			gui.z = ((baseline*focal_length) *1000/ (sencor_size_horizontal*average_value_of_dispar));
			gui.y = 2 * sin(fov_vertical / 2)*gui.z / gframe_left.rows*(gui.circles[i][1] - cy);
			gui.x = 2 * sin(fov_horizontal / 2)*gui.z / gframe_left.cols*(gui.circles[i][0] - cx);
		}

		//Отображение полученных изображений
		gui.UpdateMainMode();

		//Ожидание нажатия клавиши
		char c = waitKey(100);
		if (c == 27) break;
	}

	gui.EndMainMode();
	return 0;
}