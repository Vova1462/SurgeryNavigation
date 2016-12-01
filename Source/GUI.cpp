#include "GUI.h"


using namespace cv;


void GUI::StartSetup(cv::Mat *left_p, cv::Mat *right_p, cv::Mat *disp_map) {


	left_pic = left_p;
	right_pic = right_p;
	disparity_map = disp_map;

	//Расстояние между изображениями
	const int margin = 10;

	//Размер экрана
	Size display_area_size(left_p->cols + right_p->cols + margin * 3,
		left_p->rows + right_p->rows + margin * 3);

	//Область для отображения изображений и параметров
	display_area = std::make_unique<Mat>(display_area_size, CV_8UC1);

	//Области для отображения изображений внутри основного окна
	left_area = (*display_area)(Rect(0, 0, left_p->cols, left_p->rows));
	right_area = (*display_area)(Rect(left_p->cols + margin, 0, right_p->cols, right_p->rows));
	disp_area = (*display_area)(Rect(0, left_p->rows + margin, disp_map->cols, disp_map->rows));

	//Создание окна
	namedWindow(NAME_OF_MAIN_WINDOW, WINDOW_AUTOSIZE);
	namedWindow(NAME_OF_OPTION_WINDOW, WINDOW_AUTOSIZE);

	//Создание ползунков
	createTrackbar("Threshold", NAME_OF_OPTION_WINDOW, &threshold, THRESHOLD_MAX);
	createTrackbar("Thresholdofstoradge", NAME_OF_OPTION_WINDOW, &threshold_of_storadge, ACCOMULATOR_THRESHOLD);
	createTrackbar("Mindist", NAME_OF_OPTION_WINDOW, &mindist, МАХ_DIST);
	createTrackbar("MinRad", NAME_OF_OPTION_WINDOW, &minrad, MIN_RAD);
	createTrackbar("MaxRad", NAME_OF_OPTION_WINDOW, &maxrad, MAX_RAD);
	createTrackbar("Blured", NAME_OF_OPTION_WINDOW, &blur, MAX_BLUR);

	createTrackbar("setPreFilterCap", NAME_OF_OPTION_WINDOW, &prefilter_cap, MAX_PREFILTER);
	createTrackbar("setBlockSize", NAME_OF_OPTION_WINDOW, &block_size, MAX_BLOCK_SIZE);
	createTrackbar("setTextureThreshold", NAME_OF_OPTION_WINDOW, &texture_threshold, MAX_TEXTURE_THRESHOLD);
	createTrackbar("setNumDisparities", NAME_OF_OPTION_WINDOW, &num_disparities, MAX_DISPARITI);
	createTrackbar("setUniquenessRatio", NAME_OF_OPTION_WINDOW, &uniqueness_ratio, MAX_UNIQUENESS);
}

void GUI::UpdateSetup() {
	
	//Обновление значений параметров для поиска окружностей
	threshold			  = getTrackbarPos("Threshold", NAME_OF_OPTION_WINDOW);
	threshold_of_storadge = getTrackbarPos("Thresholdofstoradge", NAME_OF_OPTION_WINDOW);
	mindist				  = getTrackbarPos("Mindist", NAME_OF_OPTION_WINDOW);
	minrad				  = getTrackbarPos("MinRad", NAME_OF_OPTION_WINDOW);
	maxrad				  = getTrackbarPos("MaxRad", NAME_OF_OPTION_WINDOW);
	blur				  = getTrackbarPos("Blured", NAME_OF_OPTION_WINDOW);

	//Обновление значений параметров для поиска карты глубины
	prefilter_cap		  = getTrackbarPos("setPreFilterCap", NAME_OF_OPTION_WINDOW);
	block_size			  = getTrackbarPos("setBlockSize", NAME_OF_OPTION_WINDOW);
	texture_threshold	  = getTrackbarPos("setTextureThreshold", NAME_OF_OPTION_WINDOW);
	num_disparities		  =	getTrackbarPos("setNumDisparities", NAME_OF_OPTION_WINDOW);
	uniqueness_ratio	  =	getTrackbarPos("setUniquenessRatio", NAME_OF_OPTION_WINDOW);

	//Отрисовка окружностей
	for (int i = 0;i < circles.size();i++)
		circle(*left_pic,Point(circles[i][0],circles[i][1]),circles[i][2],Scalar(255,255,255),3,LINE_AA);
	
	//Копирование изображений в области
	left_pic->copyTo(left_area);
	right_pic->copyTo(right_area);
	disparity_map->copyTo(disp_area);

	//Отображение основной области в окне
	imshow(NAME_OF_MAIN_WINDOW, *display_area);
}


void GUI::EndSetup()
{
	destroyWindow(NAME_OF_OPTION_WINDOW);
}


void GUI::StartMainMode() {


}

void GUI::UpdateMainMode()
{

	//Отрисовка окружностей
	for (int i = 0;i < circles.size();i++)
	{
		//Отрисовка окружностей
		circle(*left_pic, Point(circles[i][0], circles[i][1]), circles[i][2], Scalar(255, 255, 255), 3, LINE_AA);

		//Вывод координат X,Y,Z
		/*putText(*left_pic, to_string(Z), Point(circles[i][0], circles[i][1]), 1, 1, Scalar(0, 0, 255));
		putText(*left_pic, to_string(X), Point(circles[i][0], circles[i][1] + 15), 1, 1, Scalar(0, 0, 255));
		putText(*left_pic, to_string(Y), Point(circles[i][0], circles[i][1] + 30), 1, 1, Scalar(0, 0, 255));*/
	}

	//Копирование изображений в области
	left_pic->copyTo(left_area);
	right_pic->copyTo(right_area);
	disparity_map->copyTo(disp_area);

	//Отображение основной области в окне
	imshow(NAME_OF_MAIN_WINDOW, *display_area);
}




void GUI::EndMainMode() {
	destroyWindow(NAME_OF_MAIN_WINDOW);
}
