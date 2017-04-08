#pragma once
#include<cv.hpp>
#include<highgui.h>

#include <memory>



class GUI {
	cv::Mat *left_pic;		//Картинка с левой камеры
	cv::Mat *right_pic;		//Картинка с правой камеры
	cv::Mat *disparity_map; //Карта неравенства
	cv::Mat *canny_pic;
	cv::Mat left_area;
	cv::Mat right_area;
	cv::Mat	disp_area;
	cv::Mat canny_area;
	//std::unique_ptr<cv::Mat> display_area;
	cv::Mat display_area;

	//Максимальные значения для ползунков
	const int THRESHOLD_MAX = 100;
	const int ACCOMULATOR_THRESHOLD = 25;
	const int МАХ_DIST = 50;
	const int MIN_RAD = 30;
	const int MAX_RAD = 80;
	const int MAX_BLUR = 15;
	const int MAX_PREFILTER = 63;
	const int MAX_BLOCK_SIZE = 50;
	const int MAX_TEXTURE_THRESHOLD = 20;
	const int MAX_DISPARITI = 20;
	const int MAX_UNIQUENESS = 20;



public:
	//Константы
	cv::String NAME_OF_MAIN_WINDOW = "Main Window";
	cv::String NAME_OF_OPTION_WINDOW = "Options";
	cv::String NAME_OF_WORKING_WINDOW = "Working Window";
	
	//Массив окружностей
	std::vector <cv::Vec3f> circles;

	//Параметры для поиска окружностей
	int threshold =			    56;
	int threshold_of_storadge = 2;
	int mindist	  =				13;
	int minrad	  =				14;
	int maxrad	  =				5;
	int blur	  =				5;
	
	//Параметры для вычисления карты неравенства
	int prefilter_cap		=30;
	int	block_size			=14;
	int	texture_threshold	=7;
	int	num_disparities		=16;
	int	uniqueness_ratio	=0;

	//Координаты положения маркера
	double x = 0, y = 0, z = 0;

	//Инициализирует окна для настройки параметров
	void StartSetup(cv::Mat *left_pic, cv::Mat *right_pic, cv::Mat *disparity_map, cv::Mat *canny_pic);

	//Обнавляет содержимое окон режима настройки
	void UpdateSetup();

	//Закрывает окна режима настройки
	void EndSetup();
		
	//Инициализирует окна для основного режима работы
	void StartMainMode();
		
	//Обнавляет содержимое окон основного режима
	void UpdateMainMode();
		
	//Закрывает окна осовного режима
	void EndMainMode();

};