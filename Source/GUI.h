#pragma once
#include<cv.hpp>
#include<highgui.h>

#include <memory>



class GUI {
	cv::Mat *left_pic;		//�������� � ����� ������
	cv::Mat *right_pic;		//�������� � ������ ������
	cv::Mat *disparity_map; //����� �����������
	cv::Mat left_area;
	cv::Mat right_area;
	cv::Mat	disp_area;
	std::unique_ptr<cv::Mat> display_area;
	
	//������������ �������� ��� ���������
	const int THRESHOLD_MAX = 100;
	const int ACCOMULATOR_THRESHOLD = 25;
	const int ���_DIST = 50;
	const int MIN_RAD = 30;
	const int MAX_RAD = 80;
	const int MAX_BLUR = 15;
	const int MAX_PREFILTER = 63;
	const int MAX_BLOCK_SIZE = 50;
	const int MAX_TEXTURE_THRESHOLD = 20;
	const int MAX_DISPARITI = 20;
	const int MAX_UNIQUENESS = 20;



public:
	//���������
	cv::String NAME_OF_MAIN_WINDOW = "Main Window";
	cv::String NAME_OF_OPTION_WINDOW = "Options";
	
	//������ �����������
	std::vector <cv::Vec3f> circles;

	//��������� ��� ������ �����������
	int threshold =			    23;
	int threshold_of_storadge = 21;
	int mindist	  =				40;
	int minrad	  =				9;
	int maxrad	  =				47;
	int blur	  =				7;
	
	//��������� ��� ���������� ����� �����������
	int prefilter_cap	=	31;
	int	block_size		=	 9;
	int	texture_threshold =	 9;
	int	num_disparities	=	16;
	int	uniqueness_ratio=	 0;


	//�������������� ���� ��� ��������� ����������
	void StartSetup(cv::Mat *left_pic, cv::Mat *right_pic, cv::Mat *disparity_map);

	//��������� ���������� ���� ������ ���������
	void UpdateSetup();

	//��������� ���� ������ ���������
	void EndSetup();
		
	//�������������� ���� ��� ��������� ������ ������
	void StartMainMode();
		
	//��������� ���������� ���� ��������� ������
	void UpdateMainMode();
		
	//��������� ���� �������� ������
	void EndMainMode();

};