#include<Calibration.h>

void Calibration::SCalibration(Customozation *cust)
{
	int nx = 10, ny = 7, cycle=40, result1=0, result2=0; //����������� ����� �� ���� � � �, ����������� ������, ���������� ������ �����
	int n = nx*ny; //����������� ����� �� �������� �����
	int successes1 = 0, successes2 = 0;
	int frame = 0, corners_count1=0, corners_count2=0, step=0;
	CvSize board_size = cvSize(nx, ny); //������ �����
	vector<CvPoint2D32f> temp1(n);
	vector<CvPoint2D32f> temp2(n);
	
	//������ ���-�����
	cust->stereoGrabberGrabFrames();
	cust->stereoGrabberInitFrames();
	IplImage *frame1 = cust->imageLeft;
	IplImage *frame2 = cust->imageRight; //������ ����������� � 1 � 2 ������
	IplImage *gr_frame1,*gr_frame2;

	//��������� ������ ��� ���������

	CvMat* image_points1 = cvCreateMat(cycle*n,2,CV_32FC1);
	CvMat* image_points2 = cvCreateMat(cycle*n, 2, CV_32FC1);
	CvMat* object_points1 = cvCreateMat(cycle*n, 3, CV_32FC1);
	CvMat* object_points2 = cvCreateMat(cycle*n, 3, CV_32FC1);
	CvMat* point_counts1 = cvCreateMat(cycle, 1, CV_32SC1);
	CvMat* point_counts2 = cvCreateMat(cycle, 1, CV_32SC1);
	//�������� ���� ��� ����������� ����������� ����������� � �����
	cvNamedWindow("Camera 1", 1);
	cvNamedWindow("Camera 2", 1);

	while ((successes1>cycle)||(successes2>cycle))
	{
		//�������� 20 ������ ����� � ������ �������� ��� ������� �����������
		if (frame++ % 20 == 0)
		{
			//������� ��� ������ ����� �� �����(� ��������� �������), �� ���������� ������������� � ������� ������ ��������� ��� ���������� �������� ����������
			result1 = cvFindChessboardCorners(frame1,
				board_size,
				&temp1[0],
				NULL,
				CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);
			result2 = cvFindChessboardCorners(frame2,
				board_size,
				&temp2[0],
				NULL,
				CV_CALIB_CB_NORMALIZE_IMAGE | CV_CALIB_CB_FILTER_QUADS);

			//����������� ����������� � �������� ������
			cvConvertImage(frame1, gr_frame1, CV_8S);
			cvConvertImage(frame1, gr_frame1, CV_8S);

			//������� ��� ������ ����� �� �����(� ��������� ����������), �� ���������� ������������� � ������� ������ ��������� ��� ���������� �������� ����������
			cvFindCornerSubPix(gr_frame1,
				&temp1[0],
				corners_count1,
				cvSize(11, 11),
				cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 0.01));

			cvFindCornerSubPix(gr_frame2,
				&temp2[0],
				corners_count2,
				cvSize(11, 11),
				cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 100, 0.01));

			//��������� �������� �����
			cvDrawChessboardCorners(frame1,
				board_size,
				&temp1[0],
				corners_count1,
				result1);
			cvDrawChessboardCorners(frame2,
				board_size,
				&temp2[0],
				corners_count2,
				result2);

			//����������� �����������
			cvShowImage("Camera1", frame1);
			cvShowImage("Camera2", frame2);

			
			//���������� ��������(� ������� ��������������� ��� ����) ������������� ����� � ���������
			if (corners_count1 == n || corners_count2 == n)
			if (corners_count1==n)
			{
				step = successes1*n;
				for (int i = step, j = 0; j < n; ++i, ++j)
				{
					CV_MAT_ELEM(*image_points1, float,i,0)=temp1[j].x;
					CV_MAT_ELEM(*image_points1, float, i, 1) = temp1[j].y;
					CV_MAT_ELEM(*object_points1, float, i, 0) = j / nx;
					CV_MAT_ELEM(*object_points1, float, i, 1) = j%nx;
					CV_MAT_ELEM(*object_points1, float, i, 1) = 0.0f;
				}
			}
			if (corners_count2==n)
			{
				step = successes2*n;
				for (int i = step, j = 0; j < n; ++i, ++j)
				{
					CV_MAT_ELEM(*image_points2, float, i, 0) = temp2[j].x;
					CV_MAT_ELEM(*image_points2, float, i, 1) = temp2[j].y;
					CV_MAT_ELEM(*object_points2, float, i, 0) = j / nx;
					CV_MAT_ELEM(*object_points2, float, i, 1) = j%nx;
					CV_MAT_ELEM(*object_points2, float, i, 1) = 0.0f;
				}
				CV_MAT_ELEM(*point_counts1, int, successes1, 0) = n;
				CV_MAT_ELEM(*point_counts2, int, successes2, 0) = n;
				successes1++;
				successes2++;
			}
		}

		//������ �������� ����������
		cust->stereoGrabberGrabFrames();
		frame1 = cust->imageLeft;
		cvShowImage("Camera 1",frame1);
		frame2 = cust->imageRight;
		cvShowImage("Camera 1", frame2);

		//�������� ������� ������� ��� ����������
		if (cvWaitKey(15) == 27) break;
	}

	//���������� ������
	cvCalibrateCamera2(
		object_points1,
		image_points1,
		point_counts1,
		cvGetSize(frame1),
		
		);

	//���������� ������ �����
	cust->stereoGrabberStopCam();
	
}