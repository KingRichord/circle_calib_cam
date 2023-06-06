#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

cv::Mat colorImg,grayImg;//用于存取原图，以及存取转化后的灰度图
int boardWidth = 11;//标定板上的行数
int boardHeight = 8;//标定板上的列数
std::vector < cv::Point2f > circleGridCenters;//用于记录检测到的圆的中心点（一张图片）
double circleDistance=0.003f;

std::vector < std::vector <cv::Point3f> > objectPoints;//圆点中心的世界坐标,每张图片上的点构成内层vector，所有图片构成外层vector
std::vector < std::vector <cv::Point2f> > imagePoints;//圆点中心的图像坐标,,每张图片上的点构成内层vector，所有图片构成外层vector
int main()
{
	
	VideoCapture cap;
		cap.open(0); //打开摄像头
	if(!cap.isOpened())
          return -1;
	int count =0;
	int image_count =8;
	Mat frame;
	while (1) {
		cap >> frame;//等价于cap.read(frame);
		// std::cout <<frame.size <<std::endl;
		cv::imshow("f", frame);
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		cv::Size boardSize = cv::Size(boardWidth, boardHeight);
		bool foundCircles = false;
		foundCircles = cv::findChessboardCorners(frame, boardSize, circleGridCenters,
		                                   cv::CALIB_CB_SYMMETRIC_GRID);
		
		// std::cout << "foundCircles:  " << foundCircles << std::endl;
		if (foundCircles) {
			//提取亚像素坐标
			// cv::find4QuadCornerSubpix(frame, circleGridCenters, cv::Size(5, 5));
			cv::cornerSubPix(frame, circleGridCenters, cv::Size(11, 11), cv::Size(-1, -1),
			                 TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(frame, boardSize, circleGridCenters, foundCircles);
		}
		cv::imshow("检测到的点", frame);
		std::vector<cv::Point3f> objs;//一张图片上的世界坐标
		for (int j = 0; j < boardHeight; j++) {
			for (int k = 0; k < boardWidth; k++) {
				objs.emplace_back(k * circleDistance, j * circleDistance, 0.0);
			}
		}
		cv::waitKey(10);
		if (foundCircles) {
			if (waitKey(30) == 27) //Esc键退出，ESC的ASCLL码为27
			{
				count++;
				std::cout << "count  "<<count <<std::endl;
				imagePoints.push_back(circleGridCenters);
				objectPoints.push_back(objs);
				if (imagePoints.size() > 20) {
					cv::Mat cameraMatrix;//相机内参矩阵（最后输出用）
					cv::Mat distortMatrix;//相机畸变矩阵（最后输出用）
					cv::Mat rotationMatrix;//标定板到相机的旋转矩阵（最后输出用）
					cv::Mat translationMatrix;//标定板到相机的平移矩阵(最后输出用)
					
					std::vector<cv::Mat> camRVec;//每幅图像到相机的旋转矩阵
					std::vector<cv::Mat> camTVec;//每幅图像到相机的平移矩阵
					//求内参
					int flags = 0;
					flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
					flags |= cv::fisheye::CALIB_CHECK_COND;
					flags |= cv::fisheye::CALIB_FIX_SKEW;/*非常重要*/
					cv::fisheye::calibrate(objectPoints, imagePoints, frame.size(),
					                       cameraMatrix,
					                       distortMatrix,
					                       camRVec, camTVec,
					                       flags,
					                       cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS),
					                                        200, 1e-6));
					std::cout << "calibrateCamera已通过" << std::endl;
					std::cout << "cameraMatrix:  " << cameraMatrix << std::endl;
					std::cout << "distortMatrix:  " << distortMatrix << std::endl;
					
				}
			}
		} else
			continue;
	}
}
void main2()
{
	string path = "/home/moi/";
	vector<String> fn;
	glob(path, fn, false);
	size_t count = fn.size();
	vector<Mat> images;
	cv::Mat cur_photo;
	cout<<" read images ..."<<endl;
	for (int i = 0; i < count; i++)
	{
		images.push_back(imread(fn[i]));
	}
	
	std::vector<cv::Point3f> objs;//一张图片上的世界坐标
	for (int j = 0; j < boardHeight; j++) {
		for (int k = 0; k < boardWidth; k++) {
			objs.emplace_back(k * circleDistance, j * circleDistance, 0.0);
		}
	}
	
	for (int i = 0; i < images.size(); ++i) {
		cv::Mat frame = images[i];
		cur_photo = frame;
		cv::imshow("f", frame);
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		cv::Size boardSize = cv::Size(boardWidth, boardHeight);
		bool foundCircles = false;
		foundCircles = cv::findChessboardCorners(frame, boardSize, circleGridCenters,
		                                         cv::CALIB_CB_SYMMETRIC_GRID);
		
		if (foundCircles) {
			//提取亚像素坐标
			// cv::find4QuadCornerSubpix(frame, circleGridCenters, cv::Size(5, 5));
			cv::cornerSubPix(frame, circleGridCenters, cv::Size(11, 11), cv::Size(-1, -1),
			                 TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			imagePoints.push_back(circleGridCenters);
			objectPoints.push_back(objs);
			cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(frame, boardSize, circleGridCenters, foundCircles);
		}
		cv::imshow("检测到的点", frame);
	}
	if(!imagePoints.empty()) {
		cv::Mat camera_matrix;//相机内参矩阵（最后输出用）
		cv::Mat distort_matrix;//相机畸变矩阵（最后输出用）
		cv::Mat rotation_matrix;//标定板到相机的旋转矩阵（最后输出用）
		cv::Mat translation_matrix;//标定板到相机的平移矩阵(最后输出用)
		std::vector<cv::Mat> cam_r_vec;//每幅图像到相机的旋转矩阵
		std::vector<cv::Mat> cam_t_vec;//每幅图像到相机的平移矩阵
		//求内参
		int flags = 0;
		flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
		flags |= cv::fisheye::CALIB_CHECK_COND;
		flags |= cv::fisheye::CALIB_FIX_SKEW;/*非常重要*/
		cv::fisheye::calibrate(objectPoints, imagePoints, cur_photo.size(),
		                       camera_matrix,
		                       distort_matrix,
		                       cam_r_vec, cam_t_vec,
		                       flags,
		                       cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS),
		                                        200, DBL_EPSILON));
		
		std::cout << "calibrateCamera已通过" << std::endl;
		std::cout << "cameraMatrix:  " << camera_matrix << std::endl;
		std::cout << "distortMatrix:  " << distort_matrix << std::endl;
	}
}
