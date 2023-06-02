#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
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
		cv::Size boardSize = cv::Size(boardWidth, boardHeight);
		bool foundCircles = false;
		foundCircles = cv::findChessboardCorners(frame, boardSize, circleGridCenters,
		                                   cv::CALIB_CB_SYMMETRIC_GRID);
		cv::find4QuadCornerSubpix(frame, circleGridCenters, cv::Size(5, 5));
		// std::cout << "foundCircles:  " << foundCircles << std::endl;
		if (foundCircles) {
			//提取亚像素坐标
			
			// cv::cornerSubPix(grayImg, circleGridCenters, cv::Size(11, 11), cv::Size(-1, -1),
			//                  TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
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
			if (waitKey(30) == 83) //Esc键退出，ESC的ASCLL码为27
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
					                                        200, DBL_EPSILON));
					
					std::cout << "calibrateCamera已通过" << std::endl;
					std::cout << "cameraMatrix:  " << cameraMatrix << std::endl;
					std::cout << "distortMatrix:  " << distortMatrix << std::endl;
					
				}
			}
		} else
			continue;
	}
}
   //  for (int i = 0; i < image_count  ; ++i) {
   //      string path = "../" + to_string(i) + ".jpg";
   //
   //      colorImg = cv::imread(path, 1);
   //      threshold(colorImg, colorImg, 50, 255, THRESH_BINARY_INV);
   //      cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
   //      bool foundCircles = false;
   //      cv::Size boardSize = cv::Size(boardWidth, boardHeight);
   //      foundCircles = cv::findCirclesGrid(grayImg, boardSize, circleGridCenters,
   //                                         cv::CALIB_CB_SYMMETRIC_GRID);
   //      std::cout << "foundCircles:  " << foundCircles << std::endl;
	//
	//
   //      if (foundCircles) {
   //          //提取亚像素坐标
   //
   //          // cv::cornerSubPix(grayImg, circleGridCenters, cv::Size(11, 11), cv::Size(-1, -1),
   //          //                  TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
   //          cv::drawChessboardCorners(colorImg, boardSize, circleGridCenters, foundCircles);
   //      }
   //      cv::imshow("检测到的点", colorImg);
   //      cv::waitKey(0);
   //      std::vector<cv::Point3f> objs;//一张图片上的世界坐标
   //
   //      for (int j = 0; j < boardHeight; j++)
   //      {
   //          for (int k = 0; k < boardWidth; k++)
   //          {
   //              objs.emplace_back(k * circleDistance, j * circleDistance, 0.0);
   //          }
   //      }
   //      if (foundCircles) {
   //          imagePoints.push_back(circleGridCenters);
   //          objectPoints.push_back(objs);
   //      }
   //  }
   //  cv::Mat cameraMatrix;//相机内参矩阵（最后输出用）
   //  cv::Mat distortMatrix;//相机畸变矩阵（最后输出用）
   //  cv::Mat rotationMatrix;//标定板到相机的旋转矩阵（最后输出用）
   //  cv::Mat translationMatrix;//标定板到相机的平移矩阵(最后输出用)
   //
   // std::vector <cv::Mat> camRVec;//每幅图像到相机的旋转矩阵
   //  std::vector <cv::Mat> camTVec;//每幅图像到相机的平移矩阵
   //  //求内参
	// int flags = 0;
	// cv::fisheye::calibrate(objectPoints,imagePoints,colorImg.size(),
	// 					   cameraMatrix,
	// 					   distortMatrix,
	// 					   camRVec,camTVec,
	// 					   flags,
	// 					   cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS),
	// 							                        100, DBL_EPSILON));
	//
   //  std::cout << "calibrateCamera已通过" << std::endl;
   //  std::cout <<"cameraMatrix:  " <<cameraMatrix<<std::endl;
   //  std::cout <<"distortMatrix:  " <<distortMatrix<<std::endl;
	//
	//
	// cv::Mat map1, map2;
	// cv::Mat newCamMat;
	// cv::Size img_size(640,480);
	// cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
	// 		cameraMatrix, distortMatrix, img_size,
	// 		cv::Matx33d::eye(), newCamMat, 1);
	// cv::fisheye::initUndistortRectifyMap(cameraMatrix, distortMatrix,
	//                                      cv::Matx33d::eye(), newCamMat, img_size,
	//                                      CV_16SC2, map1, map2);
	//
	// std::cout <<newCamMat <<std::endl;
	// for (int i = 0; i < image_count  ; ++i) {
	// 	string path = "../" + to_string(i) + ".jpg";
	//
	// 	colorImg = cv::imread(path, 1);
	// 	// threshold(colorImg, colorImg, 50, 255, THRESH_BINARY_INV);
	// 	// cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);
	// 	cv::Mat out_put;
   //
	// 	cv::remap(colorImg, out_put, map1, map2, cv::INTER_LINEAR);
	// 	// cv::fisheye::undistortImage(grayImg, out_put, cameraMatrix, distortMatrix,newCamMat);
	//
	// 	cv::imshow("畸变恢复", out_put);
	// 	cv::waitKey(0);
	// }
   //  return 0;
// }
