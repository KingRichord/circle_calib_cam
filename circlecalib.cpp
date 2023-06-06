#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace cv;

int board_width = 11;//标定板上的行数
int board_height = 8;//标定板上的列数
std::vector < cv::Point2d > circle_grid_centers;//用于记录检测到的圆的中心点（一张图片）
double circle_distance = 0.003f;

std::vector < std::vector <cv::Point3d> > object_points;//圆点中心的世界坐标,每张图片上的点构成内层vector，所有图片构成外层vector
std::vector < std::vector <cv::Point2d> > image_points;//圆点中心的图像坐标,,每张图片上的点构成内层vector，所有图片构成外层vector
int main()
{
	
	VideoCapture cap;
		cap.open(0); //打开摄像头
	if(!cap.isOpened())
          return -1;
	int count =0;
	Mat frame;
	while (true) {
		cap >> frame;//等价于cap.read(frame);
		// std::cout <<frame.size <<std::endl;
		cv::imshow("f", frame);
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		cv::Size board_size = cv::Size(board_width, board_height);
		bool found_circles = false;
		found_circles = cv::findChessboardCorners(frame, board_size, circle_grid_centers,
		                                   cv::CALIB_CB_SYMMETRIC_GRID);
		
		// std::cout << "foundCircles:  " << foundCircles << std::endl;
		if (found_circles) {
			cv::cornerSubPix(frame, circle_grid_centers, cv::Size(11, 11), cv::Size(-1, -1),
			                 TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
			cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(frame, board_size, circle_grid_centers, found_circles);
		}
		cv::imshow("检测到的点", frame);
		std::vector<cv::Point3d> objs;//一张图片上的世界坐标
		for (int j = 0; j < board_height; j++) {
			for (int k = 0; k < board_width; k++) {
				objs.emplace_back(k * circle_distance, j * circle_distance, 0.0);
			}
		}
		if (waitKey(50) == 27) //Esc键退出，ESC的ASCLL码为27
		{
			if (found_circles) {
				std::cout << "count  "<<count++ <<std::endl;
				image_points.push_back(circle_grid_centers);
				object_points.push_back(objs);
				if (image_points.size() > 20) {
					cv::Mat camera_matrix;//相机内参矩阵（最后输出用）
					cv::Mat distort_matrix;//相机畸变矩阵（最后输出用）
					cv::Mat rotation_matrix;//标定板到相机的旋转矩阵（最后输出用）
					cv::Mat translation_matrix;//标定板到相机的平移矩阵(最后输出用)
					
					std::vector<cv::Mat> cam_r_vec;//每幅图像到相机的旋转矩阵
					std::vector<cv::Mat> cam_t_vec;//每幅图像到相机的平移矩阵
					//求内参
					/*
					    CALIB_USE_INTRINSIC_GUESS cameraMatrix 包含进一步优化的 fx、fy、cx、cy 的有效初始值。 否则，(cx, cy) 最初设置为图像中心（使用 imageSize），并以最小二乘法计算焦距。 请注意，如果内部参数已知，则无需仅使用此函数来估计外部参数。 请 改用solvePnP 。
					    CALIB_FIX_PRINCIPAL_POINT 全局优化过程中主点不变。 当CALIB_USE_INTRINSIC_GUESS 也被设置时，它停留在中心或指定的不同位置 。
					    CALIB_FIX_ASPECT_RATIO 函数仅将 fy 视为自由参数。 fx/fy 比率与输入 cameraMatrix 中的比率相同。 当 CALIB_USE_INTRINSIC_GUESS 未设置时，fx 和 fy 的实际输入值将被忽略，仅计算并进一步使用它们的比率。
					    CALIB_ZERO_TANGENT_DIST 切向畸变系数 ( p1 , p2 ) _ _ 被设置为零并保持零。
					    CALIB_FIX_FOCAL_LENGTH 如果设置了 CALIB_USE_INTRINSIC_GUESS ，则焦距在全局优化期间不会改变。
					    CALIB_FIX_K1 ,..., CALIB_FIX_K6 对应的径向畸变系数在优化过程中没有改变。 如果 设置了CALIB_USE_INTRINSIC_GUESS ，则使用提供的 distCoeffs 矩阵中的系数。 否则，它被设置为 0。
					    CALIB_RATIONAL_MODEL 系数 k4、k5 和 k6 已启用。 为了提供向后兼容性，应明确指定此额外标志以使校准函数使用有理模型并返回 8 个或更多系数。
					    CALIB_THIN_PRISM_MODEL 系数 s1、s2、s3 和 s4 已启用。 为了提供向后兼容性，应明确指定此额外标志以使校准函数使用薄棱镜模型并返回 12 个或更多系数。
					    CALIB_FIX_S1_S2_S3_S4 薄棱镜畸变系数在优化过程中没有改变。 如果 设置了CALIB_USE_INTRINSIC_GUESS ，则使用提供的 distCoeffs 矩阵中的系数。 否则，它被设置为 0。
					    CALIB_TILTED_MODEL 系数 tauX 和 tauY 已启用。 为了提供向后兼容性，应明确指定此额外标志以使校准函数使用倾斜传感器模型并返回 14 个系数。
					    CALIB_FIX_TAUX_TAUY 倾斜传感器模型的系数在优化过程中没有改变。 如果 设置了CALIB_USE_INTRINSIC_GUESS ，则使用提供的 distCoeffs 矩阵中的系数。 否则，它被设置为 0。
					 * */
					int flags = 0;
					cv::calibrateCamera(object_points, image_points, frame.size(),
					                    camera_matrix,
					                    distort_matrix,
					                    cam_r_vec, cam_t_vec,
					                    flags,
					                    cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS),
					                                     200, DBL_EPSILON));
					/*
					    // fisheye：：CALIB_USE_INTRINSIC_GUESS cameraMatrix包含fx，fy，cx，cy的有效初始值，这些值将进一步优化。否则，（cx， cy） 最初设置为图像中心（使用 imageSize），并以最小二乘方式计算焦距。
					    // fisheye：：CALIB_RECOMPUTE_EXTRINSIC 在每次内部优化迭代后，将重新计算外在优化。
					    // fisheye：：CALIB_CHECK_COND 函数将检查条件编号的有效性。
					    // fisheye：：CALIB_FIX_SKEW 偏斜系数 （alpha） 设置为零并保持零。
					    // fisheye：：CALIB_FIX_K1 ,..., 鱼眼：：CALIB_FIX_K4 选定的失真系数设置为零并保持零。
					    // fisheye：：CALIB_FIX_PRINCIPAL_POINT 在全局优化期间，主点不会改变。它也停留在中心或 设置鱼眼：：CALIB_USE_INTRINSIC_GUESS 时指定的其他位置。
						int flags = 0;
						flags |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
						flags |= cv::fisheye::CALIB_CHECK_COND;
						flags |= cv::fisheye::CALIB_FIX_SKEW; //非常重要
						cv::fisheye::calibrate(objectPoints, imagePoints, frame.size(),
						                       cameraMatrix,
						                       distortMatrix,
						                       camRVec, camTVec,
						                       flags,
						                       cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS),
						                                        200, DBL_EPSILON));
					*/
					std::cout << "calibrateCamera已通过" << std::endl;
					std::cout << "cameraMatrix:  " << std::endl<< camera_matrix << std::endl;
					std::cout << "distortMatrix:  " << std::endl<< distort_matrix << std::endl;
					
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
	
	std::vector<cv::Point3d> objs;//一张图片上的世界坐标
	for (int j = 0; j < board_height; j++) {
		for (int k = 0; k < board_width; k++) {
			objs.emplace_back(k * circle_distance, j * circle_distance, 0.0);
		}
	}
	
	for (int i = 0; i < images.size(); ++i) {
		cv::Mat frame = images[i];
		cur_photo = frame;
		cv::imshow("f", frame);
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
		cv::Size boardSize = cv::Size(board_width, board_height);
		bool foundCircles = false;
		foundCircles = cv::findChessboardCorners(frame, boardSize, circle_grid_centers,
		                                         cv::CALIB_CB_SYMMETRIC_GRID);
		
		if (foundCircles) {
			//提取亚像素坐标
			// cv::find4QuadCornerSubpix(frame, circleGridCenters, cv::Size(5, 5));
			cv::cornerSubPix(frame, circle_grid_centers, cv::Size(11, 11), cv::Size(-1, -1),
			                 TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001));
			image_points.push_back(circle_grid_centers);
			object_points.push_back(objs);
			cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
			cv::drawChessboardCorners(frame, boardSize, circle_grid_centers, foundCircles);
		}
		cv::imshow("检测到的点", frame);
	}
	if(!image_points.empty()) {
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
		cv::fisheye::calibrate(object_points, image_points, cur_photo.size(),
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
