#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include <string>


using namespace std;
using namespace cv;


cv::Mat colorImg,grayImg;//用于存取原图，以及存取转化后的灰度图
int boardWidth=5;//标定板上的行数
int boardHeight=4;//标定板上的列数
std::vector < cv::Point2f > circleGridCenters;//用于记录检测到的圆的中心点（一张图片）
std::vector < cv::Point2f > circleGridCenter1;//记录第一张图上中心点的相机坐标
std::vector < cv::Point3f > obj1;//记录第一张图片上中心点的世界坐标
float circleDistance=0.05;//圆点之间的距离????????????不准确

std::vector < std::vector <cv::Point3f> > objectPoints;//圆点中心的世界坐标,每张图片上的点构成内层vector，所有图片构成外层vector
std::vector < std::vector <cv::Point2f> > imagePoints;//圆点中心的图像坐标,,每张图片上的点构成内层vector，所有图片构成外层vector
//测试图片testcircle2.jpg width=rows=1080,height=cols=2277
//std::string imgFile="/home/yaoyishen/stereo-calibration/circle-calibration/testcircle2.jpg";//测试路径
int main()
{

    for (int i = 0; i < 13  ; ++i) {
        string path = "../" + to_string(i) + ".png";

        colorImg = cv::imread(path, 1);
        threshold(colorImg, colorImg, 40, 255, THRESH_BINARY_INV);
        cv::cvtColor(colorImg, grayImg, cv::COLOR_BGR2GRAY);

        bool foundCircles = false;
        cv::Size boardSize = cv::Size(boardWidth, boardHeight);
        foundCircles = cv::findCirclesGrid(grayImg, boardSize, circleGridCenters,
                                           cv::CALIB_CB_SYMMETRIC_GRID);
        std::cout << "foundCircles:  " << foundCircles << std::endl;
        if (foundCircles) {
            //提取亚像素坐标
            cv::cornerSubPix(grayImg, circleGridCenters, cv::Size(5, 5), cv::Size(-1, -1),
                             TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
            //标记检测到的点
            //cv::drawChessboardCorners(grayImg,boardSize,circleGridCenters,foundCircles);
            cv::drawChessboardCorners(colorImg, boardSize, circleGridCenters, foundCircles);
        }
        cv::imshow("检测到的点", colorImg);
        cv::waitKey(0);
        //
        std::vector<cv::Point3f> objs;//一张图片上的世界坐标

        for (int j = 0; j < boardHeight; j++)
        {
            for (int k = 0; k < boardWidth; k++)
            {
                objs.push_back(Point3f(k * circleDistance, j * circleDistance, 0));
            }
        }

        if (foundCircles) {
            imagePoints.push_back(circleGridCenters);
            objectPoints.push_back(objs);
        }
        //std::cout << "二维图像坐标" << imagePoints[0] << std::endl;
        //std::cout << "三维世界坐标" << objectPoints[0] << std::endl;
    }
    cv::Mat cameraMatrix;//相机内参矩阵（最后输出用）
    cv::Mat distortMatrix;//相机畸变矩阵（最后输出用）
    cv::Mat rotationMatrix;//标定板到相机的旋转矩阵（最后输出用）
    cv::Mat translationMatrix;//标定板到相机的平移矩阵(最后输出用)

   std::vector <cv::Mat> camRVec;//每幅图像到相机的旋转矩阵
    std::vector <cv::Mat> camTVec;//每幅图像到相机的平移矩阵
    //求内参
    cv::calibrateCamera(objectPoints,imagePoints,colorImg.size(),cameraMatrix,distortMatrix,camRVec,camTVec,0,cv::TermCriteria((cv::TermCriteria::COUNT) + (cv::TermCriteria::EPS),
                         30, DBL_EPSILON));//相机内参
    std::cout << "calibrateCamera已通过" << std::endl;
    std::cout <<"cameraMatrix:  " <<cameraMatrix<<std::endl;
    std::cout <<"distortMatrix:  " <<distortMatrix<<std::endl;
    return 0;
}
