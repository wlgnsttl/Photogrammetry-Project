#ifndef _3D_IMAGE_SAMPLE_H_
#define _3D_IMAGE_SAMPLE_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <opencv2/highgui.hpp>

void Practice_2_1();
void Practice_2_2();
void Practice_2_3();

void Practice_3_1();
void Practice_3_2();
void Practice_3_3();

cv::Mat_<uchar> Practice_4_1();
cv::Mat_<uchar> Practice_4_2();

void Practice_7_1();
void Practice_7_2();
void Practice_7_3();

cv::Mat_<double> GetTranslationMatrix(double tx, double ty);
cv::Mat_<double> GetRoatationMatrix(double angle, cv::Point2d pivotPoint);

void ComputeOutImageSize(cv::Size inputSize, cv::Mat_<double> src2dstMat, cv::Size& outputSize, cv::Point2d& offset);
void BuildRemapMatrix(cv::Size& OutSize, cv::Mat_<double>& dst2srcMat, cv::Mat_<float>& MapX, cv::Mat_<float>& MapY);

#endif
