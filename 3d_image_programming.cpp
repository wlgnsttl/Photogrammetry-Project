#include "3d_image_programming.h"

void week2_1()
{
    cv::Mat ex1_img = cv::imread("resource/R0047607.JPG", cv::IMREAD_COLOR);

    cv::Mat ex1_img_ch1;
    cv::Mat ex1_img_ch2;
    cv::Mat ex1_img_ch3;

    std::vector<cv::Mat> channels;
    cv::split(ex1_img, channels);

    ex1_img_ch1 = channels[0]; // Blue
    ex1_img_ch2 = channels[1]; // Green
    ex1_img_ch3 = channels[2]; // Red

    int rows = ex1_img_ch2.rows;
    int cols = ex1_img_ch2.cols;

    for (int y = 0; y < rows; y++) {
        uchar* row_ptr = ex1_img_ch2.ptr<uchar>(y);
        for (int x = 0; x < cols; x++) {

			row_ptr[x] = 255 - row_ptr[x];
            //row_ptr[x] = (row_ptr[x] > 128) ? 255 : 0;
        }
    }

    cv::namedWindow("Green", cv::WINDOW_NORMAL);
    cv::imshow("Green", ex1_img_ch2);
    cv::waitKey(0);
}

void week2_2()
{
    cv::Mat ex1_img = cv::imread("resource/R0047607.JPG", cv::IMREAD_COLOR);
    if (ex1_img.empty()) return;

    std::vector<cv::Mat> channels;
    cv::split(ex1_img, channels);

    // Blue 채널만 그대로 두고, Green과 Red는 0으로 만든다
    channels[1] = cv::Mat::zeros(ex1_img.size(), CV_8UC1); // Green -> 0
    channels[2] = cv::Mat::zeros(ex1_img.size(), CV_8UC1); // Red   -> 0

    cv::Mat blue_only;
    cv::merge(channels, blue_only);

    cv::namedWindow("Blue Only", cv::WINDOW_NORMAL);
    cv::imshow("Blue Only", blue_only);
    cv::waitKey(0);
}

void week2_3()
{
    cv::Mat img = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);
    if (img.empty()) return;

    // 1차원 Gaussian Kernel (5x1)
    cv::Mat kernel = cv::getGaussianKernel(3, 0, CV_64F);

    // 2차원 Gaussian Kernel = kernel * kernel^T
    cv::Mat gaussian_2d = kernel * kernel.t();

    // 가우시안 블러 적용
    cv::Mat gaussian_blur;
    cv::filter2D(img, gaussian_blur, -1, gaussian_2d);

    // 라플라시안 필터 적용
    cv::Mat laplacian;
    cv::Laplacian(gaussian_blur, laplacian, CV_16S, 3);  // 3x3 커널
    cv::convertScaleAbs(laplacian, laplacian);           // 부호 있는 결과를 절댓값 + 8비트 변환

}
