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

void week4_1()
{
    // 입력 이미지 (흑백)
    cv::Mat img = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "이미지를 읽을 수 없습니다.\n";
        return;
    }

    int rows = img.rows;
    int cols = img.cols;

    // 변환 행렬 (스케일 + 이동)
    cv::Mat_<double> transMat = cv::Mat::zeros(3, 3, CV_64FC1);
    transMat(0, 0) = 3; transMat(0, 1) = 0; transMat(0, 2) = 0;
    transMat(1, 0) = 0; transMat(1, 1) = 3; transMat(1, 2) = 0;
    transMat(2, 0) = 3; transMat(2, 1) = 0; transMat(2, 2) = 1;

    // 역행렬
    cv::Mat_<double> invMat = transMat.inv();

    // 출력 영상 (3배 확대)
    int outRows = rows * 3;
    int outCols = cols * 3;
    cv::Mat outImg = cv::Mat::zeros(outRows, outCols, CV_8UC1);

    // 역변환 매핑
    for (int r = 0; r < outRows; r++) {
        for (int c = 0; c < outCols; c++) {
            // 출력 좌표 (동차좌표계)
            cv::Mat_<double> outPoint(3, 1);
            outPoint(0, 0) = c;
            outPoint(1, 0) = r;
            outPoint(2, 0) = 1;

            // 입력 좌표 계산
            cv::Mat_<double> inPoint = invMat * outPoint;
            double inX = inPoint(0, 0);
            double inY = inPoint(1, 0);

            int x = static_cast<int>(inX + 0.5);
            int y = static_cast<int>(inY + 0.5);

            // --- 범위 검사 (예외 방지) ---
            if (x < 0 || x >= cols || y < 0 || y >= rows) {
                // 디버깅용 로그 (필요 없으면 주석 처리)
                // std::cerr << "Skip out of range: "
                //           << "x=" << x << " y=" << y
                //           << " (cols=" << cols << ", rows=" << rows << ")\n";
                continue;
            }

            // 안전한 매핑
            outImg.at<uchar>(r, c) = img.at<uchar>(y, x);
        }
    }

    // 결과 출력
    cv::imshow("Input", img);
    cv::imshow("Output", outImg);
    cv::waitKey(0);
}

void week4_2()
{
    // 입력 이미지 (흑백)
    cv::Mat img = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "이미지를 읽을 수 없습니다.\n";
        return;
    }

    int rows = img.rows;
    int cols = img.cols;

    // 변환 행렬 (스케일 + 이동)
    cv::Mat_<double> transMat = cv::Mat::zeros(3, 3, CV_64FC1);
    transMat(0, 0) = 3; transMat(0, 1) = 0; transMat(0, 2) = 0;
    transMat(1, 0) = 0; transMat(1, 1) = 3; transMat(1, 2) = 0;
    transMat(2, 0) = 3; transMat(2, 1) = 0; transMat(2, 2) = 1;

    // 역행렬
    cv::Mat_<double> invMat = transMat.inv();

    // 출력 영상 (3배 확대)
    int outRows = rows * 3;
    int outCols = cols * 3;
    cv::Mat outImg = cv::Mat::zeros(outRows, outCols, CV_8UC1);

    // 역변환 매핑 + 바이리니어 보간
    for (int r = 0; r < outRows; r++) {
        for (int c = 0; c < outCols; c++) {
            // 출력 좌표 (동차좌표계)
            cv::Mat_<double> outPoint(3, 1);
            outPoint(0, 0) = c;
            outPoint(1, 0) = r;
            outPoint(2, 0) = 1;

            // 입력 좌표 계산
            cv::Mat_<double> inPoint = invMat * outPoint;
            double inX = inPoint(0, 0);
            double inY = inPoint(1, 0);

            int i = static_cast<int>(floor(inX));
            int j = static_cast<int>(floor(inY));

            // 범위 검사 (보간은 i+1, j+1 필요)
            if (i < 0 || i + 1 >= cols || j < 0 || j + 1 >= rows) {
                continue;
            }

            double alpha = inX - i; // x 방향 분율
            double beta = inY - j; // y 방향 분율

            // 주변 4픽셀 값
            double I00 = img.at<uchar>(j, i);
            double I10 = img.at<uchar>(j, i + 1);
            double I01 = img.at<uchar>(j + 1, i);
            double I11 = img.at<uchar>(j + 1, i + 1);

            // 바이리니어 보간
            double value =
                (1 - alpha) * (1 - beta) * I00 +
                alpha * (1 - beta) * I10 +
                (1 - alpha) * beta * I01 +
                alpha * beta * I11;

            outImg.at<uchar>(r, c) = cv::saturate_cast<uchar>(value);
        }
    }

    // 결과 출력
    cv::imshow("Input", img);
    cv::imshow("Output - Bilinear", outImg);
    cv::waitKey(0);
}