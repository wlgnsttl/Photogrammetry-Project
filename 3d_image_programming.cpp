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

    // Blue ä�θ� �״�� �ΰ�, Green�� Red�� 0���� �����
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

    // 1���� Gaussian Kernel (5x1)
    cv::Mat kernel = cv::getGaussianKernel(3, 0, CV_64F);

    // 2���� Gaussian Kernel = kernel * kernel^T
    cv::Mat gaussian_2d = kernel * kernel.t();

    // ����þ� �� ����
    cv::Mat gaussian_blur;
    cv::filter2D(img, gaussian_blur, -1, gaussian_2d);

    // ���ö�þ� ���� ����
    cv::Mat laplacian;
    cv::Laplacian(gaussian_blur, laplacian, CV_16S, 3);  // 3x3 Ŀ��
    cv::convertScaleAbs(laplacian, laplacian);           // ��ȣ �ִ� ����� ���� + 8��Ʈ ��ȯ

}

void week4_1()
{
    // �Է� �̹��� (���)
    cv::Mat img = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "�̹����� ���� �� �����ϴ�.\n";
        return;
    }

    int rows = img.rows;
    int cols = img.cols;

    // ��ȯ ��� (������ + �̵�)
    cv::Mat_<double> transMat = cv::Mat::zeros(3, 3, CV_64FC1);
    transMat(0, 0) = 3; transMat(0, 1) = 0; transMat(0, 2) = 0;
    transMat(1, 0) = 0; transMat(1, 1) = 3; transMat(1, 2) = 0;
    transMat(2, 0) = 3; transMat(2, 1) = 0; transMat(2, 2) = 1;

    // �����
    cv::Mat_<double> invMat = transMat.inv();

    // ��� ���� (3�� Ȯ��)
    int outRows = rows * 3;
    int outCols = cols * 3;
    cv::Mat outImg = cv::Mat::zeros(outRows, outCols, CV_8UC1);

    // ����ȯ ����
    for (int r = 0; r < outRows; r++) {
        for (int c = 0; c < outCols; c++) {
            // ��� ��ǥ (������ǥ��)
            cv::Mat_<double> outPoint(3, 1);
            outPoint(0, 0) = c;
            outPoint(1, 0) = r;
            outPoint(2, 0) = 1;

            // �Է� ��ǥ ���
            cv::Mat_<double> inPoint = invMat * outPoint;
            double inX = inPoint(0, 0);
            double inY = inPoint(1, 0);

            int x = static_cast<int>(inX + 0.5);
            int y = static_cast<int>(inY + 0.5);

            // --- ���� �˻� (���� ����) ---
            if (x < 0 || x >= cols || y < 0 || y >= rows) {
                // ������ �α� (�ʿ� ������ �ּ� ó��)
                // std::cerr << "Skip out of range: "
                //           << "x=" << x << " y=" << y
                //           << " (cols=" << cols << ", rows=" << rows << ")\n";
                continue;
            }

            // ������ ����
            outImg.at<uchar>(r, c) = img.at<uchar>(y, x);
        }
    }

    // ��� ���
    cv::imshow("Input", img);
    cv::imshow("Output", outImg);
    cv::waitKey(0);
}

void week4_2()
{
    // �Է� �̹��� (���)
    cv::Mat img = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "�̹����� ���� �� �����ϴ�.\n";
        return;
    }

    int rows = img.rows;
    int cols = img.cols;

    // ��ȯ ��� (������ + �̵�)
    cv::Mat_<double> transMat = cv::Mat::zeros(3, 3, CV_64FC1);
    transMat(0, 0) = 3; transMat(0, 1) = 0; transMat(0, 2) = 0;
    transMat(1, 0) = 0; transMat(1, 1) = 3; transMat(1, 2) = 0;
    transMat(2, 0) = 3; transMat(2, 1) = 0; transMat(2, 2) = 1;

    // �����
    cv::Mat_<double> invMat = transMat.inv();

    // ��� ���� (3�� Ȯ��)
    int outRows = rows * 3;
    int outCols = cols * 3;
    cv::Mat outImg = cv::Mat::zeros(outRows, outCols, CV_8UC1);

    // ����ȯ ���� + ���̸��Ͼ� ����
    for (int r = 0; r < outRows; r++) {
        for (int c = 0; c < outCols; c++) {
            // ��� ��ǥ (������ǥ��)
            cv::Mat_<double> outPoint(3, 1);
            outPoint(0, 0) = c;
            outPoint(1, 0) = r;
            outPoint(2, 0) = 1;

            // �Է� ��ǥ ���
            cv::Mat_<double> inPoint = invMat * outPoint;
            double inX = inPoint(0, 0);
            double inY = inPoint(1, 0);

            int i = static_cast<int>(floor(inX));
            int j = static_cast<int>(floor(inY));

            // ���� �˻� (������ i+1, j+1 �ʿ�)
            if (i < 0 || i + 1 >= cols || j < 0 || j + 1 >= rows) {
                continue;
            }

            double alpha = inX - i; // x ���� ����
            double beta = inY - j; // y ���� ����

            // �ֺ� 4�ȼ� ��
            double I00 = img.at<uchar>(j, i);
            double I10 = img.at<uchar>(j, i + 1);
            double I01 = img.at<uchar>(j + 1, i);
            double I11 = img.at<uchar>(j + 1, i + 1);

            // ���̸��Ͼ� ����
            double value =
                (1 - alpha) * (1 - beta) * I00 +
                alpha * (1 - beta) * I10 +
                (1 - alpha) * beta * I01 +
                alpha * beta * I11;

            outImg.at<uchar>(r, c) = cv::saturate_cast<uchar>(value);
        }
    }

    // ��� ���
    cv::imshow("Input", img);
    cv::imshow("Output - Bilinear", outImg);
    cv::waitKey(0);
}