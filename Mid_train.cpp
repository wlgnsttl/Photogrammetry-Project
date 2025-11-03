#include "Mid_train.h"
#include "3d_image_sample.h"

cv::Mat transNegative(cv::Mat& img)
{
	cv::Mat res_img = img.clone();

	int rows = img.rows;	// 행
	int cols = img.cols;	// 열

	for (int y = 0; y < img.rows; y++) {
		for (int x = 0; x < img.cols; x++) {
			res_img.at<uchar>(y, x) = 255 - img.at<uchar>(y,x);
		}
	}

	return res_img;
}

cv::Mat transLog(cv::Mat& img)
{
	float c = 255.0 / log(1 + 255);

	cv::Mat res_img = img.clone();

	int rows = img.rows;	// 행
	int cols = img.cols;	// 열

	for (int y = 0; y < img.rows; y++) {
		for (int x = 0; x < img.cols; x++) {
			uchar r = img.at<uchar>(y, x);
			res_img.at<uchar>(y, x) = static_cast<uchar> (c * log(1.0 + r));
		}
	}

	return res_img;
}

cv::Mat transInvLog(cv::Mat& img)
{
	float c = 255.0 / log(1 + 255);

	cv::Mat res_img = img.clone();

	int rows = img.rows;	// 행
	int cols = img.cols;	// 열

	for (int y = 0; y < img.rows; y++) {
		for (int x = 0; x < img.cols; x++) {

			uchar r = static_cast<uchar>(c * (exp(img.at<uchar>(y,x) / c) - 1.0f)); // B

			res_img.at<uchar>(y, x) = r;
		}
	}

	return res_img;
}

cv::Mat transRoot(cv::Mat& img, float n)
{
	cv::Mat res_img = img.clone();

	int rows = img.rows;	// 행
	int cols = img.cols;	// 열

	for (int y = 0; y < img.rows; y++) {
		for (int x = 0; x < img.cols; x++) {
			uchar r = img.at<uchar>(y, x);
			uchar s = static_cast<uchar>(pow(r, 1.0f / n));

			res_img.at<uchar>(y, x) = s;
		}
	}

	return res_img;
}

cv::Mat transPower(cv::Mat& img, float n)
{
	cv::Mat res_img = img.clone();

	int rows = img.rows;	// 행
	int cols = img.cols;	// 열

	for (int y = 0; y < img.rows; y++) {
		for (int x = 0; x < img.cols; x++) {
			uchar r = img.at<uchar>(y, x);
			uchar s = static_cast<uchar>(pow(r, n));

			res_img.at<uchar>(y, x) = s;
		}
	}

	return res_img;
}

cv::Mat applyGaussianBlur(cv::Mat& img)
{
	cv::Mat GaussianKernel, ret_img;
	GaussianKernel = cv::getGaussianKernel(3, 0);
	cv::Mat GaussianKernel_2d = GaussianKernel * GaussianKernel.t();

	cv::filter2D(img, ret_img, -1, GaussianKernel_2d);

	return ret_img;
}

cv::Mat applyLaplacian(cv::Mat& img)
{
	cv::Mat_<char> LaplacianKernel(3, 3);
	LaplacianKernel(0, 0) = 1;	LaplacianKernel(0, 1) = 1;	LaplacianKernel(0, 2) = 1;
	LaplacianKernel(1, 0) = 1;	LaplacianKernel(1, 1) = -8;	LaplacianKernel(1, 2) = 1;
	LaplacianKernel(2, 0) = 1;	LaplacianKernel(2, 1) = 1;	LaplacianKernel(2, 2) = 1;

	cv::Mat ret_img;
	cv::filter2D(img, ret_img, -1, LaplacianKernel);

	//cv::Mat res_img2 = img - res_img;

	return ret_img;
}

cv::Mat applyAffine(cv::Mat& img)
{
	// Afiine
	cv::Mat_<double> AffineMatrix = cv::Mat::zeros(2, 3, CV_64FC1);
	AffineMatrix(0, 0) = 1.;	AffineMatrix(0, 1) = 0.;	AffineMatrix(0, 2) = 50.;
	AffineMatrix(1, 0) = 0.;	AffineMatrix(1, 1) = 1.;	AffineMatrix(1, 2) = 100.;

	cv::Mat ret_img;
	cv::warpAffine(img, ret_img, AffineMatrix, cv::Size());

	return ret_img; 
}

cv::Mat applyHomography(cv::Mat& img)
{
	// Homography
	cv::Mat_<double> Homography = cv::Mat::zeros(3, 3, CV_64FC1);
	Homography(0, 0) = 1.;	Homography(0, 1) = 0.;	Homography(0, 2) = 0.;
	Homography(1, 0) = 0.;	Homography(1, 1) = 1.;	Homography(1, 2) = 0.;
	Homography(2, 0) = 0.001;	Homography(2, 1) = 0.001;	Homography(2, 2) = 1.;

	cv::Mat ret_img; 
	cv::warpPerspective(img, ret_img, Homography, cv::Size());

	return ret_img; 
}

void mid_1(void) 
{
	cv::Mat inputImage = cv::imread("resource/100_0007_0001.JPG", cv::IMREAD_UNCHANGED);
	int RowSize = inputImage.rows;
	int ColSize = inputImage.cols;
	int OutRowSize = RowSize * 1/3;
	int OutColSize = ColSize * 1/3;

	cv::Mat ReducedImage;
	cv::resize(inputImage, ReducedImage, cv::Size(OutColSize, OutRowSize));

	//std::cout << "row size : " << RowSize << std::endl;
	//std::cout << "col size : " << ColSize << std::endl << std::endl;

	//std::cout << "row size : " << OutRowSize << std::endl;
	//std::cout << "col size : " << OutColSize << std::endl << std::endl;

	//----------------------------------------------------------------------------------

	cv::Point2d pivotPoint(OutColSize/2., OutRowSize/2.);

	cv::Size imageSize(OutColSize, OutRowSize);

	double angle = -25;
	cv::Mat_<double> RotationMatrix = GetRoatationMatrix(angle, pivotPoint);
	cv::Mat_<double> RotationMatrix_Inv = RotationMatrix.inv();

	cv::Mat_<float> MapX, MapY;

	BuildRemapMatrix(imageSize, RotationMatrix_Inv, MapX, MapY);

	cv::Mat RotatedImage;
	cv::remap(ReducedImage, RotatedImage, MapX, MapY, cv::InterpolationFlags::INTER_LINEAR);

	cv::Mat_<double> c1(3, 1, CV_64FC1);
	c1(0, 0) = 0;   c1(1, 0) = 0;   c1(2, 0) = 1.;	//동차 좌표 입력 0,0
	cv::Mat_<double> c1p = RotationMatrix * c1;

	cv::Mat_<double> c2(3, 1, CV_64FC1);
	c2(0, 0) = OutColSize;   c2(1, 0) = 0;   c2(2, 0) = 1.;	//동차 좌표 입력 
	cv::Mat_<double> c2p = RotationMatrix * c2;

	cv::Mat_<double> c3(3, 1, CV_64FC1);
	c3(0, 0) = OutColSize;   c3(1, 0) = OutRowSize;   c3(2, 0) = 1.;	//동차 좌표 입력
	cv::Mat_<double> c3p = RotationMatrix * c3;

	cv::Mat_<double> c4(3, 1, CV_64FC1);
	c4(0, 0) = 0;   c4(1, 0) = OutRowSize;   c4(2, 0) = 1.;	//동차 좌표 입력
	cv::Mat_<double> c4p = RotationMatrix * c4;

	//std::cout << "p1 row idx : " << c1p(1,0) << std::endl;
	//std::cout << "p1 col idx : " << c1p(0, 0) << std::endl << std::endl;

	//std::cout << "p2 row idx : " << c2p(1, 0) << std::endl;
	//std::cout << "p2 col idx : " << c2p(0, 0) << std::endl << std::endl;

	//std::cout << "p3 row idx : " << c3p(1, 0) << std::endl;
	//std::cout << "p3 col idx : " << c3p(0, 0) << std::endl << std::endl;

	//std::cout << "p4 row idx : " << c4p(1, 0) << std::endl;
	//std::cout << "p4 col idx : " << c4p(0, 0) << std::endl << std::endl;

	//--------------------------------------------------------------------------------------

	cv::Mat expanedImage = cv::Mat::zeros(1873, 2168, inputImage.type());

	cv::Mat_<double> TranslationMatrix_inv = GetTranslationMatrix(171.5, 328.463).inv();

	cv::Mat_<float> MapX2, MapY2;
	cv::Size outSize2(2168, 1873);
	BuildRemapMatrix(outSize2, TranslationMatrix_inv, MapX2, MapY2);
	cv::remap(ReducedImage, expanedImage, MapX2, MapY2, cv::InterpolationFlags::INTER_LINEAR);




	// -----------------------------------------------------------------------
	//cv::Point2d pivotPoint(OutColSize / 2., OutRowSize / 2.);

	//cv::Size imageSize(OutColSize, OutRowSize);

	//double angle = -25;
	//cv::Mat_<double> RotationMatrix = GetRoatationMatrix(angle, pivotPoint);
	//cv::Mat_<double> RotationMatrix_Inv = RotationMatrix.inv();

	//cv::Mat_<float> MapX, MapY;

	//BuildRemapMatrix(imageSize, RotationMatrix_Inv, MapX, MapY);

	//cv::Mat RotatedImage;
	//cv::remap(ReducedImage, RotatedImage, MapX, MapY, cv::InterpolationFlags::INTER_LINEAR);


}




void main_3d(void) 
{
	//cv::Mat img = cv::imread("resource/lena.jpg", cv::IMREAD_UNCHANGED);
	//cv::Mat res_img = applyHomography(img)

	


	Practice_7_3();

}