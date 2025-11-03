#include "3d_image_sample.h"


void Practice_2_1()
{
	cv::Mat ex1_img = cv::imread("resource/lena.jpg", cv::IMREAD_COLOR);

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

void Practice_2_2()
{
	cv::Mat inputImage = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);

	cv::Mat GaussianKernel;
	GaussianKernel = cv::getGaussianKernel(5, 0);
	cv::Mat GaussianKernel_2d = GaussianKernel * GaussianKernel.t();

	cv::Mat GaussianBlurredImage;
	cv::filter2D(inputImage, GaussianBlurredImage, -1, GaussianKernel_2d);
}

void Practice_2_3()
{
	cv::Mat inputImage = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);

	// Create Gaussian Kernel
	cv::Mat_<float> GaussianKernel(3, 3, CV_32FC1);
	GaussianKernel(0, 0) = 1. / 16;	GaussianKernel(0, 1) = 2. / 16;	GaussianKernel(0, 2) = 1. / 16;
	GaussianKernel(1, 0) = 2. / 16;	GaussianKernel(1, 1) = 4. / 16;	GaussianKernel(1, 2) = 2. / 16;
	GaussianKernel(2, 0) = 1. / 16;	GaussianKernel(2, 1) = 2. / 16;	GaussianKernel(2, 2) = 1. / 16;

	cv::Mat GaussianBlurredImage;
	cv::filter2D(inputImage, GaussianBlurredImage, -1, GaussianKernel);


	cv::Mat_<char> LaplacianKernel(3, 3, CV_8SC1);
	LaplacianKernel(0, 0) = 1;	LaplacianKernel(0, 1) = 1;	LaplacianKernel(0, 2) = 1;
	LaplacianKernel(1, 0) = 1;	LaplacianKernel(1, 1) = -8;	LaplacianKernel(1, 2) = 1;
	LaplacianKernel(2, 0) = 1;	LaplacianKernel(2, 1) = 1;	LaplacianKernel(2, 2) = 1;

	cv::Mat LaplacianImage;
	cv::filter2D(GaussianBlurredImage, LaplacianImage, -1, LaplacianKernel);

	//cv::Mat SharpenedImage;
	//SharpenedImage = inputImage - 0.5 * LaplacianImage;

}

void Practice_3_1()
{
	cv::Mat inputImage = cv::imread("resource/lena.jpg", cv::IMREAD_UNCHANGED);
	int rowSize = inputImage.rows;
	int colSize = inputImage.cols;

	cv::Mat HSVImage;
	cv::cvtColor(inputImage, HSVImage, cv::COLOR_BGR2HSV);	//HSV 로 변환

	// Brightness 감소, V 채널 절반으로 감소
	std::vector<cv::Mat> HSV_vec;
	cv::split(HSVImage, HSV_vec);
	for (int row = 0; row < rowSize; row++) {
		for (int col = 0; col < colSize; col++) {
			HSV_vec[2].at<uchar>(row, col) = HSV_vec[2].at<uchar>(row, col) / 2.;
		}
	}

	cv::Mat HSVMerge, BGRImage;
	cv::merge(HSV_vec, HSVMerge);	//HSV 채널 합침
	cv::cvtColor(HSVMerge, BGRImage, cv::COLOR_HSV2BGR);	//BGR 로 변환



	// Rotation Resize
	cv::Mat RotatedImage;
	cv::rotate(inputImage, RotatedImage, cv::ROTATE_180);	// 180도 회전

	cv::Mat ResizedImage;
	cv::resize(inputImage, ResizedImage, cv::Size(204, 102));	// 가로 204, 세로 102 로 리사이즈
}

void Practice_3_2()
{

	cv::Mat_<uchar> inputImage = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);

	// Afiine
	cv::Mat_<double> AffineMatrix = cv::Mat::zeros(2, 3, CV_64FC1);
	AffineMatrix(0, 0) = 1.;	AffineMatrix(0, 1) = 0.;	AffineMatrix(0, 2) = 50.;
	AffineMatrix(1, 0) = 0.;	AffineMatrix(1, 1) = 1.;	AffineMatrix(1, 2) = 100.;

	cv::Mat_<uchar> AffineApplyImage = cv::Mat::zeros(inputImage.rows, inputImage.cols, CV_8UC1);

	cv::warpAffine(inputImage, AffineApplyImage, AffineMatrix, cv::Size());


	// Homography
	cv::Mat_<double> Homography = cv::Mat::zeros(3, 3, CV_64FC1);
	Homography(0, 0) = 1.;	Homography(0, 1) = 0.;	Homography(0, 2) = 0.;
	Homography(1, 0) = 0.;	Homography(1, 1) = 1.;	Homography(1, 2) = 0.;
	Homography(2, 0) = 0.001;	Homography(2, 1) = 0.001;	Homography(2, 2) = 1.;

	cv::Mat_<uchar> HApplyImage = cv::Mat::zeros(inputImage.rows, inputImage.cols, CV_8UC1);

	cv::warpPerspective(inputImage, HApplyImage, Homography, cv::Size());

}

void Practice_3_3()
{
	cv::Mat_<uchar> inputImage = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);

	int rowSize = inputImage.rows;
	int colSize = inputImage.cols;
	cv::Size imageSize(colSize, rowSize);

	cv::Point2d pivotPoint(colSize / 2., rowSize / 2.);

	double angle = 30.;
	cv::Mat_<double> RotationMatrix = GetRoatationMatrix(angle, pivotPoint);
	cv::Mat_<double> RotationMatrix_Inv = RotationMatrix.inv();

	cv::Mat_<float> MapX, MapY;

	BuildRemapMatrix(imageSize, RotationMatrix_Inv, MapX, MapY);


	// MapXY를 기반으로 result의 특정 픽셀이 어디서왔지?를 계산하고 
	// 이는 거진 실수이므로 선형 보간 방법을 사용하여 매핑한다는 이야기
	cv::Mat ResultImage;
	cv::remap(inputImage, ResultImage, MapX, MapY, cv::InterpolationFlags::INTER_LINEAR);




}

cv::Mat_<uchar> Practice_4_1()
{
	// 리사이즈 후 매핑 하는건 바로 알겠음. 보간 방법은 최근접 이웃보간
	cv::Mat_<uchar> inputImage = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);
	int RowSize = inputImage.rows;
	int ColSize = inputImage.cols;

	cv::Mat_<double> TransMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
	TransMatrix(0, 0) = 3.;	TransMatrix(0, 1) = 0.;	TransMatrix(0, 2) = 0.;
	TransMatrix(1, 0) = 0.;	TransMatrix(1, 1) = 3.;	TransMatrix(1, 2) = 0.;
	TransMatrix(2, 0) = 0.;	TransMatrix(2, 1) = 0.;	TransMatrix(2, 2) = 1.;
	cv::Mat_<double> InverseMatrix = TransMatrix.inv();

	int OutRowSize = RowSize * 3;
	int OutColSize = ColSize * 3;
	cv::Mat_<uchar> ResultImage = cv::Mat::zeros(OutRowSize, OutColSize, CV_8UC1);
	for (int r = 0; r < OutRowSize; r++) {
		for (int c = 0; c < OutColSize; c++) {
			cv::Mat_<double> out_point = cv::Mat(3, 1, CV_64FC1);
			cv::Mat_<double> in_point = cv::Mat(3, 1, CV_64FC1);
			out_point(0, 0) = c;
			out_point(1, 0) = r;
			out_point(2, 0) = 1;

			in_point = InverseMatrix * out_point;
			double in_x = in_point(0, 0);
			double in_y = in_point(1, 0);
			int x = static_cast<int>(in_x + 0.5);
			int y = static_cast<int>(in_y + 0.5);
			if (x < 0 || y < 0 || x >= ColSize || y >= RowSize) {
				ResultImage.at<uchar>(r, c) = 0;
				continue;
			}

			uchar value = inputImage.at<uchar>(y, x);
			ResultImage.at<uchar>(r, c) = value;

		}
	}

	return ResultImage;
}

cv::Mat_<uchar>  Practice_4_2() // Bilinear Interpolation
{
	cv::Mat_<uchar> inputImage = cv::imread("resource/lena.jpg", cv::IMREAD_GRAYSCALE);
	int RowSize = inputImage.rows;
	int ColSize = inputImage.cols;

	cv::Mat_<double> TransMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
	TransMatrix(0, 0) = 3.;	TransMatrix(0, 1) = 0.;	TransMatrix(0, 2) = 0.;
	TransMatrix(1, 0) = 0.;	TransMatrix(1, 1) = 3.;	TransMatrix(1, 2) = 0.;
	TransMatrix(2, 0) = 0.;	TransMatrix(2, 1) = 0.;	TransMatrix(2, 2) = 1.;
	cv::Mat_<double> InverseMatrix = TransMatrix.inv();

	int OutRowSize = RowSize * 3;
	int OutColSize = ColSize * 3;
	cv::Mat_<uchar> ResultImage = cv::Mat::zeros(OutRowSize, OutColSize, CV_8UC1);
	for (int r = 0; r < OutRowSize; r++) {
		for (int c = 0; c < OutColSize; c++) {
			cv::Mat_<double> out_point = cv::Mat(3, 1, CV_64FC1);
			cv::Mat_<double> in_point = cv::Mat(3, 1, CV_64FC1);
			out_point(0, 0) = c;
			out_point(1, 0) = r;
			out_point(2, 0) = 1;

			in_point = InverseMatrix * out_point;
			double in_x = in_point(0, 0);
			double in_y = in_point(1, 0);
			int x = static_cast<int>(in_x);
			int y = static_cast<int>(in_y);

			if (x < 0 || y < 0 || (x+1) >= ColSize || (y+1) >= RowSize) {
				ResultImage.at<uchar>(r, c) = 0;
				continue;
			}
			uchar p1 = inputImage.at<uchar>(y, x);
			uchar p2 = inputImage.at<uchar>(y, x + 1);
			uchar p3 = inputImage.at<uchar>(y + 1, x);
			uchar p4 = inputImage.at<uchar>(y + 1, x + 1);

			double dp1 = in_x - x;
			double dp2 = in_y - y;

			double R12 = p2 * dp1 + p1 * (1 - dp1);
			double R34 = p4 * dp1 + p3 * (1 - dp1);

			double dvalue = R34 * dp2 + R12 * (1 - dp2);
			if (dvalue > 255)
				ResultImage.at<uchar>(r, c) = 255;
			else if (dvalue < 0) 
				ResultImage.at<uchar>(r, c) = 0;
			else {
				uchar value = static_cast<uchar>(dvalue + 0.5);
				ResultImage.at<uchar>(r, c) = value;
			}
		}
	}

	return ResultImage;
}

void Practice_7_1() // 3D Transformation Scale 변환
{
	cv::Mat inputImage = cv::imread("resource/100_0007_0001.JPG", cv::IMREAD_UNCHANGED);
	std::cout << inputImage.cols << "\t" << inputImage.rows << std::endl;

	// 원본 영상보다 2배 작게 리사이즈
	cv::Mat ReducedImage;
	int reduce_factor = 2.;
	cv::resize(inputImage, ReducedImage, cv::Size(inputImage.cols / reduce_factor, inputImage.rows / reduce_factor));
	std::cout << ReducedImage.cols << "\t" << ReducedImage.rows << std::endl;

	cv::Mat_<float> MapX = cv::Mat(ReducedImage.rows, ReducedImage.cols, CV_32FC1);
	cv::Mat_<float> MapY = cv::Mat(ReducedImage.rows, ReducedImage.cols, CV_32FC1);

	float F_Pixel = 8800. / 2.4;				//카메라 초점거리 (픽셀단위)
	float FNumber = F_Pixel / reduce_factor;	//픽셀 단위 초점거리이기 때문에 0.5배

	float New_FNumber = FNumber * 2;			//새로운 초점거리 -> 확대 축소

	//픽셀 중심 좌표(리사이즈된 이미지의)
	float Co = ReducedImage.cols / 2.;			
	float Ro = ReducedImage.rows / 2.;

	// 3D Transformation Scale 변환- in Camera Coordinate System
	cv::Mat_<float> output_vec = cv::Mat(3, 1, CV_32F);
	cv::Mat_<float> input_vec = cv::Mat(3, 1, CV_32F);
	for (int r = 0; r < ReducedImage.rows; r++) {
		for (int c = 0; c < ReducedImage.cols; c++) {
			// 영상좌표계를 카메라 좌표계로 변환 (단위는 픽셀)
			output_vec(0) = c - Co;
			output_vec(1) = -(r - Ro);
			output_vec(2) = -New_FNumber;

			float weight = FNumber / New_FNumber;
			input_vec(0) = weight * output_vec(0);
			input_vec(1) = weight * output_vec(1);

			MapX(r, c) = input_vec(0) + Co;
			MapY(r, c) = -input_vec(1) + Ro;
		}
	}

	cv::Mat image3;
	cv::remap(ReducedImage, image3, MapX, MapY, cv::INTER_LINEAR);


	// 3D Transformation Scale 변환- in Image Coordinate System
	cv::Mat_<float> Homography = cv::Mat::eye(3, 3, CV_64FC1);
	float Weight = New_FNumber / FNumber;
	Homography(2, 2) = Weight;
	cv::Mat_<float> outputVector(3, 1, CV_32FC1);
	cv::Mat_<float> inputVector(3, 1, CV_32FC1);
	for (int r = 0; r < ReducedImage.rows; r++) {
		for (int c = 0; c < ReducedImage.cols; c++) {
			outputVector(0) = c;
			outputVector(1) = r;
			outputVector(2) = 1;
			inputVector = Homography * outputVector;

			MapX(r, c) = inputVector(0) / inputVector(2);
			MapY(r, c) = inputVector(1) / inputVector(2);
		}
	}
	cv::Mat image4;
	cv::remap(ReducedImage, image4, MapX, MapY, cv::INTER_LINEAR);

	cv::imshow("", image3);
	cv::waitKey(0);
}

void Practice_7_2() // 3D Transformation Translation 변환
{
	cv::Mat inputImage = cv::imread("resource/100_0007_0001.JPG", cv::IMREAD_UNCHANGED);
	std::cout << inputImage.cols << "\t" << inputImage.rows << std::endl;

	cv::Mat ReducedImage;
	int reduce_factor = 2.;
	cv::resize(inputImage, ReducedImage, cv::Size(inputImage.cols / reduce_factor, inputImage.rows / reduce_factor));
	std::cout << ReducedImage.cols << "\t" << ReducedImage.rows << std::endl;


	cv::Mat_<float> MapX = cv::Mat(ReducedImage.rows, ReducedImage.cols, CV_32FC1);
	cv::Mat_<float> MapY = cv::Mat(ReducedImage.rows, ReducedImage.cols, CV_32FC1);

	float F_Pixel = 8800. / 2.4;
	float FNumber = F_Pixel / reduce_factor;
	float Co = ReducedImage.cols / 2.;
	float Ro = ReducedImage.rows / 2.;

	float x_move, y_move, z_move;
	x_move = 0.;
	y_move = 0.;
	z_move = FNumber / 5.;

	cv::Mat_<float> output_vec = cv::Mat(3, 1, CV_32F);
	cv::Mat_<float> input_vec = cv::Mat(3, 1, CV_32F);
	for (int r = 0; r < ReducedImage.rows; r++) {
		for (int c = 0; c < ReducedImage.cols; c++) {
			output_vec(0) = c - Co;
			output_vec(1) = -(r - Ro);
			output_vec(2) = -FNumber;

			float weight = (-FNumber + z_move) / (-FNumber);
			input_vec(0) = (output_vec(0) + x_move) / weight;
			input_vec(1) = (output_vec(1) + y_move) / weight;

			MapX(r, c) = input_vec(0) + Co;
			MapY(r, c) = -input_vec(1) + Ro;
		}
	}
	cv::Mat TranslatedImage;
	cv::remap(ReducedImage, TranslatedImage, MapX, MapY, cv::INTER_LINEAR);

	cv::waitKey(0);
}

void Practice_7_3() // 3D Transformation rotation 변환
{
	cv::Mat inputImage = cv::imread("resource/010264.png", cv::IMREAD_UNCHANGED);
	std::cout << inputImage.cols << "\t" << inputImage.rows << std::endl;

	cv::Mat ReducedImage;
	int reduce_factor = 2.;
	cv::resize(inputImage, ReducedImage, cv::Size(inputImage.cols / reduce_factor, inputImage.rows / reduce_factor));
	std::cout << ReducedImage.cols << "\t" << ReducedImage.rows << std::endl;


	cv::Mat_<float> MapX = cv::Mat(ReducedImage.rows, ReducedImage.cols, CV_32FC1);
	cv::Mat_<float> MapY = cv::Mat(ReducedImage.rows, ReducedImage.cols, CV_32FC1);

	float F_Pixel = 8800. / 2.4;
	float FNumber = 3667;
	float Co = ReducedImage.cols / 2.;
	float Ro = ReducedImage.rows / 2.;


	cv::Mat_<float> RotX = cv::Mat(3, 3, CV_32F);
	cv::Mat_<float> RotY = cv::Mat(3, 3, CV_32F);
	cv::Mat_<float> RotZ = cv::Mat(3, 3, CV_32F);

	float Deg2Rad = acos(-1.) / 180;
	float omega = 30 * Deg2Rad;;
	float phi = -0 * Deg2Rad; // -25. * Deg2Rad;
	float kappa = 0 * Deg2Rad;

	RotX(0, 0) = 1.;            RotX(0, 1) = 0.;            RotX(0, 2) = 0.;
	RotX(1, 0) = 0.;            RotX(1, 1) = cos(omega);    RotX(1, 2) = -sin(omega);
	RotX(2, 0) = 0.;            RotX(2, 1) = sin(omega);    RotX(2, 2) = cos(omega);

	RotY(0, 0) = cos(phi);      RotY(0, 1) = 0.;            RotY(0, 2) = sin(phi);
	RotY(1, 0) = 0.;            RotY(1, 1) = 1.;            RotY(1, 2) = 0.;
	RotY(2, 0) = -sin(phi);     RotY(2, 1) = 0.;            RotY(2, 2) = cos(phi);

	RotZ(0, 0) = cos(kappa);    RotZ(0, 1) = -sin(kappa);   RotZ(0, 2) = 0.;
	RotZ(1, 0) = sin(kappa);    RotZ(1, 1) = cos(kappa);    RotZ(1, 2) = 0.;
	RotZ(2, 0) = 0.;            RotZ(2, 1) = 0.;            RotZ(2, 2) = 1.;

	cv::Mat_<float> RotAll = RotZ * RotY * RotX;

	cv::Mat_<float> output_vec = cv::Mat(3, 1, CV_32F);
	cv::Mat_<float> input_vec = cv::Mat(3, 1, CV_32F);
	for (int r = 0; r < ReducedImage.rows; r++) {
		for (int c = 0; c < ReducedImage.cols; c++) {
			output_vec(0) = c - Co;
			output_vec(1) = -(r - Ro);
			output_vec(2) = -FNumber;

			input_vec = RotAll.t() * output_vec;
			float weight = input_vec(2) / (-FNumber);
			MapX(r, c) = input_vec(0) / weight + Co;
			MapY(r, c) = -input_vec(1) / weight + Ro;
		}
	}


	cv::Mat image3;
	cv::remap(ReducedImage, image3, MapX, MapY, cv::INTER_LINEAR);

	cv::imshow("", image3);
	cv::waitKey(0);
}

cv::Mat_<double> GetTranslationMatrix(double Tx, double Ty)
{
	cv::Mat_<double> OutTranslationMatrix = cv::Mat::zeros(3, 3, CV_64FC1);

	OutTranslationMatrix(0, 0) = 1.;    OutTranslationMatrix(0, 1) = 0.;    OutTranslationMatrix(0, 2) = Tx;
	OutTranslationMatrix(1, 0) = 0.;    OutTranslationMatrix(1, 1) = 1.;    OutTranslationMatrix(1, 2) = Ty;
	OutTranslationMatrix(2, 0) = 0.;    OutTranslationMatrix(2, 1) = 0.;    OutTranslationMatrix(2, 2) = 1.;

	return OutTranslationMatrix;
}

cv::Mat_<double> GetRoatationMatrix(double Angle, cv::Point2d pivotPoint) {

	double AngleRad = Angle * CV_PI / 180.;

	cv::Mat_<double> OutRotationMatrix(3, 3, CV_64FC1);

	cv::Mat_<double> RotationMatrix = cv::Mat::zeros(3, 3, CV_64FC1);

	cv::Mat_<double> PivotMatrix = GetTranslationMatrix(pivotPoint.x, pivotPoint.y);

	RotationMatrix(0, 0) = cos(AngleRad);	RotationMatrix(0, 1) = -sin(AngleRad);	RotationMatrix(0, 2) = 0.;
	RotationMatrix(1, 0) = sin(AngleRad);	RotationMatrix(1, 1) = cos(AngleRad);	RotationMatrix(1, 2) = 0.;
	RotationMatrix(2, 0) = 0.;				RotationMatrix(2, 1) = 0.;				RotationMatrix(2, 2) = 1.;

	// 원점으로 피봇포인트를 옮기고, 회전(반시계)이지만 y축이 -방향이므로 시계방향, 그리고 다시 복구
	OutRotationMatrix = PivotMatrix * RotationMatrix * PivotMatrix.inv();

	return OutRotationMatrix;
}

void ComputeOutImageSize(cv::Size inputSize, cv::Mat_<double> src2dstMat, cv::Size& outputSize, cv::Point2d& offset)
{
	std::vector<cv::Point2d> corners = {
		{0., 0.},                               // Top-Left
		{(double)inputSize.width, 0.},                  // Top-Right
		{0., (double)inputSize.height},                 // Bottom-Left
		{(double)inputSize.width, (double)inputSize.height }    // Bottom-Right
	};
	std::vector<cv::Point2d> warped(4);

	for (int i = 0; i < 4; i++) {
		cv::Mat_<double> iMat = cv::Mat::zeros(3, 1, CV_64FC1);
		cv::Mat_<double> oMat = cv::Mat::zeros(3, 1, CV_64FC1);
		iMat(0, 0) = corners[i].x;  iMat(1, 0) = corners[i].y;  iMat(2, 0) = 1.;
		oMat = src2dstMat * iMat;
		warped[i].x = oMat(0, 0);
		warped[i].y = oMat(1, 0);
	}

	double xmin = warped[0].x, xmax = warped[0].x;
	double ymin = warped[0].y, ymax = warped[0].y;
	for (int i = 0; i < 4; i++) {
		xmin = std::min(xmin, warped[i].x);
		xmax = std::max(xmax, warped[i].x);
		ymin = std::min(ymin, warped[i].y);
		ymax = std::max(ymin, warped[i].y);
	}

	offset = cv::Point2d(-xmin, -ymin);
	//int Width = (int)std::ceil(xmax - xmin);
	//int Height = (int)std::ceil(ymax - ymin);
	int Width = (int)std::ceil(xmax);
	int Height = (int)std::ceil(ymax);
	outputSize = cv::Size(std::max(Width, 1), std::max(Height, 1));
}

void BuildRemapMatrix(cv::Size& OutSize, cv::Mat_<double>& dst2srcMat, cv::Mat_<float>& MapX, cv::Mat_<float>& MapY)
{
	// 인버스 매트릭스를 통해 이 점이 어디에서 온거노? 를 계산
	MapX.create(OutSize);
	MapY.create(OutSize);

	for (int row = 0; row < OutSize.height; row++) {
		float* MapXptr = MapX.ptr<float>(row);
		float* MapYptr = MapY.ptr<float>(row);
		for (int col = 0; col < OutSize.width; col++) {
			cv::Mat_<double> dstPoint(3, 1, CV_64FC1);
			cv::Mat_<double> srcPoint(3, 1, CV_64FC1);

			//만약 Homography변환이라면
			//double w = srcPoint(2, 0);
			//MapXptr[col] = (float)(srcPoint(0, 0) / w);
			//MapYptr[col] = (float)(srcPoint(1, 0) / w);

			dstPoint(0, 0) = col;   dstPoint(1, 0) = row;   dstPoint(2, 0) = 1.;	//동차 좌표 입력
			srcPoint = dst2srcMat * dstPoint;	//출력 영상의 포인트가 입력영상의 어디서 온거지?

			//매핑 테이블에 기록
			MapXptr[col] = static_cast<float>(srcPoint(0, 0));
			MapYptr[col] = static_cast<float>(srcPoint(1, 0));
		}
	}
}
