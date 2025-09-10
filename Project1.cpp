#include "Project1.h"

void Project1()
{
	// 경로설정
	std::string folder_dir = "resource/";
	std::string image_name = "R0047633.JPG";
	std::string image_dir = folder_dir + image_name;
	std::string IOP_name = folder_dir + "IOFile_Metric.txt";
	std::string EOP_name = folder_dir + "input_EOP.txt";
	std::string GCP_name = folder_dir + "GCP.txt";

	// IOP, EOP, GCP Read
	IOP ImageIOP;

	std::vector<EOP> ImageEOPs;
	EOP ImageEOP;

	std::vector<GCP> ImageGCPs;
	std::vector<GCP> ImageGCP;

	ReadIOP(IOP_name, ImageIOP);
	ReadEOP(EOP_name, ImageEOPs);
	ReadGCP(GCP_name, ImageGCPs);
	
	// EOP vector중 입력 영상에 해당하는 EOP Pick
	ReadImageEOP(ImageEOPs, image_name, ImageEOP);

	// GCP vector중 입력 영상에 해당하는 GCP Pick
	ReadImageGCP(ImageGCPs, image_name, ImageGCP);

	// Rotation Matrix 초기화 및 계산
	cv::Mat_<double> RotationMat = cv::Mat::zeros(3, 3, CV_64FC1);

	double omega = ImageEOP.O * ToRad;
	double phi = ImageEOP.P * ToRad;
	double kappa = ImageEOP.K * ToRad;

	RotationMat = GetRotationMatrix(omega, phi, kappa);

	// 공선조건식 적용 (X, Y, Z) to (C, R)
	for (int i = 0; i < ImageGCP.size(); i++) {
		// GCP XYZ에 기록된 월드 XYZ좌표
		double X = ImageGCP[i].xyz.X;
		double Y = ImageGCP[i].xyz.Y;
		double Z = ImageGCP[i].xyz.Z;

		// GCP에 기록된 픽셀 좌표
		double c = ImageGCP[i].order[0].CR.Col;
		double r = ImageGCP[i].order[0].CR.Row;

		double cal_c, cal_r;
		// GCP XYZ -> Calc C,R
		InverseMapping(ImageIOP, ImageEOP, RotationMat, X, Y, Z, cal_c, cal_r);

		// Inverce mapping function으로 계산된 픽셀좌표와 GCP 픽셀좌표 비교
		std::cout << "Calc (C,R) = " << cal_c << "\t" << cal_r << endl;
		std::cout << "True (C,R) = " << c << "\t" << r << endl;
		std::cout << "Error X = " << cal_c - c << endl;
		std::cout << "Error Y = " << cal_r - r << endl;
		std::cout << "Error   = " << std::sqrt(std::pow(cal_c - c , 2) + std::pow(cal_c - c , 2))<< endl<<endl;
	}
}
