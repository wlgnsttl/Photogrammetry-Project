/*
- 내용: 프레임카메라 클래스
- 작성: 김태정, 이태윤, 이수암, 김재인, 김수현
- 버전: 2.3 (2018-05-17)
- 이력사항
- SHKim(180517): SetupPrcsnFCamModel_byCL(), GCP를 이용한 공선조건식 기반의 센서모델링 함수 추가
- SHKim(180517): SetupPrcsnFCamModel_byDLT(), GCP를 이용한 DLT 기반의 센서모델링 함수 추가
- SHKim(180518): SetupPrcsnFCamModel_byCL(), ppsOffset_X, Y, 초기값 0으로 설정
- SHKim(180518): MDD용 프로젝트에 맞게 코드 병합
- TJKim(180809): Omega, Phi, Kappa Orientation Angle 회전방향 및 회전순서 설명 변경 
- TJKim(201007): Modified for 3D Image Programming Class
*/
#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <opencv.hpp>

using namespace std;
#define TCHAR char
#define SM_ERROR -9999

#define cM_PI	acos(-1)

const double tORAD = cM_PI / 180.;
const double tODEG = 180. / cM_PI;

struct GCP {
	double col, row, XX, YY, ZZ;
};

enum{
	_ONRM_
	, _DLT_
	, _SPOTOA_
	, _SPOT5_OA_
	, _QUICKBIRD_
	, _KOMPSAT2_PAN_
	, _KOMPSAT2_MS_
	, _OrbviewDLT_
	, _RFM_
	, _DMC_
	, _Rollei_
	, _UltraCam_
	, _ADS_
	, _UnknownCam_ = 10000
};

// 180809 - Revised By TJKim
// Inha Model: omega, phi, kappa : from Object to Image
// Rotaiton sequence: 1-2-3 system. omega --> phi --> kappa

class CFrmCamera
{
public:
	CFrmCamera(int sType = _DMC_);
	virtual ~CFrmCamera(void);

	int GetCameraInfo( double& Xs, double& Ys, double& Zs,
					double& R_i2o_11, double& R_i2o_12, double& R_i2o_13, double& R_i2o_21, double& R_i2o_22, double& R_i2o_23,
					double& R_i2o_31, double& R_i2o_32, double& R_i2o_33, double& FLength);
	int ForwardMapping( double col, double row, double& GX, double& GY, double& GZ );
	int ForwardMapping( double col, double row, double& GX, double& GY, double& GZ, double H );
	int InverseMapping( double GX, double GY, double GZ, double& col, double &row);
	int InverseMapping_byDLT(double GX, double GY, double GZ, double& col, double &row);
	int SetCameraType(int sType);
	int SetCameraPara(double f, double pxlSize, int XSize, int YSize);
	int SetCameraPara(double f, double pxlSizeX, double pxlSizeY, int XSize, int YSize, double PPSOff_X = 0., double PPSOff_Y = 0.);
	int SetCameraPara(double f, double PPSOff_X, double PPSOff_Y);

	int SetupFCamModel(TCHAR* GPSINAF, fstream& log); //GPS/INS
	int SetupFCamModel(TCHAR* GPSINAF); //GPS/INS
	int SetupFCamModel(double Bx, double By, double Bz, double Omega, double Phi, double Kappa); //GPS/INS

	int SetupPrcsnFCamModel(TCHAR* GCPFILE, fstream& log);
	int SetupPrcsnFCamModel(vector<GCP> & ModelGCP, fstream& log);
	int SetupPrcsnFCamModel(vector<GCP> & ModelGCP, vector<double>&Weight, fstream& log);
	int SetupPrcsnFCamModel(vector<GCP> & ModelGCP, double & RMSErrorinPixel, fstream& log);
	int SetupPrcsnFCamModel(vector<GCP> & ModelGCP, vector<double>& Weight, double & RMSErrorinPixel, fstream& log);
	int SetupPrcsnFCamModel_byCL(vector<GCP> & ModelGCP, fstream& log);		// SHKim(180517): GCP를 이용한 공선조건식 기반의 센서모델링 함수 추가
	int SetupPrcsnFCamModel_byDLT(vector<GCP> & ModelGCP, fstream& log, bool UpdateEOPFlag = false);	// SHKim(180517): GCP를 이용한 DLT 기반의 센서모델링 함수 추가

	void GenerateOutput( fstream& report );
	int GetNumCols();
	int GetNumRows();
	int Image2Cam(double col, double row, double &x, double &y);
	int Cam2Image(double x, double y, double &col, double &row);

	// JIKim(150910): 함수추가
	void SetNumCols(int nColSize);  
	void SetNumRows(int nRowSize);
	void SetPPsOffset(double dPPX, double dPPY);
	void GetPPsOffset(double &dPPX, double &dPPY);

	void GetEOPara (double& X, double& Y, double& Z, double& W_rad, double& P_rad, double& K_rad);
	void SetEOPara (double X, double Y, double Z, double W_rad, double P_rad, double K_rad);
	double* GetDLT_Denom() { return DLT_M; }

	void GetIOPara (double& ColSize, double& RowSize, double& CCDSizeX, double& CCDSizeY, double& FocalLength,  double &PPSOff_X, double &PPSOff_Y);

	void GetLensDistortion( double &k1, double &k2, double &k3, double& p1, double& p2 );
	void SetLensDistortion( double  k1, double  k2, double  k3, double  p1, double  p2 );
	void UnsetLensDistortion();

	int Origin2Undistorted(double col, double row, double &undist_col, double &undist_row);
	int Undistorted2Origin(double undist_col, double undist_row, double &col, double &row);

	int ForwardMappingLD( double col, double row, double& GX, double& GY, double& GZ );
	int ForwardMappingLD( double col, double row, double& GX, double& GY, double& GZ, double H );
	int InverseMappingLD( double GX, double GY, double GZ, double& col, double &row);
	int Image2CamLD(double col, double row, double &x, double &y);
	int Cam2ImageLD(double x, double y, double &col, double &row);

	int ForwardMappingULD( double col, double row, double& GX, double& GY, double& GZ );
	int ForwardMappingULD( double col, double row, double& GX, double& GY, double& GZ, double H );
	int InverseMappingULD( double GX, double GY, double GZ, double& col, double &row);
	int Image2CamULD(double col, double row, double &x, double &y);
	int Cam2ImageULD(double x, double y, double &col, double &row);

	int GetRadialLD(double x, double y, double &delta_x, double &delta_y);
	int GetTangentialLD(double x, double y, double &delta_x, double &delta_y);

	double GetCenterCol( double col ) { return col - (m_nColSize*0.5+m_ppsOff_X ); }
	double GetCenterRow( double row ) { return -row + (m_nRowSize*0.5+m_ppsOff_Y ); }

	// Get Omega, Phi, Kappa Angles from RotationMatrix
	// Note that due to historical reason Rotation Matrix is defined as rotation from Image to Object
	// and Omega, Phi, Kappa are angles from Object to Image
	void Rotation2Angles(double R_i2o[9], double &W_o2i_rad, double &P_o2i_rad, double &K_o2i_rad);
	bool GetRotationByTwoVector(double Vec1[3], double Vec2[3], double Rot[9]);
	void GetDetectorVector(double col, double row, double& Dx, double& Dy, double& Dz);

	void CheckImageGSD(double RefHeight, double& ColGSD, double& RowGSD, double& AllGSD);

	int LSE(double *LSE_A, double *LSE_x_hat, double *LSE_l, int ParameterNo, int ObservationNo, double* Covariance);
	double LSE(double *A, double *x_hat, double *l, double *w, int ParameterNo, int ObservationNo);

private:
	double m_dFocalLen;
	double m_dCPixelSizeX;
	double m_dCPixelSizeY;
	int m_nColSize;
	int m_nRowSize;

	int m_eSensorType;

	double m_i_dCPixelSizeX;
	double m_i_dCPixelSizeY;

	string m_strID;
	double m_dBx;
	double m_dBy;
	double m_dBz;
	double m_dOmega;
	double m_dPhi;
	double m_dKappa;
	//double m_adRm[4][4]; 
	double m_R_image2obj[4][4]; // 편하기 위해서 4 x 4이고 실제로는 1~3까지만 사용됨.
	double m_ppsOff_X;
	double m_ppsOff_Y;

	double m_dK1, m_dK2, m_dK3;
	double m_dP1, m_dP2;
	bool m_bLensDistortion;

	double m_dU2DK1, m_dU2DK2, m_dU2DK3, m_dU2DK4, m_dU2DK5, m_dU2DP1, m_dU2DP2; // , m_dU2DP3, m_dU2DP4;

	double DLT_M[12];  // un-normalized DLT M matrix

	void SetUndistorted2DistortedTransform();

};