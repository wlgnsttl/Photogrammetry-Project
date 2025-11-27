#pragma once

#define _CRT_SECURE_NO_WARNINGS
#include "FrmCamera.h"
#include <random>


#define RO_BX_CASE 100
#define RO_BY_CASE 200

// The following RASAC constants are also defined in MultiModel.h
#ifndef RANSAC_THRESHOLD
#define RANSAC_TIE_THRESHOLD  3	
#define RANSAC_GCP_THRESHOLD  3	
#define RANSAC_LOOP		  200
#endif

#ifndef DEFAULT_ITERATION_RO
#define DEFAULT_ITERATION_RO 200
#endif

#ifndef LEFT_FLAG
#define LEFT_FLAG		0
#define RIGHT_FLAG		1
#endif

struct TP {
	double lcol, lrow, rcol, rrow;
};

class CFrmCameraStereoModel
{
private:
	CFrmCamera *Left;
	CFrmCamera *Right;

	int _InputType, _OutputType;
	double Math_PI;
	int ROIterationNumber;

	int SetUpPrecisionModel(int LRFlag, TCHAR GCPFileName[], fstream& log);

	int GetXYZ_LinearCL(double xl, double yl, double xr, double yr, double& lon, double& lat, double& height);
	int GetXYZ_MinDist(double xl, double yl, double xr, double yr, double& lon, double& lat, double& height);

	double left_epi_angle, right_epi_angle;

public:
	CFrmCameraStereoModel(void);
	~CFrmCameraStereoModel(void);

	void SetLeftModel(CFrmCamera* lmodel);
	void SetRightModel(CFrmCamera* rmodel);
	CFrmCamera* GetLeftModel() { return Left; }
	CFrmCamera* GetRightModel() { return Right; }

	void SetInputOutputXYZ(int inputxyz, int outputxyz);

	double _EMat[9], _FMat[9], _KL[9], _KR[9];
	double m_dLColOffset, m_dLRowOffset, m_dRColOffset, m_dRRowOffset;
	double m_dLColScale, m_dLRowScale, m_dRColScale, m_dRRowScale;

	void GetNormalizedColRow(int LRFlag, double col, double row, double& norm_col, double& norm_row);
	void CalcFundamentalMatrix(char* TiePointFile);
	void CalcFundamentalMatrix(vector<TP> & tp);
	double CalcEssentialMatrix(vector<TP>& tp);

	// Temporary code for test. Estimate F by normalized coordinates (Cn, Rn, 1)
	void CalcDenomFundamentalMatrix(vector<TP> & tp);
	void CalcFundamentalMatrix_byDirectEstimation(vector<TP> & tp);
	void CheckFundamentalMatrix(vector<TP> &tp);

	void SetFundamentalMatrix(double Fmat[9]);


	int GetXYZ(double xl, double yl, double xr, double yr, double& lon, double& lat, double& height);
	int GetXYZLD(double xl, double yl, double xr, double yr, double& lon, double& lat, double& height);
	int GetXYZ_byDLT(double xl, double yl, double xr, double yr, double& lon, double& lat, double& height);

	int ForwardMapping(int LRFlag, double col, double row, double& XX, double& YY, double& dumHeight, double RealHeight);
	int ForwardMapping(int LRFlag, double col, double row, double& XX, double& YY, double& dumHeight);

	int InverseMapping(int LRFlag, int InputType, double GX, double GY, double GZ, double& col, double& row);
	int InverseMapping(int LRFlag, double GX, double GY, double GZ, double& col, double& row);

	int SetupROModel(vector<TP>& tp, fstream& log, int iteration_number = DEFAULT_ITERATION_RO);

	// Determine Inler and Outlier by Relative Orientation Based Ransac. Coplanar Eq is used
	// Thie method does not affect EOP of left and right images
	int Ransac_RO(vector<TP>&tie, vector<bool>& InlierFlag, fstream& log, double error_bound = RANSAC_TIE_THRESHOLD, int RansacLoopLimit = RANSAC_LOOP);

	// Determine Inler and Outlier by Fundamental Matrix Based Ransac. 
	// Thie method does not affect EOP of left and right images
	int Ransac_F(vector<TP>& tie, vector<bool>& InlierFlag, fstream& log, double error_bound = RANSAC_TIE_THRESHOLD, int RansacLoopLimit = RANSAC_LOOP);
	int Ransac_H(vector<TP>& tie, vector<bool>& InlierFlag, fstream& log, double error_bound = RANSAC_TIE_THRESHOLD, int RansacLoopLimit = RANSAC_LOOP);

	double LSE(double *LSE_A, double *LSE_x_hat, double *LSE_l, int ParameterNo, int ObservationNo, double* Covariance);

	int GetInputType() { return _InputType; }
	int GetOutputType() { return _OutputType; }

	int SetupROModelBx(TCHAR TieFileName[], fstream& log);
	int SetupROModelBy(TCHAR TieFileName[], fstream& log);
	int SetupROModelBx(vector<TP>& TiePoint, fstream& log, int iteration_number = DEFAULT_ITERATION_RO);
	int SetupROModelBy(vector<TP>& TiePoint, fstream& log, int iteration_number = DEFAULT_ITERATION_RO);

};
