/*
- 내용: 프레임카메라 클래스
- 작성: 김태정, 이태윤, 이수암, 김재인, 김수현
- 버전: 2.3 (2018-05-17)
- 이력사항
- SHKim(180517): SetupPrcsnFCamModel_byCL(), GCP를 이용한 공선조건식 기반의 센서모델링 함수 추가
- SHKim(180517): SetupPrcsnFCamModel_byDLT(), GCP를 이용한 DLT 기반의 센서모델링 함수 추가
- SHKim(180518): SetupPrcsnFCamModel_byCL(), ppsOffset_X, Y, 초기값 0으로 설정
- SHKim(180518): MDD용 프로젝트에 맞게 코드 병합
*/

#include "FrmCamera.h"
#include <iostream>
#include <cmath>

const double M_PI = acos(-1.);

CFrmCamera::CFrmCamera(int sType)
{
	m_eSensorType = sType;

	if(SetCameraType(sType) == SM_ERROR){
		cout << "Sensor Type Error\n";
	}

	m_dK1 = m_dK2 = m_dK3 = m_dP1 = m_dP2 = 0.;
	m_bLensDistortion = false;		
}

CFrmCamera::~CFrmCamera(void)
{
}

int CFrmCamera::GetNumCols(){ return m_nColSize; }
int CFrmCamera::GetNumRows(){ return m_nRowSize; }

// JIKim(150910): 함수추가
void CFrmCamera::SetNumCols(int nColSize){ m_nColSize = nColSize; }
void CFrmCamera::SetNumRows(int nRowSize){ m_nRowSize = nRowSize; }
void CFrmCamera::SetPPsOffset(double dPPX, double dPPY){ m_ppsOff_X = dPPX; m_ppsOff_Y = dPPY; }
void CFrmCamera::GetPPsOffset(double &dPPX, double &dPPY) {
	dPPX = m_ppsOff_X;
	dPPY = m_ppsOff_Y; 
}

// Retrieve EOP of an Image
// Nate that W, P, K are angles from object to image in Radian
void CFrmCamera::GetEOPara (double& X, double& Y, double& Z, double& W_o2i_Rad, double& P_o2i_Rad, double& K_o2i_Rad) {
	X = m_dBx; Y = m_dBy; Z = m_dBz; W_o2i_Rad = m_dOmega; P_o2i_Rad = m_dPhi; K_o2i_Rad = m_dKappa; 
}

void CFrmCamera::GetIOPara (double& ColSize, double& RowSize, double& CCDSizeX, double& CCDSizeY, double& FocalLength,  double &PPSOff_X, double &PPSOff_Y ) {
	ColSize = double(m_nColSize); RowSize = double(m_nRowSize); CCDSizeX = double(m_dCPixelSizeX); CCDSizeY = double(m_dCPixelSizeY); FocalLength = double(m_dFocalLen);
	PPSOff_X = m_ppsOff_X;
	PPSOff_Y = m_ppsOff_Y;
}

// Note that Rotation Matrix is from image to object frame
int CFrmCamera::GetCameraInfo( double& Xs, double& Ys, double& Zs,
							double& R_i2o_11, double& R_i2o_12, double& R_i2o_13, double& R_i2o_21, double& R_i2o_22, double& R_i2o_23,
							double& R_i2o_31, double& R_i2o_32, double& R_i2o_33, double& FLength){

	Xs = m_dBx;  Ys = m_dBy;  Zs = m_dBz;
	R_i2o_11 = m_R_image2obj[1][1];  R_i2o_12 = m_R_image2obj[1][2];  R_i2o_13 = m_R_image2obj[1][3];
	R_i2o_21 = m_R_image2obj[2][1];  R_i2o_22 = m_R_image2obj[2][2];  R_i2o_23 = m_R_image2obj[2][3];
	R_i2o_31 = m_R_image2obj[3][1];  R_i2o_32 = m_R_image2obj[3][2];  R_i2o_33 = m_R_image2obj[3][3];
	FLength = m_dFocalLen;
			
	return 0;
}

int CFrmCamera::ForwardMapping( double col, double row, double& GX, double& GY, double& GZ ){

	if( m_bLensDistortion == true ) return ForwardMappingLD(col, row, GX, GY, GZ ); 
	else return ForwardMappingULD(col, row, GX, GY, GZ ); 

}

int CFrmCamera::ForwardMappingULD( double col, double row, double& GX, double& GY, double& GZ ){

	double xx, yy;
	Image2CamULD( col,  row, xx, yy);

	double ex, ey, ez;
	ex = m_R_image2obj[1][1] * xx + m_R_image2obj[1][2] * yy + m_R_image2obj[1][3] * (-m_dFocalLen);
	ey = m_R_image2obj[2][1] * xx + m_R_image2obj[2][2] * yy + m_R_image2obj[2][3] * (-m_dFocalLen);
	ez = m_R_image2obj[3][1] * xx + m_R_image2obj[3][2] * yy + m_R_image2obj[3][3] * (-m_dFocalLen);

	GZ = 0.;
	GX = (GZ - m_dBz)*ex/ez + m_dBx;
	GY = (GZ - m_dBz)*ey/ez + m_dBy;

	return 0;
}

int CFrmCamera::ForwardMapping( double col, double row, double& GX, double& GY, double& GZ, double H ){
		
	if( m_bLensDistortion == true ) return ForwardMappingLD(col, row, GX, GY, GZ, H ); 
	else ForwardMappingULD(col, row, GX, GY, GZ, H ); 

}

int CFrmCamera::ForwardMappingULD( double col, double row, double& GX, double& GY, double& GZ, double H ){
		
	double xx, yy;
	Image2CamULD( col,  row, xx, yy);

	double ex, ey, ez;
	ex = m_R_image2obj[1][1] * xx + m_R_image2obj[1][2] * yy + m_R_image2obj[1][3] * (-m_dFocalLen);
	ey = m_R_image2obj[2][1] * xx + m_R_image2obj[2][2] * yy + m_R_image2obj[2][3] * (-m_dFocalLen);
	ez = m_R_image2obj[3][1] * xx + m_R_image2obj[3][2] * yy + m_R_image2obj[3][3] * (-m_dFocalLen);

	double i_ez = 1 / ez;
	GZ = H;
//	GX = (GZ - m_dBz)*ex/ez + m_dBx;
//	GY = (GZ - m_dBz)*ey/ez + m_dBy;
	GX = (GZ - m_dBz)*ex*i_ez + m_dBx;
	GY = (GZ - m_dBz)*ey*i_ez + m_dBy;

	return 0;
}

void CFrmCamera::GetDetectorVector(double col, double row, double& Dx, double& Dy, double& Dz){

	double xx, yy;
	Image2CamULD(col, row, xx, yy);

	double ex, ey, ez;
	ex = m_R_image2obj[1][1] * xx + m_R_image2obj[1][2] * yy + m_R_image2obj[1][3] * (-m_dFocalLen);
	ey = m_R_image2obj[2][1] * xx + m_R_image2obj[2][2] * yy + m_R_image2obj[2][3] * (-m_dFocalLen);
	ez = m_R_image2obj[3][1] * xx + m_R_image2obj[3][2] * yy + m_R_image2obj[3][3] * (-m_dFocalLen);

	Dx = ex;
	Dy = ey;
	Dz = ez;
}

int CFrmCamera::InverseMapping( double GX, double GY, double GZ, double& col, double &row){
	
	if( m_bLensDistortion == true ) return InverseMappingLD(GX, GY, GZ, col, row ); 
	else return InverseMappingULD(GX, GY, GZ, col, row ); 

}

int CFrmCamera::InverseMappingULD( double GX, double GY, double GZ, double& col, double &row){
	
	double xx,yy;

	//xx = -m_dFocalLen * ( ( m_adRm[1][1]*(GX-m_dBx) + m_adRm[2][1]*(GY-m_dBy) + m_adRm[3][1]*(GZ-m_dBz) ) 
	//		 / ( m_adRm[1][3]*(GX-m_dBx) + m_adRm[2][3]*(GY-m_dBy) + m_adRm[3][3]*(GZ-m_dBz) ) );

	//yy = -m_dFocalLen * ( ( m_adRm[1][2]*(GX-m_dBx) + m_adRm[2][2]*(GY-m_dBy) + m_adRm[3][2]*(GZ-m_dBz) )
	//		 / ( m_adRm[1][3]*(GX-m_dBx) + m_adRm[2][3]*(GY-m_dBy) + m_adRm[3][3]*(GZ-m_dBz) ) );

	double denom;
	denom = 1 / (m_R_image2obj[1][3] * (GX - m_dBx) + m_R_image2obj[2][3] * (GY - m_dBy) + m_R_image2obj[3][3] * (GZ - m_dBz));

	xx = -m_dFocalLen * (m_R_image2obj[1][1] * (GX - m_dBx) + m_R_image2obj[2][1] * (GY - m_dBy) + m_R_image2obj[3][1] * (GZ - m_dBz)) * denom;
	yy = -m_dFocalLen * (m_R_image2obj[1][2] * (GX - m_dBx) + m_R_image2obj[2][2] * (GY - m_dBy) + m_R_image2obj[3][2] * (GZ - m_dBz)) * denom;

	Cam2ImageULD( xx, yy, col, row ); 

	return 0;
}

int CFrmCamera::InverseMapping_byDLT(double GX, double GY, double GZ, double& col, double &row) {

	col = (DLT_M[0] * GX + DLT_M[1] * GY + DLT_M[2] * GZ + DLT_M[3]) / (DLT_M[8] * GX + DLT_M[9] * GY + DLT_M[10] * GZ + DLT_M[11]);
	row = (DLT_M[4] * GX + DLT_M[5] * GY + DLT_M[6] * GZ + DLT_M[7]) / (DLT_M[8] * GX + DLT_M[9] * GY + DLT_M[10] * GZ + DLT_M[11]);

	return true;
}

int CFrmCamera::SetCameraType(int sType){

	if(sType == _DMC_)
		return SetCameraPara(120./1000, 0.0120/1000, 7680, 13824);
	else if(sType == _Rollei_)
		return SetCameraPara(51.607/1000, 0.0068/1000, 7228, 5428);
	else if(sType == _UltraCam_)
		return SetCameraPara(101.400/1000, 0.009/1000, 11500, 7500);
	else
		return SM_ERROR;	

	m_eSensorType = sType;
	return 0;
}


int CFrmCamera::SetCameraPara(double f, double pxlSizeX, double pxlSizeY, int XSize, int YSize, double PPSOff_X, double PPSOff_Y)
{
	m_dFocalLen = f; 
	m_dCPixelSizeX = pxlSizeX; 
	m_dCPixelSizeY = pxlSizeY; 
	m_nColSize = XSize; 
	m_nRowSize = YSize;
	m_ppsOff_X = PPSOff_X;
	m_ppsOff_Y = PPSOff_Y;

	m_i_dCPixelSizeX = 1. / pxlSizeX;
	m_i_dCPixelSizeY = 1. / pxlSizeY;
	
	// JIKim(150910): 미사용 변수 제거
	//m_dLDC[0] = m_dLDC[1] = m_dLDC[2] = m_dLDC[3] = m_dLDC[4] = 0;
	//m_dFx =  m_dFocalLen / pxlSizeX;
	//m_dFy =  m_dFocalLen / pxlSizeY;
	//m_dCx = m_nColSize*0.5 + m_ppsOff_X;
	//m_dCy = m_nRowSize*0.5 + m_ppsOff_Y;

	if(m_nColSize <= 1 || m_nRowSize <= 1) return SM_ERROR;

	return 0;
}


int CFrmCamera::SetCameraPara(double f, double pxlSize, int XSize, int YSize)
{
	m_dFocalLen = f; 
	m_dCPixelSizeX = pxlSize; 
	m_dCPixelSizeY = pxlSize; 
	m_nColSize = XSize; 
	m_nRowSize = YSize;
	m_ppsOff_X = 0;
	m_ppsOff_Y = 0;

	// JIKim(150910): 미사용 변수 제거
	//m_dLDC[0] = m_dLDC[1] = m_dLDC[2] = m_dLDC[3] = m_dLDC[4] = 0;
	//m_dFx =  m_dFocalLen / pxlSizeX;
	//m_dFy =  m_dFocalLen / pxlSizeY;
	//m_dCx = m_nColSize*0.5 + m_ppsOff_X;
	//m_dCy = m_nRowSize*0.5 + m_ppsOff_Y;

	if(m_nColSize <= 1 || m_nRowSize <= 1) return SM_ERROR;

	return 0;
}

int CFrmCamera::SetCameraPara(double f, double PPSOff_X, double PPSOff_Y)
{
	m_dFocalLen = f;

	m_ppsOff_X = PPSOff_X;
	m_ppsOff_Y = PPSOff_Y;

	// JIKim(150910): 미사용 변수 제거
	//m_dLDC[0] = m_dLDC[1] = m_dLDC[2] = m_dLDC[3] = m_dLDC[4] = 0;
	//m_dFx =  m_dFocalLen / pxlSizeX;
	//m_dFy =  m_dFocalLen / pxlSizeY;
	//m_dCx = m_nColSize*0.5 + m_ppsOff_X;
	//m_dCy = m_nRowSize*0.5 + m_ppsOff_Y;

	// SHKim(180517): 미사용 변수 제거
	//m_dCPixelSizeX = pxlSizeX;
	//m_dCPixelSizeY = pxlSizeY;
	//m_nColSize = XSize;
	//m_nRowSize = YSize;
	//m_i_dCPixelSizeX = 1. / pxlSizeX;
	//m_i_dCPixelSizeY = 1. / pxlSizeY;

	if (m_nColSize <= 1 || m_nRowSize <= 1) return SM_ERROR;

	return 0;
}

int CFrmCamera::SetupFCamModel(TCHAR* GPSINAF, fstream& log){ //GPS/INS

	setlocale(LC_ALL, ""); //ifstream 사용하여 파일입력시 한글경로 인식을 위한 설정
	ifstream in;
	in.open(GPSINAF);
	if(in.is_open() == false) return SM_ERROR;

	in >> m_strID >> m_dBx >> m_dBy >> m_dBz >> m_dOmega >> m_dPhi >> m_dKappa;
	in.close();

	log << "GPSINS information file: " << GPSINAF << endl;
	log << "Number of Columns " << this->GetNumCols() << endl;
	log << "Number of Rows " << this->GetNumRows() << endl;
	log << "Focal length: " << m_dFocalLen << endl;

	m_dOmega *= tORAD;
	m_dPhi *= tORAD;
	m_dKappa *= tORAD;

	//m_dBx *= 1000;
	//m_dBy *= 1000;
	//m_dBz *= 1000;

	m_R_image2obj[1][1] = cos(m_dPhi)*cos(m_dKappa);
	m_R_image2obj[1][2] = -cos(m_dPhi)*sin(m_dKappa);
	m_R_image2obj[1][3] = sin(m_dPhi);
	m_R_image2obj[2][1] = sin(m_dOmega)*sin(m_dPhi)*cos(m_dKappa) + cos(m_dOmega)*sin(m_dKappa);
	m_R_image2obj[2][2] = -sin(m_dOmega)*sin(m_dPhi)*sin(m_dKappa) + cos(m_dOmega)*cos(m_dKappa);
	m_R_image2obj[2][3] = -sin(m_dOmega)*cos(m_dPhi);
	m_R_image2obj[3][1] = -cos(m_dOmega)*sin(m_dPhi)*cos(m_dKappa) + sin(m_dOmega)*sin(m_dKappa);
	m_R_image2obj[3][2] = cos(m_dOmega)*sin(m_dPhi)*sin(m_dKappa) + sin(m_dOmega)*cos(m_dKappa);
	m_R_image2obj[3][3] = cos(m_dOmega)*cos(m_dPhi);

	// JIKim(150910): 미사용 변수 제거
	/*m_dX = m_dBx; 
	m_dY = m_dBy;
	m_dZ = m_dBz;
	m_dW = m_dOmega;
	m_dP = m_dPhi;
	m_dK = m_dKappa;*/

	log << "Exterier Parameter : " << endl;
	log << m_strID <<" "<< m_dBx <<" "<< m_dBy <<" "<< m_dBz <<" "
		<< m_dOmega <<" "<< m_dPhi <<" "<< m_dKappa << endl;
		
	return 0;
}

int CFrmCamera::SetupFCamModel(TCHAR* GPSINAF){ //GPS/INS

	setlocale(LC_ALL, ""); //ifstream 사용하여 파일입력시 한글경로 인식을 위한 설정

	ifstream in;
	in.open(GPSINAF);
	if(in.is_open() == false) return SM_ERROR;

	in >> m_strID >> m_dBx >> m_dBy >> m_dBz >> m_dOmega >> m_dPhi >> m_dKappa;

//	if((m_dOmega == (int)(m_dOmega)) || (m_dPhi == (int)(m_dPhi)) || (m_dKappa == (int)(m_dKappa))) {
//		return SM_ERROR;		
//	}

	in.close();

	m_dOmega *= tORAD;
	m_dPhi *= tORAD;
	m_dKappa *= tORAD;

	m_R_image2obj[1][1] = cos(m_dPhi)*cos(m_dKappa);
	m_R_image2obj[1][2] = -cos(m_dPhi)*sin(m_dKappa);
	m_R_image2obj[1][3] = sin(m_dPhi);
	m_R_image2obj[2][1] = sin(m_dOmega)*sin(m_dPhi)*cos(m_dKappa) + cos(m_dOmega)*sin(m_dKappa);
	m_R_image2obj[2][2] = -sin(m_dOmega)*sin(m_dPhi)*sin(m_dKappa) + cos(m_dOmega)*cos(m_dKappa);
	m_R_image2obj[2][3] = -sin(m_dOmega)*cos(m_dPhi);
	m_R_image2obj[3][1] = -cos(m_dOmega)*sin(m_dPhi)*cos(m_dKappa) + sin(m_dOmega)*sin(m_dKappa);
	m_R_image2obj[3][2] = cos(m_dOmega)*sin(m_dPhi)*sin(m_dKappa) + sin(m_dOmega)*cos(m_dKappa);
	m_R_image2obj[3][3] = cos(m_dOmega)*cos(m_dPhi);

	return 0;
}

// Set EOP of an image.
// Note that Omega, Phi, Kappa are angles from Object to Image
int CFrmCamera::SetupFCamModel(double Bx, double By, double Bz, double Omega_o2i_Rad, double Phi_o2i_Rad, double Kappa_o2i_Rad){ //GPS/INS

	m_dBx = Bx; m_dBy = By; m_dBz = Bz; m_dOmega = Omega_o2i_Rad; m_dPhi = Phi_o2i_Rad; m_dKappa = Kappa_o2i_Rad;

	m_dOmega *= tORAD;
	m_dPhi *= tORAD;
	m_dKappa *= tORAD;

	m_R_image2obj[1][1] = cos(m_dPhi)*cos(m_dKappa);
	m_R_image2obj[1][2] = -cos(m_dPhi)*sin(m_dKappa);
	m_R_image2obj[1][3] = sin(m_dPhi);
	m_R_image2obj[2][1] = sin(m_dOmega)*sin(m_dPhi)*cos(m_dKappa) + cos(m_dOmega)*sin(m_dKappa);
	m_R_image2obj[2][2] = -sin(m_dOmega)*sin(m_dPhi)*sin(m_dKappa) + cos(m_dOmega)*cos(m_dKappa);
	m_R_image2obj[2][3] = -sin(m_dOmega)*cos(m_dPhi);
	m_R_image2obj[3][1] = -cos(m_dOmega)*sin(m_dPhi)*cos(m_dKappa) + sin(m_dOmega)*sin(m_dKappa);
	m_R_image2obj[3][2] = cos(m_dOmega)*sin(m_dPhi)*sin(m_dKappa) + sin(m_dOmega)*cos(m_dKappa);
	m_R_image2obj[3][3] = cos(m_dOmega)*cos(m_dPhi);

	return 0;
}

// 180809 - Revised By TJKim
// Inha Model: omega, phi, kappa : from Object to Image
// Rotaiton sequence: 1-2-3 system. omega --> phi --> kappa
// Total R_obi2image = Rz(kappa) * Ry(phi) * Rx(omega)
// Rotation matrix m_adRm is from Ditector-->Object Space.
// m_adRm = R_obj2image ^ T = R_image2obj = Rx(-omega) * Ry(-phi) * Rz(-kappa)
// For historic reason, the following Euler rotation matrix is applied to calculate a_adRm.
// Rx(-omega) = 1     0    0                Ry(-phi) =  cp    0   sp     Rz(-k) =  ck   -sk    0
//              0     cW   -sW                          0     1   0                sk    ck    0
//              0     sW   cW                          -sp    0   cp               0     0    1
// 190201 - More Explanation Added
// Formula: Rx(w) = 1   0   0        Ry(p) =  cP  0  -sP		Rz(k) = cK  sK  0
//					0  cW  sW				  0   1   0					-sK cK  0
//					0  -sW cW				  sP  0   cP				 0   0  1
// R_obj2image = Rz(k) Ry(p) Rx(w)
// m_R_image2obj = Rx(-w) Ry(-p) Rz(-k) ==> implemented as follows
// r11 = cosP*cosK;                          r12 = -cosP*sinK;                      r13 = sinP;
// r21 = sinW*sinP*cosK + cosW*sinK;		 r22 = -sinW*sinP*sinK + cosW*cosK;		r23 = -sinW*cosP;
// r31 = -cosW*sinP*cosK + sinW*sinK;		 r32 = cosW*sinP*sinK + sinW*cosK;		r33 = cosW*cosP;
void CFrmCamera::SetEOPara (double X, double Y, double Z, double W_rad, double P_rad, double K_rad ) { // in mm and radian
	m_dBx = X; m_dBy = Y; m_dBz = Z; m_dOmega = W_rad; m_dPhi = P_rad; m_dKappa = K_rad; 

	m_R_image2obj[1][1] = cos(m_dPhi)*cos(m_dKappa);
	m_R_image2obj[1][2] = -cos(m_dPhi)*sin(m_dKappa);
	m_R_image2obj[1][3] = sin(m_dPhi);
	m_R_image2obj[2][1] = sin(m_dOmega)*sin(m_dPhi)*cos(m_dKappa) + cos(m_dOmega)*sin(m_dKappa);
	m_R_image2obj[2][2] = -sin(m_dOmega)*sin(m_dPhi)*sin(m_dKappa) + cos(m_dOmega)*cos(m_dKappa);
	m_R_image2obj[2][3] = -sin(m_dOmega)*cos(m_dPhi);
	m_R_image2obj[3][1] = -cos(m_dOmega)*sin(m_dPhi)*cos(m_dKappa) + sin(m_dOmega)*sin(m_dKappa);
	m_R_image2obj[3][2] = cos(m_dOmega)*sin(m_dPhi)*sin(m_dKappa) + sin(m_dOmega)*cos(m_dKappa);
	m_R_image2obj[3][3] = cos(m_dOmega)*cos(m_dPhi);

}


int CFrmCamera::SetupPrcsnFCamModel(vector<GCP> & ModelGCP, fstream& log) {

	//	cout << "GCPNo = " << ModelGCP.Length() << "\n";

	if (ModelGCP.size() < 4) return false;

	int ParameterNum = 6;
	int ObservationNum = 2 * ModelGCP.size();

	double col, row, iX, iY, iZ, xx, yy;
	double* Re_L_matrix = new double[ObservationNum];
	double* Re_A_matrix = new double[ParameterNum * ObservationNum];
	double* Re_X_matrix = new double[ParameterNum]; 			// By, Bz, w, p, k
	double* Covariance = new double[ParameterNum * ParameterNum];

	double omega = m_dOmega;
	double phi = m_dPhi;
	double kappa = m_dKappa;
	double Xs = m_dBx;
	double Ys = m_dBy;
	double Zs = m_dBz;
	double zz = m_dFocalLen;

	double sinW, cosW, sinP, cosP, sinK, cosK;
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;

	double dr11_dW, dr12_dW, dr13_dW, dr21_dW, dr22_dW, dr23_dW, dr31_dW, dr32_dW, dr33_dW;
	double dr11_dP, dr12_dP, dr13_dP, dr21_dP, dr22_dP, dr23_dP, dr31_dP, dr32_dP, dr33_dP;
	double dr11_dK, dr12_dK, dr13_dK, dr21_dK, dr22_dK, dr23_dK, dr31_dK, dr32_dK, dr33_dK;

	double Rxx, Ryy, Rzz;
	double dRxx_dW, dRxx_dP, dRxx_dK, dRxx_dXs, dRxx_dYs, dRxx_dZs;
	double dRyy_dW, dRyy_dP, dRyy_dK, dRyy_dXs, dRyy_dYs, dRyy_dZs;
	double dRzz_dW, dRzz_dP, dRzz_dK, dRzz_dXs, dRzz_dYs, dRzz_dZs;

	double F1, F2;
	double dF1_dW, dF1_dP, dF1_dK, dF1_dXs, dF1_dYs, dF1_dZs;
	double dF2_dW, dF2_dP, dF2_dK, dF2_dXs, dF2_dYs, dF2_dZs;

	int LoopNumber = 0;
	for (int Loop = 0, LoopNumber = 0; Loop < 200; Loop++, LoopNumber++)
	{
		cosW = cos(omega);	sinW = sin(omega);
		cosP = cos(phi);	sinP = sin(phi);
		cosK = cos(kappa);	sinK = sin(kappa);

		r11 = cosP*cosK;                        r12 = -cosP*sinK;                      r13 = sinP;
		r21 = sinW*sinP*cosK + cosW*sinK;		 r22 = -sinW*sinP*sinK + cosW*cosK;		r23 = -sinW*cosP;
		r31 = -cosW*sinP*cosK + sinW*sinK;		 r32 = cosW*sinP*sinK + sinW*cosK;		r33 = cosW*cosP;

		dr11_dW = 0.;					   dr12_dW = 0.;						dr13_dW = 0.;
		dr21_dW = -r31;					   dr22_dW = -r32;						dr23_dW = -r33;
		dr31_dW = r21;					   dr32_dW = r22;						dr33_dW = r23;

		dr11_dP = -sinP*cosK;			   dr12_dP = sinP*sinK;			    dr13_dP = cosP;
		dr21_dP = sinW*cosP*cosK;		   dr22_dP = -sinW*cosP*sinK;			dr23_dP = sinW*sinP;
		dr31_dP = -cosW*cosP*cosK;		   dr32_dP = cosW*cosP*sinK;			dr33_dP = -cosW*sinP;

		dr11_dK = r12;					   dr12_dK = -r11;					    dr13_dK = 0.;
		dr21_dK = r22;					   dr22_dK = -r21;					    dr23_dK = 0.;
		dr31_dK = r32;					   dr32_dK = -r31;					    dr33_dK = 0.;

		for (int i = 0; i < ModelGCP.size(); i++)
		{
			// 정합점의 사진좌표(주점보정)
			col = ModelGCP[i].col;
			row = ModelGCP[i].row; 
			iX = ModelGCP[i].XX;
			iY = ModelGCP[i].YY;
			iZ = ModelGCP[i].ZZ;
			Image2Cam(col, row, xx, yy);

			Rxx = r11 * (iX - Xs) + r21 * (iY - Ys) + r31 * (iZ - Zs);
			Ryy = r12 * (iX - Xs) + r22 * (iY - Ys) + r32 * (iZ - Zs);
			Rzz = r13 * (iX - Xs) + r23 * (iY - Ys) + r33 * (iZ - Zs);

			dRxx_dW = dr11_dW * (iX - Xs) + dr21_dW * (iY - Ys) + dr31_dW * (iZ - Zs);
			dRyy_dW = dr12_dW * (iX - Xs) + dr22_dW * (iY - Ys) + dr32_dW * (iZ - Zs);
			dRzz_dW = dr13_dW * (iX - Xs) + dr23_dW * (iY - Ys) + dr33_dW * (iZ - Zs);

			dRxx_dP = dr11_dP * (iX - Xs) + dr21_dP * (iY - Ys) + dr31_dP * (iZ - Zs);
			dRyy_dP = dr12_dP * (iX - Xs) + dr22_dP * (iY - Ys) + dr32_dP * (iZ - Zs);
			dRzz_dP = dr13_dP * (iX - Xs) + dr23_dP * (iY - Ys) + dr33_dP * (iZ - Zs);

			dRxx_dK = dr11_dK * (iX - Xs) + dr21_dK * (iY - Ys) + dr31_dK * (iZ - Zs);
			dRyy_dK = dr12_dK * (iX - Xs) + dr22_dK * (iY - Ys) + dr32_dK * (iZ - Zs);
			dRzz_dK = dr13_dK * (iX - Xs) + dr23_dK * (iY - Ys) + dr33_dK * (iZ - Zs);

			dRxx_dXs = -r11;
			dRyy_dXs = -r12;
			dRzz_dXs = -r13;

			dRxx_dYs = -r21;
			dRyy_dYs = -r22;
			dRzz_dYs = -r23;

			dRxx_dZs = -r31;
			dRyy_dZs = -r32;
			dRzz_dZs = -r33;

			F1 = xx + zz * Rxx / Rzz;
			F2 = yy + zz * Ryy / Rzz;

			dF1_dW = zz * (dRxx_dW * Rzz - Rxx * dRzz_dW) / (Rzz*Rzz);
			dF1_dP = zz * (dRxx_dP * Rzz - Rxx * dRzz_dP) / (Rzz*Rzz);
			dF1_dK = zz * (dRxx_dK * Rzz - Rxx * dRzz_dK) / (Rzz*Rzz);
			dF1_dXs = zz * (dRxx_dXs* Rzz - Rxx * dRzz_dXs) / (Rzz*Rzz);
			dF1_dYs = zz * (dRxx_dYs* Rzz - Rxx * dRzz_dYs) / (Rzz*Rzz);
			dF1_dZs = zz * (dRxx_dZs* Rzz - Rxx * dRzz_dZs) / (Rzz*Rzz);

			dF2_dW = zz * (dRyy_dW * Rzz - Ryy * dRzz_dW) / (Rzz*Rzz);
			dF2_dP = zz * (dRyy_dP * Rzz - Ryy * dRzz_dP) / (Rzz*Rzz);
			dF2_dK = zz * (dRyy_dK * Rzz - Ryy * dRzz_dK) / (Rzz*Rzz);
			dF2_dXs = zz * (dRyy_dXs* Rzz - Ryy * dRzz_dXs) / (Rzz*Rzz);
			dF2_dYs = zz * (dRyy_dYs* Rzz - Ryy * dRzz_dYs) / (Rzz*Rzz);
			dF2_dZs = zz * (dRyy_dZs* Rzz - Ryy * dRzz_dZs) / (Rzz*Rzz);

			Re_L_matrix[2 * i + 0] = -F1;
			Re_L_matrix[2 * i + 1] = -F2;

			Re_A_matrix[i * 6 * 2 + 0] = dF1_dXs;
			Re_A_matrix[i * 6 * 2 + 1] = dF1_dYs;
			Re_A_matrix[i * 6 * 2 + 2] = dF1_dZs;
			Re_A_matrix[i * 6 * 2 + 3] = dF1_dW;
			Re_A_matrix[i * 6 * 2 + 4] = dF1_dP;
			Re_A_matrix[i * 6 * 2 + 5] = dF1_dK;

			Re_A_matrix[i * 6 * 2 + 6] = dF2_dXs;
			Re_A_matrix[i * 6 * 2 + 7] = dF2_dYs;
			Re_A_matrix[i * 6 * 2 + 8] = dF2_dZs;
			Re_A_matrix[i * 6 * 2 + 9] = dF2_dW;
			Re_A_matrix[i * 6 * 2 + 10] = dF2_dP;
			Re_A_matrix[i * 6 * 2 + 11] = dF2_dK;

		}

		double SquareofMisclosure = 0;
		for (int i = 0; i < ObservationNum; i++) SquareofMisclosure += Re_L_matrix[i] * Re_L_matrix[i];
		log << "In Loop " << SquareofMisclosure << " " << Xs << " " << Ys << " " << Zs << " " << omega << " " << phi << " " << kappa << "\n";
		cout << "In Loop " << SquareofMisclosure << " " << Xs << " " << Ys << " " << Zs << " " << omega << " " << phi << " " << kappa << "\n";

		LSE(Re_A_matrix, Re_X_matrix, Re_L_matrix, ParameterNum, ObservationNum, Covariance);//최소제곱법 적용

		Xs = Xs + Re_X_matrix[0];
		Ys = Ys + Re_X_matrix[1];
		Zs = Zs + Re_X_matrix[2];
		omega = omega + Re_X_matrix[3];
		phi = phi + Re_X_matrix[4];
		kappa = kappa + Re_X_matrix[5];

		if (fabs(Re_X_matrix[0]) < 0.001 && fabs(Re_X_matrix[1]) < 0.001 && fabs(Re_X_matrix[2]) < 0.001 &&
			fabs(Re_X_matrix[3]) < 0.00001 && fabs(Re_X_matrix[4]) < 0.00001 && fabs(Re_X_matrix[5]) < 0.00001)  Loop = 400;
		if (!_finite(Xs) || !_finite(Ys) || !_finite(Zs) || !_finite(omega) || !_finite(phi) || !_finite(kappa)) break;
	}

	//double ToDeg = 180. / acos(-1.);
	//cout << "Adjustment Value " << Xs - m_dBx << " " << Ys - m_dBy << " " << Zs - m_dBz << " "
	//	<< ToDeg * (omega - m_dOmega) << " " << ToDeg * (phi - m_dPhi) << " " << ToDeg * (kappa - m_dKappa) << "\n";

	delete[] Re_X_matrix;
	delete[] Re_A_matrix;
	delete[] Re_L_matrix;
	delete[] Covariance;

	if (LoopNumber >= 200) return false;
	if (!_finite(Xs) || !_finite(Ys) || !_finite(Zs) || !_finite(omega) || !_finite(phi) || !_finite(kappa)) return false;

	SetEOPara(Xs, Ys, Zs, omega, phi, kappa);

	return true;
}

int CFrmCamera::SetupPrcsnFCamModel(vector<GCP> & ModelGCP, vector<double>& Weight, fstream& log) {

	//	cout << "GCPNo = " << ModelGCP.Length() << "\n";

	if (ModelGCP.size() < 4) return false;

	int ParameterNum = 6;
	int ObservationNum = 2 * ModelGCP.size();

	double col, row, iX, iY, iZ, xx, yy;
	double* Re_L_matrix = new double[ObservationNum];
	double* Re_A_matrix = new double[ParameterNum * ObservationNum];
	double* Re_W_matrix = new double[ObservationNum * ObservationNum];
	double* Re_X_matrix = new double[ParameterNum]; 			// By, Bz, w, p, k
	double* Covariance = new double[ParameterNum * ParameterNum];

	double omega = m_dOmega;
	double phi = m_dPhi;
	double kappa = m_dKappa;
	double Xs = m_dBx;
	double Ys = m_dBy;
	double Zs = m_dBz;
	double zz = m_dFocalLen;

	double sinW, cosW, sinP, cosP, sinK, cosK;
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;

	double dr11_dW, dr12_dW, dr13_dW, dr21_dW, dr22_dW, dr23_dW, dr31_dW, dr32_dW, dr33_dW;
	double dr11_dP, dr12_dP, dr13_dP, dr21_dP, dr22_dP, dr23_dP, dr31_dP, dr32_dP, dr33_dP;
	double dr11_dK, dr12_dK, dr13_dK, dr21_dK, dr22_dK, dr23_dK, dr31_dK, dr32_dK, dr33_dK;

	double Rxx, Ryy, Rzz;
	double dRxx_dW, dRxx_dP, dRxx_dK, dRxx_dXs, dRxx_dYs, dRxx_dZs;
	double dRyy_dW, dRyy_dP, dRyy_dK, dRyy_dXs, dRyy_dYs, dRyy_dZs;
	double dRzz_dW, dRzz_dP, dRzz_dK, dRzz_dXs, dRzz_dYs, dRzz_dZs;

	double F1, F2;
	double dF1_dW, dF1_dP, dF1_dK, dF1_dXs, dF1_dYs, dF1_dZs;
	double dF2_dW, dF2_dP, dF2_dK, dF2_dXs, dF2_dYs, dF2_dZs;

	for (int i = 0; i < ObservationNum; i++)
	for (int j = 0; j < ObservationNum; j++) {
		Re_W_matrix[i*ObservationNum + j] = 0;
		if (i == j && i < Weight.size()) Re_W_matrix[i*ObservationNum + j] = Weight[i];
	}

	int LoopNumber = 0;
	for (int Loop = 0, LoopNumber = 0; Loop < 200; Loop++, LoopNumber++)
	{
		cosW = cos(omega);	sinW = sin(omega);
		cosP = cos(phi);	sinP = sin(phi);
		cosK = cos(kappa);	sinK = sin(kappa);

		r11 = cosP*cosK;                        r12 = -cosP*sinK;                      r13 = sinP;
		r21 = sinW*sinP*cosK + cosW*sinK;		 r22 = -sinW*sinP*sinK + cosW*cosK;		r23 = -sinW*cosP;
		r31 = -cosW*sinP*cosK + sinW*sinK;		 r32 = cosW*sinP*sinK + sinW*cosK;		r33 = cosW*cosP;

		dr11_dW = 0.;					   dr12_dW = 0.;						dr13_dW = 0.;
		dr21_dW = -r31;					   dr22_dW = -r32;						dr23_dW = -r33;
		dr31_dW = r21;					   dr32_dW = r22;						dr33_dW = r23;

		dr11_dP = -sinP*cosK;			   dr12_dP = sinP*sinK;			    dr13_dP = cosP;
		dr21_dP = sinW*cosP*cosK;		   dr22_dP = -sinW*cosP*sinK;			dr23_dP = sinW*sinP;
		dr31_dP = -cosW*cosP*cosK;		   dr32_dP = cosW*cosP*sinK;			dr33_dP = -cosW*sinP;

		dr11_dK = r12;					   dr12_dK = -r11;					    dr13_dK = 0.;
		dr21_dK = r22;					   dr22_dK = -r21;					    dr23_dK = 0.;
		dr31_dK = r32;					   dr32_dK = -r31;					    dr33_dK = 0.;

		for (int i = 0; i < ModelGCP.size(); i++)
		{
			// 정합점의 사진좌표(주점보정)
			col = ModelGCP[i].col;
			row = ModelGCP[i].row;
			iX = ModelGCP[i].XX;
			iY = ModelGCP[i].YY;
			iZ = ModelGCP[i].ZZ;
			Image2Cam(col, row, xx, yy);

			Rxx = r11 * (iX - Xs) + r21 * (iY - Ys) + r31 * (iZ - Zs);
			Ryy = r12 * (iX - Xs) + r22 * (iY - Ys) + r32 * (iZ - Zs);
			Rzz = r13 * (iX - Xs) + r23 * (iY - Ys) + r33 * (iZ - Zs);

			dRxx_dW = dr11_dW * (iX - Xs) + dr21_dW * (iY - Ys) + dr31_dW * (iZ - Zs);
			dRyy_dW = dr12_dW * (iX - Xs) + dr22_dW * (iY - Ys) + dr32_dW * (iZ - Zs);
			dRzz_dW = dr13_dW * (iX - Xs) + dr23_dW * (iY - Ys) + dr33_dW * (iZ - Zs);

			dRxx_dP = dr11_dP * (iX - Xs) + dr21_dP * (iY - Ys) + dr31_dP * (iZ - Zs);
			dRyy_dP = dr12_dP * (iX - Xs) + dr22_dP * (iY - Ys) + dr32_dP * (iZ - Zs);
			dRzz_dP = dr13_dP * (iX - Xs) + dr23_dP * (iY - Ys) + dr33_dP * (iZ - Zs);

			dRxx_dK = dr11_dK * (iX - Xs) + dr21_dK * (iY - Ys) + dr31_dK * (iZ - Zs);
			dRyy_dK = dr12_dK * (iX - Xs) + dr22_dK * (iY - Ys) + dr32_dK * (iZ - Zs);
			dRzz_dK = dr13_dK * (iX - Xs) + dr23_dK * (iY - Ys) + dr33_dK * (iZ - Zs);

			dRxx_dXs = -r11;
			dRyy_dXs = -r12;
			dRzz_dXs = -r13;

			dRxx_dYs = -r21;
			dRyy_dYs = -r22;
			dRzz_dYs = -r23;

			dRxx_dZs = -r31;
			dRyy_dZs = -r32;
			dRzz_dZs = -r33;

			F1 = xx + zz * Rxx / Rzz;
			F2 = yy + zz * Ryy / Rzz;

			dF1_dW = zz * (dRxx_dW * Rzz - Rxx * dRzz_dW) / (Rzz*Rzz);
			dF1_dP = zz * (dRxx_dP * Rzz - Rxx * dRzz_dP) / (Rzz*Rzz);
			dF1_dK = zz * (dRxx_dK * Rzz - Rxx * dRzz_dK) / (Rzz*Rzz);
			dF1_dXs = zz * (dRxx_dXs* Rzz - Rxx * dRzz_dXs) / (Rzz*Rzz);
			dF1_dYs = zz * (dRxx_dYs* Rzz - Rxx * dRzz_dYs) / (Rzz*Rzz);
			dF1_dZs = zz * (dRxx_dZs* Rzz - Rxx * dRzz_dZs) / (Rzz*Rzz);

			dF2_dW = zz * (dRyy_dW * Rzz - Ryy * dRzz_dW) / (Rzz*Rzz);
			dF2_dP = zz * (dRyy_dP * Rzz - Ryy * dRzz_dP) / (Rzz*Rzz);
			dF2_dK = zz * (dRyy_dK * Rzz - Ryy * dRzz_dK) / (Rzz*Rzz);
			dF2_dXs = zz * (dRyy_dXs* Rzz - Ryy * dRzz_dXs) / (Rzz*Rzz);
			dF2_dYs = zz * (dRyy_dYs* Rzz - Ryy * dRzz_dYs) / (Rzz*Rzz);
			dF2_dZs = zz * (dRyy_dZs* Rzz - Ryy * dRzz_dZs) / (Rzz*Rzz);

			Re_L_matrix[2 * i + 0] = -F1;
			Re_L_matrix[2 * i + 1] = -F2;

			Re_A_matrix[i * 6 * 2 + 0] = dF1_dXs;
			Re_A_matrix[i * 6 * 2 + 1] = dF1_dYs;
			Re_A_matrix[i * 6 * 2 + 2] = dF1_dZs;
			Re_A_matrix[i * 6 * 2 + 3] = dF1_dW;
			Re_A_matrix[i * 6 * 2 + 4] = dF1_dP;
			Re_A_matrix[i * 6 * 2 + 5] = dF1_dK;

			Re_A_matrix[i * 6 * 2 + 6] = dF2_dXs;
			Re_A_matrix[i * 6 * 2 + 7] = dF2_dYs;
			Re_A_matrix[i * 6 * 2 + 8] = dF2_dZs;
			Re_A_matrix[i * 6 * 2 + 9] = dF2_dW;
			Re_A_matrix[i * 6 * 2 + 10] = dF2_dP;
			Re_A_matrix[i * 6 * 2 + 11] = dF2_dK;

		}

		double SquareofMisclosure = 0;
		for (int i = 0; i < ObservationNum; i++) SquareofMisclosure += Re_L_matrix[i] * Re_L_matrix[i];
		//log << "In Loop " << SquareofMisclosure << " " << Xs << " " << Ys << " " << Zs << " " << omega << " " << phi << " " << kappa << "\n";
		//cout << "In Loop " << SquareofMisclosure << " " << Xs << " " << Ys << " " << Zs << " " << omega << " " << phi << " " << kappa << "\n";

		LSE(Re_A_matrix, Re_X_matrix, Re_L_matrix, Re_W_matrix, ParameterNum, ObservationNum);//최소제곱법 적용

		Xs = Xs + Re_X_matrix[0];
		Ys = Ys + Re_X_matrix[1];
		Zs = Zs + Re_X_matrix[2];
		omega = omega + Re_X_matrix[3];
		phi = phi + Re_X_matrix[4];
		kappa = kappa + Re_X_matrix[5];

		if (fabs(Re_X_matrix[0]) < 0.001 && fabs(Re_X_matrix[1]) < 0.001 && fabs(Re_X_matrix[2]) < 0.001 &&
			fabs(Re_X_matrix[3]) < 0.00001 && fabs(Re_X_matrix[4]) < 0.00001 && fabs(Re_X_matrix[5]) < 0.00001)  Loop = 400;
		if (!_finite(Xs) || !_finite(Ys) || !_finite(Zs) || !_finite(omega) || !_finite(phi) || !_finite(kappa)) break;
	}

	//double ToDeg = 180. / acos(-1.);
	//cout << "Adjustment Value " << Xs - m_dBx << " " << Ys - m_dBy << " " << Zs - m_dBz << " "
	//	<< ToDeg * (omega - m_dOmega) << " " << ToDeg * (phi - m_dPhi) << " " << ToDeg * (kappa - m_dKappa) << "\n";

	delete[] Re_X_matrix;
	delete[] Re_A_matrix;
	delete[] Re_W_matrix;
	delete[] Re_L_matrix;
	delete[] Covariance;

	if (LoopNumber >= 200) return false;
	if (!_finite(Xs) || !_finite(Ys) || !_finite(Zs) || !_finite(omega) || !_finite(phi) || !_finite(kappa)) return false;

	SetEOPara(Xs, Ys, Zs, omega, phi, kappa);

	return true;
}


int CFrmCamera::SetupPrcsnFCamModel(vector<GCP> & ModelGCP, double& RMSErrorinPixel, fstream& log) {

	int ReturnValue = SetupPrcsnFCamModel(ModelGCP, log);
	if (ReturnValue == false) return false;

	double lcol, lrow, lcol2, lrow2, XX, YY, ZZ;
	double ErrorSum = 0., ErrorSquare = 0., Error;
	for (int i = 0; i < ModelGCP.size(); i++) {
		lcol = ModelGCP[i].col;
		lrow = ModelGCP[i].row;
		XX = ModelGCP[i].XX;
		YY = ModelGCP[i].YY;
		ZZ = ModelGCP[i].ZZ;
		InverseMapping(XX, YY, ZZ, lcol2, lrow2);
		Error = sqrt((lcol - lcol2)*(lcol - lcol2) + (lrow - lrow2) * (lrow - lrow2));
		ErrorSum += Error;
		ErrorSquare += Error*Error;
	}
	RMSErrorinPixel = sqrt(ErrorSquare / ModelGCP.size());

	return true;
}

int CFrmCamera::SetupPrcsnFCamModel(vector<GCP> & ModelGCP, vector<double>& Weight, double& RMSErrorinPixel, fstream& log) {

	int ReturnValue = SetupPrcsnFCamModel(ModelGCP, Weight, log);
	if (ReturnValue == false) return false;

	double lcol, lrow, lcol2, lrow2, XX, YY, ZZ;
	double ErrorSum = 0., ErrorSquare = 0., Error;
	for (int i = 0; i < ModelGCP.size(); i++) {
		lcol = ModelGCP[i].col;
		lrow = ModelGCP[i].row;
		XX = ModelGCP[i].XX;
		YY = ModelGCP[i].YY;
		ZZ = ModelGCP[i].ZZ;
		InverseMapping(XX, YY, ZZ, lcol2, lrow2);
		Error = sqrt((lcol - lcol2)*(lcol - lcol2) + (lrow - lrow2) * (lrow - lrow2));
		ErrorSum += Error;
		ErrorSquare += Error*Error;
	}
	RMSErrorinPixel = sqrt(ErrorSquare / ModelGCP.size());

	return true;
}

int CFrmCamera::SetupPrcsnFCamModel_byCL(vector<GCP> & ModelGCP, fstream& log) {

	//	cout << "GCPNo = " << ModelGCP.Length() << "\n";

	if (ModelGCP.size() < 4) return false;

	int ParameterNum = 9;
	int ObservationNum = 2 * ModelGCP.size();

	double col, row, iX, iY, iZ, xx, yy;
	double* Re_L_matrix = new double[ObservationNum];
	double* Re_A_matrix = new double[ParameterNum * ObservationNum];
	double* Re_X_matrix = new double[ParameterNum]; 			// Xs, Ys, Zs, w, p, k, fs, ppsOff_X, ppsOff_Y
	double* Covariance = new double[ParameterNum * ParameterNum];

	double omega = m_dOmega;
	double phi = m_dPhi;
	double kappa = m_dKappa;
	double Xs = m_dBx;
	double Ys = m_dBy;
	double Zs = m_dBz;
	double zz = m_dFocalLen;
	double dPixelX = m_dCPixelSizeX;
	double dPixelY = m_dCPixelSizeY;
	double CenterX = m_nColSize / 2.;
	double CenterY = m_nRowSize / 2.;
	double ppsOff_X = 0;
	double ppsOff_Y = 0;

	if ((Xs == 0) || (Ys == 0) || (Zs == 0)) {
		printf("SetupPrcsnFCamModel_byCL() 함수를 실행하기 위해서는 초기 EO값의 설정이 필요합니다.\n");
		printf("예: 입력된 GCP의 중심점(X, Y) 및 센서 위치의 대략적인 높이값(Z)\n");

		return false;
	}

	if (zz == 0) {
		printf("SetupPrcsnFCamModel_byCL() 함수를 실행하기 위해서는 초기 IO값의 설정이 필요합니다.\n");
		printf("예: 추정하려는 센서의 스펙상의 초점거리(fs) 및 주점의 위치 오프셋(ppsOff_X = 0, ppsOff_Y = 0\n");

		return false;
	}

	double sinW, cosW, sinP, cosP, sinK, cosK;
	double r11, r12, r13, r21, r22, r23, r31, r32, r33;

	double dr11_dW, dr12_dW, dr13_dW, dr21_dW, dr22_dW, dr23_dW, dr31_dW, dr32_dW, dr33_dW;
	double dr11_dP, dr12_dP, dr13_dP, dr21_dP, dr22_dP, dr23_dP, dr31_dP, dr32_dP, dr33_dP;
	double dr11_dK, dr12_dK, dr13_dK, dr21_dK, dr22_dK, dr23_dK, dr31_dK, dr32_dK, dr33_dK;

	double Rxx, Ryy, Rzz;
	double dRxx_dW, dRxx_dP, dRxx_dK, dRxx_dXs, dRxx_dYs, dRxx_dZs;
	double dRyy_dW, dRyy_dP, dRyy_dK, dRyy_dXs, dRyy_dYs, dRyy_dZs;
	double dRzz_dW, dRzz_dP, dRzz_dK, dRzz_dXs, dRzz_dYs, dRzz_dZs;

	double F1, F2;
	double dF1_dW, dF1_dP, dF1_dK, dF1_dXs, dF1_dYs, dF1_dZs, dF1_dzz, dF1_dppsOff_X, dF1_dppsOff_Y;
	double dF2_dW, dF2_dP, dF2_dK, dF2_dXs, dF2_dYs, dF2_dZs, dF2_dzz, dF2_dppsOff_X, dF2_dppsOff_Y;

	int LoopNumber = 0;
	for (int Loop = 0, LoopNumber = 0; Loop < 200; Loop++, LoopNumber++)
	{
		cosW = cos(omega);	sinW = sin(omega);
		cosP = cos(phi);	sinP = sin(phi);
		cosK = cos(kappa);	sinK = sin(kappa);

		r11 = cosP*cosK;                        r12 = -cosP*sinK;                      r13 = sinP;
		r21 = sinW*sinP*cosK + cosW*sinK;		 r22 = -sinW*sinP*sinK + cosW*cosK;		r23 = -sinW*cosP;
		r31 = -cosW*sinP*cosK + sinW*sinK;		 r32 = cosW*sinP*sinK + sinW*cosK;		r33 = cosW*cosP;

		dr11_dW = 0.;					   dr12_dW = 0.;						dr13_dW = 0.;
		dr21_dW = -r31;					   dr22_dW = -r32;						dr23_dW = -r33;
		dr31_dW = r21;					   dr32_dW = r22;						dr33_dW = r23;

		dr11_dP = -sinP*cosK;			   dr12_dP = sinP*sinK;			    dr13_dP = cosP;
		dr21_dP = sinW*cosP*cosK;		   dr22_dP = -sinW*cosP*sinK;			dr23_dP = sinW*sinP;
		dr31_dP = -cosW*cosP*cosK;		   dr32_dP = cosW*cosP*sinK;			dr33_dP = -cosW*sinP;

		dr11_dK = r12;					   dr12_dK = -r11;					    dr13_dK = 0.;
		dr21_dK = r22;					   dr22_dK = -r21;					    dr23_dK = 0.;
		dr31_dK = r32;					   dr32_dK = -r31;					    dr33_dK = 0.;

		for (int i = 0; i < ModelGCP.size(); i++)
		{
			// 정합점의 사진좌표(주점보정)
			col = ModelGCP[i].col;
			row = ModelGCP[i].row;
			iX = ModelGCP[i].XX;
			iY = ModelGCP[i].YY;
			iZ = ModelGCP[i].ZZ;

			//Image2Cam(col, row, xx, yy);
			//xx = xx - ppsOff_X;
			//yy = ppsOff_Y + yy;
			xx = dPixelX * (col - CenterX) - ppsOff_X;
			yy = -dPixelY * (row - CenterY) + ppsOff_Y;

			Rxx = r11 * (iX - Xs) + r21 * (iY - Ys) + r31 * (iZ - Zs);
			Ryy = r12 * (iX - Xs) + r22 * (iY - Ys) + r32 * (iZ - Zs);
			Rzz = r13 * (iX - Xs) + r23 * (iY - Ys) + r33 * (iZ - Zs);

			dRxx_dW = dr11_dW * (iX - Xs) + dr21_dW * (iY - Ys) + dr31_dW * (iZ - Zs);
			dRyy_dW = dr12_dW * (iX - Xs) + dr22_dW * (iY - Ys) + dr32_dW * (iZ - Zs);
			dRzz_dW = dr13_dW * (iX - Xs) + dr23_dW * (iY - Ys) + dr33_dW * (iZ - Zs);

			dRxx_dP = dr11_dP * (iX - Xs) + dr21_dP * (iY - Ys) + dr31_dP * (iZ - Zs);
			dRyy_dP = dr12_dP * (iX - Xs) + dr22_dP * (iY - Ys) + dr32_dP * (iZ - Zs);
			dRzz_dP = dr13_dP * (iX - Xs) + dr23_dP * (iY - Ys) + dr33_dP * (iZ - Zs);

			dRxx_dK = dr11_dK * (iX - Xs) + dr21_dK * (iY - Ys) + dr31_dK * (iZ - Zs);
			dRyy_dK = dr12_dK * (iX - Xs) + dr22_dK * (iY - Ys) + dr32_dK * (iZ - Zs);
			dRzz_dK = dr13_dK * (iX - Xs) + dr23_dK * (iY - Ys) + dr33_dK * (iZ - Zs);

			dRxx_dXs = -r11;
			dRyy_dXs = -r12;
			dRzz_dXs = -r13;

			dRxx_dYs = -r21;
			dRyy_dYs = -r22;
			dRzz_dYs = -r23;

			dRxx_dZs = -r31;
			dRyy_dZs = -r32;
			dRzz_dZs = -r33;

			F1 = xx + zz * Rxx / Rzz;
			F2 = yy + zz * Ryy / Rzz;

			dF1_dW = zz * (dRxx_dW * Rzz - Rxx * dRzz_dW) / (Rzz*Rzz);
			dF1_dP = zz * (dRxx_dP * Rzz - Rxx * dRzz_dP) / (Rzz*Rzz);
			dF1_dK = zz * (dRxx_dK * Rzz - Rxx * dRzz_dK) / (Rzz*Rzz);
			dF1_dXs = zz * (dRxx_dXs* Rzz - Rxx * dRzz_dXs) / (Rzz*Rzz);
			dF1_dYs = zz * (dRxx_dYs* Rzz - Rxx * dRzz_dYs) / (Rzz*Rzz);
			dF1_dZs = zz * (dRxx_dZs* Rzz - Rxx * dRzz_dZs) / (Rzz*Rzz);
			dF1_dzz = Rxx / Rzz;
			dF1_dppsOff_X = -1; 

			dF2_dW = zz * (dRyy_dW * Rzz - Ryy * dRzz_dW) / (Rzz*Rzz);
			dF2_dP = zz * (dRyy_dP * Rzz - Ryy * dRzz_dP) / (Rzz*Rzz);
			dF2_dK = zz * (dRyy_dK * Rzz - Ryy * dRzz_dK) / (Rzz*Rzz);
			dF2_dXs = zz * (dRyy_dXs* Rzz - Ryy * dRzz_dXs) / (Rzz*Rzz);
			dF2_dYs = zz * (dRyy_dYs* Rzz - Ryy * dRzz_dYs) / (Rzz*Rzz);
			dF2_dZs = zz * (dRyy_dZs* Rzz - Ryy * dRzz_dZs) / (Rzz*Rzz);
			dF2_dzz = Ryy / Rzz;
			dF2_dppsOff_Y = 1; 

			Re_L_matrix[2 * i + 0] = -F1;
			Re_L_matrix[2 * i + 1] = -F2;

			Re_A_matrix[i * ParameterNum * 2 + 0] = dF1_dXs;
			Re_A_matrix[i * ParameterNum * 2 + 1] = dF1_dYs;
			Re_A_matrix[i * ParameterNum * 2 + 2] = dF1_dZs;
			Re_A_matrix[i * ParameterNum * 2 + 3] = dF1_dW;
			Re_A_matrix[i * ParameterNum * 2 + 4] = dF1_dP;
			Re_A_matrix[i * ParameterNum * 2 + 5] = dF1_dK;
			Re_A_matrix[i * ParameterNum * 2 + 6] = dF1_dzz;
			Re_A_matrix[i * ParameterNum * 2 + 7] = dF1_dppsOff_X;
			Re_A_matrix[i * ParameterNum * 2 + 8] = 0;

			Re_A_matrix[i * ParameterNum * 2 + 9] = dF2_dXs;
			Re_A_matrix[i * ParameterNum * 2 + 10] = dF2_dYs;
			Re_A_matrix[i * ParameterNum * 2 + 11] = dF2_dZs;
			Re_A_matrix[i * ParameterNum * 2 + 12] = dF2_dW;
			Re_A_matrix[i * ParameterNum * 2 + 13] = dF2_dP;
			Re_A_matrix[i * ParameterNum * 2 + 14] = dF2_dK;
			Re_A_matrix[i * ParameterNum * 2 + 15] = dF2_dzz;
			Re_A_matrix[i * ParameterNum * 2 + 16] = 0;
			Re_A_matrix[i * ParameterNum * 2 + 17] = dF2_dppsOff_Y;

		}

		double SquareofMisclosure = 0;
		for (int i = 0; i < ObservationNum; i++) SquareofMisclosure += Re_L_matrix[i] * Re_L_matrix[i];
		//log << "In Loop " << SquareofMisclosure << " " << Xs << " " << Ys << " " << Zs << " " << omega << " " << phi << " " << kappa << "\n";
		//cout << "In Loop " << SquareofMisclosure << " " << Xs << " " << Ys << " " << Zs << " " << omega << " " << phi << " " << kappa << "\n";

		LSE(Re_A_matrix, Re_X_matrix, Re_L_matrix, ParameterNum, ObservationNum, Covariance);//최소제곱법 적용

		Xs = Xs + Re_X_matrix[0];
		Ys = Ys + Re_X_matrix[1];
		Zs = Zs + Re_X_matrix[2];
		omega = omega + Re_X_matrix[3];
		phi = phi + Re_X_matrix[4];
		kappa = kappa + Re_X_matrix[5];
		zz = zz + Re_X_matrix[6];
		ppsOff_X = ppsOff_X + Re_X_matrix[7];
		ppsOff_Y = ppsOff_Y + Re_X_matrix[8];

		if (fabs(Re_X_matrix[0]) < 0.0001 && fabs(Re_X_matrix[1]) < 0.0001 && fabs(Re_X_matrix[2]) < 0.0001 &&
			fabs(Re_X_matrix[3]) < 0.00001 && fabs(Re_X_matrix[4]) < 0.00001 && fabs(Re_X_matrix[5]) < 0.00001 && fabs(Re_X_matrix[6]) < 0.00001 && fabs(Re_X_matrix[7]) < 0.0000001 && fabs(Re_X_matrix[8]) < 0.0000001)  Loop = 400;
		if (!_finite(Xs) || !_finite(Ys) || !_finite(Zs) || !_finite(omega) || !_finite(phi) || !_finite(kappa) || !_finite(zz) || !_finite(kappa) || !_finite(ppsOff_X) || !_finite(ppsOff_Y)) break;


	}

	//double ToDeg = 180. / acos(-1.);
	//cout << "Adjustment Value " << Xs - m_dBx << " " << Ys - m_dBy << " " << Zs - m_dBz << " "
	//	<< ToDeg * (omega - m_dOmega) << " " << ToDeg * (phi - m_dPhi) << " " << ToDeg * (kappa - m_dKappa) << "\n";

	delete[] Re_X_matrix;
	delete[] Re_A_matrix;
	delete[] Re_L_matrix;
	delete[] Covariance;

	if (LoopNumber >= 200) {
		return false;
	}

	if (!_finite(Xs) || !_finite(Ys) || !_finite(Zs) || !_finite(omega) || !_finite(phi) || !_finite(kappa) || !_finite(zz) || !_finite(ppsOff_X) || !_finite(ppsOff_Y)) return false;
	
	// 주점 위치 오프셋 값의 단위 변환 (meter --> px)
	ppsOff_X = ppsOff_X / m_dCPixelSizeX;
	ppsOff_Y = ppsOff_Y / m_dCPixelSizeY;

	SetCameraPara(zz, ppsOff_X, ppsOff_Y);
	SetEOPara(Xs, Ys, Zs, omega, phi, kappa);

	return true;
}

void CFrmCamera::GenerateOutput( fstream& report ) {

	TCHAR strType[10];
	if(m_eSensorType == _DMC_)
		strcpy(strType, "DMC");
	else if(m_eSensorType == _Rollei_)
		strcpy(strType, "Rollei");
	else if(m_eSensorType == _UltraCam_)
		strcpy(strType, "UltraCam");
	else
		strcpy(strType, "Unknown");

	report << "Sensor Type = Frame Camera< " << strType << ">\n";

	report << "Model Parameters\n";
	report.precision(20);
	report << m_dBx << "\t" << m_dBy << "\t" << m_dBz << "\n";
	report << m_dOmega << "\t" << m_dPhi << "\t" << m_dKappa << "\n";
}

int CFrmCamera::Image2Cam(double col, double row, double &x, double &y){

	if( m_bLensDistortion == true ) return Image2CamLD(col, row, x, y );
	else return Image2CamULD(col, row, x, y );	
}

int CFrmCamera::Image2CamULD(double col, double row, double &x, double &y){

	double C_Col = m_nColSize*0.5 + m_ppsOff_X;
	double C_Row = m_nRowSize*0.5 + m_ppsOff_Y;

	x = m_dCPixelSizeX*(col-C_Col);
	y = m_dCPixelSizeY*(-(row-C_Row));

	return 0;
}

int CFrmCamera::Cam2Image(double x, double y, double &col, double &row){
	
	if( m_bLensDistortion == true ) return Cam2ImageLD( x, y, col, row);
	else return Cam2ImageULD( x, y, col, row);	
}

int CFrmCamera::Cam2ImageULD(double x, double y, double &col, double &row){
	
	double C_Col = m_nColSize*0.5 + m_ppsOff_X;
	double C_Row = m_nRowSize*0.5 + m_ppsOff_Y;

	//col = x / m_dCPixelSizeX + C_Col;
	//row = -y / m_dCPixelSizeY + C_Row;

	col = x * m_i_dCPixelSizeX + C_Col;
	row = -y * m_i_dCPixelSizeY + C_Row;

	return 0;
}

int CFrmCamera::LSE( double *LSE_A, double *LSE_x_hat, double *LSE_l, int ParameterNo, int ObservationNo, double* Covariance)
{

//  Input A = 1D array, A11 A12... A1M A21 A22... A2M... AN1... ANM
   // calculate LSE_B matrix, LSE_B = A'A (A' : Transpose of A)

   int SizeOfMatrix = ParameterNo * ParameterNo;
   double *LSE_B = new double[SizeOfMatrix];
   if(LSE_B == NULL) { cerr << "Fatal error: Fail to allocate memory space\n";
 		   return false; }

   int i;
   for(i = 0; i < SizeOfMatrix; i++) *(LSE_B+i) = 0.;

   int j, k, n, y;
   for(i = 0, n = 0; i < ParameterNo; i++)
      for(j = 0; j < ParameterNo; j++)
	for(k = 0, y = 0; k < ObservationNo; k++, y+= ParameterNo )
	   LSE_B[i*ParameterNo+j] += *(LSE_A+y+i) * *(LSE_A+y+j);

   // Cholesky's decomposition
   // calculate C matrix, CC' = LSE_B
   double *LSE_C = new double[SizeOfMatrix];
   if(LSE_C == NULL) { cerr << "Fatal error: Fail to allocate memory space.\n";
   return false;
   }

   for(i = 0; i < SizeOfMatrix; i++) *(LSE_C+i) = 0.;

   for(i = 0, n = 0; i < ParameterNo; i++, n+=ParameterNo)
      for(j = i, y = n; j < ParameterNo; j++, y+=ParameterNo) {
	if(i == j) {
	   for(k = 0; k <= i-1; k++)
	      *(LSE_C+n+j) -= *(LSE_C+n+k) * *(LSE_C+n+k);
	   *(LSE_C+n+j) += *(LSE_B+n+i);
	   *(LSE_C+n+j) = sqrt( *(LSE_C+n+j) );
        } // end of if(i==j)
	else {
	   for(k = 0; k <= i-1; k++)
	      *(LSE_C+y+i) -= *(LSE_C+n+k) * *(LSE_C+y+k);
	   *(LSE_C+y+i) += *(LSE_B+n+j);
	   *(LSE_C+y+i) /= *(LSE_C+n+i);
	} // end of else (i == j)
      } // end of for -- i, j

   // Solve the matrix using UL decomposition

   double *LSE_Y = new double[ParameterNo];
   if(LSE_Y == NULL) { cerr << "Fatal error: fail to allocate memory space\n";
   return false;
   }
   double *LSE_L = new double[ParameterNo];
   if(LSE_L == NULL) { cerr << "Fatal error: fail to allocate memory space\n";
   return false;
   }

   for(i = 0; i < ParameterNo; i++) *(LSE_L+i) = 0;
   for(i = 0; i < ParameterNo; i++)
      for(j = 0, y = 0; j < ObservationNo; j++, y += ParameterNo)
	*(LSE_L+i) += *(LSE_A+y+i) * *(LSE_l+j);

   for(i = 0, n = 0; i < ParameterNo; i++, n+=ParameterNo) {
      *(LSE_Y+i) = *(LSE_L+i);
      for(j = 0; j <= i-1; j++) *(LSE_Y+i) -= *(LSE_C+n+j) * *(LSE_Y+j);
      *(LSE_Y+i) /= *(LSE_C+n+i);
   } // end of for -- i

   for(i=ParameterNo-1, n=i*ParameterNo; i>=0; i--, n-=ParameterNo) {
      *(LSE_x_hat+i) = *(LSE_Y+i);
      for(j = i+1, y = j*ParameterNo; j < ParameterNo;
		   			j++, y+=ParameterNo) {
	*(LSE_x_hat+i) -= *(LSE_C+y+i) * *(LSE_x_hat+j);
      } // end of for -- j
      *(LSE_x_hat+i) /= *(LSE_C+n+i);
   } // end of for -- i

   //*** calculate inverse of A directly
   //*** LU*INV_A = I, L = C, U = C'
   //*** First set YMat = U*INV_A and find YMat
   //*** then solve U*INV_A = YMat
   double* IMat = new double[ParameterNo*ParameterNo];  // identity matrix
   double* YMat = new double[ParameterNo*ParameterNo];
   double* INV_A = new double[ParameterNo*ParameterNo];
   if(IMat == NULL || YMat == NULL || INV_A == NULL) {
	   cerr << "Fatal error: fail to allocate memory space\n";
	   return false;
   }

   // make IMat as identity matrix
   for(i = 0; i < ParameterNo*ParameterNo; i++) IMat[i] = 0;
   for(i = 0; i < ParameterNo; i++) IMat[i*ParameterNo+i] = 1;

   // calculate YMat using L*YMat = IMat;
   for(k = 0; k < ParameterNo; k++)
   for(i = 0; i < ParameterNo; i++) {
      YMat[i*ParameterNo+k] = IMat[i*ParameterNo+k];
      for(j = 0; j <= i-1; j++) YMat[i*ParameterNo+k] -= LSE_C[i*ParameterNo+j] * YMat[j*ParameterNo+k];
      YMat[i*ParameterNo+k] /= LSE_C[i*ParameterNo+i];
   } // end of for -- i

   // calculate INV_A using U*INV_A = YMAt
   for(k = 0; k < ParameterNo; k++)
   for(i=ParameterNo-1; i>=0; i--) {
      INV_A[i*ParameterNo+k] = YMat[i*ParameterNo+k];
      for(j = i+1; j < ParameterNo; j++) {
		INV_A[i*ParameterNo+k] -= LSE_C[j*ParameterNo+i] * INV_A[j*ParameterNo+k];
      } // end of for -- j
      INV_A[i*ParameterNo+k] /= LSE_C[i*ParameterNo+i];
   } // end of for -- i

   //*** Calculate Residual and save them onto LSE_l
   //*** And GetCovariance Matrix
	double* VMatrix = new double[ObservationNo];

	for(i = 0; i < ObservationNo; i++) {
		VMatrix[i] = 0;
		for(j = 0; j < ParameterNo; j++)
			VMatrix[i] += LSE_A[i*ParameterNo+j]*LSE_x_hat[j];
		VMatrix[i] -= LSE_l[i];
		LSE_l[i] = VMatrix[i];
		//cout << "Residual = " << VMatrix[i] << "\n";
	}

	double Variance = 0;
	for(i = 0; i < ObservationNo; i++) Variance += VMatrix[i]*VMatrix[i];
	Variance /= ObservationNo - ParameterNo;

	//cout << "Total Variance = " << Variance << "\n" << flush;

   for(i = 0; i < ParameterNo*ParameterNo; i++) Covariance[i] = INV_A[i]*Variance;

   delete[] IMat;
   delete[] YMat;
   delete[] INV_A;
   delete[] LSE_C;
   delete[] LSE_Y;
   delete[] LSE_B;
   delete[] LSE_L;
   delete[] VMatrix;

   return 0;  // 0 -- successful estimation

} // end of function -- LSE

//********  Ax = l ***************************************
//********   x = (A^tw A)^-1 A^tw l
//*********Weighted Least Square Estimator 
//*********Updated by Nadigaa based on the code of Prof.Taejung Kim
double CFrmCamera::LSE(double *A, double *x_hat, double *l, double *w, int ParameterNo, int ObservationNo)
{

	//  Input A = 1D array, A11 A12... A1M A21 A22... A2M... AN1... ANM

	// calculate LSE_B = A'W

	int i, j, k, n, y;
	int SizeOfMatrix = ObservationNo * ParameterNo;
	double *LSE_B = new double[SizeOfMatrix];
	if (LSE_B == NULL) cerr << "Fatal error: Fail to allocate memory space\n";

	//   for(i = 0; i < SizeOfMatrix; i++) *(LSE_B+i) = 0.;

	for (i = 0; i < ParameterNo; i++)
	for (j = 0; j < ObservationNo; j++)
		LSE_B[i*ObservationNo + j] = A[j*ParameterNo + i] * w[j*ObservationNo + j];
	//for (k = 0, y = 0; k < ObservationNo; k++, y+= ParameterNo)
	//	LSE_B[i*ObservationNo+j] += A[k*ParameterNo+i] * w[k*ObservationNo+j];


	// calculate B matrix, B = LSE_B * A 

	SizeOfMatrix = ParameterNo * ParameterNo;
	double *B = new double[SizeOfMatrix];
	if (B == NULL) cerr << "Fatal error: Fail to allocate memory space\n";

	//   double *MatrixPtr, *FinishPtr;
	//   FinishPtr = B+SizeOfMatrix;
	//   for(MatrixPtr = B; MatrixPtr < FinishPtr; MatrixPtr++) 
	//      *(MatrixPtr) = 0;
	for (i = 0; i < SizeOfMatrix; i++) B[i] = 0.;

	for (i = 0, n = 0; i < ParameterNo; i++)
	for (j = 0; j < ParameterNo; j++)
	for (k = 0, y = 0; k < ObservationNo; k++, y += ParameterNo)
		B[i*ParameterNo + j] += LSE_B[i*ObservationNo + k] * A[k*ParameterNo + j];


	// Cholesky's decomposition
	// calculate C matrix, CC' = B
	double *C = new double[SizeOfMatrix];
	if (C == NULL) cerr << "Fatal error: Fail to allocate memory space.\n";

	//   FinishPtr = C+SizeOfMatrix;
	//  for(MatrixPtr = C; MatrixPtr < FinishPtr; MatrixPtr++) 
	//	*(MatrixPtr) = 0;

	for (i = 0; i < SizeOfMatrix; i++) C[i] = 0.;

	for (i = 0, n = 0; i < ParameterNo; i++, n += ParameterNo)
	for (j = i, y = n; j < ParameterNo; j++, y += ParameterNo) {
		if (i == j) {
			for (k = 0; k <= i - 1; k++)
				*(C + n + j) -= *(C + n + k) * *(C + n + k);
			*(C + n + j) += *(B + n + i);
			*(C + n + j) = sqrt(*(C + n + j));
		} // end of if(i==j)
		else {
			for (k = 0; k <= i - 1; k++)
				*(C + y + i) -= *(C + n + k) * *(C + y + k);
			*(C + y + i) += *(B + n + j);
			*(C + y + i) /= *(C + n + i);
		} // end of else (i == j)
	} // end of for -- i, j 


	// Solve the matrix using UL decomposition

	double *Y = new double[ParameterNo];
	if (Y == NULL) cerr << "Fatal error: fail to allocate memory space\n";
	double *L = new double[ParameterNo];
	if (L == NULL) cerr << "Fatal error: fail to allocate memory space\n";

	for (i = 0; i < ParameterNo; i++) *(L + i) = 0;
	for (i = 0; i < ParameterNo; i++)
	for (j = 0; j < ObservationNo; j++, y += ParameterNo)

		L[i] += LSE_B[i*ObservationNo + j] * l[j];


	for (i = 0, n = 0; i < ParameterNo; i++, n += ParameterNo) {
		*(Y + i) = *(L + i);
		for (j = 0; j <= i - 1; j++) *(Y + i) -= *(C + n + j) * *(Y + j);
		*(Y + i) /= *(C + n + i);
	} // end of for -- i

	delete(B); delete(L);

	for (i = ParameterNo - 1, n = i*ParameterNo; i >= 0; i--, n -= ParameterNo) {
		*(x_hat + i) = *(Y + i);
		for (j = i + 1, y = j*ParameterNo; j < ParameterNo;
			j++, y += ParameterNo) {
			*(x_hat + i) -= *(C + y + i) * *(x_hat + j);
		} // end of for -- j
		*(x_hat + i) /= *(C + n + i);
	} // end of for -- i


	delete(C);

	delete(Y);
	delete(LSE_B);

	return 0;  // 0 -- successful estimation

} // end of function -- Weighted LSE


void CFrmCamera::GetLensDistortion( double &k1, double &k2, double &k3, double& p1, double& p2 ) {

	k1 = m_dK1;
	k2 = m_dK2;
	k3 = m_dK3;
	p1 = m_dP1;
	p2 = m_dP2;
}

void CFrmCamera::SetLensDistortion( double  k1, double  k2, double  k3, double  p1, double  p2 ) {

	m_dK1 = k1;
	m_dK2 = k2;
	m_dK3 = k3;
	m_dP1 = p1;
	m_dP2 = p2;

	m_bLensDistortion = true;

	//SetUndistorted2DistortedTransform();

}

void CFrmCamera::UnsetLensDistortion() {

	m_dK1 = 0.;
	m_dK2 = 0.;
	m_dK3 = 0.;
	m_dP1 = 0.;
	m_dP2 = 0.;
	
	m_bLensDistortion = false;

	// JIKim(150910): 미사용 변수 제거
	//m_dLDC[0] = k1;
	//m_dLDC[1] = k2;
	//m_dLDC[2] = p1;
	//m_dLDC[3] = p2;
	//m_dLDC[4] = k3;
}

int CFrmCamera::ForwardMappingLD( double col, double row, double& GX, double& GY, double& GZ ){

	double xx, yy;
	Image2CamLD( col,  row, xx, yy);

	double ex, ey, ez;
	ex = m_R_image2obj[1][1] * xx + m_R_image2obj[1][2] * yy + m_R_image2obj[1][3] * (-m_dFocalLen);
	ey = m_R_image2obj[2][1] * xx + m_R_image2obj[2][2] * yy + m_R_image2obj[2][3] * (-m_dFocalLen);
	ez = m_R_image2obj[3][1] * xx + m_R_image2obj[3][2] * yy + m_R_image2obj[3][3] * (-m_dFocalLen);

	GZ = 0.;
	GX = (GZ - m_dBz)*ex/ez + m_dBx;
	GY = (GZ - m_dBz)*ey/ez + m_dBy;

	return 0;
}

int CFrmCamera::ForwardMappingLD( double col, double row, double& GX, double& GY, double& GZ, double H ){
		
	double xx, yy;
	Image2CamLD( col,  row, xx, yy);

	double ex, ey, ez;
	ex = m_R_image2obj[1][1] * xx + m_R_image2obj[1][2] * yy + m_R_image2obj[1][3] * (-m_dFocalLen);
	ey = m_R_image2obj[2][1] * xx + m_R_image2obj[2][2] * yy + m_R_image2obj[2][3] * (-m_dFocalLen);
	ez = m_R_image2obj[3][1] * xx + m_R_image2obj[3][2] * yy + m_R_image2obj[3][3] * (-m_dFocalLen);

	GZ = H;
	GX = (GZ - m_dBz)*ex/ez + m_dBx;
	GY = (GZ - m_dBz)*ey/ez + m_dBy;

	return 0;
}

int CFrmCamera::InverseMappingLD( double GX, double GY, double GZ, double& col, double &row){
	
	double xx,yy;

	xx = -m_dFocalLen * ((m_R_image2obj[1][1] * (GX - m_dBx) + m_R_image2obj[2][1] * (GY - m_dBy) + m_R_image2obj[3][1] * (GZ - m_dBz))
		/ (m_R_image2obj[1][3] * (GX - m_dBx) + m_R_image2obj[2][3] * (GY - m_dBy) + m_R_image2obj[3][3] * (GZ - m_dBz)));

	yy = -m_dFocalLen * ((m_R_image2obj[1][2] * (GX - m_dBx) + m_R_image2obj[2][2] * (GY - m_dBy) + m_R_image2obj[3][2] * (GZ - m_dBz))
		/ (m_R_image2obj[1][3] * (GX - m_dBx) + m_R_image2obj[2][3] * (GY - m_dBy) + m_R_image2obj[3][3] * (GZ - m_dBz)));

	Cam2ImageLD( xx, yy, col, row ); 

	return 0;
}

int CFrmCamera::Image2CamLD(double col, double row, double &x, double &y){
		
	double C_Col = m_nColSize*0.5 + m_ppsOff_X;
	double C_Row = m_nRowSize*0.5 + m_ppsOff_Y;

	double ColOrigin = col-C_Col;
	double RowOrigin = -(row-C_Row);

	double ColDistortionRadial, RowDistortionRadial;
	GetRadialLD( ColOrigin, RowOrigin, ColDistortionRadial, RowDistortionRadial );

	double ColDistortionTangential, RowDistortionTangential; 
	GetTangentialLD( ColOrigin, RowOrigin, ColDistortionTangential, RowDistortionTangential );

	x = m_dCPixelSizeX * ( ColOrigin + ColDistortionRadial + ColDistortionTangential );
	y = m_dCPixelSizeY * ( RowOrigin + RowDistortionRadial + RowDistortionTangential );

	return 0;
}

int CFrmCamera::Cam2ImageLD(double x, double y, double &col, double &row){
		
	double LOOP_EXIT_CONDITION = 1e-3;  //  0.001 pixel

	double C_Col = m_nColSize*0.5 + m_ppsOff_X;
	double C_Row = m_nRowSize*0.5 + m_ppsOff_Y;

	double ColCam = x / m_dCPixelSizeX;
	double RowCam = y / m_dCPixelSizeY;

	double ColOri = ColCam;
	double RowOri = RowCam;

	double *AMatrix = new double[ 4 ];
	double *XMatrix = new double[ 2 ];
	double *LMatrix = new double[ 2 ];
	double *CMatrix = new double[ 4 ];

	double F1, F2, dF1_dX, dF1_dY, dF2_dX, dF2_dY;
	double ColRad, ColTan, RowRad, RowTan;
	double dColRad_dX, dColRad_dY, dColTan_dX, dColTan_dY, dRowRad_dX, dRowRad_dY, dRowTan_dX, dRowTan_dY;
	double R2, R4, R6;
	double dR2_dX, dR2_dY, dR4_dX, dR4_dY, dR6_dX, dR6_dY;

	int loop;
	for(loop = 0; loop < 100; loop++ ) {

		R2 = ColOri*ColOri + RowOri*RowOri;
		R4 = R2*R2;
		R6 = R2*R4;
		dR2_dX = 2*ColOri;		dR2_dY = 2*RowOri;
		dR4_dX = 4*ColOri*R2;	dR4_dY = 4*RowOri*R2;
		dR6_dX = 6*ColOri*R4;	dR6_dY = 6*RowOri*R4;

		ColRad     = ColOri * (m_dK1*R2 + m_dK2*R4 + m_dK3*R6 );
		dColRad_dX =  (m_dK1*R2 + m_dK2*R4 + m_dK3*R6 ) + ColOri * ( m_dK1*dR2_dX + m_dK2*dR4_dX + m_dK3*dR6_dX );
		dColRad_dY =                                      ColOri * ( m_dK1*dR2_dY + m_dK2*dR4_dY + m_dK3*dR6_dY );

		RowRad     = RowOri * (m_dK1*R2 + m_dK2*R4 + m_dK3*R6 );
		dRowRad_dX =                                      RowOri * ( m_dK1*dR2_dX + m_dK2*dR4_dX + m_dK3*dR6_dX );
		dRowRad_dY =  (m_dK1*R2 + m_dK2*R4 + m_dK3*R6 ) + RowOri * ( m_dK1*dR2_dY + m_dK2*dR4_dY + m_dK3*dR6_dY );

		ColTan     = 2*m_dP1*ColOri*RowOri + m_dP2*(  R2    + 2*ColOri*ColOri );
		dColTan_dX = 2*m_dP1*RowOri        + m_dP2*( dR2_dX + 4*ColOri );
		dColTan_dY = 2*m_dP1*ColOri        + m_dP2*( dR2_dY            );

		RowTan     = m_dP1*(  R2 + 2*RowOri*RowOri ) + 2*m_dP2*ColOri*RowOri;
		dRowTan_dX = m_dP1*( dR2_dX              ) + 2*m_dP2*RowOri;
		dRowTan_dY = m_dP1*( dR2_dY + 4*RowOri   ) + 2*m_dP2*ColOri;

		F1     = ColOri + ColRad     + ColTan - ColCam;
		dF1_dX = 1      + dColRad_dX + dColTan_dX;
		dF1_dY =          dColRad_dY + dColTan_dY;
		
		F2     = RowOri + RowRad     + RowTan - RowCam;
		dF2_dX =          dRowRad_dX + dRowTan_dX;
		dF2_dY = 1      + dRowRad_dY + dRowTan_dY;
		

		LMatrix[0] = -F1;
		LMatrix[1] = -F2;

		AMatrix[0*2 + 0] = dF1_dX;
		AMatrix[0*2 + 1] = dF1_dY;
		AMatrix[1*2 + 0] = dF2_dX;
		AMatrix[1*2 + 1] = dF2_dY;

		LSE( AMatrix, XMatrix, LMatrix, 2, 2, CMatrix );

		ColOri += XMatrix[0];
		RowOri += XMatrix[1];

		//if(loop==99) 
		//	printf("%lf %lf --> %lf %lf converson failed\n", ColCam+C_Col, -RowCam+C_Row, ColOri+C_Col, -RowOri+C_Row );

		if( fabs(XMatrix[0]) < LOOP_EXIT_CONDITION &&  fabs(XMatrix[1]) < LOOP_EXIT_CONDITION ) loop = 200;

	}

	col =  ColOri + C_Col;
	row = -RowOri + C_Row;

	delete[] AMatrix; 
	delete[] XMatrix; 
	delete[] LMatrix; 
	delete[] CMatrix; 

	if( loop == 100 ) return -1;
	else return 0;
}

int CFrmCamera::GetRadialLD(double x, double y, double &delta_x, double &delta_y) {

	double Radius2 = x*x + y*y;
	double Radius4 = Radius2*Radius2;
	double Radius6 = Radius2*Radius4;

	delta_x = x * (m_dK1*Radius2 + m_dK2*Radius4 + m_dK3*Radius6 );
	delta_y = y * (m_dK1*Radius2 + m_dK2*Radius4 + m_dK3*Radius6 );

	return 0;
}

int CFrmCamera::GetTangentialLD(double x, double y, double &delta_x, double &delta_y) {

	double Radius2 = x*x + y*y;

	delta_x = 2*m_dP1*x*y + m_dP2*( Radius2 + 2*x*x );
	delta_y = m_dP1*( Radius2 + 2*y*y ) + 2*m_dP2*x*y;

	return 0;
}

// Convert Original (col, row) with Lens Distortion into distortion free (Col, Row) coordinates
int CFrmCamera::Origin2Undistorted(double col, double row, double &undist_col, double &undist_row) {

	double C_Col = m_nColSize*0.5 + m_ppsOff_X;
	double C_Row = m_nRowSize*0.5 + m_ppsOff_Y;

	double ColOrigin = col - C_Col;
	double RowOrigin = -(row - C_Row);

	double ColDistortionRadial, RowDistortionRadial;
	GetRadialLD(ColOrigin, RowOrigin, ColDistortionRadial, RowDistortionRadial);

	double ColDistortionTangential, RowDistortionTangential;
	GetTangentialLD(ColOrigin, RowOrigin, ColDistortionTangential, RowDistortionTangential);

	undist_col = ColOrigin + ColDistortionRadial + ColDistortionTangential;
	undist_row = RowOrigin + RowDistortionRadial + RowDistortionTangential;

	undist_col = undist_col + C_Col;
	undist_row = -undist_row + C_Row;

	return 0;
}

// Convert distortion free (col', row') into original image (col, row) coordinates
int CFrmCamera::Undistorted2Origin(double undist_col, double undist_row, double &col, double &row) {

	double detectorX, detectorY;

	Image2CamULD( undist_col, undist_row, detectorX, detectorY );
	Cam2ImageLD( detectorX, detectorY, col, row );

	return 0;
}


void CFrmCamera::SetUndistorted2DistortedTransform() {

	double m_dD2OA[10], m_dD2OB[10], m_dD2OC[10], m_dD2OD[10];
	
	int Scale = 1;
	while( (int)(1+m_nRowSize/Scale)*(int)(1+m_nColSize/Scale) > 1000 ) Scale++;
	int MeasurementSize = 0;
	for( int i = 0; i < m_nRowSize; i += Scale )
		for( int j = 0; j < m_nColSize; j += Scale ) MeasurementSize++;
	MeasurementSize *= 2;

	int ParameterSize = 7; // 19;
	double* AMatrix = new double[MeasurementSize*ParameterSize];
	double* LMatrix = new double[MeasurementSize];
	double* XMatrix = new double[ParameterSize];
	double* Covariance = new double[ ParameterSize*ParameterSize];

	int Index = 0;
	double Ro, Co, X_Origin, Y_Origin; // C, R for Original (Distorted) Image
	double Ru, Cu, X_Undist, Y_Undist;   // C, R for Ideal (Undistored) Image

	cout << "Lens Distortion Para " << m_dK1 << " " << m_dK2 << " " << m_dK3 << " " << m_dP1 << " " << m_dP2 << "\n";

/*	for(Ro = 0.; Ro < m_nRowSize; Ro += Scale )
		for( Co = 0; Co < m_nColSize; Co += Scale ) {

			Origin2Undistorted( Co, Ro, Cu, Ru );

			AMatrix[ Index*ParameterSize + 0 ] =     Cu*Cu*Cu;
			AMatrix[ Index*ParameterSize + 1 ] =     Cu*Cu*Ru;
			AMatrix[ Index*ParameterSize + 2 ] =     Cu*Ru*Ru;
			AMatrix[ Index*ParameterSize + 3 ] =     Ru*Ru*Ru;
			AMatrix[ Index*ParameterSize + 4 ] =        Cu*Cu;
			AMatrix[ Index*ParameterSize + 5 ] =        Cu*Ru;
			AMatrix[ Index*ParameterSize + 6 ] =        Ru*Ru;
			AMatrix[ Index*ParameterSize + 7 ] =           Cu;
			AMatrix[ Index*ParameterSize + 8 ] =           Ru;
			AMatrix[ Index*ParameterSize + 9 ] =            1.;

			AMatrix[ Index*ParameterSize + 10] = -Co*Cu*Cu*Cu;
			AMatrix[ Index*ParameterSize + 11] = -Co*Cu*Cu*Ru;
			AMatrix[ Index*ParameterSize + 12] = -Co*Cu*Ru*Ru;
			AMatrix[ Index*ParameterSize + 13] = -Co*Ru*Ru*Ru;
			AMatrix[ Index*ParameterSize + 14] = -Co*   Cu*Cu;
			AMatrix[ Index*ParameterSize + 15] = -Co*   Cu*Ru;
			AMatrix[ Index*ParameterSize + 16] = -Co*   Ru*Ru;
			AMatrix[ Index*ParameterSize + 17] = -Co*      Cu;
			AMatrix[ Index*ParameterSize + 18] = -Co*      Ru;

			LMatrix[ Index ] = Co;
			Index++;
		}
	LSE( AMatrix, XMatrix, LMatrix, ParameterSize, MeasurementSize, Covariance );

	m_dD2OA[0] = XMatrix[0];
	m_dD2OA[1] = XMatrix[1];
	m_dD2OA[2] = XMatrix[2];
	m_dD2OA[3] = XMatrix[3];
	m_dD2OA[4] = XMatrix[4];
	m_dD2OA[5] = XMatrix[5];
	m_dD2OA[6] = XMatrix[6];
	m_dD2OA[7] = XMatrix[7];
	m_dD2OA[8] = XMatrix[8];
	m_dD2OA[9] = XMatrix[9];

	m_dD2OB[0] = XMatrix[10];
	m_dD2OB[1] = XMatrix[11];
	m_dD2OB[2] = XMatrix[12];
	m_dD2OB[3] = XMatrix[13];
	m_dD2OB[4] = XMatrix[14];
	m_dD2OB[5] = XMatrix[15];
	m_dD2OB[6] = XMatrix[16];
	m_dD2OB[7] = XMatrix[17];
	m_dD2OB[8] = XMatrix[18];
	m_dD2OB[9] = 1.;

	Index = 0;
	for(Ro = 0.; Ro < m_nRowSize; Ro += Scale )
		for( Co = 0; Co < m_nColSize; Co += Scale ) {

			Origin2Undistorted( Co, Ro, Cu, Ru );

			AMatrix[ Index*ParameterSize + 0 ] =     Cu*Cu*Cu;
			AMatrix[ Index*ParameterSize + 1 ] =     Cu*Cu*Ru;
			AMatrix[ Index*ParameterSize + 2 ] =     Cu*Ru*Ru;
			AMatrix[ Index*ParameterSize + 3 ] =     Ru*Ru*Ru;
			AMatrix[ Index*ParameterSize + 4 ] =        Cu*Cu;
			AMatrix[ Index*ParameterSize + 5 ] =        Cu*Ru;
			AMatrix[ Index*ParameterSize + 6 ] =        Ru*Ru;
			AMatrix[ Index*ParameterSize + 7 ] =           Cu;
			AMatrix[ Index*ParameterSize + 8 ] =           Ru;
			AMatrix[ Index*ParameterSize + 9 ] =            1.;

			AMatrix[ Index*ParameterSize + 10] = -Ro*Cu*Cu*Cu;
			AMatrix[ Index*ParameterSize + 11] = -Ro*Cu*Cu*Ru;
			AMatrix[ Index*ParameterSize + 12] = -Ro*Cu*Ru*Ru;
			AMatrix[ Index*ParameterSize + 13] = -Ro*Ru*Ru*Ru;
			AMatrix[ Index*ParameterSize + 14] = -Ro*   Cu*Cu;
			AMatrix[ Index*ParameterSize + 15] = -Ro*   Cu*Ru;
			AMatrix[ Index*ParameterSize + 16] = -Ro*   Ru*Ru;
			AMatrix[ Index*ParameterSize + 17] = -Ro*      Cu;
			AMatrix[ Index*ParameterSize + 18] = -Ro*      Ru;

			LMatrix[ Index ] = Ro;
			Index++;
		}
	LSE( AMatrix, XMatrix, LMatrix, ParameterSize, MeasurementSize, Covariance );

	m_dD2OC[0] = XMatrix[0];
	m_dD2OC[1] = XMatrix[1];
	m_dD2OC[2] = XMatrix[2];
	m_dD2OC[3] = XMatrix[3];
	m_dD2OC[4] = XMatrix[4];
	m_dD2OC[5] = XMatrix[5];
	m_dD2OC[6] = XMatrix[6];
	m_dD2OC[7] = XMatrix[7];
	m_dD2OC[8] = XMatrix[8];
	m_dD2OC[9] = XMatrix[9];

	m_dD2OD[0] = XMatrix[10];
	m_dD2OD[1] = XMatrix[11];
	m_dD2OD[2] = XMatrix[12];
	m_dD2OD[3] = XMatrix[13];
	m_dD2OD[4] = XMatrix[14];
	m_dD2OD[5] = XMatrix[15];
	m_dD2OD[6] = XMatrix[16];
	m_dD2OD[7] = XMatrix[17];
	m_dD2OD[8] = XMatrix[18];
	m_dD2OD[9] = 1.;

	
	double Con, Ron;
	for(int i = 0; i < m_nRowSize; i+=5)
		for(int j = 0; j < m_nColSize; j+=5 ) {

			Origin2Undistorted( j, i, Cu, Ru );
			Undistorted2Origin( Cu, Ru, Co, Ro );

			Con =		  m_dD2OA[0]*Cu*Cu*Cu 
						+ m_dD2OA[1]*Cu*Cu*Ru
						+ m_dD2OA[2]*Cu*Ru*Ru
						+ m_dD2OA[3]*Ru*Ru*Ru
						+ m_dD2OA[4]*   Cu*Cu
						+ m_dD2OA[5]*   Cu*Ru
						+ m_dD2OA[6]*   Ru*Ru
						+ m_dD2OA[7]*      Cu
					    + m_dD2OA[8]*      Ru
						+ m_dD2OA[9]*       1.;

			Con /=		  m_dD2OB[0]*Cu*Cu*Cu 
						+ m_dD2OB[1]*Cu*Cu*Ru
						+ m_dD2OB[2]*Cu*Ru*Ru
						+ m_dD2OB[3]*Ru*Ru*Ru
						+ m_dD2OB[4]*   Cu*Cu
						+ m_dD2OB[5]*   Cu*Ru
						+ m_dD2OB[6]*   Ru*Ru
						+ m_dD2OB[7]*      Cu
					    + m_dD2OB[8]*      Ru
						+ m_dD2OB[9]*       1.;

			Ron =		  m_dD2OC[0]*Cu*Cu*Cu 
						+ m_dD2OC[1]*Cu*Cu*Ru
						+ m_dD2OC[2]*Cu*Ru*Ru
						+ m_dD2OC[3]*Ru*Ru*Ru
						+ m_dD2OC[4]*   Cu*Cu
						+ m_dD2OC[5]*   Cu*Ru
						+ m_dD2OC[6]*   Ru*Ru
						+ m_dD2OC[7]*      Cu
					    + m_dD2OC[8]*      Ru
						+ m_dD2OC[9]*       1.;

			Ron /=		  m_dD2OD[0]*Cu*Cu*Cu 
						+ m_dD2OD[1]*Cu*Cu*Ru
						+ m_dD2OD[2]*Cu*Ru*Ru
						+ m_dD2OD[3]*Ru*Ru*Ru
						+ m_dD2OD[4]*   Cu*Cu
						+ m_dD2OD[5]*   Cu*Ru
						+ m_dD2OD[6]*   Ru*Ru
						+ m_dD2OD[7]*      Cu
					    + m_dD2OD[8]*      Ru
						+ m_dD2OD[9]*       1.;

			cout << "Origin " << j << " " << i << " Dist " << Cu << " " << Ru << " Cam2Image " << Co << " " << Ro << " new " << Con << " " << Ron << "\n";
		}

*/

	Index = 0;
	double R2, R4, R6, R8, R10;
	double DistCol, DistRow, UndistCol, UndistRow;

	for(Ro = 0.; Ro < m_nRowSize; Ro += Scale )
		for( Co = 0; Co < m_nColSize; Co += Scale ) {

			Origin2Undistorted( Co, Ro, Cu, Ru );
			
			DistCol   = GetCenterCol( Co );		DistRow   = GetCenterRow( Ro );
			UndistCol = GetCenterCol( Cu );		UndistRow = GetCenterRow( Ru );

			R2  = UndistCol*UndistCol + UndistRow*UndistRow;
			R4  = R2*R2;
			R6  = R4*R2;
			R8  = R6*R2;
			R10 = R8*R2;

			AMatrix[Index * ParameterSize + 0 ] = UndistCol * R2;
			AMatrix[Index * ParameterSize + 1 ] = UndistCol * R4;
			AMatrix[Index * ParameterSize + 2 ] = UndistCol * R6;
			AMatrix[Index * ParameterSize + 3 ] = UndistCol * R8;
			AMatrix[Index * ParameterSize + 4 ] = UndistCol * R10;

			AMatrix[Index * ParameterSize + 5 ] = 2 * UndistCol * UndistRow;
			AMatrix[Index * ParameterSize + 6 ] = R2 * 2 * UndistCol * UndistCol;
			LMatrix[Index] = DistCol - UndistCol;
			Index++;
	
			AMatrix[Index * ParameterSize + 0 ] = UndistRow * R2;
			AMatrix[Index * ParameterSize + 1 ] = UndistRow * R4;
			AMatrix[Index * ParameterSize + 2 ] = UndistRow * R6;
			AMatrix[Index * ParameterSize + 3 ] = UndistRow * R8;
			AMatrix[Index * ParameterSize + 4 ] = UndistRow * R10;

			AMatrix[Index * ParameterSize + 5 ] = R2 + 2 * UndistRow * UndistRow;
			AMatrix[Index * ParameterSize + 6 ] = 2 * UndistCol * UndistRow;
			LMatrix[Index] = DistRow - UndistRow;
			Index++;
		}

	LSE ( AMatrix, XMatrix, LMatrix, ParameterSize, MeasurementSize, Covariance );
	m_dU2DK1 = XMatrix[0];
	m_dU2DK2 = XMatrix[1];
	m_dU2DK3 = XMatrix[2];
	m_dU2DK4 = XMatrix[3];
	m_dU2DK5 = XMatrix[4];

	m_dU2DP1 = XMatrix[5];
	m_dU2DP2 = XMatrix[6];

/*	double Con, Ron;
	for(int i = 0; i < m_nRowSize; i+=5)
		for(int j = 0; j < m_nColSize; j+=5 ) {

			Origin2Undistorted( j, i, Cu, Ru );
			Undistorted2Origin( Cu, Ru, Co, Ro );

			UndistCol = GetCenterCol( Cu );		UndistRow = GetCenterRow( Ru );
			R2 = UndistCol*UndistCol + UndistRow*UndistRow;
			R4 = R2*R2;
			R6 = R4*R2;
			R8  = R6*R2;
			R10 = R8*R2;

			DistCol = UndistCol * ( 1 + m_dU2DK1 * R2 + m_dU2DK2 * R4 + m_dU2DK3 * R6 + m_dU2DK4 * R8 + m_dU2DK5 * R10 ) + 2 * m_dU2DP1 * UndistCol * UndistRow + m_dU2DP2 * ( R2 + 2 * UndistCol * UndistCol );
			DistRow = UndistRow * ( 1 + m_dU2DK1 * R2 + m_dU2DK2 * R4 + m_dU2DK3 * R6 + m_dU2DK4 * R8 + m_dU2DK5 * R10 ) + 2 * m_dU2DP2 * UndistCol * UndistRow + m_dU2DP1 * ( R2 + 2 * UndistRow * UndistRow );

			Con =	DistCol + (m_nColSize*0.5+m_ppsOff_X );
			Ron =  -DistRow + (m_nRowSize*0.5+m_ppsOff_Y );

			cout << "Origin " << j << " " << i << " Dist " << Cu << " " << Ru << " Cam2Image " << Co << " " << Ro << " new " << Con << " " << Ron << "\n";
		}
*/

	return;
}

void CFrmCamera::CheckImageGSD(double RefHeight, double& ColGSD, double& RowGSD, double& AllGSD) {

	double Col1, Col2, Row1, Row2;
	double XX1, XX2, YY1, YY2, ZZ1, ZZ2;

	Col1 = m_nColSize / 2.;
	Row1 = m_nRowSize / 2.;
	this->ForwardMapping(Col1, Row1, XX1, YY1, ZZ1, RefHeight);
	Col2 = Col1 + 1;
	Row2 = Row1;
	this->ForwardMapping(Col2, Row2, XX2, YY2, ZZ2, RefHeight);
	ColGSD = sqrt((XX1 - XX2)*(XX1 - XX2) + (YY1 - YY2)*(YY1 - YY2));

	Col2 = Col1;
	Row2 = Row1 + 1;
	this->ForwardMapping(Col2, Row2, XX2, YY2, ZZ2, RefHeight);
	RowGSD = sqrt((XX1 - XX2)*(XX1 - XX2) + (YY1 - YY2)*(YY1 - YY2));

	// Arithmetic Mean
	AllGSD = (ColGSD + RowGSD) / 2;
}


void CFrmCamera::Rotation2Angles(double R_i2o[9], double &W_o2i_rad, double &P_o2i_rad, double &K_o2i_rad )
{

	double r11 = R_i2o[0];
	double r12 = R_i2o[1];
	double r13 = R_i2o[2];
	double r21 = R_i2o[3];
	double r22 = R_i2o[4];
	double r23 = R_i2o[5];
	double r31 = R_i2o[6];
	double r32 = R_i2o[7];
	double r33 = R_i2o[8];

	P_o2i_rad = atan2(r13, sqrt(r11*r11 + r12*r12));  // good as long as fabs(P) < 90
	K_o2i_rad = atan2(-r12, r11);
	W_o2i_rad = atan2(-r23, r33);

}

bool CFrmCamera::GetRotationByTwoVector(double Vec1[3], double Vec2[3], double Rot[9]) {

	cv::Mat_<double> AVec = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat_<double> BVec = cv::Mat::zeros(3, 1, CV_64F);
	AVec(0) = Vec1[0];	AVec(1) = Vec1[1];	AVec(2) = Vec1[2];
	BVec(0) = Vec2[0];	BVec(1) = Vec2[1];	BVec(2) = Vec2[2];
	double mag1 = sqrt(AVec(0)*AVec(0) + AVec(1)*AVec(1) + AVec(2)*AVec(2));
	double mag2 = sqrt(BVec(0)*BVec(0) + BVec(1)*BVec(1) + BVec(2)*BVec(2));
	AVec(0) = AVec(0) / mag1;	AVec(1) = AVec(1) / mag1;	AVec(2) = AVec(2) / mag1;
	BVec(0) = BVec(0) / mag2;	BVec(1) = BVec(1) / mag2;	BVec(2) = BVec(2) / mag2;

	cv::Mat_<double> VVec = cv::Mat::zeros(3, 1, CV_64F);
	cv::Mat_<double> WVec = cv::Mat::zeros(3, 1, CV_64F);
	VVec = BVec - AVec.dot(BVec) * AVec;
	WVec = BVec.cross(AVec);
	mag1 = sqrt(VVec(0)*VVec(0) + VVec(1)*VVec(1) + VVec(2)*VVec(2));
	mag2 = sqrt(WVec(0)*WVec(0) + WVec(1)*WVec(1) + WVec(2)*WVec(2));
	VVec(0) = VVec(0) / mag1;	VVec(1) = VVec(1) / mag1;	VVec(2) = VVec(2) / mag1;
	WVec(0) = WVec(0) / mag2;	WVec(1) = WVec(1) / mag2;	WVec(2) = WVec(2) / mag2;

	double cosT, sinT;
	cosT = AVec.dot(BVec);
	sinT = mag2;

	cv::Mat_<double> GMat = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat_<double> FMat_T = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat_<double> FMat = cv::Mat::zeros(3, 3, CV_64F);

	// Rotation matrix from AVec to BVec
	GMat(0, 0) = cosT;	GMat(0, 1) = -sinT;		GMat(0, 2) = 0.;
	GMat(1, 0) = sinT;	GMat(1, 1) = cosT;		GMat(1, 2) = 0.;
	GMat(2, 0) = 0.;	GMat(2, 1) = 0.;		GMat(2, 2) = 1.;

	FMat_T(0, 0) = AVec(0);		FMat_T(0, 1) = VVec(0);		FMat_T(0, 2) = WVec(0);
	FMat_T(1, 0) = AVec(1);		FMat_T(1, 1) = VVec(1);		FMat_T(1, 2) = WVec(1);
	FMat_T(2, 0) = AVec(2);		FMat_T(2, 1) = VVec(2);		FMat_T(2, 2) = WVec(2);

	FMat = FMat_T.t();

	double RotationMat[9] = { 0. };
	cv::Mat RMat(3, 3, CV_64F, RotationMat);
	RMat = FMat_T * GMat * FMat;

	for (int i = 0; i < 9; i++) Rot[i] = RotationMat[i];

	return true;

}