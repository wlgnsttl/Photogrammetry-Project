#ifndef READ_PARAM_H_INCLUDED
#define READ_PARAM_H_INCLUDED

using namespace std;

#include <fstream>   // 파일 입출력을 위한 헤더
#include <vector>    // STL 벡터 사용을 위한 헤더

// OpenCV 관련 헤더 (영상 처리용)
#include "opencv.hpp"
#include "core.hpp"
#include "imgproc.hpp"

// ---------------------------
// 외부 표정 요소 (Exterior Orientation Parameters)
// 각 이미지에 대한 촬영 위치 및 방향 정보
// ---------------------------
struct EOP {
	char filename[1024]; // 이미지 파일 이름
	double Xs;           // 카메라 위치의 X 좌표 (지상 기준)
	double Ys;           // Y 좌표
	double Zs;           // Z 좌표
	double O;            // Omega (회전각: X축 기준)
	double P;            // Phi (회전각: Y축 기준)
	double K;            // Kappa (회전각: Z축 기준)
};

// ---------------------------
// 내부 표정 요소 (Interior Orientation Parameters)
// 카메라 내부 정보 (렌즈, CCD 등)
// ---------------------------
struct IOP {
	double Focal;     // 초점 거리 (focal length) [mm]
	double CellSize;  // 픽셀 크기 (pixel size) [um]
	double ColSize;   // 이미지의 가로 픽셀 수
	double RowSize;   // 이미지의 세로 픽셀 수
	double PPx;       // 주점 x좌표 (Principal Point X) [pixel]
	double PPy;       // 주점 y좌표 (Principal Point Y) [pixel]
	double K1, K2, K3; // 방사 왜곡 계수 (Radial distortion coefficients)
	double P1, P2;     // 접선 왜곡 계수 (Tangential distortion coefficients)
};

// ---------------------------
// 3차원 좌표 구조체
// ---------------------------
struct XYZ {
	double X;
	double Y;
	double Z;
};

// ---------------------------
// 이미지 상에서의 2D 좌표 (Column, Row)
// ---------------------------
struct ColRow {
	double Col;  // 열 좌표 (x)
	double Row;  // 행 좌표 (y)
};

// ---------------------------
// 하나의 GCP(지상 기준점)에 대한 이미지 매칭 정보
// ---------------------------
struct GCPpair
{
	int Order;               // 이미지 번호 또는 식별 번호
	std::string ImageName;   // 해당 이미지 이름
	ColRow CR;               // 이미지에서의 위치 (픽셀 좌표)

	// 비교 연산자 오버로딩 (GCPpair 비교 시)
	bool operator ==(const GCPpair& other) const
	{
		if (Order == other.Order &&
			CR.Col == other.CR.Col &&
			CR.Row == other.CR.Row) return true;
		else return false;  // 이 부분 빠져있었음 (원래 코드에는 return false가 없음)
	}
};

// ---------------------------
// 하나의 GCP(지상 기준점) 구조체
// 하나의 3D 위치에 대해 여러 이미지에서의 좌표가 있을 수 있음
// ---------------------------
struct GCP {
	int paircount;                // 이 GCP를 참조하는 이미지 수
	XYZ xyz;                      // 지상 기준점의 실제 3D 좌표
	std::vector<GCPpair> order;   // 해당 GCP의 이미지 내 매칭 좌표들
};

// ---------------------------
// 각도 단위 변환 상수
// ---------------------------
const double ToRad = acos(-1.) / 180.;  // 도 → 라디안 변환 상수
const double ToDeg = 180. / acos(-1.);  // 라디안 → 도 변환 상수

// IOP 정보를 파일에서 읽어와 구조체에 저장하는 함수
int ReadIOP(string filename, IOP& iop);

// EOP(Exterior Orientation Parameters)를 파일에서 읽어 벡터에 저장
int ReadEOP(string filename, vector<EOP>& eop);

// 특정 이미지 이름에 해당하는 EOP 정보를 찾고 out_eop에 저장
int ReadImageEOP(std::vector<EOP> in_eop_vec, string in_image_name,  EOP& out_eop);

// GCP(Ground Control Point) 정보를 파일에서 읽어 벡터에 저장
int ReadGCP(string filename, vector<GCP>& gcp);

// 특정 이미지에 해당하는 GCP들만 추출하여 출력 벡터에 저장
int ReadImageGCP(std::vector<GCP> in_gcp_vec, string in_image_name, std::vector<GCP>& out_gcp_vec);

#endif