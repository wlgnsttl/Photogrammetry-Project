#ifndef READ_PARAM_H_INCLUDED
#define READ_PARAM_H_INCLUDED

using namespace std;

#include <fstream>   // ���� ������� ���� ���
#include <vector>    // STL ���� ����� ���� ���

// OpenCV ���� ��� (���� ó����)
#include "opencv.hpp"
#include "core.hpp"
#include "imgproc.hpp"

// ---------------------------
// �ܺ� ǥ�� ��� (Exterior Orientation Parameters)
// �� �̹����� ���� �Կ� ��ġ �� ���� ����
// ---------------------------
struct EOP {
	char filename[1024]; // �̹��� ���� �̸�
	double Xs;           // ī�޶� ��ġ�� X ��ǥ (���� ����)
	double Ys;           // Y ��ǥ
	double Zs;           // Z ��ǥ
	double O;            // Omega (ȸ����: X�� ����)
	double P;            // Phi (ȸ����: Y�� ����)
	double K;            // Kappa (ȸ����: Z�� ����)
};

// ---------------------------
// ���� ǥ�� ��� (Interior Orientation Parameters)
// ī�޶� ���� ���� (����, CCD ��)
// ---------------------------
struct IOP {
	double Focal;     // ���� �Ÿ� (focal length) [mm]
	double CellSize;  // �ȼ� ũ�� (pixel size) [um]
	double ColSize;   // �̹����� ���� �ȼ� ��
	double RowSize;   // �̹����� ���� �ȼ� ��
	double PPx;       // ���� x��ǥ (Principal Point X) [pixel]
	double PPy;       // ���� y��ǥ (Principal Point Y) [pixel]
	double K1, K2, K3; // ��� �ְ� ��� (Radial distortion coefficients)
	double P1, P2;     // ���� �ְ� ��� (Tangential distortion coefficients)
};

// ---------------------------
// 3���� ��ǥ ����ü
// ---------------------------
struct XYZ {
	double X;
	double Y;
	double Z;
};

// ---------------------------
// �̹��� �󿡼��� 2D ��ǥ (Column, Row)
// ---------------------------
struct ColRow {
	double Col;  // �� ��ǥ (x)
	double Row;  // �� ��ǥ (y)
};

// ---------------------------
// �ϳ��� GCP(���� ������)�� ���� �̹��� ��Ī ����
// ---------------------------
struct GCPpair
{
	int Order;               // �̹��� ��ȣ �Ǵ� �ĺ� ��ȣ
	std::string ImageName;   // �ش� �̹��� �̸�
	ColRow CR;               // �̹��������� ��ġ (�ȼ� ��ǥ)

	// �� ������ �����ε� (GCPpair �� ��)
	bool operator ==(const GCPpair& other) const
	{
		if (Order == other.Order &&
			CR.Col == other.CR.Col &&
			CR.Row == other.CR.Row) return true;
		else return false;  // �� �κ� �����־��� (���� �ڵ忡�� return false�� ����)
	}
};

// ---------------------------
// �ϳ��� GCP(���� ������) ����ü
// �ϳ��� 3D ��ġ�� ���� ���� �̹��������� ��ǥ�� ���� �� ����
// ---------------------------
struct GCP {
	int paircount;                // �� GCP�� �����ϴ� �̹��� ��
	XYZ xyz;                      // ���� �������� ���� 3D ��ǥ
	std::vector<GCPpair> order;   // �ش� GCP�� �̹��� �� ��Ī ��ǥ��
};

// ---------------------------
// ���� ���� ��ȯ ���
// ---------------------------
const double ToRad = acos(-1.) / 180.;  // �� �� ���� ��ȯ ���
const double ToDeg = 180. / acos(-1.);  // ���� �� �� ��ȯ ���

// IOP ������ ���Ͽ��� �о�� ����ü�� �����ϴ� �Լ�
int ReadIOP(string filename, IOP& iop);

// EOP(Exterior Orientation Parameters)�� ���Ͽ��� �о� ���Ϳ� ����
int ReadEOP(string filename, vector<EOP>& eop);

// Ư�� �̹��� �̸��� �ش��ϴ� EOP ������ ã�� out_eop�� ����
int ReadImageEOP(std::vector<EOP> in_eop_vec, string in_image_name,  EOP& out_eop);

// GCP(Ground Control Point) ������ ���Ͽ��� �о� ���Ϳ� ����
int ReadGCP(string filename, vector<GCP>& gcp);

// Ư�� �̹����� �ش��ϴ� GCP�鸸 �����Ͽ� ��� ���Ϳ� ����
int ReadImageGCP(std::vector<GCP> in_gcp_vec, string in_image_name, std::vector<GCP>& out_gcp_vec);

#endif