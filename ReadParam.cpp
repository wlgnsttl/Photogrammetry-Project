#include "ReadParam.h"

// IOP ������ ���Ͽ��� �о�� ����ü�� �����ϴ� �Լ�
int ReadIOP(string filename, IOP& iop)
{
	int result = 0;
	ifstream fin;
	fin.open(filename);

	char chTemp[20];

	// �� �׸��� �� Ȯ�� �� �� �б�
	fin >> chTemp;
	if (strncmp(chTemp, "focal(mm):", 10) != 0) return -1;
	fin >> iop.Focal >> chTemp;

	if (strncmp(chTemp, "pixel_size(um):", 15) != 0) return -1;
	fin >> iop.CellSize >> chTemp;

	if (strncmp(chTemp, "pixel_num_in_x:", 15) != 0) return -1;
	fin >> iop.ColSize >> chTemp;

	if (strncmp(chTemp, "pixel_num_in_y:", 15) != 0) return -1;
	fin >> iop.RowSize >> chTemp;

	if (strncmp(chTemp, "PPx:", 4) != 0) return -1;
	fin >> iop.PPx >> chTemp;

	if (strncmp(chTemp, "PPy:", 4) != 0) return -1;
	fin >> iop.PPy >> chTemp;

	if (strncmp(chTemp, "K1:", 3) != 0) return -1;
	fin >> iop.K1 >> chTemp;

	if (strncmp(chTemp, "K2:", 3) != 0) return -1;
	fin >> iop.K2 >> chTemp;

	if (strncmp(chTemp, "P1:", 3) != 0) return -1;
	fin >> iop.P1 >> chTemp;

	if (strncmp(chTemp, "P2:", 3) != 0) return -1;
	fin >> iop.P2 >> chTemp;

	if (strncmp(chTemp, "K3:", 3) != 0) return -1;
	fin >> iop.K3 >> chTemp;

	fin.close();

	// ���� ��ȯ: mm �� m, um �� m
	iop.Focal /= 1000.0;
	iop.CellSize /= 1000000.0;

	return result;
}

// EOP(Exterior Orientation Parameters)�� ���Ͽ��� �о� ���Ϳ� ����
int ReadEOP(string filename, vector<EOP>& eop)
{
	int result = 1;
	ifstream fin;
	fin.open(filename);

	char chTemp[20];

	// ��� �κ� �ǳʶٱ�
	fin >> chTemp >> chTemp >> chTemp >> chTemp >> chTemp >> chTemp >> chTemp;

	EOP temp_EOP;

	// ���� ������ �� �پ� ������ EOP ����
	while (fin)
	{
		fin >> temp_EOP.filename >> temp_EOP.Xs >> temp_EOP.Ys >> temp_EOP.Zs >> temp_EOP.O >> temp_EOP.P >> temp_EOP.K;
		eop.push_back(temp_EOP);
	}

	eop.pop_back(); // ������ ���� �ߺ� �Ǵ� �߸��� �� ����

	return result;
}

// Ư�� �̹��� �̸��� �ش��ϴ� EOP ������ ã�� out_eop�� ����
int ReadImageEOP(std::vector<EOP> in_eop_vec, string in_image_name, EOP& out_eop)
{
	int result = -1;

	for (int i = 0; i < in_eop_vec.size(); i++) {
		if (in_image_name == in_eop_vec[i].filename) {
			out_eop = in_eop_vec[i];
			result = 0;
		}
	}

	return result;
}

// GCP(Ground Control Point) ������ ���Ͽ��� �о� ���Ϳ� ����
int ReadGCP(string filename, vector<GCP>& gcp)
{
	int result = 1;
	ifstream fin;
	fin.open(filename);

	vector<pair<int, string>> image_name_vec; // �̹��� ��ȣ-�̸� ����
	char chTemp[20];
	int image_num;

	// �̹��� �̸� ���� ���� �б�
	fin >> chTemp >> image_num;
	image_name_vec.resize(image_num);

	for (int i = 0; i < image_num; i++)
	{
		fin >> image_name_vec[i].first >> image_name_vec[i].second;
	}

	GCP temp_gcp;
	GCPpair temp_GCPpair;

	// GCP ���� �б�
	while (fin)
	{
		if (fin.eof()) break;

		// GCP�� �̹��� ����
		fin >> temp_gcp.paircount;
		temp_gcp.order.resize(temp_gcp.paircount);

		for (int i = 0; i < temp_gcp.paircount; i++)
		{
			// �� �̹��������� ��ġ ����
			fin >> temp_GCPpair.Order >> temp_GCPpair.CR.Col >> temp_GCPpair.CR.Row;

			// �̹��� �̸� ã��
			for (int t = 0; t < image_name_vec.size(); t++) {
				if (image_name_vec[t].first == temp_GCPpair.Order)
					temp_GCPpair.ImageName = image_name_vec[t].second;
			}

			// �̸��� ������ ��ŵ
			if (temp_GCPpair.ImageName.empty()) continue;

			temp_gcp.order[i] = temp_GCPpair;
		}

		// GCP�� ���� ��ǥ
		fin >> temp_gcp.xyz.X >> temp_gcp.xyz.Y >> temp_gcp.xyz.Z;

		gcp.push_back(temp_gcp);
	}

	return result;
}

// Ư�� �̹����� �ش��ϴ� GCP�鸸 �����Ͽ� ��� ���Ϳ� ����
int ReadImageGCP(std::vector<GCP> in_gcp_vec, string in_image_name, std::vector<GCP>& out_gcp_vec)
{
	int result = 0;
	out_gcp_vec.resize(0);

	for (int i = 0; i < in_gcp_vec.size(); i++) {
		GCP one_GCP;
		one_GCP.order.resize(1);
		one_GCP.paircount = 1;

		int pairCount = in_gcp_vec[i].paircount;

		for (int j = 0; j < pairCount; j++) {
			if (in_gcp_vec[i].order[j].ImageName == in_image_name) {
				// GCP ��ǥ �� �̹��� �� ��ǥ ����
				one_GCP.xyz = in_gcp_vec[i].xyz;
				one_GCP.order[0] = in_gcp_vec[i].order[j];
				out_gcp_vec.push_back(one_GCP);
			}
		}
	}

	// ��� ��ȯ
	if (out_gcp_vec.size() == 0)
		result = -1;
	else
		result = 0;

	return result;
}
