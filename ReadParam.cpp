#include "ReadParam.h"

// IOP 정보를 파일에서 읽어와 구조체에 저장하는 함수
int ReadIOP(string filename, IOP& iop)
{
	int result = 0;
	ifstream fin;
	fin.open(filename);

	char chTemp[20];

	// 각 항목의 라벨 확인 후 값 읽기
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

	// 단위 변환: mm → m, um → m
	iop.Focal /= 1000.0;
	iop.CellSize /= 1000000.0;

	return result;
}

// EOP(Exterior Orientation Parameters)를 파일에서 읽어 벡터에 저장
int ReadEOP(string filename, vector<EOP>& eop)
{
	int result = 1;
	ifstream fin;
	fin.open(filename);

	char chTemp[20];

	// 헤더 부분 건너뛰기
	fin >> chTemp >> chTemp >> chTemp >> chTemp >> chTemp >> chTemp >> chTemp;

	EOP temp_EOP;

	// 파일 끝까지 한 줄씩 읽으며 EOP 저장
	while (fin)
	{
		fin >> temp_EOP.filename >> temp_EOP.Xs >> temp_EOP.Ys >> temp_EOP.Zs >> temp_EOP.O >> temp_EOP.P >> temp_EOP.K;
		eop.push_back(temp_EOP);
	}

	eop.pop_back(); // 마지막 라인 중복 또는 잘못된 값 제거

	return result;
}

// 특정 이미지 이름에 해당하는 EOP 정보를 찾고 out_eop에 저장
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

// GCP(Ground Control Point) 정보를 파일에서 읽어 벡터에 저장
int ReadGCP(string filename, vector<GCP>& gcp)
{
	int result = 1;
	ifstream fin;
	fin.open(filename);

	vector<pair<int, string>> image_name_vec; // 이미지 번호-이름 매핑
	char chTemp[20];
	int image_num;

	// 이미지 이름 매핑 정보 읽기
	fin >> chTemp >> image_num;
	image_name_vec.resize(image_num);

	for (int i = 0; i < image_num; i++)
	{
		fin >> image_name_vec[i].first >> image_name_vec[i].second;
	}

	GCP temp_gcp;
	GCPpair temp_GCPpair;

	// GCP 정보 읽기
	while (fin)
	{
		if (fin.eof()) break;

		// GCP의 이미지 개수
		fin >> temp_gcp.paircount;
		temp_gcp.order.resize(temp_gcp.paircount);

		for (int i = 0; i < temp_gcp.paircount; i++)
		{
			// 각 이미지에서의 위치 정보
			fin >> temp_GCPpair.Order >> temp_GCPpair.CR.Col >> temp_GCPpair.CR.Row;

			// 이미지 이름 찾기
			for (int t = 0; t < image_name_vec.size(); t++) {
				if (image_name_vec[t].first == temp_GCPpair.Order)
					temp_GCPpair.ImageName = image_name_vec[t].second;
			}

			// 이름이 없으면 스킵
			if (temp_GCPpair.ImageName.empty()) continue;

			temp_gcp.order[i] = temp_GCPpair;
		}

		// GCP의 실제 좌표
		fin >> temp_gcp.xyz.X >> temp_gcp.xyz.Y >> temp_gcp.xyz.Z;

		gcp.push_back(temp_gcp);
	}

	return result;
}

// 특정 이미지에 해당하는 GCP들만 추출하여 출력 벡터에 저장
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
				// GCP 좌표 및 이미지 내 좌표 저장
				one_GCP.xyz = in_gcp_vec[i].xyz;
				one_GCP.order[0] = in_gcp_vec[i].order[j];
				out_gcp_vec.push_back(one_GCP);
			}
		}
	}

	// 결과 반환
	if (out_gcp_vec.size() == 0)
		result = -1;
	else
		result = 0;

	return result;
}
