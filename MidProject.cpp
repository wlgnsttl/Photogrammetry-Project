#include "MidProject.h"
#include <filesystem>  // C++17부터 사용 가능
namespace fs = std::filesystem;

void print_error_analysis(const std::string& label, size_t idx, double cal_c, double cal_r, double true_c, double true_r) {
    std::cout << std::fixed << std::setprecision(10);  // 소수점 10자리까지 고정 출력
    std::cout << label << " : check : " << idx + 1 << std::endl;
    std::cout << "Calc (C,R) = " << cal_c << "\t" << cal_r << std::endl;
    std::cout << "True (C,R) = " << true_c << "\t" << true_r << std::endl;
    std::cout << "Error X = " << cal_c - true_c << std::endl;
    std::cout << "Error Y = " << cal_r - true_r << std::endl;
    std::cout << "Error   = " << std::sqrt(std::pow(cal_c - true_c, 2) + std::pow(cal_r - true_r, 2)) << std::endl << std::endl;
}

void MidProject1()
{
    std::string file_path = "resource/";
    std::string image_file_name_1 = "R0047633.JPG";
    std::string image_file_name_2 = "R0047634.JPG";

    //std::vector<ColRow> R0047633_model_A1 = { {4254, 895}, {4535, 1872}, {4668, 914}, {4225, 1345}, {4560, 1620} };
    //std::vector<ColRow> R0047634_model_A1 = { {2824, 1288}, {3006, 2271}, {3224, 1333}, {2745, 1727}, {3037, 2021} };
    //std::vector<ColRow> R0047633_check_A2 = { {4682, 817}, {4803, 2093} };
    //std::vector<ColRow> R0047634_check_A2 = { {3242, 1240}, {3293, 2509} };

    //std::vector<ColRow> R0047633_model_B1 = { {1638, 373}, {2428, 1446}, {2516, 2763}, {3670, 2248}, {3308, 634} };
    //std::vector<ColRow> R0047634_model_B1 = { {565, 623}, {1241, 1713}, {1246, 3040}, {2385, 2594}, {2110, 977} };
    //std::vector<ColRow> R0047633_check_B2 = { {3658, 2807}, {2433, 1860} };
    //std::vector<ColRow> R0047634_check_B2 = { {2350, 3164}, {1220, 2121} };

    std::vector<ColRow> R0047633_model_A1 = { {3303, -1847}, {4149, -2048}, {4060, -2440},
    {3726, -2360}, {3688, -2511}, {3443, -2452} };
    std::vector<ColRow> R0047634_model_A1 = { {2458, -1196}, {3284, -1394}, {3183, -1778},
        {2845, -1692}, {2806, -1839}, {2595, -1788} };
    std::vector<ColRow> R0047633_check_A2 = { {3379, -2730}, {3107, -2663} };
    std::vector<ColRow> R0047634_check_A2 = { {2525, -2043}, {2271, -1977} };

    std::vector<ColRow> R0047633_model_B1 = { {1224, -1578}, {1694, -1636}, {1535, -1488},
        {1165, -1417}, {1325, -1570} };
    std::vector<ColRow> R0047634_model_B1 = { {426, -1110}, {904, -1174}, {745, -1022},
        {373, -948}, {534, -1107} };
    std::vector<ColRow> R0047633_check_B2 = { {1452, -1330}, {1272, -1271} };
    std::vector<ColRow> R0047634_check_B2 = { {662, -865}, {483, -802} };



    //-----------------------------------------Affine 모델--------------------------------------------------//
    // Affine 변환
    std::cout << "Affine Transformations" << endl << endl;

    //-----------------------------------------A 모델--------------------------------------------------//
    // Affine A1 -> A2
    cv::Mat_<double> affine_mat_A = estm_affine(R0047633_model_A1, R0047634_model_A1);

    //모델점 정확도 분석
    std::vector<ColRow> cal_model_A1;
    transform_affine(R0047633_model_A1, affine_mat_A, cal_model_A1);
    for (size_t i = 0; i < R0047634_model_A1.size(); ++i)
        print_error_analysis("A1 -> A2 (Affine) model", i, cal_model_A1[i].Col, cal_model_A1[i].Row, R0047634_model_A1[i].Col, R0047634_model_A1[i].Row);
    std::cout << endl << endl;


    // 검사점 정확도 분석
    std::vector<ColRow> cal_check_A2;
    transform_affine(R0047633_check_A2, affine_mat_A, cal_check_A2);
    for (size_t i = 0; i < R0047633_check_A2.size(); ++i)
        print_error_analysis("A1 -> A2 (Affine) check", i, cal_check_A2[i].Col, cal_check_A2[i].Row, R0047634_check_A2[i].Col, R0047634_check_A2[i].Row);
    std::cout << endl << endl;


    //-----------------------------------------B 모델--------------------------------------------------//
    // Affine B1 -> B2
    cv::Mat_<double> affine_mat_B = estm_affine(R0047633_model_B1, R0047634_model_B1);

    //모델점 정확도 분석
    std::vector<ColRow> cal_model_B1;
    transform_affine(R0047633_model_B1, affine_mat_B, cal_model_B1);
    for (size_t i = 0; i < R0047634_model_B1.size(); ++i)
        print_error_analysis("B1 -> B2 (Affine) model", i, cal_model_B1[i].Col, cal_model_B1[i].Row, R0047634_model_B1[i].Col, R0047634_model_B1[i].Row);
    std::cout << endl << endl;


    // 검사점 정확도 분석
    std::vector<ColRow> cal_check_B2;
    transform_affine(R0047633_check_B2, affine_mat_B, cal_check_B2);
    for (size_t i = 0; i < R0047633_check_B2.size(); ++i)
        print_error_analysis("B1 -> B2 (Affine) check", i, cal_check_B2[i].Col, cal_check_B2[i].Row, R0047634_check_B2[i].Col, R0047634_check_B2[i].Row);
    std::cout << endl << endl;

    //-----------------------------------------A + B 모델--------------------------------------------------//
    // Affine AB1 -> AB2
    std::vector<ColRow> comb_model_AB1 = R0047633_model_A1;
    comb_model_AB1.insert(comb_model_AB1.end(), R0047633_model_B1.begin(), R0047633_model_B1.end());

    std::vector<ColRow> comb_model_AB2 = R0047634_model_A1;
    comb_model_AB2.insert(comb_model_AB2.end(), R0047634_model_B1.begin(), R0047634_model_B1.end());

    cv::Mat_<double> affine_mat_AB = estm_affine(comb_model_AB1, comb_model_AB2);

    //모델점 정확도 분석
    std::vector<ColRow> cal_model_AB_A1;
    transform_affine(R0047633_model_A1, affine_mat_AB, cal_model_AB_A1);
    for (size_t i = 0; i < R0047633_model_A1.size(); ++i)
        print_error_analysis("A1 -> A2 (Affine A+B) model", i, cal_model_AB_A1[i].Col, cal_model_AB_A1[i].Row, R0047634_model_A1[i].Col, R0047634_model_A1[i].Row);

    std::vector<ColRow> cal_model_AB_B1;
    transform_affine(R0047633_model_B1, affine_mat_AB, cal_model_AB_B1);
    for (size_t i = 0; i < R0047633_model_B1.size(); ++i)
        print_error_analysis("B1 -> B2 (Affine A+B) model", i, cal_model_AB_B1[i].Col, cal_model_AB_B1[i].Row, R0047634_model_B1[i].Col, R0047634_model_B1[i].Row);

    // 검사점 정확도 분석
    std::vector<ColRow> cal_check_AB_A2;
    transform_affine(R0047633_check_A2, affine_mat_AB, cal_check_AB_A2);
    for (size_t i = 0; i < R0047633_check_A2.size(); ++i)
        print_error_analysis("A1 -> A2 (Affine A+B) check", i, cal_check_AB_A2[i].Col, cal_check_AB_A2[i].Row, R0047634_check_A2[i].Col, R0047634_check_A2[i].Row);

    std::vector<ColRow> cal_check_AB_B2;
    transform_affine(R0047633_check_B2, affine_mat_AB, cal_check_AB_B2);
    for (size_t i = 0; i < R0047633_check_B2.size(); ++i)
        print_error_analysis("B1 -> B2 (Affine A+B) check", i, cal_check_AB_B2[i].Col, cal_check_AB_B2[i].Row, R0047634_check_B2[i].Col, R0047634_check_B2[i].Row);

    std::cout << endl << endl;

    //-----------------------------------------Homography 모델--------------------------------------------------//
    // homography 변환
    std::cout << "homography Transformations" << endl << endl;

    //-----------------------------------------A 모델--------------------------------------------------//
    // homography A1 -> A2
    cv::Mat_<double> homography_mat_A = estm_homography(R0047633_model_A1, R0047634_model_A1);

    //모델점 정확도 분석
    //std::vector<ColRow> cal_model_A1;
    transform_homography(R0047633_model_A1, homography_mat_A, cal_model_A1);
    for (size_t i = 0; i < R0047634_model_A1.size(); ++i)
        print_error_analysis("A1 -> A2 (homography) model", i, cal_model_A1[i].Col, cal_model_A1[i].Row, R0047634_model_A1[i].Col, R0047634_model_A1[i].Row);
    std::cout << endl << endl;


    // 검사점 정확도 분석
    //std::vector<ColRow> cal_check_A2;
    transform_homography(R0047633_check_A2, homography_mat_A, cal_check_A2);
    for (size_t i = 0; i < R0047633_check_A2.size(); ++i)
        print_error_analysis("A1 -> A2 (homography) check", i, cal_check_A2[i].Col, cal_check_A2[i].Row, R0047634_check_A2[i].Col, R0047634_check_A2[i].Row);
    std::cout << endl << endl;


    //-----------------------------------------B 모델--------------------------------------------------//
    // homography B1 -> B2
    cv::Mat_<double> homography_mat_B = estm_homography(R0047633_model_B1, R0047634_model_B1);

    //모델점 정확도 분석
    //std::vector<ColRow> cal_model_B1;
    transform_homography(R0047633_model_B1, homography_mat_B, cal_model_B1);
    for (size_t i = 0; i < R0047634_model_B1.size(); ++i)
        print_error_analysis("B1 -> B2 (homography) model", i, cal_model_B1[i].Col, cal_model_B1[i].Row, R0047634_model_B1[i].Col, R0047634_model_B1[i].Row);
    std::cout << endl << endl;


    // 검사점 정확도 분석
    //std::vector<ColRow> cal_check_B2;
    transform_homography(R0047633_check_B2, homography_mat_B, cal_check_B2);
    for (size_t i = 0; i < R0047633_check_B2.size(); ++i)
        print_error_analysis("B1 -> B2 (homography) check", i, cal_check_B2[i].Col, cal_check_B2[i].Row, R0047634_check_B2[i].Col, R0047634_check_B2[i].Row);
    std::cout << endl << endl;

    //-----------------------------------------A + B 모델--------------------------------------------------//
    // homography AB1 -> AB2
    //std::vector<ColRow> comb_model_AB1 = R0047633_model_A1;
    //comb_model_AB1.insert(comb_model_AB1.end(), R0047633_model_B1.begin(), R0047633_model_B1.end());

    //std::vector<ColRow> comb_model_AB2 = R0047634_model_A1;
    //comb_model_AB2.insert(comb_model_AB2.end(), R0047634_model_B1.begin(), R0047634_model_B1.end());

    cv::Mat_<double> homography_mat_AB = estm_homography(comb_model_AB1, comb_model_AB2);

    //모델점 정확도 분석
    //std::vector<ColRow> cal_model_AB_A1;
    transform_homography(R0047633_model_A1, homography_mat_AB, cal_model_AB_A1);
    for (size_t i = 0; i < R0047633_model_A1.size(); ++i)
        print_error_analysis("A1 -> A2 (homography A+B) model", i, cal_model_AB_A1[i].Col, cal_model_AB_A1[i].Row, R0047634_model_A1[i].Col, R0047634_model_A1[i].Row);

    //std::vector<ColRow> cal_model_AB_B1;
    transform_homography(R0047633_model_B1, homography_mat_AB, cal_model_AB_B1);
    for (size_t i = 0; i < R0047633_model_B1.size(); ++i)
        print_error_analysis("B1 -> B2 (homography A+B) model", i, cal_model_AB_B1[i].Col, cal_model_AB_B1[i].Row, R0047634_model_B1[i].Col, R0047634_model_B1[i].Row);

    // 검사점 정확도 분석
    //std::vector<ColRow> cal_check_AB_A2;
    transform_homography(R0047633_check_A2, homography_mat_AB, cal_check_AB_A2);
    for (size_t i = 0; i < R0047633_check_A2.size(); ++i)
        print_error_analysis("A1 -> A2 (homography A+B) check", i, cal_check_AB_A2[i].Col, cal_check_AB_A2[i].Row, R0047634_check_A2[i].Col, R0047634_check_A2[i].Row);

    //std::vector<ColRow> cal_check_AB_B2;
    transform_homography(R0047633_check_B2, homography_mat_AB, cal_check_AB_B2);
    for (size_t i = 0; i < R0047633_check_B2.size(); ++i)
        print_error_analysis("B1 -> B2 (homography A+B) check", i, cal_check_AB_B2[i].Col, cal_check_AB_B2[i].Row, R0047634_check_B2[i].Col, R0047634_check_B2[i].Row);

    std::cout << endl << endl;

    //--------------------------------------------end------------------------------------------------------//
    // 변환 결과 이미지 출력
    cv::Mat img1 = cv::imread(file_path + image_file_name_1);
    cv::Mat img2 = cv::imread(file_path + image_file_name_2);

    cv::Mat transformed_img_affine;
    cv::warpAffine(img1, transformed_img_affine, affine_mat_AB(cv::Rect(0, 0, 3, 2)), img2.size());

    cv::Mat transformed_img_homography;
    cv::Mat homography_param_A_32f;
    homography_mat_AB.convertTo(homography_param_A_32f, CV_32F);
    cv::warpPerspective(img1, transformed_img_homography, homography_param_A_32f, img2.size());
}

void MidProject2()
{
    std::string rsc_path = "resource/";
    std::vector<std::string> img_name_list = { "R0047633.JPG", "R0047634.JPG", "R0047635.JPG" };
    std::string GCP_name = rsc_path + "GCP.txt";
    std::string IOP_name = rsc_path + "IOFile_Metric.txt";

    std::vector<GCP> ImageGCPs;
    ReadGCP(GCP_name, ImageGCPs);

    IOP ImageIOP;
    ReadIOP(IOP_name, ImageIOP);

    for (int iter_img = 0; iter_img < img_name_list.size(); iter_img++)
    {
        const int numb_model = 6;

        std::string image_dir = rsc_path + img_name_list[iter_img];
        std::vector<GCP> ImageGCP;
        ReadImageGCP(ImageGCPs, img_name_list[iter_img], ImageGCP);

        std::vector<XYZ> GCP_XYZ(ImageGCP.size());
        std::vector<ColRow> GCP_CR(ImageGCP.size());

        for (int iter_GCP = 0; iter_GCP < ImageGCP.size(); iter_GCP++) {
            GCP_XYZ[iter_GCP] = ImageGCP[iter_GCP].xyz;
            GCP_CR[iter_GCP] = ImageGCP[iter_GCP].order[0].CR;
        }

        std::vector<XYZ> model_XYZ(GCP_XYZ.begin(), GCP_XYZ.begin() + numb_model);
        std::vector<XYZ> check_XYZ(GCP_XYZ.begin() + numb_model, GCP_XYZ.end());
        
        std::vector<ColRow> model_CR(GCP_CR.begin(), GCP_CR.begin() + numb_model);
        std::vector<ColRow> check_CR(GCP_CR.begin() + numb_model, GCP_CR.end());

        std::vector<XYZ> model_xy;
        CR2xy(ImageIOP, model_CR, model_xy);

        cv::Mat_<double> DLT_XYZ2CR_matrix = estm_DLT_XYZ2CR(model_XYZ, model_CR);
        cv::Mat_<double> DLT_XYZ2xy_matrix = estm_DLT_XYZ2xy(model_XYZ, model_xy);

        // Transform
        std::cout << "DLT XYZ -> CR" << endl << endl;
        std::vector<ColRow> cal_check_CR;
        transform_DLT_XYZ2CR(check_XYZ, DLT_XYZ2CR_matrix, cal_check_CR);
        for (int iter_check = 0; iter_check < check_XYZ.size(); iter_check++) {
            print_error_analysis(img_name_list[iter_img], iter_check, cal_check_CR[iter_check].Col, cal_check_CR[iter_check].Row, check_CR[iter_check].Col, check_CR[iter_check].Row);
        }
        std::cout << endl << endl;


        std::cout << "DLT [XYZ -> xy] -> CR" << endl << endl;
        std::vector<XYZ> cal_check_xy;
        std::vector<ColRow> cal_check_xy2CR;
        transform_DLT_XYZ2xy(check_XYZ, DLT_XYZ2xy_matrix, cal_check_xy);

        xy2CR(ImageIOP, cal_check_xy, cal_check_xy2CR);

        for (int iter_check = 0; iter_check < check_XYZ.size(); iter_check++) {
            print_error_analysis(img_name_list[iter_img], iter_check, cal_check_xy2CR[iter_check].Col, cal_check_xy2CR[iter_check].Row, check_CR[iter_check].Col, check_CR[iter_check].Row);
        }
        std::cout << endl << endl;
    }
}

void MidProject3()
{
    const int numb_model = 5;
    std::string rsc_path = "resource/";
    std::vector<std::string> img_name_list = {"R0047633.JPG"};
    std::string GCP_name = rsc_path + "GCP.txt";
    std::string IOP_name = rsc_path + "IOFile_Metric.txt";
    std::string EOP_name = rsc_path + "input_EOP.txt";

    std::vector<GCP> ImageGCPs;
    std::vector<GCP> ImageGCP;
    ReadGCP(GCP_name, ImageGCPs);

    IOP ImageIOP;
    ReadIOP(IOP_name, ImageIOP);

    std::vector<EOP> ImageEOPs;
    EOP ImageEOP;
    ReadEOP(EOP_name, ImageEOPs);

    ReadImageEOP(ImageEOPs, img_name_list[0], ImageEOP);
    ReadImageGCP(ImageGCPs, img_name_list[0], ImageGCP);

    std::vector<GCP> modelGCP(ImageGCP.begin(), ImageGCP.begin() + numb_model);
    std::vector<GCP> checkGCP(ImageGCP.begin() + numb_model, ImageGCP.end());

    cv::Mat_<double> RotationMat = cv::Mat::zeros(3, 3, CV_64FC1);
    double omega = ImageEOP.O * ToRad;
    double phi = ImageEOP.P * ToRad;
    double kappa = ImageEOP.K * ToRad;
    RotationMat = GetRotationMatrix(omega, phi, kappa);


    std::cout << "Non-Calibrated\n\n";
    for (int i = 0; i < checkGCP.size(); i++) {
        // GCP XYZ에 기록된 월드 XYZ좌표
        double X = checkGCP[i].xyz.X;
        double Y = checkGCP[i].xyz.Y;
        double Z = checkGCP[i].xyz.Z;

        // GCP에 기록된 픽셀 좌표
        double c = checkGCP[i].order[0].CR.Col;
        double r = checkGCP[i].order[0].CR.Row;

        double cal_c, cal_r;
        // GCP XYZ -> Calc C,R
        InverseMapping(ImageIOP, ImageEOP, RotationMat, X, Y, Z, cal_c, cal_r);


        // Inverce mapping function으로 계산된 픽셀좌표와 GCP 픽셀좌표 비교
        std::cout << "Calc (C,R) = " << cal_c << "\t" << cal_r << endl;
        std::cout << "True (C,R) = " << c << "\t" << r << endl;
        std::cout << "Error X = " << cal_c - c << endl;
        std::cout << "Error Y = " << cal_r - r << endl;
        std::cout << "Error   = " << std::sqrt(std::pow(cal_c - c, 2) + std::pow(cal_r - r, 2)) << endl << endl;
    }


    EOP CalibEOP = calibrate_EOP(ImageIOP, ImageEOP, modelGCP);

    cv::Mat_<double> CalibRotationMat = cv::Mat::zeros(3, 3, CV_64FC1);
    double calib_omega = CalibEOP.O * ToRad;
    double calib_phi = CalibEOP.P * ToRad;
    double calib_kappa = CalibEOP.K * ToRad;
    CalibRotationMat = GetRotationMatrix(calib_omega, calib_phi, calib_kappa);

    std::cout << "Calibrated\n\n";
    for (int i = 0; i < checkGCP.size(); i++) {
        // GCP XYZ에 기록된 월드 XYZ좌표
        double X = checkGCP[i].xyz.X;
        double Y = checkGCP[i].xyz.Y;
        double Z = checkGCP[i].xyz.Z;

        // GCP에 기록된 픽셀 좌표
        double c = checkGCP[i].order[0].CR.Col;
        double r = checkGCP[i].order[0].CR.Row;

        double cal_c, cal_r;
        // GCP XYZ -> Calc C,R
        InverseMapping(ImageIOP, CalibEOP, CalibRotationMat, X, Y, Z, cal_c, cal_r);


        // Inverce mapping function으로 계산된 픽셀좌표와 GCP 픽셀좌표 비교
        std::cout << "Calc (C,R) = " << cal_c << "\t" << cal_r << endl;
        std::cout << "True (C,R) = " << c << "\t" << r << endl;
        std::cout << "Error X = " << cal_c - c << endl;
        std::cout << "Error Y = " << cal_r - r << endl;
        std::cout << "Error   = " << std::sqrt(std::pow(cal_c - c, 2) + std::pow(cal_r - r, 2)) << endl << endl;
    }
}

int _MidProject3(const string& in_image_name, vector<double>& out)
{
    const int numb_model = 5;
    std::string rsc_path = "resource/";
    std::string GCP_name = rsc_path + "GCP.txt";
    std::string IOP_name = rsc_path + "IOFile_Metric.txt";
    std::string EOP_name = rsc_path + "input_EOP.txt";

    std::vector<GCP> ImageGCPs;
    std::vector<GCP> ImageGCP;
    ReadGCP(GCP_name, ImageGCPs);

    IOP ImageIOP;
    ReadIOP(IOP_name, ImageIOP);

    std::vector<EOP> ImageEOPs;
    EOP ImageEOP;
    ReadEOP(EOP_name, ImageEOPs);

    ReadImageEOP(ImageEOPs, in_image_name, ImageEOP);
    ReadImageGCP(ImageGCPs, in_image_name, ImageGCP);

    if (ImageGCP.size() < 6) return 1;

    std::vector<GCP> modelGCP(ImageGCP.begin(), ImageGCP.begin() + numb_model);
    std::vector<GCP> checkGCP(ImageGCP.begin() + numb_model, ImageGCP.end());

    cv::Mat_<double> RotationMat = cv::Mat::zeros(3, 3, CV_64FC1);
    double omega = ImageEOP.O * ToRad;
    double phi = ImageEOP.P * ToRad;
    double kappa = ImageEOP.K * ToRad;
    RotationMat = GetRotationMatrix(omega, phi, kappa);

    double out_1 = 0;
    for (int i = 0; i < modelGCP.size(); i++) {
        // GCP XYZ에 기록된 월드 XYZ좌표
        double X = modelGCP[i].xyz.X;
        double Y = modelGCP[i].xyz.Y;
        double Z = modelGCP[i].xyz.Z;

        // GCP에 기록된 픽셀 좌표
        double c = modelGCP[i].order[0].CR.Col;
        double r = modelGCP[i].order[0].CR.Row;

        double cal_c, cal_r;
        // GCP XYZ -> Calc C,R
        InverseMapping(ImageIOP, ImageEOP, RotationMat, X, Y, Z, cal_c, cal_r);
        out_1 += pow(cal_c - c, 2) + pow(cal_r - r, 2);
    }
    out_1 = sqrt(out_1 / modelGCP.size());

    double out_2 = 0;
    for (int i = 0; i < checkGCP.size(); i++) {
        // GCP XYZ에 기록된 월드 XYZ좌표
        double X = checkGCP[i].xyz.X;
        double Y = checkGCP[i].xyz.Y;
        double Z = checkGCP[i].xyz.Z;

        // GCP에 기록된 픽셀 좌표
        double c = checkGCP[i].order[0].CR.Col;
        double r = checkGCP[i].order[0].CR.Row;

        double cal_c, cal_r;
        // GCP XYZ -> Calc C,R
        InverseMapping(ImageIOP, ImageEOP, RotationMat, X, Y, Z, cal_c, cal_r);
        out_2 += pow(cal_c - c, 2) + pow(cal_r - r, 2);
    }
    out_2 = sqrt(out_2 / checkGCP.size());

    EOP CalibEOP = calibrate_EOP(ImageIOP, ImageEOP, modelGCP);

    cv::Mat_<double> CalibRotationMat = cv::Mat::zeros(3, 3, CV_64FC1);
    double calib_omega = CalibEOP.O * ToRad;
    double calib_phi = CalibEOP.P * ToRad;
    double calib_kappa = CalibEOP.K * ToRad;
    CalibRotationMat = GetRotationMatrix(calib_omega, calib_phi, calib_kappa);

    double out_3 = 0;
    for (int i = 0; i < modelGCP.size(); i++) {
        // GCP XYZ에 기록된 월드 XYZ좌표
        double X = modelGCP[i].xyz.X;
        double Y = modelGCP[i].xyz.Y;
        double Z = modelGCP[i].xyz.Z;

        // GCP에 기록된 픽셀 좌표
        double c = modelGCP[i].order[0].CR.Col;
        double r = modelGCP[i].order[0].CR.Row;

        double cal_c, cal_r;
        // GCP XYZ -> Calc C,R
        InverseMapping(ImageIOP, CalibEOP, CalibRotationMat, X, Y, Z, cal_c, cal_r);
        out_3 += pow(cal_c - c, 2) + pow(cal_r - r, 2);
    }
    out_3 = sqrt(out_3 / modelGCP.size());

    double out_4 = 0;
    for (int i = 0; i < checkGCP.size(); i++) {
        // GCP XYZ에 기록된 월드 XYZ좌표
        double X = checkGCP[i].xyz.X;
        double Y = checkGCP[i].xyz.Y;
        double Z = checkGCP[i].xyz.Z;

        // GCP에 기록된 픽셀 좌표
        double c = checkGCP[i].order[0].CR.Col;
        double r = checkGCP[i].order[0].CR.Row;

        double cal_c, cal_r;
        // GCP XYZ -> Calc C,R
        InverseMapping(ImageIOP, CalibEOP, CalibRotationMat, X, Y, Z, cal_c, cal_r);
        out_4 += pow(cal_c - c, 2) + pow(cal_r - r, 2);
    }
    out_4 = sqrt(out_4 / checkGCP.size());

    out.resize(4);
    out[0] = out_1;
    out[1] = out_2;
    out[2] = out_3;
    out[3] = out_4;
    return 0;
}

void MidProject3_1()
{
    std::string rsc_path = "resource/";
    std::vector<std::string> img_name_list;

    for (const auto& entry : fs::directory_iterator(rsc_path))
    {
        if (entry.is_regular_file())
        {
            std::string filename = entry.path().filename().string();
            if (entry.path().extension() == ".jpg" || entry.path().extension() == ".JPG")
            {
                img_name_list.push_back(filename);
            }
        }
    }

    for (int i = 0; i < img_name_list.size(); i++)
    {
        std::vector<double> res;
        if (!_MidProject3(img_name_list[i], res))
        {
            std::cout << img_name_list[i] << "\t"
                << res[0] << "\t" << res[1] << "\t" << res[2] << "\t" << res[3] << "\n";
        }
    }
}