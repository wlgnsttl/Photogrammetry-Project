#include "main.h"

cv::Mat_<double> GetRotationMatrix(double in_omega, double in_phi, double in_kappa) {
    // in: 카메라 외부 파라미터 중 오일러 각도 (omega, phi, kappa)
    // out: Rotation Matrix

    cv::Mat_<double> RotationMatrix;

    // 회전 행렬 초기화 (3x3 영행렬)
    cv::Mat_<double> Rotation_X = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat_<double> Rotation_Y = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat_<double> Rotation_Z = cv::Mat::zeros(3, 3, CV_64FC1);

    // 각 축 회전에 대한 삼각함수 값 계산
    double cw = std::cos(in_omega);  // cos(omega)
    double cp = std::cos(in_phi);    // cos(phi)
    double ck = std::cos(in_kappa);  // cos(kappa)
    double sw = std::sin(in_omega);  // sin(omega)
    double sp = std::sin(in_phi);    // sin(phi)
    double sk = std::sin(in_kappa);  // sin(kappa)

    // X축 회전 (Omega)
    Rotation_X(0, 0) = 1.;  Rotation_X(0, 1) = 0.;   Rotation_X(0, 2) = 0.;
    Rotation_X(1, 0) = 0.;  Rotation_X(1, 1) = cw;   Rotation_X(1, 2) = sw;
    Rotation_X(2, 0) = 0.;  Rotation_X(2, 1) = -sw;  Rotation_X(2, 2) = cw;

    // Y축 회전 (Phi)
    Rotation_Y(0, 0) = cp;   Rotation_Y(0, 1) = 0.;  Rotation_Y(0, 2) = -sp;
    Rotation_Y(1, 0) = 0.;   Rotation_Y(1, 1) = 1.;  Rotation_Y(1, 2) = 0.;
    Rotation_Y(2, 0) = sp;   Rotation_Y(2, 1) = 0.;  Rotation_Y(2, 2) = cp;

    // Z축 회전 (Kappa)
    Rotation_Z(0, 0) = ck;   Rotation_Z(0, 1) = sk;   Rotation_Z(0, 2) = 0.;
    Rotation_Z(1, 0) = -sk;  Rotation_Z(1, 1) = ck;   Rotation_Z(1, 2) = 0.;
    Rotation_Z(2, 0) = 0.;   Rotation_Z(2, 1) = 0.;   Rotation_Z(2, 2) = 1.;

    // 최종 회전 행렬 = Rz * Ry * Rx
    RotationMatrix = Rotation_Z * Rotation_Y * Rotation_X;

    return RotationMatrix;
}

int InverseMapping(IOP in_IOP, EOP in_EOP, cv::Mat_<double> in_RotationMatrix,
                   double in_X, double in_Y, double in_Z,
                   double& out_c, double& out_r)
{
    // Rotation 행렬 요소 추출
    double r11 = in_RotationMatrix(0, 0);   double r12 = in_RotationMatrix(0, 1);   double r13 = in_RotationMatrix(0, 2);
    double r21 = in_RotationMatrix(1, 0);   double r22 = in_RotationMatrix(1, 1);   double r23 = in_RotationMatrix(1, 2);
    double r31 = in_RotationMatrix(2, 0);   double r32 = in_RotationMatrix(2, 1);   double r33 = in_RotationMatrix(2, 2);

    // 카메라 중심 좌표
    double Xs = in_EOP.Xs;
    double Ys = in_EOP.Ys;
    double Zs = in_EOP.Zs;

    // 입력 3D 점과 카메라 중심의 벡터 차
    double XX = in_X - Xs;
    double YY = in_Y - Ys;
    double ZZ = in_Z - Zs;

    // DEN
    double Denom = r31 * XX + r32 * YY + r33 * ZZ;

    // x, y 축 방향 투영 값 계산 (numerator)
    double Numer_X = r11 * XX + r12 * YY + r13 * ZZ;
    double Numer_Y = r21 * XX + r22 * YY + r23 * ZZ;

    // 중심투영된 이미지 좌표 (단위: mm)
    double x_prime = -in_IOP.Focal * Numer_X / Denom;
    double y_prime = -in_IOP.Focal * Numer_Y / Denom;

    // Image -> pixel

    // 이미지 중심 픽셀 좌표 (c_0, r_0 단위: pixel)
    double c_0 = in_IOP.ColSize / 2.;
    double r_0 = in_IOP.RowSize / 2.;

    // v픽셀 중심좌표를 기준으로 픽셀 단위로 변환
    double c_prime = (x_prime / in_IOP.CellSize) + c_0;
    double r_prime = -(y_prime / in_IOP.CellSize) + r_0;

    // 최종 결과 전달
    out_c = c_prime;
    out_r = r_prime;

    return 0; // 정상 종료
}

EOP calibrate_EOP(const IOP& in_IOP, const EOP& in_EOP, const std::vector<GCP>& in_GCPs) {
    const size_t numb_GCPs = in_GCPs.size();
    CV_Assert(numb_GCPs >= 3);

    const int MAX_ITER = 100;
    const double EPSILON = 1e-5;

    EOP estm_EOP = in_EOP;

    for (int iter = 0; iter < MAX_ITER; iter++) {
        cv::Mat_<double> H(2 * numb_GCPs, 6);
        cv::Mat_<double> L(2 * numb_GCPs, 1);

        double Xs = estm_EOP.Xs;
        double Ys = estm_EOP.Ys;
        double Zs = estm_EOP.Zs;

        double O = estm_EOP.O * ToRad;
        double P = estm_EOP.P * ToRad;
        double K = estm_EOP.K * ToRad;

        const double sin_w = std::sin(O);
        const double cos_w = std::cos(O);
        const double sin_p = std::sin(P);
        const double cos_p = std::cos(P);
        const double sin_k = std::sin(K);
        const double cos_k = std::cos(K);

        cv::Mat_<double> RotationMat = GetRotationMatrix(O, P, K);

        double r11 = RotationMat(0, 0), r12 = RotationMat(0, 1), r13 = RotationMat(0, 2);
        double r21 = RotationMat(1, 0), r22 = RotationMat(1, 1), r23 = RotationMat(1, 2);
        double r31 = RotationMat(2, 0), r32 = RotationMat(2, 1), r33 = RotationMat(2, 2);

        cv::Mat_<double> r_w = cv::Mat::zeros(3, 3, CV_64FC1);
        cv::Mat_<double> r_p = cv::Mat::zeros(3, 3, CV_64FC1);
        cv::Mat_<double> r_k = cv::Mat::zeros(3, 3, CV_64FC1);

        r_w(0, 0) = 0;      r_w(0, 1) = -r13;   r_w(0, 2) = r12;
        r_w(1, 0) = 0;      r_w(1, 1) = -r23;   r_w(1, 2) = r22;
        r_w(2, 0) = 0;      r_w(2, 1) = -r33;   r_w(2, 2) = r32;

        r_p(0, 0) = -r31 * cos_k;  r_p(0, 1) = r11 * sin_w;    r_p(0, 2) = -r11 * cos_w;
        r_p(1, 0) = r31 * sin_k;   r_p(1, 1) = r21 * sin_w;    r_p(1, 2) = r33 * sin_k;
        r_p(2, 0) = cos_p;         r_p(2, 1) = r31 * sin_w;    r_p(2, 2) = -r31 * cos_w;

        r_k(0, 0) = r21;    r_k(0, 1) = r22;    r_k(0, 2) = r23;
        r_k(1, 0) = -r11;   r_k(1, 1) = -r12;   r_k(1, 2) = -r13;
        r_k(2, 0) = 0;      r_k(2, 1) = 0;      r_k(2, 2) = 0;

        for (int i = 0; i < numb_GCPs; i++) {
            double Xp = in_GCPs[i].xyz.X;
            double Yp = in_GCPs[i].xyz.Y;
            double Zp = in_GCPs[i].xyz.Z;

            double x, y, z;
            _CR2xy(in_IOP, in_GCPs[i].order[0].CR, x, y, z);

            cv::Mat_<double> vec_world = (cv::Mat_<double>(3, 1) << Xp - Xs, Yp - Ys, Zp - Zs);
            cv::Mat_<double> vec_cam = RotationMat * vec_world;

            double Rxx = vec_cam(0, 0);
            double Ryy = vec_cam(1, 0);
            double Rzz = vec_cam(2, 0);

            double Rxx_Xs = -r11, Rxx_Ys = -r12, Rxx_Zs = -r13;
            double Ryy_Xs = -r21, Ryy_Ys = -r22, Ryy_Zs = -r23;
            double Rzz_Xs = -r31, Rzz_Ys = -r32, Rzz_Zs = -r33;

            cv::Mat_<double> Rw = r_w * vec_world; // 간결화: r_w * vec_world
            cv::Mat_<double> Rp = r_p * vec_world; // 간결화: r_p * vec_world
            cv::Mat_<double> Rk = r_k * vec_world; // 간결화: r_k * vec_world

            double Rxx_w = Rw(0, 0), Ryy_w = Rw(1, 0), Rzz_w = Rw(2, 0);
            double Rxx_p = Rp(0, 0), Ryy_p = Rp(1, 0), Rzz_p = Rp(2, 0);
            double Rxx_k = Rk(0, 0), Ryy_k = Rk(1, 0), Rzz_k = Rk(2, 0);

            double c_x = -in_IOP.Focal * Rxx / Rzz;
            double c_y = -in_IOP.Focal * Ryy / Rzz;

            double Fx = -x + c_x;
            double Fy = -y + c_y;

            double f = in_IOP.Focal;
            double invZ2 = 1.0 / (Rzz * Rzz);

            // 간결화: 중복 항 계산을 변수로 정리
            double cXs = (Rxx_Xs * Rzz - Rxx * Rzz_Xs) * invZ2;
            double cYs = (Rxx_Ys * Rzz - Rxx * Rzz_Ys) * invZ2;
            double cZs = (Rxx_Zs * Rzz - Rxx * Rzz_Zs) * invZ2;
            double cXs_y = (Ryy_Xs * Rzz - Ryy * Rzz_Xs) * invZ2;
            double cYs_y = (Ryy_Ys * Rzz - Ryy * Rzz_Ys) * invZ2;
            double cZs_y = (Ryy_Zs * Rzz - Ryy * Rzz_Zs) * invZ2;

            double cOw = (Rxx_w * Rzz - Rxx * Rzz_w) * invZ2;
            double cOp = (Rxx_p * Rzz - Rxx * Rzz_p) * invZ2;
            double cOk = (Rxx_k * Rzz - Rxx * Rzz_k) * invZ2;
            double cOw_y = (Ryy_w * Rzz - Ryy * Rzz_w) * invZ2;
            double cOp_y = (Ryy_p * Rzz - Ryy * Rzz_p) * invZ2;
            double cOk_y = (Ryy_k * Rzz - Ryy * Rzz_k) * invZ2;

            H(2 * i, 0) = -f * cXs;
            H(2 * i, 1) = -f * cYs;
            H(2 * i, 2) = -f * cZs;
            H(2 * i + 1, 0) = -f * cXs_y;
            H(2 * i + 1, 1) = -f * cYs_y;
            H(2 * i + 1, 2) = -f * cZs_y;

            H(2 * i, 3) = -f * cOw;
            H(2 * i, 4) = -f * cOp;
            H(2 * i, 5) = -f * cOk;
            H(2 * i + 1, 3) = -f * cOw_y;
            H(2 * i + 1, 4) = -f * cOp_y;
            H(2 * i + 1, 5) = -f * cOk_y;

            L(2 * i, 0) = -Fx;
            L(2 * i + 1, 0) = -Fy;
        }

        cv::Mat_<double> delta;
        cv::solve(H, L, delta, cv::DECOMP_QR);

        estm_EOP.Xs += delta(0, 0);
        estm_EOP.Ys += delta(1, 0);
        estm_EOP.Zs += delta(2, 0);
        estm_EOP.O += delta(3, 0) * ToDeg;
        estm_EOP.P += delta(4, 0) * ToDeg;
        estm_EOP.K += delta(5, 0) * ToDeg;

        if (cv::norm(delta) < EPSILON)
            break;
    }

    return estm_EOP;
}



