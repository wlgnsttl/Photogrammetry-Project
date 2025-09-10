#include "GeometricTransformation.h"

cv::Mat_<double> estm_affine(const std::vector<ColRow>& srcPts,
						     const std::vector<ColRow>& dstPts)
{
	cv::Mat_<double> estm_mat = cv::Mat::zeros(3, 3, CV_64FC1);

    size_t numb_src = srcPts.size();
    size_t numb_dst = dstPts.size();

	CV_Assert(numb_src >= 3);
	CV_Assert(numb_src == numb_dst);

    cv::Mat_<double> A = cv::Mat::zeros(2 * static_cast<int>(numb_src), 6, CV_64FC1);
    cv::Mat_<double> b = cv::Mat::zeros(2 * static_cast<int>(numb_src), 1, CV_64FC1);

    for (int i = 0; i < numb_src; ++i) {
        double x = srcPts[i].Col;
        double y = srcPts[i].Row;
        double xp = dstPts[i].Col;
        double yp = dstPts[i].Row;

        A(2 * i, 0) = x;  A(2 * i, 1) = y;  A(2 * i, 2) = 1;
        A(2 * i, 3) = 0;  A(2 * i, 4) = 0;  A(2 * i, 5) = 0;

        A(2 * i + 1, 0) = 0;  A(2 * i + 1, 1) = 0;  A(2 * i + 1, 2) = 0;
        A(2 * i + 1, 3) = x;  A(2 * i + 1, 4) = y;  A(2 * i + 1, 5) = 1;

        b(2 * i, 0) = xp;
        b(2 * i + 1, 0) = yp;
    }

    cv::Mat_<double> p;
    cv::solve(A, b, p, cv::DECOMP_QR);

    estm_mat(0, 0) = p(0, 0);  // a11
    estm_mat(0, 1) = p(1, 0);  // a12
    estm_mat(0, 2) = p(2, 0);  // tx
    estm_mat(1, 0) = p(3, 0);  // a21
    estm_mat(1, 1) = p(4, 0);  // a22
    estm_mat(1, 2) = p(5, 0);  // ty
    estm_mat(2, 2) = 1;

    return estm_mat;
}
int _transform_affine(const ColRow& pt, const cv::Mat_<double>& affineMat,
                      double& out_c, double& out_r)
{
    out_c = affineMat(0, 0) * pt.Col + affineMat(0, 1) * pt.Row + affineMat(0, 2);
    out_r = affineMat(1, 0) * pt.Col + affineMat(1, 1) * pt.Row + affineMat(1, 2);

    return 0;
}
int transform_affine(const std::vector<ColRow>& pts, const cv::Mat_<double>& affineMat,
                     std::vector<ColRow>& out_CR)
{
    const size_t length = pts.size();
    out_CR.resize(length);

    for (int i = 0; i < length; i++) {
        double col, row;
        _transform_affine(pts[i], affineMat, col, row);
        out_CR[i].Col = col;
        out_CR[i].Row = row;
    }
    return 0;
}


cv::Mat_<double> estm_homography(const std::vector<ColRow>& srcPts,
                                 const std::vector<ColRow>& dstPts)
{
    cv::Mat_<double> estm_mat = cv::Mat::zeros(3, 3, CV_64FC1);

    size_t numb_src = srcPts.size();
    size_t numb_dst = dstPts.size();

    CV_Assert(numb_src >= 4);  // √÷º“ 4Ω÷ « ø‰
    CV_Assert(numb_src == numb_dst);

    cv::Mat_<double> A = cv::Mat::zeros(2 * static_cast<int>(numb_src), 8, CV_64FC1);
    cv::Mat_<double> b = cv::Mat::zeros(2 * static_cast<int>(numb_src), 1, CV_64FC1);

    for (int i = 0; i < numb_src; ++i) {
        double x = srcPts[i].Col;
        double y = srcPts[i].Row;
        double xp = dstPts[i].Col;
        double yp = dstPts[i].Row;

        A(2 * i, 0) = x;
        A(2 * i, 1) = y;
        A(2 * i, 2) = 1;
        A(2 * i, 6) = -x * xp;
        A(2 * i, 7) = -y * xp;

        A(2 * i + 1, 3) = x;
        A(2 * i + 1, 4) = y;
        A(2 * i + 1, 5) = 1;
        A(2 * i + 1, 6) = -x * yp;
        A(2 * i + 1, 7) = -y * yp;

        b(2 * i, 0) = xp;
        b(2 * i + 1, 0) = yp;
    }

    cv::Mat_<double> p;
    cv::solve(A, b, p, cv::DECOMP_QR);

    estm_mat(0, 0) = p(0, 0);  // h11
    estm_mat(0, 1) = p(1, 0);  // h12
    estm_mat(0, 2) = p(2, 0);  // h13
    estm_mat(1, 0) = p(3, 0);  // h21
    estm_mat(1, 1) = p(4, 0);  // h22
    estm_mat(1, 2) = p(5, 0);  // h23
    estm_mat(2, 0) = p(6, 0);  // h31
    estm_mat(2, 1) = p(7, 0);  // h32
    estm_mat(2, 2) = 1;        // h33

    return estm_mat;
}
int _transform_homography(const ColRow& pt, const cv::Mat_<double>& homographyMat,
                          double& out_c, double& out_r)
{
    double normal = homographyMat(2, 0) * pt.Col + homographyMat(2, 1) * pt.Row + homographyMat(2, 2);

    out_c = (homographyMat(0, 0) * pt.Col + homographyMat(0, 1) * pt.Row + homographyMat(0, 2))/normal;
    out_r = (homographyMat(1, 0) * pt.Col + homographyMat(1, 1) * pt.Row + homographyMat(1, 2))/normal;

    return 0;
}
int transform_homography(const std::vector<ColRow>& pts, const cv::Mat_<double>& homographyMat,
                         std::vector<ColRow>& out_CR)
{
    const size_t length = pts.size();
    out_CR.resize(length);

    for (int i = 0; i < length; i++) {
        double col, row;
        _transform_homography(pts[i], homographyMat, col, row);
        out_CR[i].Col = col;
        out_CR[i].Row = row;
    }
    return 0;
}


cv::Mat_<double> estm_DLT_XYZ2CR(const std::vector<XYZ>& srcPts,
                                 const std::vector<ColRow>& dstPts)
{
    const double scale = 1000.0;

    size_t numb_src = srcPts.size();
    size_t numb_dst = dstPts.size();

    CV_Assert(numb_src >= 6);
    CV_Assert(numb_src == numb_dst);

    cv::Mat_<double> A = cv::Mat::zeros(2 * static_cast<int>(numb_src), 11, CV_64FC1);
    cv::Mat_<double> b = cv::Mat::zeros(2 * static_cast<int>(numb_src), 1, CV_64FC1);

    for (int i = 0; i < numb_src; ++i) {
        double x = srcPts[i].X / scale;
        double y = srcPts[i].Y / scale;
        double z = srcPts[i].Z / scale;

        double xp = dstPts[i].Col;
        double yp = dstPts[i].Row;

        A(2 * i, 0) = x;
        A(2 * i, 1) = y;
        A(2 * i, 2) = z;
        A(2 * i, 3) = 1;
        A(2 * i, 8) = -x * xp;
        A(2 * i, 9) = -y * xp;
        A(2 * i, 10) = -z * xp;

        A(2 * i + 1, 4) = x;
        A(2 * i + 1, 5) = y;
        A(2 * i + 1, 6) = z;
        A(2 * i + 1, 7) = 1;
        A(2 * i + 1, 8) = -x * yp;
        A(2 * i + 1, 9) = -y * yp;
        A(2 * i + 1, 10) = -z * yp;

        b(2 * i, 0) = xp;
        b(2 * i + 1, 0) = yp;
    }

    cv::Mat_<double> p;
    cv::solve(A, b, p, cv::DECOMP_QR);

    cv::Mat_<double> estm_mat = cv::Mat::zeros(3, 4, CV_64FC1);

    estm_mat(0, 0) = p(0, 0);  // m11
    estm_mat(0, 1) = p(1, 0);  // m12
    estm_mat(0, 2) = p(2, 0);  // m13
    estm_mat(0, 3) = p(3, 0);  // m14

    estm_mat(1, 0) = p(4, 0);  // m21
    estm_mat(1, 1) = p(5, 0);  // m22
    estm_mat(1, 2) = p(6, 0);  // m23
    estm_mat(1, 3) = p(7, 0);  // m24

    estm_mat(2, 0) = p(8, 0);  // m31
    estm_mat(2, 1) = p(9, 0);  // m32
    estm_mat(2, 2) = p(10, 0); // m33
    estm_mat(2, 3) = 1;        // m34

    return estm_mat;
}
int _transform_DLT_XYZ2CR(const double scale, const XYZ& pt, const cv::Mat_<double>& DLT_Mat,
                          double& out_c, double& out_r)
{
    double X = pt.X / scale;
    double Y = pt.Y / scale;
    double Z = pt.Z / scale;

    double normal = DLT_Mat(2, 0) * X + DLT_Mat(2, 1) * Y + DLT_Mat(2, 2) * Z + DLT_Mat(2, 3);

    out_c = (DLT_Mat(0, 0) * X + DLT_Mat(0, 1) * Y + DLT_Mat(0, 2) * Z + DLT_Mat(0, 3)) / normal;
    out_r = (DLT_Mat(1, 0) * X + DLT_Mat(1, 1) * Y + DLT_Mat(1, 2) * Z + DLT_Mat(1, 3)) / normal;

    return 0;
}
int transform_DLT_XYZ2CR(const std::vector<XYZ>& pts, const cv::Mat_<double>& DLT_Mat,
                         std::vector<ColRow>& out_cr)
{
    const double scale = 1000.0;  // XYZ normalization scale

    const size_t length_xyz = pts.size();
    out_cr.resize(length_xyz);

    for (int i = 0; i < length_xyz; i++) {
        double col, row;
        _transform_DLT_XYZ2CR(scale, pts[i], DLT_Mat, col, row);
        out_cr[i].Col = col;
        out_cr[i].Row = row;
    }

    return 0;
}


cv::Mat_<double> estm_DLT_XYZ2xy(const std::vector<XYZ>& srcPts,
                                 const std::vector<XYZ>& dstPts)
{
    const double scale = 1000.0;

    size_t numb_src = srcPts.size();
    size_t numb_dst = dstPts.size();

    CV_Assert(numb_src >= 6);
    CV_Assert(numb_src == numb_dst);

    cv::Mat_<double> A = cv::Mat::zeros(2 * static_cast<int>(numb_src), 11, CV_64FC1);
    cv::Mat_<double> b = cv::Mat::zeros(2 * static_cast<int>(numb_src), 1, CV_64FC1);

    for (int i = 0; i < numb_src; ++i) {
        double x = srcPts[i].X / scale;
        double y = srcPts[i].Y / scale;
        double z = srcPts[i].Z / scale;

        double xp = dstPts[i].X;
        double yp = dstPts[i].Y;

        A(2 * i, 0) = x;
        A(2 * i, 1) = y;
        A(2 * i, 2) = z;
        A(2 * i, 3) = 1;
        A(2 * i, 8) = -x * xp;
        A(2 * i, 9) = -y * xp;
        A(2 * i, 10) = -z * xp;

        A(2 * i + 1, 4) = x;
        A(2 * i + 1, 5) = y;
        A(2 * i + 1, 6) = z;
        A(2 * i + 1, 7) = 1;
        A(2 * i + 1, 8) = -x * yp;
        A(2 * i + 1, 9) = -y * yp;
        A(2 * i + 1, 10) = -z * yp;

        b(2 * i, 0) = xp;
        b(2 * i + 1, 0) = yp;
    }

    cv::Mat_<double> p;
    cv::solve(A, b, p, cv::DECOMP_QR);

    cv::Mat_<double> estm_mat = cv::Mat::zeros(3, 4, CV_64FC1);

    estm_mat(0, 0) = p(0, 0);  // m11
    estm_mat(0, 1) = p(1, 0);  // m12
    estm_mat(0, 2) = p(2, 0);  // m13
    estm_mat(0, 3) = p(3, 0);  // m14

    estm_mat(1, 0) = p(4, 0);  // m21
    estm_mat(1, 1) = p(5, 0);  // m22
    estm_mat(1, 2) = p(6, 0);  // m23
    estm_mat(1, 3) = p(7, 0);  // m24

    estm_mat(2, 0) = p(8, 0);  // m31
    estm_mat(2, 1) = p(9, 0);  // m32
    estm_mat(2, 2) = p(10, 0); // m33
    estm_mat(2, 3) = 1;        // m34

    return estm_mat;
}
int _transform_DLT_XYZ2xy(const double& scale, const XYZ& pt, const cv::Mat_<double>& DLT_Mat,
                          double& out_x, double& out_y)
{
    double X = pt.X / scale;
    double Y = pt.Y / scale;
    double Z = pt.Z / scale;

    double normal = DLT_Mat(2, 0) * X + DLT_Mat(2, 1) * Y + DLT_Mat(2, 2) * Z + DLT_Mat(2, 3);

    out_x = (DLT_Mat(0, 0) * X + DLT_Mat(0, 1) * Y + DLT_Mat(0, 2) * Z + DLT_Mat(0, 3)) / normal;
    out_y = (DLT_Mat(1, 0) * X + DLT_Mat(1, 1) * Y + DLT_Mat(1, 2) * Z + DLT_Mat(1, 3)) / normal;

    return 0;
}
int transform_DLT_XYZ2xy(const std::vector<XYZ>& pts, const cv::Mat_<double>& DLT_Mat,
                         std::vector<XYZ>& out_xyz)
{
    const double scale = 1000.0;  // XYZ normalization scale

    const size_t length_xyz = pts.size();
    out_xyz.resize(length_xyz);

    for (int i = 0; i < length_xyz; i++) {
        double x, y;
        _transform_DLT_XYZ2xy(scale, pts[i], DLT_Mat, x, y);
        out_xyz[i].X = x;
        out_xyz[i].Y = y;
        out_xyz[i].Z = 0;
    }

    return 0;
}


int _CR2xy(const IOP& in_IOP, const ColRow& in_CR, double& out_x, double& out_y, double& out_z)
{
    const double c_0 = in_IOP.ColSize / 2.0;
    const double r_0 = in_IOP.RowSize / 2.0;
    const double focal_length = in_IOP.Focal;

    out_x = (in_CR.Col - c_0) * in_IOP.CellSize;
    out_y = -(in_CR.Row - r_0) * in_IOP.CellSize;
    out_z = focal_length;

    return 0;
}
int CR2xy(const IOP& in_IOP, const std::vector<ColRow>& in_CR, std::vector<XYZ>& out_xyz)
{
    const size_t length_CR = in_CR.size();
    out_xyz.resize(length_CR);

    for (int i = 0; i < length_CR; i++) {
        double x, y, z;
        _CR2xy(in_IOP, in_CR[i], x, y, z);
        out_xyz[i].X = x;
        out_xyz[i].Y = y;
        out_xyz[i].Z = z;
    }

    return 0;
}


int _xy2CR(const IOP& in_IOP, const double& x, const double& y, double& out_c, double& out_r)
{
    const double c_0 = in_IOP.ColSize / 2.0;
    const double r_0 = in_IOP.RowSize / 2.0;
    const double focal_length = in_IOP.Focal;

    out_c = (x / in_IOP.CellSize) + c_0;
    out_r = -(y / in_IOP.CellSize) + r_0;

    return 0;
}
int xy2CR(const IOP& in_IOP, const std::vector<XYZ>& in_XYZ, std::vector<ColRow>& out_CR)
{
    const size_t length = in_XYZ.size();
    out_CR.resize(length);

    for (size_t i = 0; i < length; ++i) {
        double c, r;
        _xy2CR(in_IOP, in_XYZ[i].X, in_XYZ[i].Y, c, r);
        out_CR[i].Col = c;
        out_CR[i].Row = r;
    }

    return 0;
}

