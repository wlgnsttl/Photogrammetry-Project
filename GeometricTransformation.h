#ifndef GEOMETRIC_TRANSFORMATION_H
#define GEOMETRIC_TRANSFORMATION_H

#include "ReadParam.h"
#include "FrameCamera.h"

// Affine
cv::Mat_<double> estm_affine(const std::vector<ColRow>& srcPts, const std::vector<ColRow>& dstPts);
int _transform_affine(const ColRow& pt, const cv::Mat_<double>& affineMat, double& out_c, double& out_r);
int transform_affine(const std::vector<ColRow>& pts, const cv::Mat_<double>& affineMat, std::vector<ColRow>& out_CR);

// Homography
cv::Mat_<double> estm_homography(const std::vector<ColRow>& srcPts, const std::vector<ColRow>& dstPts);
int _transform_homography(const ColRow& pt, const cv::Mat_<double>& homographyMat, double& out_c, double& out_r);
int transform_homography(const std::vector<ColRow>& pts, const cv::Mat_<double>& homographyMat, std::vector<ColRow>& out_CR);

// DLT XYZ to CR
cv::Mat_<double> estm_DLT_XYZ2CR(const std::vector<XYZ>& srcPts, const std::vector<ColRow>& dstPts);
int _transform_DLT_XYZ2CR(const double scale, const XYZ& pt, const cv::Mat_<double>& DLT_Mat, double& out_c, double& out_r);
int transform_DLT_XYZ2CR(const std::vector<XYZ>& pts, const cv::Mat_<double>& DLT_Mat, std::vector<ColRow>& out_cr);

// DLT XYZ to xy
cv::Mat_<double> estm_DLT_XYZ2xy(const std::vector<XYZ>& srcPts, const std::vector<XYZ>& dstPts);
int _transform_DLT_XYZ2xy(const double& scale, const XYZ& pt, const cv::Mat_<double>& DLT_Mat, double& out_x, double& out_y);
int transform_DLT_XYZ2xy(const std::vector<XYZ>& pts, const cv::Mat_<double>& DLT_Mat, std::vector<XYZ>& out_xyz);

// CR to xy
int _CR2xy(const IOP& in_IOP, const ColRow& in_CR, double& out_x, double& out_y, double& out_z);
int CR2xy(const IOP& in_IOP, const std::vector<ColRow>& in_CR, std::vector<XYZ>& out_xyz);

// xy to CR
int _xy2CR(const IOP& in_IOP, const double& x, const double& y, double& out_c, double& out_r);
int xy2CR(const IOP& in_IOP, const std::vector<XYZ>& in_XYZ, std::vector<ColRow>& out_CR);

#endif