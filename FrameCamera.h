#ifndef FRAME_CAMERA_INCLUDED
#define FRAME_CAMERA_INCLUDED

#include "ReadParam.h"
#include "GeometricTransformation.h"

cv::Mat_<double> GetRotationMatrix(double in_omega, double in_phi, double in_kappa);

int InverseMapping(IOP in_IOP, EOP in_EOP, cv::Mat_<double> in_RotationMatrix,
                   double in_X, double in_Y, double in_Z,
                   double& out_c, double& out_r);

EOP calibrate_EOP(const IOP& in_IOP, const EOP& in_EOP, const std::vector<GCP>& in_GCPs);

#endif
