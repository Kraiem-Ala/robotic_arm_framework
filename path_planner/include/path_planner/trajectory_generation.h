#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <tuple>
using Eigen::MatrixXd;
using Eigen::MatrixXf;

#define Cubic_order         "order4"
#define Quintic_order       "order5"
#define Septic_order        "order7"

// One Joint Robot // 
//----------------------- Cubic order polynomial trajectory : order 3 -----------------------------------------------
// AX = B
// A = inv(X)*B - here we compute matrix A
MatrixXf computeCubicCoeff(float t0, float tf, std::vector<float> vec_q0, std::vector<float> vec_qf);
std::tuple<std::vector<double>, std::vector<double>, std::vector<double>> computeCubicTraj(MatrixXf A, float t0, float tf, int n);
std::tuple<std::vector<float>, std::vector<MatrixXf>, std::vector<MatrixXf>> computeCubicTraj_vect(MatrixXf q0 , MatrixXf qf, MatrixXf q0_d, MatrixXf qf_d, float t0, float tf, int n);
//-------------------------- Quintic order polynomial trajectory : order 5 --------------------------------------------
// AX = B
// A = inv(X)*B - here we compute matrix A
MatrixXf computeQuinticCoeff(float t0, float tf, std::vector<float> vec_q0, std::vector<float> vec_qf);
std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> computeQuinticTraj(MatrixXf A, float t0, float tf, int n);

//----------------------7 order------------------------------------------------
// compute seven degree polynomial applicable for jerk in start time and end to be zero
// for waypoints this type of polynomial is used also (4 POINTS)
// AX = B
// A = inv(X)*B - here we compute matrix A
MatrixXf compute7orderCoeff(float t0, float t1, float t2, float t3, std::vector<float> vec_q0x, std::vector<float> vec_qwx, std::vector<float> vec_q3x);
std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>> computeWaypointTraj(MatrixXf A, float t0, float t3, int n);

