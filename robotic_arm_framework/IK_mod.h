#pragma once
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <vector>
# define M_PI           3.14159265358979323846
using Eigen::Map;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Vector3f;
using Eigen::VectorXf;
/**
* @breif Calculate the rotation Matrix on the Z axis
* @param[theta] rotation angle over Z
* @return Rotation matrix of Yaw
*/
MatrixXf RotYaw(double theta);


/**
* @breif Calculate the rotation Matrix on the Y axis
* @param[theta] rotation angle over Y
* @return Rotation matrix of Pitch
*/
MatrixXf RotPitch(double theta);


/**
* @breif Calculate the rotation Matrix on the X axis
* @param[theta] rotation angle over X
* @return Rotation matrix of Roll
*/
MatrixXf RotRoll(double theta);

/**
* @breif Calculate the Identity Matrix
* @return Identity Matrix type MatrixXf
*/
MatrixXf Idendity();

/**
* @breif Calculate the Transformation Matrix using DH table
* @param[alfa,theta,r,d] DH parameters of the Link 
* @return Transformation Matrix type MatrixXf
*/
MatrixXf Transformation_mat(double alfa, double theta, double r, double d);


/**
* @breif converts Vector to a matrix
* @param[v] Vector
* @return Projected Matrix type MatrixXf
*/
MatrixXf projections(std::vector<float> v);


/**
* @breif Calculate the Jacobian Matrix
* @param[theta_i] joint' angles
* @return Jacobian Matrix of the robot type MatrixXf
*/
MatrixXf jacobian(double theta1, double theta2, double theta3);

/**
* @breif Calculate the Jacobian Matrix
* @param[tfull_jacobianheta_i] joint' angles
* @return Jacobian Matrix of the robot type MatrixXf
*/
MatrixXf jacobian_vect(MatrixXf theta, std::vector<double> alpha, std::vector<double> r , std::vector<double> d);
MatrixXf jacobian(std::vector<double> theta, MatrixXf DH_table);
MatrixXf full_jacobian(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d);


/**
* @breif Calculate the Pseudo Inverse or Inverse of a jacobian Matrix
* @param[theta_i] joints' angles
* @return PseudoInverse Matrix type MatrixXf
*/
MatrixXf computePseudoInverse(float theta1, float theta2, float theta3);
MatrixXf computePseudoInverse_vector(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d);
MatrixXf computePseudoInverse_full(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d);


/**
* @breif Calculate the Forward_Kinematics of a certain configuration
* @param[theta_i] joints' angles
* @return X,Y,Z of the end effector type MatrixXf
*/
MatrixXf FWD_Kinematics(float theta1, float theta2, float theta3);
MatrixXf FWD_Kinematics_vector(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d);


/**
* @breif Calculate the Inverse_Kinematics of X,Y,Z of the end effector
* @param[desired config] joints' angles
* @returnjoints' angles Matrix of the robot type MatrixXf
*/
MatrixXf Newton_Raphson_IK();
MatrixXf Newton_Raphson_IK_vector(MatrixXf X_d, std::vector<double> alpha, std::vector<double> r, std::vector<double> d);

/**
* @breif Calculate the rotation Matrix on the Y axis
* @param[theta] rotation angle over Y
* @return Rotation matrix of Pitch
*/
MatrixXf Compute_orientation(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d);