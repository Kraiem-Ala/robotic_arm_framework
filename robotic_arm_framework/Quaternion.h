#pragma once
#include<Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Vector3f;
using Eigen::VectorXf;
class Quaternion{
public:
	double r; // real part of a unit quaternion
	MatrixXf i; // imaginary part of a unit quaternion
	Quaternion();
	Quaternion(MatrixXf euleur_angles);
	~Quaternion();
	double Magnitude();
	Quaternion Conjugate();
	Quaternion Inverse();
	Quaternion operator*(Quaternion Q2);
	Quaternion operator+(Quaternion Q2);
	MatrixXf operator-(Quaternion Q2);
	Quaternion operator*(double d);
	Quaternion operator/(double d);


	MatrixXf Quaternion2euleur();
};

