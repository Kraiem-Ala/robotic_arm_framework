#include "IK_mod.h"
MatrixXf RotYaw(double theta)
{
	MatrixXf Yaw(3, 3);

	Yaw(0, 0) = std::cos(theta);
	Yaw(0, 1) = -std::sin(theta);
	Yaw(0, 2) = 0;

	Yaw(1, 0) = std::sin(theta);
	Yaw(1, 1) = std::cos(theta);
	Yaw(1, 2) = 0;

	Yaw(1, 2) = 0;
	Yaw(1, 2) = 0;
	Yaw(1, 2) = 1;

	return Yaw;
}

MatrixXf RotPitch(double theta)
{
	MatrixXf Pitch(3, 3);

	Pitch(0, 0) = std::cos(theta);
	Pitch(0, 1) = 0;
	Pitch(0, 2) = std::sin(theta);

	Pitch(1, 0) = 0;
	Pitch(1, 1) = 1;
	Pitch(1, 2) = 0;

	Pitch(1, 2) = -std::sin(theta);
	Pitch(1, 2) = 0;
	Pitch(1, 2) = std::cos(theta);

	return Pitch;
}

MatrixXf RotRoll(double theta)
{
	MatrixXf Roll(3, 3);

	Roll(0, 0) = 1;
	Roll(0, 1) = 0;
	Roll(0, 2) = 0;

	Roll(1, 0) = 0;
	Roll(1, 1) = std::cos(theta);
	Roll(1, 2) = -std::sin(theta);

	Roll(2, 0) = 0;
	Roll(2, 1) = std::sin(theta);
	Roll(2, 2) = std::cos(theta);

	return Roll;
}

MatrixXf Idendity()
{
	MatrixXf I(4, 4);
	for (size_t i = 0; i < 4; i++)
	{
		for (size_t j = 0; j < 4; j++)
		{
			if (i == j)
				I(i, j) = 1;
			else
				I(i, j) = 0;
		}

	}
	return I;
}

MatrixXf Transformation_mat(double alfa, double theta, double r, double d)
{
	MatrixXf dh(4, 4);

	dh(0, 0) = std::cos(theta);
	dh(0, 1) = -std::sin(theta) * std::cos(alfa);
	dh(0, 2) = std::sin(theta) * std::sin(alfa);
	dh(0, 3) = r * std::cos(theta);

	dh(1, 0) = std::sin(theta);
	dh(1, 1) = std::cos(theta) * std::cos(alfa);
	dh(1, 2) = -std::cos(theta) * std::sin(alfa);
	dh(1, 3) = r * std::sin(theta);

	dh(2, 0) = 0;
	dh(2, 1) = std::sin(alfa);
	dh(2, 2) = std::cos(alfa);
	dh(2, 3) = d;

	dh(3, 0) = 0;
	dh(3, 1) = 0;
	dh(3, 2) = 0;
	dh(3, 3) = 1;

	return dh;
}

MatrixXf projections(std::vector<float> v)
{
	/*
	[0,1,2
	 3,4,5
	 6,7,8] 
	*/
	MatrixXf proj(3, 3);
	proj(0, 0) = v[0];
	proj(0, 1) = v[1];
	proj(0, 2) = v[2];
	proj(1, 0) = v[3];
	proj(1, 1) = v[4];
	proj(1, 2) = v[5];
	proj(2, 0) = v[6];
	proj(2, 1) = v[7];
	proj(2, 2) = v[8];
	return proj;
}

MatrixXf jacobian(double theta1, double theta2, double theta3) // for dynamic : std::vector<double> theta , alfa , r , d or pass DH table of the robot
{
	double alfa1 = M_PI / 2;
	double alfa2 = 0.0;
	double alfa3 = 0.0;

	double r1 = 0.0;
	double r2 = 1.0;
	double r3 = 1.0;

	double d1 = 1.0;
	double d2 = 0.0;
	double d3 = 0.0;

	MatrixXf DH00(4, 4);
	MatrixXf DH01(4, 4);
	MatrixXf DH12(4, 4);
	MatrixXf DH23(4, 4);

	// DH00 = 0;
	DH01 = Transformation_mat(alfa1, theta1, r1, d1);
	DH12 = Transformation_mat(alfa2, theta2, r2, d2);
	DH23 = Transformation_mat(alfa3, theta3, r3, d3);

	MatrixXf D01 = DH01;
	MatrixXf D01R = D01.block<3, 3>(0, 0); // rotation matrix
	MatrixXf D01T = D01.block<3, 1>(0, 3); // position vector
	MatrixXf D02 = DH01 * DH12;
	MatrixXf D02R = D02.block<3, 3>(0, 0);
	MatrixXf D02T = D02.block<3, 1>(0, 3);
	MatrixXf D03 = DH01 * DH12 * DH23;
	MatrixXf D03R = D03.block<3, 3>(0, 0);
	MatrixXf D03T = D03.block<3, 1>(0, 3);
	Vector3f Ri(0, 0, 1);

	Vector3f vecD01R = D01R * Ri;
	Vector3f vecD02R = D02R * Ri;

	Vector3f vecD01T(Map<Vector3f>(D01T.data(), D01T.cols() * D01T.rows()));
	Vector3f vecD02T(Map<Vector3f>(D02T.data(), D01T.cols() * D02T.rows()));
	Vector3f vecD03T(Map<Vector3f>(D03T.data(), D03T.cols() * D03T.rows()));

	MatrixXf J1 = Ri.cross(vecD03T); //R00
	MatrixXf J2 = (vecD01R).cross(vecD03T - vecD01T);
	MatrixXf J3 = (vecD02R).cross(vecD03T - vecD02T);
	MatrixXf J4 = Ri;
	MatrixXf J5 = vecD01R;
	MatrixXf J6 = vecD02R;
	MatrixXf J(3, 3); // we consider only linear velocities

	J(0, 0) = J1(0, 0);
	J(1, 0) = J1(1, 0);
	J(2, 0) = J1(2, 0);

	J(0, 1) = J2(0, 0);
	J(1, 1) = J2(1, 0);
	J(2, 1) = J2(2, 0);

	J(0, 2) = J3(0, 0);
	J(1, 2) = J3(1, 0);
	J(2, 2) = J3(2, 0);

	return J;
}

MatrixXf computePseudoInverse(float theta1, float theta2, float theta3)
{
	MatrixXf J = jacobian(theta1, theta2, theta3);
	MatrixXf invJ = J.completeOrthogonalDecomposition().pseudoInverse();

	return invJ;
}

MatrixXf FWD_Kinematics(float theta1, float theta2, float theta3)
{
	double alfa1 = M_PI / 2;
	double alfa2 = 0.0;
	double alfa3 = 0.0;

	double r1 = 0.0;
	double r2 = 1.0;
	double r3 = 1.0;

	double d1 = 1.0;
	double d2 = 0.0;
	double d3 = 0.0;

	MatrixXf DH00(4, 4);
	MatrixXf DH01(4, 4);
	MatrixXf DH12(4, 4);
	MatrixXf DH23(4, 4);

	DH01 = Transformation_mat(alfa1, theta1, r1, d1);
	DH12 = Transformation_mat(alfa2, theta2, r2, d2);
	DH23 = Transformation_mat(alfa3, theta3, r3, d3);

	MatrixXf DH03 = DH01 * DH12 * DH23;

	MatrixXf FWD = DH03.block<3, 1>(0, 3);

	return FWD;
}

MatrixXf Newton_Raphson_IK()
{
	MatrixXf e(3, 1);
	MatrixXf Xd(3, 1);
	int iteration = 0;
	MatrixXf i_1_theta(3, 1);
	MatrixXf i_theta(3, 1);

	i_theta(0, 0) = M_PI / 6;
	i_theta(1, 0) = M_PI / 9;
	i_theta(2, 0) = M_PI / 10;

	MatrixXf FWD = FWD_Kinematics(i_theta(0, 0), i_theta(1, 0), i_theta(2, 0));


	Xd(0, 0) = 0.544895;
	Xd(1, 0) = 0.943786;
	Xd(2, 0) = 2.63099;

	e = Xd - FWD;

	while ((std::abs(e(0, 0)) > 0.00001) || (std::abs(e(1, 0)) > 0.00001) || (std::abs(e(2, 0)) > 0.00001))
	{

		iteration++;
		MatrixXf invJ = computePseudoInverse(i_theta(0, 0), i_theta(1, 0), i_theta(2, 0));

		i_1_theta = i_theta + invJ * e;

		FWD = FWD_Kinematics(i_1_theta(0, 0), i_1_theta(1, 0), i_1_theta(2, 0));

		e = Xd - FWD;
		i_theta = i_1_theta;

		// std::cout << i_theta * (180 / M_PI) << std::endl;
	}
	std::cout << "Done in " << iteration << " iterations"<<std::endl;
	return i_theta;
}

MatrixXf jacobian_vect(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d)
{
	int dimension = alpha.size();

	std::vector<MatrixXf> DH;

	MatrixXf DH1(4, 4);

	for (size_t i = 0; i < alpha.size(); i++)
	{
		DH1 = Transformation_mat(alpha[i], theta(i,0), r[i], d[i]);
		DH.push_back(DH1);
	}

	std::vector<MatrixXf> D0;
	std::vector<MatrixXf> D0R;
	std::vector<MatrixXf> D0T;
	MatrixXf D00;
	D00 = Idendity();
	D00 = D00 * DH[0];
	D0.push_back(D00);
	D0R.push_back(D00.block<3, 3>(0, 0));// rotation matrix
	D0T.push_back(D00.block<3, 1>(0, 3));// position vector
	for (size_t i = 1; i < DH.size(); i++)
	{
		D00 = D0.back() * DH[i];
		D0.push_back(D00);
		D0R.push_back(D00.block<3, 3>(0, 0));// rotation matrix
		D0T.push_back(D00.block<3, 1>(0, 3));// position vector
		
	}

	/*MatrixXf D01 = DH01;
	MatrixXf D01R = D01.block<3, 3>(0, 0); 
	MatrixXf D01T = D01.block<3, 1>(0, 3); 

	MatrixXf D02 = DH01 * DH12;
	MatrixXf D02R = D02.block<3, 3>(0, 0);
	MatrixXf D02T = D02.block<3, 1>(0, 3);

	MatrixXf D03 = DH01 * DH12 * DH23;
	MatrixXf D03R = D03.block<3, 3>(0, 0);
	MatrixXf D03T = D03.block<3, 1>(0, 3);*/

	Vector3f Ri(0, 0, 1);

	std::vector<Vector3f> vecD0R;
	for (size_t i = 0; i < D0R.size(); i++)
	{
		vecD0R.push_back(D0R[i] * Ri);
	}
	/*Vector3f vecD01R = D01R * Ri;
	Vector3f vecD02R = D02R * Ri;*/

	std::vector<Vector3f> vecD0T;
	for (size_t i = 0; i < vecD0R.size(); i++)
	{
		vecD0T.push_back(Map<Vector3f>(D0T[i].data(), D0T[i].cols() * D0T[i].rows()));
	}
	/*Vector3f vecD01T(Map<Vector3f>(D01T.data(), D01T.cols() * D01T.rows()));
	Vector3f vecD02T(Map<Vector3f>(D02T.data(), D01T.cols() * D02T.rows()));
	Vector3f vecD03T(Map<Vector3f>(D03T.data(), D03T.cols() * D03T.rows()));*/

	std::vector<MatrixXf> J0;
	J0.push_back(Ri.cross(vecD0T.back()));
	for (size_t i = 0; i < vecD0T.size(); i++)
	{
		J0.push_back(vecD0R[i].cross(vecD0T.back() - vecD0T[i]));
	}
	
	//
	//MatrixXf J1 = Ri.cross(vecD03T); //R00
	//MatrixXf J2 = (vecD01R).cross(vecD03T - vecD01T);
	//MatrixXf J3 = (vecD02R).cross(vecD03T - vecD02T);
	//MatrixXf J4 = Ri;
	//MatrixXf J5 = vecD01R;
	//MatrixXf J6 = vecD02R;


	MatrixXf J(3, dimension); // we consider only linear velocities

	for (size_t i = 0; i < dimension; i++)
	{
		J(0, i) = J0[i](0, 0);
		J(1, i) = J0[i](1, 0);
		J(2, i) = J0[i](2, 0);
	}

	return J;
}

MatrixXf full_jacobian(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d)
{
	int dimension = alpha.size();

	std::vector<MatrixXf> DH;

	MatrixXf DH1(4, 4);

	for (size_t i = 0; i < alpha.size(); i++)
	{
		DH1 = Transformation_mat(alpha[i], theta(i, 0), r[i], d[i]);
		DH.push_back(DH1);
	}

	std::vector<MatrixXf> D0;
	std::vector<MatrixXf> D0R;
	std::vector<MatrixXf> D0T;
	MatrixXf D00;
	D00 = Idendity();
	D00 = D00 * DH[0];
	D0.push_back(D00);
	D0R.push_back(D00.block<3, 3>(0, 0));// rotation matrix
	D0T.push_back(D00.block<3, 1>(0, 3));// position vector
	for (size_t i = 1; i < DH.size(); i++)
	{
		D00 = D0.back() * DH[i];
		D0.push_back(D00);
		D0R.push_back(D00.block<3, 3>(0, 0));// rotation matrix
		D0T.push_back(D00.block<3, 1>(0, 3));// position vector

	}
	Vector3f Ri(0, 0, 1);

	std::vector<Vector3f> vecD0R;
	for (size_t i = 0; i < D0R.size(); i++)
	{
		vecD0R.push_back(D0R[i] * Ri);
	}
	std::vector<Vector3f> vecD0T;
	for (size_t i = 0; i < vecD0R.size(); i++)
	{
		vecD0T.push_back(Map<Vector3f>(D0T[i].data(), D0T[i].cols() * D0T[i].rows()));
	}
	std::vector<MatrixXf> J0,J1;
	J0.push_back(Ri.cross(vecD0T.back()));
	for (size_t i = 0; i < vecD0T.size(); i++)
	{
		J0.push_back(vecD0R[i].cross(vecD0T.back() - vecD0T[i]));
		J1.push_back(vecD0R[i]);
	}

	//
	//MatrixXf J1 = Ri.cross(vecD03T); //R00
	//MatrixXf J2 = (vecD01R).cross(vecD03T - vecD01T);
	//MatrixXf J3 = (vecD02R).cross(vecD03T - vecD02T);
	//MatrixXf J4 = Ri;
	//MatrixXf J5 = vecD01R;
	//MatrixXf J6 = vecD02R;


	MatrixXf J(6, dimension); // we consider only linear velocities

	for (size_t i = 0; i < dimension; i++)
	{
		J(0, i) = J0[i](0, 0);
		J(1, i) = J0[i](1, 0);
		J(2, i) = J0[i](2, 0);
		J(3, i) = J1[i](0, 0);
		J(4, i) = J1[i](1, 0);
		J(5, i) = J1[i](2, 0);

	}

	return J;
}

MatrixXf computePseudoInverse_vector(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d)
{
	
	MatrixXf J = jacobian_vect(theta, alpha,r,d);
	MatrixXf invJ = J.completeOrthogonalDecomposition().pseudoInverse();

	return invJ;
}

MatrixXf computePseudoInverse_full(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d)
{
	MatrixXf J = full_jacobian(theta, alpha, r, d);
	MatrixXf invJ = J.completeOrthogonalDecomposition().pseudoInverse();

	return invJ;
}

MatrixXf FWD_Kinematics_vector(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d)
{
	

	std::vector<MatrixXf> DH;
	MatrixXf DH0 = Idendity();
	for (size_t i = 0; i < alpha.size(); i++)
	{
		DH.push_back(Transformation_mat(alpha[i], theta(i,0), r[i], d[i]));
		DH0 = DH0 * DH.back();
	}

	MatrixXf FWD = DH0.block<3, 1>(0, 3);

	return FWD;
}

MatrixXf Newton_Raphson_IK_vector(MatrixXf X_d, std::vector<double> alpha, std::vector<double> r, std::vector<double> d)
{
	
	//MatrixXf Xd(3, 1);
	int iteration = 0 , size = alpha.size();
	MatrixXf i_1_theta(size, 1);
	MatrixXf i_theta(size, 1);
	MatrixXf e(size, 1);
	for (size_t i = 0; i < size; i++)
	{
		i_theta(i, 0) = M_PI/7;
	}
	MatrixXf FWD = FWD_Kinematics_vector(i_theta,alpha,r,d);


	e = X_d - FWD;

	while (((std::abs(e(0, 0)) > 0.00001) || (std::abs(e(1, 0)) > 0.00001) || (std::abs(e(2, 0)) > 0.00001)) && (iteration < 2500))
	{

		iteration++;
		MatrixXf invJ = computePseudoInverse_vector(i_theta, alpha, r, d);

		i_1_theta = i_theta + invJ * e;

		FWD = FWD_Kinematics_vector(i_1_theta, alpha, r, d);

		e = X_d - FWD;
		i_theta = i_1_theta;
		//std::cout << iteration <<"\n";
		// std::cout << i_theta * (180 / M_PI) << std::endl;
	}
	std::cout << "Done in " << iteration << " iterations" << std::endl;
	return i_theta;
}

MatrixXf Compute_orientation(MatrixXf theta, std::vector<double> alpha, std::vector<double> r, std::vector<double> d)
{
	std::vector<MatrixXf> DH;
	MatrixXf DH0 = Idendity();
	MatrixXf rot(3, 1);
	for (size_t i = 0; i < alpha.size(); i++)
	{
		DH.push_back(Transformation_mat(alpha[i], theta(i, 0), r[i], d[i]));
		DH0 = DH0 * DH.back();
	}
	rot(2,0) = atan2(DH0(1,0), DH0(0,0));
	rot(1,0) = atan2(-1 * DH0(2, 0), sqrt(pow(DH0(2, 1), 2) + pow(DH0(2, 2), 2)));
	rot(0,0) = atan2(DH0(2, 1), DH0(2, 2));
	return rot;
}
