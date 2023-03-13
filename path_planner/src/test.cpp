#include"IK_mod.h"
int main_test()
{
	//MatrixXf thetaRobot = Newton_Raphson_IK();
	//MatrixXf FWD = FWD_Kinematics(M_PI / 3, M_PI / 4, M_PI / 8); //0.544895, 0.943786, 2.63099
	std::vector<double> alfa, d, r;
	MatrixXf Xd(3,1) , theta(4,1);
	alfa.push_back(M_PI / 2);
	alfa.push_back(0.0);
	alfa.push_back(0.0);
	alfa.push_back(0.0);

	r.push_back(0.0);
	r.push_back(1.0);
	r.push_back(1.0);
	r.push_back(1.0);

	d.push_back(1.0);
	d.push_back(0.0);
	d.push_back(0.0);
	d.push_back(0.0);
	
	Xd(0, 0) = 0.544895;
	Xd(1, 0) = 0.943786;
	Xd(2, 0) = 2.63099;
	
	theta(0, 0) = M_PI / 10;
	theta(1, 0) = M_PI / 10;
	theta(2, 0) = M_PI / 10;
	theta(3,0) = M_PI / 10;
	MatrixXf Jacobian , Jacobian2,fwd , fwd2;
	/*Jacobian = jacobian_vect(theta, alfa, r, d);
	Jacobian2 = jacobian(theta(0, 0), theta(1, 0), theta(2, 0));
	
	fwd = FWD_Kinematics_vector(theta, alfa, r, d);
	fwd2 = FWD_Kinematics(theta(0, 0), theta(1, 0), theta(2, 0));
	std::cout << fwd << "\n \n";
	std::cout << fwd2 << "\n";*/
	theta =  Newton_Raphson_IK_vector(Xd, alfa, r, d);
	std::cout << FWD_Kinematics_vector(theta,alfa,r,d) << "\n";
	return 0;
}