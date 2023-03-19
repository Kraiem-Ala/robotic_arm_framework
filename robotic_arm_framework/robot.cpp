#include "robot.h"
robot::robot(std::vector<double> alfa, std::vector<double> r, std::vector<double> d) {
	this->_alfa = alfa;
	this->_r = r;
	this->_d = d;
}
robot::robot(){}
robot::~robot(){}
/**
* @breif adds a link to the robot's tree
* @param[name] link's name
* @param[length] the length of the regid body
* @param[Type] type of the joint "revolute" or "prismatic"
* @param[Axis] the name of the Axis of the Joint
* @param[base_link] used to calculate the DH parameters
*/
void robot::add_link(std::string Name, double length,std::string Type, std::string Axis, bool base_link = false)
{
	this->lengths.push_back(length);
	this->link_names.push_back(Name);
	if (base_link == false)
	{
		if (Axis == Joint_Axis_Y) {
			if (this->second == true)
			{
				this->_d.push_back(length + this->l0);
				this->second = false;
			}
			else
			{
				this->_d.push_back(length);
			}
			this->_alfa.push_back(M_PI / 2);
			this->_r.push_back(0.0);
		}
		else if (Axis == Joint_Axis_Z) {
			if (this->second == true)
			{
				this->_r.push_back(length + this->l0);
				this->second = false;
			}
			else
			{
				this->_r.push_back(length);
			}
			this->_alfa.push_back(0.0);
			this->_d.push_back(0.0);
		}
	}
	else if (base_link == true)
	{
		this->axis0 = Axis;
		this->l0 = length;
		this->second = true;
	}
}
/**
* @breif adds DH_line to the robot's DH table
* @param[alpha] alpha angle of the link i-1 to link i
* @param[r] distance between the link i-1 to link i on the X axis
* @param[d] distance between the link i-1 to link i on the Z axis"
**/
void robot::add_DH_line(double alfa, double r, double d)
{
	this->_alfa.push_back(alfa);
	this->_d.push_back(d);
	this->_r.push_back(r);
}
/**
* @breif alculates the X,Y,Z coordinates of the end effector 
* @param[Q] joints angles of the robot
**/
MatrixXf robot::FWD_kinematics(MatrixXf Q)
{
	//Q(1, 0) = Q(1,0)+ M_PI/2;
	assert(Q.rows() == _alfa.size() && "Number of joint values in Q don't match number of robot's joints");
	assert((link_names.size() > 1 || _alfa.size()>0)  && "Please Insert at least 2 links to start");
	return(FWD_Kinematics_vector(Q, this->_alfa, this->_r, this->_d));
}
MatrixXf robot::FWD_orientation(MatrixXf Q)
{
	//Q(1, 0) = Q(1, 0) + M_PI / 2;
	return(Compute_orientation(Q, this->_alfa, this->_r, this->_d));
}

/**
* @breif alculates joints angles of the robot
* @param[Xd] the X,Y,Z coordinates of the end effector
**/
MatrixXf robot::Inverse_kinematics(MatrixXf Xd)
{	
	assert(Xd.rows() == 3 && "Please Enter 3 values :  X,Y,Z of the end effector");
	assert((link_names.size() > 1 || _alfa.size() > 0) && "Please Insert at least 2 links to start");
	return(Newton_Raphson_IK_vector(Xd, this->_alfa, this->_r, this->_d));
}
std::vector<std::tuple<std::vector<float>, std::vector<float>, std::vector<float>>> robot::Trajectory_generation_Qubic(float t0, float tf, std::vector<std::vector<float>> vec_q0, std::vector<std::vector<float>> vec_qf, int n)
{
	assert(vec_q0[0].size()>=2 && vec_qf[0].size() >= 2 && "Please provide at least 2 mesures to vec_q0 and vec_qf");
	std::vector<std::tuple<std::vector<float>, std::vector<float>, std::vector<float>>> trajectory;
	MatrixXf A(4, 1);
	for (size_t i = 0; i < vec_q0.size(); i++)
	{
		
		A = computeCubicCoeff(t0, tf, vec_q0[i], vec_qf[i]);
		trajectory.push_back(computeCubicTraj(A,t0,tf,n));
		
	}
	return trajectory;
}
std::vector < std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>>> robot::Trajectory_generation_via_points(float t0, float t1, float t2, float t3, std::vector<std::vector<float>> vec_q0x, std::vector<std::vector<float>> vec_qwx, std::vector<std::vector<float>> vec_q3x, int n)
{
	assert(vec_q0x[0].size() >= 4 && vec_q3x[0].size() >= 4 && "Please provide at least 2 mesures to vec_q0 and vec_qf");
	std::vector<std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>>> trajectory;
	MatrixXf A(4, 1);
	for (size_t i = 0; i < vec_q0x.size(); i++)
	{
		A = compute7orderCoeff(t0, t1, t2, t3, vec_q0x[i], vec_qwx[i], vec_q3x[i]);
		trajectory.push_back(computeWaypointTraj(A,t0,t3,n));
	}
	return trajectory;
}
std::vector<std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>>> robot::Trajectory_generation_Quintic(float t0, float tf, std::vector<std::vector<float>> vec_q0, std::vector<std::vector<float>> vec_qf, int n)
{
	assert(vec_q0[0].size() >= 3 && vec_qf[0].size() >= 3 && "Please provide at least 2 mesures to vec_q0 and vec_qf");
	std::vector<std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>>> trajectory;
	MatrixXf A(4, 1);
	for (size_t i = 0; i < vec_q0.size(); i++)
	{
		A = computeQuinticCoeff(t0, tf, vec_q0[i], vec_qf[i]);
		trajectory.push_back(computeQuinticTraj(A, t0, tf, n));
	}
	return trajectory;

}
/**
* @breif Prints the important informations of the robot
**/
void robot::Robot_resume()
{
	for (size_t i = 0; i < _alfa.size(); i++)
	{
		std::cout << _d[i] << "|" << _r[i] << "|" << _alfa[i] << "|" << "\n";
	}
}
MatrixXf robot::Inverse_kinematics_d(MatrixXf Xd, double gamma)
{
	int iteration = 0, size = this->_alfa.size();
	MatrixXf i_1_theta(size, 1);
	MatrixXf i_theta(size, 1);
	MatrixXf e(size, 1);
	for (size_t i = 0; i < size; i++)
	{
		i_theta(i, 0) = M_PI / 7;
	}
	MatrixXf FWD = FWD_Kinematics_vector(i_theta, _alfa, _r, _d);
	e = Xd - FWD;

	while (((std::abs(e(0, 0)) > 0.00001) || (std::abs(e(1, 0)) > 0.00001) || (std::abs(e(2, 0)) > 0.00001)) && (iteration < 2500))
	{

		iteration++;
		MatrixXf invJ = computePseudoInverse_vector(i_theta, _alfa, _r, _d);

		i_1_theta = i_theta + gamma*invJ * e;

		FWD = FWD_Kinematics_vector(i_1_theta, _alfa, _r, _d);

		e = Xd - FWD;
		i_theta = i_1_theta;
		//std::cout << iteration <<"\n";
		// std::cout << i_theta * (180 / M_PI) << std::endl;
	}
	std::cout << "Done in " << iteration << " iterations" << std::endl;
	return i_theta;
}
MatrixXf robot::Inverse_kinematics_Q(MatrixXf Xd, MatrixXf Od, MatrixXf Init, double Gamma)
{
	int iteration = 0, size = this->_alfa.size();
	MatrixXf i_1_theta(size, 1);
	MatrixXf i_theta(size, 1);
	MatrixXf ep(3, 1);
	MatrixXf eo(3, 1);
	i_theta = Init;
	//MatrixXf FWD_p = FWD_Kinematics_vector(i_theta, _alfa, _r, _d);
	MatrixXf FWD_p = FWD_kinematics(i_theta);
	MatrixXf FWD_o = FWD_orientation(i_theta);
	Quaternion ori = Quaternion(Od);
	Quaternion orientation = Quaternion(FWD_o);
	ep = Xd - FWD_p;
	eo = ori - orientation;
	MatrixXf e(6, 1);
	e << ep, eo;
	while (((std::abs(ep(0, 0)) > 0.00001) || (std::abs(ep(1, 0)) > 0.00001) || (std::abs(ep(2, 0)) > 0.00001) || (std::abs(eo(0, 0)) > 0.00001) || (std::abs(eo(1, 0)) > 0.01) /*|| (std::abs(eo(2, 0)) > 0.01)*/) && (iteration < 5000))
	{

		iteration++;
		MatrixXf invJ = computePseudoInverse_full(i_theta, _alfa, _r, _d);

		i_1_theta = i_theta + Gamma * invJ * e;

		//FWD_p = FWD_Kinematics_vector(i_1_theta, _alfa, _r, _d);
		FWD_p = FWD_kinematics(i_1_theta);
		FWD_o = FWD_orientation(i_1_theta);
		orientation = Quaternion(FWD_o);
		ep = Xd - FWD_p;
		eo = ori - orientation;
		e << ep, eo;
		i_theta = i_1_theta;
		//std::cout << iteration <<"\n";
		// std::cout << i_theta * (180 / M_PI) << std::endl;
	}
	std::cout << "Done in " << iteration << " iterations" << std::endl;
	//std::cout << "Last error \n" << ep << "\n" << eo << "\n";
	return i_theta;
}

MatrixXf robot::Jacobian_xyz(MatrixXf theta)
{
	return( jacobian_vect(theta, this->_alfa, this->_r, this->_d));
}

MatrixXf robot::Full_jacobian(MatrixXf theta)
{
	return(full_jacobian(theta, this->_alfa, this->_r, this->_d));
}