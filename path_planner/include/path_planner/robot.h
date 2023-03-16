#pragma once
#include "IK_mod.h"
#include "trajectory_generation.h"
#include "Quaternion.h"
#include <string>
#include <assert.h>
#include "trajectory_msgs/JointTrajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#define Joint_Prismatique	"prismatic"
#define Joint_Revolute		"revolute"
#define Joint_Axis_X		"x"
#define Joint_Axis_Y		"y"
#define Joint_Axis_Z		"z"

class robot
{
	public:
		robot(std::vector<double> alfa, std::vector<double> r, std::vector<double> d);
		robot();
		~robot();
		void add_link(std::string Name, double length, std::string Type, std::string Axis,bool base_link);
		void add_DH_line(double alfa, double r, double d);
		MatrixXf FWD_kinematics(MatrixXf Q);
		MatrixXf FWD_orientation(MatrixXf Q);
		MatrixXf Inverse_kinematics(MatrixXf Xd, MatrixXf init);
		MatrixXf Inverse_kinematics_Q(MatrixXf Xd, MatrixXf Od, MatrixXf Init, double Gamma);
		MatrixXf Jacobian_xyz(MatrixXf theta);
		MatrixXf Full_jacobian(MatrixXf theta);
		std::vector<std::tuple<std::vector<double>, std::vector<double>, std::vector<double>>> Trajectory_generation_Qubic(float t0, float tf, std::vector<std::vector<float>> vec_q0, std::vector<std::vector<float>> vec_qf, int n);
		std::vector<std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>>>Trajectory_generation_Quintic(float t0, float tf, std::vector<std::vector<float>> vec_q0, std::vector<std::vector<float>> vec_qf, int n);
		std::vector < std::tuple<std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>, std::vector<float>>>Trajectory_generation_via_points(float t0, float t1, float t2, float t3, std::vector<std::vector<float>> vec_q0x, std::vector<std::vector<float>> vec_qwx, std::vector<std::vector<float>> vec_q3x, int n);
		void Robot_resume();
	private:
		std::vector<double> _alfa, _r, _d , lengths;
		std::vector<std::string> link_names;
		double l0; std::string axis0; bool second = false;
		MatrixXf Jacobian;
		MatrixXf Jacob_Inv;
};

