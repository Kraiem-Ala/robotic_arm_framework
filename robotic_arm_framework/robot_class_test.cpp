#include "robot.h"

void main()
{
	robot robot1;
	robot1.add_link("Base_link", 0.0, Joint_Revolute, Joint_Axis_Y, true);
	robot1.add_link("link1", 1.0, Joint_Revolute, Joint_Axis_Y, false);
	robot1.add_link("link2", 1.0, Joint_Revolute, Joint_Axis_Z, false);
	robot1.add_link("link3", 1.0, Joint_Revolute, Joint_Axis_Z, false);
	//robot1.add_link("link4", 1.0, Joint_Revolute, Joint_Axis_Z, false);
	MatrixXf theta(3,1), Xd(3, 1), fwd(3,1), Xf(3, 1);
	theta(0, 0) = M_PI / 10;
	theta(1, 0) = M_PI / 10;
	theta(2, 0) = M_PI / 10;
	
	
	Xd(0, 0) = 0.544895;
	Xd(1, 0) = 0.943786;
	Xd(2, 0) = 2.63099;

	Xf(0, 0) = 0.0;
	Xf(1, 0) = 0.0;
	Xf(2, 0) = 3.0;

	std::vector<std::vector<float>> vec_q0;
	std::vector<std::vector<float>> vec_qf;

	float t0c = 0.0, tfc = 2.0;
	MatrixXf INV = robot1.Inverse_kinematics(Xd);
	float q00 = INV(0, 0);
	float q01 = INV(1, 0);
	float q02 = INV(2, 0);

	float d_q00 = 15;
	float d_q01 = 15;
	float d_q02 = 15;

	std::vector<float> vec_q0c{ q00, d_q00 };
	vec_q0.push_back(vec_q0c);
	std::vector<float> vec_q01c{ q01, d_q01 };
	vec_q0.push_back(vec_q01c);
	std::vector<float> vec_q02c{ q02, d_q02 };
	vec_q0.push_back(vec_q02c);

	std::cout << INV<<"\n simple FWD 1\n";/*
	std::cout << robot1.FWD_kinematics(INV) << "\n \n";*/
	INV = robot1.Inverse_kinematics(Xf);
	float q0f = INV(0, 0);
	float q1f = INV(1, 0);
	float q2f = INV(2, 0);

	float d_q0f = 0;
	std::vector<float> vec_qfc{ q0f, d_q0f };
	vec_qf.push_back(vec_qfc);
	float d_q1f = 0;
	std::vector<float> vec_qf1c{ q1f, d_q1f };
	vec_qf.push_back(vec_qf1c);
	float d_q2f = 0;
	std::vector<float> vec_qf2c{ q2f, d_q2f };
	vec_qf.push_back(vec_qf2c);
	std::cout << INV << "\n simple FWD 2\n";

	std::vector<std::tuple<std::vector<float>, std::vector<float>, std::vector<float>>> qdtc = robot1.Trajectory_generation_Qubic(t0c, tfc, vec_q0, vec_qf, 20);
	for (size_t i = 0; i < qdtc.size(); i++)
	{
		std::cout << "joint "<< i <<"\n";
		
		for (size_t k = 0; k < std::get<0>(qdtc[i]).size(); k++)
		{
			std::cout << std::get<1>(qdtc[i])[k] <<"\n";
		}
		std::cout << std::endl;

		std::cout << "\n";

	}

}