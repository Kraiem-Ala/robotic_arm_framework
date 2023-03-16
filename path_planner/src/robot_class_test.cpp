#include "robot.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include <random>
#define initial 0.8
double constrainAngle2(double x){
    double y = atan2(sin(x),cos(x));
	return y ;
}
double constrainAngle(double x){
    double y = atan(sin(x)/cos(x));
	return y ;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "joint_calculator");
	robot robot1;
	
	/*robot1.add_link("Base_link", 0.25, Joint_Revolute, Joint_Axis_Y, true);
	robot1.add_link("link1", 0.175, Joint_Revolute, Joint_Axis_Y, false);
	robot1.add_link("link2", 0.75, Joint_Revolute, Joint_Axis_Z, false);
	robot1.add_link("link3", 0.75, Joint_Revolute, Joint_Axis_Z, false);
	robot1.add_link("link4", 0.55, Joint_Revolute, Joint_Axis_Z, false);
	robot1.add_link("gripper", 0.1, Joint_Revolute, Joint_Axis_Y, false);
	robot1.Robot_resume();
	std::uniform_real_distribution<double> unif(-M_PI,M_PI);
	std::default_random_engine re;
	MatrixXf theta(5,1), Xd(3, 1),Od(3, 1), fwd(5,1), Xf(3, 1),Of(3, 1),Xw(3, 1);
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64MultiArray>("/robot_teta_controllers/command/", 10);
	ros::Publisher trajectory_pub = n.advertise<trajectory_msgs::JointTrajectory>("/robot_arm_controller/command/", 10);
	ros::Publisher gripper = n.advertise<std_msgs::Float64MultiArray>("/gripper_position_controller/command/", 10);
	theta(0, 0) = rand() / (double(RAND_MAX) + 1.0);
	theta(1, 0) = rand() / (double(RAND_MAX) + 1.0);
	theta(2, 0) = rand() / (double(RAND_MAX) + 1.0);
	theta(3, 0) = rand() / (double(RAND_MAX) + 1.0);
	theta(4, 0) = rand() / (double(RAND_MAX) + 1.0);
	
	std::cout<<"Random Init = \n"<<theta<<"\n\n";
	Xd(0, 0) = -0.75;
	Xd(1, 0) = 0.0;
	Xd(2, 0) = 1.0;

	Od<<0.5,0.3,0.0;
	
	Xw(0, 0) = 0.0;
	Xw(1, 0) = 0.75;
	Xw(2, 0) = 1.75;
	
	Of<<0.0,M_PI_2,0.0;
	Xf(0, 0) = -1.0;
	Xf(1, 0) = -1.0;
	Xf(2, 0) = 1.25;

	MatrixXf INV = robot1.Inverse_kinematics_Q(Xd,Od,theta,1.0);
	std::cout <<"\n FWD of INV \n"<< robot1.FWD_kinematics(INV) <<"\n" << robot1.FWD_orientation(INV)<<"\n INV = \n";
	INV(0, 0) = constrainAngle2(INV(0, 0));
	INV(1, 0) = constrainAngle(INV(1, 0));
	INV(2, 0) = constrainAngle(INV(2, 0));
	INV(3, 0) = constrainAngle(INV(3, 0));
	INV(4, 0) = constrainAngle2(INV(4, 0));
	
	std::cout << INV <<"\n \n";

	INV = robot1.Inverse_kinematics(Xd,theta);

	//std_msgs::Float64MultiArray cmd;
	//trajectory_msgs::JointTrajectory trajectory;
	//cmd.data.push_back(-1.0);
	//cmd.data.push_back(-1.0);
	//cmd.data.push_back(-1.0);
	//float t0c = 0.0, tfc = 5.0;
	///*MatrixXf INV = robot1.Inverse_kinematics(Xw,theta);
	//INV(0, 0) = constrainAngle2(INV(0, 0));
	//INV(1, 0) = constrainAngle(INV(1, 0));
	//INV(2, 0) = constrainAngle(INV(2, 0));
	//INV(3, 0) = constrainAngle(INV(3, 0));
	////INV(4, 0) = constrainAngle(INV(4, 0));
	//std::cout<<INV << "\n FWd\n" << robot1.FWD_kinematics(INV)<< "\n Goal \n"<<Xd << "\n";*/
//
	//MatrixXf INV2 = robot1.Inverse_kinematics(Xf,INV);
	//
	//INV2(0, 0) = 0.0; //constrainAngle(INV2(0, 0));
	//INV2(1, 0) = 0.0; //constrainAngle(INV2(1, 0));
	//INV2(2, 0) = 0.0; //constrainAngle(INV2(2, 0));
	//INV2(3, 0) = 0.0; //constrainAngle(INV2(3, 0));
	//INV2(4, 0) = 0.0;  //constrainAngle(INV2(4, 0));
	//std::cout<<INV2 << "\n FWd2\n" << robot1.FWD_kinematics(INV2)<< "\n Goal \n"<<Xf << "\n";
	//
	//trajectory.joint_names.push_back("theta1");
	//trajectory.joint_names.push_back("theta2");
	//trajectory.joint_names.push_back("theta3");
	//trajectory.joint_names.push_back("theta4");
	//trajectory.joint_names.push_back("yaw");
	//
	//MatrixXf q0d(5,1) , qfd(5,1) ;
	//q0d << 0.0, 0.0, 0.0, 0.0,0.0;
	//qfd << 0.0, 0.0, 0.0, 0.0,0.0;
	//std::tuple<std::vector<float>, std::vector<MatrixXf>, std::vector<MatrixXf>> tc = computeCubicTraj_vect(INV2, INV, q0d, qfd, t0c, tfc, 50);
	//
	//for (size_t i = 0; i < std::get<1>(tc).size(); i++)
	//{
	//	trajectory_msgs::JointTrajectoryPoint point;
	//	MatrixXf q = std::get<1>(tc)[i] ;
	//	MatrixXf q_d = std::get<2>(tc)[i] ;
	//	//std::cout<<q <<"\n";
	//	for (size_t j = 0; j < q.rows(); j++)
	//	{
	//		point.positions.push_back(q(j,0));
	//		point.velocities.push_back(q_d(j,0));
	//	}
	//	
	//	
	//	point.time_from_start = ros::Duration(std::get<0>(tc)[i]);
	//	trajectory.points.push_back(point);
	//}
	//gripper.publish(cmd);
	//sleep(1);
	//for (size_t i = 0; i < 2; i++)
	//{
	//	gripper.publish(cmd);
	//	trajectory_pub.publish(trajectory);
	//	sleep(1);
	//	ros::spinOnce();
	//}
	//sleep(6);
	////cmd.data.clear();
	//cmd.data.push_back(1.0);
	//cmd.data.push_back(1.0);
	//cmd.data.push_back(1.0);
	//gripper.publish(cmd);
	//sleep(1);
	///*dfff*/
	//trajectory.points.clear();
	//MatrixXf INV3 = robot1.Inverse_kinematics(Xd,theta);
	//INV3(0, 0) = constrainAngle2(INV3(0, 0));
	//INV3(1, 0) = constrainAngle(INV3(1, 0));
	//INV3(2, 0) = constrainAngle(INV3(2, 0));
	//INV3(3, 0) = constrainAngle(INV3(3, 0));
	//tc = computeCubicTraj_vect(INV, INV3, q0d, qfd, t0c, tfc, 50);
	//
	//for (size_t i = 0; i < std::get<1>(tc).size(); i++)
	//{
	//	trajectory_msgs::JointTrajectoryPoint point;
	//	MatrixXf q = std::get<1>(tc)[i] ;
	//	MatrixXf q_d = std::get<2>(tc)[i] ;
	//	//std::cout<<q <<"\n";
	//	for (size_t j = 0; j < q.rows(); j++)
	//	{
	//		point.positions.push_back(q(j,0));
	//		point.velocities.push_back(q_d(j,0));
	//	}
	//	
	//	
	//	point.time_from_start = ros::Duration(std::get<0>(tc)[i]);
	//	trajectory.points.push_back(point);
	//}
	//gripper.publish(cmd);
	//sleep(1);
	//for (size_t i = 0; i < 2; i++)
	//{
	//	gripper.publish(cmd);
	//	trajectory_pub.publish(trajectory);
	//	sleep(1);
	//	ros::spinOnce();
	//}
	//sleep(6);
	///*end dfff*/
	//cmd.data.clear();
	//cmd.data.push_back(-1.0);
	//cmd.data.push_back(-1.0);
	//cmd.data.push_back(-1.0);
	//gripper.publish(cmd);
	//sleep(1);
	////sleep(15);
	//
	///*for (size_t i = 0; i < trajectory.points.size(); i++)
	//{
	//	
	//	for (size_t j = 0; j < trajectory.points[i].positions.size(); j++)
	//	{
	//		std::cout << trajectory.points[i].positions[j]<<"\n" ;
	//	}
//
	//	
	//}*/
//
return 0;
}//