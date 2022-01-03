#include <robotic_arm_kinematics/inverse_kinematics.h>

#include <iostream>
#include <cmath>
#include <cstdlib>

using namespace robotic_arm_kinematics;

#define DEG_TO_RAD(x) ((x) * M_PI / 180.0)
#define RAD_TO_DEG(x) ((x) * 180.0 / M_PI)

InverseKinematics::InverseKinematics()
{
}

InverseKinematics::~InverseKinematics()
{
}

int InverseKinematics::CartToJnt(const geometry_msgs::Pose &cart_ip,
        std::vector<std::array<double, 4>> &angle_op)
{
    std::vector<double> solution_vec;

    // clear the vector
    angle_op.clear();

    // get the two set of solutions
    angle_op = ik_solver(cart_ip);

    if (angle_op.size() > 0) {
        std::cout<<"Inverse kinematics found a solution!"<<'\n';

        return 1;
    } else {
        std::cout<<"Inverse kinematics found no solution!"<<'\n';

        return -1;
    }
}

std::vector<std::array<double, 4>> InverseKinematics::ik_solver(const geometry_msgs::Pose &cart_ip)
{
  //vector containing the two sets of solutions
  std::vector<std::array<double, 4>>solution_vec;

  //Initialise joint values to 0
	double joint_1 = 0.0;
	double joint_2 = 0.0;
  double joint_2_2 = 0.0;
	double joint_3 = 0.0;
  double joint_3_2 = 0.0;
	double joint_4 = 0.0;
  double joint_4_2 = 0.0;

  //Set the link values for the arm (values are in meters)
  double l1 = 0.065;
	double l2 = 0.06172 ;
	double l3 = 0.01717 ;  

  tf::Quaternion quat;
  tf::quaternionMsgToTF(cart_ip.orientation, quat);
  double R, P, Y;
  tf::Matrix3x3(quat).getRPY(R, P, Y);

  std::cout<<"Printing the R,P,Y values"<<'\n';
  std::cout<<"["<<R<<"]--["<<P<<"]--["<<Y<<"]"<<'\n';

	double p_x = cart_ip.position.x ;
	double p_y = cart_ip.position.y ;
	double p_z = cart_ip.position.z ;

  /* Yaw in the joint 1 position calculated */
  //Need to add or subtract a constant amount as the gripper is shifted a bit due to servo positions
  //Constant value to be added when calibrating DEG_TO_RAD(8.6767936) or atan2(0.015925, 0.104353);
  joint_1=-(atan2(p_y, p_x))-atan2(0.015925, 0.104353);

  /* Using the analytical inverse Kinematic solution as give in paper
    https://ocw.mit.edu/courses/mechanical-engineering/2-12-introduction-to-robotics-fall-2005/lecture-notes/chapter4.pdf */

  //Convert the 3D problem into a planar one
  //Get gripper coordinates in planar world, and remove offset in the Z-axis of 0.06485
  double xg=std::abs(p_x);  //X value taken absolute
  double yg=p_z-0.06485;  //Taking link 2 as the origin

  //Angle of gripper with respect to X-axis (Need to add a constant of -1.57 as the gripper is in that orientation in rest pose)
  double phie=P;

  //Coordinates of Joint 4 which is the end effector is found as follows
  double xe=xg-(l3*cos(phie));
  double ye=yg-(l3*sin(phie));

  double alpha=atan2(ye, xe);

  /*Finding joint 3 angle*/
  double val_to_acos_1=(pow(l1, 2)+pow(l2, 2)-pow(xe, 2)-pow(ye, 2))/(2*l1*l2);
  double theta3=M_PI-acos(val_to_acos_1);
  //Since the rest pose has a set angle of 57.06deg between link 2 and link 3, we need to subtract it 
  joint_3=DEG_TO_RAD(57.06)+theta3;   


  /*Finding joint 2 angle*/
  double val_to_acos_2=(pow(l1, 2)-pow(l2, 2)+pow(xe, 2)+pow(ye, 2))/(2*l1*sqrt(pow(xe, 2)+pow(ye, 2)));
  double gamma=acos(val_to_acos_2);
  double theta2=alpha-gamma;
  //Since the rest pose has a set angle of 38.15deg between link 1 and link 2, we need to subtract it 
  joint_2=theta2-DEG_TO_RAD(38.15);

  /*Finding joint 4 angle*/
  double theta4=phie-theta2-theta3;
  //Since the rest pose has a set angle of 71.09deg between link 3 and link 4, we need to subtract it 
  joint_4=DEG_TO_RAD(71.09)+theta4;    

  /*Finding the other set of possible solution*/
  double theta2_2=theta2+(2*gamma);
  joint_2_2=theta2_2-DEG_TO_RAD(38.15);    //Note only works for second set of solutions for now

  double theta3_2=-theta3;
  joint_3_2=DEG_TO_RAD(57.06)+theta3_2;    //Note only works for second set of solutions for now

  double theta4_2=phie-theta2_2-theta3_2;
  joint_4_2=DEG_TO_RAD(71.09)+theta4_2;    //Note only works for second set of solutions for now


	std::array<double, 4> solution_1;
  std::array<double, 4> solution_2;

  std::cout<<"Printing the first set of theta values"<<'\n';
  std::cout<<"["<<RAD_TO_DEG(joint_1)<<"]--["<<RAD_TO_DEG(theta2)<<"]--["<<RAD_TO_DEG(theta3)<<"]--["<<RAD_TO_DEG(theta4)<<"]"<<'\n';

  std::cout<<"Printing the second set of theta values"<<'\n';  
  std::cout<<"["<<RAD_TO_DEG(joint_1)<<"]--["<<RAD_TO_DEG(theta2_2)<<"]--["<<RAD_TO_DEG(theta3_2)<<"]--["<<RAD_TO_DEG(theta4_2)<<"]"<<'\n';

	solution_1[0] = joint_1;
	solution_1[1] = joint_2;
	solution_1[2] = joint_3;
	solution_1[3] = joint_4;

  solution_vec.push_back(solution_1);

	solution_2[0] = joint_1;
	solution_2[1] = joint_2_2;
	solution_2[2] = joint_3_2;
	solution_2[3] = joint_4_2;

  solution_vec.push_back(solution_2);

	return solution_vec ;
}


