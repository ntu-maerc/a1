#ifndef INCLUDE_INVERSE_KINEMATICS_H_
#define INCLUDE_INVERSE_KINEMATICS_H_

#include <vector>
#include <array>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

namespace robotic_arm_kinematics
{

/**
 * An analytical IK solver for a four-dof arm
 */
class InverseKinematics
{
public:

     InverseKinematics();

     virtual ~InverseKinematics();

     int CartToJnt(const geometry_msgs::Pose &cart_ip,
        std::vector<std::array<double, 4>> &angle_op);

private:

     //bool isSolutionValid(const KDL::JntArray &solution) const;

     /*
      * Using the analytical inverse Kinematic solution as give in paper
        https://ocw.mit.edu/courses/mechanical-engineering/2-12-introduction-to-robotics-fall-2005/lecture-notes/chapter4.pdf 

    */
     std::vector<std::array<double, 4>> ik_solver(const geometry_msgs::Pose &cart_ip);

private:
     std::vector<double> min_angles_;
     std::vector<double> max_angles_;

}; // class end

} // namespace end

#endif /* INCLUDE_INVERSE_KINEMATICS_H_ */