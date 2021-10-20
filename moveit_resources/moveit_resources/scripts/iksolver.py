#!/usr/bin/env python3

import rospy
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse
from geometry_msgs.msg import PoseStamped

"""
Class to make IK calls using the /compute_ik service.


Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class GetIK(object):
    def __init__(self, group, ik_timeout=5.0, ik_attempts=5,
                 avoid_collisions=False):
        """
        A class to do IK calls thru the MoveIt!'s /compute_ik service.

        :param str group: MoveIt! group name
        :param float ik_timeout: default timeout for IK
        :param int ik_attempts: default number of attempts
        :param bool avoid_collisions: if to ask for IKs that take
        into account collisions
        """
        rospy.logerr("Initalizing GetIK...")
        self.group_name = group
        self.ik_timeout = ik_timeout
        self.ik_attempts = ik_attempts
        self.avoid_collisions = avoid_collisions
        rospy.logerr("Computing IKs for group: " + self.group_name)
        rospy.logerr("With IK timeout: " + str(self.ik_timeout))
        rospy.logerr("And IK attempts: " + str(self.ik_attempts))
        rospy.logerr("Setting avoid collisions to: " +
                      str(self.avoid_collisions))
        self.ik_srv = rospy.ServiceProxy('/arm/compute_ik',
                                         GetPositionIK)
        rospy.logerr("Waiting for /compute_ik service...")
        self.ik_srv.wait_for_service()
        rospy.logerr("Connected!")

    def get_ik(self, pose_stamped,
               group=None,
               ik_timeout=None,
               ik_attempts=None,
               avoid_collisions=None):
        """
        Do an IK call to pose_stamped pose.

        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        :param str group: The MoveIt! group.
        :param float ik_timeout: The timeout for the IK call.
        :param int ik_attemps: The maximum # of attemps for the IK.
        :param bool avoid_collisions: If to compute collision aware IK.
        """
        if group is None:
            group = self.group_name
        if ik_timeout is None:
            ik_timeout = self.ik_timeout
        if ik_attempts is None:
            ik_attempts = self.ik_attempts
        if avoid_collisions is None:
            avoid_collisions = self.avoid_collisions
        req = GetPositionIKRequest()
        req.ik_request.group_name = group
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rospy.Duration(ik_timeout)
        req.ik_request.robot_state.is_diff=True
#        req.ik_request.attempts = ik_attempts
        req.ik_request.avoid_collisions = avoid_collisions

        try:
            resp = self.ik_srv.call(req)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp
if __name__ == '__main__':
    rospy.init_node('test_ik')
    rospy.logerr("Querying for IK")
    ps = PoseStamped()
    ps.header.frame_id = 'end_effector'
    ps.pose.position.x = 0.09990060417228698
    ps.pose.position.y = -6.126239024427333e-07
    ps.pose.position.z = 0.07499724338840368
    ps.pose.orientation.w = 0.99
    gik=GetIK("robotic_arm")
    resp = gik.get_ik(ps)
    rospy.logerr(resp)


##! /usr/bin/env python3

#from geometry_msgs.msg import PoseStamped
#import rospy
#from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
#from moveit_msgs.msg import MoveItErrorCodes

## In __init__ method
#compute_ik = rospy.ServiceProxy('/arm/compute_ik', GetPositionIK)
#x,y,z=0.05,0,0.1
#timeout=rospy.Duration(20)
#ps = PoseStamped()
#ps.header.frame_id = 'end_effector'
#ps.pose.position.x = 0.09990060417228698
#ps.pose.position.y = -6.126239024427333e-07
#ps.pose.position.z = 0.07499724338840368
#ps.pose.orientation.w = 0.99
##rospy.logerr(1)
#request = GetPositionIKRequest()
#request.ik_request.pose_stamped = ps
#request.ik_request.group_name = 'robotic_arm'
#request.ik_request.timeout = timeout
#request.ik_request.robot_state.is_diff=True
#response = compute_ik(request)
#error_str = response.error_code
#rospy.logerr(error_str)
#joint_state = response.solution.joint_state
#for name, position in zip(joint_state.name, joint_state.position):
#         rospy.logerr('{}: {}'.format(name, position))

