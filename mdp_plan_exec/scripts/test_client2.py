#! /usr/bin/env python

import sys
import rospy
import rospkg

import actionlib

from mdp_plan_exec.mdp_extended import TopMapMdp, ProductMdp
from mdp_plan_exec.prism_mdp_manager import PrismMdpManager

from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyGoal

    
class MdpPlannerClient(object):
    
    
    def main(self):
        target_waypoint = "WayPoint24"
        nav_action_client = actionlib.SimpleActionClient('mdp_plan_exec/execute_policy', ExecutePolicyAction)
        nav_action_client.wait_for_server()
        rospy.loginfo(" ...done")
        nav_goal = ExecutePolicyGoal(task_type=ExecutePolicyGoal.GOTO_WAYPOINT, target_id=target_waypoint, time_of_day='all_day')
        nav_action_client.send_goal(nav_goal)
        nav_action_client.wait_for_result()
        
        
    

if __name__ == '__main__':
    rospy.init_node('test_client')
    mdp_planner_client =  MdpPlannerClient()
    mdp_planner_client.main()
