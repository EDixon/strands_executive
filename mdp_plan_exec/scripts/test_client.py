#! /usr/bin/env python

import sys
import rospy
import rospkg

from mdp_plan_exec.mdp import TopMapMdp, ProductMdp
    
class MdpPlanner(object):
    def main(self):     
        top_map_mdp=TopMapMdp('lift_simple')
        top_map_mdp.update_nav_statistics()

if __name__ == '__main__':
    rospy.init_node('test_client')
    

        
    mdp_planner =  MdpPlanner()
    mdp_planner.main()
