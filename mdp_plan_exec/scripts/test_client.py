#! /usr/bin/env python

import sys
import rospy
import rospkg

from mdp_plan_exec.mdp import TopMapMdp, ProductMdp
from mdp_plan_exec.prism_mdp_manager import PrismMdpManager

    
class MdpPlanner(object):
    
    
    def main(self):     
        manager = PrismMdpManager(8085, 'eliot_dir', 'cs_lg_large2')
        manager.top_map_mdp.update_nav_statistics()
        
        target_waypoint = "WayPoint4"
        

        
        specification='R{"time"}min=? [ (F "' + target_waypoint + '") ]'
        expected_time=float(manager.prism_client.get_policy("all_day",specification))
        result_dir=manager.get_working_dir() + '/all_day'
        
        manager.top_map_mdp.set_reachability_policy(result_dir + "/adv.tra", result_dir + "/prod.sta")
        
        manager.prism_client.shutdown(False)
        
        
        
    

if __name__ == '__main__':
    rospy.init_node('test_client')
    mdp_planner =  MdpPlanner()
    mdp_planner.main()
