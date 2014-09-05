#! /usr/bin/env python

import sys
import rospy
import os
import math


#from mdp_plan_exec.prism_client import PrismClient
#from mdp_plan_exec.mdp import TopMapMdp, ProductMdp
from mdp_plan_exec.mdp import ProductMdp
from mdp_plan_exec.prism_mdp_manager import PrismMdpManager

from strands_executive_msgs.srv import AddMdpModel, GetExpectedTravelTime, UpdateNavStatistics, AddDeleteSpecialWaypoint, AddDeleteSpecialWaypointRequest


from strands_executive_msgs.msg import ExecutePolicyAction, ExecutePolicyFeedback, ExecutePolicyGoal, LearnTravelTimesAction
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

from actionlib import SimpleActionServer, SimpleActionClient
from std_msgs.msg import String
from actionlib_msgs.msg import GoalStatus

from strands_navigation_msgs.msg import NavStatistics, MonitoredNavigationAction, MonitoredNavigationActionResult

from mongodb_store.message_store import MessageStoreProxy
from scitos_apps_msgs.msg import DoorCheckAction, DoorWaitGoal, DoorCheckGoal, DoorWaitAction
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from robblog.msg import RobblogEntry
import robblog.utils
from sensor_msgs.msg import Image
import datetime

    
class MdpPlanner(object):

    def __init__(self,top_map):
    
        self.exp_times_handler=PrismMdpManager(8085,'exp_times', top_map)
        self.policy_handler=PrismMdpManager(8086,'policy',top_map)
        
        
        self.travel_time_to_node_service = rospy.Service('/mdp_plan_exec/get_expected_travel_time_to_node', GetExpectedTravelTime, self.travel_time_to_node_cb)
        self.add_mdp_service = rospy.Service('/mdp_plan_exec/add_mdp_model', AddMdpModel, self.add_mdp_cb)
        #self.generate_policy=rospy.Service('/mdp_plan_exec/generate_policy', GeneratePolicy, self.policy_cb)
        self.update_nav_statistics=rospy.Service('mdp_plan_exec/update_nav_statistics',UpdateNavStatistics,self.update_cb)
        
        
        rospy.loginfo("Creating topological navigation client.")
        self.top_nav_action_client= SimpleActionClient('topological_navigation', GotoNodeAction)
        self.top_nav_action_client.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.sleep(0.3)

        rospy.loginfo("Creating door checking client.")
        self.door_check_action_client= SimpleActionClient('door_check', DoorCheckAction)
        self.door_check_action_client.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.sleep(0.3)

        rospy.loginfo("Creating door waiting client.")
        self.door_wait_action_client= SimpleActionClient('door_wait', DoorWaitAction)
        self.door_wait_action_client.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.sleep(0.3)

        rospy.loginfo("Creating monitored navigation client.")
        self.mon_nav_action_client= SimpleActionClient('monitored_navigation', MonitoredNavigationAction)
        self.mon_nav_action_client.wait_for_server()
        rospy.loginfo(" ...done")
        rospy.sleep(0.3)
        
        
        self.executing_policy=False
        self.mdp_navigation_action=SimpleActionServer('mdp_plan_exec/execute_policy', ExecutePolicyAction, execute_cb = self.execute_policy_cb, auto_start = False)
        self.mdp_navigation_action.register_preempt_callback(self.preempt_policy_execution_cb)
        self.mdp_navigation_action.start()
        
        self.learning_travel_times=False
        self.learn_travel_times_action=SimpleActionServer('mdp_plan_exec/learn_travel_times', LearnTravelTimesAction, execute_cb = self.execute_learn_travel_times_cb, auto_start = False)
        self.learn_travel_times_action.register_preempt_callback(self.preempt_learning_cb)
        self.learn_travel_times_action.start()
        
        

        #self.top_map_mdp=TopMapMdp(top_map)
        #self.top_map_mdp.update_nav_statistics()
        
        #self.directory = os.path.expanduser("~") + '/tmp/prism'
        #try:
            #os.makedirs(self.directory)
        #except OSError as ex:
            #print 'error creating PRISM directory:',  ex
            
        #self.mdp_prism_file=self.directory+'/'+top_map+'.prism'    
        
        #self.top_map_mdp.write_prism_model(self.mdp_prism_file)
        
        #self.prism_client.add_model('all_day',self.mdp_prism_file)
        
        self.closest_node=None
        self.current_node=None
        self.nav_action_outcome=''
        self.closest_state_subscriber=rospy.Subscriber('/closest_node', String, self.closest_node_cb)
        self.current_state_subscriber=rospy.Subscriber('/current_node', String, self.current_node_cb)
        self.nav_stats_subscriber = rospy.Subscriber('/topological_navigation/Statistics', NavStatistics, self.get_nav_status_cb)
        
        self.nonitored_nav_result=None
        self.monitored_nav_sub=rospy.Subscriber('/monitored_navigation/result', MonitoredNavigationActionResult, self.get_monitored_nav_status_cb)
        
        self.get_to_exact_pose_timeout=120 #60 secs
        
        
        self.forbidden_waypoints=[]
        self.forbidden_waypoints_ltl_string=''
        
        self.safe_waypoints=[]
        self.safe_waypoints_ltl_string=''
        
        self.special_waypoint_handler_service = rospy.Service('/mdp_plan_exec/add_delete_special_node', AddDeleteSpecialWaypoint, self.add_delete_special_waypoint_cb)
        
        self.msg_store_blog = MessageStoreProxy(collection='robblog')
        self.origin_waypoint=''
        self.target_waypoint=''
        #self.last_stuck_image=None

        
    def add_delete_special_waypoint_cb(self,req):
        if req.waypoint_type == AddDeleteSpecialWaypointRequest.FORBIDDEN:
            if req.is_addition:
                self.add_forbidden_waypoint(req.waypoint)
            else:
                self.del_forbidden_waypoint(req.waypoint)
                
        if req.waypoint_type == AddDeleteSpecialWaypointRequest.SAFE:
            if req.is_addition:
                self.add_safe_waypoint(req.waypoint)
            else:
                self.del_safe_waypoint(req.waypoint)
                
        return True
    
    
    def add_forbidden_waypoint(self,waypoint):
        if waypoint in self.forbidden_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' already in forbidden waypoint list.')
        else:
            self.forbidden_waypoints.append(waypoint)
            self.set_forbidden_waypoints_ltl_string()
                   
    def del_forbidden_waypoint(self,waypoint):
        if waypoint not in self.forbidden_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' not in forbidden waypoint list.')
        else:
            del self.forbidden_waypoints[self.forbidden_waypoints.index(waypoint)]
            self.set_forbidden_waypoints_ltl_string()        
        
        
    def add_safe_waypoint(self,waypoint):
        if waypoint in self.safe_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' already in safe waypoint list.')
        else:
            self.safe_waypoints.append(waypoint)
            self.set_safe_waypoints_ltl_string()
    
    def del_safe_waypoint(self,waypoint):
        if waypoint not in self.safe_waypoints:
            rospy.logwarn('Waypoint ' + waypoint + ' not in safe waypoint list.')
        else:
            del self.safe_waypoints[self.safe_waypoints.index(waypoint)]
            self.set_safe_waypoints_ltl_string()    
        
        
    
    def set_forbidden_waypoints_ltl_string(self):
        self.forbidden_waypoints_ltl_string=''
        
        for i in range(0,len(self.forbidden_waypoints)):
            self.forbidden_waypoints_ltl_string=self.forbidden_waypoints_ltl_string + '"' + self.forbidden_waypoints[i] + '" & !'
        
        if not self.forbidden_waypoints_ltl_string=='':
            self.forbidden_waypoints_ltl_string='(!' + self.forbidden_waypoints_ltl_string[:-4] + ')'
        
        print self.forbidden_waypoints_ltl_string
            

    def set_safe_waypoints_ltl_string(self):
        self.safe_waypoints_ltl_string=''
        
        for i in range(0,len(self.safe_waypoints)):
            self.safe_waypoints_ltl_string=self.safe_waypoints_ltl_string  + '"' + self.safe_waypoints[i] + '"'  + ' | '
        
        if not self.safe_waypoints_ltl_string=='':
            self.safe_waypoints_ltl_string='(' + self.safe_waypoints_ltl_string[:-3] + ')'
            
        print self.safe_waypoints_ltl_string
 
 
 
#-------------------------updating models for both expected time and policy generation 
    def add_mdp_cb(self,req):
        self.exp_times_handler.prism_client.add_model(req.time_of_day,req.mdp_file)
        self.policy_handler.prism_client.add_model(req.time_of_day,req.mdp_file)
        return True
     
    
    def update_cb(self,req): 
        self.exp_times_handler.update_current_top_mdp(req.time_of_day)
        self.policy_handler.update_current_top_mdp(req.time_of_day)
        return True
 
 
#-------------------------expected times     
    def travel_time_to_node_cb(self,req):
        starting_node= req.start_id
        self.exp_times_handler.top_map_mdp.set_initial_state_from_name(starting_node)
        self.exp_times_handler.update_current_top_mdp(req.time_of_day, False)
        specification='R{"time"}min=? [ ( F "' + req.target_id + '") ]'
        result=self.exp_times_handler.prism_client.check_model(req.time_of_day,specification)
        result=float(result)
        return result
        
        

#-------------------------policy generation/execution    
    def execute_learn_travel_times_cb(self,goal):
        
        if self.executing_policy:
            self.preempt_policy_execution_cb()
        
        rospy.set_param('/topological_navigation/mode', 'Node_by_node')
        self.learning_travel_times=True
        timer=rospy.Timer(rospy.Duration(goal.timeout), self.finish_learning_callback,oneshot=True)
        n_successive_fails=0
        current_door_state = 0
        current_wait_state = 0
        while self.learning_travel_times:
            if self.current_node == 'none' or self.current_node is None:
                self.policy_handler.top_map_mdp.set_initial_state_from_name(self.closest_node) 
            else:
                self.policy_handler.top_map_mdp.set_initial_state_from_name(self.current_node)
            current_waypoint=self.policy_handler.top_map_mdp.initial_waypoint
            current_waypoint_trans=self.policy_handler.top_map_mdp.waypoint_transitions[current_waypoint]
            current_trans_count=self.policy_handler.top_map_mdp.waypoint_transitions_transversal_count[current_waypoint]

            #new transitions has action index of all actions
            #waypoint_transitions_transversal_count has action index of waypoint actions only
            current_min=-1
            current_min_index=-1
            for i in range(0,self.policy_handler.top_map_mdp.n_waypoint_actions):
                if current_waypoint_trans[i] is not False:
                    if current_min==-1:
                        current_min=current_trans_count[i]
                        current_min_index=i
                    elif current_trans_count[i]<current_min:
                        current_min=current_trans_count[i]
                        current_min_index=i
            current_action=self.policy_handler.top_map_mdp.actions[current_min_index][current_door_state][current_wait_state]
            split_action = current_action.split('_')
            if split_action[0] == 'goto':
                print 'going somewhere'
                top_nav_goal=GotoNodeGoal()
                top_nav_goal.target=current_action.split('_')[2]
                self.top_nav_action_client.send_goal(top_nav_goal)
                self.top_nav_action_client.wait_for_result()
                if self.nav_action_outcome=='fatal' or self.nav_action_outcome=='failed':
                    n_successive_fails=n_successive_fails+1
                else:
                    n_successive_fails=0
                if n_successive_fails>10:
                    self.policy_handler.update_current_top_mdp('all_day')
                    self.learn_travel_times_action.set_aborted()
                    return
                self.policy_handler.top_map_mdp.waypoint_transitions_transversal_count[current_waypoint][current_min_index]+=1
            elif split_action[0] == 'takedoor':
                print 'Checking if door is open'
                door_check_goal = DoorCheckGoal()
                door_goal = split_action[2]
                door_check_goal.target_pose.pose = self.policy_handler.top_map_mdp.get_waypoint_pose(door_goal)
                self.door_check_action_client.send_goal(door_check_goal)
                self.door_check_action_client.wait_for_result()
                door_status = self.door_check_action_client.get_result()
                if door_status.is_open == True:
                    print 'door is open'
                    current_door_state = 2
                else:
                    print 'door is closed'
                    current_door_state = 1
                if current_door_state == 2:
                    print 'going through door'
                    top_nav_goal = GotoNodeGoal()
                    top_nav_goal.target = split_action[2]
                    self.top_nav_action_client.send_goal(top_nav_goal)
                    self.top_nav_action_client.wait_for_result()
                    current_waypoint = self.current_node
                    current_door_state = 0
                elif current_door_state == 1:
                    attempts = 0
                    while current_door_state == 1:
                        attempts += 1
                        print 'waiting for door'
                        door_wait_goal = DoorWaitGoal()
                        #print dir(door_wait_goal)
                        door_goal = split_action[2]
                        door_wait_goal.target_pose.pose = self.policy_handler.top_map_mdp.get_waypoint_pose(door_goal)
                        door_wait_goal.timeout = 300
                        self.door_wait_action_client.send_goal(door_wait_goal)
                        self.door_wait_action_client.wait_for_result()
                        door_status = self.door_wait_action_client.get_result()
                        if door_status.opened == True:
                            print 'door is open'
                            current_door_state = 2
                            print 'going through door'
                            top_nav_goal=GotoNodeGoal()
                            top_nav_goal.target=split_action[2]
                            self.top_nav_action_client.send_goal(top_nav_goal)
                            self.top_nav_action_client.wait_for_result()
                            if self.nav_action_outcome=='fatal' or self.nav_action_outcome=='failed':
                                n_successive_fails=n_successive_fails+1
                            else:
                                n_successive_fails=0
                            if n_successive_fails>10:
                                self.policy_handler.update_current_top_mdp('all_day')
                                self.learn_travel_times_action.set_aborted()
                                return
                            self.policy_handler.top_map_mdp.waypoint_transitions_transversal_count[current_waypoint][current_min_index]+=1
                        else:
                            print 'door is closed'
                            current_door_state = 1
                    current_door_state = 0
                        # if attempts >= 5:
                        #     door_state = 0
                        #     attempts = 0
        self.exp_times_handler.update_current_top_mdp("all_day")    
        timer.shutdown()       
        
    def finish_learning_callback(self,event):
        self.policy_handler.update_current_top_mdp('all_day')
        self.learn_travel_times_action.set_succeeded()
        self.learning_travel_times=False
        
    def preempt_learning_cb(self):
        self.learning_travel_times=False
        self.policy_handler.update_current_top_mdp('all_day')
        self.top_nav_action_client.cancel_all_goals()
        self.learn_travel_times_action.set_preempted()
 
    def get_monitored_nav_status_cb(self,msg):
        self.monitored_nav_result=msg.status.status

    def execute_policy_cb(self, goal):
        rospy.loginfo("started policy execution")
        if self.learning_travel_times:
            self.preempt_learning_cb()
  
        rospy.set_param('/topological_navigation/mode', 'Normal')
        if self.current_node == 'none' or self.current_node is None:
            self.policy_handler.top_map_mdp.set_initial_state_from_name(self.closest_node) 
        else:
            self.policy_handler.top_map_mdp.set_initial_state_from_name(self.current_node)
        self.policy_handler.update_current_top_mdp(goal.time_of_day)
        if goal.task_type==ExecutePolicyGoal.GOTO_WAYPOINT:
            specification='R{"time"}min=? [ (F "' + goal.target_id + '") ]'
        else:
            #ignore other task types
            print "task type not supported"
            self.mdp_navigation_action.set_aborted()
            return

        feedback=ExecutePolicyFeedback()
        feedback.expected_time=float(self.policy_handler.prism_client.get_policy(goal.time_of_day,specification))
        self.mdp_navigation_action.publish_feedback(feedback)
        if feedback.expected_time==float("inf"):
            rospy.logerr("The goal is unattainable with the current forbidden nodes. Aborting...")
            self.mdp_navigation_action.set_aborted()
            return
        result_dir=self.policy_handler.get_working_dir() + '/' + goal.time_of_day
        
        self.policy_handler.top_map_mdp.set_reachability_policy(result_dir + "/adv.tra", result_dir + "/prod.sta")
        
        self.executing_policy=True

        current_waypoint = self.closest_node
        self.executing_policy = True
        n_successive_fails=0
        door_state = 0
        wait_state = 0
        total_waits = 0
        while current_waypoint != goal.target_id and self.executing_policy and not rospy.is_shutdown():
            waypoint_id = self.policy_handler.top_map_mdp.get_waypoint_from_name(current_waypoint)
            new_action = self.get_next_action(waypoint_id, door_state, wait_state)
            print ' '
            print 'current state: ' + str(self.policy_handler.top_map_mdp.get_waypoint_from_name(current_waypoint)) + ' ' + str(door_state) + ' ' + str(wait_state)
            print 'New action: ' + new_action[0] + '_' + new_action[1] + '_' + new_action[2]
            if new_action[0] == 'goto':
                print 'Moving to new location'
                top_nav_goal = GotoNodeGoal()
                top_nav_goal.target = new_action[2]
                self.top_nav_action_client.send_goal(top_nav_goal)
                self.top_nav_action_client.wait_for_result()
                current_waypoint = self.current_node
                door_state = 0
                wait_state = 0
            elif new_action[0] == 'takedoor':
                print 'Moving to new location'
                top_nav_goal = GotoNodeGoal()
                top_nav_goal.target = new_action[2]
                self.top_nav_action_client.send_goal(top_nav_goal)
                self.top_nav_action_client.wait_for_result()
                current_waypoint = self.current_node
                door_state = 0
                wait_state = 0
            elif new_action[0] == 'checking':
                print 'Checking if door is open'
                door_check_goal = DoorCheckGoal()
                door_goal = new_action[2]
                door_check_goal.target_pose.pose = self.policy_handler.top_map_mdp.get_waypoint_pose(door_goal)
                self.door_check_action_client.send_goal(door_check_goal)
                self.door_check_action_client.wait_for_result()
                door_status = self.door_check_action_client.get_result()
                if door_status.is_open == True:
                    print 'door is open'
                    door_state = 2
                    wait_state = 0
                else:
                    print 'door is closed'
                    door_state = 1
                    wait_state = 0
            elif new_action[0] == 'waiting':
                print 'waiting for door'
                door_wait_goal = DoorWaitGoal()
                #print dir(door_wait_goal)
                door_goal = new_action[2]
                door_wait_goal.target_pose.pose = self.policy_handler.top_map_mdp.get_waypoint_pose(door_goal)
                door_wait_goal.timeout = 300
                self.door_wait_action_client.send_goal(door_wait_goal)
                self.door_wait_action_client.wait_for_result()
                door_status = self.door_wait_action_client.get_result()
                if door_status.opened == True:
                    print 'door is open'
                    door_state = 2
                    wait_state = 1
                else:
                    print 'door is closed'
                    door_state = 1
                    wait_state = 1
                    total_waits += 1
                    closed_prob = self.policy_handler.top_map_mdp.get_door_closed_probability(current_waypoint, self.policy_handler.top_map_mdp.policy[waypoint_id][1][0])
                    k = math.log(0.05, closed_prob)
                    print 'Attempts before replan: ' + str(k)
                    print 'Current attempts: ' + str(total_waits)
                    if total_waits >= k:
                        print 'Tried the door a significant number of times, replanning'
                        self.policy_handler.top_map_mdp.delete_closed_door(current_waypoint)
                        self.policy_handler.update_current_top_mdp(goal.time_of_day)
                        feedback.expected_time=float(self.policy_handler.prism_client.get_policy(goal.time_of_day,specification))
                        if feedback.expected_time==float("inf"):
                            rospy.logerr("The goal is unattainable with the current forbidden nodes. Aborting...")
                            self.mdp_navigation_action.set_aborted()
                            return
            elif new_action[0] == 'setting':
                if new_action[2] == 'open':
                    print 'door set to open'
                    door_state = 2
                    wait_state = 0
                if new_action[2] == 'closed':
                    print 'door set to closed'
                    door_state = 1
                    wait_state = 0
            else:
                print 'No idea what to do, aborting'
                self.mdp_navigation_action.set_aborted()
                self.top_nav_action_client.cancel_all_goals()
                self.executing_policy=False
                return

            if self.nav_action_outcome=='fatal' or self.nav_action_outcome=='failed':
                n_successive_fails=n_successive_fails+1
            else:
                n_successive_fails=0

            if n_successive_fails>4:
                rospy.logerr("Five successive fails in topological navigation. Aborting...")
                self.executing_policy=False
                self.mon_nav_action_client.cancel_all_goals()
                self.top_nav_action_client.cancel_all_goals()
                self.mdp_navigation_action.set_aborted()
                return

        print 'got to goal'
        self.mdp_navigation_action.set_succeeded()
        return

    def get_next_action(self, w, d, t):
        next_action = self.policy_handler.top_map_mdp.policy[w][d][t]
        split_action = next_action.split('_')
        action_type = 'none'
        initial_waypoint = 'none'
        final_waypoint = 'none'
        if split_action[0] == 'goto':
            initial_waypoint = split_action[1]
            final_waypoint = split_action[2]
            action_type = 'goto'
        elif split_action[0] == 'takedoor':
            initial_waypoint = split_action[1]
            final_waypoint = split_action[2]
            action_type = 'takedoor'
        elif split_action[0] == 'wait':
            action_type = 'waiting'
            final_waypoint = self.policy_handler.top_map_mdp.get_door_waypoint_from_door_name(split_action[2])
            #action_type = 'none'
        elif split_action[0] == 'check':
            action_type = 'checking'
            final_waypoint = self.policy_handler.top_map_mdp.get_door_waypoint_from_door_name(split_action[1])
            #action_type = 'none'
        elif split_action[0] == 'set':
            action_type = 'setting'
            initial_waypoint = split_action[1]
            final_waypoint = split_action[2]
        return [action_type, initial_waypoint, final_waypoint]
                
        
        self.exp_times_handler.update_current_top_mdp(goal.time_of_day)
        
        self.monitored_nav_result=None
        timeout_counter=0
        while self.monitored_nav_result is None and self.executing_policy and timeout_counter < self.get_to_exact_pose_timeout:     
            rospy.sleep(0.5)
            timeout_counter=timeout_counter+1
        
        
        if self.executing_policy:
            self.executing_policy=False
            if self.monitored_nav_result==GoalStatus.PREEMPTED:
                self.mdp_navigation_action.set_preempted()
                return
            if self.monitored_nav_result==GoalStatus.SUCCEEDED and self.current_node == goal.target_id:
                self.mdp_navigation_action.set_succeeded()
                return
            if self.monitored_nav_result==GoalStatus.ABORTED or self.monitored_nav_result is None or not self.current_node == goal.target_id:
                rospy.logerr("Failure in getting to exact pose in goal waypoint")
                self.mdp_navigation_action.set_aborted()
                return
            

 
    def get_monitored_nav_status_cb(self,msg):
        self.monitored_nav_result=msg.status.status
        
 
 
            
    def preempt_policy_execution_cb(self):
        self.executing_policy=False
        self.mon_nav_action_client.cancel_all_goals()
        self.top_nav_action_client.cancel_all_goals()
        self.mdp_navigation_action.set_preempted()
        
        
    def unexpected_trans_time_cb(self,event):
        last_stuck_image = None
        image_topic =  '/head_xtion/rgb/image_color'
        #image_topic =  '/head_xtion/rgb/image_mono'  #simulation topic
        
        #count = 0
        #while self.last_stuck_image == None and  not rospy.is_shutdown() and count < 10:
            #rospy.loginfo('waiting for image of possible blocked path %s' % count)
            #count += 1
            #rospy.sleep(1)
            
        last_stuck_image=rospy.wait_for_message(image_topic, Image , timeout=10.0)
            
        e = RobblogEntry(title=datetime.datetime.now().strftime("%H:%M:%S") + ' - Possible Blocked Path')
        e.body = 'It took me a lot more time to go between ' + self.origin_waypoint + ' and ' + self.target_waypoint + ' than I was expecting. Something might be blocking the way.'
            
        if last_stuck_image != None:
            img_id = self.msg_store_blog.insert(last_stuck_image)
            rospy.loginfo('adding possible blockage image to blog post')
            e.body += ' Here is what I saw: \n\n![Image of the door](ObjectID(%s))' % img_id
        self.msg_store_blog.insert(e)
        
        
    #def img_callback(self, img):
        #self.last_stuck_image = img
    
        
    
    def closest_node_cb(self,msg):
        self.closest_node=msg.data
        
    def current_node_cb(self,msg):
        self.current_node=msg.data

    def get_nav_status_cb(self,msg):   
        self.nav_action_outcome=msg.status
    
    def main(self):


        # Wait for control-c
        rospy.spin()
        
        if rospy.is_shutdown():
            self.exp_times_handler.prism_client.shutdown(True)
            self.policy_handler.prism_client.shutdown(True)


if __name__ == '__main__':
    rospy.init_node('mdp_planner')
    
    if len(sys.argv)<2:
        print "usage: rosrun mdp_plan_exec mdp_planner <topological_map_name>"
        sys.exit(2)
        
    mdp_planner =  MdpPlanner(sys.argv[1])
    
    
    
    
    mdp_planner.main()
    
    
