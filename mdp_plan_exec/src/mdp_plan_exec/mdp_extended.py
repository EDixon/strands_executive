#!/usr/bin/python


import sys
import rospy
import math
import copy

from mongodb_store.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import NavStatistics
from scitos_apps_msgs.msg import DoorWaitStat, DoorCheckStat
from strands_navigation_msgs.msg import NavRoute
from geometry_msgs.msg import Pose


class Mdp(object):

    def __init__(self):
        #list of attributes of an MDP object
        self.top_map=''
        self.n_waypoint_props = 0
        self.waypoint_props = []
        self.n_waypoint_actions=0
        self.waypoint_actions=[]
        self.waypoint_prop_map = [[]]
        self.waypoint_transitions=[[]]
        self.waypoint_transitions_transversal_count=[[]]
        self.waypoint_rewards=[[]]
        self.current_policy=[]
        self.initial_waypoint = 0
        self.n_waypoints = 0
        self.waypoint_names = []
        self.door_ids=[]
        self.initial_door_state = 0
        self.n_door_states = 3
        self.initial_wait_state = 0
        self.n_wait_states = 2
        self.n_doors = 0
        self.n_edges = 0
        self.door_open_probs = []
        self.normal_door_open_prob = 0.5
        self.door_wait_open_prob = []
        self.new_transitions = [[[[]]]]
        self.new_rewards = [[[[]]]]
        self.new_actions = []
        self.n_new_actions = 0
        self.door_names = []
        self.door_waypoints = []
        self.nearside_door_waypoint_names = []
        self.n_unique_doors = 0
        self.unique_doors = [[]]
        self.unique_door_ids = []

    def write_prism_model(self,file_name):
        f=open(file_name,'w')
        #states
        #print 'test version'
        f.write('mdp\n \n')
        f.write('module M \n \n')
        f.write('w:[0..'+str(self.n_waypoints-1)+'] init ' + str(self.initial_waypoint) + ';\n')
        f.write('d:[0..'+str(self.n_unique_doors-1)+'] init ' + str(self.initial_door_state) + ';\n')
        f.write('t:[0..'+str(self.n_wait_states-1)+'] init ' + str(self.initial_wait_state) + ';\n \n')
        for i in range(0,self.n_waypoints):
            for j in self.unique_door_ids:
                for k in range(0 , self.n_wait_states):
                    for l in range(0 , self.n_new_actions):
                        current_trans_list = self.new_transitions[i][j][k][l]
                        if current_trans_list:
                            trans_string = '[' + self.new_actions[l] + '] (w=' + str(i) + ') & (d=' + str(j) + ') & (t=' + str(k) + ') -> '
                            for trans in current_trans_list:
                                trans_string = trans_string + str(trans[3]) + ":(w'=" + str(trans[0]) + ") & (d'=" + str(trans[1]) + ") & (t'=" + str(trans[2]) + ') + '
                                #print 'goto_'+self.waypoint_names[i] + '_' + self.waypoint_names[trans[0]]
                            f.write(trans_string[:-3] + ';\n')

        f.write('\nendmodule\n\n')

        # f.write('mdp\n \n')
        # f.write('module M \n \n')
        # f.write('w:[0..'+str(self.n_waypoints-1)+'] init ' + str(self.initial_waypoint) + ';\n')
        # f.write('d:[0..'+str(self.n_unique_doors-1)+'] init ' + str(self.initial_door_state) + ';\n')
        # f.write('t:[0..'+str(self.n_wait_states-1)+'] init ' + str(self.initial_wait_state) + ';\n \n')
        # for i in range(0,self.n_waypoints):
        #     for j in self.unique_door_ids:
        #         for k in range(0 , self.n_wait_states):
        #             for l in range(0 , self.n_new_actions):
        #                 current_trans_list = self.new_transitions[i][j][k][l]
        #                 if current_trans_list:
        #                     door_list = self.decimal_to_ternary(j)
        #                     trans_string = '[' + self.new_actions[l] + '] (w=' + str(i) + ') '
        #                     for d_id in range(len(door_list)):
        #                         trans_string = trans_string + '& (d' + str(d_id) + '=' + str(door_list[d_id]) + ') '
        #                     trans_string = trans_string + '& (d=' + str(j) + ') & (t=' + str(k) + ') -> '
        #                     for trans in current_trans_list:
        #                         door_list = self.decimal_to_ternary(trans[1])
        #                         trans_string = trans_string + str(trans[3]) + ":(w'=" + str(trans[0]) + ") "
        #                         for d_id in range(len(door_list)):
        #                             trans_string = trans_string + '& (d' + str(d_id) + '=' + str(door_list[d_id]) + ') '
        #                         trans_string = trans_string + "& (t'=" + str(trans[2]) + ') + '
        #                         #print 'goto_'+self.waypoint_names[i] + '_' + self.waypoint_names[trans[0]]
        #                     f.write(trans_string[:-3] + ';\n')
        #
        # f.write('\nendmodule\n\n')

        i = 0
        for i in range(0,self.n_waypoint_props):
            f.write('label "'+ self.waypoint_props[i] + '" = ')
            prop_string=''
            for j in range(0,self.n_waypoints):
                if self.waypoint_prop_map[j][i]:
                   prop_string=prop_string + 'w=' + str(j) + ' | '
            f.write(prop_string[:-3] + ';\n')

        f.write('label "door_waited" = t=1;\n')
        f.write('label "door_not_waited" =t=0;\n')

        for i in range(self.n_doors):
            open_string = 'label "door' + str(i) + '_open" ='
            closed_string = 'label "door' + str(i) + '_closed" ='
            unknown_string = 'label "door' + str(i) + '_unknown" ='
            for k in range(self.n_unique_doors):
                if self.unique_doors[k][i] == 2:
                    open_string = open_string + 'd=' + str(self.unique_door_ids[k]) + ' | '
                    #f.write('label "door' + str(i) + '_open" = d=' + str(self.unique_door_ids[k]) + ';\n')
                if self.unique_doors[k][i] == 1:
                    closed_string = closed_string + 'd=' + str(self.unique_door_ids[k]) + ' | '
                    #f.write('label "door' + str(i) + '_closed" =d=' + str(self.unique_door_ids[k]) + ';\n')
                if self.unique_doors[k][i] == 0:
                    unknown_string = unknown_string + 'd=' + str(self.unique_door_ids[k]) + ' | '
                    #f.write('label "door' + str(i) + '_unknown" =d=' + str(self.unique_door_ids[k]) + ';\n')
            f.write(open_string[:-3] + ';\n')
            f.write(closed_string[:-3] + ';\n')
            f.write(unknown_string[:-3] + ';\n')

        f.write('\n')
        f.write('rewards "time"\n')

        for i in range(0,self.n_waypoints):
            for j in self.unique_door_ids:
                for k in range(0 , self.n_wait_states):
                    for l in range(0 , self.n_new_actions):
                        if self.new_rewards[i][j][k][l] != 0:
                            f.write('    [' + self.new_actions[l] + '] (w=' + str(i) + ') & (d=' + str(j) + ') & (t=' +
                                    str(k) + '):' + str(self.new_rewards[i][j][k][l]) + ';\n')

        f.write('endrewards\n')

        f.close()

    def set_initial_state(self,initial_state):
        self.initial_waypoint=initial_state

    def set_initial_waypoint(self, initial_waypoint):
        self.initial_waypoint=initial_waypoint

    def set_initial_door_state(self, initial_door_state):
        self.initial_door_state=initial_door_state

    def set_initial_wait_state(self, initial_wait_state):
        self.initial_wait_state=initial_wait_state

    def get_expected_edge_transversal_time(self,state_index,action_name):
        action_index=self.waypoint_actions.index(action_name)
        return self.waypoint_rewards[state_index][action_index]

    def get_total_transversals(self,state_index,action_name):
        action_index=self.waypoint_actions.index(action_name)
        return self.waypoint_transitions_transversal_count[state_index][action_index]

    def set_door_ids(self, unique_door_ids):
        self.unique_door_ids = unique_door_ids

    def decimal_to_ternary(self, state):
        doors = []
        remainder = 0
        while state > 0:
            state, remainder = divmod(state, 3)
            doors.append(remainder)
        return doors

class TopMapMdp(Mdp):
    def __init__(self, top_map_name):

        self.top_map=top_map_name

        self.initial_waypoint=0
        self.initial_door_state = 0
        self.initial_wait_state = 0
        self.top_nodes=self.read_top_map()

        self.normal_door_open_prob = 0.5
        self.n_waypoints=len(self.top_nodes)
        self.n_door_states = 3
        self.n_wait_states = 2
        self.waypoint_names=[None]*self.n_waypoints
        self.n_unique_doors = 0
        self.unique_doors = [[]]
        self.nearside_door_waypoint_names = []

        self.n_props=self.n_waypoints
        self.n_waypoint_props = self.n_waypoints
        self.waypoint_props = [None]*self.n_waypoint_props
        self.waypoint_prop_map=[[False]*self.n_waypoint_props for i in range(self.n_waypoints)]
        for i in range(0,self.n_waypoint_props):
            self.waypoint_prop_map[i][i]=True
        i = 0
        self.n_waypoint_actions=0
        for entry in self.top_nodes:
            self.waypoint_props[i]=entry[0].name
            self.waypoint_names[i]=entry[0].name
            #print 'Waypoint ' + str(i) + '=' + entry[0].name
            self.n_waypoint_actions=self.n_waypoint_actions+len(entry[0].edges)
            i=i+1

        i=0
        self.waypoint_rewards=[[0]*self.n_waypoint_actions for i in range(self.n_waypoints)]
        self.waypoint_transitions=[[False]*self.n_waypoint_actions for i in range(self.n_waypoints)]
        self.waypoint_transitions_transversal_count=[[0]*self.n_waypoint_actions for i in range(self.n_waypoints)]
        self.waypoint_actions=[None]*self.n_waypoint_actions

        #self.new_actions = [None]*self.n_waypoint_actions

        action_index=0
        state_index=0
        doors=0
        print 'Generating list of doors'
        for entry in self.top_nodes:
            current_edges=entry[0].edges
            for edge in current_edges:
                if edge.action == 'doorPassing':
                    doors += 1
                    self.waypoint_actions[action_index] = 'takedoor_'+self.waypoint_names[state_index] + '_' + edge.node
                    action_index=action_index+1
                else:
                    #self.new_actions[action_index] = 'goto_'+self.waypoint_names[state_index] + '_' + edge.node
                    self.waypoint_actions[action_index] = 'goto_'+self.waypoint_names[state_index] + '_' + edge.node
                    action_index=action_index+1
            state_index=state_index+1
        self.n_doors = doors
        self.generate_door_ids()
        self.new_transitions = [[[[False]*((action_index+1) + (5*doors)) for i in range(self.n_wait_states)] for j in range(self.n_unique_doors)] for k in range(self.n_waypoints)]
        self.new_rewards = [[[[0]*((action_index+1) + (5*doors)) for i in range(self.n_wait_states)] for j in range(self.n_unique_doors)] for k in range(self.n_waypoints)]
        self.door_open_probs = [0]*doors
        self.door_wait_open_probs = [0]*doors
        self.door_names = [None]*doors
        self.door_waypoints = [None]*doors
        self.nearside_door_waypoint_names = [None]*doors
        self.new_actions = []
        doors_state = []
        for i in range(self.n_doors):
            doors_state.append(0)
        initial_door_state = self.ternary_to_decimal(doors_state)
        self.initial_door_state = initial_door_state
        self.set_initial_door_state(initial_door_state)
        
        self.policy = [[[None for i in range(self.n_wait_states)] for j in range(self.n_unique_doors)] for k in range(self.n_waypoints)]

        state_index=0
        action_index=0
        waypoint_action_index = 0
        door_id=0
        for entry in self.top_nodes:
            current_edges=entry[0].edges
            for edge in current_edges:
                target_index=self.waypoint_names.index(edge.node)
                #print ' '
                #print edge.action
                if edge.action == 'doorPassing':
                    #print 'adding door passing'
                    #print '  '
                    #print str(door_id)
                    self.door_open_probs[door_id] = self.normal_door_open_prob
                    self.door_wait_open_probs[door_id] = self.normal_door_open_prob
                    self.new_actions.append('check_door' + str(door_id))
                    for k in range(self.n_unique_doors):
                        if self.unique_doors[k][door_id] == 0:
                            door_config = copy.copy(self.unique_doors[k])
                            door_config[door_id] = 1
                            door_closed_id = self.ternary_to_decimal(door_config)
                            door_config[door_id] = 2
                            door_open_id = self.ternary_to_decimal(door_config)
                            self.new_transitions[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = [[state_index,door_closed_id,0,(1-self.door_open_probs[door_id])], [state_index,door_open_id,0, self.door_open_probs[door_id]]]
                            self.new_rewards[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = 1
                    self.new_actions.append('wait_for_door' + str(door_id))
                    for k in range(self.n_unique_doors):
                        if self.unique_doors[k][door_id] == 1:
                            door_config = copy.copy(self.unique_doors[k])
                            door_config[door_id] = 1
                            door_closed_id = self.ternary_to_decimal(door_config)
                            door_config[door_id] = 2
                            door_open_id = self.ternary_to_decimal(door_config)
                            self.new_transitions[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = [[state_index,door_closed_id,1,(1-self.door_wait_open_probs[door_id])],[state_index,door_open_id,1,self.door_wait_open_probs[door_id]]]
                            self.new_rewards[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = 1
                    self.new_actions.append('set_door' + str(door_id) + '_open')
                    for k in range(self.n_unique_doors):
                        if self.unique_doors[k][door_id] == 2:
                            self.new_transitions[state_index][self.unique_door_ids[k]][1][len(self.new_actions)-1] = [[state_index,self.unique_door_ids[k],0,1]]
                            self.new_rewards[state_index][self.unique_door_ids[k]][1][len(self.new_actions)-1] = 120
                    self.new_actions.append('takedoor_' + self.waypoint_names[state_index] + '_' + edge.node)
                    for k in range(self.n_unique_doors):
                        if self.unique_doors[k][door_id] == 2:
                            door_config = copy.copy(self.unique_doors[k])
                            door_config[door_id] = 0
                            door_unknown_id = self.ternary_to_decimal(door_config)
                            self.new_transitions[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = [[target_index,door_unknown_id,0,1]]
                            self.new_rewards[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = 15
                    self.new_actions.append('set_door' + str(door_id) + '_closed')
                    for k in range(self.n_unique_doors):
                        if self.unique_doors[k][door_id] == 1:
                            self.new_transitions[state_index][self.unique_door_ids[k]][1][len(self.new_actions)-1] = [[state_index,self.unique_door_ids[k],0,1]]
                            self.new_rewards[state_index][self.unique_door_ids[k]][1][len(self.new_actions)-1] = 300
                    self.door_names[door_id] = ('door' + str(door_id))
                    self.door_waypoints[door_id] = edge.node
                    self.waypoint_transitions[state_index][waypoint_action_index]= [[target_index,1]]
                    self.waypoint_rewards[state_index][waypoint_action_index]=1
                    self.nearside_door_waypoint_names[door_id] = self.waypoint_names[state_index]
                    door_id += 1
                    waypoint_action_index += 1
                else:
                    #print 'adding goto'
                    self.new_actions.append('goto_'+self.waypoint_names[state_index] + '_' + edge.node)
                    #print 'goto_'+self.waypoint_names[state_index] + '_' + edge.node
                    #print 'stored action: ' + self.new_actions[len(self.new_actions)-1]
                    #print 'waypoint: ' + str(state_index)
                    #print 'waypoint id: ' + str(state_index) + ' Waypoint name: ' + self.waypoint_names[state_index]
                    for k in range(self.n_unique_doors):
                        self.new_transitions[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = [[target_index, self.unique_door_ids[k], 0, 1]]
                        self.new_rewards[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = 1
                    self.waypoint_transitions[state_index][waypoint_action_index]= [[target_index,1]]
                    self.waypoint_rewards[state_index][waypoint_action_index]=1
                    waypoint_action_index += 1
                action_index=action_index+1


            state_index=state_index+1

        self.n_new_actions = len(self.new_actions)

    def get_door_id_from_waypoint(self, waypoint_name):
        door_id = self.nearside_door_waypoint_names.index(waypoint_name)
        return door_id

    def generate_door_lists(self):
        n_doors = self.n_doors
        n_door_states = 3
        old_array = []
        k = 0
        for i in range(n_door_states):
            old_array.append([i])
        for d in range(n_doors - 1):
            new_array = []
            for i in range(n_door_states):
                for j in range(len(old_array)):
                    new_array.append(copy.copy(old_array[j]))
                    new_array[k].append(i)
                    k += 1
            old_array = new_array
            k=0
        self.n_unique_doors = len(old_array)
        self.unique_doors = old_array

    def generate_door_ids(self):
        self.generate_door_lists()
        door_ids = [0]*self.n_unique_doors
        for i in range(self.n_unique_doors):
            door_ids[i] = self.ternary_to_decimal(self.unique_doors[i])
        self.unique_door_ids = door_ids
        self.set_door_ids(door_ids)

    def ternary_to_decimal(self, doors):
        n_doors = len(doors)
        dec = 0
        for i in range(n_doors):
            dec = dec + doors[i]*pow(3, i)
        return dec

    def decimal_to_ternary(self, state):
        doors = []
        remainder = 0
        while state > 0:
            state, remainder = divmod(state, 3)
            doors.append(remainder)
        return doors

    def read_top_map(self):
        msg_store = MessageStoreProxy(collection='topological_maps')

        query_meta = {}
        query_meta["pointset"] = self.top_map
        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0


        if available <= 0 :
            rospy.logerr("Desired pointset "+ self.top_map +" not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")

        else :
            query_meta = {}
            query_meta["pointset"] = self.top_map
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)

        return message_list


    def update_nav_statistics(self):
        msg_store = MessageStoreProxy()
        door_store = MessageStoreProxy(collection='door_stats')
        query_meta = {}
        query_meta["pointset"] = self.top_map
        #print self.top_map
        message_list = msg_store.query(NavStatistics._type, {}, query_meta)
        wait_list = door_store.query(DoorWaitStat._type, {}, {})
        check_list = door_store.query(DoorCheckStat._type, {}, {})
        n_data=len(message_list)
        n_unprocessed_data=n_data
        total_door_checks = 0
        total_doors_open = 0
        for i in range(0,self.n_new_actions):
            current_action=self.new_actions[i]
            #print 'Current action: ' + current_action
            if 'goto' in current_action:
                new_action_index = self.new_actions.index(current_action)
                action_index=self.waypoint_actions.index(current_action)
                current_action=current_action.split('_')
                source_index=self.waypoint_names.index(current_action[1])
                target_index=self.waypoint_names.index(current_action[2])
                #door_id = self.nearside_door_waypoint_names.index(current_action[1])
                j=0
                n_total_data=1
                expected_time=0
                total_outcomes_count=1
                outcomes_count=[0]*self.n_waypoints
                outcomes_count[target_index]=1
                while j<n_unprocessed_data:
                    entry=message_list[j]
                    if current_action[1]==entry[0].origin and current_action[2]==entry[0].target and not entry[0].final_node == 'Unknown':
                        n_total_data=n_total_data+1
                        expected_time=expected_time+float(entry[0].operation_time)#-float(entry[0].time_to_waypoint)
                        outcomes_count[self.waypoint_names.index(entry[0].final_node)]+=1
                        total_outcomes_count=total_outcomes_count+1
                        del message_list[j]
                        n_unprocessed_data=n_unprocessed_data-1
                    else:
                        j=j+1
                for k in range(self.n_unique_doors):
                        if n_total_data==1:
                            rospy.logwarn("No data for edge between waypoints " + current_action[1] + " and " + current_action[2] + ". Assuming it to be 20 seconds. Expected time between nodes will not be correct.")
                            self.waypoint_rewards[source_index][action_index]=20
                            self.new_rewards[source_index][self.unique_door_ids[k]][0][new_action_index] = 20
                        else:
                            self.waypoint_rewards[source_index][action_index]=expected_time/(total_outcomes_count-1)
                            self.new_rewards[source_index][self.unique_door_ids[k]][0][new_action_index]= expected_time/(total_outcomes_count-1)
                            self.waypoint_transitions_transversal_count[source_index][action_index]=total_outcomes_count-1
                            transition=None
                            new_transition=None
                            for j in range(0,self.n_waypoints):
                                count=outcomes_count[j]
                                if count > 0:
                                    probability=float(count)/float(total_outcomes_count)
                                    if transition is None:
                                        transition=[[j, probability]]
                                        new_transition = [[j,self.unique_door_ids[k],0,probability]]
                                    else:
                                        transition.append([j,probability])
                                        new_transition.append([j,self.unique_door_ids[k],0,probability])
                            if transition is not None:
                                self.waypoint_transitions[source_index][action_index]=transition
                                self.new_transitions[source_index][self.unique_door_ids[k]][0][new_action_index] = new_transition
            if 'takedoor' in current_action:
                #print 'found door transition'
                n_data = len(message_list)
                n_unprocessed_data=n_data
                action_index=self.waypoint_actions.index(current_action)
                new_action_index = self.new_actions.index(current_action)
                current_action=current_action.split('_')
                source_index=self.waypoint_names.index(current_action[1])
                target_index=self.waypoint_names.index(current_action[2])
                door_id = self.nearside_door_waypoint_names.index(current_action[1])
                j=0
                n_total_data=1
                expected_time=0
                total_outcomes_count=1 #total number of outcomes of the action
                outcomes_count=[0]*self.n_waypoints #the number of times waypoint is an outcome for the current action
                outcomes_count[target_index]=1
                while j<n_unprocessed_data:
                    entry=message_list[j]
                    if current_action[1]==entry[0].origin and current_action[2]==entry[0].target and not entry[0].final_node == 'Unknown':
                        n_total_data=n_total_data+1
                        expected_time=expected_time+float(entry[0].operation_time)#-float(entry[0].time_to_waypoint)
                        outcomes_count[self.waypoint_names.index(entry[0].final_node)]+=1
                        total_outcomes_count=total_outcomes_count+1
                        del message_list[j]
                        n_unprocessed_data=n_unprocessed_data-1
                    else:
                        j=j+1
                for k in range(self.n_unique_doors):
                        if self.unique_doors[k][door_id] == 2:
                            door_config = copy.copy(self.unique_doors[k])
                            door_config[door_id] = 0
                            door_unknown_id = self.ternary_to_decimal(door_config)
                            if n_total_data==1:
                                rospy.logwarn("No data for edge between door waypoints " + current_action[1] + " and " + current_action[2] + ". Assuming it to be 20 seconds. Expected time between nodes will not be correct.")
                                self.new_rewards[source_index][self.unique_door_ids[k]][0][new_action_index] = 20
                            else:
                                self.new_rewards[source_index][self.unique_door_ids[k]][0][new_action_index]= expected_time/(total_outcomes_count-1)
                                self.waypoint_transitions_transversal_count[source_index][action_index]=total_outcomes_count-1
                                new_transition=None
                                transition=None
                                for j in range(0,self.n_waypoints):
                                    count=outcomes_count[j]
                                    if count > 0:
                                        probability=float(count)/float(total_outcomes_count)
                                        if new_transition is None:
                                            transition=[[j, probability]]
                                            new_transition = [[j,door_unknown_id,0,probability]]
                                        else:
                                            transition.append([j,probability])
                                            new_transition.append([j,door_unknown_id,0,probability])
                                if new_transition is not None:
                                    self.waypoint_transitions[source_index][action_index]=transition
                                    self.new_transitions[source_index][self.unique_door_ids[k]][0][new_action_index] = new_transition
            if 'check' in current_action:
                #print 'found a door check'
                n_data = len(check_list)
                n_unprocessed_data = n_data
                new_action_index = self.new_actions.index(current_action)
                current_action = current_action.split('_')
                door_name = current_action[1]
                door_index = self.door_names.index(door_name)
                door_waypoint = self.nearside_door_waypoint_names[door_index]
                waypoint_id = self.waypoint_names.index(door_waypoint)
                door_id = self.door_names.index(current_action[1])
                j=0
                total_data_count = 1
                successes = 0
                while j<n_unprocessed_data:
                    entry = check_list[j]
                    if door_waypoint == entry[0].origin:
                        #print 'data match found'
                        total_data_count += 1
                        total_door_checks += 1
                        if entry[0].is_open == True:
                            #print 'success found'
                            successes += 1
                            total_doors_open += 1
                        del check_list[j]
                        n_unprocessed_data = n_unprocessed_data - 1
                    else:
                        j = j + 1
                for k in range(self.n_unique_doors):
                        if self.unique_doors[k][door_id] == 0:
                            door_config = copy.copy(self.unique_doors[k])
                            door_config[door_id] = 0
                            door_unknown_id = self.ternary_to_decimal(door_config)
                            door_config[door_id] = 1
                            door_closed_id = self.ternary_to_decimal(door_config)
                            door_config[door_id] = 2
                            door_open_id = self.ternary_to_decimal(door_config)
                            new_transition = None
                            if total_data_count > 0 and successes > 0:
                                success_probability = float(successes)/float(total_data_count)
                                fail_probability = 1-success_probability
                                new_transition = [[waypoint_id, door_open_id, 0, success_probability]]
                                new_transition.append([waypoint_id, door_closed_id, 0, fail_probability])
                                self.new_transitions[waypoint_id][door_unknown_id][0][new_action_index] = new_transition
                            else:
                                #in the case that there is no data or no success, assume a 50:50 chance
                                #print 'adding new door check'
                                new_transition = [[waypoint_id, door_open_id, 0, 0.5]]
                                new_transition.append([waypoint_id, door_closed_id, 0, 0.5])
                                self.new_transitions[waypoint_id][door_unknown_id][0][new_action_index] = new_transition
            if 'wait' in current_action:
                #print 'found a door wait'
                n_data = len(wait_list)
                n_unprocessed_data = n_data
                new_action_index = self.new_actions.index(current_action)
                current_action = current_action.split('_')
                door_name = current_action[2]
                door_index = self.door_names.index(door_name)
                door_waypoint = self.nearside_door_waypoint_names[door_index]
                waypoint_id = self.waypoint_names.index(door_waypoint)
                reset_action_fail = self.new_actions.index('set_' + door_name + '_closed')
                reset_action_success = self.new_actions.index('set_' + door_name + '_open')
                door_id = self.door_names.index(current_action[2])
                j=0
                total_data_count = 0
                successes = 0
                failures = 0
                total_failure_time = 0
                total_success_time = 0
                #possibly could add a system that detects a repeated waiting action and adds them up and assigns their sum to the failure time.
                #the final position in that set is taken away from from the total_success_time, and the final success is not allowed to be added
                #also, the sum of the failures in that set is taken away from total_failure_time
                #and the
                while j<n_unprocessed_data:
                    entry = wait_list[j]
                    if door_waypoint == entry[0].origin:
                        #print 'data match found'
                        total_data_count += 1
                        if entry[0].opened == True:
                            #print 'success found'
                            total_success_time=total_success_time+(float(entry[0].wait_time.secs))
                            successes += 1
                        elif entry[0].opened == False:
                            total_failure_time=total_failure_time+(float(entry[0].wait_time.secs))
                            failures += 1
                        del wait_list[j]
                        n_unprocessed_data = n_unprocessed_data - 1
                    else:
                        j = j + 1
                for k in range(self.n_unique_doors):
                        if self.unique_doors[k][door_id] == 0:
                            door_config = copy.copy(self.unique_doors[k])
                            door_config[door_id] = 0
                            door_unknown_id = self.ternary_to_decimal(door_config)
                            door_config[door_id] = 1
                            door_closed_id = self.ternary_to_decimal(door_config)
                            door_config[door_id] = 2
                            door_open_id = self.ternary_to_decimal(door_config)
                            new_transition = None
                            if total_data_count > 0 and successes > 0:
                                success_probability = float(successes)/float(total_data_count)
                                fail_probability = 1-success_probability
                                new_transition = [[waypoint_id, door_open_id, 1, success_probability]]
                                new_transition.append([waypoint_id, door_closed_id, 1, fail_probability])
                                self.new_transitions[waypoint_id][door_closed_id][0][new_action_index] = new_transition
                                self.new_rewards[waypoint_id][door_open_id][1][reset_action_success]= total_success_time/successes
                                if failures > 0:
                                    self.new_rewards[waypoint_id][door_closed_id][1][reset_action_fail]= total_failure_time/failures
                                else:
                                    self.new_rewards[waypoint_id][door_closed_id][1][reset_action_fail]= 300
                            else:
                                #in the case that there is no data or no success, assume a 50:50 chance
                                #print 'adding new door wait'
                                new_transition = [[waypoint_id, door_open_id, 1, 0.5]]
                                new_transition.append([waypoint_id, door_closed_id, 1, 0.5])
                                self.new_transitions[waypoint_id][door_closed_id][0][new_action_index] = new_transition
                                self.new_rewards[waypoint_id][door_open_id][1][reset_action_success]= 120
                                self.new_rewards[waypoint_id][door_closed_id][1][reset_action_fail]= 300


        if total_doors_open > 0:
            self.door_open_probs = total_doors_open/total_door_checks

    def set_initial_state_from_name(self,state_name):
        index=self.waypoint_names.index(state_name)
        self.set_initial_state(index)
        self.set_initial_waypoint(index)
        doors_state = []
        for i in range(self.n_doors):
            doors_state.append(0)
        initial_door_state = self.ternary_to_decimal(doors_state)
        self.set_initial_door_state(initial_door_state)
        
    def set_reachability_policy(self, policy_file, product_sta):
            #    self.policy = [[[[None] for i in range(self.n_wait_states)] for j in range(self.n_door_states)] for k in range(self.n_waypoints)]
 
        policy_f = open(policy_file, 'r')    
        sta_f = open(product_sta, 'r')
        
        n_states = int(policy_f.readline().split(' ')[0])
        sta_f.readline()
        
        sta_line = sta_f.readline()
        [sta_state_id, state_labels] = sta_line.split(':')
        for policy_line in policy_f:
            [pol_state_id, foo, foo, action] = policy_line.split(' ')
            action = action.rstrip('\n')
            found_id = False
            while not found_id:
                if pol_state_id == sta_state_id:
                    [foo, w, d, t] = state_labels.split(',')
                    t = t[:-2]
                    self.policy[int(w)][int(d)][int(t)] = action
                    found_id = True
                else:
                    sta_line = sta_f.readline()
                    [sta_state_id, state_labels] = sta_line.split(':')
                
                
    def get_waypoint_pose(self, waypoint_name):
        for entry in self.top_nodes:
            if waypoint_name == entry[0].name:
                return entry[0].pose
        rospy.logwarn("cannot find waypoint pose: " + waypoint_name)
        return False
        
    def get_waypoint_from_name(self, waypoint_name):
        index = self.waypoint_names.index(waypoint_name)
        return index

    def get_door_waypoint_from_door_name(self, door_name):
        door_index = self.door_names.index(door_name)
        waypoint = self.door_waypoints[door_index]
        return waypoint

    def get_door_closed_probability(self, waypoint_name, new_action):
        waypoint_id = self.waypoint_names.index(waypoint_name)
        new_action_index = self.new_actions.index(new_action)
        door_id = self.nearside_door_waypoint_names.index(waypoint_name)
        for k in range(self.n_unique_doors):
            if self.unique_doors[k][door_id] == 1:
                new_k = copy.copy(k)
        transitions = self.new_transitions[waypoint_id][self.unique_door_ids[new_k]][0][new_action_index]
        for trans in transitions:
            if (trans[1] == 1):
                prob = trans[3]
        return prob

    def delete_closed_door(self, waypoint_name):
        state_index = self.waypoint_names.index(waypoint_name)
        door_id = self.nearside_door_waypoint_names.index(waypoint_name)
        action_id = self.new_actions.index('check_door' + str(door_id))
        for k in range(self.n_unique_doors):
            if self.unique_doors[k][door_id] == 0:
                self.new_transitions[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = False
                self.new_rewards[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = 0
        action_id = self.new_actions.index('wait_for_door' + str(door_id))
        for k in range(self.n_unique_doors):
            if self.unique_doors[k][door_id] == 1:
                self.new_transitions[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = False
                self.new_rewards[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = 0
        action_id = self.new_actions.index('set_door' + str(door_id) + '_open')
        for k in range(self.n_unique_doors):
            if self.unique_doors[k][door_id] == 2:
                self.new_transitions[state_index][self.unique_door_ids[k]][1][len(self.new_actions)-1] = False
                self.new_rewards[state_index][self.unique_door_ids[k]][1][len(self.new_actions)-1] = 0
        for i in range(len(self.new_actions)):
            a_copy = copy.copy(self.new_actions[i])
            action = a_copy.split('_')
            if action[0] == 'takedoor' and action[1] == self.waypoint_names[state_index]:
                takedoor_action = a_copy
        action_id = self.new_actions.index(takedoor_action)
        for k in range(self.n_unique_doors):
            if self.unique_doors[k][door_id] == 2:
                self.new_transitions[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = False
                self.new_rewards[state_index][self.unique_door_ids[k]][0][len(self.new_actions)-1] = 0
        action_id = self.new_actions.index('set_door' + str(door_id) + '_closed')
        for k in range(self.n_unique_doors):
            if self.unique_doors[k][door_id] == 1:
                self.new_transitions[state_index][self.unique_door_ids[k]][1][len(self.new_actions)-1] = False
                self.new_rewards[state_index][self.unique_door_ids[k]][1][len(self.new_actions)-1] = 0

class ProductMdp(Mdp):

    def __init__(self, original_mdp,product_sta,product_lab,product_tra):

        self.original_mdp=original_mdp

        self.read_states(product_sta,product_lab)
        self.read_actions(product_tra)
        self.read_transitions(product_tra)
        self.set_rewards_and_trans_count()
        self.set_props()

        self.policy_publisher = rospy.Publisher('/mdp_plan_exec/current_policy_mode', NavRoute)


    def set_rewards_and_trans_count(self):

        self.waypoint_rewards=[[0]*self.n_waypoint_actions for i in range(self.n_waypoints)]
        self.waypoint_transitions_transversal_count=[[0]*self.n_waypoint_actions for i in range(self.n_waypoints)]

        for i in range(0,self.n_waypoints):
            original_state_index=self.state_labels[i][1]
            for j in range(0,self.n_waypoint_actions):
                original_action_index=self.original_mdp.actions.index(self.waypoint_actions[j])
                self.waypoint_rewards[i][j]=self.original_mdp.rewards[original_state_index][original_action_index]
                self.waypoint_transitions_transversal_count[i][j]=self.original_mdp.transitions_transversal_count[original_state_index][original_action_index]

    def set_props(self):
        self.n_waypoint_props=self.original_mdp.n_props
        self.waypoint_props=self.original_mdp.props
        #self.props.append('ltl_goal')
        #self.n_props=self.n_props+1

        self.waypoint_prop_map=[[False]*self.n_waypoint_props for i in range(self.n_waypoints)]

        for i in range(0,self.n_waypoints):
            prop_line=self.original_mdp.waypoint_prop_map[self.state_labels[i][1]]
            prop_line.append(False)
            self.waypoint_prop_map[i]=prop_line


    def read_transitions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()

        self.waypoint_transitions=[[False]*self.n_waypoint_actions for i in range(self.n_waypoints)]

        for line in f:
            line=line.split(' ')
            from_state=int(line[0])
            to_state=int(line[2])
            probability=float(line[3])
            action=self.waypoint_actions.index(line[4].rstrip('\n'))
            if not self.waypoint_transitions[from_state][action]:
                self.waypoint_transitions[from_state][action]= [[to_state,probability]]
            else:
                self.waypoint_transitions[from_state][action].append([to_state,probability])

        f.close()


    def read_actions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()

        self.waypoint_actions=[]
        self.n_waypoint_actions=0

        for line in f:
            current_action=line.split(' ')[-1].rstrip('\n')
            if current_action not in self.waypoint_actions:
                self.waypoint_actions.append(current_action)
                self.n_waypoint_actions=self.n_waypoint_actions+1

        f.close()



    def read_states(self,product_sta,product_lab):
        f = open(product_sta, 'r')
        f.readline()

        self.n_waypoints=0
        self.state_labels=[]


        for line in f:
            current_state_label=line.split(':')[1]
            current_state_label=current_state_label.replace(')', '')
            current_state_label=current_state_label.replace('(', '')
            current_state_label=current_state_label.split(',')
            current_state_label[0]=int(current_state_label[0])
            current_state_label[1]=int(current_state_label[1])

            self.state_labels.append(current_state_label)



            self.n_waypoints=self.n_waypoints+1

        f.close()



        f = open(product_lab, 'r')

        line=f.readline()

        init_index=int(line.split('="init"')[0])

        target_index=line.split('="target"')[0]
        target_index=int(target_index.split(' ')[-1])




        self.goal_states=[]
        for line in f:
            line=line.split(':')
            state_index=int(line[0])
            labels=line[1].split(' ')
            del labels[0]
            n_labels=len(labels)
            for i in range(0,n_labels):
                if int(labels[i])==target_index:
                    self.goal_states.append(state_index)
                if int(labels[i])==init_index:
                    self.initial_waypoint=state_index

        f.close()




    def set_policy(self,policy_file):
        self.policy=[None]*self.n_waypoints
        f=open(policy_file,'r')
        f.readline()
        for line in f:
            line=line.split(' ')
            self.policy[int(line[0])]=line[3].strip('\n')
        #print self.policy
        self.publish_current_policy_mode(self.initial_waypoint)
        f.close()


    def set_initial_state_from_name(self,state_name):
        state_name_prop_index=self.waypoint_props.index(state_name)
        for i in range(0,self.n_waypoints):
            if self.waypoint_prop_map[i][state_name_prop_index] and self.state_labels[i][0]==self.state_labels[self.initial_waypoint][0]:
                self.set_initial_state(i)
                return
        print "set initial state of product MDP error"


    def get_new_state(self,current_state,action,final_node):
        action_index=self.waypoint_actions.index(action)
        possible_next_states=self.waypoint_transitions[current_state][action_index]
        n_possible_next_states=len(possible_next_states)
        final_node_prop_index=self.props.index(final_node)
        for i in range(0,n_possible_next_states):
            next_possible_state=possible_next_states[i][0]
            if self.waypoint_prop_map[next_possible_state][final_node_prop_index]:
                self.publish_current_policy_mode(next_possible_state)
                return next_possible_state
        return -1

    def publish_current_policy_mode(self, current_state):
        sources = []
        targets = []
        current_mode = self.state_labels[current_state][0]
        policy_msg = NavRoute()
        print current_mode
        for i in range(0,self.n_waypoints):
            current_action = self.policy[i]
            if current_action is not None and self.state_labels[i][0] == current_mode:
                action_split = current_action.split('_')
                source = action_split[1]
                target = action_split[2]
                policy_msg.source.append(source)
                policy_msg.target.append(target)

        self.policy_publisher.publish(policy_msg)
