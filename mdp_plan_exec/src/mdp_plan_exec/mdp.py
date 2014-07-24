#!/usr/bin/python


import sys
import rospy


from ros_datacentre.message_store import MessageStoreProxy
from strands_navigation_msgs.msg import TopologicalNode
from strands_navigation_msgs.msg import NavStatistics
from strands_navigation_msgs.msg import NavRoute


class Mdp(object):
    
    def __init__(self):
        #list of attributes of an MDP object
        self.top_map=''
        self.initial_state=0
        self.n_states=0
        self.state_names=[]
        self.n_props=0
        self.n_door_props = []
        self.n_waypoint_props = 0
        self.n_wait_props = []
        self.props=[]
        self.door_props = [[]]
        self.waypoint_props = []
        self.wait_props = [[]]
        self.n_waypoint_actions=0
        self.n_actions = 0
        self.waypoint_actions=[] #a name
        self.prop_map=[[]]
        self.waypoint_prop_map = [[]]
        self.wait_prop_map = [[[]]]
        self.door_prop_map = [[[]]]
        self.waypoint_transitions=[[]]
        self.waypoint_transitions_transversal_count=[[]]
        self.waypoint_rewards=[[]]
        self.current_policy=[]
        self.wait_state_names=[[]]
        self.door_state_names=[[]]

        self.initial_waypoint = 0
        self.n_waypoints = 0
        self.waypoint_names = []

        self.door_state = []
        self.door_ids=[]
        self.initial_door_state = 0
        self.n_door_states = 3

        self.wait_state = []
        self.initial_wait_state = 0
        self.n_wait_states = 2

        self.n_doors = 0
        self.states = [[[]]]
        self.n_edges = 0
        self.check_actions = []
        self.wait_actions = []
        self.waypoints_actions_doors = []
        self.door_actions = []
        self.door_open_probs = []
        self.normal_door_open_prob = 0.5
        self.door_wait_open_prob = []
        self.door_target_waypoints = []
        self.waypoints = []

        self.new_transitions = [[[[]]]]
        self.new_rewards = [[[[]]]]
        self.new_actions = []
        self.n_new_actions =0

    def write_prism_model(self,file_name):
        f=open(file_name,'w')
        #states
        f.write('mdp\n \n')
        f.write('module M \n \n')
        f.write('w:[0..'+str(self.n_waypoints-1)+'] init ' + str(self.initial_waypoint) + ';\n')
        f.write('d:[0..'+str(self.n_door_states-1)+'] init ' + str(self.initial_door_state) + ';\n')
        f.write('t:[0..'+str(self.n_wait_states-1)+'] init ' + str(self.initial_wait_state) + ';\n \n')

        for i in range(0,self.n_waypoints):
            for j in range(0,self.n_waypoint_actions):
                current_trans_list=self.waypoint_transitions[i][j]
                if current_trans_list:
                    if j not in self.door_actions:
                        trans_string='[' + self.waypoint_actions[j] + '] w=' + str(i) + ' -> '
                        for trans in current_trans_list:
                            trans_string=trans_string + str(trans[1]) + ":(w'=" + str(trans[0]) + ') + '
                        f.write(trans_string[:-3] + ';\n')

        # for i in range(0,self.n_waypoints):
        #     for j in range(0,self.n_door_states):
        #         for k in range(0 , self.n_wait_states):
        #             for l in range(0 , self.n_new_actions):
        #                 current_trans_list = self.new_transitions[i][j][k][l]
        #                 if current_trans_list:
        #                     trans_string = '[' + self.new_actions[l] + '] w=' + str(i) + ' & d=' + str(j) + ' &t=' + str(k) + ' -> '
        #                     for trans in current_trans_list:
        #                         trans_string = trans_string + str(trans[3]) + ":(w'=" + str(trans[0]) + " & d'=" + str(trans[1]) + " & t'=" + str(trans[2]) + ') + '
        #                     f.write(trans_string[:-3] + ':\n')



        for i in self.waypoints_actions_doors:
            waypoint = self.waypoints_actions_doors[i][0]
            door_prob = self.door_open_probs[i]
            door_wait_prob = self.door_wait_open_prob[i]
            f.write('[check_door' + waypoint + '] w=' + waypoint + ' & d=0 -> ' + (1-door_prob) +
                    ":(w'=" + waypoint + " & d'=1) + " + door_prob + ":(w'=" + waypoint + " & d'=1; \n" )
            f.write('[wait_for_door' + waypoint + '] w=' + waypoint + ' & t=0 & d=0 -> ' + (1-door_wait_prob) +
                    ":(w'=" + waypoint + " & t'=1 & d'=1) + " + door_wait_prob  + ":(w'=" + waypoint + " & t'=1 & d'=2); \n")
            f.write('[set_door' + waypoint + '_open] w=' + waypoint + ' & t=1 & d=2 -> '
                    + " 1.0:(w'=" + waypoint + " & d'=2 + t'=0; \n")
            f.write('[enter_door' + waypoint + '] w=' + waypoint + ' & t=0 & d=2 -> '
                    + " 1.0:(w'=" + str(self.door_target_waypoints[i]) + " & d'=0 + t'=0; \n")
            f.write('[set_door' + waypoint + '_closed] w=' + waypoint + ' & t=1 & d=1 -> '
                    + " 1.0:(w'=" + waypoint + " & d'=0 + t'=0; \n")
        
        f.write('\nendmodule\n\n')
        
        for i in range(0,self.n_waypoint_propsprops):
            f.write('label "'+ self.waypoint_props[i] + '" = ')
            prop_string=''
            for j in range(0,self.n_waypoints):
                if self.waypoint_prop_map[j][i]:
                   prop_string=prop_string + 'w=' + str(j) + ' | '
            f.write(prop_string[:-3] + ';\n')  
        f.write('label "door_open" = d=2;\n')
        f.write('label "door_closed" =d=1;\n')
        f.write('label "door_unknown" =d=0;\n')
        f.write('label "door_waited" = t=1;\n')
        f.write('label "door_not_waited" =t=0;\n \n')
        
  
        #f.write('label "goal" = ')
        
        #goal_states_string=''
        #for goal_state in self.goal_states:
            #goal_states_string=goal_states_string + 's=' + str(goal_state) + ' | '
            
        #f.write(goal_states_string[:-3] + ';\n\n')
        
        f.write('rewards "time"\n')

        #outputs
        for i in range(0,self.n_waypoints):
            for j in range(0,self.n_waypoint_actions):
                if self.waypoint_rewards[i][j] != 0:
                    f.write('    [' + self.waypoint_actions[j] + '] w=' + str(i) + ':' + str(self.waypoint_rewards[i][j]) + ';\n')


        # for i in range(0,self.n_waypoints):
        #     for j in range(0,self.n_door_states):
        #         for k in range(0 , self.n_wait_states):
        #             for l in range(0 , self.n_new_actions):
        #                 if self.new_rewards[i][j][k][l] != 0:
        #                     f.write('    [' + self.new_actions[l] + '] w=' + str(i) + ' & d=' + str(j) + ' & t=' +
        #                             str(k) + ':' + str(self.new_rewards[i][j][k][l]) + ';\n')



        for i in self.waypoints_actions_doors:
            waypoint = str(self.waypoints_actions_doors[i][0])
            #could modify so each door has its own set of timings. perhaps could even change by hour.
            f.write('[check_door' + waypoint + '] w=' + waypoint + ":3 \n" )
            f.write('[wait_for_door' + waypoint + '] w=' + waypoint +  ":3 \n")
            f.write('[set_door' + waypoint + '_open] w=' + waypoint + ":120 \n")
            f.write('[enter_door' + waypoint + '] w=' + waypoint +  ":20 \n")
            f.write('[set_door' + waypoint + '_closed] w=' + waypoint +  ":300 \n")
        
        f.write('endrewards\n')
        
        f.close()
        
    def set_initial_state(self,initial_state):
        self.initial_state=initial_state
        
    def get_expected_edge_transversal_time(self,state_index,action_name):
        action_index=self.waypoint_actions.index(action_name)
        return self.rewards[state_index][action_index]
        
    def get_total_transversals(self,state_index,action_name):
        action_index=self.waypoint_actions.index(action_name)
        return self.waypoint_transitions_transversal_count[state_index][action_index]


class TopMapMdp(Mdp):
    def __init__(self,top_map_name):
        
        self.top_map=top_map_name
        
        self.initial_state=0

        top_nodes=self.read_top_map()


        self.n_states=len(top_nodes)
        self.n_waypoints=len(top_nodes)
        self.state_names=[None]*self.n_states
        self.waypoint_names=[None]*self.n_waypoints
        self.door_state_names=[None]*self.n_door_states
        self.wait_state_names=[None]*self.n_wait_states

        self.n_props=(self.n_states)
        self.n_waypoint_props =  self.n_waypoints
        self.props=[None]*self.n_props
        self.waypoint_props = [None]*self.n_waypoint_props
        self.prop_map=[[False]*self.n_props for i in range(self.n_states)]
        self.waypoint_prop_map=[[False]*self.n__waypoint_props for i in range(self.n_waypoint_states)]

        self.n_wait_props = self.n_wait_states
        self.wait_props = [None]*self.n_wait_props
        self.wait_prop_map = [[False]*self.n_wait_props for i in range(self.n_wait_states)]
        self.wait_state_names=[None]*self.n_wait_states
        self.n_door_props = self.n_door_states
        self.door_props = [None]*self.n_door_props
        self.door_prop_map = [[False]*self.n_door_props for i in range(self.n_door_states)]
        #self.wait_props[0] = 'door_not_waited'
        #self.wait_props[1] = 'door_waited'
        #self.door_props[0] = 'door_unknown'
        #self.door_props[1] = 'door_closed'
        #self.door_props[2] = 'door_open'
        #self.wait_state_names[1] = 'waited_for_door'
        #self.wait_state_names[0] = 'not_waited_for_door'
        #self.door_state_names[0] = 'door_unknown'
        #self.door_state_names[1] = 'door_closed'
        #self.door_state_names[2] = 'door_open'
        for i in range(0,self.n_props):
            self.prop_map[i][i]=True
        for i in range(0,self.n_waypoint_props):
            self.waypoint_prop_map[i][i]=True
        for i in range(0,self.n_wait_props):
            self.wait_prop_map[i][i]=True
        for i in range(0,self.n_door_props):
            self.door_prop_map[i][i]=True

        i=0
        self.n_actions=0
        self.n_waypoint_actions=0
        for entry in top_nodes:
            self.props[i]=entry[0].name
            self.waypoint_props[i]=entry[0].name
            self.state_names[i]=entry[0].name
            self.waypoint_names[i]=entry[0].name
            self.n_waypoint_actions=self.n_waypoint_actions+len(entry[0].edges)
            self.n_actions=self.n_actions+len(entry[0].edges)
            i=i+1
        i=0
        self.waypoint_rewards=[[0]*self.n_waypoint_actions for i in range(self.n__waypoints)]
        self.rewards=[[0]*self.n_actions for i in range(self.n_states)]
        self.waypoint_transitions=[[False]*self.n_waypoint_actions for i in range(self.n_waypoints)]
        self.transitions=[[False]*self.n_actions for i in range(self.n_states)]
        self.waypoint_transitions_transversal_count=[[0]*self.n_waypoint_actions for i in range(self.n_waypoints)]
        self.transitions_transversal_count=[[0]*self.n_actions for i in range(self.n_states)]
        self.waypoint_actions=[None]*self.n_waypoint_actions
        self.actions=[None]*self.n_actions

        self.new_actions = [None]*self.n_actions

        action_index=0
        state_index=0
        doors=0
        for entry in top_nodes:
            current_edges=entry[0].edges
            for edge in current_edges:
                target_index=self.state_names.index(edge.node)
                self.new_actions[action_index] = 'goto_'+self.state_names[state_index] + '_' + edge.node
                action_index=action_index+1
                if edge.action == 'doorPassing':
                    doors += 1
            state_index=state_index+1

        self.new_transitions = [[[[False]*(self.n__actions + (5*doors)) for i in range(self.n_wait_states)] for j in range(self.n_door_states)] for k in range(self.n_waypoints)]
        self.new_rewards = [[[[0]*(self.n__actions + (5*doors)) for i in range(self.n_wait_states)] for j in range(self.n_door_states)] for k in range(self.n_waypoints)]

	print 'something'
        state_index=0
        action_index=0
        door_id=0
        current_state=[[None for k in xrange(2)] for i in xrange(self.n_state_types)]
        for entry in top_nodes:
            current_edges=entry[0].edges
            for edge in current_edges:
                target_index=self.state_names.index(edge.node)
                if edge.action == 'move_base':
                    self.new_transitions[state_index][0][0][action_index] = [[target_index , 1]]
                    self.new_rewards[state_index][0][0][action_index] = 1
                elif edge.action == 'doorPassing':
                    self.door_open_probs[door_id] = self.normal_door_open_prob
                    self.new_actions.append('check_door' + str(door_id))
                    self.new_transitions[state_index][0][0][len(self.new_actions)-1] = [[state_index,1,0,(1-self.door_open_probs[door_id])],[state_index,2,0,self.door_open_probs[door_id]]]
                    self.new_rewards[state_index][0][0][len(self.new_actions)-1] = 3
                    self.new_actions.append('wait_for_door' + str(door_id))
                    self.new_transitions[state_index][1][0][len(self.new_actions)-1] = [[state_index,1,1,(1-self.door_wait_open_probs[door_id])],[state_index,2,1,self.door__wait_open_probs[door_id]]]
                    self.new_rewards[state_index][1][0][len(self.new_actions)-1] = 3
                    self.new_actions.append('set_door' + str(door_id) + '_open')
                    self.new_transitions[state_index][2][1][len(self.new_actions)-1] = [[state_index,2,0,1]]
                    self.new_rewards[state_index][2][1][len(self.new_actions)-1] = 120
                    self.new_actions.append('enter_door' + str(door_id))
                    self.new_transitions[state_index][2][0][len(self.new_actions)-1] = [[target_index,0,0,1]]
                    self.new_rewards[state_index][2][0][len(self.new_actions)-1] = 20
                    self.new_actions.append('set_door' + str(door_id) + '_closed')
                    self.new_transitions[state_index][2][1][len(self.new_actions)-1] = [[state_index,0,0,1]]
                    self.new_rewards[state_index][2][1][len(self.new_actions)-1] = 300
                    self.door_target_waypoints[door_id] = target_index
                    self.door_actions[door_id] = action_index
                    self.door_wait_open_probs[door_id] = self.normal_door_open_prob
                    self.waypoints_actions_doors[door_id] = [state_index, action_index]
                    self.door_id += 1
                self.waypoint_actions[action_index]='goto_'+self.state_names[state_index] + '_' + edge.node
                self.waypoint_transitions[state_index][action_index]= [[target_index,1]]
                self.waypoint_rewards[state_index][action_index]=1
                self.actions[action_index]='goto_'+self.state_names[state_index] + '_' + edge.node
                self.transitions[state_index][action_index]= [[target_index,1]]
                self.rewards[state_index][action_index]=1
                action_index=action_index+1


            state_index=state_index+1


        self.n_new_actions = len(self.new_actions)
    #given waypoint, door state, wait state, action, give the transition probability
    #check 331 and 352
    #use topological mdp

    #0) use a proper transition function mapping state to actions
    #1) get prism going
    #2) send both to Bruno
    #3) get info back

    def read_top_map(self):


        msg_store = MessageStoreProxy(collection='topological_maps')
    
        query_meta = {}
        query_meta["point_set"] = self.top_map
        available = len(msg_store.query(TopologicalNode._type, {}, query_meta)) > 0


        if available <= 0 :
            rospy.logerr("Desired pointset "+ "something" +" not in datacentre")
            rospy.logerr("Available pointsets: "+str(available))
            raise Exception("Can't find waypoints.")
    
        else :
            query_meta = {}
            query_meta["pointset"] = self.top_map
            message_list = msg_store.query(TopologicalNode._type, {}, query_meta)
    
        return message_list


        
    def update_nav_statistics(self):
        msg_store = MessageStoreProxy()
    
        query_meta = {}
        query_meta["pointset"] = self.top_map
        print self.top_map
        message_list = msg_store.query(NavStatistics._type, {}, query_meta)
        n_data=len(message_list)
        n_unprocessed_data=n_data

        
        
        for i in range(0,self.n_actions):
            current_action=self.actions[i]
            if 'goto' in current_action:
                action_index=self.actions.index(current_action)
                current_action=current_action.split('_')
                source_index=self.state_names.index(current_action[1])
                target_index=self.state_names.index(current_action[2])
                j=0
                n_total_data=1
                expected_time=0
                total_outcomes_count=1
                outcomes_count=[0]*self.n_states
                outcomes_count[target_index]=1
                while j<n_unprocessed_data:
                    entry=message_list[j]
                    if current_action[1]==entry[0].origin and current_action[2]==entry[0].target and not entry[0].final_node == 'Unknown':
                        n_total_data=n_total_data+1
                        expected_time=expected_time+float(entry[0].operation_time)-float(entry[0].time_to_waypoint)
                        outcomes_count[self.state_names.index(entry[0].final_node)]+=1
                        total_outcomes_count=total_outcomes_count+1
                        del message_list[j]
                        n_unprocessed_data=n_unprocessed_data-1
                    else:
                        j=j+1
                if n_total_data==1:
                    rospy.logwarn("No data for edge between waypoints " + current_action[1] + " and " + current_action[2] + ". Assuming it to be 20 seconds. Expected time between nodes will not be correct.")
                    self.rewards[source_index][action_index]=20
                else:
                    self.rewards[source_index][action_index]=expected_time/(total_outcomes_count-1)
                    self.transitions_transversal_count[source_index][action_index]=total_outcomes_count-1
                    transition=None
                    for j in range(0,self.n_states):
                        count=outcomes_count[j]
                        if count > 0:
                            probability=float(count)/float(total_outcomes_count)
                            if transition is None:
                                transition=[[j, probability]]
                            else:
                                transition.append([j,probability])
                    if transition is not None:
                        self.transitions[source_index][action_index]=transition
        
        
    def set_initial_state_from_name(self,state_name):
        index=self.state_names.index(state_name)
        self.set_initial_state(index)



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
        
        self.rewards=[[0]*self.n_actions for i in range(self.n_states)]
        self.transitions_transversal_count=[[0]*self.n_actions for i in range(self.n_states)]
        
        for i in range(0,self.n_states):
            original_state_index=self.state_labels[i][1]
            for j in range(0,self.n_actions):
                original_action_index=self.original_mdp.actions.index(self.actions[j])
                self.rewards[i][j]=self.original_mdp.rewards[original_state_index][original_action_index]
                self.transitions_transversal_count[i][j]=self.original_mdp.transitions_transversal_count[original_state_index][original_action_index]
        
    def set_props(self):
        self.n_props=self.original_mdp.n_props
        self.props=self.original_mdp.props
        #self.props.append('ltl_goal')
        #self.n_props=self.n_props+1
        
        self.prop_map=[[False]*self.n_props for i in range(self.n_states)]
        
        for i in range(0,self.n_states):
            prop_line=self.original_mdp.prop_map[self.state_labels[i][1]]
            prop_line.append(False)
            self.prop_map[i]=prop_line
        

       
        
        
    
    def read_transitions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()
        
        self.transitions=[[False]*self.n_actions for i in range(self.n_states)]
        
        for line in f:
            line=line.split(' ')
            from_state=int(line[0])
            to_state=int(line[2])
            probability=float(line[3])
            action=self.actions.index(line[4].rstrip('\n'))
            if not self.transitions[from_state][action]:
                self.transitions[from_state][action]= [[to_state,probability]]
            else:
                self.transitions[from_state][action].append([to_state,probability])

        f.close()        
            
    
    def read_actions(self,product_tra):
        f = open(product_tra, 'r')
        f.readline()
        
        self.actions=[]
        self.n_actions=0
        
        for line in f:
            current_action=line.split(' ')[-1].rstrip('\n')
            if current_action not in self.actions:
                self.actions.append(current_action)
                self.n_actions=self.n_actions+1
                
        f.close()                
    
        
    
    def read_states(self,product_sta,product_lab):
        f = open(product_sta, 'r')
        f.readline()
        
        self.n_states=0
        self.state_labels=[]
        
        
        for line in f:
            current_state_label=line.split(':')[1]
            current_state_label=current_state_label.replace(')', '')
            current_state_label=current_state_label.replace('(', '')
            current_state_label=current_state_label.split(',')
            current_state_label[0]=int(current_state_label[0])
            current_state_label[1]=int(current_state_label[1])
            
            self.state_labels.append(current_state_label)
            
            
            
            self.n_states=self.n_states+1
            
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
                    self.initial_state=state_index
        
        f.close()
        
    


    def set_policy(self,policy_file):
        self.policy=[None]*self.n_states
        f=open(policy_file,'r')
        f.readline()
        for line in f:
            line=line.split(' ')
            self.policy[int(line[0])]=line[3].strip('\n')
        print self.policy
        self.publish_current_policy_mode(self.initial_state)
        f.close()    
                    
    
    def set_initial_state_from_name(self,state_name):
        state_name_prop_index=self.props.index(state_name)
        for i in range(0,self.n_states):
            if self.props_map[i][state_name_prop_index] and self.state_labels[i][0]==self.state_labels[self.initial_state][0]:
                self.set_initial_state(i)
                return
        print "set initial state of product MDP error"
    
    
    def get_new_state(self,current_state,action,final_node):
        action_index=self.actions.index(action)
        possible_next_states=self.transitions[current_state][action_index]
        n_possible_next_states=len(possible_next_states)
        final_node_prop_index=self.props.index(final_node)
        for i in range(0,n_possible_next_states):
            next_possible_state=possible_next_states[i][0]
            if self.prop_map[next_possible_state][final_node_prop_index]:
                self.publish_current_policy_mode(next_possible_state)
                return next_possible_state
        return -1
        
    def publish_current_policy_mode(self, current_state):
        sources = []
        targets = []
        current_mode = self.state_labels[current_state][0]
        policy_msg = NavRoute()
        print current_mode
        for i in range(0,self.n_states):
            current_action = self.policy[i]
            if current_action is not None and self.state_labels[i][0] == current_mode:
                action_split = current_action.split('_')
                source = action_split[1]
                target = action_split[2]
                policy_msg.source.append(source)
                policy_msg.target.append(target)
                
        self.policy_publisher.publish(policy_msg)
                
        
        
        
        
        
        
        
    

