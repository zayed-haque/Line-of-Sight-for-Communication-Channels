#!/usr/bin/python
#-*-coding:utf-8-*-

class UCTControl(OperateInterface):

    class _TreePolicyController(OperateInterface):
        '''This is the tree policy controller which conform to test_tools.OperateInterface'''
        def __init__(self, search_tree, decision_plane, reward_list, 
                     CP, knowledge_of_partners, reward_gather = 2, 
                     reward_find = 0.5, reward_endangerous = -0.1,
                     reward_lost_plane = -1, reward_intruder_life_plus = -0.1):
            self.reward_gather = reward_gather
            self.reward_find = reward_find
            self.reward_endangerous = reward_endangerous
            self.reward_lost_plane = reward_lost_plane
            self.reward_intruder_life_plus = reward_intruder_life_plus
            self.search_tree = search_tree
            self.decision_plane = decision_plane
            self.reward_list = reward_list
            self.CP = CP
            self.knowledge_of_partners = knowledge_of_partners
            return super(UCTControl._TreePolicyController, self).__init__()

        def initiate(self, planes, bases, region_size, max_plane_battery, intruder_exposed_time, plane_sight, max_semo_intruder, target_move):
            self.bases = bases
            self.region_size = region_size
            self.max_plane_battery = max_plane_battery
            self.intruder_exposed_time = intruder_exposed_time
            self.plane_sight = plane_sight
            self.max_semo_intruder = max_semo_intruder
            self.target_move = target_move
            self.cmd_list = []
            self.reward_list = []
            self.reach_leaf = False

        def decide(self, plane, planes, bases, found_intruders_position):
            '''this decision is made according to UCT algorithm.'''
            #judge which plane to control
            if plane == self.decision_plane:
                if not self.reach_leaf:    #if not reach the node
                    #do the UCT part
                    node = self.search_tree.get_node(self.cmd_list)
                    if node.has_untried_moves():
                        #if this node has untried moves, which means leaf node
                        CMD = random.choice(node.untried_moves)
                        self.cmd_list.append(CMD)
                        assert CMD in plane.moveable_direction()
                        self.reach_leaf = True
                        return CMD
                    else:
                        #if it already pass the leaf node, do the default policy
                        CMD = sorted(node.children.values(), key = self._choose_priority_calculate_rule)[-1].action
                        #refresh the cmd_list
                        self.cmd_list.append(CMD)
                        assert CMD in plane.moveable_direction()
                        return CMD
                else:
                    #if it reach the leaf node
                    CMD = random.choice(plane.moveable_direction())
                    assert CMD in plane.moveable_direction()
                    return CMD
            else:
                #simulate other planes based on learning probability
                moveable_direction = plane.moveable_direction()
                if is_endangerous(bases, plane):
                    #if this plane is endangerous
                    situation = 'endangerous'
                else:
                    #if this plane is in normal situation
                    situation = 'normal'
                if found_intruders_position:
                    #if find intruder
                    #get the command of the heighest probability
                    CMD = sorted(moveable_direction, key = lambda c: self.knowledge_of_partners[plane.num][situation]['found'][realetive_position(plane.position, found_intruders_position[0])][c])[-1]
                else:   #don't find intruder
                    #get the command of the heighest probability
                    CMD = sorted(moveable_direction, key = lambda c: self.knowledge_of_partners[plane.num][situation]['notfound'][c])[-1]
                assert CMD in plane.moveable_direction()
                return CMD

        def get_cmd_list(self):
            '''this function return the cmd list of UCT search to help back propagation.'''
            return self.cmd_list

        def get_reward_list(self):
            '''this function return reward list.'''
            return self.reward_list

        def get_info(self, step_info):
            '''refresh the reward list'''
            reward = self.reward_gather * step_info['intruder_exposed'] +\
                self.reward_endangerous * step_info['endangerous'][self.decision_plane.num] +\
                self.reward_lost_plane * step_info['plane_lost'][self.decision_plane.num] +\
                self.reward_find * step_info['find_intruder'] +\
                self.reward_intruder_life_plus * step_info['intruder_add_life_time']
            self.reward_list.append(reward)

        def _choose_priority_calculate_rule(self, node):
            assert type(node) is UCTTreeNode
            Q_value = node.data["average_benefit"] + 2 * self.CP * math.sqrt((2 * math.log(node.parent.data["visit_time"]) / node.data["visit_time"]))
            dif = abs(abs(node.data["average_benefit"]) - abs(2 * self.CP * math.sqrt((2 * math.log(node.parent.data["visit_time"]) / node.data["visit_time"]))))
            dif_por = 2 * dif / (abs(node.data["average_benefit"]) + abs(2 * self.CP * math.sqrt((2 * math.log(node.parent.data["visit_time"]) / node.data["visit_time"]))))
            return Q_value





    def __init__(self, CP=0.5, max_trajectory = 200, max_depth = 8,
                 reward_gather = 2, reward_find = 0.5, reward_endangerous = -0.1,
                 reward_lost_plane = -1, reward_intruder_life_plus = -0.1, 
                 show_info = False, gama = 0.9):
        self.CP = CP   #CP is used to adjust the priority of explore
        self.max_trajectory = max_trajectory
        self.max_depth = max_depth
        self.reward_gather = reward_gather
        self.reward_find = reward_find
        self.reward_endangerous = reward_endangerous
        self.reward_lost_plane = reward_lost_plane
        self.show_info = show_info
        self.reward_intruder_life_plus = reward_intruder_life_plus
        self.gama = gama
        return super(UCTControl, self).__init__()

    def initiate(self,planes,bases,region_size,max_plane_battery,intruder_exposed_time, plane_sight, max_semo_intruder, target_move):
        self.bases = deepcopy(bases)
        self.region_size = region_size
        self.max_plane_battery = max_plane_battery
        self.intruder_exposed_time = intruder_exposed_time
        self.plane_sight = plane_sight
        self.max_semo_intruder = max_semo_intruder
        self.target_move = target_move
        self.knowledge_of_partners = {}
        _base_dict1 = {'L':1, 'R':1, 'U':1, 'D':1, 'S':1}
        _base_dict2 = {'LU':deepcopy(_base_dict1), 'LD':deepcopy(_base_dict1), 'RU':deepcopy(_base_dict1), 'RD':deepcopy(_base_dict1)}
        _base_dict3 = {'found':deepcopy(_base_dict2), 'notfound':deepcopy(_base_dict1)}
        for one_plane in planes:
            self.knowledge_of_partners[one_plane.num] = {'normal':deepcopy(_base_dict3), 'endangerous':deepcopy(_base_dict3)}

    def decide(self,plane, planes, bases, found_intruders_position):
        '''This method find the best move based on UCT algorithem.
        It would not change each parameter.'''
        #initialization: construct a searching tree
        next_moveable_direction = self._get_moveable_direction([], plane)
        search_tree = UCTSearchTree({"visit_time":0, "average_benefit":0}, next_moveable_direction)
        #if there is no found intruders, simulate one randomly
        if len(found_intruders_position) <= 0:  
            random_simulate_position =  (random.randint(0,self.region_size[0]-1), random.randint(0,self.region_size[1]-1))  
            found_intruders_position.append(random_simulate_position)   #cirtified. It is random.

        #generate tree policy controller
        reward_list = []
        tree_policy_controller = self._TreePolicyController(search_tree, plane, reward_list, self.CP, 
                                                            self.knowledge_of_partners, reward_gather = self.reward_gather, 
                                                            reward_find = self.reward_find, reward_endangerous = self.reward_endangerous, 
                                                            reward_lost_plane = self.reward_lost_plane, reward_intruder_life_plus = self.reward_intruder_life_plus)
        #start UCT algorithem
        for time in range(self.max_trajectory):
            #simulate based on UCT principle and get the final result
            test_platform = PlatForm(self.region_size, bases, deepcopy(planes), 1, tree_policy_controller,
                                                            self.max_semo_intruder, self.target_move, self.max_plane_battery,
                                                            self.intruder_exposed_time, self.plane_sight, show_info = False,
                                                            max_simulate_time = self.max_depth)
            test_platform.test(found_intruders_position)
            result = test_platform.get_result()
            #expand new node in UCT Search tree
            cmd_list = tree_policy_controller.get_cmd_list()
            if len(cmd_list) == 0:      #if no cmd list, start next round
                continue
            #first get moveable directions
            next_moveable_direction = self._get_moveable_direction(cmd_list, plane)   #this function return the moveable list
            #if it meets the leaf node, the add_children func would not add
            search_tree.add_children(cmd_list[:], {"visit_time":0, "average_benefit":0}, next_moveable_direction)
            assert search_tree.has_node(cmd_list)
            #back-propogate
            #generate real reward list
            raw_reward_list = tree_policy_controller.get_reward_list()
            reward_list = raw_reward_list[:len(cmd_list)]
            last_raw_reward = 0
            for raw_reward in raw_reward_list[len(cmd_list):]:
                last_raw_reward += raw_reward
            reward_list.append(last_raw_reward)
            assert len(reward_list) >= 2
            #back-refresh
            assert search_tree.has_node(cmd_list)
            search_tree.refresh_tree(cmd_list[:], self._get_refresh_rule(reward_list))
            if self.show_info:
                print "The benefit is: ", sorted(search_tree.root.children.values(), key = self._choose_priority_calculate_rule)[-1].data["average_benefit"]       
        #choose the best move command
        CMD = sorted(search_tree.root.children.values(), key = lambda c: c.data["average_benefit"])[-1].action
        assert CMD in plane.moveable_direction()
        #refresh knowledge of parteners 
        if is_endangerous(bases, plane):
            #if this plane is endangerous
            situation = 'endangerous'
        else:
            #if this plane is in normal situation
            situation = 'normal'
        if found_intruders_position:
            #if find intruder
            #refresh specific statistic
            self.knowledge_of_partners[plane.num][situation]['found'][realetive_position(plane.position, found_intruders_position[0])][CMD] += 1
        else:   #don't find intruder
            #get the command of the heighest probability
            #refresh specific statistic
            self.knowledge_of_partners[plane.num][situation]['notfound'][CMD] += 1
        return CMD



    def _get_moveable_direction(self, cmd_list, plane):
        '''This function return the moveable direction based on the given cmd_list.
            It would not change the plane.'''
        simulating_plane = Components.MoveComponent(plane.position, 0, self.region_size)
        for cmd in cmd_list:
            assert cmd in simulating_plane.moveable_direction()
            simulating_plane.move(cmd)
        return simulating_plane.moveable_direction()

    def _refresh_rule(self, used_data):
        #accumulate reward to their parent nodes
        if len(self.__reward_list) >= 2:
            self.__reward_list[-2] += self.__reward_list[-1] * self.gama
        new_data = {}
        new_data["visit_time"] = used_data["visit_time"] + 1
        new_data["average_benefit"] = (used_data["average_benefit"] *
                                       used_data["visit_time"] +
                                       self.__reward_list.pop()) / new_data["visit_time"]
        return new_data

    def _get_refresh_rule(self, reward_list):
        '''This function set self.__reward and return _refresh_rule function handler.'''
        self.__reward_list = reward_list
        return self._refresh_rule
