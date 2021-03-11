import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            if path is None:
                raise BaseException('No solutions')
            result.append(path)

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches
            # constraints.append({'agent': 0, 'loc': [(1,5)], 'time_step': 4})
            # constraints.append({'agent': 0, 'loc':[(1,5)], 'time_step': 10})
            # constraints.append({'agent': 1, 'loc': [(1,2), (1,3)], 'time_step': 1})

            # constraints.append({'agent': 1, 'loc':[(1, 2)], 'time_step': 2})
            # constraints.append({'agent': 1, 'loc':[(1, 4)], 'time_step': 2})
            
            prev_pos = None
            priority_goal = None
            priority_goal_timestep = 0
            for j in range(len(path)):
                for k in range(self.num_of_agents):
                    if k == 0:
                        if j == len(path)-1: # track pos and goal when priority reaches goal
                            priority_goal = path[j]
                            priority_goal_timestep = j+1
                            constraints.append({'agent': k+1, 'loc': [priority_goal], 'time_step': j+2})
                    if k == 1:
                        if j == len(path)-1:
                            secondary_goal_timestep = j+1

                        # vertex constraints
                        constraints.append({'agent': k, 'loc': [path[j]], 'time_step': j+1})
                        if priority_goal != None and priority_goal_timestep == 3: # secondary agent can't entire goal loc of priority and can't backtrack
                            constraints.append({'agent': k, 'loc': [path[j-2]], 'time_step': priority_goal_timestep+1})
                            constraints.append({'agent': k, 'loc': [path[j-1]], 'time_step': priority_goal_timestep+2})
                            
                            # constraints on future loc after priority goal has been reached
                            index = j
                            while index < len(path):
                                constraints.append({'agent': k, 'loc': [priority_goal], 'time_step': priority_goal_timestep+3+index})
                                index += 1

                        # edge constraints
                        if prev_pos != None:
                            if constraints[0]['loc'][0] != (1, 1):
                                constraints.append({'agent': k, 'loc':[path[j], prev_pos], 'time_step': j})
                            else:
                                constraints.append({'agent': k, 'loc':[path[j], prev_pos], 'time_step': j+1})

                prev_pos = path[j]

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
