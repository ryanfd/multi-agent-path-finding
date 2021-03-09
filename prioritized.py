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
            for j in range(len(path)):
                for k in range(self.num_of_agents):
                    if k == 1:
                        # vertex constraints
                        constraints.append({'agent': k, 'loc': [path[j]], 'time_step': j+1})

                        # edge constraints
                        if prev_pos != None:
                            if constraints[0]['loc'][0] != (1, 1):
                                constraints.append({'agent': k, 'loc':[path[j], prev_pos], 'time_step': j})
                            else:
                                constraints.append({'agent': k, 'loc':[path[j], prev_pos], 'time_step': j+1})

                prev_pos = path[j]





            # for i in range(len(path)):
            #     print(path[i], "-", i+1)
            #     for j in range(self.num_of_agents):
            #         if j == 0:
            #             # vertex constraints
            #             constraints.append({'agent': j+1, 'loc':[path[i]], 'time_step': i+1}) # define constraints for agent 1, agent 0 has priority

            #             constraints.append({'agent': j+1, 'loc':[path[i], prev_pos], 'time_step': i+1}) # edge constraints

            #     prev_pos = path[i]




            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
