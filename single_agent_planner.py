import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)] 
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    table = []
    for i in constraints:
        if len(i) == 3: # i['positive'] does not exist
            i['positive'] = False
        table.append(i)
    return table


def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.


    for i in range(len(constraint_table)):
        if next_time == constraint_table[i]['time_step']:
            # vertex constraints
            if len(constraint_table[i]['loc']) == 1:
                if next_loc == constraint_table[i]['loc'][0]:
                    # positive constraints
                    if constraint_table[i]['positive'] == True:
                        return False
                    else:
                        return True
            # edge constraints
            if len(constraint_table[i]['loc']) > 1:
                if curr_loc == constraint_table[i]['loc'][0] and next_loc == constraint_table[i]['loc'][1]:
                    # positive constraints
                    if constraint_table[i]['positive'] == True:
                        return False
                    else:
                        return True
        # at goal, wrong time
        if next_time <= constraint_table[i]['time_step'] and curr_loc == next_loc :
            return True

    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map          - binary obstacle map
        start_loc       - start position
        goal_loc        - goal position
        agent           - the agent that is being re-planned
        constraints     - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 1
    curr_time_step = earliest_goal_timestep
    h_value = h_values[start_loc]
    constraint_table = build_constraint_table(constraints, agent) # build constraint table for root is generated
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time_step': curr_time_step}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time_step'])] = root

    # for i in constraint_table:
    #     print("[A_STAR] CONSTRAINTS:", i)

    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and not is_constrained(curr['loc'], goal_loc, curr_time_step+1, constraint_table):
            print("GOAL REACHED:", agent, "-", curr['loc'], "-", curr_time_step)
            return get_path(curr)

        for dir in range(4):
            child_loc = move(curr['loc'], dir)

            try:
                if my_map[child_loc[0]][child_loc[1]]: # getting index error on loc (8, 6)
                    continue
            except IndexError:
                continue

            # if child_loc == (8, 6):
            #     continue
            # if my_map[child_loc[0]][child_loc[1]]:
            #     continue
            if child_loc[1] < 0 or child_loc[0] < 0: # getting errors with loc (1, -1)
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time_step' : curr_time_step+1}

            # if constrained, increment time and re-search the same node
            if not is_constrained(curr['loc'], child['loc'], curr_time_step+1, constraint_table): 
                if (child['loc'], child['time_step']) in closed_list:
                    existing_node = closed_list[(child['loc'])]
                    if compare_nodes(child, existing_node):
                        closed_list[(child['loc'], child['time_step'])] = child
                        push_node(open_list, child)
                else:
                    closed_list[(child['loc'], child['time_step'])] = child
                    # print("\nchild:", child)
                    push_node(open_list, child)

        curr_time_step += 1 # increment time
    # end of while loop
                

    return None  # Failed to find solutions
