#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

import os #for time functions
from search import * #for search engines
import collections
from sokoban import SokobanState, Direction, PROBLEMS #for Sokoban specific classes and problems

def sokoban_goal_state(state):
  '''
  @return: Whether all boxes are paired.
  '''
  for box in state.boxes:
    if box not in state.storage:
      return False
  return True

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #We want an admissible heuristic, which is an optimistic heuristic.
    #It must never overestimate the total_cost to get from the current state to the goal.
    #The sum of the Manhattan distances between each box that has yet to be paired and the storage point nearest to it is such a heuristic.
    #When calculating distances, assume there are no obstacles on the grid.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.

    total_dis = 0
    for box in state.boxes:
      if box not in state.storage:
          dis = []
          for storage in state.storage:
            dis.append(abs(storage[0]-box[0])+abs(storage[1]-box[1]))
          total_dis += min(dis)
    return total_dis


#SOKOBAN HEURISTICS
def trivial_heuristic(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
  count = 0
  for box in state.boxes:
    if box not in state.storage:
        count += 1
  return count


def heur_alternate(state):
#IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    #heur_manhattan_distance has flaws.
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    if sokoban_goal_state(state):
        return 0
    for storage in state.storage:
        if storage not in state.boxes and surrounding_checker(storage, state):
            return float('inf')
    for box in state.boxes:
        if blob_checker(box, state):
            return float('inf')

    global init_dis
    global dis_index
    global dis_index_robot
    total_dis = 0
    paired = []
    paired_robot = []

    if not state.parent:
        init_dis = 0
        dis_index = collections.defaultdict(dict)
        dis_index_robot = collections.defaultdict(dict)
    else:
        if state.parent.boxes == state.boxes and\
                state.parent.robots != state.robots:
            return init_dis

    for box in state.boxes:
        min_dis = float('inf')
        optimal = None
        for storage in state.storage:
            if box in dis_index[storage]:
                dis = dis_index[storage][box]
            else:
                dis = shortest_dis_calculator(box, storage, state)
                dis_index[storage][box] = dis
            if dis < min_dis and (storage not in paired):
                min_dis = dis
                optimal = storage
        if optimal is not None:
            paired.append(optimal)
            total_dis += min_dis
        else:
            return float('inf')

        # min_dis_robot = float('inf')
        # optimal_robot = None
        # for robot in state.robots:
        #   if box in dis_index_robot[robot]:
        #     dis_robot = dis_index_robot[robot][box]
        #   else:
        #       dis_robot = abs(box[0]-robot[0])+abs(box[1]-robot[1])
        #       dis_index_robot[robot][box] = dis_robot
        #   if dis_robot < min_dis_robot and (robot not in paired_robot):
        #     min_dis_robot = dis_robot
        #     optimal_robot = robot
        # if optimal_robot is not None and optimal_robot not in paired_robot:
        #   paired_robot.append(optimal_robot)
        #   total_dis += min_dis_robot
        # else:
        #   return float('inf')

    init_dis = total_dis
    return total_dis


    # total_dis = 0
    # for box in state.boxes:
    #   dis = []
    #   for storage in state.storage:
    #     dis.append(abs(storage[0]-box[0])+abs(storage[1]-box[1]))
    #   total_dis += min(dis)
    #   for stor in state.storage:
    #     if abs(stor[0]-box[0])+abs(stor[1]-box[1]) == min(dis):
    #       total_dis += obstacle_finder(stor, box, state) * 1.2
    #       break
    # for boxx in state.boxes:
    #   dis = []
    #   for robot in state.robots:
    #     dis.append(abs(robot[0]-boxx[0])+abs(robot[1]-boxx[1]))
    #   total_dis += min(dis)
    #   for rob in state.robots:
    #     if abs(rob[0]-boxx[0])+abs(rob[1]-boxx[1]) == min(dis):
    #       total_dis += obstacle_finder(rob, boxx, state) * 1.2
    #       break
    # return total_dis

def shortest_dis_calculator(box, storage, state):
    '''helper function to return the shortest path between a box and storage
    without obstacles'''
    frontier = collections.deque([(box, 0)])
    path = []
    while frontier:
        curr_pos, curr_dis = frontier.pop()
        if curr_pos == storage:
            return curr_dis
        next_pos = []
        if ((curr_pos[0]+1) < state.width) and\
                ((curr_pos[0]+1, curr_pos[1]) not in state.obstacles):
            next_pos.append(((curr_pos[0]+1), curr_pos[1]))
        if ((curr_pos[0]-1) >= 0) and\
                ((curr_pos[0]-1, curr_pos[1]) not in state.obstacles):
            next_pos.append(((curr_pos[0]-1), curr_pos[1]))
        if ((curr_pos[1]+1) < state.height) and\
                ((curr_pos[0], curr_pos[1]+1) not in state.obstacles):
            next_pos.append((curr_pos[0], (curr_pos[1]+1)))
        if ((curr_pos[1]-1) >= 0) and\
                ((curr_pos[0], curr_pos[1]-1) not in state.obstacles):
            next_pos.append((curr_pos[0], (curr_pos[1]-1)))
        for pos in next_pos:
            if (0 <= pos[0] < state.width) and (0 <= pos[1] <= state.height) and\
                    (pos not in path) and (pos not in state.obstacles):
                path.append(pos)
                frontier.appendleft((pos, curr_dis+1))
    return float('inf')

def blob_checker(coordinate, state):
    '''helper function to return true if this position in this state is
    in a four-box-blob '''
    check = (coordinate[0]+1, coordinate[1]) in state.boxes and\
            (coordinate[0], coordinate[1]+1) in state.boxes and\
            (coordinate[0]+1, coordinate[1]+1) in state.boxes and not\
                ((coordinate[0]+1, coordinate[1]) in state.storage and
                 (coordinate[0], coordinate[1]+1) in state.storage and
                 (coordinate[0]+1, coordinate[1]+1) in state.storage and\
                 coordinate in state.storage)
    if coordinate[0] == 0 or coordinate[0] == state.width - 1:
        if (coordinate[0], coordinate[1]+1) in state.boxes and not\
            ((coordinate[0], coordinate[1]+1) in state.storage and
             (coordinate[0], coordinate[1]) in state.storage):
            if not check:
                check = True
    if coordinate[1] == 0 or coordinate[1] == state.height - 1:
        if (coordinate[0]+1, coordinate[1]) in state.boxes and not\
            ((coordinate[0]+1, coordinate[1]) in state.storage and
             (coordinate[0], coordinate[1]) in state.storage):
            if not check:
                check = True
    return check

def surrounding_checker(coordinate, state):
    '''helper function to return true if this position in this state is
    in a unreachable corner'''
    return (((coordinate[0]-1, coordinate[1]) in state.boxes and
             (coordinate[0]-2, coordinate[1]) in state.boxes) or (coordinate[0] == 0)) and\
        (((coordinate[0]+1, coordinate[1]) in state.boxes and
         (coordinate[0]+2, coordinate[1]) in state.boxes) or (coordinate[0]+1 == state.width)) and\
        (((coordinate[0], coordinate[1]-1) in state.boxes and
         (coordinate[0], coordinate[1]-2) in state.boxes) or (coordinate[1] == 0)) and\
        (((coordinate[0], coordinate[1]+1) in state.boxes and
         (coordinate[0], coordinate[1]+2) in state.boxes) or (coordinate[1]+1 == state.height))

def obstacle_finder(coor_one, coor_two, state):
    '''helper function to return the number of obstacles in the area between two positions'''
    count = 0
    if coor_one[0] <= coor_two[0]:
      hor_start = coor_one
      hor_end = coor_two
    else:
      hor_start = coor_two
      hor_end = coor_one
    if coor_one[1] <= coor_two[1]:
      ver_start = coor_one
      ver_end = coor_two
    else:
      ver_start = coor_two
      ver_end = coor_one
    for obs in state.obstacles:
      if hor_start[0] < obs[0] < hor_end[0] and ver_start[1] < obs[1] < ver_end[1]:
        count += 1
    return count

def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform total_cost search'''
    return 0

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.
    Use this function stub to encode the standard form of weighted A* (i.e. g + w*h)

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """

    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.
    return sN.gval + (weight * sN.hval)

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime weighted astar algorithm'''

  engine = SearchEngine('custom', 'full')
  wrapped_fval_fn = (lambda sN : fval_function(sN, weight))
  engine.init_search(initial_state, sokoban_goal_state, heur_fn, wrapped_fval_fn)
  time_start = os.times()[0]
  time_spent = 0
  optimal = False
  total_costbound = float('inf'), float('inf'), float('inf')
  new_weight = weight

  state = engine.search(timebound)[0]
  while time_spent < timebound:
    time_spent = os.times()[0] - time_start
    if not state:
      return optimal
    elif (state.gval + new_weight * heur_fn(state)) <= total_costbound[2]:
      new_weight -= 0.1
      engine.fval_function = (lambda sN : fval_function(sN, new_weight))
      total_costbound = float('inf'), float('inf'), (state.gval + new_weight * heur_fn(state))
      optimal = state
    state = engine.search(timebound - time_spent, total_costbound)[0]
  return optimal

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
  '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
  '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
  '''OUTPUT: A goal state (if a goal is found), else False'''
  '''implementation of anytime greedy best-first search'''

  engine = SearchEngine('best_first', 'full')
  engine.init_search(initial_state, sokoban_goal_state, heur_fn)
  time_start = os.times()[0]
  time_spent = 0
  optimal = False
  total_costbound = float('inf'), float('inf'), float('inf')

  state = engine.search(timebound)[0]
  while time_spent < timebound:
    time_spent = os.times()[0] - time_start
    if not state:
      return optimal
    elif state.gval <= total_costbound[0]:
      total_costbound = state.gval, float('inf'), float('inf')
      optimal = state
    state = engine.search(timebound - time_spent, total_costbound)[0]
  return optimal
