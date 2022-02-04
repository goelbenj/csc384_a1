#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

from audioop import mul
from json.encoder import INFINITY
import os  # for time functions
import time
import math  # for infinity
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

global multiplier

def sokoban_goal_state(state):
    '''
    @return: Whether all boxes are stored.
    '''
    for box in state.boxes:
        if box not in state.storage:
            return False
    return True

def heur_manhattan_distance(state):
    # IMPLEMENT
    '''admissible sokoban puzzle heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # We want an admissible heuristic, which is an optimistic heuristic.
    # It must never overestimate the cost to get from the current state to the goal.
    # The sum of the Manhattan distances between each box that has yet to be stored and the storage point nearest to it is such a heuristic.
    # When calculating distances, assume there are no obstacles on the grid.
    # You should implement this heuristic function exactly, even if it is tempting to improve it.
    # Your function should return a numeric value; this is the estimate of the distance to the goal.
    sum = 0
    for box in state.boxes:
      man_dist = math.inf
      for space in state.storage:
        temp_dist = abs(box[0] - space[0]) + abs(box[1] - space[1])
        man_dist = temp_dist if temp_dist < man_dist else man_dist
      sum += man_dist
    return sum  # CHANGE THIS

# SOKOBAN HEURISTICS
def trivial_heuristic(state):
    '''trivial admissible sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state (# of moves required to get) to the goal.'''
    return 0  # CHANGE THIS

def corner_deadlock(state, box):
  ''' Checks if box is corner_deadlock, BUT NOT AT GOAL STATE!!, using both the dimensions of map and the obstacles '''

  if box in state.storage: return False

  up_block = (box[1] == 0) or ((box[0], box[1] + 1) in state.obstacles) 
  down_block = (box[1] == state.height - 1) or ((box[0], box[1] - 1) in state.obstacles) 

  left_block = (box[0] == 0) or ((box[0] - 1, box[1]) in state.obstacles)
  right_block = (box[0] == state.width - 1) or ((box[0] + 1, box[1]) in state.obstacles)

  return (up_block or down_block) and (left_block or right_block)

def edge_deadlock(state, box):
  ''' Checks if there is a deadlock due to map walls on a box and a possible storage point '''
  if box in state.storage: return False
  # Check if box is either at the leftmost or rightmost wall, and check if storage is not along that wall
  if ((box[0] == 0) or (box[0] == state.width - 1)) and (box[0] not in [space[0] for space in state.storage]):
      return True
  # Check if box is either at the topmost or bottommost wall, and check if storage is not along that wall
  elif ((box[1] == state.height - 1) or (box[1] == 0)) and (box[1] not in [space[1] for space in state.storage]):
      return True
  return False

def adjacency_deadlock(boxes, box):
  if (box[0] - 1, box[1]) in boxes[1:]:
    print(box)
  if (box[0] + 1, box[1]) in boxes[1:]:
    print(box)
  if (box[0], box[1] - 1) in boxes[1:]:
    print(box)
  if (box[0], box[1] + 1) in boxes[1:]:
    print(box)
  return 1

def heur_alternate(state):
    # IMPLEMENT
    '''a better heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''
    # heur_manhattan_distance has flaws.
    # Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    # Your function should return a numeric value for the estimate of the distance to the goal.

    # Heuristic description: exact same as manhattan distance from above, but now, only one box can occupy a single storage location.
    # storage_spaces = [space for space in state.storage]
    # sum = 0
    # for box in state.boxes:
    #   box_copy = [tempbox for tempbox in state.boxes]
    #   if adjacency_deadlock(box_copy.pop(box_copy.index(box)), box): 
    #     sum += 1
    #   if corner_deadlock(state, box): 
    #     return math.inf
    #   if edge_deadlock(state, box): 
    #     return math.inf
    #   man_dist = math.inf
    #   man_space = (0,0)
    #   for space in storage_spaces:
    #     temp_dist = abs(box[0] - space[0]) + abs(box[1] - space[1])
    #     if temp_dist < man_dist:
    #       man_dist = temp_dist
    #       man_space = space
    #   index = storage_spaces.index(man_space)
    #   storage_spaces.pop(index)
    #   sum += man_dist
    # return sum  # CHANGE THIS

    # base recursive case
    if not len(state.boxes): return 0

    storages = list(state.storage)
    boxes = list(state.boxes)
    box = boxes[0]

    if box in state.storage:
      new_storage = storages.copy()
      new_storage.remove(box)
      new_state = SokobanState(state.action, state.gval, state.parent, state.width,
        state.height, state.robots, frozenset(boxes[1:]), frozenset(new_storage), state.obstacles)
      return heur_alternate(new_state)
    
    walls = 0
    if(box[0] + 1, box[1]) in state.obstacles:
      walls += 2
    if(box[0] - 1, box[1]) in state.obstacles:
      walls += 2
    if(box[0], box[1] + 1) in state.obstacles:
      walls += 2
    if(box[0], box[1] - 1) in state.obstacles:
      walls += 2
    if(box[0] == 0 or box[0] == state.width - 1):
      walls += 2
    if(box[1] == 0 or box[1] == state.height - 1):
      walls += 2
    if walls >= 2:
      return math.inf
    walls *= 2

    adjacency = 0
    if (box[0] + 1, box[1]) in boxes[1:]:
      adjacency += 1
    if (box[0] - 1, box[1]) in boxes[1:]:
      adjacency += 1
    if (box[0], box[1] + 1) in boxes[1:]:
      adjacency += 1
    if (box[0], box[1] - 1) in boxes[1:]:
      adjacency += 1
    
    min = math.inf
    storage_space = None
    for storage in state.storage:
      dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
      if dist < min:
        min = dist
        storage_space = storage

    new_storage = storages.copy()
    new_storage.remove(storage_space)
    new_state = SokobanState(state.action, state.gval, state.parent, state.width, state.height,
      state.robots, frozenset(boxes[1:]), frozenset(new_storage), state.obstacles)

    return min + walls + adjacency + heur_alternate(new_state)


def heur_zero(state):
    '''Zero Heuristic can be used to make A* search perform uniform cost search'''
    return 0

def fval_function(sN, weight):
    # IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    return sN.gval + weight * sN.hval

# SEARCH ALGORITHMS
def weighted_astar(initial_state, heur_fn, weight, timebound):
    '''Provides an implementation of weighted a-star, as described in the HW1 handout'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of weighted astar algorithm'''

    se = SearchEngine('custom', 'full')
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn, fval_function=wrapped_fval_function)
    return se.search(timebound)

def iterative_astar(initial_state, heur_fn, weight=1, timebound=5):  # uses f(n), see how autograder initializes a search line 88
    # IMPLEMENT
    '''Provides an implementation of realtime a-star, as described in the HW1 handout'''
    '''INPUT: a warehouse state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False as well as a SearchStats object'''
    '''implementation of realtime astar algorithm'''

    # weight will halve by // 2 at ever iteration
    start_time = time.time()
    curr_best = math.inf
    best_final, best_stats = None, None
    while (weight > 0):
      se = SearchEngine('custom', 'full')
      wrapped_fval_function = (lambda sN: fval_function(sN, weight))
      se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn, fval_function=wrapped_fval_function)
      final, stats = se.search(timebound - (time.time() - start_time), (math.inf, math.inf, curr_best))
      if final:
        best_final = final
        best_stats = stats
        curr_best = final.gval if final.gval < curr_best else curr_best
      weight //= 2
    return best_final, best_stats #CHANGE THIS

def iterative_gbfs(initial_state, heur_fn, timebound=5):  # only use h(n)
    # IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False'''
    '''implementation of weighted astar algorithm'''

    start_time = time.time()
    curr_best = math.inf
    best_final, best_stats = None, None
    while ((time.time() - start_time) < timebound - 0.1):
      se = SearchEngine('best_first', 'full')
      se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn=heur_fn)
      final, stats = se.search(timebound - (time.time() - start_time), (curr_best, math.inf, math.inf))
      if final:
        best_final = final
        best_stats = stats
        curr_best = final.gval if final.gval < curr_best else curr_best
      if (stats.states_expanded == (stats.states_generated - stats.states_pruned_cycles - stats.states_pruned_cost)):
        return best_final, best_stats
    return best_final, best_stats #CHANGE THIS
