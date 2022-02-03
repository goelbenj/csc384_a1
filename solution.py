#   Look for #IMPLEMENT tags in this file. These tags indicate what has
#   to be implemented to complete the warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

from json.encoder import INFINITY
import os  # for time functions
import time
import math  # for infinity
from search import *  # for search engines
from sokoban import SokobanState, Direction, PROBLEMS  # for Sokoban specific classes and problems

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
    #   # if corner_deadlock(state, box): return math.inf
    #   man_dist = math.inf
    #   man_space = (0,0)
    #   for space in storage_spaces:
    #     # if edge_deadlock(state, box, space): return math.inf
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

    storage_spaces = [space for space in state.storage]
    boxes = [box for box in state.boxes]
    box = boxes[0]

    man_dist = math.inf
    man_space = None
    for storage in state.storage:
      temp_dist = abs(box[0] - storage[0]) + abs(box[1] - storage[1])
      if temp_dist < man_dist:
        man_dist = temp_dist
        man_space = storage

    new_boxes = boxes.copy()
    new_storage = storage_spaces.copy()
    new_state = state
    new_boxes.remove(box)
    new_storage.remove(man_space)
    new_state.boxes = frozenset(new_boxes)
    new_state.storage = frozenset(new_storage)

    return man_dist + heur_alternate(new_state)

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
    while ((time.time() - start_time) < timebound):
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
