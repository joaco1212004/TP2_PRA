from utils import *
from dijkstra import update_costs_and_predecessors

# =============================================================================
# A*
# =============================================================================

def get_heuristic(cell, goal):
    # (Ejercicio 2.3.2)
    heuristic = 0
    return heuristic

def run_astar(occ_map, start, goal):
    run_planning(occ_map, start, goal, get_heuristic, update_costs_and_predecessors)