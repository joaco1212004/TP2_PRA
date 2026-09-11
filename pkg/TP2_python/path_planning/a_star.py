from utils import *
from dijkstra import update_costs_and_predecessors

# =============================================================================
# A*
# =============================================================================

def get_heuristic(cell, goal):
    # (Ejercicio 2.3.2)

    # Diferencia entre la celda actual y el objetivo.
    dx = goal[0] - cell[0]
    dy = goal[1] - cell[1]

    factor = 1

    # Usamos la distancia euclídea como estimación del costo restante hasta el objetivo.
    heuristic = factor * np.sqrt(dx**2 + dy**2)

    return heuristic

def run_astar(occ_map, start, goal):
    run_planning(occ_map, start, goal, get_heuristic, update_costs_and_predecessors)