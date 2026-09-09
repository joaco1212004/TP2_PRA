from utils import *

# =============================================================================
# Dijkstra
# =============================================================================

def get_neighborhood(cell, occ_map_shape):
    # (Ejercicio 2.2.1)
    # Devuelve una lista de celdas vecinas (8-conectividad) dentro de los límites del mapa
    neighbors = []
    return neighbors

def get_edge_cost(parent, child, occ_map):
    # (Ejercicio 2.2.2 y 2.2.3)
    edge_cost = np.inf
    neighbors = get_neighborhood(parent, occ_map.shape)
    return edge_cost

def update_costs_and_predecessors(parent, occ_map, costs, predecessors, closed_flags):
    # (Ejercicio 2.2.4)
    # Deberan actualizar los costos acumulados de los vecinos del nodo expandido 'parent' y sus predecesores
    # closed_flags indica si un nodo ya fue expandido por completo (1) o no (0)
    for neighbor in get_neighborhood(parent, occ_map.shape):
        pass


def run_dijkstra(occ_map, start, goal):
    run_planning(occ_map, start, goal, no_heuristic, update_costs_and_predecessors)


