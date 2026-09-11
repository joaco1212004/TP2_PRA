from utils import *

# =============================================================================
# Dijkstra
# =============================================================================

def get_neighborhood(cell, occ_map_shape):
    # (Ejercicio 2.2.1)
    # Devuelve una lista de celdas vecinas (8-conectividad) dentro de los límites del mapa
    x, y = cell

    max_x, max_y = occ_map_shape

    neighbors = []

    # Recorremos los 8 posibles vecinos:
    # horizontales, verticales y diagonales.
    for dx in [-1, 0, 1]:
        for dy in [-1, 0, 1]:

            # Evitamos agregar la propia celda.
            if dx == 0 and dy == 0:
                continue

            nx = x + dx
            ny = y + dy

            # Solo agregamos vecinos que estén dentro del mapa.
            if 0 <= nx < max_x and 0 <= ny < max_y:
                neighbors.append((nx, ny))

    return neighbors

def get_edge_cost(parent, child, occ_map):
    # (Ejercicio 2.2.2 y 2.2.3)
    # Si child no es vecino de parent, no existe un arco válido.
    neighbors = get_neighborhood(parent, occ_map.shape)

    if child not in neighbors:
        return np.inf

    # Consideramos obstáculo una celda cuya probabilidad
    # de ocupación sea mayor o igual al 50%.
    occupancy_threshold = 0.5

    occupancy = occ_map[child]

    # No permitimos atravesar obstáculos.
    if occupancy >= occupancy_threshold:
        return np.inf

    # Calculamos la distancia entre parent y child.
    dx = child[0] - parent[0]
    dy = child[1] - parent[1]

    distance = np.sqrt(dx**2 + dy**2)

    # Además de la distancia, agregamos la probabilidad de
    # ocupación para favorecer celdas con menor ocupación.
    edge_cost = distance + occupancy

    return edge_cost

def update_costs_and_predecessors(parent, occ_map, costs, predecessors, closed_flags):
    # (Ejercicio 2.2.4)
    # Deberan actualizar los costos acumulados de los vecinos del nodo expandido 'parent' y sus predecesores
    # closed_flags indica si un nodo ya fue expandido por completo (1) o no (0)
    # parent puede venir como un array de NumPy.
    # Lo convertimos a tupla para indexar correctamente matrices 2D.
    parent = tuple(parent)

    for neighbor in get_neighborhood(parent, occ_map.shape):

        # Si el vecino ya fue expandido por completo, lo ignoramos.
        if closed_flags[neighbor] == 1:
            continue

        # Costo acumulado de llegar al vecino pasando por parent.
        new_cost = (
            costs[parent]
            + get_edge_cost(parent, neighbor, occ_map)
        )

        # Si encontramos un camino más barato, actualizamos
        # el costo y guardamos el nuevo predecesor.
        if new_cost < costs[neighbor]:
            costs[neighbor] = new_cost
            predecessors[neighbor] = parent


def run_dijkstra(occ_map, start, goal):
    run_planning(occ_map, start, goal, no_heuristic, update_costs_and_predecessors)


