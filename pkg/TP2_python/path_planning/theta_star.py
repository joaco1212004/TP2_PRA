from utils import *
from dijkstra import get_neighborhood, get_edge_cost

# Implemente la misma heuristica que en A* (distancia euclídea) para que Theta* pueda guiar la búsqueda de manera más eficiente.
def get_heuristic(cell, goal):
    # (Ejercicio 2.3.2)
    
    dx = goal[0] - cell[0]
    dy = goal[1] - cell[1]

    heuristic = np.sqrt(dx**2 + dy**2)

    return heuristic

def update_costs_and_predecessors(parent, occ_map, costs, predecessors, closed_flags):
    
    # parent llega como un array de NumPy.
    parent = tuple(parent)

    for neighbor in get_neighborhood(parent, occ_map.shape):

        x, y = neighbor

        if closed_flags[x, y] == 1:
            continue

        # -------------------------------------------------------------
        # Caso normal: parent -> neighbor
        # -------------------------------------------------------------

        normal_cost = (
            costs[parent]
            + get_edge_cost(parent, neighbor, occ_map)
        )

        best_cost = normal_cost
        best_predecessor = parent

        # -------------------------------------------------------------
        # Caso Theta*: predecessor(parent) -> neighbor
        # -------------------------------------------------------------

        grandparent = predecessors[parent]

        # Verificamos que parent tenga un predecesor válido.
        if grandparent[0] != -1:

            grandparent = tuple(grandparent)

            # Si existe línea de visión directa entre el abuelo
            # y el vecino, evaluamos ese camino alternativo.
            if bresenham_line_of_sight(
                grandparent,
                neighbor,
                occ_map
            ):

                # Como grandparent y neighbor pueden no ser vecinos,
                # calculamos directamente la distancia euclídea.
                dx = neighbor[0] - grandparent[0]
                dy = neighbor[1] - grandparent[1]

                direct_distance = np.sqrt(dx**2 + dy**2)

                # Se mantiene también la penalización por ocupación
                # utilizada en el costo de Dijkstra/A*.
                direct_cost = (
                    costs[grandparent]
                    + direct_distance
                    + occ_map[neighbor]
                )

                # Si el salto angular es más barato, lo elegimos.
                if direct_cost < best_cost:
                    best_cost = direct_cost
                    best_predecessor = grandparent

        # Actualizamos solamente si mejoramos el costo conocido.
        if best_cost < costs[neighbor]:
            costs[neighbor] = best_cost
            predecessors[neighbor] = best_predecessor


        # (Ejercicio 2.4.1)
        # -------------------------------------------------------------
        # Caso normal:
        #
        # Calcular el costo de llegar desde 'parent' hasta 'neighbor'.
        # Actualizar el costo acumulado y el predecesor de 'neighbor'
        # si este camino resulta más económico que el conocido hasta
        # el momento.
        #
        # Este es el comportamiento utilizado por Dijkstra y A*.
        # -------------------------------------------------------------


        # -------------------------------------------------------------
        # Caso Theta*:
        #
        # Theta* intenta evitar la restricción de que el camino tenga
        # que pasar necesariamente por nodos vecinos.
        #
        # Obtener el predecesor de 'parent'. Si existe, verificar si
        # existe una línea de visión directa entre ese nodo y 'neighbor'
        # utilizando:
        #
        #     bresenham_line_of_sight(p1, p2, occ_map)
        #
        # Si existe línea de visión, comparar el costo de:
        #
        #     predecessor(parent) -> neighbor
        #
        # contra el costo del camino normal:
        #
        #     parent -> neighbor
        #
        # Si la conexión directa resulta más económica, actualizar el
        # costo de 'neighbor' y establecer predecessor(parent) como
        # su nuevo predecesor.
        #
        # Para calcular el costo de la conexión directa, recuerden que
        # predecessor(parent) y neighbor no necesariamente son vecinos
        # de la grilla.
        # -------------------------------------------------------------


def run_theta_star(occ_map, start, goal):
    run_planning(occ_map, start, goal, get_heuristic, update_costs_and_predecessors)