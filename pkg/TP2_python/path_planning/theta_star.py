from utils import *
from dijkstra import get_neighborhood, get_edge_cost

# Implemente la misma heuristica que en A* (distancia euclídea) para que Theta* pueda guiar la búsqueda de manera más eficiente.
def get_heuristic(cell, goal):
    # (Ejercicio 2.3.2)
    heuristic = 0
    return heuristic

def update_costs_and_predecessors(parent, occ_map, costs, predecessors, closed_flags):
    for neighbor in get_neighborhood(parent, occ_map.shape):
        x, y = neighbor
        if closed_flags[x, y] == 1:
            continue


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