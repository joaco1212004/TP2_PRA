import numpy as np
import matplotlib.pyplot as plt
import random


class PathPlannerVisualizer:

    def __init__(self, occ_map, start, goal):
        self.occ_map = occ_map
        self.start = np.array(start)
        self.goal = np.array(goal)

        self.fig, self.ax = plt.subplots()

        self.ax.imshow(
            occ_map.T,
            cmap=plt.cm.gray,
            interpolation='none',
            origin='upper'
        )

        self.ax.plot(
            start[0], start[1],
            'ro',
            label='Inicio'
        )

        self.ax.plot(
            goal[0], goal[1],
            'go',
            label='Meta'
        )

        self.ax.set_xlim(0, occ_map.shape[0] - 1)
        self.ax.set_ylim(0, occ_map.shape[1] - 1)

        self.ax.set_xlabel('x')
        self.ax.set_ylabel('y')

        self.closed = False

        # Called automatically when the user closes the window
        self.fig.canvas.mpl_connect(
            'close_event',
            self._on_close
        )

        plt.ion()
        plt.show(block=False)

    def _on_close(self, event):
        self.closed = True

    def is_open(self):
        """Returns False if the user closed the visualization."""
        return not self.closed and plt.fignum_exists(self.fig.number)

    def update(self):
        """Update the GUI and return False if the window was closed."""
        if not self.is_open():
            return False

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)

        return self.is_open()

    def plot_expanded(self, node, update=True):
        if not self.is_open():
            return False

        node = np.asarray(node)

        if np.array_equal(node, self.start):
            return True

        if np.array_equal(node, self.goal):
            return True

        self.ax.plot(
            node[0],
            node[1],
            'yo',
            markersize=3
        )

        if update:
            return self.update()

        return True

    def plot_path(self, node, predecessor=None):
        """Plot a path node, optionally connected to its predecessor."""
        if not self.is_open():
            return False

        node = np.asarray(node)

        if np.array_equal(node, self.goal):
            return True

        if predecessor is not None:
            predecessor = np.asarray(predecessor)

            self.ax.plot(
                [predecessor[0], node[0]],
                [predecessor[1], node[1]],
                'b-',
                linewidth=2,
                zorder=5
            )
        else:
            self.ax.plot(
                node[0],
                node[1],
                'bo',
                markersize=4,
                zorder=5
            )

        return self.update()

    def plot_current(self, node):
        """Plot the current position."""
        if not self.is_open():
            return False

        node = np.asarray(node)

        self.ax.plot(
            node[0],
            node[1],
            'bo',
            markersize=2
        )

        return self.update()

    def wait(self):
        """Keep the final plot open until the user closes it."""
        if self.is_open():
            plt.ioff()
            plt.show()

    def close(self):
        """Close the visualization."""
        if self.is_open():
            plt.close(self.fig)

def run_planning(occ_map, start, goal, get_heuritic, update_costs):
    '''
    Calcula el camino de costo mínimo utilizando el algoritmo de Dijkstra.
    La heurística se fija explícitamente en 0.
    '''
    print("Ejecutando Algoritmo de Dijkstra...")
    viz = PathPlannerVisualizer(occ_map, start, goal)

    costs = np.ones(occ_map.shape) * np.inf
    closed_flags = np.zeros(occ_map.shape)
    predecessors = -np.ones(occ_map.shape + (2,), dtype=np.int32)

    # Precalcular matriz de heurística para optimizar el bucle
    heuristic = np.zeros(occ_map.shape)
    for x in range(occ_map.shape[0]):
        for y in range(occ_map.shape[1]):
            heuristic[x, y] = get_heuritic([x, y], goal)

    costs[start[0], start[1]] = 0
    parent = start

    skipper = 0

    while not np.array_equal(parent, goal):
        if not viz.is_open():
            print("Visualización cerrada. Terminando Dijkstra.")
            return
        
        # Dijkstra solo evalúa el costo acumulado 'costs' sin sumar heurística
        open_costs = np.where(closed_flags == 1, np.inf, costs) + heuristic

        x, y = np.unravel_index(open_costs.argmin(), open_costs.shape)
        
        if open_costs[x, y] == np.inf:
            break  # No hay más nodos alcanzables
        
        parent = np.array([x, y])

        # Marcamos el nodo como expandido
        closed_flags[x, y] = 1
        
        update_costs(parent, occ_map, costs, predecessors, closed_flags)
        
        skipper += 1
        if skipper % 15 == 0:
            if not viz.plot_expanded(parent, update=True):
                print("Visualización cerrada. Terminando Dijkstra.")
                return
        else:
            viz.plot_expanded(parent, update=False)
    
    # Reconstrucción del camino
    if np.array_equal(parent, goal):

        current = np.array(goal)

        # Variable temporal para medir la longitud geométrica del camino
        path_length = 0.0

        while not np.array_equal(current, start):

            previous = predecessors[current[0], current[1]]

            # No debería ocurrir si encontramos un camino válido,
            # pero evita errores si el predecesor no existe.
            if previous[0] == -1:
                print("Error reconstruyendo el camino.")
                break

            # Sumamos la distancia real entre dos puntos consecutivos del camino
            path_length += np.linalg.norm(current - previous)

            if not viz.plot_path(current, previous):
                print("Visualización cerrada.")
                return

            current = previous

        print(f"Camino encontrado. Costo: {costs[goal[0], goal[1]]:.2f}")
        print(f"Longitud real del camino: {path_length:.2f}")

    else:
        print("No se encontró un camino válido.")

    # Mantener la ventana abierta al terminar
    viz.wait()

def no_heuristic(cell, goal):
    # La heurística de Dijkstra es siempre 0 ya que no se utiliza información adicional para guiar la búsqueda.
    return 0

def bresenham_line_of_sight(p1, p2, occ_map):
    '''
    Determina si hay línea de visión directa entre p1 y p2 usando el algoritmo de Bresenham.
    Arguments:
    p1, p2 -- Coordenadas de celda como [y, x] o [x, y] de manera consistente.
    occ_map -- Matriz del mapa de ocupación.
    
    Output:
    True si el camino está libre de obstáculos, False de lo contrario.
    '''
    y1, x1 = int(p1[0]), int(p1[1])
    y2, x2 = int(p2[0]), int(p2[1])
    
    dy = abs(y2 - y1)
    dx = abs(x2 - x1)
    
    sy = 1 if y1 < y2 else -1
    sx = 1 if x1 < x2 else -1
    
    err = dx - dy
    
    while True:
        # Verificar límites del mapa
        if y1 < 0 or y1 >= occ_map.shape[0] or x1 < 0 or x1 >= occ_map.shape[1]:
            return False
        # Verificar si la celda actual es un obstáculo (umbral de ocupación >= 0.4)
        if occ_map[y1, x1] >= 0.4:
            return False
            
        if y1 == y2 and x1 == x2:
            break
            
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x1 += sx
        if e2 < dx:
            err += dx
            y1 += sy
            
    return True