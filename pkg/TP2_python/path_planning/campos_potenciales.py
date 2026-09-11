from utils import *

# =============================================================================
# CAMPOS POTENCIALES ARTIFICIALES (APF)
# =============================================================================

def get_attractive_force(cell, goal, k_att=1.0):
    # (Ejercicio 2.1.1)
    # Implementar una fuerza lineal o cuadratica hacia la meta dependiendo de la distancia
    # La meta es goal [y, x] y la celda actual es cell [y, x]
    # La fuerza deben calcularla como un vector que apunta desde la celda actual hacia la meta,
    # normalizado y escalado por la magnitud de la fuerza.

    # Vector que apunta desde la celda actual hacia la meta.
    direction = np.array(goal, dtype=float) - np.array(cell, dtype=float)

    # Distancia entre la celda actual y la meta.
    distance = np.linalg.norm(direction)

    # Si ya estamos en la meta, no hay fuerza atractiva.
    if distance == 0:
        return np.zeros(2)

    # Modelo lineal: la magnitud de la fuerza aumenta con la distancia a la meta.
    magnitude = k_att * distance

    # Normalizamos la dirección y la escalamos por la magnitud.
    f_att = (direction / distance) * magnitude

    return f_att

def get_repulsive_force(cell, occ_map, k_rep=100.0, d_0=5.0):
    # (Ejercicio 2.1.2)
    # Encontrar obstáculos cercanos (puntos con alta probabilidad de ocupación)
    # occ_map es una matriz donde cada celda tiene un valor entre 0 (libre) y 1 (ocupado).
    # tendrán que decidir a partir de que valor se considera un obstáculo (ejemplo: >= 0.4)
    # para luego calcular la fuerza inversamente proporcional a la distancia si dist < d_0
    # La fuerza deben calcularla como un vector que apunta desde el obstáculo hacia la celda
    # actual, normalizado y escalado por la magnitud de la fuerza.

    # Consideramos obstáculos las celdas cuya probabilidad de ocupación sea mayor o igual a 0.4.
    obstacles = np.argwhere(occ_map >= 0.4)

    # Si no hay obstáculos, no existe fuerza repulsiva.
    if len(obstacles) == 0:
        return np.zeros(2)

    cell = np.array(cell, dtype=float)

    # Calculamos el vector desde cada obstáculo hacia la posición actual.
    vectors = cell - obstacles

    # Calculamos la distancia desde la celda actual a cada obstáculo.
    distances = np.linalg.norm(vectors, axis=1)

    # Buscamos el obstáculo más cercano.
    closest_index = np.argmin(distances)
    distance = distances[closest_index]

    # La fuerza repulsiva solamente actúa dentro de la distancia de influencia d_0.
    if distance >= d_0:
        return np.zeros(2)

    # Evitamos una división por cero si la posición coincide exactamente con un obstáculo.
    if distance == 0:
        distance = 1e-6

    # Dirección desde el obstáculo más cercano hacia la posición actual.
    direction = vectors[closest_index] / distance

    # Magnitud de la fuerza repulsiva.
    magnitude = (
        k_rep
        * (1 / distance - 1 / d_0)
        / distance**2
    )

    f_rep = direction * magnitude

    return f_rep


# =============================================================================
# =============================================================================


def run_potential_fields(occ_map, start, goal, max_steps=500, step_size=0.5):

    viz = PathPlannerVisualizer(occ_map, start, goal)

    current = np.array(start, dtype=float)
    path = [np.copy(current)]

    for _ in range(max_steps):

        # User closed the window
        if not viz.is_open():
            print("Visualización cerrada. Terminando.")
            break

        if np.linalg.norm(current - goal) < 1.0:
            print("Campos Potenciales: ¡Meta alcanzada!")
            break

        f_att = get_attractive_force(current, goal)
        f_rep = get_repulsive_force(current, occ_map)
        f_total = f_att + f_rep

        if np.linalg.norm(f_total) > 0.01:
            current += (
                f_total / np.linalg.norm(f_total)
            ) * step_size

        path.append(np.copy(current))

        if not viz.plot_current(current):
            print("Visualización cerrada. Terminando.")
            break

        if (len(path) >= 10 and np.all(np.abs(path[-10:] - path[-1]) < 1.0)):
            print("Campos Potenciales: Estancado, terminando.")
            break

    viz.wait()