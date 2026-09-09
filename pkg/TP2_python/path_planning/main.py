from utils import *
from campos_potenciales import run_potential_fields
from dijkstra import run_dijkstra
from a_star import run_astar
from theta_star import run_theta_star

def main():
    # Cargar mapa de ocupación falso para pruebas locales
    # En producción usar: occ_map = np.loadtxt('map.txt')
    occ_map = np.loadtxt('map.txt')
    # double the size of the map for better visualization
    occ_map = np.kron(occ_map, np.ones((2, 2)))
    
    start = np.array([44,66])
    goal = np.array([80, 30])

    # Selector de algoritmos para que el alumno evalúe sus implementaciones
    seleccion = int(input("Seleccione Algoritmo: 1:APF | 2:Dijkstra | 3:A* | 4:Theta* : "))
    
    if seleccion == 1:
        run_potential_fields(occ_map, start, goal)
    elif seleccion == 2:
        run_dijkstra(occ_map, start, goal)
    elif seleccion == 3:
        run_astar(occ_map, start, goal)
    elif seleccion == 4:
        run_theta_star(occ_map, start, goal)

if __name__ == "__main__":
    main()