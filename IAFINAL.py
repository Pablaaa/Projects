import time
import heapq
import tkinter as tk

class VacuumWorld:
    def __init__(self, dirt_levels, robot_location):
        # Inicializa el mundo de la aspiradora con los niveles de suciedad y la ubicación del robot.
        self.dirt_levels = dirt_levels  # Niveles de suciedad en cada habitación
        self.robot_location = robot_location  # Ubicación del robot
        self.move_count = 0  # Contador de movimientos realizados por el robot

    def is_goal_state(self):
        # Comprueba si el estado actual del mundo es un estado objetivo, es decir, si todas las habitaciones están limpias.
        return all(dirt == '0' for dirt in self.dirt_levels.values())  # Verifica si todas las habitaciones están limpias

    def get_actions(self):
        # Obtiene las acciones disponibles para el robot en el estado actual del mundo.
        actions = []  # Lista para almacenar las acciones disponibles
        if self.robot_location > 0:
            actions.append("Up")  # Si el robot puede subir, añadir 'Up' a las acciones disponibles
        if self.robot_location < len(self.dirt_levels) - 1:
            actions.append("Down")  # Si el robot puede bajar, añadir 'Down' a las acciones disponibles
        if self.dirt_levels[chr(65 + self.robot_location)] == '1':
            actions.append("Suck")  # Si la habitación actual está sucia, añadir 'Suck' a las acciones disponibles
        return actions

    def take_action(self, action):
        # Realiza una acción en el mundo y actualiza su estado.
        new_dirt_levels = self.dirt_levels.copy()  # Copia los niveles de suciedad actuales
        new_robot_location = self.robot_location  # Ubicación del robot después de la acción

        # Actualiza la ubicación del robot y el contador de movimientos según la acción realizada
        if action == "Up":
            new_robot_location -= 1
            self.move_count += 1
        elif action == "Down":
            new_robot_location += 1
            self.move_count += 1
        elif action == "Suck":
            new_dirt_levels[chr(65 + new_robot_location)] = '0'  # Marca la habitación actual como limpia
            self.move_count += 1

        return VacuumWorld(new_dirt_levels, new_robot_location)  # Devuelve un nuevo estado del mundo

    def __hash__(self):
        # Calcula el hash del estado del mundo.
        return hash((frozenset(self.dirt_levels.items()), self.robot_location))

    def __eq__(self, other):
        # Compara dos estados del mundo para ver si son iguales.
        return self.dirt_levels == other.dirt_levels and self.robot_location == other.robot_location


class VacuumWorldGUI:
    def __init__(self, master, dirt_levels, robot_location):
        # Inicializa la interfaz gráfica del mundo de la aspiradora.
        self.master = master  # Ventana maestra de la interfaz gráfica
        self.dirt_levels = dirt_levels  # Niveles de suciedad en cada habitación
        self.robot_location = robot_location  # Ubicación inicial del robot
        self.tiles = []  # Lista para almacenar los widgets de etiquetas (tiles)

        # Crea e inicializa los widgets de etiquetas (tiles) para representar las habitaciones
        self.create_tiles()
        # Actualiza los widgets de etiquetas (tiles) para reflejar el estado inicial del mundo
        self.update_tiles()

    def create_tiles(self):
        # Crea los widgets de etiquetas (tiles) para representar las habitaciones del mundo.
        for i in range(8):
            row = []  # Lista para almacenar los widgets de etiquetas en una fila
            for j in range(8):
                tile = tk.Label(self.master, width=2, height=1, relief="raised", borderwidth=1)  # Crea una etiqueta
                tile.grid(row=i, column=j)  # Posiciona la etiqueta en la cuadrícula
                row.append(tile)  # Agrega la etiqueta a la fila
            self.tiles.append(row)  # Agrega la fila de etiquetas a la lista de tiles

    def update_tiles(self):
        # Actualiza los widgets de etiquetas (tiles) para reflejar el estado actual del mundo.
        for i, row in enumerate(self.tiles):
            for j, tile in enumerate(row):
                location = chr(65 + i)  # Convierte el índice de la fila en una letra
                if j == self.robot_location and self.dirt_levels[location] == '1':
                    tile.config(bg='red', text='V')  # Marca la posición del robot en rojo con una 'V'
                elif self.dirt_levels[location] == '1':
                    tile.config(bg='brown')  # Marca las habitaciones sucias en marrón
                elif j == self.robot_location:
                    tile.config(bg='green', text='V')  # Marca la posición del robot en verde con una 'V'
                else:
                    tile.config(bg='white')  # Habitaciones limpias en blanco

        # Resalta la ubicación del robot
        row = self.robot_location  # Fila de la ubicación del robot
        col = next(key for key, value in self.dirt_levels.items() if value == '1')  # Encuentra cualquier habitación sucia
        self.tiles[row][ord(col) - 65].config(bg='blue', text='R')  # Destaca la ubicación del robot en azul con una 'R'


def greedy_best_first_search(vacuum_world):
    # Realiza la búsqueda de la mejor opción mediante el algoritmo Greedy Best-First Search.
    start_time = time.time()  # Marca el tiempo de inicio de la búsqueda
    initial_state = vacuum_world  # Estado inicial del mundo de la aspiradora

    # Verifica si el estado inicial es el estado objetivo
    if initial_state.is_goal_state():
        # Si el estado inicial es el estado objetivo, devuelve una solución vacía y el tiempo de ejecución
        return [], time.time() - start_time, initial_state.move_count

    # Inicializa el heap con una tupla que contiene la heurística, una lista de acciones y el estado inicial
    heap = [(heuristic(initial_state), [], initial_state)]
    explored = set()  # Conjunto para almacenar los estados explorados

    # Realiza la búsqueda hasta que el heap esté vacío
    while heap:
        _, actions, current_state = heapq.heappop(heap)  # Extrae el elemento con la menor heurística

        # Verifica si el estado actual es el estado objetivo
        if current_state.is_goal_state():
            # Si el estado actual es el estado objetivo, devuelve las acciones, el tiempo de ejecución y el conteo de movimientos
            return actions, time.time() - start_time, current_state.move_count

        # Verifica si el estado actual ya ha sido explorado
        if current_state in explored:
            continue  # Si ya ha sido explorado, pasa al siguiente estado

        explored.add(current_state)  # Agrega el estado actual al conjunto de estados explorados

        # Genera los sucesores del estado actual y los agrega al heap
        for action in current_state.get_actions():
            new_state = current_state.take_action(action)  # Toma la acción para obtener el nuevo estado
            # Calcula la heurística del nuevo estado y agrega la nueva tupla al heap
            heapq.heappush(heap, (heuristic(new_state), actions + [action], new_state))

    # Si no se encontró una solución, devuelve None, el tiempo de ejecución y el conteo de movimientos del estado inicial
    return None, time.time() - start_time, initial_state.move_count

def heuristic(vacuum_world):
    # Calcula la heurística para un estado dado, que es el número de habitaciones sucias.
    return sum(1 for dirt in vacuum_world.dirt_levels.values() if dirt == '1')


def find_solution_dfs(vacuum_world, max_depth):
    # Encuentra una solución utilizando el algoritmo Depth-First Search (DFS).
    start_time = time.time()  # Registra el tiempo de inicio de la búsqueda
    initial_state = vacuum_world  # Estado inicial del mundo de la aspiradora

    # Verifica si el estado inicial es el estado objetivo
    if initial_state.is_goal_state():
        # Si el estado inicial es el estado objetivo, devuelve una solución vacía y el tiempo de ejecución
        return [], time.time() - start_time, initial_state.move_count

    stack = [(initial_state, [], 0)]  # Inicializa la pila con el estado inicial, una lista de acciones vacía y profundidad 0
    explored = set()  # Conjunto para almacenar los estados explorados

    # Realiza la búsqueda hasta que la pila esté vacía
    while stack:
        current_state, actions, depth = stack.pop()  # Extrae el estado actual, las acciones y la profundidad actual

        # Verifica si la profundidad actual excede la profundidad máxima
        if depth > max_depth:
            continue  # Si es así, continúa con el siguiente estado en la pila

        # Verifica si el estado actual es el estado objetivo
        if current_state.is_goal_state():
            # Si el estado actual es el estado objetivo, devuelve las acciones, el tiempo de ejecución y el conteo de movimientos
            return actions, time.time() - start_time, current_state.move_count

        explored.add(current_state)  # Agrega el estado actual al conjunto de estados explorados

        # Genera los sucesores del estado actual y los agrega a la pila si no han sido explorados
        for action in current_state.get_actions():
            new_state = current_state.take_action(action)  # Toma la acción para obtener el nuevo estado
            if new_state not in explored:
                # Agrega el nuevo estado, las acciones y la profundidad aumentada a la pila
                stack.append((new_state, actions + [action], depth + 1))
                new_state.move_count = current_state.move_count  # Mantiene el contador de movimientos

    # Si no se encontró una solución, devuelve None, el tiempo de ejecución y el conteo de movimientos del estado inicial
    return None, time.time() - start_time, initial_state.move_count


def dijkstra(vacuum_world):
    # Función para realizar la búsqueda utilizando Dijkstra's Algorithm.
    start_time = time.time()  # Registra el tiempo de inicio de la búsqueda
    initial_state = vacuum_world  # Estado inicial del mundo de la aspiradora

    # Verifica si el estado inicial es el estado objetivo
    if initial_state.is_goal_state():
        # Si el estado inicial es el estado objetivo, devuelve una solución vacía y el tiempo de ejecución
        return [], time.time() - start_time, initial_state.move_count

    heap = [(0, [], initial_state)]  # Inicializa el montículo con el costo, acciones y estado inicial
    explored = set()  # Conjunto para almacenar los estados explorados

    # Realiza la búsqueda hasta que el montículo esté vacío
    while heap:
        cost, actions, current_state = heapq.heappop(heap)  # Extrae el costo, las acciones y el estado actual

        # Verifica si el estado actual es el estado objetivo
        if current_state.is_goal_state():
            # Si el estado actual es el estado objetivo, devuelve las acciones, el tiempo de ejecución y el conteo de movimientos
            return actions, time.time() - start_time, current_state.move_count

        # Verifica si el estado actual ya ha sido explorado
        if current_state in explored:
            continue  # Si es así, continúa con el siguiente estado en el montículo

        explored.add(current_state)  # Agrega el estado actual al conjunto de estados explorados

        # Genera los sucesores del estado actual y los agrega al montículo
        for action in current_state.get_actions():
            new_state = current_state.take_action(action)  # Toma la acción para obtener el nuevo estado
            heapq.heappush(heap, (cost + 1, actions + [action], new_state))  # Agrega el nuevo estado al montículo

    # Si no se encontró una solución, devuelve None, el tiempo de ejecución y el conteo de movimientos del estado inicial
    return None, time.time() - start_time, initial_state.move_count

def weighted_a_star(vacuum_world, weight):
    # Función para realizar la búsqueda utilizando Weighted A* Search.
    start_time = time.time()  # Registra el tiempo de inicio de la búsqueda
    initial_state = vacuum_world  # Estado inicial del mundo de la aspiradora

    # Verifica si el estado inicial es el estado objetivo
    if initial_state.is_goal_state():
        # Si el estado inicial es el estado objetivo, devuelve una solución vacía y el tiempo de ejecución
        return [], time.time() - start_time, initial_state.move_count

    heap = [(0, [], initial_state)]  # Inicializa el montículo con el costo, acciones y estado inicial
    explored = set()  # Conjunto para almacenar los estados explorados

    # Realiza la búsqueda hasta que el montículo esté vacío
    while heap:
        cost, actions, current_state = heapq.heappop(heap)  # Extrae el costo, las acciones y el estado actual

        # Verifica si el estado actual es el estado objetivo
        if current_state.is_goal_state():
            # Si el estado actual es el estado objetivo, devuelve las acciones, el tiempo de ejecución y el conteo de movimientos
            return actions, time.time() - start_time, initial_state.move_count

        # Verifica si el estado actual ya ha sido explorado
        if current_state in explored:
            continue  # Si es así, continúa con el siguiente estado en el montículo

        explored.add(current_state)  # Agrega el estado actual al conjunto de estados explorados

        # Genera los sucesores del estado actual y los agrega al montículo
        for action in current_state.get_actions():
            new_state = current_state.take_action(action)  # Toma la acción para obtener el nuevo estado
            heuristic_cost = weight * heuristic(new_state)  # Calcula el costo heurístico ponderado
            heapq.heappush(heap, (cost + 1 + heuristic_cost, actions + [action], new_state))  # Agrega el nuevo estado al montículo

    # Si no se encontró una solución, devuelve None, el tiempo de ejecución y el conteo de movimientos del estado inicial
    return None, time.time() - start_time, initial_state.move_count

def heuristic(vacuum_world):
    # Función heurística: cuenta el número de habitaciones sucias
    return sum(1 for dirt in vacuum_world.dirt_levels.values() if dirt == '1')


def main():
    root = tk.Tk()  # Crea una ventana principal para la interfaz gráfica
    root.title("Vacuum World")  # Establece el título de la ventana como "Vacuum World"

    dirt_levels = {}  # Diccionario para almacenar el nivel de suciedad de cada habitación
    for i in range(8):
        status = input(f"Ingresa el estado de la Habitación {chr(65 + i)} (0 para LIMPIO, 1 para SUCIO): ")
        if status not in ['0', '1']:
            print("¡Estado inválido!")
            return
        dirt_levels[chr(65 + i)] = status  # Almacena el estado de la habitación en el diccionario
    
    location_input = input("Ingresa la Ubicación de la Aspiradora (A-H): ").upper()  # Solicita la ubicación de la aspiradora
    if location_input not in dirt_levels:
        print("¡Ubicación inválida!")
        return
    
    robot_location = ord(location_input) - 65  # Convierte la ubicación de la aspiradora a un índice
    
    vacuum_world = VacuumWorld(dirt_levels, robot_location)  # Crea una instancia del mundo de la aspiradora

    vacuum_world_gui = VacuumWorldGUI(root, dirt_levels, robot_location)  # Crea una interfaz gráfica para el mundo de la aspiradora
    
    vacuum_world_gui.update_tiles()  # Actualiza la interfaz gráfica con los niveles de suciedad y la ubicación de la aspiradora

    print("Elige el algoritmo de búsqueda:")
    print("1. Búsqueda en Profundidad (DFS)")
    print("2. Búsqueda en Profundidad con Límite (DLS)")
    print("3. Algoritmo de Dijkstra")
    print("4. Búsqueda Mejor Primero (GBFS)")
    print("5. Búsqueda A* Ponderada")  # Añadido el algoritmo Búsqueda A* Ponderada

    choice = int(input("Ingresa tu elección (1, 2, 3, 4 o 5): "))

    if choice == 1:
        solution, execution_time, move_count = find_solution_dfs(vacuum_world, float('inf'))
    elif choice == 2:
        max_depth = int(input("Ingresa la profundidad máxima para la Búsqueda en Profundidad con Límite: "))
        solution, execution_time, move_count = find_solution_dfs(vacuum_world, max_depth)
    elif choice == 3:
        solution, execution_time, move_count = dijkstra(vacuum_world)
    elif choice == 4:
        solution, execution_time, move_count = greedy_best_first_search(vacuum_world)
    elif choice == 5:
        weight = float(input("Ingresa el peso para la Búsqueda A* Ponderada: "))
        solution, execution_time, move_count = weighted_a_star(vacuum_world, weight)
    else:
        print("¡Elección inválida!")
        return

    print("Solución:", solution)
    print("Tiempo de ejecución: {:.10f} segundos".format(execution_time))
    print("Número de movimientos:", move_count)

if __name__ == "__main__":
    main()


