from queue import Queue, LifoQueue, PriorityQueue

# Helper function to check if a state is valid
def is_valid(state):
    missionaries_left, cannibals_left, boat_left, missionaries_right, cannibals_right = state

    if (missionaries_left < 0 or missionaries_right < 0 or
        cannibals_left < 0 or cannibals_right < 0):
        return False

    if missionaries_left > 3 or missionaries_right > 3 or cannibals_left > 3 or cannibals_right > 3:
        return False

    if (cannibals_left > missionaries_left > 0) or (cannibals_right > missionaries_right > 0):
        return False

    return True

# Helper function to generate possible next states
def generate_next_states(state):
    possible_states = []
    missionaries_left, cannibals_left, boat_left, missionaries_right, cannibals_right = state

    if boat_left == 1:
        for m in range(3):
            for c in range(3):
                if m + c >= 1 and m + c <= 2:
                    new_state = (
                        missionaries_left - m,
                        cannibals_left - c,
                        0,
                        missionaries_right + m,
                        cannibals_right + c
                    )
                    if is_valid(new_state):
                        possible_states.append(new_state)
    else:
        for m in range(3):
            for c in range(3):
                if m + c >= 1 and m + c <= 2:
                    new_state = (
                        missionaries_left + m,
                        cannibals_left + c,
                        1,
                        missionaries_right - m,
                        cannibals_right - c
                    )
                    if is_valid(new_state):
                        possible_states.append(new_state)
    
    return possible_states

# Helper function to check if the goal state is reached
def is_goal_state(state):
    return state == goal_state

# Breadth-First Search (BFS)
def bfs(initial_state):
    visited = set()
    queue = Queue()
    queue.put([initial_state])

    while not queue.empty():
        path = queue.get()
        current_state = path[-1]

        if is_goal_state(current_state):
            return path

        for next_state in generate_next_states(current_state):
            if next_state not in visited:
                visited.add(next_state)
                new_path = list(path)
                new_path.append(next_state)
                queue.put(new_path)

# Depth-First Search (DFS)
def dfs(initial_state):
    visited = set()
    stack = LifoQueue()
    stack.put([initial_state])

    while not stack.empty():
        path = stack.get()
        current_state = path[-1]

        if is_goal_state(current_state):
            return path

        for next_state in generate_next_states(current_state):
            if next_state not in visited:
                visited.add(next_state)
                new_path = list(path)
                new_path.append(next_state)
                stack.put(new_path)

# Hill Climbing (Greedy Best-First Search)
def hill_climbing(initial_state, max_sideways_moves=100):
    current_state = initial_state
    sideways_moves = 0

    while not is_goal_state(current_state):
        possible_states = generate_next_states(current_state)
        possible_states.sort(key=lambda state: heuristic(state), reverse=True)

        # Find the best state among neighbors
        best_state = possible_states[0]
        best_heuristic = heuristic(best_state)

        # Check if sideways move is needed
        sideways_moves += 1
        if best_heuristic <= heuristic(current_state):
            sideways_moves = 0

        # If no improvement or too many sideways moves, terminate
        if best_heuristic <= heuristic(current_state) or sideways_moves >= max_sideways_moves:
            break

        current_state = best_state

    return [initial_state, current_state]

# A* Search
def astar(initial_state):
    visited = set()
    queue = PriorityQueue()
    queue.put((heuristic(initial_state), [initial_state]))

    while not queue.empty():
        _, path = queue.get()
        current_state = path[-1]

        if is_goal_state(current_state):
            return path

        for next_state in generate_next_states(current_state):
            if next_state not in visited:
                visited.add(next_state)
                new_path = list(path)
                new_path.append(next_state)
                queue.put((heuristic(next_state) + len(new_path), new_path))

# Heuristic function for A*
def heuristic(state):
    missionaries_left, cannibals_left, boat_left, missionaries_right, cannibals_right = state
    return missionaries_left + cannibals_left - 2 * boat_left

# Initial state: (3, 3, 1, 0, 0)
initial_state = (3, 3, 1, 0, 0)
goal_state = (0, 0, 0, 3, 3)

# Breadth-First Search
bfs_path = bfs(initial_state)
print("BFS Path:", bfs_path)

# Depth-First Search
dfs_path = dfs(initial_state)
print("DFS Path:", dfs_path)

# Hill Climbing
hill_climbing_path = hill_climbing(initial_state)
print("Hill Climbing Path:", hill_climbing_path)

# A* Search
astar_path = astar(initial_state)
print("A* Path:", astar_path)
