# ====================================================
# Program: Comparison of Dijkstra and A* Algorithms
# Case Study: SMKN 9 Medan to Gramedia Gajah Mada
# ====================================================

import heapq
import math

# --- Graph data (based on the table of distances between nodes)
graph = {
    'A': {'B': 1.2, 'C': 1.2},
    'B': {'D': 4.0, 'E': 2.8, 'F': 3.3},
    'C': {'F': 3.3},
    'D': {'G': 0.6},
    'E': {'D': 1.5, 'G': 1.7, 'F': 0.7},
    'F': {'G': 1.5},
    'G': {}
}

# --- Heuristic value (direct distance to Gramedia destination)
heuristic = {
    'A': 4.7,
    'B': 3.9,
    'C': 4.1,
    'D': 0.4,
    'E': 1.3,
    'F': 1.4,
    'G': 0.0
}

# --- Implementation of Dijkstra's Algorithm ---
def dijkstra(graph, start, goal):
    queue = [(0, start, [])]
    visited = set()
    nodes_explored = 0

    while queue:
        (cost, node, path) = heapq.heappop(queue)
        if node in visited:
            continue

        visited.add(node)
        path = path + [node]
        nodes_explored += 1

        if node == goal:
            return (cost, path, nodes_explored)

        for neighbor, weight in graph[node].items():
            if neighbor not in visited:
                heapq.heappush(queue, (cost + weight, neighbor, path))

    return (math.inf, [], nodes_explored)


# --- Implementation of the A* Algorithm ---
def a_star(graph, start, goal, heuristic):
    open_set = [(heuristic[start], 0, start, [])]
    visited = set()
    nodes_explored = 0

    while open_set:
        (f, g, node, path) = heapq.heappop(open_set)
        if node in visited:
            continue

        visited.add(node)
        path = path + [node]
        nodes_explored += 1

        if node == goal:
            return (g, path, nodes_explored)

        for neighbor, weight in graph[node].items():
            if neighbor not in visited:
                g_new = g + weight
                f_new = g_new + heuristic[neighbor]
                heapq.heappush(open_set, (f_new, g_new, neighbor, path))

    return (math.inf, [], nodes_explored)


# --- Execution and Comparison ---
start = 'A'
goal = 'G'

print("=== Shortest Path Analysis SMKN 9 Medan → Gramedia Gajah Mada ===")

# Dijkstra
d_cost, d_path, d_explored = dijkstra(graph, start, goal)
print("\n[Dijkstra Algorithm]")
print(f"Shortest Path  : {' → '.join(d_path)}")
print(f"Total Distance : {d_cost:.1f} km")
print(f"Nodes Explored : {d_explored}")

# A*
a_cost, a_path, a_explored = a_star(graph, start, goal, heuristic)
print("\n[A* Algorithm]")
print(f"Shortest Path  : {' → '.join(a_path)}")
print(f"Total Distance : {a_cost:.1f} km")
print(f"Nodes Explored : {a_explored}")

# --- Comparison ---
print("\n=== Comparison of Results ===")
print(f"{'Parameter':25s} {'Dijkstra':15s} {'A*':15s}")
print(f"{'-'*55}")
print(f"{'Shortest Path':25s} {str(d_path):15s} {str(a_path):15s}")
print(f"{'Total Distance (km)':25s} {d_cost:<15.2f} {a_cost:<15.2f}")
print(f"{'Nodes Explored':25s} {d_explored:<15d} {a_explored:<15d}")
