import java.util.*;

/**
 * AStarAlgorithm.java
 * Implements the A* (A-Star) shortest-path algorithm.
 *
 * A* is an informed (heuristic) search algorithm that combines the actual
 * distance traveled g(n) with a heuristic estimate h(n) of the remaining
 * distance to the goal. It uses the evaluation function:
 *
 *     f(n) = g(n) + h(n)
 *
 * Where:
 *   g(n) = cumulative distance from start to current node
 *   h(n) = heuristic estimate (straight-line distance) from current node to goal
 *   f(n) = estimated total cost of the path through node n
 *
 * A* preserves optimality (when h(n) is admissible) while significantly
 * reducing the number of nodes explored compared to Dijkstra.
 *
 * Priority: f(n) = g(n) + h(n)
 */
public class AStarAlgorithm {

    /**
     * Finds the shortest path from 'start' to 'goal' using the A* algorithm.
     *
     * @param graph The weighted graph to search (includes heuristic values).
     * @param start The starting node identifier (e.g., "A").
     * @param goal  The goal node identifier (e.g., "G").
     * @return A PathResult containing the shortest path, total distance, and nodes explored.
     */
    public static PathResult findShortestPath(Graph graph, String start, String goal) {

        // Priority queue ordered by f(n) = g(n) + h(n).
        // Each entry: [f(n), g(n), nodeIndex]
        PriorityQueue<double[]> openSet = new PriorityQueue<>(
            Comparator.comparingDouble(a -> a[0])
        );

        // Maps each node name to an integer index for use in the priority queue array
        Map<String, Integer> nodeIndex = new HashMap<>();
        String[] nodeNames = graph.getNodes().toArray(new String[0]);
        Arrays.sort(nodeNames); // Sort for deterministic ordering
        for (int i = 0; i < nodeNames.length; i++) {
            nodeIndex.put(nodeNames[i], i);
        }

        // Track the best known g(n) distance to each node
        Map<String, Double> gScore = new HashMap<>();
        for (String node : graph.getNodes()) {
            gScore.put(node, Double.MAX_VALUE);
        }
        gScore.put(start, 0.0);

        // Track the predecessor of each node for path reconstruction
        Map<String, String> previous = new HashMap<>();

        // Track visited (closed) nodes to avoid reprocessing
        Set<String> visited = new HashSet<>();

        // Counter: incremented each time a node is dequeued (explored)
        int nodesExplored = 0;

        // Enqueue the start node: f(start) = g(start) + h(start) = 0 + h(start)
        double startF = 0.0 + graph.getHeuristic(start);
        openSet.add(new double[]{startF, 0.0, nodeIndex.get(start)});

        while (!openSet.isEmpty()) {
            // Dequeue the node with the smallest f(n)
            double[] current = openSet.poll();
            double currentG = current[1];
            String currentNode = nodeNames[(int) current[2]];

            // Skip if already visited (closed set)
            if (visited.contains(currentNode)) {
                continue;
            }

            // Mark as visited and count this exploration
            visited.add(currentNode);
            nodesExplored++;

            // If we've reached the goal, reconstruct and return the path
            if (currentNode.equals(goal)) {
                List<String> path = reconstructPath(previous, start, goal);
                return new PathResult(path, currentG, nodesExplored);
            }

            // Expand all neighbors of the current node
            for (Graph.Edge edge : graph.getNeighbors(currentNode)) {
                if (!visited.contains(edge.destination)) {
                    // Calculate tentative g(n) for the neighbor
                    double tentativeG = currentG + edge.weight;

                    // Only proceed if this path is better than any previously known
                    if (tentativeG < gScore.get(edge.destination)) {
                        gScore.put(edge.destination, tentativeG);
                        previous.put(edge.destination, currentNode);

                        // Calculate f(n) = g(n) + h(n) for the neighbor
                        double f = tentativeG + graph.getHeuristic(edge.destination);
                        openSet.add(new double[]{f, tentativeG, nodeIndex.get(edge.destination)});
                    }
                }
            }
        }

        // No path found (should not happen in a connected graph)
        return new PathResult(Collections.emptyList(), Double.MAX_VALUE, nodesExplored);
    }

    /**
     * Reconstructs the path from start to goal using the predecessor map.
     */
    private static List<String> reconstructPath(Map<String, String> previous,
                                                 String start, String goal) {
        List<String> path = new ArrayList<>();
        String current = goal;

        while (current != null) {
            path.add(current);
            if (current.equals(start)) break;
            current = previous.get(current);
        }

        Collections.reverse(path);
        return path;
    }
}
