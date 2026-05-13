import java.util.*;

/**
 * DijkstraAlgorithm.java
 * Implements Dijkstra's shortest-path algorithm.
 *
 * Dijkstra is an uninformed (blind) search algorithm that explores nodes
 * based solely on the cumulative distance g(n) from the start node.
 * It guarantees finding the globally optimal shortest path, but may
 * explore more nodes than necessary since it has no knowledge of the
 * goal's location.
 *
 * Priority: g(n) — the total distance traveled from the start to the current node.
 */
public class DijkstraAlgorithm {

    /**
     * Finds the shortest path from 'start' to 'goal' using Dijkstra's algorithm.
     *
     * @param graph The weighted graph to search.
     * @param start The starting node identifier (e.g., "A").
     * @param goal  The goal node identifier (e.g., "G").
     * @return A PathResult containing the shortest path, total distance, and nodes explored.
     */
    public static PathResult findShortestPath(Graph graph, String start, String goal) {

        // Priority queue ordered by g(n) — cumulative distance from start.
        // Each entry: [g(n), nodeName]
        PriorityQueue<double[]> pq = new PriorityQueue<>(
            Comparator.comparingDouble(a -> a[0])
        );

        // Maps each node name to an integer index for use in the priority queue array
        Map<String, Integer> nodeIndex = new HashMap<>();
        String[] nodeNames = graph.getNodes().toArray(new String[0]);
        Arrays.sort(nodeNames); // Sort for deterministic ordering
        for (int i = 0; i < nodeNames.length; i++) {
            nodeIndex.put(nodeNames[i], i);
        }

        // Track the best known distance to each node
        Map<String, Double> dist = new HashMap<>();
        for (String node : graph.getNodes()) {
            dist.put(node, Double.MAX_VALUE);
        }
        dist.put(start, 0.0);

        // Track the predecessor of each node for path reconstruction
        Map<String, String> previous = new HashMap<>();

        // Track visited nodes to avoid reprocessing
        Set<String> visited = new HashSet<>();

        // Counter: incremented each time a node is dequeued (explored)
        int nodesExplored = 0;

        // Enqueue the start node with distance 0
        pq.add(new double[]{0.0, nodeIndex.get(start)});

        while (!pq.isEmpty()) {
            // Dequeue the node with the smallest g(n)
            double[] current = pq.poll();
            double currentDist = current[0];
            String currentNode = nodeNames[(int) current[1]];

            // Skip if already visited (a shorter path was already found)
            if (visited.contains(currentNode)) {
                continue;
            }

            // Mark as visited and count this exploration
            visited.add(currentNode);
            nodesExplored++;

            // If we've reached the goal, reconstruct and return the path
            if (currentNode.equals(goal)) {
                List<String> path = reconstructPath(previous, start, goal);
                return new PathResult(path, currentDist, nodesExplored);
            }

            // Relax all neighbors of the current node
            for (Graph.Edge edge : graph.getNeighbors(currentNode)) {
                if (!visited.contains(edge.destination)) {
                    double newDist = currentDist + edge.weight;

                    // Update if this path is shorter than the previously known one
                    if (newDist < dist.get(edge.destination)) {
                        dist.put(edge.destination, newDist);
                        previous.put(edge.destination, currentNode);
                        pq.add(new double[]{newDist, nodeIndex.get(edge.destination)});
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
