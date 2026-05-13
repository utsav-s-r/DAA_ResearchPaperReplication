import java.util.*;

/**
 * Graph.java
 * Represents the weighted, undirected/directed graph for the road network.
 * Stores adjacency list and heuristic values for A*.
 */
public class Graph {

    // Adjacency list: node -> list of (neighbor, weight) pairs
    private final Map<String, List<Edge>> adjacencyList;

    // Heuristic values h(n): straight-line distance estimate to the goal
    private final Map<String, Double> heuristics;

    /**
     * Inner class representing a weighted edge.
     */
    public static class Edge {
        String destination;
        double weight;

        public Edge(String destination, double weight) {
            this.destination = destination;
            this.weight = weight;
        }
    }

    public Graph() {
        this.adjacencyList = new HashMap<>();
        this.heuristics = new HashMap<>();
    }

    /**
     * Adds a node to the graph (creates an empty neighbor list if not present).
     */
    public void addNode(String node) {
        adjacencyList.putIfAbsent(node, new ArrayList<>());
    }

    /**
     * Adds a directed edge from 'source' to 'destination' with the given weight.
     * Both nodes are created if they don't already exist.
     */
    public void addEdge(String source, String destination, double weight) {
        addNode(source);
        addNode(destination);
        adjacencyList.get(source).add(new Edge(destination, weight));
    }

    /**
     * Sets the heuristic value h(n) for a given node.
     */
    public void setHeuristic(String node, double value) {
        heuristics.put(node, value);
    }

    /**
     * Returns the list of edges (neighbors) for a given node.
     */
    public List<Edge> getNeighbors(String node) {
        return adjacencyList.getOrDefault(node, Collections.emptyList());
    }

    /**
     * Returns the heuristic value h(n) for a given node.
     * Returns 0.0 if not set (safe default for the goal node).
     */
    public double getHeuristic(String node) {
        return heuristics.getOrDefault(node, 0.0);
    }

    /**
     * Returns all node names in the graph.
     */
    public Set<String> getNodes() {
        return adjacencyList.keySet();
    }
}
