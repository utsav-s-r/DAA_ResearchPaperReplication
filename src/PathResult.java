import java.util.*;

/**
 * PathResult.java
 * Encapsulates the result of a shortest-path algorithm execution.
 * Stores the path found, total distance, and number of nodes explored.
 */
public class PathResult {

    private final List<String> path;
    private final double totalDistance;
    private final int nodesExplored;

    public PathResult(List<String> path, double totalDistance, int nodesExplored) {
        this.path = path;
        this.totalDistance = totalDistance;
        this.nodesExplored = nodesExplored;
    }

    /** Returns the ordered list of nodes in the shortest path. */
    public List<String> getPath() {
        return path;
    }

    /** Returns the total distance (cost) of the shortest path in km. */
    public double getTotalDistance() {
        return totalDistance;
    }

    /** Returns how many nodes were dequeued (explored) during the search. */
    public int getNodesExplored() {
        return nodesExplored;
    }

    /**
     * Returns the path as a formatted string, e.g. "A -> B -> E -> G".
     */
    public String getPathString() {
        return String.join(" -> ", path);
    }
}
