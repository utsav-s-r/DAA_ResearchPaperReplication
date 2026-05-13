/**
 * Main.java
 * ====================================================================
 * Program: Comparison of Dijkstra and A* Shortest Path Algorithms
 * Case Study: Finding the shortest route from SMKN 9 Medan (Node A)
 *             to Gramedia Gajah Mada (Node G) in Medan, Indonesia.
 *
 * Based on: Research paper comparing Dijkstra and A* algorithms
 *           in urban navigation using real Google Maps distances.
 *
 * Expected Output:
 *   Shortest Path : A -> B -> E -> G
 *   Total Distance: 5.7 km
 *   A* explores fewer nodes than Dijkstra
 * ====================================================================
 */
public class Main {

    public static void main(String[] args) {

        // ============================================================
        // 1. BUILD THE GRAPH
        // ============================================================
        // The road network is modeled as a directed weighted graph.
        // Nodes represent key locations in Medan, Indonesia.
        // Edge weights are distances in km derived from Google Maps.
        //
        // Node Identifiers:
        //   A: SMKN 9 Medan (Start)
        //   B: Kantor Imigrasi (Immigration Office)
        //   C: Gedung MICC
        //   D: Bank Muamalat
        //   E: Indomaret Darussalam Bhakti
        //   F: RS. Bunda Thamrin
        //   G: Gramedia Gajah Mada (Goal)
        // ============================================================

        Graph graph = new Graph();

        // Add all nodes
        graph.addNode("A");
        graph.addNode("B");
        graph.addNode("C");
        graph.addNode("D");
        graph.addNode("E");
        graph.addNode("F");
        graph.addNode("G");

        // Add directed edges with distances (km)
        // From Node A (SMKN 9 Medan)
        graph.addEdge("A", "B", 1.2);  // A to Kantor Imigrasi
        graph.addEdge("A", "C", 1.2);  // A to Gedung MICC

        // From Node B (Kantor Imigrasi)
        graph.addEdge("B", "D", 4.0);  // B to Bank Muamalat
        graph.addEdge("B", "E", 2.8);  // B to Indomaret Darussalam Bhakti
        graph.addEdge("B", "F", 3.3);  // B to RS. Bunda Thamrin

        // From Node C (Gedung MICC)
        graph.addEdge("C", "F", 3.3);  // C to RS. Bunda Thamrin

        // From Node D (Bank Muamalat)
        graph.addEdge("D", "G", 0.6);  // D to Gramedia Gajah Mada

        // From Node E (Indomaret Darussalam Bhakti)
        graph.addEdge("E", "D", 1.5);  // E to Bank Muamalat
        graph.addEdge("E", "G", 1.7);  // E to Gramedia Gajah Mada
        graph.addEdge("E", "F", 0.7);  // E to RS. Bunda Thamrin

        // From Node F (RS. Bunda Thamrin)
        graph.addEdge("F", "G", 1.5);  // F to Gramedia Gajah Mada

        // ============================================================
        // 2. SET HEURISTIC VALUES h(n) FOR A*
        // ============================================================
        // These values represent the straight-line (Euclidean) distance
        // from each node to the goal (Node G: Gramedia Gajah Mada).
        // They were calculated by drawing straight-line routes on maps.
        // The heuristic must be admissible: h(n) <= actual cost to goal.
        // ============================================================

        graph.setHeuristic("A", 4.7);
        graph.setHeuristic("B", 3.9);
        graph.setHeuristic("C", 4.1);
        graph.setHeuristic("D", 0.4);
        graph.setHeuristic("E", 1.3);
        graph.setHeuristic("F", 1.4);
        graph.setHeuristic("G", 0.0);

        // Define start and goal nodes
        String start = "A";
        String goal = "G";

        // ============================================================
        // 3. RUN DIJKSTRA'S ALGORITHM
        // ============================================================
        System.out.println("================================================================");
        System.out.println("  Shortest Path Analysis: SMKN 9 Medan -> Gramedia Gajah Mada");
        System.out.println("================================================================");

        System.out.println("\n[Dijkstra's Algorithm]");
        System.out.println("Strategy: Uninformed search using g(n) only");
        System.out.println("Priority: d(v) = min(d(v), d(u) + w(u,v))");
        System.out.println("------------------------------------------------");

        PathResult dijkstraResult = DijkstraAlgorithm.findShortestPath(graph, start, goal);

        System.out.println("  Shortest Path  : " + dijkstraResult.getPathString());
        System.out.printf("  Total Distance : %.1f km%n", dijkstraResult.getTotalDistance());
        System.out.println("  Nodes Explored : " + dijkstraResult.getNodesExplored());

        // ============================================================
        // 4. RUN A* ALGORITHM
        // ============================================================
        System.out.println("\n[A* Algorithm]");
        System.out.println("Strategy: Informed search using f(n) = g(n) + h(n)");
        System.out.println("Priority: f(n) = cumulative distance + heuristic estimate");
        System.out.println("------------------------------------------------");

        PathResult aStarResult = AStarAlgorithm.findShortestPath(graph, start, goal);

        System.out.println("  Shortest Path  : " + aStarResult.getPathString());
        System.out.printf("  Total Distance : %.1f km%n", aStarResult.getTotalDistance());
        System.out.println("  Nodes Explored : " + aStarResult.getNodesExplored());

        // ============================================================
        // 5. COMPARISON TABLE
        // ============================================================
        System.out.println("\n================================================================");
        System.out.println("  Comparison of Results");
        System.out.println("================================================================");
        System.out.printf("  %-25s %-20s %-20s%n", "Parameter", "Dijkstra", "A*");
        System.out.println("  " + "-".repeat(60));
        System.out.printf("  %-25s %-20s %-20s%n", "Shortest Path",
                dijkstraResult.getPathString(), aStarResult.getPathString());
        System.out.printf("  %-25s %-20.1f %-20.1f%n", "Total Distance (km)",
                dijkstraResult.getTotalDistance(), aStarResult.getTotalDistance());
        System.out.printf("  %-25s %-20d %-20d%n", "Nodes Explored",
                dijkstraResult.getNodesExplored(), aStarResult.getNodesExplored());
        System.out.println();

        // ============================================================
        // 6. EFFICIENCY ANALYSIS
        // ============================================================
        System.out.println("================================================================");
        System.out.println("  Efficiency Analysis");
        System.out.println("================================================================");

        if (aStarResult.getNodesExplored() < dijkstraResult.getNodesExplored()) {
            int saved = dijkstraResult.getNodesExplored() - aStarResult.getNodesExplored();
            System.out.println("  A* is MORE EFFICIENT than Dijkstra.");
            System.out.println("  A* explored " + saved + " fewer node(s) than Dijkstra.");
            System.out.println("  This demonstrates the advantage of heuristic-guided search.");
        } else if (aStarResult.getNodesExplored() == dijkstraResult.getNodesExplored()) {
            System.out.println("  Both algorithms explored the same number of nodes.");
        } else {
            System.out.println("  Dijkstra explored fewer nodes in this specific case.");
        }

        System.out.println();
        System.out.println("  Both algorithms found the same optimal path: "
                + dijkstraResult.getPathString());
        System.out.printf("  Both confirm the shortest distance is: %.1f km%n",
                dijkstraResult.getTotalDistance());
        System.out.println("================================================================");
    }
}
