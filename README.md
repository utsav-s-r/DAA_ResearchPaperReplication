# Visualizer: Dijkstra vs A*

This workspace contains a standalone Python visualizer that animates and compares the node exploration and final paths of Dijkstra's algorithm and A*.

Files added:
- `resources/visualize_compare.py` — standalone visualizer script (uses `graph_example.json` by default).
- `resources/graph_example.json` — example graph + heuristic + coordinates.
- `requirements.txt` — Python dependencies.

Quick start:

1. Create a virtual environment and install requirements:

```bash
python -m venv .venv
.venv\Scripts\activate    # Windows
pip install -r requirements.txt
```

2. Run the visualizer (show animation window):

```bash
python resources/visualize_compare.py --graph resources/graph_example.json
```

Useful animation controls:

```bash
python resources/visualize_compare.py --graph resources/graph_example.json --fps 30 --step-frames 18 --settle-frames 16
```

The visualizer is independent and does not modify the existing Java project. It re-implements the algorithms for visualization purposes so it won't interfere with the Java code. The animation now uses eased transitions, pulsing current nodes, and highlighted path flow to make the comparison smoother and easier to follow.

The figure now starts playing continuously, with a `Play/Pause` button and a `Reset` button placed below the table. The plots, table, and controls are separated into distinct sections so they do not overlap.

The visualizer also includes a scaling benchmark chart that compares explored nodes as the input graph grows, so you can see where A* becomes more efficient than Dijkstra at a glance.
