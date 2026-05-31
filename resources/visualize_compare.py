import argparse
import heapq
import json
import math

import matplotlib.pyplot as plt
import matplotlib.patheffects as pe
import networkx as nx
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button


PALETTE = {
    'bg': '#0a1320',
    'panel': '#111c2c',
    'panel_edge': '#31465f',
    'grid': '#223246',
    'base_node': '#345d84',
    'frontier': '#e8be5b',
    'current': '#ff8b45',
    'visited': '#72d8ff',
    'path': '#7ced95',
    'goal': '#d7ef71',
    'text': '#edf4fb',
    'muted': '#a7b7cb',
    'edge': '#33495f',
    'edge_text': '#c9d5e3',
}


def smoothstep(value):
    return value * value * (3.0 - 2.0 * value)


def load_graph(path):
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    graph = data.get('graph', {})
    heuristic = data.get('heuristic', {})
    coords = data.get('coords', {})
    start = data.get('start')
    goal = data.get('goal')

    if not graph and 'A' in data:
        graph = data
        heuristic = {}
        coords = {}

    return graph, heuristic, coords, start, goal


def dijkstra_trace(graph, start, goal):
    heap = [(0.0, start)]
    visited = set()
    previous = {}
    dist = {node: math.inf for node in graph}
    dist[start] = 0.0
    steps = []

    while heap:
        cost, node = heapq.heappop(heap)
        if node in visited:
            continue

        visited.add(node)
        frontier = sorted({item[1] for item in heap})
        steps.append({
            'algorithm': 'Dijkstra',
            'current': node,
            'visited': set(visited),
            'frontier': set(frontier),
            'previous': dict(previous),
            'scores': dict(dist),
            'cost': cost,
            'done': node == goal,
        })

        if node == goal:
            break

        for neighbor, weight in graph.get(node, {}).items():
            if neighbor in visited:
                continue
            new_cost = cost + weight
            if new_cost < dist.get(neighbor, math.inf):
                dist[neighbor] = new_cost
                previous[neighbor] = node
                heapq.heappush(heap, (new_cost, neighbor))

    if not steps:
        steps.append({
            'algorithm': 'Dijkstra',
            'current': None,
            'visited': set(),
            'frontier': set(),
            'previous': {},
            'scores': dict(dist),
            'cost': math.inf,
            'done': False,
        })
    return steps


def astar_trace(graph, heuristic, start, goal):
    heap = [(heuristic.get(start, 0.0), 0.0, start)]
    visited = set()
    previous = {}
    g_score = {node: math.inf for node in graph}
    g_score[start] = 0.0
    steps = []

    while heap:
        f_score, g_cost, node = heapq.heappop(heap)
        if node in visited:
            continue

        visited.add(node)
        frontier = sorted({item[2] for item in heap})
        steps.append({
            'algorithm': 'A*',
            'current': node,
            'visited': set(visited),
            'frontier': set(frontier),
            'previous': dict(previous),
            'scores': dict(g_score),
            'cost': g_cost,
            'done': node == goal,
            'f_score': f_score,
        })

        if node == goal:
            break

        for neighbor, weight in graph.get(node, {}).items():
            if neighbor in visited:
                continue
            tentative_g = g_cost + weight
            if tentative_g < g_score.get(neighbor, math.inf):
                g_score[neighbor] = tentative_g
                previous[neighbor] = node
                heapq.heappush(heap, (tentative_g + heuristic.get(neighbor, 0.0), tentative_g, neighbor))

    if not steps:
        steps.append({
            'algorithm': 'A*',
            'current': None,
            'visited': set(),
            'frontier': set(),
            'previous': {},
            'scores': dict(g_score),
            'cost': math.inf,
            'done': False,
            'f_score': math.inf,
        })
    return steps


def reconstruct_path(previous, start, goal):
    if start is None or goal is None:
        return []
    path = []
    current = goal
    while current is not None:
        path.append(current)
        if current == start:
            break
        current = previous.get(current)
    if not path or path[-1] != start:
        return []
    path.reverse()
    return path


def build_nx(graph):
    graph_nx = nx.DiGraph()
    for source, neighbors in graph.items():
        graph_nx.add_node(source)
        for destination, weight in neighbors.items():
            graph_nx.add_edge(source, destination, weight=weight)
    return graph_nx


def infer_start_goal(graph, start, goal):
    nodes = list(graph.keys())
    inferred_start = start or (nodes[0] if nodes else None)
    inferred_goal = goal or ('G' if 'G' in graph else (nodes[-1] if nodes else None))
    return inferred_start, inferred_goal


def layout_graph(graph_nx, coords):
    if coords:
        return {node: tuple(coords[node]) for node in coords if node in graph_nx}
    return nx.spring_layout(graph_nx, seed=12, k=1.2)


def build_input_scaling_benchmark(layer_counts, branches_per_layer=4):
    benchmark_rows = []

    for layers in layer_counts:
        graph = {}
        heuristic = {}
        start = 'S0'
        goal = f'S{layers}'

        for layer in range(layers + 1):
            node = f'S{layer}'
            graph.setdefault(node, {})
            heuristic[node] = float(layers - layer)

        for layer in range(layers):
            current = f'S{layer}'
            next_main = f'S{layer + 1}'
            graph[current][next_main] = 1.0

            for branch_index in range(branches_per_layer):
                branch = f'B{layer}_{branch_index}'
                graph[current][branch] = 1.0
                graph.setdefault(branch, {})
                graph[branch][next_main] = 2.2
                heuristic[branch] = float(layers - layer)

        graph[goal] = graph.get(goal, {})
        heuristic[goal] = 0.0

        dijkstra_steps = dijkstra_trace(graph, start, goal)
        astar_steps = astar_trace(graph, heuristic, start, goal)
        benchmark_rows.append({
            'layers': layers,
            'nodes': len(graph),
            'dijkstra_visited': len(dijkstra_steps[-1]['visited']),
            'astar_visited': len(astar_steps[-1]['visited']),
            'dijkstra_steps': len(dijkstra_steps),
            'astar_steps': len(astar_steps),
        })

    return benchmark_rows


def render_efficiency_chart(ax, benchmark_rows):
    ax.set_facecolor(PALETTE['panel'])

    input_sizes = [row['nodes'] for row in benchmark_rows]
    dijkstra_counts = [row['dijkstra_visited'] for row in benchmark_rows]
    astar_counts = [row['astar_visited'] for row in benchmark_rows]
    savings_pct = [100.0 * (1.0 - astar / dijkstra) if dijkstra else 0.0 for astar, dijkstra in zip(astar_counts, dijkstra_counts)]

    ax.grid(True, axis='y', color=PALETTE['grid'], alpha=0.55, linewidth=1.0)
    ax.grid(False, axis='x')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.spines['bottom'].set_color(PALETTE['panel_edge'])
    ax.spines['left'].set_color(PALETTE['panel_edge'])

    ax.fill_between(input_sizes, dijkstra_counts, color='#2f5f8d', alpha=0.18, zorder=1)
    ax.fill_between(input_sizes, astar_counts, color='#f59e0b', alpha=0.12, zorder=1)
    ax.plot(
        input_sizes,
        dijkstra_counts,
        color='#7dd3fc',
        linewidth=2.8,
        marker='o',
        markersize=7.5,
        label='Dijkstra nodes explored',
        zorder=3,
    )
    ax.plot(
        input_sizes,
        astar_counts,
        color='#f59e0b',
        linewidth=3.0,
        marker='D',
        markersize=7,
        label='A* nodes explored',
        zorder=4,
    )

    ax.set_xlabel('Input size (graph nodes)', fontsize=11.5, color=PALETTE['text'], labelpad=10)
    ax.set_ylabel('Nodes explored', fontsize=11.5, color=PALETTE['text'], labelpad=10)
    ax.tick_params(colors=PALETTE['text'], labelsize=10)

    legend = ax.legend(
        loc='upper left',
        frameon=True,
        framealpha=0.92,
        facecolor='#0a1626',
        edgecolor=PALETTE['panel_edge'],
        fontsize=10,
    )
    for text in legend.get_texts():
        text.set_color(PALETTE['text'])

    max_y = max(dijkstra_counts + astar_counts)
    top_y = max_y * 1.10 if max_y else 1.0
    ax.set_ylim(0, top_y)
    ax.set_xlim(min(input_sizes) - 1, max(input_sizes) + 1)

    final_row = benchmark_rows[-1]
    final_savings = savings_pct[-1]
    ax.text(
        0.98,
        0.93,
        f"Final graph: A* visits {final_row['astar_visited']} vs {final_row['dijkstra_visited']}\n"
        f"{final_savings:.1f}% fewer explored nodes",
        transform=ax.transAxes,
        ha='right',
        va='top',
        color=PALETTE['text'],
        fontsize=9.8,
        bbox=dict(boxstyle='round,pad=0.42', facecolor='#0a1626', edgecolor=PALETTE['panel_edge'], alpha=0.95),
    )

    for x, dijkstra_y, astar_y, savings in zip(input_sizes, dijkstra_counts, astar_counts, savings_pct):
        ax.text(
            x,
            astar_y + max_y * 0.03,
            f'{savings:.0f}%',
            ha='center',
            va='bottom',
            color=PALETTE['path'],
            fontsize=9.2,
            weight='bold',
            path_effects=[pe.withStroke(linewidth=3, foreground='#07111f')],
        )


def create_efficiency_figure(benchmark_rows):
    fig, ax = plt.subplots(figsize=(12.8, 7.2), facecolor=PALETTE['bg'])
    fig.subplots_adjust(left=0.08, right=0.975, top=0.87, bottom=0.12)
    fig.suptitle(
        'Input-Size Efficiency Comparison',
        fontsize=21,
        weight='bold',
        color=PALETTE['text'],
        y=0.965,
    )
    fig.text(
        0.5,
        0.915,
        'A* becomes more efficient as the graph grows because the heuristic keeps the search focused.',
        ha='center',
        va='center',
        fontsize=11.2,
        color=PALETTE['muted'],
    )
    render_efficiency_chart(ax, benchmark_rows)
    return fig



def prepare_timeline(trace, steps_per_transition=18, settle_frames=10):
    if len(trace) == 1:
        return [{'index': 0, 'progress': 1.0} for _ in range(settle_frames)]

    timeline = []
    for index in range(len(trace)):
        final_step = index == len(trace) - 1
        for frame in range(steps_per_transition):
            if final_step:
                progress = 1.0
            else:
                progress = smoothstep(frame / float(steps_per_transition - 1))
            timeline.append({'index': index, 'progress': progress})

    timeline.extend({'index': len(trace) - 1, 'progress': 1.0} for _ in range(settle_frames))
    return timeline


def score_for_node(step, node, start, goal, progress):
    visited = step['visited']
    frontier = step['frontier']
    current = step['current']
    previous = step['previous']

    if node == goal and goal in visited:
        return 'goal', 1.0
    if node == current:
        return 'current', 1.0 + 0.10 * math.sin(progress * math.pi)
    if node in visited:
        return 'visited', 1.0
    if node in frontier:
        return 'frontier', 0.85 + 0.15 * progress
    if node == start:
        return 'start', 0.95
    if node in previous:
        return 'discovered', 0.50 + 0.15 * progress
    return 'idle', 0.30


def node_style(kind):
    if kind == 'goal':
        return PALETTE['goal'], PALETTE['goal']
    if kind == 'current':
        return PALETTE['current'], PALETTE['current']
    if kind == 'visited':
        return PALETTE['visited'], PALETTE['visited']
    if kind == 'frontier':
        return PALETTE['frontier'], PALETTE['frontier']
    if kind == 'start':
        return '#b4f8c8', '#b4f8c8'
    if kind == 'discovered':
        return '#87a8d0', '#87a8d0'
    return PALETTE['base_node'], PALETTE['base_node']


def draw_panel(ax, graph_nx, pos, trace, frame_state, start, goal, title, accent):
    step = trace[frame_state['index']]
    progress = frame_state['progress']
    path_to_current = reconstruct_path(step['previous'], start, step['current'])
    final_path = reconstruct_path(step['previous'], start, goal) if step['done'] else []

    ax.set_facecolor(PALETTE['panel'])
    ax.clear()
    ax.set_facecolor(PALETTE['panel'])
    ax.set_title(title, color=PALETTE['text'], fontsize=17, weight='bold', pad=18)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.patch.set_edgecolor(PALETTE['panel_edge'])
    ax.patch.set_linewidth(1.2)

    # subtle background glow
    xs = [point[0] for point in pos.values()]
    ys = [point[1] for point in pos.values()]
    x_mid = sum(xs) / len(xs)
    y_mid = sum(ys) / len(ys)
    ax.scatter([x_mid], [y_mid], s=9000, color=accent, alpha=0.03, zorder=0, linewidths=0)

    base_edge_colors = []
    base_edge_widths = []
    base_edge_alphas = []
    path_edges = set(zip(path_to_current, path_to_current[1:]))
    final_edges = set(zip(final_path, final_path[1:]))

    for source, destination in graph_nx.edges():
        if (source, destination) in final_edges:
            base_edge_colors.append(PALETTE['path'])
            base_edge_widths.append(3.8)
            base_edge_alphas.append(0.95)
        elif (source, destination) in path_edges:
            base_edge_colors.append(accent)
            base_edge_widths.append(3.0)
            base_edge_alphas.append(0.88)
        elif source in step['visited']:
            base_edge_colors.append('#56708a')
            base_edge_widths.append(1.8)
            base_edge_alphas.append(0.55)
        else:
            base_edge_colors.append(PALETTE['edge'])
            base_edge_widths.append(1.35)
            base_edge_alphas.append(0.45)

    nx.draw_networkx_edges(
        graph_nx,
        pos,
        ax=ax,
        edge_color=base_edge_colors,
        width=base_edge_widths,
        alpha=base_edge_alphas,
        arrows=True,
        arrowsize=17,
        connectionstyle='arc3,rad=0.06',
        min_source_margin=12,
        min_target_margin=12,
    )

    node_sizes = []
    node_colors = []
    node_edges = []
    current_node = step['current']

    for node in graph_nx.nodes():
        kind, size_factor = score_for_node(step, node, start, goal, progress)
        fill, edge = node_style(kind)
        size = 820 * size_factor
        if node == current_node:
            size *= 1.18
        elif node in final_path:
            size *= 1.08
        node_sizes.append(size)
        node_colors.append(fill)
        node_edges.append(edge)

    nx.draw_networkx_nodes(
        graph_nx,
        pos,
        ax=ax,
        node_size=node_sizes,
        node_color=node_colors,
        edgecolors=node_edges,
        linewidths=[2.2 if node == current_node else 1.5 for node in graph_nx.nodes()],
        alpha=0.98,
    )

    nx.draw_networkx_labels(
        graph_nx,
        pos,
        ax=ax,
        font_size=12,
        font_weight='bold',
        font_color=PALETTE['text'],
        bbox=dict(boxstyle='round,pad=0.25', facecolor='#07111f', edgecolor='none', alpha=0.15),
    )

    edge_labels = nx.get_edge_attributes(graph_nx, 'weight')
    nx.draw_networkx_edge_labels(
        graph_nx,
        pos,
        edge_labels=edge_labels,
        ax=ax,
        font_size=9,
        font_color=PALETTE['edge_text'],
        rotate=False,
        bbox=dict(boxstyle='round,pad=0.12', facecolor='#07111f', edgecolor='none', alpha=0.45),
    )

    panel_box = dict(boxstyle='round,pad=0.55', facecolor='#0a1626', edgecolor=PALETTE['panel_edge'], alpha=0.92)
    info_lines = [
        f"Current: {current_node or '—'}",
        f"Visited: {len(step['visited'])}",
        f"Frontier: {len(step['frontier'])}",
        f"Cost: {step['cost']:.1f}" if math.isfinite(step['cost']) else 'Cost: ∞',
    ]
    if final_path:
        info_lines.append(f"Final path: {' -> '.join(final_path)}")
    elif path_to_current:
        info_lines.append(f"Flow: {' -> '.join(path_to_current)}")

    ax.text(
        0.03,
        0.98,
        f"{title}\n{chr(10).join(info_lines)}",
        transform=ax.transAxes,
        va='top',
        ha='left',
        color=PALETTE['text'],
        fontsize=10.5,
        linespacing=1.35,
        bbox=panel_box,
        path_effects=[pe.withStroke(linewidth=3.5, foreground='#00000066')],
    )

    status = 'found goal' if step['done'] else 'searching'
    status_color = PALETTE['path'] if step['done'] else accent
    ax.text(
        0.97,
        0.03,
        status.upper(),
        transform=ax.transAxes,
        ha='right',
        va='bottom',
        color=status_color,
        fontsize=9.5,
        weight='bold',
        bbox=dict(boxstyle='round,pad=0.3', facecolor='#0a1626', edgecolor=status_color, alpha=0.88),
    )


def visualize(graph_file, fps=30, step_frames=18, settle_frames=16):
    graph, heuristic, coords, start, goal = load_graph(graph_file)
    start, goal = infer_start_goal(graph, start, goal)

    if start is None or goal is None:
        raise ValueError('Graph must contain at least a start and goal node')

    graph_nx = build_nx(graph)
    pos = layout_graph(graph_nx, coords)

    dijkstra_steps = dijkstra_trace(graph, start, goal)
    astar_steps = astar_trace(graph, heuristic, start, goal)
    dijkstra_timeline = prepare_timeline(dijkstra_steps, steps_per_transition=step_frames, settle_frames=settle_frames)
    astar_timeline = prepare_timeline(astar_steps, steps_per_transition=step_frames, settle_frames=settle_frames)
    frame_count = max(len(dijkstra_timeline), len(astar_timeline))
    benchmark_rows = build_input_scaling_benchmark([4, 6, 8, 10, 12, 14], branches_per_layer=4)

    plt.style.use('dark_background')
    plt.rcParams.update({
        'figure.facecolor': PALETTE['bg'],
        'axes.facecolor': PALETTE['panel'],
        'savefig.facecolor': PALETTE['bg'],
        'text.color': PALETTE['text'],
        'axes.labelcolor': PALETTE['text'],
        'xtick.color': PALETTE['text'],
        'ytick.color': PALETTE['text'],
    })

    fig = plt.figure(figsize=(15.8, 9.8), facecolor=PALETTE['bg'])
    grid = fig.add_gridspec(
        3,
        2,
        height_ratios=[7.1, 1.95, 0.72],
        left=0.035,
        right=0.985,
        top=0.905,
        bottom=0.06,
        hspace=0.24,
        wspace=0.07,
    )
    ax_dijkstra = fig.add_subplot(grid[0, 0])
    ax_astar = fig.add_subplot(grid[0, 1])
    ax_table = fig.add_subplot(grid[1, :])
    ax_controls = fig.add_subplot(grid[2, :])
    fig.suptitle(
        'Dijkstra vs A* Path Flow',
        fontsize=22,
        weight='bold',
        color=PALETTE['text'],
        y=0.972,
    )
    fig.text(
        0.5,
        0.935,
        'Animated exploration, side-by-side path flow, and a compact comparison table',
        ha='center',
        va='center',
        fontsize=11.5,
        color=PALETTE['muted'],
    )
    status_text = None

    ax_table.set_facecolor(PALETTE['panel'])
    ax_controls.set_facecolor(PALETTE['panel'])
    ax_table.axis('off')
    ax_controls.axis('off')

    efficiency_fig = create_efficiency_figure(benchmark_rows)

    play_ax = fig.add_axes([0.41, 0.062, 0.12, 0.04])
    reset_ax = fig.add_axes([0.54, 0.062, 0.12, 0.04])
    play_button = Button(play_ax, 'Pause', color='#13243a', hovercolor='#1d3552')
    reset_button = Button(reset_ax, 'Reset', color='#13243a', hovercolor='#1d3552')
    for button in (play_button, reset_button):
        button.label.set_color(PALETTE['text'])
        button.label.set_fontsize(11)

    is_running = {'value': True}
    current_frame = {'value': 0}

    def build_summary_table(d_step, a_step):
        ax_table.clear()
        ax_table.set_facecolor(PALETTE['panel'])
        ax_table.axis('off')
        ax_table.set_title('Comparison Table', color=PALETTE['text'], fontsize=15, weight='bold', pad=8)

        def format_path(step):
            path = reconstruct_path(step['previous'], start, goal)
            return ' -> '.join(path) if path else '—'

        rows = [
            ['Algorithm', 'Cost', 'Visited', 'Frontier', 'Path'],
            ['Dijkstra', f"{d_step['cost']:.1f}" if math.isfinite(d_step['cost']) else '∞', str(len(d_step['visited'])), str(len(d_step['frontier'])), format_path(d_step)],
            ['A*', f"{a_step['cost']:.1f}" if math.isfinite(a_step['cost']) else '∞', str(len(a_step['visited'])), str(len(a_step['frontier'])), format_path(a_step)],
        ]

        table = ax_table.table(
            cellText=rows[1:],
            colLabels=rows[0],
            cellLoc='center',
            colLoc='center',
            loc='center',
            bbox=[0.08, 0.10, 0.84, 0.72],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9.2)
        table.scale(1, 1.28)

        for (row, col), cell in table.get_celld().items():
            cell.set_edgecolor(PALETTE['panel_edge'])
            if row == 0:
                cell.set_facecolor('#12243b')
                cell.get_text().set_color(PALETTE['text'])
                cell.get_text().set_weight('bold')
            elif row == 1:
                cell.set_facecolor('#0f2740')
                cell.get_text().set_color(PALETTE['text'])
            elif row == 2:
                cell.set_facecolor('#152f26')
                cell.get_text().set_color(PALETTE['text'])

        ax_table.text(
            0.5,
            0.03,
            'Green edges show the confirmed shortest path once the goal is reached.',
            transform=ax_table.transAxes,
            ha='center',
            va='bottom',
            color=PALETTE['muted'],
            fontsize=9.5,
        )

    status_text = ax_controls.text(
        0.03,
        0.5,
        'Playing continuously. Use Pause to stop or Reset to restart from the beginning.',
        transform=ax_controls.transAxes,
        ha='left',
        va='center',
        color=PALETTE['text'],
        fontsize=10.0,
        weight='bold',
    )

    def update(frame_index):
        current_frame['value'] = frame_index % frame_count
        d_state = dijkstra_timeline[min(current_frame['value'], len(dijkstra_timeline) - 1)]
        a_state = astar_timeline[min(current_frame['value'], len(astar_timeline) - 1)]
        d_step = dijkstra_steps[d_state['index']]
        a_step = astar_steps[a_state['index']]

        draw_panel(ax_dijkstra, graph_nx, pos, dijkstra_steps, d_state, start, goal, 'Dijkstra', '#7dd3fc')
        draw_panel(ax_astar, graph_nx, pos, astar_steps, a_state, start, goal, 'A*', '#f59e0b')
        d_done = d_step['done']
        a_done = a_step['done']
        d_cost = d_step['cost']
        a_cost = a_step['cost']

        build_summary_table(d_step, a_step)

        status = f"Start: {start}    Goal: {goal}    "
        status += f"Dijkstra cost: {d_cost:.1f}    " if math.isfinite(d_cost) else 'Dijkstra cost: ∞    '
        status += f"A* cost: {a_cost:.1f}"
        status_text.set_text(status)

        if d_done and a_done:
            status_text.set_text(status + '    Both paths completed and will loop continuously.')

    anim = FuncAnimation(fig, update, frames=range(frame_count), interval=1000 / fps, repeat=True)
    anim._draw_was_started = True
    fig._animation = anim
    fig._efficiency_fig = efficiency_fig

    def toggle_play_pause(_event):
        if is_running['value']:
            anim.event_source.stop()
            play_button.label.set_text('Play')
            is_running['value'] = False
            status_text.set_text('Paused. Press Play to continue the animation.')
        else:
            anim.event_source.start()
            play_button.label.set_text('Pause')
            is_running['value'] = True
            status_text.set_text('Playing continuously. Use Pause to stop or Reset to restart from the beginning.')
        fig.canvas.draw_idle()

    def reset_animation(_event):
        was_running = is_running['value']
        anim.event_source.stop()
        current_frame['value'] = 0
        update(0)
        fig.canvas.draw_idle()
        if was_running:
            anim.event_source.start()
            play_button.label.set_text('Pause')
            is_running['value'] = True
        else:
            play_button.label.set_text('Play')
            is_running['value'] = False

    play_button.on_clicked(toggle_play_pause)
    reset_button.on_clicked(reset_animation)

    build_summary_table(dijkstra_steps[0], astar_steps[0])
    update(0)
    anim.event_source.start()
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visually compare Dijkstra and A* with smooth animated path flow')
    parser.add_argument('--graph', '-g', default='resources/graph_example.json', help='Path to graph JSON file')
    parser.add_argument('--fps', type=int, default=30, help='Animation playback speed in frames per second')
    parser.add_argument('--step-frames', type=int, default=18, help='Frames used to animate each search step')
    parser.add_argument('--settle-frames', type=int, default=16, help='Frames to hold the final state')
    args = parser.parse_args()
    visualize(args.graph, fps=args.fps, step_frames=args.step_frames, settle_frames=args.settle_frames)


if __name__ == '__main__':
    main()
