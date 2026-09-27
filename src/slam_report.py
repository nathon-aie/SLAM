"""Post-run map/trajectory rendering and offline ground-truth evaluation.

Ground truth is used only by this evaluator, never by the exploration backend.
"""
import csv
import json
import shutil
from html import escape
import textwrap
from pathlib import Path
from .grid_slam import GridMap, NAMES


def action_steps(data):
    events = data['events']
    marked = any(e['type'] == 'motion_start' for e in events)
    boundaries, last_scan = [(0, None)], -1
    for i, e in enumerate(events):
        if e['type'] == 'scan':
            last_scan = i
        if e['type'] == ('motion_start' if marked else 'move'):
            boundaries.append((i if marked else last_scan + 1, e))
    steps, heading = [], 0
    for n, (begin, motion) in enumerate(boundaries):
        end = boundaries[n + 1][0] if n + 1 < len(boundaries) else len(events)
        actions = []
        if motion:
            delta = (motion['direction'] - heading) % 4
            if delta and not any(e['type'] == 'chassis_turn' for e in events[begin:end]):
                actions.append('TURN (planned/inferred) ' + {1: 'right 90 deg', 2: 'around 180 deg', 3: 'left 90 deg'}[delta])
            heading = motion['direction']
        for e in events[begin:end]:
            kind = e['type']
            if kind == 'hardware_action' and e['name'] != 'Gimbal move action':
                actions.append('{} [{}]'.format(e['name'], 'OK' if e['succeeded'] else 'FAILED'))
            elif kind == 'gimbal' and e.get('phase') == 'move':
                actions.append('GIMBAL move {:+g} deg -> {:+g} deg'.format(e['delta_yaw'], e['target_yaw']))
            elif kind == 'tof_sample':
                actions.append('ToF {:+g} deg: {:.0f} mm'.format(e['gimbal_yaw'], e['range_m'] * 1000))
            elif kind == 'move':
                actions.append('ARRIVED {} -> {}{}'.format(tuple(e['from']), tuple(e['to']), ' BACKTRACK' if e['backtrack'] else ''))
            elif kind == 'chassis_turn':
                degrees = e['degrees']
                actions.append('TURN {} {} deg'.format('right' if degrees < 0 else 'around' if abs(degrees) == 180 else 'left', abs(degrees)))
            elif kind == 'walk_start':
                actions.append('WALK {} | nominal {:.2f} m | Sharp centering + ToF guard'.format(NAMES[e['direction']], e['cell_size_m']))
            elif kind == 'walk_end':
                actions.append('WALK stop: {} | actual {:.2f} m | {}'.format(e['reason'], e['distance_m'], 'OK' if e['completed'] else 'FAILED'))
            elif kind == 'chassis_wheel_stop':
                actions.append('STOP wheels [ACK]')
            elif kind == 'scan':
                actions.append('MAP cell {}'.format(tuple(e['cell'])))
                heading = round(e['pose'][2] / 90) % 4
            elif kind == 'finish':
                actions.append('END: {}{}'.format(e['status'], ' | ' + e['error'] if e.get('error') else ''))
            elif kind in ('range_mismatch', 'wall_mismatch', 'odometry_mismatch', 'scan_position_drift', 'scan_pose_wait', 'scan_pose_recovered'):
                actions.append('NOTE: ' + kind)
        steps.append({'completed': n == 0 or any(e['type'] == 'move' for e in events[begin:end]), 'step': n, 'from': motion['from'] if motion else data.get('start_cell', [0, 0]),
                      'to': motion['to'] if motion else data.get('start_cell', [0, 0]), 'actions': actions})
    return steps


def save_actions_html(data, output):
    """Standalone, searchable step list; no network or external assets required."""
    steps = action_steps(data)
    cards = []
    for step in steps:
        items = ''.join('<li>{}</li>'.format(escape(action)) for action in step['actions'])
        title = 'Step {:02d} · {} → {}'.format(step['step'], tuple(step['from']), tuple(step['to']))
        cards.append('<details class="step"><summary>{}</summary><ol>{}</ol></details>'.format(escape(title), items))
    header = '{} · สำรวจ {} ช่อง · เดินสำเร็จ {} ครั้ง'.format(data['status'], len(data['visited']),
        sum(e['type'] == 'move' for e in data['events']))
    page = """<!doctype html><html lang="th"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>รายการ Step และ Action</title><style>
body{margin:0;background:#f4f6f8;color:#203040;font:16px/1.65 system-ui,sans-serif}
main{max-width:1000px;margin:32px auto;padding:0 24px}h1{margin-bottom:0;font-size:28px}
.meta{color:#526373}nav{position:sticky;top:0;background:#f4f6f8;padding:12px 0;display:flex;gap:8px;flex-wrap:wrap}
input,button{font:inherit;border:1px solid #bbc9d2;border-radius:8px;padding:8px 12px;background:white}
input{flex:1;min-width:180px}button{cursor:pointer}details{background:white;border:1px solid #dbe3e8;border-radius:10px;margin:12px 0}
summary{padding:14px 18px;font-weight:650;color:#124c72;cursor:pointer}ol{padding:0 32px 16px 52px}
li{padding:6px;border-top:1px solid #eef1f4;overflow-wrap:anywhere}a{color:#126fa4}
@media print{nav{display:none}details{break-inside:avoid}body{background:white}}
</style><main><h1>รายการ Step / Action</h1><p class="meta">HEADER</p>
<p>Step 0 คือสแกนจุดเริ่ม แต่ละ Step ถัดไปคือการเดินหรือย้อนกลับหนึ่งครั้ง
พิกัด (x,y) คือแถวและคอลัมน์ของช่อง · <a href="map.png">เปิดแผนที่</a></p>
<p class="meta">TURN (planned/inferred) คือคำสั่งที่วางแผนหรืออนุมานจาก log เก่า
ARRIVED คือยืนยันถึงช่องแล้ว; FAILED คือคำสั่งไม่สำเร็จ</p>
<nav><input id="search" type="search" placeholder="ค้นหา step, พิกัด หรือ action" aria-label="ค้นหา">
<button onclick="toggleAll(true)">เปิดทั้งหมด</button><button onclick="toggleAll(false)">พับทั้งหมด</button></nav>
CARDS</main><script>
const steps=Array.from(document.querySelectorAll('.step'));
function toggleAll(open){steps.forEach(s=>{if(!s.hidden)s.open=open})}
document.getElementById('search').addEventListener('input',e=>{let q=e.target.value.trim().toLowerCase();steps.forEach(s=>{s.hidden=!s.textContent.toLowerCase().includes(q);if(q&&!s.hidden)s.open=true})});
if(steps.length)steps[0].open=true;
</script></html>"""
    Path(output).write_text(page.replace('HEADER', escape(header)).replace('CARDS', ''.join(cards)), encoding='utf-8')
    return Path(output)


def save_run_report(map_file, run_dir, basename):
    folder = Path(run_dir)
    folder.mkdir(parents=True, exist_ok=True)
    archive = folder / (basename + '_map.json')
    if Path(map_file).resolve() != archive.resolve():
        shutil.copyfile(str(map_file), str(archive))
    plot, log = save_report(archive)
    final_plot = folder / 'map.png'
    plot.replace(final_plot)
    path_actions = archive.with_name(archive.stem + '_actions.html')
    path_actions.replace(folder / 'actions.html')
    final_log = folder / 'events.csv'
    log.replace(final_log)
    return archive, final_plot, final_log


def save_report(map_file):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    path = Path(map_file)
    data = json.loads(path.read_text(encoding='utf-8'))
    size = data['cell_size_m']
    steps = action_steps(data)
    figure, axis = plt.subplots(figsize=(9, 9))
    for x, y in data['visited']:
        axis.add_patch(plt.Rectangle(((y - 0.5) * size, (x - 0.5) * size), size, size,
                                     facecolor='#e4f1f8', edgecolor='#b6c5d0', linewidth=0.5))
    for edge in data['edges']:
        if not edge['wall']:
            continue
        (x1, y1), (x2, y2) = edge['cells']
        x, y = (x1 + x2) * size / 2, (y1 + y2) * size / 2
        if x1 != x2:
            axis.plot([y - size / 2, y + size / 2], [x, x], color='#263747', linewidth=3)
        else:
            axis.plot([y, y], [x - size / 2, x + size / 2], color='#263747', linewidth=3)
    trajectory = data['trajectory']
    if trajectory:
        axis.plot([p['pose'][1] for p in trajectory], [p['pose'][0] for p in trajectory],
                  'o-', color='#1689c1', markersize=3, linewidth=1, label='Estimated trajectory')
    start = data.get('start_pose', [0, 0, 0])
    axis.scatter([start[1]], [start[0]], marker='s', color='#26a269', s=90,
                 label='Start {}'.format(tuple(data.get('start_cell', [0, 0]))), zorder=5)
    axis.scatter([data['pose'][1]], [data['pose'][0]], marker='x', color='#e33b35', s=90,
                 label='Last estimated pose', zorder=6)
    if 'map_info' in data:
        axis.set_xlim(-size / 2, (data['map_info']['columns'] - 0.5) * size)
        axis.set_ylim(-size / 2, (data['map_info']['rows'] - 0.5) * size)
    labels = {}
    for step in steps:
        if not step['completed']:
            continue
        labels.setdefault(tuple(step['to']), []).append(str(step['step']))
    for (x, y), numbers in labels.items():
        axis.text(y * size, x * size, '\n'.join(textwrap.wrap(','.join(numbers), width=12)),
                  ha='center', va='center', fontsize=8, color='#124c72', weight='bold')
    axis.set_aspect('equal')
    axis.set_xlabel('Initial right axis y (m)')
    axis.set_ylabel('Initial forward axis x (m)')
    axis.set_title('Explored map and trajectory | {} | {} cells'.format(data['status'], len(data['visited'])))
    axis.legend(loc='best')
    axis.grid(alpha=0.15)

    figure.tight_layout()
    save_actions_html(data, path.with_name(path.stem + '_actions.html'))
    plot = path.with_name(path.stem + '_map.png')
    figure.savefig(str(plot), dpi=160)
    plt.close(figure)
    log = path.with_name(path.stem + '_events.csv')
    with log.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream)
        writer.writerow(['event_index', 'timestamp', 'type', 'details_json'])
        for index, event in enumerate(data['events']):
            writer.writerow([index, event['timestamp'], event['type'], json.dumps(event, ensure_ascii=False)])
    return plot, log


def evaluate(map_file, truth_file):
    """Truth: cells=[{cell:[x,y], walls:{N:bool,E:bool,S:bool,W:bool}}].

A cell is correct only if all four walls are known and match. Unvisited/unknown
cells count as incorrect; denominators include every ground-truth cell.
"""
    data = json.loads(Path(map_file).read_text(encoding='utf-8'))
    truth = json.loads(Path(truth_file).read_text(encoding='utf-8'))
    grid = GridMap()
    for edge in data['edges']:
        grid.edges[tuple(sorted(tuple(c) for c in edge['cells']))] = edge['wall']
    visited = {tuple(c) for c in data['visited']}
    cells = {}
    for item in truth['cells']:
        cell = tuple(item['cell'])
        if len(cell) != 2 or any(type(v) is not int for v in cell) or cell in cells:
            raise ValueError('Ground truth cells must be unique integer coordinates')
        walls = item['walls']
        if set(walls) != set(NAMES) or any(type(v) is not bool for v in walls.values()):
            raise ValueError('Each truth cell must have N/E/S/W boolean walls')
        cells[cell] = walls
    if not cells:
        raise ValueError('Ground truth must contain at least one cell')
    correct = sum(cell in visited and all(grid.wall(cell, d) == walls[name] for d, name in enumerate(NAMES))
                  for cell, walls in cells.items())
    covered = len(visited.intersection(cells))
    return {'total_cells': len(cells), 'correct_cells': correct, 'covered_cells': covered,
            'map_accuracy_percent': correct / len(cells) * 100,
            'coverage_percent': covered / len(cells) * 100,
            'unexpected_cells': [list(c) for c in sorted(visited.difference(cells))]}
