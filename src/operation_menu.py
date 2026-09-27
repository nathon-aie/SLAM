"""Interactive operation selection; configuration stays in settings.yaml."""
from pathlib import Path
from .settings import project_path


def choose(title, options):
    print('\n' + title)
    for number, (label, _) in enumerate(options, 1):
        print('  {}. {}'.format(number, label))
    print('  0. ออก / ยกเลิก')
    while True:
        value = input('เลือกหมายเลข: ').strip()
        if value == '0':
            return None
        if value.isdigit() and 1 <= int(value) <= len(options):
            return options[int(value) - 1][1]
        print('กรุณาเลือกหมายเลข 0–{}'.format(len(options)))


def mode():
    return choose('เลือกการทำงาน', [('หุ่นจริง', False), ('จำลอง', True)])


def calibration_action():
    return choose('Calibration', [
        ('เก็บตัวอย่าง Sharp ซ้ายจากหุ่นจริง', 'sharp_left'),
        ('เก็บตัวอย่าง Sharp ขวาจากหุ่นจริง', 'sharp_right'),
        ('เก็บตัวอย่าง ToF จากหุ่นจริง', 'tof'),
        ('คำนวณสมการจากข้อมูลที่เก็บไว้', 'fit'),
    ])


def select_operation():
    """Return ordinary CLI arguments for the selected task, or None on cancellation."""
    try:
        task = choose('RoboMaster EP — เลือกงาน (ค่าพื้นฐานอ่านจาก config/settings.yaml)', [
            ('สำรวจและสร้างแผนที่ SLAM + DFS', 'explore'),
            ('ทดสอบเดินหน้า 1 ช่อง', 'step-test'),
            ('ทดสอบเลี้ยว', 'turn-test'),
            ('ดูข้อมูลเซนเซอร์สด', 'monitor'),
            ('ทดสอบชุดคำสั่งเคลื่อนที่', 'motion'),
            ('Calibration เซนเซอร์', 'calibrate'),
            ('วิเคราะห์ผลการสำรวจ / Log', 'analysis'),
        ])
        if task is None:
            return None
        if task in ('explore', 'step-test', 'turn-test', 'monitor', 'motion'):
            mock = mode()
            if mock is None:
                return None
            if task == 'motion':
                commands = input('คำสั่งเคลื่อนที่ เช่น fwd 1, right, fwd 1 (เว้นว่างเพื่อยกเลิก): ').strip()
                if not commands:
                    return None
                return ['simulate' if mock else 'run', '--commands', commands, '-y']
            arguments = [task]
            if mock:
                arguments.append('--mock')
            if task == 'turn-test':
                direction = choose('เลือกการเลี้ยว', [
                    ('ขวา 90 องศา', 'right'), ('ซ้าย 90 องศา', 'left'), ('กลับหลัง 180 องศา', 'around')])
                if direction is None:
                    return None
                arguments += ['--direction', direction]
            return arguments
        if task == 'calibrate':
            action = calibration_action()
            if action is None:
                return None
            if action == 'fit':
                return ['calibrate', 'fit', project_path('paths.measurements')]
            return ['calibrate', 'collect-live', action]
        action = choose('วิเคราะห์ผล', [
            ('วิเคราะห์ Log เซนเซอร์', 'analyze'),
            ('วัด Map Accuracy / Coverage เทียบ Ground Truth', 'evaluate-map'),
        ])
        if action is None:
            return None
        if action == 'evaluate-map':
            map_file = project_path('slam.output')
            truth = project_path('paths.ground_truth')
            for file in (map_file, truth):
                if not Path(file).is_file():
                    print('ยังไม่มีไฟล์สำหรับประเมิน: {}'.format(file))
                    return None
            return ['evaluate-map', map_file, truth]
        base = Path(project_path('paths.telemetry'))
        runs = [p for p in base.glob('run*') if p.is_dir() and any(p.glob('*.json'))]
        runs += list(base.glob('*.json'))
        runs.sort(key=lambda path: path.stat().st_mtime, reverse=True)
        if not runs:
            print('ยังไม่มี Log สำหรับวิเคราะห์ใน {}'.format(base))
            return None
        selected = choose('เลือก Log ที่ต้องการวิเคราะห์', [(p.name, str(p)) for p in runs])
        return ['analyze', selected] if selected else None
    except (EOFError, KeyboardInterrupt):
        print('\nออกจากเมนู')
        return None
