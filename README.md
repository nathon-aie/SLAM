# RoboMaster EP — Grid SLAM และ DFS

หุ่นสำรวจแผนที่เองโดยใช้ ToF บน Gimbal วัดกำแพง ใช้ DFS เลือกทางใหม่และย้อนกลับเมื่อไม่มีทางไปต่อ Sharp ซ้าย–ขวาใช้ PID Centering ระหว่างเดิน ส่วน ToF ตรวจด้านหน้าเพื่อชะลอและหยุดก่อนชน ใช้ odometry และ yaw ประเมินตำแหน่งร่วมกับกำแพงที่เคยวัด

โครงการนี้ใช้หุ่นจริง ไม่มีระบบจำลองหรือการเปลี่ยนไปใช้ข้อมูลจำลองเมื่อเชื่อมต่อไม่สำเร็จ

## ติดตั้งและรัน

รองรับ Python 3.8 และ RoboMaster EP ที่เชื่อมต่อผ่าน AP หรือ STA:

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python main.py
```

เมนู:

1. สำรวจและสร้างแผนที่ SLAM + DFS
2. ทดสอบเดินหน้า 1 ช่อง
3. ทดสอบเลี้ยว
4. ดูข้อมูลเซนเซอร์สด
5. ทดสอบชุดคำสั่งเคลื่อนที่
6. Calibration เซนเซอร์
7. วิเคราะห์ผลการสำรวจ / Log
8. ทดสอบเฉพาะ Gimbal

เลือก 0 เพื่อออก ค่าพื้นฐานแก้ใน `config/settings.yaml` เมนูไม่มีการแก้ config

## Workflow การสำรวจ

1. เริ่มจากกลางช่อง `map.start` หันหน้าตามทิศเริ่มต้น ระบบตั้งตำแหน่งและ yaw เริ่มต้นเป็นศูนย์
2. หยุดล้อก่อนสแกน โดยไม่มีคำสั่งหมุน chassis แทรกระหว่าง Gimbal สแกน
3. รอบแรก: **recenter → อ่านหน้า → ซ้าย −90° → หลัง −180° → ขวา +90° → recenter** จากหลังไปขวาส่ง `move` เดียว ไม่แวะหยุดที่ซ้าย
4. รอบต่อไป: **recenter → อ่านหน้า → ซ้าย −90° → ขวา +90° → recenter** จากซ้ายไปขวาส่ง `move(yaw=180)` เดียว ด้านหลังใช้ทางที่เดินผ่านแล้วเป็นหลักฐานว่าเปิด
5. รอ action จบและพักตาม `gimbal.settle_sec` แล้วเก็บ ToF สดหลาย packet ใช้ median ไม่ปรับแก้มุมจาก feedback
6. อัปเดตกำแพง/ทางเปิด และใช้กำแพงเดิมช่วยปรับตำแหน่ง DFS เลือกช่องที่ยังไม่สำรวจภายในขอบเขตแผนที่
7. ตั้ง Gimbal ด้านหน้าด้วย `moveto` ก่อนหุ่นเคลื่อน เลี้ยวตามทิศทาง รอ ToF ใหม่ แล้วเดินทีละช่องด้วย Sharp Centering และ PID รักษา yaw
8. เมื่อถึงช่องหรือหยุดที่กำแพงตามเงื่อนไขการมาถึง หยุดล้อและสแกนต่อ สำรวจเสร็จเมื่อสำรวจทุกช่องที่เข้าถึงได้และกลับจุดเริ่ม

ระบบใช้ `CHASSIS_LEAD` ตลอด ไม่สลับโหมดระหว่างสแกน และไม่ใช้ `suspend()` เพื่อล็อค Gimbal

## ค่าตั้งต้น

| กลุ่มใน settings.yaml | ใช้ตั้งค่า |
| --- | --- |
| `robot` | การเชื่อมต่อและเครื่องหมายคำสั่ง yaw |
| `map` | `rows`, `columns`, `start.x`, `start.y` |
| `gimbal` | ความเร็วสแกน/recenter, เวลาพัก, จำนวนรอบทดสอบ |
| `navigation`, `pid` | ขนาดช่อง ความเร็ว Centering และระยะกันชน |
| `sensors` | พอร์ต Sharp, ToF, อัตรารับข้อมูลและตัวกรอง |
| `slam`, `scan_guard` | การสร้างแผนที่ ขีดจำกัดภารกิจและความสดของข้อมูล |
| `paths`, `calibration`, `telemetry`, `system` | ไฟล์ สมการ บัฟเฟอร์และเวลาระบบ |

`x` คือแถวตามแกนหน้าเริ่มต้น และ `y` คือคอลัมน์ตามแกนขวาเริ่มต้น ตัวอย่าง rows=4, columns=3 มี 12 ช่อง: x=0–3 และ y=0–2 ขอบเขตแผนที่ไม่ใช่ข้อมูลกำแพงล่วงหน้า หุ่นจะไม่เดินออกนอกขอบเขตแม้ ToF เห็นทางเปิด

ความเร็ว recenter แยกจากความเร็วสแกน:

```yaml
gimbal:
  recenter_yaw_speed_dps: 300
  recenter_pitch_speed_dps: 120
  test_cycles: 2
```

## ทดสอบ Gimbal และ Calibration

เมนู 8 หยุดล้อแล้วสแกนตาม workflow จริง โดยไม่เริ่มชุดคำสั่งเดิน เริ่มต้นทดสอบ 2 รอบ กด Ctrl+C เพื่อหยุด แสดง ToF บนหน้าจออย่างเดียว ไม่บันทึกไฟล์หรือสร้างโฟลเดอร์ run

เมนู Calibration เก็บค่า Sharp ซ้าย/ขวา หรือ ToF จากระยะอ้างอิง พิมพ์ `q` ในช่องรับระยะเพื่อกลับเมนู Calibration โดยยังเชื่อมต่อหุ่นอยู่ เลือก 0 เพื่อออกและปิด connection ข้อมูลที่เก็บแล้วใช้คำนวณสมการจากเมนูเดียวกัน

## ผลลัพธ์การสำรวจ

แต่ละรอบบันทึกใน `telemetry_logs/runN/`:

- `map.png`: แผนที่และหมายเลขก้าวในแต่ละช่อง
- `actions.html`: รายการก้าวและ action เปิดดูใน browser ได้
- `runN_TIMESTAMP_map.json`: แผนที่ เหตุการณ์ และรายละเอียดภารกิจ
- `runN_TIMESTAMP.json`, `_plot.png`: ข้อมูลเซนเซอร์และกราฟ

ข้อมูลแต่ละรอบเก็บในโฟลเดอร์ run ที่เดียว ไม่สร้างสำเนาแผนที่ใน `data/` หรือ CSV ซ้ำกับ JSON เมื่อหยุดหรือเกิดข้อผิดพลาดจะบันทึกแผนที่บางส่วน สถานะ `completed` หมายถึงครบช่องที่เข้าถึงได้ ไม่รวมช่องปิดที่หุ่นเข้าไม่ได้

เมนู 7 ใช้วิเคราะห์ telemetry ที่บันทึกไว้

## คำสั่ง CLI

ใช้งานผ่านเมนูได้ทั้งหมด หรือระบุงานโดยตรง:

```bash
python main.py explore
python main.py gimbal-test
python main.py step-test --cells 1
python main.py turn-test --direction right
python main.py monitor
python main.py run --commands 'fwd 1, right, fwd 1' -y
python main.py analyze telemetry_logs/run1
```

## โครงสร้างโค้ด

- `main.py`, `src/operation_menu.py`: เลือกงานและเริ่มระบบ
- `src/robot_system.py`: เชื่อมต่อหุ่นและจัดการ worker
- `src/sensor_pipeline.py`: รับเซนเซอร์ แปลงหน่วย และเก็บสถานะร่วม
- `src/sensor_filters.py`: ตัวกรอง Median, Moving Average, EMA และ Outlier
- `src/robot_controller.py`, `src/pid_controller.py`: เดิน เลี้ยว Centering และกันชนด้านหน้า
- `src/slam_hardware.py`: สแกน Gimbal และเชื่อมการเดินกับ SLAM
- `src/grid_slam.py`: แผนที่ การประมาณตำแหน่ง และ DFS
- `src/telemetry.py`, `src/slam_report.py`: บันทึกและแสดงผล
- `src/calibrate.py`: เก็บข้อมูลและคำนวณสมการเซนเซอร์
- `src/sdk_connection.py`: โหลด SDK การเชื่อมต่อ และความเข้ากันได้กับ SDK โดยไม่แก้ `.venv`

## ขอบเขตและการตรวจงาน

เป็น Grid-constrained SLAM สำหรับกำแพงตามแนวช่องและสนามคงที่ ไม่ได้ทำ pose graph หรือ relocalization แบบอิสระ ความคลาด odometry และการมองเห็นกำแพงมีผลต่อความแม่นยำ

ไม่มีโฟลเดอร์ tests ในโครงการนี้ การยืนยันพฤติกรรมการเดิน สแกน Centering และกันชนใช้การรันหุ่นจริงและอ่าน telemetry เริ่มตรวจด้วยเมนู Gimbal เดินหนึ่งช่อง และเลี้ยว ก่อนสำรวจทั้งสนาม
