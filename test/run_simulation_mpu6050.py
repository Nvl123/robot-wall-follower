"""
=============================================================================
SIMULASI ROBOT WALL MAZE SOLVER DENGAN GYROSCOPE MPU-6050 (GY-521)
Berdasarkan Logika Robot_Wall_Follower_MPU6050.ino
Menghasilkan Video MP4 Berkualitas Tinggi (H.264, 1280x720, 30 FPS).
=============================================================================
"""
import os
import math
import numpy as np
import cv2
import imageio

# ==================== PARAMETER ARENA & ROBOT ====================
CELL = 0.40          # 40 cm per grid cell
COLS, ROWS = 10, 5   # 4.0 m x 2.0 m
W_ARENA = COLS * CELL
H_ARENA = ROWS * CELL

# Dimensi Robot (meter)
ROBOT_W = 0.16       # 16 cm
ROBOT_L = 0.16       # 16 cm
WHEELBASE = 0.14     # 14 cm

# Ambang Batas Sensor dari Robot_Wall_Follower_MPU6050.ino
TARGET_TENGAH  = 12.0  # cm
AMBANG_DEKAT   = 11.0  # cm
AMBANG_JAUH    = 13.0  # cm
AMBANG_TERBUKA = 22.0  # cm
DEPAN_STOP     = 15.0  # cm (ruang henti aman)

# Kecepatan Motor
SPEED_NORMAL     = 80
SPEED_SLOW       = 70
SPEED_FAST       = 85
SPEED_TURN_FAST  = 95
SPEED_TURN_SLOW  = 75
SPEED_BACK       = 70

# Start & Finish (meter)
START_X = 4.5 * CELL   # 1.80 m (Start 1)
START_Y = 0.5 * CELL   # 0.20 m
START_THETA = math.pi / 2  # Menghadap ke bawah (South, row 1)

FINISH_X_MIN = 3.0 * CELL  # 1.20 m
FINISH_X_MAX = 4.0 * CELL  # 1.60 m
FINISH_Y     = 1.85        # meter

# Dinding Horizontal: (y, x0, x1)
h_walls = [
    (0 * CELL, 0 * CELL, 10 * CELL),
    (5 * CELL, 0 * CELL, 3 * CELL),
    (5 * CELL, 4 * CELL, 6 * CELL),
    (5 * CELL, 7 * CELL, 10 * CELL),
    (2 * CELL, 1 * CELL, 4 * CELL),
    (1 * CELL, 2 * CELL, 8 * CELL),
    (3 * CELL, 3 * CELL, 7 * CELL),
    (4 * CELL, 1 * CELL, 3 * CELL),
    (4 * CELL, 7 * CELL, 9 * CELL),
    (2 * CELL, 6 * CELL, 9 * CELL),
]

# Dinding Vertikal: (x, y0, y1)
v_walls = [
    (0 * CELL, 0 * CELL, 5 * CELL),
    (10 * CELL, 0 * CELL, 5 * CELL),
    (1 * CELL, 0 * CELL, 2 * CELL),
    (5 * CELL, 0 * CELL, 5 * CELL),
    (2 * CELL, 2 * CELL, 3 * CELL),
    (3 * CELL, 3 * CELL, 4 * CELL),
    (7 * CELL, 3 * CELL, 4 * CELL),
    (9 * CELL, 0 * CELL, 2 * CELL),
    (8 * CELL, 2 * CELL, 3 * CELL),
]

wall_segments = []
for y, x0, x1 in h_walls:
    wall_segments.append((x0, y, x1, y))
for x, y0, y1 in v_walls:
    wall_segments.append((x, y0, x, y1))


def ray_seg_intersect(ox, oy, dx, dy, x1, y1, x2, y2):
    v1x = ox - x1
    v1y = oy - y1
    v2x = x2 - x1
    v2y = y2 - y1
    v3x = -dy
    v3y = dx
    dot = v2x * v3x + v2y * v3y
    if abs(dot) < 1e-9:
        return None
    t1 = (v2x * v1y - v2y * v1x) / dot
    t2 = (v1x * v3x + v1y * v3y) / dot
    if t1 >= 0 and 0.0 <= t2 <= 1.0:
        return t1
    return None


def get_sensor_distance(rx, ry, theta, sensor_type):
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    if sensor_type == 'F':
        ox = rx + (ROBOT_L / 2) * cos_t
        oy = ry + (ROBOT_L / 2) * sin_t
        dx = cos_t
        dy = sin_t
    elif sensor_type == 'R':
        ox = rx + (ROBOT_W / 2) * (-sin_t)
        oy = ry + (ROBOT_W / 2) * cos_t
        dx = -sin_t
        dy = cos_t
    elif sensor_type == 'L':
        ox = rx + (ROBOT_W / 2) * sin_t
        oy = ry + (ROBOT_W / 2) * (-cos_t)
        dx = sin_t
        dy = -cos_t

    min_dist = 2.0
    hit_pt = (ox + dx * min_dist, oy + dy * min_dist)

    for x1, y1, x2, y2 in wall_segments:
        d = ray_seg_intersect(ox, oy, dx, dy, x1, y1, x2, y2)
        if d is not None and d < min_dist:
            min_dist = d
            hit_pt = (ox + dx * d, oy + dy * d)

    return min_dist * 100.0, (ox, oy), hit_pt


def generate_video_mpu6050():
    print("Memulai simulasi & video rendering untuk edisi MPU-6050...")
    WIDTH, HEIGHT = 1280, 720
    FPS = 30
    DT = 1.0 / FPS

    VIEW_LEFT = 40
    VIEW_TOP = 135
    VIEW_W = 1200
    VIEW_H = 550

    SCALE_X = VIEW_W / W_ARENA
    SCALE_Y = VIEW_H / H_ARENA
    SCALE = min(SCALE_X, SCALE_Y)

    OFF_X = VIEW_LEFT + (VIEW_W - W_ARENA * SCALE) / 2
    OFF_Y = VIEW_TOP + (VIEW_H - H_ARENA * SCALE) / 2

    def to_screen(gx, gy):
        return int(OFF_X + gx * SCALE), int(OFF_Y + gy * SCALE)

    rx = START_X
    ry = START_Y
    theta = START_THETA

    # Variabel simulasi Gyroscope MPU-6050
    gyro_relative_yaw = 0.0  # Sudut relatif saat manuver belok
    gyro_total_yaw = math.degrees(theta)
    gyro_target_deg = 0.0
    turn_direction = 0  # +1: Kanan (CW), -1: Kiri (CCW)

    state = 'FOLLOW'
    state_timer = 0.0
    action_text = "Mulai Bergerak (Gyro Heading Lock)"
    action_color = (255, 255, 255)

    pwm_l, pwm_r = SPEED_NORMAL, SPEED_NORMAL
    trajectory = []

    video_filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), "simulasi_maze_wall_follower_mpu6050.mp4")
    writer = imageio.get_writer(video_filename, fps=FPS, codec='libx264', quality=8)

    step = 0
    max_steps = 30 * 95
    finish_hold_counter = 0

    while step < max_steps:
        step += 1
        trajectory.append((rx, ry))

        # Baca 3 Sensor Ultrasonik
        cmF, ptF_orig, ptF_hit = get_sensor_distance(rx, ry, theta, 'F')
        cmRF, ptR_orig, ptR_hit = get_sensor_distance(rx, ry, theta, 'R')
        cmLF, ptL_orig, ptL_hit = get_sensor_distance(rx, ry, theta, 'L')

        # Cek Titik Finish
        if rx >= FINISH_X_MIN and rx <= FINISH_X_MAX and ry >= FINISH_Y:
            state = 'FINISH'
            action_text = "FINISH TERCAPAI!"
            action_color = (50, 255, 50)
            pwm_l, pwm_r = 0, 0
            finish_hold_counter += 1

        # ================= LOGIKA MPU-6050 GYROSCOPE =================
        if state == 'FOLLOW':
            if cmF <= DEPAN_STOP:
                state = 'OBSTACLE_BACK'
                state_timer = 0.15
                action_text = "Depan Terhalang! Mundur sejenak..."
                action_color = (50, 50, 255)
            else:
                # Penelusuran Dinding Kanan dengan Heading Lock
                if cmRF >= AMBANG_DEKAT and cmRF <= AMBANG_JAUH:
                    pwm_l, pwm_r = SPEED_NORMAL, SPEED_NORMAL
                    action_text = "Maju Lurus (Heading Lock Aktif)"
                    action_color = (255, 255, 255)
                elif cmRF > AMBANG_JAUH:
                    if cmRF >= AMBANG_TERBUKA:
                        if cmLF >= AMBANG_DEKAT and cmLF <= AMBANG_JAUH:
                            pwm_l, pwm_r = SPEED_NORMAL, SPEED_NORMAL
                            action_text = "Maju Lurus (Pemandu Dinding Kiri)"
                            action_color = (200, 230, 255)
                        elif cmLF > AMBANG_JAUH and cmLF < AMBANG_TERBUKA:
                            pwm_l, pwm_r = SPEED_SLOW, SPEED_FAST
                            action_text = "Koreksi Kiri Halus"
                            action_color = (180, 220, 255)
                        elif cmLF < AMBANG_DEKAT:
                            pwm_l, pwm_r = SPEED_FAST, SPEED_SLOW
                            action_text = "Koreksi Kanan Halus"
                            action_color = (255, 200, 180)
                        else:
                            pwm_l, pwm_r = SPEED_NORMAL, SPEED_NORMAL
                            action_text = "Maju Lurus (Simpang Terbuka)"
                            action_color = (220, 220, 220)
                    else:
                        pwm_l, pwm_r = SPEED_FAST, SPEED_SLOW
                        action_text = "Koreksi Kanan Halus"
                        action_color = (255, 220, 150)
                elif cmRF < AMBANG_DEKAT:
                    pwm_l, pwm_r = SPEED_SLOW, SPEED_FAST
                    action_text = "Koreksi Kiri Halus (Menjauhi Kanan)"
                    action_color = (150, 220, 255)

                v = ((pwm_l + pwm_r) / 2.0 / 255.0) * 0.35
                omega = ((pwm_l - pwm_r) / 255.0) * 1.5
                rx += v * math.cos(theta) * DT
                ry += v * math.sin(theta) * DT
                theta += omega * DT
                gyro_total_yaw = math.degrees(theta)

        elif state == 'OBSTACLE_BACK':
            pwm_l, pwm_r = -SPEED_BACK, -SPEED_BACK
            v = -0.15
            rx += v * math.cos(theta) * DT
            ry += v * math.sin(theta) * DT
            state_timer -= DT
            if state_timer <= 0:
                state = 'OBSTACLE_STOP'
                state_timer = 0.15

        elif state == 'OBSTACLE_STOP':
            pwm_l, pwm_r = 0, 0
            state_timer -= DT
            if state_timer <= 0:
                kananBebas = (cmRF > AMBANG_TERBUKA)
                kiriBebas  = (cmLF > AMBANG_TERBUKA)

                if not kiriBebas and kananBebas:
                    turn_direction = 1  # Kanan 90°
                    gyro_target_deg = 90.0
                    action_text = "Keputusan: Belok Kanan tepat 90.0° via Gyro"
                    action_color = (220, 80, 255)
                elif kiriBebas and not kananBebas:
                    turn_direction = -1  # Kiri 90°
                    gyro_target_deg = 90.0
                    action_text = "Keputusan: Belok Kiri tepat 90.0° via Gyro"
                    action_color = (80, 220, 255)
                elif kiriBebas and kananBebas:
                    turn_direction = 1  # Simpang T: Belok Kanan 90°
                    gyro_target_deg = 90.0
                    action_text = "Simpang T: Belok Kanan tepat 90.0° via Gyro"
                    action_color = (255, 200, 50)
                else:
                    turn_direction = 1  # Buntu: Putar Balik 180°
                    gyro_target_deg = 180.0
                    action_text = "JALAN BUNTU! Putar Balik 180.0° via Gyro"
                    action_color = (255, 80, 80)

                state = 'OBSTACLE_TURN'
                gyro_relative_yaw = 0.0

        elif state == 'OBSTACLE_TURN':
            # Kontrol kecepatan belok cerdas: deselerasi di 15 derajat terakhir
            sisa_sudut = gyro_target_deg - gyro_relative_yaw
            current_turn_speed = SPEED_TURN_SLOW if sisa_sudut <= 15.0 else SPEED_TURN_FAST

            turn_rate_rad = (current_turn_speed / 95.0) * 3.0
            d_deg = math.degrees(turn_rate_rad * DT)

            if gyro_relative_yaw + d_deg >= gyro_target_deg:
                d_deg = gyro_target_deg - gyro_relative_yaw
                gyro_relative_yaw = gyro_target_deg
                theta += turn_direction * math.radians(d_deg)
                state = 'CLEAR_TURN'
                state_timer = 0.20
                action_text = f"Selesai Belok Gyro: {gyro_target_deg:.1f}° Tercapai Tepat!"
                action_color = (100, 255, 100)
            else:
                gyro_relative_yaw += d_deg
                theta += turn_direction * math.radians(d_deg)
                if turn_direction > 0:
                    pwm_l, pwm_r = current_turn_speed, -current_turn_speed
                else:
                    pwm_l, pwm_r = -current_turn_speed, current_turn_speed

            gyro_total_yaw = math.degrees(theta)

        elif state == 'CLEAR_TURN':
            pwm_l, pwm_r = SPEED_NORMAL, SPEED_NORMAL
            v = 0.20
            rx += v * math.cos(theta) * DT
            ry += v * math.sin(theta) * DT
            state_timer -= DT
            if state_timer <= 0:
                state = 'FOLLOW'

        elif state == 'FINISH':
            pwm_l, pwm_r = 0, 0
            if finish_hold_counter >= 90:
                break

        # ================= RENDER FRAME GRAFIS =================
        frame = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 242

        # Latar Arena
        cv2.rectangle(frame, to_screen(0, 0), to_screen(W_ARENA, H_ARENA), (255, 255, 255), -1)

        # Garis Grid
        for c in range(COLS + 1):
            cv2.line(frame, to_screen(c * CELL, 0), to_screen(c * CELL, H_ARENA), (232, 232, 232), 1)
        for r in range(ROWS + 1):
            cv2.line(frame, to_screen(0, r * CELL), to_screen(W_ARENA, r * CELL), (232, 232, 232), 1)

        # Zona START & FINISH
        s1 = to_screen(4.0 * CELL, 0)
        s2 = to_screen(5.0 * CELL, 0.4 * CELL)
        cv2.rectangle(frame, s1, s2, (200, 245, 200), -1)
        cv2.putText(frame, "START 1", (s1[0] + 18, s1[1] + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 0), 2)

        f1 = to_screen(3.0 * CELL, 4.6 * CELL)
        f2 = to_screen(4.0 * CELL, 5.0 * CELL)
        cv2.rectangle(frame, f1, f2, (200, 200, 255), -1)
        cv2.putText(frame, "FINISH 1", (f1[0] + 16, f2[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 200), 2)

        # Trajectory Trail
        if len(trajectory) > 1:
            pts = np.array([to_screen(tx, ty) for tx, ty in trajectory], dtype=np.int32)
            cv2.polylines(frame, [pts], False, (220, 130, 40), 2, cv2.LINE_AA)

        # Dinding Arena
        for x1, y1, x2, y2 in wall_segments:
            cv2.line(frame, to_screen(x1, y1), to_screen(x2, y2), (35, 35, 35), 6, cv2.LINE_AA)

        # Sinar Ultrasonik
        col_f_beam = (0, 0, 240) if cmF <= DEPAN_STOP else (0, 200, 0)
        cv2.line(frame, to_screen(*ptF_orig), to_screen(*ptF_hit), col_f_beam, 2, cv2.LINE_AA)
        cv2.circle(frame, to_screen(*ptF_hit), 4, col_f_beam, -1)

        cv2.line(frame, to_screen(*ptR_orig), to_screen(*ptR_hit), (200, 0, 200), 2, cv2.LINE_AA)
        cv2.circle(frame, to_screen(*ptR_hit), 4, (200, 0, 200), -1)

        cv2.line(frame, to_screen(*ptL_orig), to_screen(*ptL_hit), (200, 160, 0), 2, cv2.LINE_AA)
        cv2.circle(frame, to_screen(*ptL_hit), 4, (200, 160, 0), -1)

        # Bodi Robot
        c_t = math.cos(theta)
        s_t = math.sin(theta)
        hw = ROBOT_W / 2
        hl = ROBOT_L / 2

        corners = [
            (rx + hl * c_t - hw * (-s_t), ry + hl * s_t - hw * c_t),
            (rx - hl * c_t - hw * (-s_t), ry - hl * s_t - hw * c_t),
            (rx - hl * c_t + hw * (-s_t), ry - hl * s_t + hw * c_t),
            (rx + hl * c_t + hw * (-s_t), ry + hl * s_t + hw * c_t),
        ]
        poly_pts = np.array([to_screen(cx, cy) for cx, cy in corners], dtype=np.int32)
        cv2.fillPoly(frame, [poly_pts], (35, 155, 235))
        cv2.polylines(frame, [poly_pts], True, (20, 80, 160), 2, cv2.LINE_AA)

        # Simbol Gyroscope MPU-6050 di tengah bodi robot
        c_pt = to_screen(rx, ry)
        cv2.circle(frame, c_pt, 7, (20, 20, 20), -1)
        cv2.circle(frame, c_pt, 4, (0, 230, 255), -1)

        # Panah Heading
        pt_front = to_screen(rx + hl * c_t, ry + hl * s_t)
        cv2.arrowedLine(frame, c_pt, pt_front, (255, 255, 255), 2, tipLength=0.35)

        # ================= TOP HUD & DASHBOARD GYRO =================
        cv2.rectangle(frame, (0, 0), (WIDTH, 125), (20, 25, 32), -1)

        cv2.putText(frame, "WALL MAZE SOLVER ROBOT - EDISI GYROSCOPE MPU-6050 (GY-521)", (25, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, (255, 255, 255), 2)
        cv2.putText(frame, "Feedback Sudut Derajat Riil (Closed-Loop) | Heading Lock Maju Lurus | Anti-Overshoot", (25, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 215, 240), 1)

        # Kartu Sensor Ultrasonik
        cv2.rectangle(frame, (25, 64), (150, 115), (38, 45, 55), -1)
        cv2.putText(frame, "KIRI (90°)", (35, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (190, 200, 210), 1)
        cv2.putText(frame, f"{cmLF:.1f} cm", (35, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (230, 180, 40), 2)

        cv2.rectangle(frame, (160, 64), (285, 115), (38, 45, 55), -1)
        cv2.putText(frame, "DEPAN", (170, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (190, 200, 210), 1)
        cv2.putText(frame, f"{cmF:.1f} cm", (170, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.58, col_f_beam, 2)

        cv2.rectangle(frame, (295, 64), (420, 115), (38, 45, 55), -1)
        cv2.putText(frame, "KANAN (90°)", (305, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (190, 200, 210), 1)
        cv2.putText(frame, f"{cmRF:.1f} cm", (305, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (220, 80, 220), 2)

        # Kartu KHUSUS GYROSCOPE MPU-6050
        cv2.rectangle(frame, (430, 64), (580, 115), (30, 50, 65), -1)
        cv2.putText(frame, "MPU-6050 YAW", (440, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 230, 255), 1)
        if state == 'OBSTACLE_TURN':
            cv2.putText(frame, f"{gyro_relative_yaw:.1f}° / {gyro_target_deg:.0f}°", (440, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 200), 2)
        else:
            cv2.putText(frame, f"{gyro_total_yaw % 360:.1f}°", (440, 105),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 230, 255), 2)

        # Kartu Status Aksi
        cv2.rectangle(frame, (590, 64), (960, 115), (32, 38, 48), -1)
        cv2.putText(frame, "STATUS KONTROL GERAK & GYRO:", (600, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 175, 190), 1)
        cv2.putText(frame, action_text, (600, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.44, action_color, 2)

        # Telemetri Waktu & PWM
        time_sec = step * DT
        cv2.putText(frame, f"Waktu: {time_sec:.1f} s", (980, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 215, 230), 1)
        cv2.putText(frame, f"PWM: L={pwm_l} R={pwm_r}", (980, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 205, 220), 1)
        cv2.putText(frame, f"Pos: ({rx:.2f}m, {ry:.2f}m)", (1120, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 195, 210), 1)
        cv2.putText(frame, "Gyro: LOCKED", (1120, 105), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 230, 100), 1)

        # Spanduk SELESAI jika Finish
        if state == 'FINISH':
            b_w, b_h = 600, 125
            bx1, by1 = (WIDTH - b_w) // 2, (HEIGHT - b_h) // 2
            bx2, by2 = bx1 + b_w, by1 + b_h
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 140, 0), -1)
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), (255, 255, 255), 3)
            cv2.putText(frame, "FINISH TERCAPAI (GYRO PRESISI)!", (bx1 + 35, by1 + 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 3)
            cv2.putText(frame, f"Waktu: {time_sec:.2f} detik | Belokan 90° & 180° Presisi 100%", (bx1 + 55, by1 + 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 255, 230), 2)

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        writer.append_data(frame_rgb)

    writer.close()
    print(f"Video simulasi MPU-6050 berhasil disimpan: {video_filename}")
    print(f"Total durasi simulasi: {step * DT:.2f} detik ({step} frame).")
    return video_filename


if __name__ == '__main__':
    generate_video_mpu6050()
