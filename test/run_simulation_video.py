"""
=============================================================================
SIMULASI ROBOT WALL MAZE SOLVER (ARENA 40 CM, ROBOT 16 CM, SENSOR 90°)
Berdasarkan Logika wall_follower_v2 (1).ino & WallMazeSolver GitHub
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
ROBOT_W = 0.16       # 16 cm lebar bodi
ROBOT_L = 0.16       # 16 cm panjang bodi
WHEELBASE = 0.14     # 14 cm jarak antar roda

# Ambang Batas Sensor dari wall_follower_v2 (1).ino (dalam cm)
TARGET_TENGAH  = 12.0  # cm: jarak ideal saat robot di tengah lorong 40 cm
AMBANG_DEKAT   = 11.0  # cm: batas koreksi jika terlalu mepet
AMBANG_JAUH    = 13.0  # cm: batas koreksi jika menjauh
AMBANG_TERBUKA = 22.0  # cm: batas deteksi celah / jalan terbuka
DEPAN_STOP     = 12.0  # cm: jarak henti sensor depan untuk belok

# Start & Finish (meter)
START_X = 4.5 * CELL   # 1.80 m (Start 1)
START_Y = 0.5 * CELL   # 0.20 m
START_THETA = math.pi / 2  # Menghadap ke bawah (South, row 1)

FINISH_X_MIN = 3.0 * CELL  # 1.20 m
FINISH_X_MAX = 4.0 * CELL  # 1.60 m
FINISH_Y     = 1.85        # meter

# Dinding Horizontal dari gen_world.py: (y, x0, x1)
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

# Dinding Vertikal dari gen_world.py: (x, y0, y1)
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
    """Menghitung perpotongan sinar ultrasonik dengan dinding."""
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
    """
    Menghitung jarak sensor ultrasonik ke dinding terdekat.
    sensor_type:
      'F': Depan (moncong robot)
      'R': Kanan (tegak lurus 90° ke sisi kanan robot)
      'L': Kiri (tegak lurus 90° ke sisi kiri robot)
    """
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)

    if sensor_type == 'F':
        ox = rx + (ROBOT_L / 2) * cos_t
        oy = ry + (ROBOT_L / 2) * sin_t
        dx = cos_t
        dy = sin_t
    elif sensor_type == 'R':
        # Sisi Kanan (90° Clockwise pada koordinat Y-down)
        ox = rx + (ROBOT_W / 2) * (-sin_t)
        oy = ry + (ROBOT_W / 2) * cos_t
        dx = -sin_t
        dy = cos_t
    elif sensor_type == 'L':
        # Sisi Kiri (90° Counter-Clockwise pada koordinat Y-down)
        ox = rx + (ROBOT_W / 2) * sin_t
        oy = ry + (ROBOT_W / 2) * (-cos_t)
        dx = sin_t
        dy = -cos_t

    min_dist = 2.0  # batas maksimum 200 cm
    hit_pt = (ox + dx * min_dist, oy + dy * min_dist)

    for x1, y1, x2, y2 in wall_segments:
        d = ray_seg_intersect(ox, oy, dx, dy, x1, y1, x2, y2)
        if d is not None and d < min_dist:
            min_dist = d
            hit_pt = (ox + dx * d, oy + dy * d)

    return min_dist * 100.0, (ox, oy), hit_pt


def generate_video():
    print("Memulai simulasi & perekaman video robot wall maze solver...")
    WIDTH, HEIGHT = 1280, 720
    FPS = 30
    DT = 1.0 / FPS

    # Konfigurasi Viewport Arena di Frame Video
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

    # State Robot
    rx = START_X
    ry = START_Y
    theta = START_THETA

    state = 'FOLLOW'
    state_timer = 0.0
    turn_target_angle = 0.0
    turn_direction = 0  # +1: Kanan, -1: Kiri
    action_text = "Mulai Bergerak"
    action_color = (255, 255, 255)

    pwm_l, pwm_r = 135, 135
    trajectory = []

    video_filename = os.path.join(os.path.dirname(os.path.abspath(__file__)), "simulasi_maze_wall_follower.mp4")
    writer = imageio.get_writer(video_filename, fps=FPS, codec='libx264', quality=8)

    step = 0
    max_steps = 30 * 95  # Maksimal 95 detik
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

        # ================= LOGIKA NAVIGASI (wall_follower_v2) =================
        if state == 'FOLLOW':
            if cmF <= DEPAN_STOP:
                # Depan Terhalang -> Masuk ke Mode Manuver
                state = 'OBSTACLE_BACK'
                state_timer = 0.15  # Mundur 150 ms
                action_text = "Depan Terhalang! Mundur sejenak..."
                action_color = (50, 50, 255)
            else:
                # Wall Following (Penelusuran Dinding Kanan)
                if cmRF >= AMBANG_DEKAT and cmRF <= AMBANG_JAUH:
                    pwm_l, pwm_r = 135, 135
                    action_text = "Maju Lurus (Posisi Kanan Ideal ~12cm)"
                    action_color = (255, 255, 255)
                elif cmRF > AMBANG_JAUH:
                    if cmRF >= AMBANG_TERBUKA:
                        # Dinding kanan hilang (celah/simpang kanan) -> ikuti dinding kiri
                        if cmLF >= AMBANG_DEKAT and cmLF <= AMBANG_JAUH:
                            pwm_l, pwm_r = 135, 135
                            action_text = "Maju Lurus (Pemandu Dinding Kiri)"
                            action_color = (200, 230, 255)
                        elif cmLF > AMBANG_JAUH and cmLF < AMBANG_TERBUKA:
                            pwm_l, pwm_r = 115, 140
                            action_text = "Koreksi Kiri Halus (Menjaga Dinding Kiri)"
                            action_color = (180, 220, 255)
                        elif cmLF < AMBANG_DEKAT:
                            pwm_l, pwm_r = 140, 115
                            action_text = "Koreksi Kanan Halus (Terlalu Dekat Kiri)"
                            action_color = (255, 200, 180)
                        else:
                            pwm_l, pwm_r = 135, 135
                            action_text = "Maju Lurus (Ruang Terbuka)"
                            action_color = (220, 220, 220)
                    else:
                        pwm_l, pwm_r = 140, 115
                        action_text = "Koreksi Kanan Halus (Mendekati Kanan)"
                        action_color = (255, 220, 150)
                elif cmRF < AMBANG_DEKAT:
                    pwm_l, pwm_r = 115, 140
                    action_text = "Koreksi Kiri Halus (Menjauhi Kanan)"
                    action_color = (150, 220, 255)

                # Pembaruan Fisika Diferensial
                v = ((pwm_l + pwm_r) / 2.0 / 255.0) * 0.35
                omega = ((pwm_l - pwm_r) / 255.0) * 1.5
                rx += v * math.cos(theta) * DT
                ry += v * math.sin(theta) * DT
                theta += omega * DT

        elif state == 'OBSTACLE_BACK':
            # Mundur perlahan
            pwm_l, pwm_r = -110, -110
            v = -0.15
            rx += v * math.cos(theta) * DT
            ry += v * math.sin(theta) * DT
            state_timer -= DT
            if state_timer <= 0:
                state = 'OBSTACLE_STOP'
                state_timer = 0.15  # Berhenti 150 ms untuk evaluasi sensor

        elif state == 'OBSTACLE_STOP':
            pwm_l, pwm_r = 0, 0
            state_timer -= DT
            if state_timer <= 0:
                # Evaluasi Sisi Kanan & Kiri
                kananBebas = (cmRF > AMBANG_TERBUKA)
                kiriBebas  = (cmLF > AMBANG_TERBUKA)

                if not kiriBebas and kananBebas:
                    turn_direction = 1  # Kanan 90°
                    turn_target_angle = math.pi / 2
                    action_text = "Kiri Dinding, Kanan Bebas -> Belok Kanan 90°"
                    action_color = (220, 80, 255)
                elif kiriBebas and not kananBebas:
                    turn_direction = -1  # Kiri 90°
                    turn_target_angle = math.pi / 2
                    action_text = "Kanan Dinding, Kiri Bebas -> Belok Kiri 90°"
                    action_color = (80, 220, 255)
                elif kiriBebas and kananBebas:
                    turn_direction = 1  # Simpang T: Belok Kanan 90°
                    turn_target_angle = math.pi / 2
                    action_text = "Simpang T (Kedua Sisi Bebas) -> Belok Kanan 90°"
                    action_color = (255, 200, 50)
                else:
                    turn_direction = 1  # Jalan Buntu: Putar Balik 180°
                    turn_target_angle = math.pi
                    action_text = "JALAN BUNTU! Putar Balik 180°"
                    action_color = (255, 80, 80)

                state = 'OBSTACLE_TURN'
                turned_so_far = 0.0

        elif state == 'OBSTACLE_TURN':
            turn_speed = 3.0  # rad/s
            d_theta = turn_speed * DT
            if turned_so_far + d_theta >= turn_target_angle:
                d_theta = turn_target_angle - turned_so_far
                theta += turn_direction * d_theta
                state = 'CLEAR_TURN'
                state_timer = 0.20  # Maju sedikit setelah belok
                action_text = "Selesai Belok -> Maju Menjauhi Sudut"
                action_color = (200, 255, 200)
            else:
                theta += turn_direction * d_theta
                turned_so_far += d_theta
                if turn_direction > 0:
                    pwm_l, pwm_r = 150, -150
                else:
                    pwm_l, pwm_r = -150, 150

        elif state == 'CLEAR_TURN':
            pwm_l, pwm_r = 135, 135
            v = 0.20
            rx += v * math.cos(theta) * DT
            ry += v * math.sin(theta) * DT
            state_timer -= DT
            if state_timer <= 0:
                state = 'FOLLOW'

        elif state == 'FINISH':
            pwm_l, pwm_r = 0, 0
            if finish_hold_counter >= 90:  # Hold 3 detik (90 frame)
                break

        # ================= RENDER FRAME GRAFIS =================
        frame = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 242

        # Latar Arena
        p_arena_tl = to_screen(0, 0)
        p_arena_br = to_screen(W_ARENA, H_ARENA)
        cv2.rectangle(frame, p_arena_tl, p_arena_br, (255, 255, 255), -1)

        # Garis Grid
        for c in range(COLS + 1):
            cv2.line(frame, to_screen(c * CELL, 0), to_screen(c * CELL, H_ARENA), (232, 232, 232), 1)
        for r in range(ROWS + 1):
            cv2.line(frame, to_screen(0, r * CELL), to_screen(W_ARENA, r * CELL), (232, 232, 232), 1)

        # Zona START 1 (Hijau Muda)
        s1 = to_screen(4.0 * CELL, 0)
        s2 = to_screen(5.0 * CELL, 0.4 * CELL)
        cv2.rectangle(frame, s1, s2, (200, 245, 200), -1)
        cv2.putText(frame, "START 1", (s1[0] + 18, s1[1] + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 0), 2)

        # Zona FINISH 1 (Merah Muda)
        f1 = to_screen(3.0 * CELL, 4.6 * CELL)
        f2 = to_screen(4.0 * CELL, 5.0 * CELL)
        cv2.rectangle(frame, f1, f2, (200, 200, 255), -1)
        cv2.putText(frame, "FINISH 1", (f1[0] + 16, f2[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 200), 2)

        # Jejak Lintasan (Trajectory Trail)
        if len(trajectory) > 1:
            pts = np.array([to_screen(tx, ty) for tx, ty in trajectory], dtype=np.int32)
            cv2.polylines(frame, [pts], False, (220, 130, 40), 2, cv2.LINE_AA)

        # Gambar Dinding Arena
        for x1, y1, x2, y2 in wall_segments:
            p1 = to_screen(x1, y1)
            p2 = to_screen(x2, y2)
            cv2.line(frame, p1, p2, (35, 35, 35), 6, cv2.LINE_AA)

        # Sinar Sensor Ultrasonik
        # Depan: Merah jika dekat, Hijau jika aman
        col_f_beam = (0, 0, 240) if cmF <= DEPAN_STOP else (0, 200, 0)
        cv2.line(frame, to_screen(*ptF_orig), to_screen(*ptF_hit), col_f_beam, 2, cv2.LINE_AA)
        cv2.circle(frame, to_screen(*ptF_hit), 4, col_f_beam, -1)

        # Kanan (Ungu) & Kiri (Biru-Kuning)
        cv2.line(frame, to_screen(*ptR_orig), to_screen(*ptR_hit), (200, 0, 200), 2, cv2.LINE_AA)
        cv2.circle(frame, to_screen(*ptR_hit), 4, (200, 0, 200), -1)

        cv2.line(frame, to_screen(*ptL_orig), to_screen(*ptL_hit), (200, 160, 0), 2, cv2.LINE_AA)
        cv2.circle(frame, to_screen(*ptL_hit), 4, (200, 160, 0), -1)

        # Gambar Bodi Robot
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
        cv2.fillPoly(frame, [poly_pts], (35, 155, 235))  # Oranye bodi
        cv2.polylines(frame, [poly_pts], True, (20, 80, 160), 2, cv2.LINE_AA)

        # Panah Arah Hadap
        pt_center = to_screen(rx, ry)
        pt_front = to_screen(rx + hl * c_t, ry + hl * s_t)
        cv2.arrowedLine(frame, pt_center, pt_front, (255, 255, 255), 2, tipLength=0.35)

        # ================= TOP HUD & DASHBOARD =================
        cv2.rectangle(frame, (0, 0), (WIDTH, 120), (22, 26, 32), -1)

        cv2.putText(frame, "WALL MAZE SOLVER ROBOT - SIMULASI ARENA", (25, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(frame, "Lebar Lorong: 40cm | Lebar Robot: 16cm | 3x Ultrasonik 90 | Aturan Dinding Kanan", (25, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 190, 205), 1)

        # Kartu Sensor Kiri
        cv2.rectangle(frame, (25, 66), (160, 110), (38, 45, 55), -1)
        cv2.putText(frame, "KIRI (90)", (35, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (190, 200, 210), 1)
        cv2.putText(frame, f"{cmLF:.1f} cm", (35, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (230, 180, 40), 2)

        # Kartu Sensor Depan
        cv2.rectangle(frame, (170, 66), (305, 110), (38, 45, 55), -1)
        cv2.putText(frame, "DEPAN", (180, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (190, 200, 210), 1)
        cv2.putText(frame, f"{cmF:.1f} cm", (180, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.60, col_f_beam, 2)

        # Kartu Sensor Kanan
        cv2.rectangle(frame, (315, 66), (450, 110), (38, 45, 55), -1)
        cv2.putText(frame, "KANAN (90)", (325, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (190, 200, 210), 1)
        cv2.putText(frame, f"{cmRF:.1f} cm", (325, 104), cv2.FONT_HERSHEY_SIMPLEX, 0.60, (220, 80, 220), 2)

        # Status Aksi
        cv2.rectangle(frame, (460, 66), (930, 110), (32, 38, 48), -1)
        cv2.putText(frame, "STATUS KONTROL MOTOR & LOGIKA:", (470, 82), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (160, 175, 190), 1)
        cv2.putText(frame, action_text, (470, 103), cv2.FONT_HERSHEY_SIMPLEX, 0.48, action_color, 2)

        # Telemetri Waktu & Kecepatan
        time_sec = step * DT
        cv2.putText(frame, f"Waktu: {time_sec:.1f} s", (950, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 215, 230), 1)
        cv2.putText(frame, f"PWM: L={pwm_l} R={pwm_r}", (950, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 205, 220), 1)
        cv2.putText(frame, f"Pos: ({rx:.2f}m, {ry:.2f}m)", (1110, 78), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 195, 210), 1)
        cv2.putText(frame, f"Sudut: {math.degrees(theta)%360:.0f} deg", (1110, 102), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 195, 210), 1)

        # Spanduk SELESAI jika Finish
        if state == 'FINISH':
            b_w, b_h = 560, 120
            bx1, by1 = (WIDTH - b_w) // 2, (HEIGHT - b_h) // 2
            bx2, by2 = bx1 + b_w, by1 + b_h
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), (0, 140, 0), -1)
            cv2.rectangle(frame, (bx1, by1), (bx2, by2), (255, 255, 255), 3)
            cv2.putText(frame, "FINISH TERCAPAI!", (bx1 + 110, by1 + 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.15, (255, 255, 255), 3)
            cv2.putText(frame, f"Waktu Tempuh: {time_sec:.2f} detik | Sukses Menembus Labirin", (bx1 + 40, by1 + 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 255, 230), 2)

        # Convert BGR (OpenCV) to RGB for imageio
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        writer.append_data(frame_rgb)

    writer.close()
    print(f"Video simulasi berhasil disimpan: {video_filename}")
    print(f"Total durasi simulasi: {step * DT:.2f} detik ({step} frame).")
    return video_filename


if __name__ == '__main__':
    generate_video()
