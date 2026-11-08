"""Controller e-puck: wall follower (mengikuti dinding KIRI) dengan kontrol P.

Sensor e-puck: ps0..ps7. Posisi (dilihat dari atas, depan = ps0/ps7):
  ps0 depan-kanan, ps1 kanan-depan, ps2 kanan, ps5 kiri, ps6 kiri-depan, ps7 depan-kiri.
Nilai sensor makin BESAR = dinding makin DEKAT.
Ganti FOLLOW_LEFT = False untuk mengikuti dinding kanan.
"""
from controller import Robot

FOLLOW_LEFT = True

MAX_SPEED = 6.28        # rad/s (batas e-puck)
BASE_SPEED = 0.6 * MAX_SPEED
TARGET = 150.0          # nilai sensor samping yang diinginkan (jarak ke dinding)
FRONT_LIMIT = 150.0     # ambang dinding depan -> belok menjauh
LOST_LIMIT = 70.0       # di bawah ini dianggap dinding samping hilang
KP = 0.004

robot = Robot()
dt = int(robot.getBasicTimeStep())

ps = []
for i in range(8):
    s = robot.getDevice(f"ps{i}")
    s.enable(dt)
    ps.append(s)

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
left.setPosition(float("inf"))
right.setPosition(float("inf"))
left.setVelocity(0.0)
right.setVelocity(0.0)


def clamp(v):
    return max(-MAX_SPEED, min(MAX_SPEED, v))


while robot.step(dt) != -1:
    v = [s.getValue() for s in ps]

    front = max(v[0], v[7])
    if FOLLOW_LEFT:
        side = max(v[5], v[6] * 0.8)
        corner = v[6]
    else:
        side = max(v[2], v[1] * 0.8)
        corner = v[1]

    if front > FRONT_LIMIT:
        # dinding di depan: putar di tempat menjauhi dinding yang diikuti
        turn = 1.0 if FOLLOW_LEFT else -1.0
        l_speed, r_speed = turn * 0.5 * MAX_SPEED, -turn * 0.5 * MAX_SPEED
    elif side < LOST_LIMIT and corner < LOST_LIMIT:
        # dinding hilang (tikungan terbuka): belok pelan ke arah dinding
        turn = -1.0 if FOLLOW_LEFT else 1.0
        l_speed = BASE_SPEED * (0.35 if FOLLOW_LEFT else 1.0)
        r_speed = BASE_SPEED * (1.0 if FOLLOW_LEFT else 0.35)
    else:
        # kontrol P menjaga jarak ke dinding
        err = TARGET - side          # >0: terlalu jauh, <0: terlalu dekat
        corr = KP * err * MAX_SPEED
        if FOLLOW_LEFT:
            l_speed, r_speed = BASE_SPEED - corr, BASE_SPEED + corr
        else:
            l_speed, r_speed = BASE_SPEED + corr, BASE_SPEED - corr

    left.setVelocity(clamp(l_speed))
    right.setVelocity(clamp(r_speed))
