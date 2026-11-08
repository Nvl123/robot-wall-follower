"""Generator world Webots (e-puck) dari denah Arena Wall Follower Robotic.

Koordinat grid: col 0..10 (kiri->kanan), row 0..5 (atas->bawah).
Ubah CELL_W / CELL_H (meter) kalau ukuran sel aslinya berbeda.
"""
import os

CELL_W = 0.40   # lebar 1 kolom (m)  -> 10 kolom = 4.0 m
CELL_H = 0.40   # tinggi 1 baris (m) -> 5 baris = 2.0 m
COLS, ROWS = 10, 5
WALL_T = 0.02   # tebal dinding
WALL_H = 0.10   # tinggi dinding

W, H = COLS * CELL_W, ROWS * CELL_H

# Dinding: (kolom/baris awal, kolom/baris akhir) pada garis grid
# 'H' = horizontal: (row, col_start, col_end) ; 'V' = vertikal: (col, row_start, row_end)
walls = []
# --- batas luar ---
walls += [("H", 0, 0, 10)]                 # atas
walls += [("V", 0, 0, 5)]                  # kiri
walls += [("V", 10, 0, 5)]                 # kanan
walls += [("H", 5, 0, 3), ("H", 5, 4, 6), ("H", 5, 7, 10)]  # bawah (celah = Finish di col 3-4 & 6-7)
# --- dinding dalam ---
walls += [("V", 1, 0, 2)]
walls += [("H", 2, 1, 4)]
walls += [("H", 1, 2, 8)]
walls += [("V", 5, 0, 5)]                  # pemisah tengah (dua start)
walls += [("V", 2, 2, 3)]
walls += [("H", 3, 3, 7)]
walls += [("V", 3, 3, 4)]
walls += [("H", 4, 1, 3)]
walls += [("V", 7, 3, 4)]
walls += [("H", 4, 7, 9)]
walls += [("H", 2, 6, 9)]
walls += [("V", 9, 0, 2)]
walls += [("V", 8, 2, 3)]

# Penanda (kolom, baris) pusat, dalam satuan grid
starts = [(4.5, 0.5), (5.5, 0.5)]
finishes = [(3.5, 4.9), (6.5, 4.9)]
retries = [(4.5, 2.0), (5.5, 2.0), (0.5, 4.0), (9.5, 4.0)]


def gx(col):
    return col * CELL_W - W / 2


def gy(row):
    return H / 2 - row * CELL_H


def wall_node(i, kind, a, b, c):
    if kind == "H":
        row, c0, c1 = a, b, c
        cx, cy = (gx(c0) + gx(c1)) / 2, gy(row)
        sx, sy = (c1 - c0) * CELL_W + WALL_T, WALL_T
    else:
        col, r0, r1 = a, b, c
        cx, cy = gx(col), (gy(r0) + gy(r1)) / 2
        sx, sy = WALL_T, (r1 - r0) * CELL_H + WALL_T
    return f"""Solid {{
  translation {cx:.4f} {cy:.4f} {WALL_H/2:.4f}
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor 0.05 0.05 0.05 roughness 1 metalness 0 }}
      geometry Box {{ size {sx:.4f} {sy:.4f} {WALL_H:.4f} }}
    }}
  ]
  name "wall{i}"
  boundingObject Box {{ size {sx:.4f} {sy:.4f} {WALL_H:.4f} }}
}}"""


def marker(name, col, row, color, sx=0.20, sy=0.12):
    return f"""Solid {{
  translation {gx(col):.4f} {gy(row):.4f} 0.001
  children [
    Shape {{
      appearance PBRAppearance {{ baseColor {color} roughness 1 metalness 0 }}
      geometry Box {{ size {sx} {sy} 0.002 }}
    }}
  ]
  name "{name}"
}}"""


parts = []
parts.append("""#VRML_SIM R2023b utf8

EXTERNPROTO "https://raw.githubusercontent.com/cyberbotics/webots/R2023b/projects/robots/gctronic/e-puck/protos/E-puck.proto"

WorldInfo {
  basicTimeStep 32
  coordinateSystem "ENU"
}
Viewpoint {
  orientation 0 0.9 -0.43 3.14
  position 0 -3.2 4.2
}
Background { skyColor [ 0.9 0.9 0.9 ] }
DirectionalLight { direction 0 0.3 -1 }
""")
parts.append(f"""Floor {{
  size {W} {H}
  appearance PBRAppearance {{ baseColor 0.95 0.95 0.95 roughness 1 metalness 0 }}
}}""")
for i, (k, a, b, c) in enumerate(walls):
    parts.append(wall_node(i, k, a, b, c))
for i, (c, r) in enumerate(starts):
    parts.append(marker(f"start{i+1}", c, r, "0.3 0.7 0.2", 0.12, 0.30))
for i, (c, r) in enumerate(finishes):
    parts.append(marker(f"finish{i+1}", c, r, "0.85 0.2 0.2", 0.30, 0.12))
for i, (c, r) in enumerate(retries):
    parts.append(marker(f"retry{i+1}", c, r, "0.9 0.5 0.1", 0.12, 0.06))

# Robot di Start 1 (kolom 4.5, baris 0.5), menghadap ke bawah (-y)
parts.append(f"""E-puck {{
  translation {gx(4.5):.4f} {gy(0.5):.4f} 0
  rotation 0 0 1 -1.5708
  name "e-puck"
  controller "wall_follower"
}}""")

out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "webots_project", "worlds")
os.makedirs(out_dir, exist_ok=True)
with open(os.path.join(out_dir, "wall_follower_arena.wbt"), "w") as f:
    f.write("\n".join(parts) + "\n")
print("OK", W, H, len(walls), "dinding")
