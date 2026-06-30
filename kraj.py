import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- Podešavanja ---
PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# --- Dimenzije kvadra ---
L, W, H = 4.0, 2.0, 1.0

vertices = np.array([
    [-L/2, -W/2, -H/2],
    [ L/2, -W/2, -H/2],
    [ L/2,  W/2, -H/2],
    [-L/2,  W/2, -H/2],
    [-L/2, -W/2,  H/2],
    [ L/2, -W/2,  H/2],
    [ L/2,  W/2,  H/2],
    [-L/2,  W/2,  H/2],
], dtype=float)

faces_idx = [
    [0, 1, 2, 3],
    [4, 5, 6, 7],
    [0, 1, 5, 4],
    [2, 3, 7, 6],
    [1, 2, 6, 5],
    [0, 3, 7, 4],
]

# --- Parametri kretanja ---
SPEED = 2.0
DEAD_ZONE = 1.5
VIEW_RANGE = 15  # koliko "sveta" se vidi oko drona

pos = np.array([0.0, 0.0, 0.0])  # STVARNA pozicija drona u svetu (koristi se samo interno)
trail_world = []  # lista stvarnih svetskih pozicija kroz vreme
MAX_TRAIL = 300


def rotation_matrix(roll_deg, pitch_deg, flip_pitch_visual=False):
    """flip_pitch_visual menja SAMO smer nagiba modela, ne utiče na kretanje."""
    pitch_sign = -1 if flip_pitch_visual else 1
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_sign * pitch_deg)

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)]
    ])

    Ry = np.array([
        [np.cos(roll), 0, np.sin(roll)],
        [0, 1, 0],
        [-np.sin(roll), 0, np.cos(roll)]
    ])

    return Ry @ Rx


def read_joystick_latest():
    latest = None
    while ser.in_waiting:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw:
            continue
        parts = raw.replace("\t", " ").split()
        x_val = y_val = None
        for i, p in enumerate(parts):
            if p.startswith("X:"):
                x_val = float(parts[i + 1])
            elif p.startswith("Y:"):
                y_val = float(parts[i + 1])
        if x_val is not None and y_val is not None:
            latest = (x_val, y_val)
    return latest


# --- Priprema 3D plota ---
plt.ion()
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1, 1, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y (napred/nazad)")
ax.set_zlabel("Z")

poly = None
trail_line, = ax.plot([], [], [], color="red", linewidth=1, alpha=0.6)

last_time = time.time()

while True:
    try:
        result = read_joystick_latest()
        if result is None:
            continue

        x_val, y_val = result  # roll (levo/desno), pitch (napred/nazad)

        now = time.time()
        dt = now - last_time
        last_time = now

        roll_eff = x_val if abs(x_val) > DEAD_ZONE else 0.0
        pitch_eff = y_val if abs(y_val) > DEAD_ZONE else 0.0

        # KRETANJE - originalni, ispravan smer (bez minusa)
        pos[0] += roll_eff * SPEED * dt
        pos[1] += pitch_eff * SPEED * dt

        # ROTACIJA modela - samo VIZUELNO obrnut pitch, kretanje ovo ne dira
        R = rotation_matrix(roll_deg=x_val, pitch_deg=y_val, flip_pitch_visual=True)
        rotated_local = vertices @ R.T  # kvadar se crta oko ishodišta (0,0,0) - uvek u centru

        faces = [[rotated_local[i] for i in face] for face in faces_idx]

        if poly is None:
            poly = Poly3DCollection(faces, facecolor="dodgerblue", edgecolor="black", alpha=0.8)
            ax.add_collection3d(poly)
        else:
            poly.set_verts(faces)

        # Pamtimo STVARNU svetsku poziciju u trag
        trail_world.append(pos.copy())
        if len(trail_world) > MAX_TRAIL:
            trail_world.pop(0)

        # Trag se prikazuje RELATIVNO u odnosu na trenutnu poziciju drona
        # (drone je uvek u (0,0,0) kadra, trag se pomera oko njega)
        trail_arr = np.array(trail_world) - pos
        trail_line.set_data(trail_arr[:, 0], trail_arr[:, 1])
        trail_line.set_3d_properties(trail_arr[:, 2])

        # Kamera fiksirana na (0,0,0) - dron je uvek u centru
        ax.set_xlim(-VIEW_RANGE, VIEW_RANGE)
        ax.set_ylim(-VIEW_RANGE, VIEW_RANGE)
        ax.set_zlim(-VIEW_RANGE, VIEW_RANGE)

        # Tick pozicije ostaju fiksne (relativne), ali LABELE prikazuju stvarnu svetsku poziciju
        tick_step = VIEW_RANGE / 3  # broj podeoka po osi (po želji menjaj)
        rel_ticks = np.arange(-VIEW_RANGE, VIEW_RANGE + 1, tick_step)

        ax.set_xticks(rel_ticks)
        ax.set_yticks(rel_ticks)
        ax.set_zticks(rel_ticks)

        ax.set_xticklabels([f"{t + pos[0]:.0f}" for t in rel_ticks])
        ax.set_yticklabels([f"{t + pos[1]:.0f}" for t in rel_ticks])
        ax.set_zticklabels([f"{t + pos[2]:.0f}" for t in rel_ticks])

        ax.set_title(f"Roll: {x_val:.1f}°  Pitch: {y_val:.1f}°  Svetska pozicija: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")

        ax.set_title(f"Roll: {x_val:.1f}°  Pitch: {y_val:.1f}°  Svetska pozicija: ({pos[0]:.1f}, {pos[1]:.1f})")

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)

    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Greška:", e)
        continue

ser.close()
