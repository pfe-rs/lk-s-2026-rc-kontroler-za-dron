import matplotlib
matplotlib.use('TkAgg')

import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import serial
import serial.tools.list_ports

# ============================================================
#  PODESAVANJA
# ============================================================
SIMULATION_MODE = False       # True = bez ESP32, testiranje
PORT            = "/dev/ttyACM0"
BAUD            = 115200
SPEED           = 2.0         # translaciona brzina (jed/s po stepenu)
YAW_SPEED       = 1.5         # brzina yaw-a (stepen/s po stepenu dzojstika)
VIEW_RANGE      = 100
MAX_TRAIL       = 300

# ============================================================
#  SERIJSKI PORT
# ============================================================
ser = None
if not SIMULATION_MODE:
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1)
        print(f"Povezan na {PORT}")
    except Exception as e:
        print(f"Ne mogu da otvorim port {PORT}: {e}")
        print("Dostupni portovi:")
        for p in serial.tools.list_ports.comports():
            print(f"  {p.device}  -  {p.description}")
        exit(1)

# ============================================================
#  GEOMETRIJA DRONA
# ============================================================
L, W, H = 4.0, 2.0, 0.6

# 8 temena kvadra
vertices = np.array([
    [-L/2, -W/2, -H/2], [ L/2, -W/2, -H/2],
    [ L/2,  W/2, -H/2], [-L/2,  W/2, -H/2],
    [-L/2, -W/2,  H/2], [ L/2, -W/2,  H/2],
    [ L/2,  W/2,  H/2], [-L/2,  W/2,  H/2],
], dtype=float)

faces_idx = [
    [0,1,2,3], [4,5,6,7],
    [0,1,5,4], [2,3,7,6],
    [1,2,6,5], [0,3,7,4],
]

# ============================================================
#  STANJE DRONA
# ============================================================
pos       = np.array([0.0, 0.0, 0.0])
yaw_angle = 0.0
armed     = False
no_signal = False

trail_world = []
sim_t       = 0.0

# ============================================================
#  POMOCNE FUNKCIJE
# ============================================================
def rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    roll  = np.radians(roll_deg)
    pitch = np.radians(-pitch_deg)   # flip vizuelni nagib
    yaw   = np.radians(yaw_deg)

    Rx = np.array([
        [1, 0,             0            ],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch),  np.cos(pitch)],
    ])
    Ry = np.array([
        [ np.cos(roll), 0, np.sin(roll)],
        [0,             1, 0           ],
        [-np.sin(roll), 0, np.cos(roll)],
    ])
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1],
    ])
    return Rz @ Ry @ Rx


def parse_line(raw):
    """Parsira jednu liniju sa serijskog porta. Vraca dict ili None."""
    parts = raw.replace("\t", " ").split()
    vals  = {}
    for i, p in enumerate(parts):
        for key in ("LX:", "LY:", "RX:", "RY:", "ARM:", "BAT:"):
            if key in p:
                try:
                    vals[key[:-1]] = float(p.replace(key, ""))
                except ValueError:
                    pass
    return vals if len(vals) == 6 else None


def read_latest():
    """Citaj sve dostupne linije, vrati samo najnoviju validnu."""
    global no_signal
    latest = None

    while ser.in_waiting:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw:
            continue
        if raw.startswith("STATUS:TIMEOUT"):
            no_signal = True
            continue
        parsed = parse_line(raw)
        if parsed:
            latest    = parsed
            no_signal = False

    return latest


def sim_joystick(t):
    """Simulirani dzojstik za testiranje bez ESP32."""
    return {
        "LX":  8.0 * math.sin(t * 0.3),
        "LY":  4.0,
        "RX": 12.0 * math.sin(t * 0.5),
        "RY": 12.0 * math.cos(t * 0.5),
        "ARM": 1.0,
        "BAT": 3.9,
    }


def update_ticks():
    """Azurira labele osa da prikazuju stvarne svetske koordinate."""
    tick_step = VIEW_RANGE / 4
    ticks     = np.arange(-VIEW_RANGE, VIEW_RANGE + 0.1, tick_step)

    ax.set_xticks(ticks)
    ax.set_yticks(ticks)
    ax.set_zticks(ticks)
    ax.set_xticklabels([f"{t + pos[0]:.0f}" for t in ticks], fontsize=7)
    ax.set_yticklabels([f"{t + pos[1]:.0f}" for t in ticks], fontsize=7)
    ax.set_zticklabels([f"{t + pos[2]:.0f}" for t in ticks], fontsize=7)

# ============================================================
#  PLOT SETUP
# ============================================================
plt.ion()
fig = plt.figure(figsize=(9, 9))
ax  = fig.add_subplot(111, projection='3d')
ax.set_box_aspect([1, 1, 1])
ax.set_xlabel("X  (levo/desno)")
ax.set_ylabel("Y  (napred/nazad)")
ax.set_zlabel("Z  (gore/dole)")
ax.set_xlim(-VIEW_RANGE, VIEW_RANGE)
ax.set_ylim(-VIEW_RANGE, VIEW_RANGE)
ax.set_zlim(-VIEW_RANGE, VIEW_RANGE)

poly        = None
trail_line, = ax.plot([], [], [], color="orangered", linewidth=1.2, alpha=0.6)

# ============================================================
#  GLAVNA PETLJA
# ============================================================
last_time = time.time()

print("Vizualizacija pokrenuta. Ctrl+C za izlaz.")

while True:
    try:
        now = time.time()
        dt  = min(now - last_time, 0.1)   # max dt 100ms da ne skoci pri lagu
        last_time = now

        # --- Uzmi podatke ---
        if SIMULATION_MODE:
            sim_t  += dt
            result  = sim_joystick(sim_t)
        else:
            result = read_latest()
            if result is None:
                plt.pause(0.01)
                continue

        lx    = result["LX"]
        ly    = result["LY"]
        rx    = result["RX"]
        ry    = result["RY"]
        armed = bool(result["ARM"])
        v_bat = result["BAT"]

        # --- Kretanje relativno na heading drona ---
        if armed:
            yaw_rad = np.radians(-yaw_angle)  # negiran u odnosu na pre

            move_x =  rx * np.cos(yaw_rad) + ry * np.sin(yaw_rad)
            move_y = -rx * np.sin(yaw_rad) + ry * np.cos(yaw_rad)

            pos[0] += move_x * SPEED * dt
            pos[1] += move_y * SPEED * dt
            pos[2] += ly * SPEED * dt

            yaw_angle -= lx * YAW_SPEED * dt

        # --- Rotacija modela ---
        # Kad nije armed - dron vizuelno stoji ravno (roll=0, pitch=0)
        visual_roll  = rx if armed else 0.0
        visual_pitch = ry if armed else 0.0

        R       = rotation_matrix(visual_roll, visual_pitch, yaw_angle)
        rotated = vertices @ R.T
        faces   = [[rotated[i] for i in f] for f in faces_idx]

        color = "dodgerblue" if armed else "#888888"

        if poly is None:
            poly = Poly3DCollection(
                faces,
                facecolor=color,
                edgecolor="black",
                alpha=0.85,
                linewidths=0.8,
            )
            ax.add_collection3d(poly)
        else:
            poly.set_verts(faces)
            poly.set_facecolor(color)

        # --- Trag ---
        trail_world.append(pos.copy())
        if len(trail_world) > MAX_TRAIL:
            trail_world.pop(0)

        trail_arr = np.array(trail_world) - pos
        trail_line.set_data(trail_arr[:, 0], trail_arr[:, 1])
        trail_line.set_3d_properties(trail_arr[:, 2])

        # --- Kamera / ose ---
        ax.set_xlim(-VIEW_RANGE, VIEW_RANGE)
        ax.set_ylim(-VIEW_RANGE, VIEW_RANGE)
        ax.set_zlim(-VIEW_RANGE, VIEW_RANGE)
        update_ticks()

        # --- Naslov ---
        if no_signal:
            status = "NEMA SIGNALA"
        elif armed:
            status = "ARMED"
        else:
            status = "DISARMED"

        mode_str = "  [SIM]" if SIMULATION_MODE else ""
        ax.set_title(
            f"BATTERY: {v_bat}V, STATUS: {status}{mode_str} \nYaw: {yaw_angle % 360:.1f}°\n"
            f"X: {pos[0]:.1f}   Y: {pos[1]:.1f}   Z: {pos[2]:.1f}",
            fontsize=11,
        )

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)

    except KeyboardInterrupt:
        print("\nIzlaz.")
        break
    except Exception as e:
        print(f"Greška: {e}")
        continue

if ser:
    ser.close()
