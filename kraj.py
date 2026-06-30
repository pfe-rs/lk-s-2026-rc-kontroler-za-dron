import serial
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- Podešavanja ---
PORT = "/dev/ttyACM0"   # promeni na svoj port (npr. "COM5" na Windows-u)
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)

# --- Definicija kvadra (kutije) ---
# Dimenzije: dužina x širina x visina
L, W, H = 4, 2, 1

# 8 temena kvadra (centriranog u koordinatnom početku)
vertices = np.array([
    [-L/2, -W/2, -H/2],
    [ L/2, -W/2, -H/2],
    [ L/2,  W/2, -H/2],
    [-L/2,  W/2, -H/2],
    [-L/2, -W/2,  H/2],
    [ L/2, -W/2,  H/2],
    [ L/2,  W/2,  H/2],
    [-L/2,  W/2,  H/2],
])

# Indeksi temena za 6 strana kvadra
faces_idx = [
    [0, 1, 2, 3],  # donja
    [4, 5, 6, 7],  # gornja
    [0, 1, 5, 4],  # prednja
    [2, 3, 7, 6],  # zadnja
    [1, 2, 6, 5],  # desna
    [0, 3, 7, 4],  # leva
]


def rotation_matrix(roll_deg, pitch_deg):
    """Rotacija oko X ose (pitch) i Y ose (roll), u stepenima."""
    roll = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)

    # Rotacija oko X ose (pitch)
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(pitch), -np.sin(pitch)],
        [0, np.sin(pitch), np.cos(pitch)]
    ])

    # Rotacija oko Y ose (roll)
    Ry = np.array([
        [np.cos(roll), 0, np.sin(roll)],
        [0, 1, 0],
        [-np.sin(roll), 0, np.cos(roll)]
    ])

    return Ry @ Rx


def read_joystick():
    """Čita liniju sa serijskog porta i vraća (x, y) ili None."""
    raw = ser.readline().decode("utf-8", errors="ignore").strip()
    if not raw:
        return None

    parts = raw.replace("\t", " ").split()
    x_val = y_val = None

    for i, p in enumerate(parts):
        if p.startswith("X:"):
            x_val = int(parts[i + 1])
        elif p.startswith("Y:"):
            y_val = int(parts[i + 1])

    if x_val is None or y_val is None:
        return None

    return x_val, y_val

def read_joystick_latest():
    """Čita SVE dostupne linije i vraća samo poslednju validnu."""
    latest = None
    while ser.in_waiting:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw:
            continue
        parts = raw.replace("\t", " ").split()
        x_val = y_val = None
        for i, p in enumerate(parts):
            if p.startswith("X:"):
                x_val = int(parts[i + 1])
            elif p.startswith("Y:"):
                y_val = int(parts[i + 1])
        if x_val is not None and y_val is not None:
            latest = (x_val, y_val)
    return latest


# --- Priprema 3D plota ---
plt.ion()
fig = plt.figure(figsize=(7, 7))
ax = fig.add_subplot(111, projection='3d')

limit = 4
ax.set_xlim(-limit, limit)
ax.set_ylim(-limit, limit)
ax.set_zlim(-limit, limit)
ax.set_box_aspect([1, 1, 1])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

poly = None

while True:
    try:
        result = read_joystick_latest()
        if result is None:
            continue

        x_val, y_val = result  # opseg -20..20

        # X -> roll (nagib levo/desno), Y -> pitch (nagib napred/nazad)
        R = rotation_matrix(roll_deg=x_val, pitch_deg=y_val)
        rotated = vertices @ R.T

        faces = [[rotated[i] for i in face] for face in faces_idx]

        if poly is None:
            poly = Poly3DCollection(faces, facecolor="dodgerblue", edgecolor="black", alpha=0.8)
            ax.add_collection3d(poly)
        else:
            poly.set_verts(faces)

        ax.set_title(f"Roll (X): {x_val}°   Pitch (Y): {y_val}°")

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)

    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Greška:", e)
        continue

ser.close()
