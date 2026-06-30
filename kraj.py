import serial
import matplotlib.pyplot as plt
from collections import deque

# --- Podešavanja ---
PORT = "/dev/ttyACM0"   # promeni na svoj port (npr. "COM5" na Windows-u)
BAUD = 115200
MAX_POINTS = 200        # koliko poslednjih tačaka da prikazuje

ser = serial.Serial(PORT, BAUD, timeout=1)

x_data = deque(maxlen=MAX_POINTS)
y_data = deque(maxlen=MAX_POINTS)
t_data = deque(maxlen=MAX_POINTS)

plt.ion()
fig, ax = plt.subplots()
line_x, = ax.plot([], [], label="X")
line_y, = ax.plot([], [], label="Y")
ax.set_ylim(0, 4095)
ax.legend()
ax.set_xlabel("Uzorak")
ax.set_ylabel("ADC vrednost")

counter = 0

while True:
    try:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw:
            continue

        # Očekivani format: "X: 1234    Y: 4321"
        parts = raw.replace("\t", " ").split()
        x_val = y_val = None

        for i, p in enumerate(parts):
            if p.startswith("X:"):
                x_val = int(parts[i + 1])
            elif p.startswith("Y:"):
                y_val = int(parts[i + 1])

        if x_val is None or y_val is None:
            continue

        counter += 1
        t_data.append(counter)
        x_data.append(x_val)
        y_data.append(y_val)

        line_x.set_data(t_data, x_data)
        line_y.set_data(t_data, y_data)
        ax.set_xlim(max(0, counter - MAX_POINTS), counter)

        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.pause(0.001)

    except KeyboardInterrupt:
        break
    except Exception as e:
        print("Greška:", e)
        continue

ser.close()
