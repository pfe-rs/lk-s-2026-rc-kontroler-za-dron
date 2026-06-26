import asyncio
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

try:
    from bleak import BleakScanner, BleakClient
except ImportError:
    raise ImportError("Instaliraj bleak: pip install bleak")

SERVICE_UUID           = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
CHARACTERISTIC_UUID_RX = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
CHARACTERISTIC_UUID_TX  = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

MAX_SAMPLES = 100
RECONNECT_DELAY = 3.0   # sekundi između pokušaja reconnecta
MAX_RECONNECT   = 10    # maksimalan broj pokušaja

# ── Deljeno stanje ────────────────────────────────────────────────────────────
pitches     = []
raw_pitches = []
lock        = threading.Lock()
stop_event  = threading.Event()

ble_loop: asyncio.AbstractEventLoop | None = None
command_queue: asyncio.Queue | None = None   # kreira se unutar BLE threada

current_Kp = 0.1
current_Ki = 0.0
current_Kd = 0.0

# ── Callback za primanje podataka ─────────────────────────────────────────────
def handle_pitch(_, data: bytes):
    global current_Kp, current_Ki, current_Kd
    try:
        decoded = data.decode("utf-8").strip()
        parts   = decoded.split(",")

        if len(parts) < 10:
            print(f"⚠️  Kratka poruka ({len(parts)} polja): {decoded}")
            return

        parts = parts[:10]

        pitch      = float(parts[0])
        voltage    = float(parts[1])
        current_Kp = float(parts[2])
        current_Ki = float(parts[3])
        current_Kd = float(parts[4])
        idle       = float(parts[5])
        motori     = parts[6:]

        with lock:
            pitches.append(pitch)
            if len(pitches) > MAX_SAMPLES:
                pitches.pop(0)
            raw_pitches.append(idle)
            if len(raw_pitches) > MAX_SAMPLES:
                raw_pitches.pop(0)

        print(
            f"Pitch: {pitch:7.2f}°  Bat: {voltage:.2f}V  "
            f"PID: {current_Kp:.5f}/{current_Ki:.5f}/{current_Kd:.5f}  "
            f"Idle: {idle}  Motori: {' | '.join(motori)}"
        )
    except Exception as e:
        print(f"Greška u handle_pitch: {e}  |  raw: {data}")

# ── BLE worker s reconnect logikom ────────────────────────────────────────────
async def ble_worker(address: str):
    attempt = 0

    while not stop_event.is_set():
        attempt += 1
        print(f"[BLE] Pokušaj konekcije #{attempt} → {address}")

        try:
            async with BleakClient(address, timeout=10.0) as client:
                print("[BLE] Povezan ✓")
                attempt = 0  # resetuj brojač nakon uspješne konekcije

                await client.start_notify(CHARACTERISTIC_UUID_TX, handle_pitch)

                while not stop_event.is_set():
                    # Čekaj komandu bez blokiranja event loopa
                    try:
                        cmd = await asyncio.wait_for(command_queue.get(), timeout=0.5)
                    except asyncio.TimeoutError:
                        # Provjeri da li je klijent još uvijek živ
                        if not client.is_connected:
                            print("[BLE] Veza prekinuta, pokušavam reconnect...")
                            break
                        continue

                    if cmd.lower() == "exit":
                        stop_event.set()
                        break

                    try:
                        await client.write_gatt_char(
                            CHARACTERISTIC_UUID_RX, cmd.encode(), response=True
                        )
                        print(f"[BLE] Poslato: {cmd}")
                    except Exception as e:
                        print(f"[BLE] Greška pri slanju '{cmd}': {e}")

                # Pošalji kill komandu pri čistom izlasku
                if client.is_connected:
                    try:
                        await client.write_gatt_char(
                            CHARACTERISTIC_UUID_RX, b"0,0.1,0,0", response=True
                        )
                        await client.stop_notify(CHARACTERISTIC_UUID_TX)
                        print("[BLE] Kill komanda poslata, notifikacije zaustavljene.")
                    except Exception:
                        pass

        except Exception as e:
            print(f"[BLE] Greška konekcije: {e}")

        if stop_event.is_set():
            break

        if attempt >= MAX_RECONNECT:
            print(f"[BLE] Maksimalan broj pokušaja ({MAX_RECONNECT}) dostignut. Prekidam.")
            stop_event.set()
            break

        print(f"[BLE] Čekam {RECONNECT_DELAY}s prije sljedećeg pokušaja...")
        await asyncio.sleep(RECONNECT_DELAY)

    print("[BLE] Worker završen.")

# ── BLE thread ────────────────────────────────────────────────────────────────
def run_ble_loop(address: str):
    global ble_loop, command_queue

    ble_loop      = asyncio.new_event_loop()
    command_queue = asyncio.Queue()          # mora se kreirati u istom loopu
    asyncio.set_event_loop(ble_loop)

    try:
        ble_loop.run_until_complete(ble_worker(address))
    finally:
        ble_loop.close()
        print("[BLE] Event loop zatvoren.")

# ── Input thread ──────────────────────────────────────────────────────────────
def input_thread_func():
    print("Komande: t<broj>  p<broj>  i<broj>  d<broj>  exit")
    print("Primjer: t50  p0.2  i0.01  d5.0")

    while not stop_event.is_set():
        try:
            cmd = input(">>> ").strip()
        except EOFError:
            break

        if not cmd:
            continue

        # Sigurno ubacivanje u asyncio queue iz non-async threada
        if ble_loop and not ble_loop.is_closed():
            ble_loop.call_soon_threadsafe(command_queue.put_nowait, cmd)
        else:
            print("[Input] BLE loop nije aktivan.")

        if cmd.lower() == "exit":
            stop_event.set()
            break

# ── Live plot ─────────────────────────────────────────────────────────────────
def live_plot():
    fig, ax = plt.subplots()
    line_pid, = ax.plot([], [], lw=2,             label="PID Pitch")
    line_raw, = ax.plot([], [], lw=2, color="orange", label="Raw/Idle")
    ax.axhline(0, color="gray", linestyle="--", linewidth=1)
    ax.set_ylim(-50, 50)
    ax.set_xlim(0, MAX_SAMPLES)
    ax.set_xlabel("Sample")
    ax.set_ylabel("Pitch")
    ax.set_title("Pitch u realnom vremenu")
    ax.legend()

    def update(_frame):
        with lock:
            pid_data = pitches.copy()
            raw_data = raw_pitches.copy()

        if pid_data:
            x = list(range(len(pid_data)))
            line_pid.set_data(x, pid_data)
        if raw_data:
            x = list(range(len(raw_data)))
            line_raw.set_data(x, raw_data)

        return line_pid, line_raw

    def on_close(_event):
        print("[Plot] Prozor zatvoren, zaustavljam program...")
        stop_event.set()
        if ble_loop and not ble_loop.is_closed():
            ble_loop.call_soon_threadsafe(command_queue.put_nowait, "exit")

    fig.canvas.mpl_connect("close_event", on_close)

    _ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.show()

# ── Discover ──────────────────────────────────────────────────────────────────
async def discover_uart_device() -> str:
    print("Tražim ESP32 (UART Service)...")
    devices = await BleakScanner.discover(timeout=5)
    for d in devices:
        print(f"  {d.address}  |  {d.name}")
        if d.name == "UART Service":
            print(f"[BLE] Pronađen: {d.address}")
            return d.address
    return input("Nije pronađen automatski. Unesi adresu: ").strip()

# ── Main ──────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    address = asyncio.run(discover_uart_device())

    ble_thread   = threading.Thread(target=run_ble_loop,      args=(address,), daemon=True)
    input_thread = threading.Thread(target=input_thread_func,                  daemon=True)

    ble_thread.start()
    input_thread.start()

    live_plot()   # blokira dok se prozor ne zatvori

    print("Čekam na završetak BLE threada...")
    stop_event.set()
    ble_thread.join(timeout=5)
    print("Program završen.")
