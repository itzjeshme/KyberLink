# Assetto Corsa Phone Controller (vJoy edition)

A Windows-based host server that turns your mobile phone into a high-performance gaming controller for *Assetto Corsa*. Using a mobile web app, you can stream steering, throttle, brake, and custom key inputs via WebSockets to your PC, which are then converted into virtual joystick movements via the **vJoy** driver.

## 🚀 Features

*   **Low Latency:** Receives 60Hz JSON streams from your phone for near-instant response.
*   **Virtual Joystick Integration:** Maps steering, throttle, and brake to vJoy axes.
*   **Keyboard Emulation:** Maps custom mobile buttons to actual keyboard keys using `pynput`.
*   **Safety Watchdog:** Automatically centers the steering and releases pedals if the phone disconnects.
*   **Smoothing & Shaping:** Configurable steering expo, deadzones, and pedal ramping to simulate analog feel.
*   **Hotkeys:** Quickly pause/engage the controller using the `F9` key.

---

## 📋 Prerequisites

1.  **Windows OS:** This project uses Windows-specific APIs (via `ctypes`).
2.  **vJoy Driver:** You **must** have the vJoy driver installed. 
    *   Recommended: [The maintained 2.1.9.x fork](https://github.com/jshafer817/vJoy).
    *   **Configuration:** Open "Configure vJoy" and ensure at least one device is enabled with X, Y, and Z axes.
3.  **Python 3.8+**

---

## 🛠️ Installation

1.  **Clone the repository:**
    ```bash
    git clone <your-repo-url>
    cd steering
    ```

2.  **Create and activate a virtual environment:**
    ```bash
    python -m venv .venv
    # On Windows:
    .venv\Scripts\activate
    ```

3.  **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

---

## 🎮 How to Use

1.  **Start the Server:**
    Run the following command in your terminal:
    ```bash
    python server.py
    ```
    The server will display your **Local IP Address** (e.g., `192.168.1.50`).

2.  **Connect your Phone:**
    *   Ensure your phone and PC are on the **same Wi-Fi network**.
    *   Open your phone's browser and navigate to: `http://<YOUR_PC_IP>:8000`
    *   In the app's connection settings, enter your PC's IP and port `8765`.

3.  **Start Driving:**
    Once connected, move your phone to steer and press the pedals. The server console will show real-time telemetry.

---

## ⚙️ Configuration

You can customize the behavior by editing `server.py`. Key settings include:

| Setting | Description |
| :--- | :--- |
| `HTTP_PORT` | The port for the web interface (default: `8000`) |
| `WS_PORT` | The port for WebSocket communication (default: `8765`) |
| `STEER_SMOOTHING` | Time constant for steering smoothing (higher = smoother, but more lag) |
| `STEER_EXPO` | 1.0 = linear; >1.0 = finer control near the center |
| `PAUSE_HOTKEY_NAME` | The key used to toggle the controller (default: `F9`) |

---

## ⚠️ Safety & Controls

*   **Pause/Resume:** Press **`F9`** on your keyboard to toggle the controller. When paused, the wheel centers and pedals release automatically.
*   **Disconnects:** If the WebSocket connection is lost, the **Watchdog** will center the steering and zero the pedals within 0.75 seconds to prevent "runaway" inputs in-game.

