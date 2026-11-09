# Dual USB MPU6050 → Madgwick → 3D arm

Quick classroom prototype using **two ESP8266 D1 Mini + GY-521 MPU6050**, each connected directly to the laptop by USB. The ESPs stream raw 6-axis IMU data at **100 Hz**, and the laptop independently runs the 6-axis Madgwick filter on both streams. **No Wi-Fi** is used.

## Wiring (same on both boards)

| GY-521 MPU6050 | ESP8266 D1 Mini |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | D2 (GPIO4) |
| SCL | D1 (GPIO5) |

Remaining IMU pins unconnected. Always power at 3V3.

## Flash both ESP8266 boards

PlatformIO (`cd firmware`):

```bash
pio run -t upload --upload-port /dev/ttyUSB0
pio run -t upload --upload-port /dev/ttyUSB1
```

Alternatively, use Arduino IDE to open `firmware/arduino/MPU6050Serial/MPU6050Serial.ino`, select a D1 Mini / ESP8266 board, and upload to both USB ports.

**Keep the sensors still during the ~1.5-second gyro bias calibration each time they boot**. Firmware emits CSV as

```
ms,ax,ay,az,gx,gy,gz
```

Units: milliseconds, acceleration in **g**, angular velocity in **degrees/s**.

## Launch the laptop visualization on Ubuntu

Create a virtual environment if desired, then:

```bash
python -m pip install -r laptop/requirements.txt
python -m serial.tools.list_ports -v
ls -l /dev/serial/by-id/
python laptop/arm3d.py --upper /dev/ttyUSB0 --forearm /dev/ttyUSB1
```

Use the actual device paths in your computer; they may instead be `/dev/ttyACM*`. `--upper` must refer to the upper-arm ESP8266. If movement appears on the wrong link, swap the arguments.

**Linux serial access:** If `Permission denied`, ensure your user has permission for the dialout group. On typical Ubuntu setups, `sudo usermod -aG dialout "$USER"`, then log out and in, resolves this. Close Arduino Serial Monitor / PlatformIO monitor before opening the Python viewer, because only one program can use each serial port.

**Try the renderer without hardware:**

```bash
python laptop/arm3d.py --demo
```

Tkinter is required (typically `sudo apt install python3-tk` on Ubuntu if missing). The only PyPI requirement is `pyserial`.

## Controls and calibration

- Wear the two sensors securely, roughly aligning each MPU6050 **+X toward the shoulder/elbow** and **+Y forward** according to your mounting plan.
- After both boards boot, hold the **upper arm and forearm straight vertically downward and stationary**; press **C** or click **Calibrate**.
- Move the arm. Drag with the mouse to orbit the 3D viewpoint; scroll to zoom. Press **C** to re-zero; **Esc** quits.
- For best results start both devices in the same orientation with both sensors' +Y axes pointing forward and +Z axes pointing outward, and avoid sensor slippage.

Calibration mathematically maps each sensor's **reference orientation** to a known downward arm segment. This eliminates a fixed sensor-to-segment mounting offset *provided both filters share a consistent heading frame*. With 6-axis MPU6050s, the initial horizontal reference is inherently ambiguous: there is **no magnetometer to correct yaw drift or relative heading errors**. This is a short-duration **3D demonstration, not medically accurate motion capture**. An initial common pose, matching physical sensor axes, occasional re-zero, and moderate movements help. Independent yaw drift will eventually bend the rendered arm incorrectly; no software-only calibration can recover absolute heading without another reference.

Additional limitations: rapid acceleration can corrupt the gravity estimate, sensor straps can slip, and USB cables restrict motion. The 3D visualization intentionally does not estimate wrist/end-effector orientation.

## Design notes

- ESP8266: I2C read (100 Hz, DLPF enabled), on-boot gyro zero, USB CSV.
- Laptop: two independent serial threads, each with a 6-axis Madgwick orientation quaternion.
- Three Cartesian points are reconstructed by applying each segment's calibrated change in orientation to a known downward reference vector.
- Upper-arm length: 0.35 (arbitrary scene units); forearm: 0.29. These can be edited in `arm3d.py`.
- Renderer: Python standard-library `tkinter` (3D projection onto a 2D canvas), no GPU required.

The gradient-descent IMU update follows the established Madgwick 6-axis algorithm. See https://github.com/arduino-libraries/MadgwickAHRS .
