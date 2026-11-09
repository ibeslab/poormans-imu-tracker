#!/usr/bin/env python3
"""3D two-link arm tracking, using two USB-serial MPU6050s and Madgwick 6-axis.

No separate Madgwick package required. Only 'pyserial' is needed for hardware.
Python's built-in tkinter projects shaded 3D cylinder meshes with an orbiting camera.

    python arm3d.py --upper /dev/ttyUSB0 --forearm /dev/ttyUSB1
    python arm3d.py --demo

Calibrate with C while BOTH arm segments hang straight down.
WARNING: 6-axis IMUs cannot fix heading (yaw); expect heading drift.
"""
import argparse
import math
import threading
import time
import tkinter as tk
from dataclasses import dataclass

try:
    import serial
except ImportError:
    serial = None


def normalize(q):
    n = math.sqrt(sum(v * v for v in q))
    return tuple(v / n for v in q) if n > 1e-12 else (1.0, 0.0, 0.0, 0.0)


def conjugate(q):
    return (q[0], -q[1], -q[2], -q[3])


def multiply(q, r):
    a, b, c, d = q
    w, x, y, z = r
    return (a*w-b*x-c*y-d*z, a*x+b*w+c*z-d*y,
            a*y-b*z+c*w+d*x, a*z+b*y-c*x+d*w)


def rotate(q, v):
    # For a normalized quaternion, perform q * (0, v) * q^(-1).
    p = multiply(multiply(q, (0.0, *v)), conjugate(q))
    return p[1:]


def from_accel(ax, ay, az):
    """Initial sensor-to-world attitude: sensor-specific force -> world +Z.

    Yaw remains inherently arbitrary without a magnetometer; initially the
    two sensors must have approximately matching forward/outward axes.
    """
    norm = math.sqrt(ax*ax + ay*ay + az*az)
    if norm < 0.15:
        return (1.0, 0.0, 0.0, 0.0)
    x, y, z = ax / norm, ay / norm, az / norm
    if z < -0.9999:
        return (0.0, 1.0, 0.0, 0.0)
    return normalize((1.0 + z, y, -x, 0.0))


def axis_angle(axis, angle):
    s = math.sin(angle/2)
    return (math.cos(angle/2), axis[0]*s, axis[1]*s, axis[2]*s)


class Madgwick:
    """Original-style IMU (6-axis) Madgwick update, gyro in deg/s."""
    def __init__(self, beta=0.09):
        self.q = (1.0, 0.0, 0.0, 0.0)
        self.beta = beta
        self.started = False

    def update(self, gx, gy, gz, ax, ay, az, dt):
        if not self.started:
            self.q = from_accel(ax, ay, az)
            self.started = True
        q0, q1, q2, q3 = self.q
        gx, gy, gz = [math.radians(v) for v in (gx, gy, gz)]
        qd0 = .5 * (-q1*gx - q2*gy - q3*gz)
        qd1 = .5 * ( q0*gx + q2*gz - q3*gy)
        qd2 = .5 * ( q0*gy - q1*gz + q3*gx)
        qd3 = .5 * ( q0*gz + q1*gy - q2*gx)

        a2 = ax*ax + ay*ay + az*az
        if a2 > 1e-9:
            inv = 1.0 / math.sqrt(a2)
            ax, ay, az = ax*inv, ay*inv, az*inv
            _2q0, _2q1, _2q2, _2q3 = 2*q0, 2*q1, 2*q2, 2*q3
            _4q0, _4q1, _4q2 = 4*q0, 4*q1, 4*q2
            _8q1, _8q2 = 8*q1, 8*q2
            q00, q11, q22, q33 = q0*q0, q1*q1, q2*q2, q3*q3
            s0 = _4q0*q22 + _2q2*ax + _4q0*q11 - _2q1*ay
            s1 = (_4q1*q33 - _2q3*ax + 4*q00*q1 - _2q0*ay - _4q1
                  + _8q1*q11 + _8q1*q22 + _4q1*az)
            s2 = (4*q00*q2 + _2q0*ax + _4q2*q33 - _2q3*ay - _4q2
                  + _8q2*q11 + _8q2*q22 + _4q2*az)
            s3 = 4*q11*q3 - _2q1*ax + 4*q22*q3 - _2q2*ay
            sn = math.sqrt(s0*s0+s1*s1+s2*s2+s3*s3)
            if sn > 1e-10:
                inv_s = 1.0 / sn
                qd0 -= self.beta*s0*inv_s
                qd1 -= self.beta*s1*inv_s
                qd2 -= self.beta*s2*inv_s
                qd3 -= self.beta*s3*inv_s
        self.q = normalize((q0+qd0*dt, q1+qd1*dt, q2+qd2*dt, q3+qd3*dt))
        return self.q


@dataclass
class Snapshot:
    q: tuple = (1.0, 0.0, 0.0, 0.0)
    count: int = 0
    timestamp: float = 0.0
    error: str = ''


class SensorReader:
    def __init__(self, port, name):
        self.port = port
        self.name = name
        self.lock = threading.Lock()
        self.data = Snapshot()
        self.filter = Madgwick(beta=.09)
        self.last_ms = None
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return Snapshot(self.data.q, self.data.count,
                            self.data.timestamp, self.data.error)

    def _loop(self):
        try:
            with serial.Serial(self.port, 115200, timeout=.25) as conn:
                # Opening a serial port may reset the ESP8266; initial '#' lines
                # and the first ~1.5s of gyro calibration are expected.
                while not self.stopping.is_set():
                    line = conn.readline().decode('ascii', errors='replace').strip()
                    if not line or line.startswith('#'):
                        continue
                    parts = line.split(',')
                    if len(parts) != 7:
                        continue
                    try:
                        ms = int(parts[0])
                        ax, ay, az, gx, gy, gz = map(float, parts[1:])
                    except ValueError:
                        continue
                    if self.last_ms is None:
                        dt = .01
                    else:
                        dt = ((ms - self.last_ms) & 0xFFFFFFFF) / 1000.0
                        if not (.004 < dt < .040):
                            dt = .01
                    self.last_ms = ms
                    q = self.filter.update(gx, gy, gz, ax, ay, az, dt)
                    with self.lock:
                        self.data.q = q
                        self.data.count += 1
                        self.data.timestamp = time.monotonic()
        except (OSError, Exception) as exc:
            with self.lock:
                self.data.error = str(exc)

    def stop(self):
        self.stopping.set()


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(v, s):
    return tuple(x*s for x in v)


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def subtract(a, b):
    return tuple(x-y for x, y in zip(a, b))


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1],
            a[2]*b[0]-a[0]*b[2],
            a[0]*b[1]-a[1]*b[0])


def unit(v):
    length = math.sqrt(dot(v, v))
    return scale(v, 1.0/length) if length > 1e-12 else (0.0, 0.0, 0.0)


def shade_color(rgb, brightness):
    # Material diffuse shading for Tk Canvas; no OpenGL dependency.
    return '#%02x%02x%02x' % tuple(
        max(0, min(255, round(c*brightness))) for c in rgb)


class Viewer:
    def __init__(self, root, upper, forearm, demo):
        self.root = root
        self.upper = upper
        self.forearm = forearm
        self.demo = demo
        self.ref_upper = None
        self.ref_forearm = None
        self.azimuth = math.radians(38)
        self.elevation = math.radians(27)
        self.zoom = 1.0
        self.drag = None
        self.t0 = time.monotonic()
        self.status_var = tk.StringVar(value='Press C to calibrate with both arms straight down')
        root.title('Dual MPU6050 | 3D Arm | Madgwick 6-axis | USB Serial')
        root.geometry('1000x740')
        root.minsize(660, 510)
        bg = '#101722'
        self.canvas = tk.Canvas(root, bg=bg, highlightthickness=0)
        self.canvas.pack(fill='both', expand=True)
        bar = tk.Frame(root, bg='#182333')
        bar.pack(fill='x')
        tk.Button(bar, text='Calibrate [C]', command=self.calibrate).pack(side='left', padx=10, pady=8)
        tk.Label(bar, textvariable=self.status_var, fg='#e6edf7', bg='#182333',
                 anchor='w').pack(side='left', padx=10, fill='x', expand=True)
        self.canvas.bind('<ButtonPress-1>', self.mouse_down)
        self.canvas.bind('<B1-Motion>', self.mouse_move)
        self.canvas.bind('<MouseWheel>', self.mouse_wheel)
        self.canvas.bind('<Button-4>', lambda e: self.change_zoom(1.10))
        self.canvas.bind('<Button-5>', lambda e: self.change_zoom(1/1.10))
        root.bind_all('<c>', lambda e: self.calibrate())
        root.bind_all('<C>', lambda e: self.calibrate())
        root.bind_all('<Escape>', lambda e: self.close())
        root.protocol('WM_DELETE_WINDOW', self.close)
        self.canvas.after(33, self.render)

    def close(self):
        if self.upper: self.upper.stop()
        if self.forearm: self.forearm.stop()
        self.root.destroy()

    def mouse_down(self, evt):
        self.drag = (evt.x, evt.y)

    def mouse_move(self, evt):
        if self.drag is not None:
            dx, dy = evt.x-self.drag[0], evt.y-self.drag[1]
            self.azimuth += dx*.009
            self.elevation = max(math.radians(-80), min(math.radians(80),
                                                         self.elevation + dy*.009))
            self.drag = (evt.x, evt.y)

    def change_zoom(self, f):
        self.zoom = max(.5, min(2.5, self.zoom*f))

    def mouse_wheel(self, evt):
        self.change_zoom(1.10 if evt.delta > 0 else 1/1.10)

    def quats(self):
        if self.demo:
            t = time.monotonic()-self.t0
            yaw = axis_angle((0,0,1), .8*math.sin(t*.32))
            shoulder = multiply(yaw, multiply(axis_angle((0,1,0),
                             .5*math.sin(t*.85)), axis_angle((1,0,0),
                             .35*math.sin(t*.51))))
            elbow = multiply(shoulder, axis_angle((0,1,0),
                              .9 + .30*math.sin(t*1.13)))
            return Snapshot(shoulder, 100, time.monotonic()), Snapshot(elbow, 100, time.monotonic())
        return self.upper.snapshot(), self.forearm.snapshot()

    def calibrate(self):
        a, b = self.quats()
        if a.count < 20 or b.count < 20:
            self.status_var.set('No fresh samples yet: check serial ports / wait for gyro calibration')
            return
        if not self.demo and (time.monotonic()-a.timestamp > .8 or
                              time.monotonic()-b.timestamp > .8):
            self.status_var.set('Cannot calibrate: one or both IMUs have no recent data')
            return
        self.ref_upper, self.ref_forearm = a.q, b.q
        self.status_var.set('Calibrated. Move your arm; press C again to re-zero')

    def project(self, p, w, h):
        # Z-up world -> rotatable orthographic 2D projection
        az, el = self.azimuth, self.elevation
        rel = (p[0], p[1], p[2]+.31)
        right = (-math.sin(az), math.cos(az), 0)
        up = (-math.sin(el)*math.cos(az), -math.sin(el)*math.sin(az), math.cos(el))
        s = min(w*.50, h*.90)*self.zoom
        return (w*.51 + s*dot(rel,right), h*.51 - s*dot(rel,up))

    def camera_direction(self):
        # World-space unit vector pointing from the model toward the camera.
        az, el = self.azimuth, self.elevation
        return (math.cos(el)*math.cos(az),
                math.cos(el)*math.sin(az), math.sin(el))

    def cylinder_faces(self, a, b, radius, rgb, sides=16):
        """Make camera-visible polygon faces for a capped 3D cylinder.

        Each entry is (depth, [vertices...], color). Render farther faces
        first (painter's algorithm). All geometry is in world coordinates.
        """
        direction = subtract(b, a)
        length = math.sqrt(dot(direction, direction))
        if length < 1e-8:
            return []
        axis = scale(direction, 1.0/length)
        # Pick a reference away from the axis so vertical arms remain stable.
        ref = (1.0, 0.0, 0.0) if abs(axis[2]) > .85 else (0.0, 0.0, 1.0)
        u = unit(cross(axis, ref))
        v = cross(axis, u)
        rings = [[], []]
        radial = []
        for i in range(sides):
            t = 2*math.pi*i/sides
            r = add(scale(u, math.cos(t)), scale(v, math.sin(t)))
            radial.append(r)
            rings[0].append(add(a, scale(r, radius)))
            rings[1].append(add(b, scale(r, radius)))

        eye = self.camera_direction()
        light = unit((-.35, -.55, 1.0))
        faces = []

        def add_face(vertices, normal):
            # Back-face culling: only draw surfaces facing the viewer.
            if dot(normal, eye) <= .00001:
                return
            brightness = .48 + .43*max(0.0, dot(normal, light)) + .10*max(0.0, dot(normal, eye))
            midpoint = tuple(sum(vertex[k] for vertex in vertices)/len(vertices) for k in range(3))
            faces.append((dot(midpoint, eye), vertices, shade_color(rgb, brightness)))

        for i in range(sides):
            j = (i+1) % sides
            normal = unit(add(radial[i], radial[j]))
            add_face([rings[0][i], rings[0][j], rings[1][j], rings[1][i]], normal)
        # End caps can become visible as the arm moves toward the camera.
        add_face(list(reversed(rings[0])), scale(axis, -1))
        add_face(rings[1], axis)
        return faces

    def draw_cylinders(self, segments):
        """Draw projected 3D meshes, retaining the existing Tkinter viewer."""
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        faces = []
        for a, b, radius, color in segments:
            faces.extend(self.cylinder_faces(a, b, radius, color))
        for _depth, vertices, color in sorted(faces, key=lambda f: f[0]):
            points = []
            for p in vertices:
                x, y = self.project(p, w, h)
                points.extend((x, y))
            self.canvas.create_polygon(*points, fill=color, outline=color)

    def line(self, a, b, color, width=2, dash=None):
        w, h = self.canvas.winfo_width(), self.canvas.winfo_height()
        x1,y1 = self.project(a,w,h)
        x2,y2 = self.project(b,w,h)
        self.canvas.create_line(x1,y1,x2,y2,fill=color,width=width,
                                capstyle='round', dash=dash)

    def joint(self, pos, color, radius=7):
        w,h=self.canvas.winfo_width(),self.canvas.winfo_height()
        x,y=self.project(pos,w,h)
        self.canvas.create_oval(x-radius,y-radius,x+radius,y+radius,
                                fill=color,outline='#e8f1ff',width=1.5)

    def label3d(self, pos, text, color='#e8f1ff'):
        w,h=self.canvas.winfo_width(),self.canvas.winfo_height()
        x,y=self.project(pos,w,h)
        self.canvas.create_text(x+10,y-10,text=text,anchor='w',fill=color,
                                font=('DejaVu Sans',11,'bold'))

    def render(self):
        c=self.canvas
        c.delete('all')
        w,h=c.winfo_width(),c.winfo_height()
        if w<20 or h<20:
            c.after(33,self.render)
            return
        # Ground plane: visual reference only; not a constraint on the wrist.
        for i in range(-4,5):
            x=i*.14
            self.line((x,-.56,-.85),(x,.56,-.85),'#263343',1)
            self.line((-.56,x,-.85),(.56,x,-.85),'#263343',1)
        self.line((0,0,0),(.5,0,0),'#eb6b6b',2)
        self.line((0,0,0),(0,.5,0),'#52d58f',2)
        self.line((0,0,0),(0,0,.5),'#67a9ff',2)
        for p,lab in [((.53,0,0),'X'),((0,.53,0),'Y'),((0,0,.53),'Z')]:
            self.label3d(p,lab,'#8fa7c2')

        a,b=self.quats()
        # Both reference IMUs must be in the same known pose with upper and
        # forearm extending down in the displayed world frame.
        q1 = multiply(a.q,conjugate(self.ref_upper)) if self.ref_upper else (1,0,0,0)
        q2 = multiply(b.q,conjugate(self.ref_forearm)) if self.ref_forearm else (1,0,0,0)
        d1 = rotate(q1,(0,0,-1))
        d2 = rotate(q2,(0,0,-1))
        shoulder=(0,0,0)
        elbow=scale(d1,.35)
        wrist=add(elbow,scale(d2,.29))
        # Actual cylindrical 3D geometry (radius in scene/world units).
        # Upper arm is slightly thicker than forearm.
        self.draw_cylinders([
            (shoulder, elbow, .029, (76, 152, 245)),
            (elbow, wrist, .024, (53, 220, 175)),
        ])
        self.joint(shoulder,'#b9d6ff',8)
        self.joint(elbow,'#c9dae8',7)
        self.joint(wrist,'#87ffe4',6)
        for p,lab in [(shoulder,'SHOULDER'),(elbow,'ELBOW'),(wrist,'WRIST')]:
            self.label3d(p,lab)

        elbow_angle=math.degrees(math.acos(max(-1,min(1,dot(d1,d2)))))
        c.create_text(20,15,anchor='nw',fill='#f0f6ff',
                      text='3D TWO-LINK ARM  |  MADGWICK 6-AXIS',
                      font=('DejaVu Sans',15,'bold'))
        c.create_text(20,48,anchor='nw',fill='#b7c9df',
                      text=f"Elbow flexion (segment angle): {elbow_angle:5.1f}°",
                      font=('DejaVu Sans',12))
        if self.demo:
            status='DEMO MODE (simulated orientations)'
        else:
            now=time.monotonic()
            def status_for(s,port):
                if s.error:return f'{port}: ERROR {s.error[:52]}'
                if s.count==0:return f'{port}: connecting / gyro calibration...'
                age=now-s.timestamp
                return f'{port}: {s.count} samples, {age:.1f}s old' + (' (STALE)' if age>.5 else '')
            status=status_for(a,'UPPER')+'\n'+status_for(b,'FOREARM')
        c.create_text(20,80,anchor='nw',fill='#99aac1',text=status,
                      font=('DejaVu Sans',10),justify='left')
        if self.ref_upper is None:
            c.create_text(w/2,h*.80,fill='#ffcc88',
                          text='Keep BOTH arms straight downward, then press C',
                          font=('DejaVu Sans',12,'bold'))
        c.create_text(18,h-14,anchor='sw',fill='#71869f',
                      text='Drag = orbit camera   |   Scroll = zoom   |   C = calibrate   |   Esc = quit',
                      font=('DejaVu Sans',10))
        c.after(33,self.render)


def main():
    ap=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--upper',help='Upper-arm USB serial port, e.g. /dev/ttyUSB0')
    ap.add_argument('--forearm',help='Forearm USB serial port, e.g. /dev/ttyUSB1')
    ap.add_argument('--demo',action='store_true',help='Animate synthetic arm without hardware')
    args=ap.parse_args()
    if not args.demo:
        if not args.upper or not args.forearm or args.upper==args.forearm:
            ap.error('provide two DIFFERENT ports with --upper and --forearm (or --demo)')
        if serial is None:
            ap.error('pyserial is missing: python -m pip install pyserial')
    upper = SensorReader(args.upper,'upper') if not args.demo else None
    forearm = SensorReader(args.forearm,'forearm') if not args.demo else None
    if upper: upper.start()
    if forearm: forearm.start()
    root=tk.Tk()
    Viewer(root,upper,forearm,args.demo)
    root.mainloop()


if __name__=='__main__':
    main()
