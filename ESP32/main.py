# main.py - ESP32 Web-Based Intelligent Control System (MicroPython)
# Sinyal dan Sistem Kontrol Elektronik
#
#   Browser --Wi-Fi--> ESP32 (Web Server) -> Controller -> DAC GPIO25 -> PLANT (RC)
#                                                ^                         |
#                                                +----- ADC GPIO33 <-------+
#
# Sampling kontroler : 500 ms
# Controller         : PID | ADAPTIVE (v1) | ADAPTIVE2 (v2, gain scheduling perbaikan) | OPEN (open-loop)
#
# Upload ke ESP32 : main.py  dan  index.html  (keduanya di root filesystem)

import network
import time
import gc
import json
import uasyncio as asyncio
from machine import Pin, ADC, DAC
from array import array

# ======================= KONFIGURASI =======================
WIFI_SSID = "Nama Wifi"          # <-- ganti
WIFI_PASS = "Password Wifi"      # <-- ganti
AP_SSID = "ESP32-CTRL"           # dipakai jika gagal terhubung ke WiFi (mode Access Point)
AP_PASS = "12345678"

TS_MS = 500                      # periode sampling (ms) -> sesuai jobsheet
TS = TS_MS / 1000.0
N_LOG = 300                      # jumlah data historis (100-300 titik)
V_MAX = 3.3                      # tegangan maksimum DAC/ADC
SP_MAX = 3.3                     # batas setpoint (turunkan jika rangkaian butuh batas aman)

# Parameter awal controller
KP0, KI0, KD0 = 1.0, 0.5, 0.0
E_REF = 1.0                      # error (V) yang dianggap "besar" pada Adaptive PID
# ===========================================================

# ---------- Perangkat keras ----------
adc = ADC(Pin(33))               # GPIO33 = ADC1_CH5 (aman dipakai bersama Wi-Fi)
adc.atten(ADC.ATTN_11DB)         # rentang ~0 - 3.3 V
try:
    adc.width(ADC.WIDTH_12BIT)
except Exception:
    pass
HAS_UV = hasattr(adc, "read_uv")  # read_uv() = tegangan terkalibrasi (MicroPython baru)

dac = DAC(Pin(25))               # GPIO25 = DAC1 (8-bit, 0..255)


def read_v():
    """Baca tegangan ADC (rata-rata 16 sampel untuk menekan noise)."""
    s = 0
    if HAS_UV:
        for _ in range(16):
            s += adc.read_uv()
        return s / 16 / 1e6
    for _ in range(16):
        s += adc.read()
    return s / 16 * 3.3 / 4095


def write_dac(u):
    """Tulis tegangan u (V) ke DAC. Return tegangan nyata setelah kuantisasi 8-bit."""
    v = int(u / V_MAX * 255 + 0.5)
    if v < 0:
        v = 0
    elif v > 255:
        v = 255
    dac.write(v)
    return v


# ---------- Controller ----------
class PID:
    """PID posisi dengan: derivative-on-measurement + low-pass filter,
    output limiting, dan anti-windup (conditional integration)."""

    def __init__(self, umin, umax, dt, alpha=0.6):
        self.umin = umin
        self.umax = umax
        self.dt = dt
        self.alpha = alpha       # filter derivatif (0 = tanpa filter)
        self.reset()

    def reset(self):
        self.i = 0.0
        self.d = 0.0
        self.prev_pv = None

    def update(self, sp, pv, kp, ki, kd):
        e = sp - pv
        if self.prev_pv is None:
            self.prev_pv = pv
        # Derivative atas pengukuran (menghindari "derivative kick" saat SP melompat)
        d_raw = -(pv - self.prev_pv) / self.dt
        self.d = self.alpha * self.d + (1 - self.alpha) * d_raw
        self.prev_pv = pv

        p = kp * e
        i_new = self.i + ki * e * self.dt
        u = p + i_new + kd * self.d

        # Output limiting + anti-windup
        if u > self.umax:
            uc = self.umax
            if e < 0:
                self.i = i_new
        elif u < self.umin:
            uc = self.umin
            if e > 0:
                self.i = i_new
        else:
            uc = u
            self.i = i_new
        if self.i > self.umax:
            self.i = self.umax
        elif self.i < self.umin:
            self.i = self.umin
        return uc


def adaptive_gains(e, kp0, ki0, kd0):
    """Adaptive PID (gain scheduling berdasarkan |error|).
    Error besar  -> Kp dinaikkan (respons cepat), Ki diturunkan (cegah overshoot/windup)
    Error kecil  -> Kp sedikit turun, Ki dinaikkan (hilangkan steady-state error)"""
    f = abs(e) / E_REF
    if f > 1:
        f = 1.0
    kp = kp0 * (0.8 + 0.7 * f)
    ki = ki0 * (1.5 - 1.2 * f)
    return kp, ki, kd0


def adaptive2_gains(e, kp0, ki0, kd0):
    """Adaptive PID v2 (gain scheduling, Ti dijaga mendekati konstanta waktu plant).
    Hasil perbaikan v1: Ki TIDAK diturunkan saat error besar (itu membuat respons merayap lambat).
    Error besar -> Kp rendah (1.5*Kp0): hindari jenuh DAC dan overshoot
    Error kecil -> Kp tinggi (3*Kp0)  : koreksi cepat mendekati set point
    Ki tetap 2.5*Ki0 -> rasio Ti = Kp/Ki tetap dekat tau plant."""
    f = abs(e) / E_REF
    if f > 1:
        f = 1.0
    return kp0 * (3.0 - 1.5 * f), ki0 * 2.5, kd0


pid = PID(0.0, V_MAX, TS)

# ---------- State & log ----------
st = {
    "sp": 0.0, "pv": 0.0, "e": 0.0, "u": 0.0, "dac": 0,
    "mode": "PID", "run": 0,
    "kp": KP0, "ki": KI0, "kd": KD0,
    "ekp": KP0, "eki": KI0, "ekd": KD0,   # gain efektif (berubah pada ADAPTIVE)
    "t": 0.0, "n": 0, "ip": "-", "net": "-",
}
t_start = time.ticks_ms()


class Ring:
    def __init__(self, n):
        self.n = n
        self.cols = [array("f", [0.0] * n) for _ in range(6)]  # t, sp, pv, e, u, dacV
        self.i = 0
        self.cnt = 0

    def clear(self):
        self.i = 0
        self.cnt = 0

    def add(self, *vals):
        for c, v in zip(self.cols, vals):
            c[self.i] = v
        self.i = (self.i + 1) % self.n
        if self.cnt < self.n:
            self.cnt += 1

    def idx(self):
        start = (self.i - self.cnt) % self.n
        for k in range(self.cnt):
            yield (start + k) % self.n


log = Ring(N_LOG)


def control_step():
    global t_start
    pv = read_v()
    sp = st["sp"]
    e = sp - pv

    if st["run"]:
        if st["mode"] == "OPEN":
            u = sp
            st["ekp"], st["eki"], st["ekd"] = 0.0, 0.0, 0.0
        else:
            kp, ki, kd = st["kp"], st["ki"], st["kd"]
            if st["mode"] == "ADAPTIVE":
                kp, ki, kd = adaptive_gains(e, kp, ki, kd)
            elif st["mode"] == "ADAPTIVE2":
                kp, ki, kd = adaptive2_gains(e, kp, ki, kd)
            st["ekp"], st["eki"], st["ekd"] = kp, ki, kd
            u = pid.update(sp, pv, kp, ki, kd)
    else:
        u = 0.0
        pid.reset()

    if u < 0:
        u = 0.0
    elif u > V_MAX:
        u = V_MAX
    dv = write_dac(u)
    dac_v = dv * V_MAX / 255

    t = time.ticks_diff(time.ticks_ms(), t_start) / 1000.0
    st.update(pv=pv, e=e, u=u, dac=dv, t=t, n=st["n"] + 1)
    log.add(t, sp, pv, e, u, dac_v)


async def control_loop():
    nxt = time.ticks_add(time.ticks_ms(), TS_MS)
    while True:
        d = time.ticks_diff(nxt, time.ticks_ms())
        if d > 0:
            await asyncio.sleep_ms(d)
        elif d < -TS_MS:                       # tertinggal terlalu jauh -> sinkron ulang
            nxt = time.ticks_ms()
        nxt = time.ticks_add(nxt, TS_MS)
        control_step()


# ---------- Wi-Fi ----------
def connect_wifi():
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    sta.connect(WIFI_SSID, WIFI_PASS)
    t0 = time.ticks_ms()
    while not sta.isconnected() and time.ticks_diff(time.ticks_ms(), t0) < 15000:
        time.sleep_ms(200)
    if sta.isconnected():
        return sta.ifconfig()[0], "STA"
    sta.active(False)
    ap = network.WLAN(network.AP_IF)
    ap.active(True)
    ap.config(essid=AP_SSID, password=AP_PASS, authmode=network.AUTH_WPA_WPA2_PSK)
    return ap.ifconfig()[0], "AP"


# ---------- Web server ----------
def parse_q(path):
    q = {}
    if "?" in path:
        path, qs = path.split("?", 1)
        for kv in qs.split("&"):
            if "=" in kv:
                k, v = kv.split("=", 1)
                q[k] = v.replace(",", ".")
    return path, q


def fnum(q, key, lo, hi):
    try:
        v = float(q[key])
    except Exception:
        return None
    return lo if v < lo else hi if v > hi else v


def state_json():
    s = {}
    for k, v in st.items():
        s[k] = round(v, 4) if isinstance(v, float) else v
    return json.dumps(s)


def hist_json():
    out = [[], [], [], []]
    t, sp, pv, e, u, dv = log.cols
    for i in log.idx():
        out[0].append(round(t[i], 1))
        out[1].append(round(sp[i], 3))
        out[2].append(round(pv[i], 3))
        out[3].append(round(u[i], 3))
    return json.dumps({"t": out[0], "sp": out[1], "pv": out[2], "u": out[3]})


async def send(w, body, ctype="application/json", extra=""):
    w.write("HTTP/1.0 200 OK\r\nContent-Type: %s\r\nCache-Control: no-store\r\n%sConnection: close\r\n\r\n" % (ctype, extra))
    w.write(body)
    await w.drain()


async def handle(r, w):
    global t_start
    try:
        line = await r.readline()
        while True:                              # buang header
            h = await r.readline()
            if not h or h == b"\r\n":
                break
        parts = line.decode().split()
        if len(parts) < 2:
            return
        path, q = parse_q(parts[1])

        if path == "/" or path == "/index.html":
            w.write("HTTP/1.0 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nConnection: close\r\n\r\n")
            with open("index.html", "rb") as f:
                while True:
                    b = f.read(1024)
                    if not b:
                        break
                    w.write(b)
                    await w.drain()

        elif path == "/api/state":
            await send(w, state_json())

        elif path == "/api/hist":
            await send(w, hist_json())

        elif path == "/api/set":
            v = fnum(q, "sp", 0.0, SP_MAX)
            if v is not None:
                st["sp"] = v
            await send(w, state_json())

        elif path == "/api/cfg":
            if q.get("mode") in ("PID", "ADAPTIVE", "ADAPTIVE2", "OPEN"):
                if q["mode"] != st["mode"]:
                    pid.reset()
                st["mode"] = q["mode"]
            for k in ("kp", "ki", "kd"):
                v = fnum(q, k, 0.0, 100.0)
                if v is not None:
                    st[k] = v
            await send(w, state_json())

        elif path == "/api/run":
            st["run"] = 1 if q.get("v") == "1" else 0
            if not st["run"]:
                pid.reset()
            await send(w, state_json())

        elif path == "/api/reset":
            pid.reset()
            log.clear()
            t_start = time.ticks_ms()
            st["sp"] = 0.0
            st["t"] = 0.0
            await send(w, state_json())

        elif path == "/api/csv":
            await send(w, "", "text/csv", 'Content-Disposition: attachment; filename="data_kontrol.csv"\r\n')
            w.write("t_s,sp_V,pv_V,error_V,u_V,dac_V\r\n")
            rows = ""
            c = 0
            for i in log.idx():
                rows += "%.1f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n" % tuple(col[i] for col in log.cols)
                c += 1
                if c % 20 == 0:
                    w.write(rows)
                    await w.drain()
                    rows = ""
            w.write(rows)
            await w.drain()

        else:
            w.write("HTTP/1.0 404 Not Found\r\n\r\nNot found")
            await w.drain()
    except Exception as ex:
        print("HTTP err:", ex)
    finally:
        try:
            w.close()
            await w.wait_closed()
        except Exception:
            pass
        gc.collect()


async def main():
    ip, net = connect_wifi()
    st["ip"], st["net"] = ip, net
    print("Mode:", net, "| Buka di browser: http://%s/" % ip)
    await asyncio.start_server(handle, "0.0.0.0", 80)
    asyncio.create_task(control_loop())
    while True:
        await asyncio.sleep(5)
        gc.collect()


asyncio.run(main())
