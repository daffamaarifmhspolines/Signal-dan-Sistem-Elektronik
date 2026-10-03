"""
perbandingan_adaptif.py - Grafik PID vs Adaptive PID (untuk Gambar 4.6).
Letakkan skrip ini satu folder dengan CSV, lalu jalankan:  python perbandingan_adaptif.py
Hasil: perbandingan_adaptif.png  (waktu disejajarkan: t = 0 saat set point naik)
"""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

FILES = [
    ("bonusPID.csv", "PID (Kp=1; Ki=0,5)", "tab:blue", "-"),
    ("bonusAdaptive.csv", "Adaptive v1", "tab:red", "--"),
    ("bonusAdaptiveV2.csv", "Adaptive v2", "tab:green", "-."),
]


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    g = lambda k: np.array([float(r[k]) for r in rows])
    t, sp, pv, u = g("t_s"), g("sp_V"), g("pv_V"), g("u_V")
    i0 = int(np.argmax(sp > 0.5))
    return t - t[i0], sp, pv, u


fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 7), sharex=True, gridspec_kw={"height_ratios": [3, 2]})
for k, (fn, lab, c, ls) in enumerate(FILES):
    t, sp, pv, u = load(fn)
    a1.plot(t, pv, color=c, ls=ls, lw=1.9, label=lab)
    a2.plot(t, u, color=c, ls=ls, lw=1.4, label=lab)
    if k == 0:
        a1.step(t, sp, where="post", color="k", ls=":", lw=1.2, label="Set Point")
a1.set_ylabel("ADC / PV (V)"); a1.set_ylim(-0.1, 2.5); a1.legend(loc="lower right"); a1.grid(alpha=.3)
a1.set_title("Perbandingan PID dan Adaptive PID (lompatan 0 → 2 V)")
a2.set_ylabel("Keluaran DAC u (V)"); a2.set_ylim(-0.1, 3.5); a2.grid(alpha=.3); a2.legend(loc="upper right")
a2.set_xlabel("Waktu sejak set point naik (s)"); a2.set_xlim(-2, 25)
plt.tight_layout(); plt.savefig("perbandingan_adaptif.png", dpi=200)
print("Tersimpan: perbandingan_adaptif.png")
