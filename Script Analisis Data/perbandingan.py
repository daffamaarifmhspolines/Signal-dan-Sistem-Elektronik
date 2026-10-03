"""
perbandingan.py - Gabungkan beberapa CSV menjadi satu grafik perbandingan (untuk Gambar 4.5).

Pemakaian (di PC, semua file CSV berada di folder yang sama dengan skrip ini):
    python perbandingan.py
Hasil: perbandingan_parameter.png

Edit daftar GRUP di bawah jika nama file / nilai parameter berbeda.
Waktu tiap respons disejajarkan: t = 0 saat set point naik.
"""
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# (nama file, label di legenda)
GRUP = {
    "(a) Variasi Kp  (Ki = 0,5; Kd = 0)": [
        ("kp05ki05kd0.csv", "Kp = 0,5"),
        ("kp1ki05kd0.csv", "Kp = 1,0"),
        ("kp3ki05kd0.csv", "Kp = 3,0"),
    ],
    "(b) Variasi Ki  (Kp = 1; Kd = 0)": [
        ("kp1ki0kd0.csv", "Ki = 0"),
        ("kp1ki05kd0.csv", "Ki = 0,5"),
        ("kp1ki15kd0.csv", "Ki = 1,5"),
    ],
}
STYLE = [("tab:blue", "-"), ("tab:red", "--"), ("tab:green", "-.")]  # warna + gaya garis berbeda
SP_STEP = 0.5          # ambang untuk mendeteksi set point naik (V)
T_MIN, T_MAX = -2, 25  # rentang waktu yang ditampilkan (s)


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    g = lambda k: np.array([float(r[k]) for r in rows])
    t, sp, pv, u = g("t_s"), g("sp_V"), g("pv_V"), g("u_V")
    i0 = int(np.argmax(sp > SP_STEP))
    return t - t[i0], sp, pv, u


fig, axs = plt.subplots(2, 2, figsize=(11, 7), sharex=True)
for col, (judul, items) in enumerate(GRUP.items()):
    ax_pv, ax_u = axs[0, col], axs[1, col]
    for k, (fn, lab) in enumerate(items):
        t, sp, pv, u = load(fn)
        c, ls = STYLE[k]
        ax_pv.plot(t, pv, color=c, ls=ls, lw=1.8, label=lab)
        ax_u.plot(t, u, color=c, ls=ls, lw=1.4, label=lab)
        if k == 0:
            ax_pv.step(t, sp, where="post", color="k", ls=":", lw=1.2, label="Set Point")
    ax_pv.set_title(judul, fontsize=11)
    ax_pv.set_ylabel("ADC / PV (V)")
    ax_u.set_ylabel("Keluaran DAC u (V)")
    ax_u.set_xlabel("Waktu sejak set point naik (s)")
    for ax in (ax_pv, ax_u):
        ax.grid(alpha=0.3)
        ax.set_xlim(T_MIN, T_MAX)
        ax.legend(fontsize=9, loc="lower right" if ax is ax_pv else "upper right")
    ax_pv.set_ylim(-0.1, 2.5)
    ax_u.set_ylim(-0.1, 3.5)
plt.tight_layout()
plt.savefig("perbandingan_parameter.png", dpi=200)
print("Tersimpan: perbandingan_parameter.png")
