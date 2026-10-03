"""
analisis.py - Hitung error, overshoot, rise time, settling time, dan steady-state error
dari file CSV yang di-download lewat GUI ESP32 (tombol "Download CSV").

Pemakaian (di PC):
    pip install numpy matplotlib
    python analisis.py data_kontrol.csv          # loop tertutup (PID/Adaptive)
    python analisis.py data_kontrol.csv --open   # uji plant mode OPEN (acuan = nilai akhir PV, awal = saat DAC naik)

Hasil: tabel di terminal, metrik.csv, dan grafik respons.png
"""
import csv
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

TS = 0.5            # periode sampling (s)
TOL_FRAC = 0.02     # toleransi settling +-2 %
TOL_MIN = 0.03      # toleransi minimum (V) agar tidak lebih kecil dari noise ADC


def load(path):
    with open(path, newline="") as f:
        rd = csv.DictReader(f)
        rows = [r for r in rd]
    g = lambda k: np.array([float(r[k]) for r in rows])
    return g("t_s"), g("sp_V"), g("pv_V"), g("error_V"), g("u_V")


def analyse(t, sp, pv, i0, i1, ref=None):
    """Metrik untuk satu segmen [i0, i1). ref = nilai akhir acuan (mode --open)."""
    sp0, sp1 = pv[i0], (sp[i0] if ref is None else ref)   # nilai awal = PV saat langkah dimulai
    d = sp1 - sp0
    seg_t, seg = t[i0:i1] - t[i0], pv[i0:i1]
    if abs(d) < 0.05 or len(seg) < 4:
        return None
    sgn = np.sign(d)
    peak = seg.max() if sgn > 0 else seg.min()
    mp = max(0.0, sgn * (peak - sp1) / sp1 * 100) if sp1 > 0 else float("nan")

    def cross(level):
        for k in range(1, len(seg)):
            if (seg[k - 1] - level) * sgn < 0 <= (seg[k] - level) * sgn:
                return seg_t[k - 1] + (level - seg[k - 1]) / (seg[k] - seg[k - 1]) * (seg_t[k] - seg_t[k - 1])
        return float("nan")

    t10, t90 = cross(sp0 + 0.1 * d), cross(sp0 + 0.9 * d)
    tol = max(TOL_FRAC * abs(sp1), TOL_MIN)
    out = np.where(np.abs(seg - sp1) > tol)[0]
    last_out = out[-1] if len(out) else -1
    if last_out < len(seg) - 1 and len(seg) >= 10:
        ts = seg_t[last_out + 1]
        ess = sp1 - seg[-5:].mean()
    else:
        ts, ess = float("nan"), float("nan")
    return dict(sp_awal=sp0, sp_akhir=sp1, overshoot_pct=mp, rise_s=t90 - t10,
                settling_s=ts, ess_V=ess, pv_max=seg.max(), n=len(seg))


def main(path, open_loop=False):
    t, sp, pv, e, u = load(path)
    key = u if open_loop else sp
    idx = [0] + [i for i in range(1, len(key)) if abs(key[i] - key[i - 1]) > 1e-6] + [len(key)]
    res = []
    for a, b in zip(idx[:-1], idx[1:]):
        ref = pv[(a + b) // 2:b].mean() if open_loop else None
        m = analyse(t, sp, pv, a, b, ref)
        if m:
            res.append(m)

    hdr = ["sp_awal", "sp_akhir", "overshoot_pct", "rise_s", "settling_s", "ess_V", "pv_max", "n"]
    print("\n%-8s %-8s %-10s %-8s %-10s %-8s %-8s %-4s" % tuple(hdr))
    for m in res:
        print("%-8.2f %-8.2f %-10.2f %-8.2f %-10.2f %-8.3f %-8.3f %-4d" % tuple(m[h] for h in hdr))
    with open("metrik.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=hdr)
        w.writeheader()
        w.writerows([{h: m[h] for h in hdr} for m in res])

    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    ax[0].step(t, sp, "--", color="tab:orange", where="post", label="Set Point")
    ax[0].plot(t, pv, color="tab:blue", label="ADC GPIO33 (PV)")
    ax[0].set_ylabel("Tegangan (V)"); ax[0].grid(alpha=.3); ax[0].legend()
    ax[1].plot(t, e, color="tab:red", label="Error"); ax[1].plot(t, u, color="tab:green", label="Output DAC")
    ax[1].set_xlabel("Waktu (s)"); ax[1].set_ylabel("V"); ax[1].grid(alpha=.3); ax[1].legend()
    plt.tight_layout(); plt.savefig("respons.png", dpi=150)
    print("\nTersimpan: metrik.csv, respons.png")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    main(args[0] if args else "data_kontrol.csv", "--open" in sys.argv)
