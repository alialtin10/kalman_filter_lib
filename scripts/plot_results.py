#!/usr/bin/env python3
"""Örneklerin ürettiği CSV dosyalarından README grafiklerini çizer.

Kullanım:
    python3 scripts/plot_results.py <csv_klasörü> <çıktı_klasörü>
    python3 scripts/plot_results.py build/results docs/images
"""
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

COLORS = {
    "truth": "#222222",
    "meas": "#9aa5b1",
    "kf": "#1f77b4",
    "ukf": "#d62728",
    "dr": "#ff7f0e",
    "naive": "#8c564b",
    "reject": "#e377c2",
}

plt.rcParams.update({
    "figure.dpi": 110,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "font.size": 10,
})


def load(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def rmse(a, b):
    return float(np.sqrt(np.nanmean((a - b) ** 2)))


def plot_1d(d, out):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    t = d["t"]
    ax1.plot(t, d["meas_pos"], ".", ms=3, color=COLORS["meas"],
             label=f"Sensör (RMSE {rmse(d['meas_pos'], d['true_pos']):.2f} m)")
    ax1.plot(t, d["true_pos"], color=COLORS["truth"], lw=1.5, label="Gerçek")
    ax1.plot(t, d["est_pos"], color=COLORS["kf"], lw=1.5,
             label=f"KF (RMSE {rmse(d['est_pos'], d['true_pos']):.2f} m)")
    ax1.fill_between(t, d["est_pos"] - 2 * d["std_pos"], d["est_pos"] + 2 * d["std_pos"],
                     color=COLORS["kf"], alpha=0.15, label="KF ±2σ")
    ax1.set_ylabel("Konum [m]")
    ax1.legend(loc="upper left")
    ax1.set_title("1B robot takibi — Kalman Filtresi (sabit hız modeli)")

    ax2.plot(t, d["true_vel"], color=COLORS["truth"], lw=1.5, label="Gerçek")
    ax2.plot(t, d["est_vel"], color=COLORS["kf"], lw=1.5, label="KF (doğrudan ölçülmüyor)")
    ax2.set_ylabel("Hız [m/s]")
    ax2.set_xlabel("Zaman [s]")
    ax2.legend(loc="upper left")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def plot_altitude(d, out):
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    t = d["t"]
    baro = ~np.isnan(d["baro"])

    ax = axes[0]
    ax.plot(t[baro], d["baro"][baro], ".", ms=2, color=COLORS["meas"],
            label=f"Barometre (RMSE {rmse(d['baro'], d['true_h']):.2f} m)")
    ax.plot(t, d["true_h"], color=COLORS["truth"], lw=1.5, label="Gerçek")
    ax.plot(t, d["est_h"], color=COLORS["kf"], lw=1.2,
            label=f"KF (RMSE {rmse(d['est_h'], d['true_h']):.2f} m)")
    imu_visible = np.abs(d["imu_only_h"] - d["true_h"]) < 8
    ax.plot(t[imu_visible], d["imu_only_h"][imu_visible], "--", color=COLORS["dr"], lw=1.2,
            label="Sadece IMU (ıraksıyor)")
    ax.set_ylabel("İrtifa [m]")
    ax.set_ylim(-3, 24)
    ax.legend(loc="upper right", ncol=2)
    ax.set_title("İHA irtifa kestirimi — IMU + barometre füzyonu (KF)")

    ax = axes[1]
    ax.plot(t, d["est_h"] - d["true_h"], color=COLORS["kf"], lw=1, label="KF irtifa hatası")
    ax.fill_between(t, -2 * d["std_h"], 2 * d["std_h"], color=COLORS["kf"], alpha=0.15, label="±2σ")
    ax.set_ylabel("Hata [m]")
    ax.legend(loc="upper right")

    ax = axes[2]
    ax.plot(t, d["true_bias"], color=COLORS["truth"], lw=1.5, label="Gerçek sapma")
    ax.plot(t, d["est_bias"], color=COLORS["kf"], lw=1.5, label="KF kestirimi")
    ax.fill_between(t, d["est_bias"] - 2 * d["std_bias"], d["est_bias"] + 2 * d["std_bias"],
                    color=COLORS["kf"], alpha=0.15, label="±2σ")
    ax.set_ylabel("İvmeölçer sapması [m/s²]")
    ax.set_xlabel("Zaman [s]")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out)
    plt.close(fig)


def plot_diff_drive(d, out):
    fig = plt.figure(figsize=(11, 8))
    gs = fig.add_gridspec(2, 2, height_ratios=[2.2, 1])
    ax = fig.add_subplot(gs[0, :])
    gps = ~np.isnan(d["gps_x"])
    rej = gps & (d["gps_rejected"] > 0)
    acc = gps & ~rej

    ax.plot(d["true_x"], d["true_y"], color=COLORS["truth"], lw=2, label="Gerçek yörünge")
    ax.plot(d["dr_x"], d["dr_y"], "--", color=COLORS["dr"], lw=1.2,
            label=f"Dead reckoning (RMSE {rmse_xy(d, 'dr'):.2f} m)")
    ax.plot(d["naive_x"], d["naive_y"], ":", color=COLORS["naive"], lw=1.2,
            label=f"EKF, ayarsız Q (RMSE {rmse_xy(d, 'naive'):.2f} m)")
    ax.plot(d["ekf_x"], d["ekf_y"], color=COLORS["kf"], lw=1.5,
            label=f"EKF (RMSE {rmse_xy(d, 'ekf'):.2f} m)")
    ax.plot(d["ukf_x"], d["ukf_y"], "--", color=COLORS["ukf"], lw=1.2,
            label=f"UKF (RMSE {rmse_xy(d, 'ukf'):.2f} m)")
    ax.plot(d["gps_x"][acc], d["gps_y"][acc], "o", ms=3.5, color=COLORS["meas"], label="GPS (kabul)")
    ax.plot(d["gps_x"][rej], d["gps_y"][rej], "x", ms=7, mew=2, color=COLORS["reject"],
            label="GPS (NIS testi ile reddedildi)")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Diferansiyel sürüşlü robot — odometri + GPS füzyonu (EKF / UKF)")
    ax.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0), fontsize=9)

    t = d["t"]
    ax = fig.add_subplot(gs[1, 0])
    for key, color, label in (("dr", COLORS["dr"], "Dead reckoning"), ("ekf", COLORS["kf"], "EKF"),
                              ("ukf", COLORS["ukf"], "UKF")):
        err = np.hypot(d[f"{key}_x"] - d["true_x"], d[f"{key}_y"] - d["true_y"])
        ax.plot(t, err, color=color, lw=1, label=label)
    ax.set_yscale("log")
    ax.set_xlabel("Zaman [s]")
    ax.set_ylabel("Konum hatası [m]")
    ax.legend(fontsize=8)

    ax = fig.add_subplot(gs[1, 1])
    ex = d["ekf_x"] - d["true_x"]
    ax.plot(t, ex, color=COLORS["kf"], lw=1, label="EKF x hatası")
    ax.fill_between(t, -2 * d["ekf_sx"], 2 * d["ekf_sx"], color=COLORS["kf"], alpha=0.15, label="±2σ")
    ax.set_xlabel("Zaman [s]")
    ax.set_ylabel("x hatası [m]")
    ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)


def rmse_xy(d, key):
    return float(np.sqrt(np.mean((d[f"{key}_x"] - d["true_x"]) ** 2 + (d[f"{key}_y"] - d["true_y"]) ** 2)))


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(1)
    src, dst = Path(sys.argv[1]), Path(sys.argv[2])
    dst.mkdir(parents=True, exist_ok=True)
    plot_1d(load(src / "1d_robot_tracking.csv"), dst / "1d_robot_tracking.png")
    plot_altitude(load(src / "altitude_estimation.csv"), dst / "altitude_estimation.png")
    plot_diff_drive(load(src / "2d_differential_kinematics.csv"), dst / "2d_differential_kinematics.png")
    print(f"Grafikler kaydedildi: {dst}")


if __name__ == "__main__":
    main()
