#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np

c0 = 299_792_458.0
f0 = 10500000000.0
lam0 = c0 / f0
k0 = 2 * np.pi / lam0

M = 16
N = 32
dy = 14.275831333333334 / 1e3
dz = 16.915 / 1e3

theta0_deg = 0.0
phi0_deg = 0.0
theta0 = np.deg2rad(theta0_deg)
phi0 = np.deg2rad(phi0_deg)

beta = 1.65
wy = np.ones(16, float)
wz = np.kaiser(32, beta)
wz /= wz.max()

m_idx = np.arange(M) - (M - 1) / 2
n_idx = np.arange(N) - (N - 1) / 2
y_positions = m_idx * dy
z_positions = n_idx * dz


def element_factor(theta_rad, phi_rad):
    return np.abs(np.cos(theta_rad))


def array_factor(theta_rad, phi_rad, y_positions, z_positions, wy, wz, theta0_rad, phi0_rad):
    k0 = 2 * np.pi / (299_792_458.0 / 10500000000.0)
    ky = k0 * np.sin(theta_rad) * np.sin(phi_rad)
    kz = k0 * np.sin(theta_rad) * np.cos(phi_rad)
    ky0 = k0 * np.sin(theta0_rad) * np.sin(phi0_rad)
    kz0 = k0 * np.sin(theta0_rad) * np.cos(phi0_rad)
    Ay = np.sum(wy[:, None] * np.exp(1j * y_positions[:, None] * (ky[None, :] - ky0)), axis=0)
    Az = np.sum(wz[:, None] * np.exp(1j * z_positions[:, None] * (kz[None, :] - kz0)), axis=0)
    return Ay * Az


def cut_curve(phi_deg, num_pts=721):
    th_deg = np.linspace(0, 90, num_pts)
    th = np.deg2rad(th_deg)
    ph = np.deg2rad(phi_deg) * np.ones_like(th)
    AF = array_factor(th, ph, y_positions, z_positions, wy, wz, theta0, phi0)
    PAT = np.abs(AF) * element_factor(th, ph)
    PAT /= PAT.max()
    return th_deg, 20 * np.log10(PAT + 1e-15)


def hpbw_deg(theta_deg, pat_db):
    p = np.argmax(pat_db)
    peak = pat_db[p]
    mask = pat_db >= (peak - 3.0)
    idx = np.where(mask)[0]
    if len(idx) < 2:
        return np.nan
    return float(theta_deg[idx[-1]] - theta_deg[idx[0]])


thE_deg, patE_db = cut_curve(0.0)
bwE = hpbw_deg(thE_deg, patE_db)
plt.figure(figsize=(7, 5), dpi=130)
plt.plot(thE_deg, patE_db, linewidth=1.5)
plt.grid(True, linestyle="--", alpha=0.5)
plt.xlabel("Theta (deg)")
plt.ylabel("Normalized Gain (dB)")
plt.title(f"E-plane (phi=0°)  |  -3 dB BW ≈ {bwE:.2f}°")
plt.tight_layout()
plt.savefig("E_plane_Kaiser25dB_like.png", bbox_inches="tight")
plt.show()

thH_deg, patH_db = cut_curve(90.0)
bwH = hpbw_deg(thH_deg, patH_db)
plt.figure(figsize=(7, 5), dpi=130)
plt.plot(thH_deg, patH_db, linewidth=1.5)
plt.grid(True, linestyle="--", alpha=0.5)
plt.xlabel("Theta (deg)")
plt.ylabel("Normalized Gain (dB)")
plt.title(f"H-plane (phi=90°) |  -3 dB BW ≈ {bwH:.2f}°")
plt.tight_layout()
plt.savefig("H_plane_Kaiser25dB_like.png", bbox_inches="tight")
plt.show()

theta_deg = np.linspace(0.0, 90.0, 121)
phi_deg = np.linspace(-90.0, 90.0, 121)
TH, PH = np.meshgrid(theta_deg, phi_deg, indexing="xy")
PAT_db = np.empty_like(TH, dtype=float)
for i in range(TH.shape[0]):
    th = np.deg2rad(TH[i, :])
    ph = np.deg2rad(PH[i, :])
    AF = array_factor(th, ph, y_positions, z_positions, wy, wz, theta0, phi0)
    EF = element_factor(th, ph)
    pat = np.abs(AF) * EF
    pat /= pat.max()
    PAT_db[i, :] = 20 * np.log10(pat + 1e-15)

plt.figure(figsize=(7, 5), dpi=130)
extent = [theta_deg[0], theta_deg[-1], phi_deg[0], phi_deg[-1]]
plt.imshow(PAT_db, origin="lower", extent=extent, aspect="auto")
plt.colorbar(label="Normalized Gain (dB)")
plt.xlabel("Theta (deg)")
plt.ylabel("Phi (deg)")
plt.title("Array Pattern Heatmap (|AF·EF|, dB) — Kaiser ~-25 dB")
plt.tight_layout()
plt.savefig("Heatmap_Kaiser25dB_like.png", bbox_inches="tight")
plt.show()

print(
    "Saved: E_plane_Kaiser25dB_like.png, H_plane_Kaiser25dB_like.png, Heatmap_Kaiser25dB_like.png"
)
