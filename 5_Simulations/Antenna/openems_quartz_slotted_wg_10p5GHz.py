# openems_quartz_slotted_wg_10p5GHz.py
# Slotted rectangular waveguide (quartz-filled, εr=3.8) tuned to 10.5 GHz.
# Builds geometry, meshes (no GetLine calls), sweeps S-params/impedance over 9.5–11.5 GHz,
# computes 3D far-field, and reports estimated max realized gain.

import os
import tempfile
import time

import matplotlib.pyplot as plt
import numpy as np

# --- openEMS / CSXCAD bindings ---
from openEMS import openEMS
from openEMS.physical_constants import C0

try:
    from CSXCAD import AppCSXCAD_BIN, ContinuousStructure

    HAVE_APP = True
except Exception:
    from CSXCAD import ContinuousStructure

    AppCSXCAD_BIN = None
    HAVE_APP = False

# Set PROFILE to "sanity" first; run and check [mesh] cells: stays reasonable.

# If it’s small, move to "balanced"; once happy, go "full".

# Toggle VIEW_GEOM=True if you want the 3D viewer (requires AppCSXCAD_BIN available).

# =========================
# USER SETTINGS / PROFILES
# =========================
PROFILE = "sanity"  # choose: "sanity" | "balanced" | "full"
VIEW_GEOM = False  # True => launch AppCSXCAD viewer (if available)
SIMULATE = True  # False => skip FDTD (for quick post-proc dev)
THREADS = 0  # 0 => all cores

# --- profiles tuned for i5-1135G7 + 16 GB ---
profiles = {
    "sanity": {
        "Nslots": 12,
        "mesh_res": 0.8,
        "air_x": 6.0,
        "air_y": 20.0,
        "air_z": 10.0,
        "n_theta": 61,
        "n_phi": 121,
        "freq_pts": 201,
        "pml": 6,
    },
    "balanced": {
        "Nslots": 24,
        "mesh_res": 0.6,
        "air_x": 8.0,
        "air_y": 30.0,
        "air_z": 12.0,
        "n_theta": 91,
        "n_phi": 181,
        "freq_pts": 301,
        "pml": 8,
    },
    "full": {
        "Nslots": 32,
        "mesh_res": 0.5,
        "air_x": 10.0,
        "air_y": 40.0,
        "air_z": 15.0,
        "n_theta": 91,
        "n_phi": 181,
        "freq_pts": 401,
        "pml": 8,
    },
}
cfg = profiles[PROFILE]

# =====================
# BAND & WAVEGUIDE SPEC
# =====================
f0 = 10.5e9
f_span = 2.0e9
f_start, f_stop = f0 - f_span / 2, f0 + f_span / 2
er_quartz = 3.8  # fused silica/quartz

# Array constraint (λ0/2 pitch, 1 mm septum) => internal a ~ 13.28 mm
lambda0_mm = (C0 / f0) * 1e3
a = 13.28  # mm (broad wall, internal)
b = 6.50  # mm (narrow wall, internal)  <-- comfortable machining

# Slot starters (tune later for taper)
slot_w = 0.60  # mm across x

# --- guide wavelength at 10.5 GHz (TE10) ---
fc10 = (C0 / (2.0 * np.sqrt(er_quartz))) * (1.0 / (a * 1e-3))  # Hz
lambda_d = (C0 / f0) / np.sqrt(er_quartz)  # m
lambda_g = lambda_d / np.sqrt(1.0 - (fc10 / f0) ** 2)  # m
lambda_g_mm = lambda_g * 1e3

# --- slots geometry (from λg) ---
slot_s = 0.5 * lambda_g_mm
slot_L = 0.47 * lambda_g_mm
margin = 0.25 * lambda_g_mm

# ===================================
# FDTD / CSX / MESH (explicit lines)
# ===================================
unit_mm = 1e-3
Sim_Path = os.path.join(tempfile.gettempdir(), f"openems_quartz_slotted_wg_{PROFILE}")

FDTD = openEMS(NrTS=int(6e5), EndCriteria=1e-5)
FDTD.SetGaussExcite(0.5 * (f_start + f_stop), 0.5 * (f_stop - f_start))
FDTD.SetBoundaryCond([f"PML_{cfg['pml']}"] * 6)
FDTD.SetOverSampling(4)
FDTD.SetTimeStepFactor(0.95)

CSX = ContinuousStructure()
FDTD.SetCSX(CSX)
mesh = CSX.GetGrid()
mesh.SetDeltaUnit(unit_mm)

# Pads & extent
t_metal = 0.8  # mm metal wall thickness
air_x = cfg["air_x"]
air_y = cfg["air_y"]
air_z = cfg["air_z"]
mesh_res = cfg["mesh_res"]

# Length from Nslots
Nslots = cfg["Nslots"]
guide_length_mm = margin + (Nslots - 1) * slot_s + margin

# Simulation extents (mm)
x_min, x_max = -air_x, a + air_x
y_min, y_max = -5.0, b + t_metal + air_y
z_min, z_max = -air_z, guide_length_mm + air_z

# Slot centers and edges (mm)
z_centers = margin + np.arange(Nslots) * slot_s
delta0 = 0.90  # mm offset from centerline (± alternated)
x_centers = (a / 2.0) + np.array([+delta0 if i % 2 == 0 else -delta0 for i in range(Nslots)])

x_edges = np.concatenate([x_centers - slot_w / 2.0, x_centers + slot_w / 2.0])
z_edges = np.concatenate([z_centers - slot_L / 2.0, z_centers + slot_L / 2.0])

# Mesh lines: explicit (NO GetLine calls)
x_lines = sorted(set([x_min, -t_metal, 0.0, a, a + t_metal, x_max] + list(x_edges)))
y_lines = [y_min, 0.0, b, b + t_metal, y_max]
z_lines = sorted(set([z_min, 0.0, guide_length_mm, z_max] + list(z_edges)))

mesh.AddLine("x", x_lines)
mesh.AddLine("y", y_lines)
mesh.AddLine("z", z_lines)

# Print complexity and rough memory (to help stay inside 16 GB)
Nx, Ny, Nz = len(x_lines) - 1, len(y_lines) - 1, len(z_lines) - 1
Ncells = Nx * Ny * Nz
print(f"[mesh] cells: {Nx} × {Ny} × {Nz} = {Ncells:,}")
mem_fields_bytes = Ncells * 6 * 8  # rough ~ (Ex,Ey,Ez,Hx,Hy,Hz) doubles
print(f"[mesh] rough field memory: ~{mem_fields_bytes / 1e9:.2f} GB (solver overhead extra)")
dx_min = min(np.diff(x_lines))
dy_min = min(np.diff(y_lines))
dz_min = min(np.diff(z_lines))
print(f"[mesh] min steps (mm): dx={dx_min:.3f}, dy={dy_min:.3f}, dz={dz_min:.3f}")

# Optional smoothing to limit max cell size
mesh.SmoothMeshLines("all", mesh_res, ratio=1.4)

# =================
# MATERIALS & SOLIDS
# =================
pec = CSX.AddMetal("PEC")
quartzM = CSX.AddMaterial("QUARTZ")
quartzM.SetMaterialProperty(epsilon=er_quartz)
airM = CSX.AddMaterial("AIR")

# Quartz full block
quartzM.AddBox([0, 0, 0], [a, b, guide_length_mm])

# PEC tube walls
pec.AddBox([-t_metal, 0, 0], [0, b, guide_length_mm])  # left
pec.AddBox([a, 0, 0], [a + t_metal, b, guide_length_mm])  # right
pec.AddBox([-t_metal, -t_metal, 0], [a + t_metal, 0, guide_length_mm])  # bottom
pec.AddBox([-t_metal, b, 0], [a + t_metal, b + t_metal, guide_length_mm])  # top (slots will pierce)

# Slots (AIR) overriding top metal
for zc, xc in zip(z_centers, x_centers):
    x1, x2 = xc - slot_w / 2.0, xc + slot_w / 2.0
    z1, z2 = zc - slot_L / 2.0, zc + slot_L / 2.0
    prim = airM.AddBox([x1, b, z1], [x2, b + t_metal, z2])
    prim.SetPriority(10)  # ensure cut

# =========
# WG PORTS
# =========
port_thick = max(4 * mesh_res, 2.0)  # mm
p1_start = [0, 0, max(0.5, 10 * mesh_res)]
p1_stop = [a, b, p1_start[2] + port_thick]
FDTD.AddRectWaveGuidePort(
    port_nr=1,
    start=p1_start,
    stop=p1_stop,
    p_dir="z",
    a=a * unit_mm,
    b=b * unit_mm,
    mode_name="TE10",
    excite=1,
)

p2_stop = [a, b, guide_length_mm - max(0.5, 10 * mesh_res)]
p2_start = [0, 0, p2_stop[2] - port_thick]
FDTD.AddRectWaveGuidePort(
    port_nr=2,
    start=p2_start,
    stop=p2_stop,
    p_dir="z",
    a=a * unit_mm,
    b=b * unit_mm,
    mode_name="TE10",
    excite=0,
)


# =========
# NF2FF BOX
# =========
def create_nf2ff(FDTD_obj, name, start, stop, frequency):
    try:
        return FDTD_obj.CreateNF2FFBox(name=name, start=start, stop=stop, frequency=frequency)
    except AttributeError:
        return FDTD_obj.AddNF2FFBox(name=name, start=start, stop=stop, frequency=frequency)


nf2ff = create_nf2ff(
    FDTD,
    name="nf2ff",
    start=[x_min + 1.0, y_min + 1.0, z_min + 1.0],
    stop=[x_max - 1.0, y_max - 1.0, z_max - 1.0],
    frequency=[f0],
)

# ==========
# VIEW GEOM
# ==========
if VIEW_GEOM and HAVE_APP and AppCSXCAD_BIN:
    os.makedirs(Sim_Path, exist_ok=True)
    csx_xml = os.path.join(Sim_Path, f"quartz_slotted_wg_{PROFILE}.xml")
    CSX.Write2XML(csx_xml)
    os.system(f'"{AppCSXCAD_BIN}" "{csx_xml}"')

# ... right before the FDTD run:
t0 = time.time()
FDTD.Run(Sim_Path, cleanup=True, verbose=2, numThreads=THREADS)
t1 = time.time()
print(f"[timing] FDTD solve elapsed: {t1 - t0:.2f} s")

# =======
# RUN FDTD
# =======
if SIMULATE:
    FDTD.Run(Sim_Path, cleanup=True, verbose=2, numThreads=THREADS)

# ==========================
# POST: S-PARAMS / IMPEDANCE
# ==========================
freq = np.linspace(f_start, f_stop, profiles[PROFILE]["freq_pts"])
ports = [p for p in FDTD.ports]  # Port 1 & 2 in creation order
for p in ports:
    p.CalcPort(Sim_Path, freq)

S11 = ports[0].uf_ref / ports[0].uf_inc
S21 = ports[1].uf_ref / ports[0].uf_inc
Zin = ports[0].uf_tot / ports[0].if_tot

plt.figure(figsize=(7.6, 4.6))
plt.plot(freq * 1e-9, 20 * np.log10(np.abs(S11)), lw=2, label="|S11|")
plt.plot(freq * 1e-9, 20 * np.log10(np.abs(S21)), lw=2, ls="--", label="|S21|")
plt.grid(True)
plt.legend()
plt.xlabel("Frequency (GHz)")
plt.ylabel("Magnitude (dB)")
plt.title(f"S-Parameters (profile: {PROFILE})")

plt.figure(figsize=(7.6, 4.6))
plt.plot(freq * 1e-9, np.real(Zin), lw=2, label="Re{Zin}")
plt.plot(freq * 1e-9, np.imag(Zin), lw=2, ls="--", label="Im{Zin}")
plt.grid(True)
plt.legend()
plt.xlabel("Frequency (GHz)")
plt.ylabel("Ohms")
plt.title("Input Impedance (Port 1)")

# ==========================
# POST: 3D FAR-FIELD / GAIN
# ==========================
n_theta, n_phi = cfg["n_theta"], cfg["n_phi"]
theta = np.linspace(0, np.pi, n_theta)
phi = np.linspace(0, 2 * np.pi, n_phi)

try:
    res = nf2ff.CalcNF2FF(Sim_Path, [f0], theta, phi)
except AttributeError:
    res = FDTD.CalcNF2FF(nf2ff, Sim_Path, [f0], theta, phi)

idx_f0 = np.argmin(np.abs(freq - f0))
Dmax_lin = float(res.Dmax[0])
mismatch = 1.0 - np.abs(S11[idx_f0]) ** 2
Gmax_lin = Dmax_lin * float(mismatch)
Gmax_dBi = 10 * np.log10(Gmax_lin)

print(f"[far-field] Dmax @ {f0 / 1e9:.3f} GHz: {10 * np.log10(Dmax_lin):.2f} dBi")
print(f"[far-field] mismatch (1-|S11|^2): {float(mismatch):.3f}")
print(f"[far-field] est. max realized gain: {Gmax_dBi:.2f} dBi")

# Normalized 3D pattern
E = np.squeeze(res.E_norm)  # [th, ph]
E = E / np.max(E)
TH, PH = np.meshgrid(theta, phi, indexing="ij")
R = E
X = R * np.sin(TH) * np.cos(PH)
Y = R * np.sin(TH) * np.sin(PH)
Z = R * np.cos(TH)

fig = plt.figure(figsize=(7.2, 6.2))
ax = fig.add_subplot(111, projection="3d")
ax.plot_surface(X, Y, Z, rstride=2, cstride=2, linewidth=0, antialiased=True, alpha=0.92)
ax.set_title(f"Normalized 3D Pattern @ {f0 / 1e9:.2f} GHz\n(peak ≈ {Gmax_dBi:.1f} dBi)")
ax.set_box_aspect((1, 1, 1))
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
plt.tight_layout()

# ==========================
# QUICK 2D GEOMETRY PREVIEW
# ==========================
plt.figure(figsize=(8.4, 2.8))
plt.fill_between(
    [0, a],
    [0, 0],
    [guide_length_mm, guide_length_mm],
    color="#dddddd",
    alpha=0.5,
    step="pre",
    label="WG top aperture",
)
for zc, xc in zip(z_centers, x_centers):
    plt.gca().add_patch(
        plt.Rectangle((xc - slot_w / 2.0, zc - slot_L / 2.0), slot_w, slot_L, fc="#3355ff", ec="k")
    )
plt.xlim(-2, a + 2)
plt.ylim(-5, guide_length_mm + 5)
plt.gca().invert_yaxis()
plt.xlabel("x (mm)")
plt.ylabel("z (mm)")
plt.title(f"Top-view slot layout (N={Nslots}, profile={PROFILE})")
plt.grid(True)
plt.legend()


plt.show()
