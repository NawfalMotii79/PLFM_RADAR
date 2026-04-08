# openems_quartz_slotted_wg_10p5GHz.py
# Full script: geometry, mesh (no GetLine calls), S-params/impedance sweep, 3D pattern & gain.
# Requires: openEMS (Python bindings), CSXCAD (Python), matplotlib, numpy.

import os
import tempfile

import matplotlib.pyplot as plt
import numpy as np
from CSXCAD import AppCSXCAD_BIN, ContinuousStructure
from openEMS import openEMS
from openEMS.physical_constants import C0

# -------------------------
# User controls
# -------------------------
view_geom_in_AppCSXCAD = True  # True => launch AppCSXCAD to view 3D geometry
simulate = True  # False => skip FDTD run
threads = 0  # 0 => auto/max

# Far-field angular sampling
n_theta, n_phi = 91, 181  # theta 0..180, phi 0..360

# -------------------------
# Band & element sizing
# -------------------------
f0 = 10.5e9
f_span = 2.0e9
f_start, f_stop = f0 - f_span / 2, f0 + f_span / 2

# Quartz-filled rectangular waveguide (full dielectric block)
er_quartz = 3.8
# Array-driven constraints (λ0/2 column pitch, 1 mm septum) ⇒ a ~ 13.28 mm
a = 13.28  # mm (broad wall, internal)
b = 6.50  # mm (narrow wall, internal; comfortable to machine)
L = 281.0  # mm (≈32-slot column incl. λg/4 margins at 10.5 GHz)

# Slot starters (tune in EM for taper)
slot_w = 0.60  # mm (across x)
lambda0_mm = (C0 / f0) * 1e3
fc10 = (C0 / (2.0 * np.sqrt(er_quartz))) * (1.0 / (a * 1e-3))  # Hz
lambda_d = (C0 / f0) / np.sqrt(er_quartz)  # m
lambda_g = lambda_d / np.sqrt(1.0 - (fc10 / f0) ** 2)  # m
lambda_g_mm = lambda_g * 1e3
slot_s = 0.5 * lambda_g_mm
slot_L = 0.47 * lambda_g_mm
margin = 0.25 * lambda_g_mm
Nslots = 32
delta0 = 0.90  # mm offset from centerline (± alternated)

# Metal & air padding for the radiation domain
t_metal = 0.8  # mm wall thickness
air_x = 10.0  # mm on each side
air_y = 40.0  # mm above slots
air_z = 15.0  # mm front/back

# Mesh resolution target (mm)
mesh_res = min(0.5, lambda0_mm / 30.0)

# -------------------------
# Build FDTD & CSX
# -------------------------
unit = 1e-3  # mm
Sim_Path = os.path.join(tempfile.gettempdir(), "openems_quartz_slotted_wg")

FDTD = openEMS(NrTS=int(6e5), EndCriteria=1e-5)
FDTD.SetGaussExcite(0.5 * (f_start + f_stop), 0.5 * (f_stop - f_start))
FDTD.SetBoundaryCond(["PML_8"] * 6)
FDTD.SetOverSampling(4)
FDTD.SetTimeStepFactor(0.95)

CSX = ContinuousStructure()
FDTD.SetCSX(CSX)

mesh = CSX.GetGrid()
mesh.SetDeltaUnit(unit)

# -------------------------
# Geometry extents (mm)
# -------------------------
x_min, x_max = -air_x, a + air_x
y_min, y_max = -5.0, b + t_metal + air_y
z_min, z_max = -air_z, L + air_z

# Slot centers and edges (mm)
z_centers = margin + np.arange(Nslots) * slot_s
x_centers = (a / 2.0) + np.array([+delta0 if i % 2 == 0 else -delta0 for i in range(Nslots)])
x_edges = np.concatenate([x_centers - slot_w / 2.0, x_centers + slot_w / 2.0])
z_edges = np.concatenate([z_centers - slot_L / 2.0, z_centers + slot_L / 2.0])

# -------------------------
# Mesh lines — EXPLICIT (no GetLine calls)
# -------------------------
x_lines = sorted(set([x_min, -t_metal, 0.0, a, a + t_metal, x_max] + list(x_edges)))
y_lines = [y_min, 0.0, b, b + t_metal, y_max]
z_lines = sorted(set([z_min, 0.0, L, z_max] + list(z_edges)))

mesh.AddLine("x", x_lines)
mesh.AddLine("y", y_lines)
mesh.AddLine("z", z_lines)

# Optional smoothing to max cell size around ~mesh_res
mesh.SmoothMeshLines("all", mesh_res, ratio=1.4)

# -------------------------
# Materials
# -------------------------
pec = CSX.AddMetal("PEC")
quartz = CSX.AddMaterial("QUARTZ")
quartz.SetMaterialProperty(epsilon=er_quartz)
air = CSX.AddMaterial("AIR")  # explicit for slot holes

# -------------------------
# Solids: quartz block + metal walls + slots
# -------------------------
# Quartz full block (inside tube)
quartz.AddBox([0, 0, 0], [a, b, L])

# PEC tube: left/right/bottom/top (top will be perforated by slots)
pec.AddBox([-t_metal, 0, 0], [0, b, L])  # left
pec.AddBox([a, 0, 0], [a + t_metal, b, L])  # right
pec.AddBox([-t_metal, -t_metal, 0], [a + t_metal, 0, L])  # bottom
pec.AddBox([-t_metal, b, 0], [a + t_metal, b + t_metal, L])  # top

# Slots = AIR boxes overriding the top metal
for zc, xc in zip(z_centers, x_centers):
    x1, x2 = xc - slot_w / 2.0, xc + slot_w / 2.0
    z1, z2 = zc - slot_L / 2.0, zc + slot_L / 2.0
    prim = air.AddBox([x1, b, z1], [x2, b + t_metal, z2])
    prim.SetPriority(10)  # ensure it cuts the metal

# -------------------------
# Ports: Rectangular WG TE10, z-directed
# -------------------------
port_thick = max(4 * mesh_res, 2.0)  # mm
p1_start = [0, 0, max(0.5, 10 * mesh_res)]
p1_stop = [a, b, p1_start[2] + port_thick]
FDTD.AddRectWaveGuidePort(
    port_nr=1,
    start=p1_start,
    stop=p1_stop,
    p_dir="z",
    a=a * unit,
    b=b * unit,
    mode_name="TE10",
    excite=1,
)

p2_stop = [a, b, L - max(0.5, 10 * mesh_res)]
p2_start = [0, 0, p2_stop[2] - port_thick]
FDTD.AddRectWaveGuidePort(
    port_nr=2,
    start=p2_start,
    stop=p2_stop,
    p_dir="z",
    a=a * unit,
    b=b * unit,
    mode_name="TE10",
    excite=0,
)


# -------------------------
# NF2FF setup (surround the radiator region)
# -------------------------
def create_nf2ff(FDTD_obj, name, start, stop, frequency):
    """Compat wrapper: older/newer openEMS builds may expose NF2FF creation differently."""
    try:
        return FDTD_obj.CreateNF2FFBox(name=name, start=start, stop=stop, frequency=frequency)
    except AttributeError:
        # Fallback: try AddNF2FFBox returning a handle-like object
        return FDTD_obj.AddNF2FFBox(name=name, start=start, stop=stop, frequency=frequency)


nf2ff = create_nf2ff(
    FDTD,
    name="nf2ff",
    start=[x_min + 1.0, y_min + 1.0, z_min + 1.0],
    stop=[x_max - 1.0, y_max - 1.0, z_max - 1.0],
    frequency=[f0],
)

# -------------------------
# (Optional) view geometry
# -------------------------
if view_geom_in_AppCSXCAD:
    os.makedirs(Sim_Path, exist_ok=True)
    csx_xml = os.path.join(Sim_Path, "quartz_slotted_wg.xml")
    CSX.Write2XML(csx_xml)
    os.system(f'"{AppCSXCAD_BIN}" "{csx_xml}"')

# -------------------------
# Run FDTD
# -------------------------
if simulate:
    FDTD.Run(Sim_Path, cleanup=True, verbose=2, numThreads=threads)

# -------------------------
# Post-processing: S-params & impedance
# -------------------------
freq = np.linspace(f_start, f_stop, 401)
ports = [p for p in FDTD.ports]  # Port 1 & Port 2 in creation order
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
plt.title("S-Parameters: Slotted Quartz-Filled WG")

plt.figure(figsize=(7.6, 4.6))
plt.plot(freq * 1e-9, np.real(Zin), lw=2, label="Re{Zin}")
plt.plot(freq * 1e-9, np.imag(Zin), lw=2, ls="--", label="Im{Zin}")
plt.grid(True)
plt.legend()
plt.xlabel("Frequency (GHz)")
plt.ylabel("Ohms")
plt.title("Input Impedance (Port 1)")

# -------------------------
# Far-field @ f0 and 3D pattern
# -------------------------
theta = np.linspace(0, np.pi, n_theta)
phi = np.linspace(0, 2 * np.pi, n_phi)

# Compatibility: some builds expect nf2ff.CalcNF2FF(...), others FDTD.CalcNF2FF(nf2ff,...)
try:
    res = nf2ff.CalcNF2FF(Sim_Path, [f0], theta, phi)
except AttributeError:
    res = FDTD.CalcNF2FF(nf2ff, Sim_Path, [f0], theta, phi)

# Max directivity (linear) & realized gain estimate
idx_f0 = np.argmin(np.abs(freq - f0))
Dmax_lin = float(res.Dmax[0])  # at f0
mismatch = 1.0 - np.abs(S11[idx_f0]) ** 2  # (1 - |S11|^2)
Gmax_lin = Dmax_lin * float(mismatch)
Gmax_dBi = 10 * np.log10(Gmax_lin)

print(f"Max directivity @ {f0 / 1e9:.3f} GHz: {10 * np.log10(Dmax_lin):.2f} dBi")
print(f"Mismatch term  (1-|S11|^2)        : {float(mismatch):.3f}")
print(f"Estimated max realized gain       : {Gmax_dBi:.2f} dBi")

# 3D normalized pattern
E = np.squeeze(res.E_norm)  # shape [f, th, ph] -> [th, ph]
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

# Quick 2D geometry preview (top view at y=b)
plt.figure(figsize=(8.4, 2.8))
plt.fill_between(
    [0, a], [0, 0], [L, L], color="#dddddd", alpha=0.5, step="pre", label="WG aperture (top)"
)
for zc, xc in zip(z_centers, x_centers):
    plt.gca().add_patch(
        plt.Rectangle((xc - slot_w / 2.0, zc - slot_L / 2.0), slot_w, slot_L, fc="#3355ff", ec="k")
    )
plt.xlim(-2, a + 2)
plt.ylim(-5, L + 5)
plt.gca().invert_yaxis()
plt.xlabel("x (mm)")
plt.ylabel("z (mm)")
plt.title("Top-view slot layout (y=b plane)")
plt.grid(True)
plt.legend()

plt.show()
