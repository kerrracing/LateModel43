# LateModel43 — 2026 Snowball Derby (Five Flags) Physics & Telemetry Package (v1.2)

An open-source Assetto Corsa Super Late Model mod, asymmetric oval baseline setup, MoTeC ACTI sample telemetry, and OpenModelica $\Delta t = \text{Work} / \text{Power}$ physics correlator built for **Kerr Racing** (`https://www.kerrracing.com/`).

## What’s Included
* **AC Car Mod (`v1.2` Release ZIP):** CSP `extended-2` physics calibrated to 2026 Snowball Derby Super Late Model targets (`2,800 lb / 1,270 kg`, locked spool, `~662 hp` @ `7,600 rpm` / `7,800` limiter, `4.625` rear gear, `61%` front brake bias, calibrated short-track tire grip & aero sideforce).
* **Asymmetric Oval Baseline (`FiveFlags_Base.ini`):** Left/right split wheel rates (`86 / 116` front, `92 / 104` rear), `+2.5° LF / -3.5° RF` camber, ride-height stagger, and `19 / 24 psi` cold pressures.
* **OpenModelica Correlator (`LateModel43.mo`):** Decomposes real ACTI/MoTeC telemetry into individual energy sinks ($W_{\text{aero}}, W_{\text{roll}}, W_{\text{scrub}}, W_{\text{brake}}, W_{\text{slip}}$) and lap-time contributions ($\Delta t = W / P$).
* **One-Click Pipeline (`clean_lap.py`, `run_lap.mos`, `make_report.py`):** Converts MoTeC CSV exports into an OpenModelica table, runs the solver, and generates a 4-panel presentation summary (`lap_summary.png`).

## $\Delta t = \text{Work} / \text{Power}$ Breakdown (Five Flags ~16.3 s Target)
| Factor | What It Represents | Est. Lap Spread | Sim + Modelica Error Bar |
| :--- | :--- | :--- | :--- |
| **Engine & Gearing** | Average power delivered to the pavement ($\bar{P}_{\text{wheel}}$) | `~0.25 – 0.45 s` | `±0.05 – 0.10 s` |
| **Aero (Drag, Downforce, Yaw)** | Straightaway drag vs. cornering aero & body sideforce | `~0.15 – 0.35 s` | `±0.08 – 0.15 s` |
| **Chassis & Tires** | Mechanical grip, asymmetric wedge, and tire wear/heat falloff | `~0.40 – 0.90 s` | `±0.15 – 0.30 s` |
| **Driver Execution** | Throttle pickup %, braking modulation, and steering smoothness | `~0.30 – 0.80 s` | `±0.10 – 0.25 s` |

## Quick Start
1. **Assetto Corsa:** Download `LateModel43_v1.2_FiveFlags.zip` from **Releases**, extract to `assettocorsa\content\cars\`, and load `FiveFlags_Base` in the pit setup menu. *(Requires Custom Shaders Patch for `extended-2` physics.)*
2. **Telemetry & OpenModelica:**
   ```bash
   python clean_lap.py
   omc run_lap.mos
   python make_report.py