import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

CANDIDATES = ["Correlator_res.csv", "LateModel43.Correlator_res.csv"]
OUT_PNG = "lap_summary.png"


def find_res_csv():
    found = [f for f in CANDIDATES if os.path.exists(f)]
    if not found:
        return None
    return max(found, key=os.path.getmtime)


def make_report():
    res_csv = find_res_csv()
    if not res_csv:
        print("ERROR: Could not find Correlator_res.csv or LateModel43.Correlator_res.csv.")
        return

    df = pd.read_csv(res_csv)
    t = df["time"]
    end = df.iloc[-1]

    # Power validation channels (supports both v1.2 and earlier .mo column names)
    if "P_net_theoretical" in df.columns:
        p_theo_kw = df["P_net_theoretical"] / 1000.0
    else:
        p_theo_kw = (df["F_net"] * df["v_meas"]) / 1000.0

    if "P_kin_actual" in df.columns:
        p_kin_kw = df["P_kin_actual"] / 1000.0
    else:
        p_kin_kw = df["der(W_kin_actual)"] / 1000.0

    rmse_kw = float(np.sqrt(np.mean((p_theo_kw - p_kin_kw) ** 2)))

    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))
    fig.suptitle(
        f"LateModel43 Telemetry & OpenModelica Report ({res_csv})  |  Lap Time: {t.iloc[-1]:.3f} s  |  Avg Wheel Power: {end['P_wheel_avg']/1000:.1f} kW",
        fontsize=12,
        fontweight="bold",
    )

    # Panel 1: Power Validation (Model vs Measured)
    ax = axes[0, 0]
    ax.plot(t, p_theo_kw, label="P_net_theoretical (Model)", linewidth=1.6)
    ax.plot(t, p_kin_kw, "--", label="P_kin_actual (Measured)", linewidth=1.4)
    ax.set_title(f"1. Longitudinal Power Correlation (RMSE = {rmse_kw:.1f} kW)")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Power (kW)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Panel 2: Engine & Wheel Power Trace
    ax = axes[0, 1]
    ax.plot(t, df["P_engine"] / 1000.0, label="P_engine", linewidth=1.5)
    ax.plot(t, df["P_wheel"] / 1000.0, label="P_wheel", linewidth=1.4)
    ax.plot(t, df["P_wheel_avg"] / 1000.0, label="P_wheel_avg", linewidth=1.8)
    ax.set_title("2. Power Delivery & Average Wheel Power")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Power (kW)")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    # Panel 3: Delta t = Work / Power Breakdown
    ax = axes[1, 0]
    labels = ["Aero Drag", "Braking", "Front Scrub", "Rolling", "Rear Slip"]
    vals = [
        float(end["dt_aero_total"]),
        float(end["dt_brake_total"]),
        float(end["dt_scrub_total"]),
        float(end["dt_roll_total"]),
        float(end["dt_slip_total"]),
    ]
    bars = ax.barh(labels, vals)
    ax.invert_yaxis()
    ax.set_xlim(0, max(max(vals) * 1.18, 1.0))
    ax.set_title("3. Δt = Work / Power Breakdown (Seconds Cost per Lap)")
    ax.set_xlabel("Time Cost (s)")
    ax.grid(True, axis="x", alpha=0.3)
    for b, v in zip(bars, vals):
        ax.text(v + 0.08, b.get_y() + b.get_height() / 2, f"{v:.2f} s", va="center", fontsize=9)

    # Panel 4: Setup Sensitivity Tornado
    ax = axes[1, 1]
    s_labels = ["+10% Power", "+10% Aero Cd", "+10 kg Mass"]
    s_vals = [
        float(end["dt_plus10pct_power"]),
        float(end["dt_plus10pct_Cd"]),
        float(end["dt_plus10kg_mass"]),
    ]
    colors = ["tab:green" if v < 0 else "tab:red" for v in s_vals]
    s_bars = ax.barh(s_labels, s_vals, color=colors)
    ax.axvline(0, color="black", linewidth=0.9)
    ax.invert_yaxis()
    ax.set_xlim(min(min(s_vals) * 1.32, -0.1), max(max(s_vals) * 1.32, 0.1))
    ax.set_title("4. Lap Time Sensitivity (Negative = Faster)")
    ax.set_xlabel("Δ Lap Time (s)")
    ax.grid(True, axis="x", alpha=0.3)
    for b, v in zip(s_bars, s_vals):
        offset = -0.02 if v < 0 else 0.02
        ha = "right" if v < 0 else "left"
        ax.text(v + offset, b.get_y() + b.get_height() / 2, f"{v:+.3f} s", va="center", ha=ha, fontsize=9)

    plt.tight_layout(rect=[0, 0.02, 1, 0.95])
    plt.savefig(OUT_PNG, dpi=160)
    print(f"\nUSED:  {res_csv}")
    print(f"SAVED: {OUT_PNG}")
    print("--- Summary for kerrracing.com ---")
    print(f"Lap Time:            {t.iloc[-1]:.3f} s")
    print(f"Avg Wheel Power:     {end['P_wheel_avg']/1000:.1f} kW ({end['P_wheel_avg']/745.7:.1f} whp)")
    print(f"Power Corr RMSE:     {rmse_kw:.1f} kW")
    print(f"W_wheel / W_loss:    {end['W_wheel']/1e6:.2f} MJ / {end['W_loss_total']/1e6:.2f} MJ")
    print(f"dt_aero_total:       {end['dt_aero_total']:.2f} s")
    print(f"dt_brake_total:      {end['dt_brake_total']:.2f} s")
    print(f"dt_scrub_total:      {end['dt_scrub_total']:.2f} s")
    print(f"dt_roll_total:       {end['dt_roll_total']:.2f} s")
    print(f"dt_slip_total:       {end['dt_slip_total']:.2f} s")
    print(f"+10% Power delta:    {end['dt_plus10pct_power']:+.3f} s")
    print(f"+10% Cd delta:       {end['dt_plus10pct_Cd']:+.3f} s")
    print(f"+10 kg Mass delta:   {end['dt_plus10kg_mass']:+.3f} s")


if __name__ == "__main__":
    make_report()