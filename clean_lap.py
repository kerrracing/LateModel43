import glob
import os
import pandas as pd

INPUT_FILE = None  # Leave None to auto-pick newest CSV in folder
TARGET_LAP = 1
OUTPUT_FILE = "telemetry_table.txt"
MOS_FILE = "run_lap.mos"
TABLE_NAME = "telemetry"


def find_input_csv():
    if INPUT_FILE and os.path.exists(INPUT_FILE):
        return INPUT_FILE
    csvs = sorted(glob.glob("*.csv"), key=os.path.getmtime, reverse=True)
    # Ignore OpenModelica output CSVs if present
    csvs = [c for c in csvs if not c.endswith("_res.csv")]
    return csvs[0] if csvs else None


def clean_to_modelica_txt():
    input_file = find_input_csv()
    if not input_file:
        print("ERROR: No MoTeC .csv file found in the current directory.")
        return

    skip = 0
    with open(input_file, "r", encoding="utf-8", errors="ignore") as f:
        for i, line in enumerate(f):
            if line.startswith('"Time"'):
                skip = i
                break

    df = pd.read_csv(input_file, skiprows=skip, low_memory=False)
    df = df.drop(0).reset_index(drop=True)

    cols_to_keep = [
        "Time",
        "Brake Pos",
        "CG Accel Lateral",
        "CG Accel Longitudinal",
        "Ground Speed",
        "Gear",
        "Engine RPM",
        "Steering Angle",
        "Throttle Pos",
        "Wheel Angular Speed FL",
        "Wheel Angular Speed FR",
        "Wheel Angular Speed RL",
        "Wheel Angular Speed RR",
    ]

    for col in cols_to_keep + ["Session Lap Count", "Lap Time"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    if "Session Lap Count" in df.columns:
        lap_df = df[df["Session Lap Count"] == TARGET_LAP].copy()
        if not lap_df.empty:
            df = lap_df

    if "Lap Time" in df.columns:
        df = df.drop_duplicates(subset=["Lap Time"], keep="first").copy()

    clean_df = df[cols_to_keep].apply(pd.to_numeric, errors="coerce").fillna(0.0)
    clean_df["Time"] = clean_df["Time"] - clean_df["Time"].iloc[0]
    clean_df = clean_df.drop_duplicates(subset=["Time"], keep="first")

    data_matrix = clean_df.to_numpy()
    rows, cols = data_matrix.shape
    stop_time = float(clean_df["Time"].iloc[-1])

    with open(OUTPUT_FILE, "w", encoding="utf-8") as f_out:
        f_out.write(f"#1\ndouble {TABLE_NAME}({rows}, {cols})\n")
        for row in data_matrix:
            f_out.write(", ".join(f"{val:.4f}" for val in row) + "\n")

    # Auto-write run_lap.mos with exact lap stopTime
    with open(MOS_FILE, "w", encoding="utf-8") as f_mos:
        f_mos.write(
            'loadModel(Modelica);\n'
            'loadFile("LateModel43.mo");\n'
            f'simulate(LateModel43.Correlator, stopTime={stop_time:.3f}, numberOfIntervals=1000, outputFormat="csv");\n'
            'getErrorString();\n'
        )

    print(f"SUCCESS: {input_file} -> {OUTPUT_FILE} ({rows}x{cols}, Lap {TARGET_LAP}, stopTime={stop_time:.3f}s)")
    print(f"UPDATED: {MOS_FILE} (run: omc {MOS_FILE})")


if __name__ == "__main__":
    clean_to_modelica_txt()