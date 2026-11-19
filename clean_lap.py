import pandas as pd
import numpy as np
import os

input_file = "laptimeoswego.csv" 
output_file = "oswego_table.txt"

def clean_to_modelica_txt():
    if not os.path.exists(input_file):
        print(f"ERROR: Could not find {input_file} in the current directory.")
        return

    skip = 0
    with open(input_file, 'r') as f:
        for i, line in enumerate(f):
            if line.startswith('"Time"'):
                skip = i
                break

    df = pd.read_csv(input_file, skiprows=skip, low_memory=False)
    df = df.drop(0).reset_index(drop=True) 

    # 11 Channels for the advanced model
    cols_to_keep = [
        'Time', 'Brake Pos', 'CG Accel Lateral', 'CG Accel Longitudinal', 
        'Ground Speed', 'Gear', 'Engine RPM', 'Steering Angle', 'Throttle Pos',
        'Wheel Angular Speed FL', 'Wheel Angular Speed FR', 
        'Wheel Angular Speed RL', 'Wheel Angular Speed RR'
    ]
    
    try:
        clean_df = df[cols_to_keep].apply(pd.to_numeric, errors='coerce')
        clean_df = clean_df.fillna(0)
        data_matrix = clean_df.to_numpy()
        rows, cols = data_matrix.shape
        
        with open(output_file, 'w') as f_out:
            f_out.write(f"#1\ndouble oswego({rows}, {cols})\n")
            for row in data_matrix:
                f_out.write(", ".join(f"{val:.4f}" for val in row) + "\n")
        
        print(f"SUCCESS: Created {output_file} with {rows} rows x {cols} columns.")
    except KeyError as e:
        print(f"ERROR: Missing channels. Did you export Wheel Speeds? {e}")

if __name__ == "__main__":
    clean_to_modelica_txt()