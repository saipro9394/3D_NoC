import os
import re
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. EXTRACTION FUNCTIONS
# ==========================================


def extract_scalar(filepath, pattern):
    """Searches for a specific regex pattern and returns the first captured float."""
    if not os.path.exists(filepath):
        return None
    try:
        with open(filepath, "r") as f:
            content = f.read()
            match = re.search(pattern, content)
            if match:
                return float(match.group(1))
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
    return None


def get_steady_router_temps(algo_dir):
    """
    Extracts both PEAK and AVERAGE steady-state temperatures from the RouterTemp file.
    Returns a tuple: (peak_temp, avg_temp)
    """
    temp_dir = os.path.join(algo_dir, "results", "TEMP")
    if not os.path.exists(temp_dir):
        return None, None

    router_files = glob.glob(os.path.join(temp_dir, "RouterTemp*.txt"))
    if not router_files:
        return None, None

    filepath = router_files[0]

    try:
        with open(filepath, "r") as f:
            content = f.read()

            # 1. Split to get everything AFTER the Steady State marker
            parts = re.split(r"(?i)#\s*Steady\s*State[^\n]*", content)

            if len(parts) < 2:
                print(f"Warning: No Steady State data found in {filepath}")
                return None, None

            steady_state_data = parts[1]

            # 2. Split again to remove the MATLAB commands at the bottom
            matrix_data_only = re.split(r"color_range", steady_state_data)[0]

            # 3. Strip away the XY0 = [ ] formatting
            clean_content = re.sub(r"XY\d+\s*=\s*\[|\]", "", matrix_data_only)

            # 4. Extract all numerical values
            numbers = re.findall(r"[-+]?\d*\.\d+|\d+", clean_content)

            # 5. Filter out the 0 padding at the edge of the mesh
            temps = [float(n) for n in numbers if float(n) > 0]

            if temps:
                peak_temp = max(temps)
                avg_temp = sum(temps) / len(temps)
                print(
                    "peak temp: "
                    + str(peak_temp)
                    + " , "
                    + "avg temp: "
                    + str(avg_temp)
                )
                return peak_temp, avg_temp

    except Exception as e:
        print(f"Error processing {filepath}: {e}")

    return None, None


# ==========================================
# 2. DATA CRAWLER
# ==========================================


def collect_surya_data(root_dir="."):
    """Crawls the surya_results directory and builds the dataset."""
    surya_dir = os.path.join(root_dir, "surya_results")
    data_rows = []

    algorithms = ["hybrid_dpso_algorithm", "onruntime_algorithm", "pair_algorithm"]

    if not os.path.exists(surya_dir):
        print(
            f"Error: Could not find '{surya_dir}'. Please run the script from the directory containing it."
        )
        return pd.DataFrame()

    for benchmark in os.listdir(surya_dir):
        bench_path = os.path.join(surya_dir, benchmark)
        if not os.path.isdir(bench_path):
            continue

        for app in os.listdir(bench_path):
            app_path = os.path.join(bench_path, app)
            if not os.path.isdir(app_path):
                continue

            for algo in algorithms:
                algo_path = os.path.join(app_path, algo)
                if not os.path.exists(algo_path):
                    continue

                report_file = os.path.join(algo_path, "report_values.txt")
                ref_file = os.path.join(algo_path, "reference_values.txt")

                # Get both temperatures
                peak_temp, avg_temp = get_steady_router_temps(algo_path)

                row = {
                    "Benchmark": benchmark,
                    "Application": app,
                    "Algorithm": algo,
                    # Result Parameters
                    "Energy": extract_scalar(
                        report_file, r"\+\s*Total energy\s*:\s*([0-9\.eE+-]+)\s*\(J\)"
                    ),
                    "Power": extract_scalar(
                        report_file,
                        r"\+\s*Avg power\s*:\s*([0-9\.eE+-]+)\s*\(J/cycle\)",
                    ),
                    "Delay": extract_scalar(
                        report_file,
                        r"Global average delay\s*:\s*([0-9\.eE+-]+)\s*\(cycles\)",
                    ),
                    "Running Time": extract_scalar(
                        ref_file, r"Running time:\s*([0-9\.eE+-]+)\s*seconds"
                    ),
                    "Peak Temperature": peak_temp,
                    "Average Temperature": avg_temp,
                    # Evidence Parameters
                    "Avg Node Layer": extract_scalar(
                        ref_file, r"Avg node layer:\s*([0-9\.eE+-]+)"
                    ),
                    "Avg Edges TSV": extract_scalar(
                        ref_file, r"Avg Edges on tsv:\s*([0-9\.eE+-]+)"
                    ),
                }
                data_rows.append(row)

    return pd.DataFrame(data_rows)


# ==========================================
# 3. GRAPH GENERATOR
# ==========================================


def create_publication_graphs(df, output_dir="surya_graphs"):
    """Generates the grouped bar charts for the research paper."""
    if df.empty:
        print("No data to plot!")
        return

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    benchmarks = df["Benchmark"].unique()
    # Notice we now have both Peak and Average Temperature in the list
    parameters = [
        "Energy",
        "Power",
        "Delay",
        "Running Time",
        "Peak Temperature",
        "Average Temperature",
        "Avg Node Layer",
        "Avg Edges TSV",
    ]

    color_map = {
        "hybrid_dpso_algorithm": "#1f77b4",  # Blue
        "onruntime_algorithm": "#ff7f0e",  # Orange
        "pair_algorithm": "#2ca02c",  # Green
    }

    for bench in benchmarks:
        bench_df = df[df["Benchmark"] == bench]

        for param in parameters:
            param_df = bench_df.dropna(subset=[param])
            if param_df.empty:
                continue

            pivot_df = param_df.pivot(
                index="Application", columns="Algorithm", values=param
            )

            ordered_algos = [a for a in color_map.keys() if a in pivot_df.columns]
            pivot_df = pivot_df[ordered_algos]

            fig, ax = plt.subplots(figsize=(10, 6))
            x_indexes = np.arange(len(pivot_df.index))
            bar_width = 0.25

            for i, algo in enumerate(pivot_df.columns):
                offset = (i - len(pivot_df.columns) / 2) * bar_width + bar_width / 2
                ax.bar(
                    x_indexes + offset,
                    pivot_df[algo],
                    width=bar_width,
                    label=algo.replace("_algorithm", "").replace("_", " ").title(),
                    color=color_map[algo],
                    edgecolor="black",
                )

            ax.set_xlabel("Applications / NoC Size", fontweight="bold", fontsize=12)
            ax.set_ylabel(param, fontweight="bold", fontsize=12)
            ax.set_title(
                f'{param} Comparison - {bench.replace("_", " ").title()}',
                fontweight="bold",
                fontsize=14,
            )

            # --- THE Y-AXIS ILLUSION FIX (Now applies to BOTH temperature graphs) ---
            if param in ["Peak Temperature", "Average Temperature"]:
                min_temp = pivot_df.min().min()
                max_temp = pivot_df.max().max()

                temp_diff = max_temp - min_temp

                if temp_diff > 0:
                    lower_bound = min_temp - (temp_diff * 0.5)
                    upper_bound = max_temp + (temp_diff * 0.3)
                    ax.set_ylim(bottom=max(0, lower_bound), top=upper_bound)
                else:
                    ax.set_ylim(bottom=max(0, min_temp * 0.95))
            else:
                ax.set_ylim(bottom=0)
            # ------------------------------------------------------------------------

            ax.set_xticks(x_indexes)
            ax.set_xticklabels(pivot_df.index, rotation=15, ha="right")
            ax.yaxis.grid(True, linestyle="--", alpha=0.7)
            ax.set_axisbelow(True)
            ax.legend(title="Algorithms", fontsize=10)

            plt.tight_layout()

            filename = f"{bench}_{param.replace(' ', '_')}.png"
            filepath = os.path.join(output_dir, filename)
            plt.savefig(filepath, dpi=300, bbox_inches="tight")
            plt.close()
            print(f"Saved: {filepath}")


# ==========================================
# RUN THE SCRIPT
# ==========================================
if __name__ == "__main__":
    print("Starting data extraction...")
    df = collect_surya_data(".")

    if not df.empty:
        csv_path = "surya_extracted_data.csv"
        df.to_csv(csv_path, index=False)
        print(f"\nExtracted data saved to {csv_path}")

        print("\nGenerating publication graphs...")
        create_publication_graphs(df)
        print("\nAll tasks completed successfully!")
