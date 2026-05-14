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

    algorithms = [
        "hybrid_dpso_algorithm",
        "onruntime_algorithm",
        "pair_algorithm",
    ]

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
                    # ==========================================
                    # RESULT PARAMETERS
                    # ==========================================
                    "Energy": extract_scalar(
                        report_file,
                        r"\+\s*Total energy\s*:\s*([0-9\.eE+-]+)\s*\(J\)",
                    ),
                    "Power": extract_scalar(
                        report_file,
                        r"\+\s*Avg power\s*:\s*([0-9\.eE+-]+)\s*\(J/cycle\)",
                    ),
                    "Delay": extract_scalar(
                        report_file,
                        r"Global average delay\s*:\s*([0-9\.eE+-]+)\s*\(cycles\)",
                    ),
                    "Throughput": extract_scalar(
                        report_file,
                        r"Global average throughput\s*:\s*([0-9\.eE+-]+)\s*\(flits/cycle\)",
                    ),
                    "Running Time": extract_scalar(
                        ref_file,
                        r"Running time:\s*([0-9\.eE+-]+)\s*seconds",
                    ),
                    "Peak Temperature": peak_temp,
                    "Average Temperature": avg_temp,
                    # ==========================================
                    # EVIDENCE PARAMETERS
                    # ==========================================
                    "Avg Node Layer": extract_scalar(
                        ref_file,
                        r"Avg node layer:\s*([0-9\.eE+-]+)",
                    ),
                    "Avg Edges TSV": extract_scalar(
                        ref_file,
                        r"Avg Edges on tsv:\s*([0-9\.eE+-]+)",
                    ),
                }

                data_rows.append(row)

    return pd.DataFrame(data_rows)


# ==========================================
# PRINT RUNNING TIME SEPARATELY
# ==========================================


def print_running_time_per_application(df):
    """Print running time separately for each application and algorithm."""

    if df.empty:
        print("No data available!")
        return

    print("\n========== RUNNING TIME RESULTS ==========")

    benchmarks = df["Benchmark"].unique()

    for bench in benchmarks:

        print(f"\nBenchmark: {bench}")

        bench_df = df[df["Benchmark"] == bench]

        applications = bench_df["Application"].unique()

        for app in applications:

            print(f"\n  Application: {app}")

            app_df = bench_df[bench_df["Application"] == app]

            for _, row in app_df.iterrows():

                algo_name = (
                    row["Algorithm"]
                    .replace("_algorithm", "")
                    .replace("_", " ")
                    .title()
                )

                running_time = row["Running Time"]

                print(
                    f"    {algo_name:<20} Running Time : {running_time:.4f} seconds"
                )


# ==========================================
# PRINT DELAYS SEPARATELY
# ==========================================


def print_delay_per_application(df):
    """Print delay separately for each application and algorithm."""

    if df.empty:
        print("No data available!")
        return

    print("\n========== DELAY RESULTS ==========")

    benchmarks = df["Benchmark"].unique()

    for bench in benchmarks:

        print(f"\nBenchmark: {bench}")

        bench_df = df[df["Benchmark"] == bench]

        applications = bench_df["Application"].unique()

        for app in applications:

            print(f"\n  Application: {app}")

            app_df = bench_df[bench_df["Application"] == app]

            for _, row in app_df.iterrows():

                algo_name = (
                    row["Algorithm"]
                    .replace("_algorithm", "")
                    .replace("_", " ")
                    .title()
                )

                delay = row["Delay"]

                print(f"    {algo_name:<20} Delay : {delay:.4f} cycles")


# ==========================================
# PRINT THROUGHPUT SEPARATELY
# ==========================================


def print_throughput_per_application(df):
    """Print throughput separately for each application and algorithm."""

    if df.empty:
        print("No data available!")
        return

    print("\n========== THROUGHPUT RESULTS ==========")

    benchmarks = df["Benchmark"].unique()

    for bench in benchmarks:

        print(f"\nBenchmark: {bench}")

        bench_df = df[df["Benchmark"] == bench]

        applications = bench_df["Application"].unique()

        for app in applications:

            print(f"\n  Application: {app}")

            app_df = bench_df[bench_df["Application"] == app]

            for _, row in app_df.iterrows():

                algo_name = (
                    row["Algorithm"]
                    .replace("_algorithm", "")
                    .replace("_", " ")
                    .title()
                )

                throughput = row["Throughput"]

                print(
                    f"    {algo_name:<20} Throughput : {throughput:.6f} flits/cycle"
                )


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

    parameters = [
        "Energy",
        "Power",
        "Delay",
        "Throughput",
        "Running Time",
        "Peak Temperature",
        "Average Temperature",
        "Avg Node Layer",
        "Avg Edges TSV",
    ]

    color_map = {
        "hybrid_dpso_algorithm": "#1f77b4",
        "onruntime_algorithm": "#ff7f0e",
        "pair_algorithm": "#2ca02c",
    }

    for bench in benchmarks:

        bench_df = df[df["Benchmark"] == bench]

        for param in parameters:

            param_df = bench_df.dropna(subset=[param])

            if param_df.empty:
                continue

            pivot_df = param_df.pivot(
                index="Application",
                columns="Algorithm",
                values=param,
            )

            ordered_algos = [
                a for a in color_map.keys() if a in pivot_df.columns
            ]

            pivot_df = pivot_df[ordered_algos]

            fig, ax = plt.subplots(figsize=(10, 6))

            x_indexes = np.arange(len(pivot_df.index))

            bar_width = 0.25

            for i, algo in enumerate(pivot_df.columns):

                offset = (
                    (i - len(pivot_df.columns) / 2)
                    * bar_width
                    + bar_width / 2
                )

                # ==========================================
                # RUNNING TIME VISUAL CAP
                # ==========================================

                CAP_VALUE = 0.2

                real_values = pivot_df[algo]

                display_values = []

                for val in real_values:

                    if param == "Running Time" and val > CAP_VALUE:
                        display_values.append(CAP_VALUE)
                    else:
                        display_values.append(val)

                bars = ax.bar(
                    x_indexes + offset,
                    display_values,
                    width=bar_width,
                    label=algo.replace("_algorithm", "")
                    .replace("_", " ")
                    .title(),
                    color=color_map[algo],
                    edgecolor="black",
                )

                # ==========================================
                # WRITE ACTUAL VALUES ON TOP OF BARS
                # ==========================================

                for bar, real_val in zip(bars, real_values):

                    height = bar.get_height()

                    ax.text(
                        bar.get_x() + bar.get_width() / 2,
                        height,
                        f"{real_val:.4f}",
                        ha="center",
                        va="bottom",
                        fontsize=8,
                        rotation=90,
                    )

                    # Break indicator for capped bars
                    if (
                        param == "Running Time"
                        and real_val > CAP_VALUE
                    ):

                        ax.text(
                            bar.get_x() + bar.get_width() / 2,
                            CAP_VALUE * 0.95,
                            "//",
                            ha="center",
                            va="bottom",
                            fontsize=12,
                            fontweight="bold",
                            color="red",
                        )

            ax.set_xlabel(
                "Applications / NoC Size",
                fontweight="bold",
                fontsize=12,
            )

            # Better y-axis for capped running time graph
            if param == "Running Time":
                ax.set_ylim(0, CAP_VALUE * 1.2)

            ax.set_title(
                f'{param} Comparison - {bench.replace("_", " ").title()}',
                fontweight="bold",
                fontsize=14,
            )

            # ==========================================
            # TEMPERATURE GRAPH FIX
            # ==========================================

            if param in ["Peak Temperature", "Average Temperature"]:

                min_temp = pivot_df.min().min()
                max_temp = pivot_df.max().max()

                temp_diff = max_temp - min_temp

                if temp_diff > 0:

                    lower_bound = min_temp - (temp_diff * 0.5)
                    upper_bound = max_temp + (temp_diff * 0.3)

                    ax.set_ylim(
                        bottom=max(0, lower_bound),
                        top=upper_bound,
                    )

                else:
                    ax.set_ylim(
                        bottom=max(0, min_temp * 0.95)
                    )

            else:
                ax.set_ylim(bottom=0)

            # ==========================================

            ax.set_xticks(x_indexes)

            ax.set_xticklabels(
                pivot_df.index,
                rotation=15,
                ha="right",
            )

            ax.yaxis.grid(True, linestyle="--", alpha=0.7)

            ax.set_axisbelow(True)

            ax.legend(title="Algorithms", fontsize=10)

            plt.tight_layout()

            filename = f"{bench}_{param.replace(' ', '_')}.png"

            filepath = os.path.join(output_dir, filename)

            plt.savefig(
                filepath,
                dpi=300,
                bbox_inches="tight",
            )

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

        # Print values
        print_delay_per_application(df)
        print_throughput_per_application(df)
        print_running_time_per_application(df)

        print("\nGenerating publication graphs...")

        create_publication_graphs(df)

        print("\nAll tasks completed successfully!")
