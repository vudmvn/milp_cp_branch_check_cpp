#!/usr/bin/env python3
import sys
import pandas as pd
import numpy as np

def compare_csvs(baseline_path, improved_path):
    try:
        df_base = pd.read_csv(baseline_path)
        df_imp = pd.read_csv(improved_path)
    except Exception as e:
        print(f"Error reading CSV files: {e}")
        return

    # Merge on instance name
    merged = pd.merge(df_base, df_imp, on="instance", suffixes=("_base", "_imp"))
    if merged.empty:
        print("No matching instances found between the two files.")
        return

    print("=" * 80)
    print(f"SO SÁNH HIỆU NĂNG: BASELINE vs. CẢI TIẾN MỚI")
    print(f"  Baseline: {baseline_path} ({len(df_base)} instances)")
    print(f"  Cải tiến: {improved_path} ({len(df_imp)} instances)")
    print(f"  Số instance chung: {len(merged)}")
    print("=" * 80)

    # Detailed table
    print(f"{'Instance':<22} | {'Obj Base':<9} {'Obj Imp':<9} | {'Gap Base':<9} {'Gap Imp':<9} | {'Time Base':<10} {'Time Imp':<10} {'Speedup':<8}")
    print("-" * 95)

    speedups = []
    gap_diffs = []

    for _, row in merged.iterrows():
        inst = str(row['instance'])[:20]
        obj_b = row.get('obj_base', -1)
        obj_i = row.get('obj_imp', -1)
        gap_b = row.get('mip_gap_base', -1)
        gap_i = row.get('mip_gap_imp', -1)
        time_b = row.get('runtime_sec_base', 0)
        time_i = row.get('runtime_sec_imp', 0)

        # Speedup ratio (time_base / time_imp)
        speedup = (time_b / time_i) if time_i > 0.05 else (time_b / 0.05)
        speedups.append(speedup)
        if gap_b >= 0 and gap_i >= 0:
            gap_diffs.append(gap_b - gap_i)

        print(f"{inst:<22} | {obj_b:<9.1f} {obj_i:<9.1f} | {gap_b*100:<8.2f}% {gap_i*100:<8.2f}% | {time_b:<9.2f}s {time_i:<9.2f}s {speedup:<7.2f}x")

    print("-" * 95)
    print("TỔNG KẾT TRUNG BÌNH:")
    print(f"  • Thời gian chạy trung bình : Baseline = {merged['runtime_sec_base'].mean():.2f}s | Cải tiến = {merged['runtime_sec_imp'].mean():.2f}s")
    print(f"  • Tốc độ tăng tốc trung bình: {np.mean(speedups):.2f}x")
    if gap_diffs:
        print(f"  • Giảm MIP Gap trung bình   : {np.mean(gap_diffs)*100:.2f}%")
    print("=" * 80)

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Cách dùng:")
        print("  python3 compare_results.py <baseline.csv> <improved.csv>")
        sys.exit(1)
    compare_csvs(sys.argv[1], sys.argv[2])
