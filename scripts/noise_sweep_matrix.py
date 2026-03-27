#!/usr/bin/env python3
"""
Noise Sweep Matrix for NEMA23 Digital Twin Simulator.

Systematically sweeps encoder/velocity/command noise parameters and records
convergence metrics for each combination.

Usage:
  python3 scripts/noise_sweep_matrix.py [--fast] [--output results.csv]

Options:
  --fast         Run 2x2x2=8 combinations with timeout=30s (default)
  --full         Run 4x4x4=64 combinations with timeout=180s
  --output       CSV output path (default: /tmp/noise_sweep_results.csv)
  --skip-docker  Skip actual sim; write all rows as skipped (useful in CI)
"""

import argparse
import csv
import itertools
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Parameter grids
FAST_GRID: Dict[str, List[float]] = {
    "encoder_noise_deg":  [0.1, 0.5],
    "velocity_noise_dps": [0.5, 3.0],
    "command_noise_hz":   [0.5, 2.5],
}

FULL_GRID: Dict[str, List[float]] = {
    "encoder_noise_deg":  [0.1, 0.3, 0.5, 1.0],
    "velocity_noise_dps": [0.5, 1.5, 3.0, 5.0],
    "command_noise_hz":   [0.5, 1.5, 2.5, 5.0],
}

# Fixed scenario parameters
AZ_TARGET = 90.0
EL_TARGET = 45.0
TOLERANCE_DEG = 2.0

CSV_FIELDNAMES = [
    "encoder_noise_deg",
    "velocity_noise_dps",
    "command_noise_hz",
    "timeout_sec",
    "converged",
    "convergence_time_sec",
    "final_az_error_deg",
    "final_el_error_deg",
]


def _docker_image_available(image: str = "antenna_tracker_env:latest") -> bool:
    """Return True if Docker is accessible and the given image exists."""
    try:
        result = subprocess.run(
            ["docker", "image", "inspect", image],
            capture_output=True,
            timeout=10,
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False


def _build_combinations(grid: Dict[str, List[float]]) -> List[Tuple[float, float, float]]:
    """Return all (encoder_noise, velocity_noise, command_noise) combos."""
    return list(itertools.product(
        grid["encoder_noise_deg"],
        grid["velocity_noise_dps"],
        grid["command_noise_hz"],
    ))


def _run_single_e2e(
    encoder_noise: float,
    velocity_noise: float,
    command_noise: float,
    timeout_sec: float,
    workspace: Path,
) -> Tuple[bool, float, Optional[float], Optional[float]]:
    """
    Run one E2E combination via Docker.

    Returns (converged, elapsed_sec, az_error, el_error).
    az_error / el_error are None when not determinable.
    """
    sim_launch_args = (
        f"encoder_noise_deg:={encoder_noise} "
        f"velocity_noise_dps:={velocity_noise} "
        f"command_noise_hz:={command_noise}"
    )

    env = os.environ.copy()
    env.update({
        "SIM_LAUNCH_ARGS": sim_launch_args,
        "SIM_AUTO_TIMEOUT": str(int(timeout_sec)),
        "SIM_AUTO_AZ_TOL": str(TOLERANCE_DEG),
        "SIM_AUTO_EL_TOL": str(TOLERANCE_DEG),
    })

    cmd = [
        "docker", "run", "--rm",
        "-e", f"SIM_LAUNCH_ARGS={sim_launch_args}",
        "-e", f"SIM_AUTO_TIMEOUT={int(timeout_sec)}",
        "-e", f"SIM_AUTO_AZ_TOL={TOLERANCE_DEG}",
        "-e", f"SIM_AUTO_EL_TOL={TOLERANCE_DEG}",
        "-v", f"{workspace}:/ros2_ws",
        "antenna_tracker_env:latest",
        "bash", "-lc",
        (
            "set -e && "
            "cd /ros2_ws && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            "bash /ros2_ws/scripts/test_sim_auto_e2e.sh"
        ),
    ]

    t_start = time.monotonic()
    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_sec + 60,
        )
    except subprocess.TimeoutExpired:
        elapsed = time.monotonic() - t_start
        return False, elapsed, None, None

    elapsed = time.monotonic() - t_start
    stdout = result.stdout + result.stderr

    converged = result.returncode == 0 and "AUTO E2E converged" in stdout

    az_error: Optional[float] = None
    el_error: Optional[float] = None
    for line in stdout.splitlines():
        if line.startswith("az_error="):
            try:
                az_error = float(line.split("=", 1)[1])
            except ValueError:
                pass
        elif line.startswith("el_error="):
            try:
                el_error = float(line.split("=", 1)[1])
            except ValueError:
                pass

    return converged, elapsed, az_error, el_error


def _skipped_row(
    encoder_noise: float,
    velocity_noise: float,
    command_noise: float,
    timeout_sec: float,
) -> Dict[str, object]:
    return {
        "encoder_noise_deg": encoder_noise,
        "velocity_noise_dps": velocity_noise,
        "command_noise_hz": command_noise,
        "timeout_sec": timeout_sec,
        "converged": "skipped",
        "convergence_time_sec": "",
        "final_az_error_deg": "",
        "final_el_error_deg": "",
    }


def _result_row(
    encoder_noise: float,
    velocity_noise: float,
    command_noise: float,
    timeout_sec: float,
    converged: bool,
    elapsed: float,
    az_error: Optional[float],
    el_error: Optional[float],
) -> Dict[str, object]:
    return {
        "encoder_noise_deg": encoder_noise,
        "velocity_noise_dps": velocity_noise,
        "command_noise_hz": command_noise,
        "timeout_sec": timeout_sec,
        "converged": "true" if converged else "false",
        "convergence_time_sec": f"{elapsed:.2f}",
        "final_az_error_deg": f"{az_error:.4f}" if az_error is not None else "",
        "final_el_error_deg": f"{el_error:.4f}" if el_error is not None else "",
    }


def run_sweep(
    grid: Dict[str, List[float]],
    timeout_sec: float,
    output_path: Path,
    skip_docker: bool,
) -> int:
    """
    Run all combinations and write results to CSV.

    Returns 0 on success, 1 if any combination failed to converge.
    """
    combos = _build_combinations(grid)
    total = len(combos)
    workspace = Path(__file__).resolve().parent.parent

    docker_available = False
    if not skip_docker:
        docker_available = _docker_image_available()
        if not docker_available:
            print(
                "WARNING: Docker or antenna_tracker_env:latest image not available. "
                "All combinations will be marked as skipped.",
                file=sys.stderr,
            )

    output_path.parent.mkdir(parents=True, exist_ok=True)

    n_converged = 0
    n_failed = 0
    n_skipped = 0

    with output_path.open("w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()

        for idx, (enc, vel, cmd) in enumerate(combos, start=1):
            print(
                f"[{idx:>3}/{total}] "
                f"encoder={enc:.2f}deg  "
                f"velocity={vel:.2f}dps  "
                f"command={cmd:.2f}hz  ...",
                end=" ",
                flush=True,
            )

            if skip_docker or not docker_available:
                row = _skipped_row(enc, vel, cmd, timeout_sec)
                writer.writerow(row)
                n_skipped += 1
                print("skipped")
                continue

            converged, elapsed, az_error, el_error = _run_single_e2e(
                enc, vel, cmd, timeout_sec, workspace
            )
            row = _result_row(enc, vel, cmd, timeout_sec, converged, elapsed, az_error, el_error)
            writer.writerow(row)

            if converged:
                n_converged += 1
                print(f"converged ({elapsed:.1f}s)")
            else:
                n_failed += 1
                print(f"FAILED ({elapsed:.1f}s)")

    print()
    print("=== Noise Sweep Summary ===")
    print(f"  Total combinations : {total}")
    print(f"  Converged          : {n_converged}")
    print(f"  Failed             : {n_failed}")
    print(f"  Skipped            : {n_skipped}")
    print(f"  Results CSV        : {output_path}")

    # Print table header
    print()
    header = (
        f"{'enc_deg':>8}  "
        f"{'vel_dps':>8}  "
        f"{'cmd_hz':>7}  "
        f"{'result':>10}  "
        f"{'time_s':>7}  "
        f"{'az_err':>8}  "
        f"{'el_err':>8}"
    )
    print(header)
    print("-" * len(header))

    with output_path.open(newline="") as fh:
        reader = csv.DictReader(fh)
        for row in reader:
            conv = row["converged"]
            print(
                f"{float(row['encoder_noise_deg']):>8.2f}  "
                f"{float(row['velocity_noise_dps']):>8.2f}  "
                f"{float(row['command_noise_hz']):>7.2f}  "
                f"{conv:>10}  "
                f"{row['convergence_time_sec'] or '':>7}  "
                f"{row['final_az_error_deg'] or '':>8}  "
                f"{row['final_el_error_deg'] or '':>8}"
            )

    return 1 if n_failed > 0 else 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Noise Sweep Matrix for NEMA23 Digital Twin Simulator"
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--fast",
        action="store_true",
        default=True,
        help="Run 2x2x2=8 combinations with timeout=30s (default)",
    )
    mode_group.add_argument(
        "--full",
        action="store_true",
        default=False,
        help="Run 4x4x4=64 combinations with timeout=180s",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/noise_sweep_results.csv"),
        help="CSV output path (default: /tmp/noise_sweep_results.csv)",
    )
    parser.add_argument(
        "--skip-docker",
        action="store_true",
        default=False,
        help="Skip actual simulation; mark all rows as skipped (for CI matrix validation)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.full:
        grid = FULL_GRID
        timeout_sec = 180.0
        mode_label = "full (4x4x4=64 combinations, timeout=180s)"
    else:
        grid = FAST_GRID
        timeout_sec = 30.0
        mode_label = "fast (2x2x2=8 combinations, timeout=30s)"

    total = len(list(itertools.product(*grid.values())))
    print(f"=== Noise Sweep Matrix — {mode_label} ===")
    print(f"Output: {args.output}")
    print(f"Skip docker: {args.skip_docker}")
    print()

    return run_sweep(
        grid=grid,
        timeout_sec=timeout_sec,
        output_path=args.output,
        skip_docker=args.skip_docker,
    )


if __name__ == "__main__":
    sys.exit(main())
