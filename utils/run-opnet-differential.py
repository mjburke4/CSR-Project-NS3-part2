#!/usr/bin/env python3
"""Run an imported CSR scenario in ns-3 and compare it with an OPNET trace."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, type=Path, help="canonical scenario CSV")
    parser.add_argument(
        "--runner",
        required=True,
        type=Path,
        help="csr-opnet-scenario-runner executable",
    )
    parser.add_argument(
        "--opnet-trace",
        required=True,
        type=Path,
        help="OPNET canonical/alias trace CSV",
    )
    parser.add_argument("--output-dir", required=True, type=Path, help="run artifacts directory")
    parser.add_argument("--stop", type=float, default=0.0, help="override imported stop time")
    parser.add_argument("--flow-limit", type=int, default=0, help="maximum packets per flow")
    parser.add_argument("--no-duty-cycling", action="store_true")
    parser.add_argument("--no-gateway-discovery", action="store_true")
    parser.add_argument(
        "--tolerance",
        action="append",
        default=[],
        metavar="FIELD=VALUE",
        help="forward an absolute tolerance to the comparator",
    )
    parser.add_argument(
        "--ignore-field",
        action="append",
        default=[],
        help="forward a canonical ignored field to the comparator",
    )
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    for path, label in (
        (arguments.scenario, "scenario"),
        (arguments.runner, "runner"),
        (arguments.opnet_trace, "OPNET trace"),
    ):
        if not path.is_file():
            raise SystemExit(f"error: {label} does not exist: {path}")
    if arguments.stop < 0.0 or arguments.flow_limit < 0:
        raise SystemExit("error: --stop and --flow-limit must be nonnegative")

    arguments.output_dir.mkdir(parents=True, exist_ok=True)
    ns3_trace = arguments.output_dir / "ns3-trace.csv"
    normalized_opnet = arguments.output_dir / "opnet-normalized.csv"
    normalized_ns3 = arguments.output_dir / "ns3-normalized.csv"
    report = arguments.output_dir / "differential-report.json"
    run_log = arguments.output_dir / "ns3-run.log"
    comparator = Path(__file__).with_name("compare-opnet-ns3-traces.py")

    runner_command = [
        str(arguments.runner),
        f"--scenario={arguments.scenario}",
        f"--trace={ns3_trace}",
        f"--stop={arguments.stop}",
        f"--flowLimit={arguments.flow_limit}",
        f"--dutyCycling={0 if arguments.no_duty_cycling else 1}",
        f"--gatewayDiscovery={0 if arguments.no_gateway_discovery else 1}",
    ]
    completed = subprocess.run(
        runner_command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    run_log.write_text(completed.stdout, encoding="utf-8")
    if completed.returncode != 0:
        print(completed.stdout, end="", file=sys.stderr)
        raise SystemExit(f"ns-3 runner failed with status {completed.returncode}")

    compare_command = [
        sys.executable,
        str(comparator),
        str(arguments.opnet_trace),
        str(ns3_trace),
        "--report",
        str(report),
        "--normalized-opnet",
        str(normalized_opnet),
        "--normalized-ns3",
        str(normalized_ns3),
    ]
    for tolerance in arguments.tolerance:
        compare_command.extend(("--tolerance", tolerance))
    for field in arguments.ignore_field:
        compare_command.extend(("--ignore-field", field))
    comparison = subprocess.run(compare_command, text=True, check=False)

    manifest = {
        "schema": "csr-differential-run-v1",
        "scenario": str(arguments.scenario.resolve()),
        "scenario_sha256": digest(arguments.scenario),
        "opnet_trace": str(arguments.opnet_trace.resolve()),
        "opnet_trace_sha256": digest(arguments.opnet_trace),
        "runner": str(arguments.runner.resolve()),
        "runner_command": runner_command,
        "comparator_command": compare_command,
        "ns3_exit_code": completed.returncode,
        "comparison_exit_code": comparison.returncode,
        "artifacts": {
            "ns3_trace": ns3_trace.name,
            "normalized_opnet": normalized_opnet.name,
            "normalized_ns3": normalized_ns3.name,
            "report": report.name,
            "run_log": run_log.name,
        },
    }
    (arguments.output_dir / "run-manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    raise SystemExit(comparison.returncode)


if __name__ == "__main__":
    main()
