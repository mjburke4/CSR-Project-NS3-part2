#!/usr/bin/env python3
"""Run an imported CSR scenario in ns-3 and compare it with an OPNET trace."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import stat
import subprocess
import sys
import tempfile


RUN_SCHEMA = "csr-differential-run-v1"
MANIFEST_NAME = "run-manifest.json"
MANAGED_FILENAMES = (
    "ns3-trace.csv",
    "opnet-normalized.csv",
    "ns3-normalized.csv",
    "differential-report.json",
    "ns3-run.log",
    MANIFEST_NAME,
)
INSTRUMENTATION_SCHEMA = "csr-opnet-instrumentation-v1"
TRACE_SCHEMA = "csr-differential-trace-v1"
INSTRUMENTATION_FIELDS = {
    "schema",
    "trace_schema",
    "generator",
    "forced_reservation_slots",
    "activation_time_s",
    "behavioral_control",
    "sources",
}
INSTRUMENTATION_SOURCE_NAMES = {
    "br_app.pr.c",
    "br_mac.pr.c",
    "br_ecc.ps.c",
    "br_support.h",
    "br_txdel.ps.c",
    "csr-opnet-trace.h",
}
INSTRUMENTATION_GENERATED_NAMES = {
    "br_app.pr.c",
    "br_mac.pr.c",
    "br_ecc.ps.c",
    "csr-opnet-trace.h",
}


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def file_record(path: Path) -> dict[str, object]:
    return {
        "path": str(path.resolve()),
        "sha256": digest(path),
        "size_bytes": path.stat().st_size,
    }


def paths_alias(first: Path, second: Path) -> bool:
    try:
        if first.resolve(strict=False) == second.resolve(strict=False):
            return True
        return first.samefile(second)
    except FileNotFoundError:
        return False
    except OSError as error:
        raise ValueError(f"cannot compare paths {first} and {second}: {error}") from error


def output_directory_identity(output_dir: Path) -> tuple[int, int]:
    """Return the stable device/inode identity of a real output directory."""

    metadata = os.lstat(output_dir)
    if not stat.S_ISDIR(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise ValueError("--output-dir changed identity or is not a real directory")
    return metadata.st_dev, metadata.st_ino


def validate_managed_outputs(
    output_dir: Path,
    expected_directory_identity: tuple[int, int],
    protected: dict[str, Path],
    *,
    allowed_present: set[str],
    required_present: set[str],
) -> None:
    """Reject injected links, special files, and unexpected cross-stage outputs."""

    if output_directory_identity(output_dir) != expected_directory_identity:
        raise ValueError("--output-dir changed identity during the workflow")
    observed: set[str] = set()
    for name in MANAGED_FILENAMES:
        path = output_dir / name
        try:
            metadata = os.lstat(path)
        except FileNotFoundError:
            continue
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise ValueError(
                f"managed output {name!r} is not a single-link regular file"
            )
        observed.add(name)
        for label, protected_path in protected.items():
            if paths_alias(path, protected_path):
                raise ValueError(
                    f"managed output {name!r} aliases {label} input"
                )
    unexpected = sorted(observed - allowed_present)
    missing = sorted(required_present - observed)
    if unexpected:
        raise ValueError(f"unexpected managed outputs appeared: {unexpected}")
    if missing:
        raise ValueError(f"required managed outputs are absent: {missing}")


def atomic_write_text(path: Path, text: str) -> None:
    """Atomically replace one parent-written artifact without following links."""

    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        metadata = os.lstat(path)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_nlink != 1:
            raise ValueError(f"atomic output {path.name!r} is not a regular file")
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def reject_duplicate_json_keys(pairs: list[tuple[str, object]]) -> dict[str, object]:
    result: dict[str, object] = {}
    for name, value in pairs:
        if name in result:
            raise ValueError(f"instrumentation metadata repeats JSON key {name!r}")
        result[name] = value
    return result


def _require_sha256(value: object, label: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise ValueError(f"{label} is not a lowercase SHA-256 digest")
    return value


def validate_instrumentation_metadata(
    manifest_path: Path, trace_path: Path
) -> dict[str, object]:
    """Validate source-bundle metadata while refusing to claim trace origin."""

    try:
        manifest = json.loads(
            manifest_path.read_text(encoding="utf-8"),
            object_pairs_hook=reject_duplicate_json_keys,
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot read OPNET instrumentation metadata: {error}") from error
    if not isinstance(manifest, dict) or set(manifest) != INSTRUMENTATION_FIELDS:
        raise ValueError("OPNET instrumentation metadata has the wrong shape")
    if (
        manifest["schema"] != INSTRUMENTATION_SCHEMA
        or manifest["trace_schema"] != TRACE_SCHEMA
    ):
        raise ValueError("OPNET instrumentation metadata has an unsupported schema")
    generator = manifest["generator"]
    if (
        not isinstance(generator, dict)
        or set(generator) != {"name", "sha256"}
        or generator["name"] != "instrument-opnet-reservation-trace.py"
    ):
        raise ValueError("OPNET instrumentation generator record is malformed")
    _require_sha256(generator["sha256"], "instrumentation generator sha256")
    activation = manifest["activation_time_s"]
    if (
        not isinstance(activation, (int, float))
        or isinstance(activation, bool)
        or not math.isfinite(float(activation))
        or float(activation) < 0.0
        or not isinstance(manifest["behavioral_control"], str)
        or not manifest["behavioral_control"]
    ):
        raise ValueError("OPNET instrumentation control metadata is malformed")
    forced = manifest["forced_reservation_slots"]
    if not isinstance(forced, dict):
        raise ValueError("OPNET instrumentation forced-slot ledger is malformed")
    for node, slot in forced.items():
        if (
            not isinstance(node, str)
            or not node.isdigit()
            or str(int(node)) != node
            or int(node) >= 0xFFFFFF
            or not isinstance(slot, int)
            or isinstance(slot, bool)
            or not 1 <= slot <= 255
        ):
            raise ValueError("OPNET instrumentation forced-slot entry is malformed")
    sources = manifest["sources"]
    if not isinstance(sources, dict) or set(sources) != INSTRUMENTATION_SOURCE_NAMES:
        raise ValueError("OPNET instrumentation source ledger is not exact")
    generated: dict[str, str] = {}
    for name, record in sources.items():
        if not isinstance(record, dict) or set(record) != {
            "original_sha256",
            "output_sha256",
        }:
            raise ValueError(f"instrumentation source {name!r} is malformed")
        _require_sha256(record["original_sha256"], f"{name} original_sha256")
        output_digest = record["output_sha256"]
        if name in INSTRUMENTATION_GENERATED_NAMES:
            expected = _require_sha256(output_digest, f"{name} output_sha256")
            generated_path = manifest_path.parent / name
            if (
                generated_path.is_symlink()
                or not generated_path.is_file()
                or digest(generated_path) != expected
            ):
                raise ValueError(
                    f"instrumentation generated source {name!r} is absent or changed"
                )
            generated[name] = expected
        elif output_digest is not None:
            raise ValueError(
                f"instrumentation reference source {name!r} claims an output"
            )

    row_count = 0
    with trace_path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if (
            reader.fieldnames is None
            or len(reader.fieldnames) != len(set(reader.fieldnames))
            or "schema" not in reader.fieldnames
        ):
            raise ValueError("instrumented OPNET trace lacks a unique schema column")
        for row_number, row in enumerate(reader, start=2):
            if (
                None in row
                or any(value is None for value in row.values())
                or row["schema"].strip() != TRACE_SCHEMA
            ):
                raise ValueError(
                    f"instrumented OPNET trace row {row_number} has invalid schema"
                )
            row_count += 1
    return {
        "classification": "validated_source_bundle_trace_origin_unverified",
        "certifying": False,
        "trace_origin_bound": False,
        "manifest_path": str(manifest_path.resolve()),
        "manifest_sha256": digest(manifest_path),
        "declared_trace_schema": TRACE_SCHEMA,
        "validated_trace_rows": row_count,
        "generated_source_sha256": generated,
    }


def validate_output_collisions(
    output_dir: Path, protected: dict[str, Path]
) -> None:
    """Reject every managed output that aliases an input or tool."""

    targets = [(name, output_dir / name) for name in MANAGED_FILENAMES]
    for target_name, target in targets:
        for label, path in protected.items():
            if paths_alias(target, path):
                raise ValueError(
                    f"managed output {target_name!r} aliases {label} input"
                )
    for index, (first_name, first) in enumerate(targets):
        for second_name, second in targets[index + 1 :]:
            if paths_alias(first, second):
                raise ValueError(
                    f"managed outputs {first_name!r} and {second_name!r} alias"
                )


def classify_output_directory(output_dir: Path) -> str:
    """Accept a new/empty directory or one carrying our valid ownership marker."""

    if output_dir.is_symlink():
        raise ValueError("--output-dir must not be a symbolic link")
    if not output_dir.exists():
        return "new"
    if not output_dir.is_dir():
        raise ValueError("--output-dir exists but is not a directory")
    entries = list(output_dir.iterdir())
    if not entries:
        return "empty"
    marker = output_dir / MANIFEST_NAME
    if marker.is_symlink() or not marker.is_file():
        raise ValueError(
            "nonempty output directory is not workflow-owned; expected a "
            f"regular {MANIFEST_NAME}"
        )
    try:
        previous = json.loads(marker.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"cannot validate output ownership manifest: {error}") from error
    if not isinstance(previous, dict) or previous.get("schema") != RUN_SCHEMA:
        raise ValueError("nonempty output directory has an invalid ownership manifest")
    for name in MANAGED_FILENAMES:
        target = output_dir / name
        if target.exists() and not target.is_file() and not target.is_symlink():
            raise ValueError(f"managed output {name!r} is not a file")
    return "owned"


def prepare_output_directory(output_dir: Path, classification: str) -> None:
    if classification == "new":
        output_dir.mkdir(parents=True, exist_ok=False)
    elif classification == "owned":
        for name in MANAGED_FILENAMES:
            target = output_dir / name
            if target.is_file() or target.is_symlink():
                target.unlink()


def validate_immutable_inputs(
    inputs: dict[str, Path], expected: dict[str, dict[str, object]]
) -> None:
    for label, path in inputs.items():
        if not path.is_file() or file_record(path) != expected[label]:
            raise ValueError(f"{label} identity changed after preflight")


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
    parser.add_argument(
        "--opnet-manifest",
        type=Path,
        help=(
            "optional OPNET source-instrumentation metadata; it is validated but "
            "does not by itself prove trace origin"
        ),
    )
    parser.add_argument("--output-dir", required=True, type=Path, help="run artifacts directory")
    parser.add_argument("--stop", type=float, default=0.0, help="override imported stop time")
    parser.add_argument("--flow-limit", type=int, default=0, help="maximum packets per flow")
    parser.add_argument("--no-duty-cycling", action="store_true")
    parser.add_argument("--no-gateway-discovery", action="store_true")
    parser.add_argument(
        "--key-fields",
        help="comma-separated comparator identity fields",
    )
    parser.add_argument(
        "--event",
        action="append",
        default=[],
        help="compare only this canonical event name (repeatable)",
    )
    parser.add_argument(
        "--select",
        action="append",
        default=[],
        metavar="EVENT:NODE",
        help="compare only this event/node pair (repeatable)",
    )
    parser.add_argument("--time-start", type=float, help="comparison window start")
    parser.add_argument("--time-stop", type=float, help="comparison window stop")
    parser.add_argument(
        "--coincident-tolerance",
        type=float,
        default=0.0,
        help="canonicalize ordering of effectively simultaneous events",
    )
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
    comparator = Path(__file__).resolve().with_name("compare-opnet-ns3-traces.py")
    inputs = {
        "scenario": arguments.scenario,
        "runner": arguments.runner,
        "OPNET trace": arguments.opnet_trace,
        "comparator": comparator,
    }
    if arguments.opnet_manifest is not None:
        inputs["OPNET manifest"] = arguments.opnet_manifest
    for label, path in inputs.items():
        if not path.is_file():
            raise SystemExit(f"error: {label} does not exist: {path}")
    if not os.access(arguments.runner, os.X_OK):
        raise SystemExit(f"error: runner is not executable: {arguments.runner}")
    if (
        not math.isfinite(arguments.stop)
        or arguments.stop < 0.0
        or arguments.flow_limit < 0
    ):
        raise SystemExit("error: --stop and --flow-limit must be nonnegative")
    if (
        not math.isfinite(arguments.coincident_tolerance)
        or arguments.coincident_tolerance < 0.0
    ):
        raise SystemExit(
            "error: --coincident-tolerance must be finite and nonnegative"
        )
    for boundary, label in (
        (arguments.time_start, "--time-start"),
        (arguments.time_stop, "--time-stop"),
    ):
        if boundary is not None and not math.isfinite(boundary):
            raise SystemExit(f"error: {label} must be finite")
    if (
        arguments.time_start is not None
        and arguments.time_stop is not None
        and arguments.time_start > arguments.time_stop
    ):
        raise SystemExit("error: --time-start must not exceed --time-stop")

    input_records = {label: file_record(path) for label, path in inputs.items()}
    try:
        instrumentation = (
            {
                "classification": "not_supplied_legacy_or_synthetic_trace",
                "certifying": False,
                "trace_origin_bound": False,
            }
            if arguments.opnet_manifest is None
            else validate_instrumentation_metadata(
                arguments.opnet_manifest, arguments.opnet_trace
            )
        )
    except (OSError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error
    try:
        validate_output_collisions(arguments.output_dir, inputs)
        output_classification = classify_output_directory(arguments.output_dir)
        validate_immutable_inputs(inputs, input_records)
        prepare_output_directory(arguments.output_dir, output_classification)
        output_identity = output_directory_identity(arguments.output_dir)
        validate_managed_outputs(
            arguments.output_dir,
            output_identity,
            inputs,
            allowed_present=set(),
            required_present=set(),
        )
    except (OSError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error
    ns3_trace = arguments.output_dir / "ns3-trace.csv"
    normalized_opnet = arguments.output_dir / "opnet-normalized.csv"
    normalized_ns3 = arguments.output_dir / "ns3-normalized.csv"
    report = arguments.output_dir / "differential-report.json"
    run_log = arguments.output_dir / "ns3-run.log"
    def guard(
        allowed_present: set[str], required_present: set[str]
    ) -> None:
        validate_immutable_inputs(inputs, input_records)
        validate_managed_outputs(
            arguments.output_dir,
            output_identity,
            inputs,
            allowed_present=allowed_present,
            required_present=required_present,
        )

    try:
        guard(set(), set())
    except (OSError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error

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
    try:
        guard({ns3_trace.name}, {ns3_trace.name} if completed.returncode == 0 else set())
        atomic_write_text(run_log, completed.stdout)
        guard(
            {ns3_trace.name, run_log.name},
            {run_log.name} | ({ns3_trace.name} if completed.returncode == 0 else set()),
        )
    except (OSError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error
    if completed.returncode != 0:
        print(completed.stdout, end="", file=sys.stderr)
        raise SystemExit(f"ns-3 runner failed with status {completed.returncode}")

    compare_command = [
        sys.executable,
        "-B",
        "-I",
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
    if arguments.key_fields:
        compare_command.extend(("--key-fields", arguments.key_fields))
    for event in arguments.event:
        compare_command.extend(("--event", event))
    for selector in arguments.select:
        compare_command.extend(("--select", selector))
    if arguments.time_start is not None:
        compare_command.extend(("--time-start", str(arguments.time_start)))
    if arguments.time_stop is not None:
        compare_command.extend(("--time-stop", str(arguments.time_stop)))
    if arguments.coincident_tolerance > 0.0:
        compare_command.extend(
            ("--coincident-tolerance", str(arguments.coincident_tolerance))
        )
    try:
        guard(
            {ns3_trace.name, run_log.name},
            {ns3_trace.name, run_log.name},
        )
    except (OSError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error
    comparison = subprocess.run(compare_command, text=True, check=False)
    try:
        comparator_outputs = {
            report.name,
            normalized_opnet.name,
            normalized_ns3.name,
        }
        final_without_manifest = {ns3_trace.name, run_log.name} | comparator_outputs
        guard(final_without_manifest, final_without_manifest)
    except (OSError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error

    manifest = {
        "schema": RUN_SCHEMA,
        "evidence_scope": "diagnostic_event_trace_not_opnet_origin_certification",
        "scenario": str(arguments.scenario.resolve()),
        "scenario_sha256": digest(arguments.scenario),
        "opnet_trace": str(arguments.opnet_trace.resolve()),
        "opnet_trace_sha256": digest(arguments.opnet_trace),
        "opnet_source_instrumentation": instrumentation,
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
    try:
        guard(final_without_manifest, final_without_manifest)
        atomic_write_text(
            arguments.output_dir / MANIFEST_NAME,
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        )
        guard(
            final_without_manifest | {MANIFEST_NAME},
            final_without_manifest | {MANIFEST_NAME},
        )
    except (OSError, ValueError) as error:
        raise SystemExit(f"error: {error}") from error
    raise SystemExit(comparison.returncode)


if __name__ == "__main__":
    main()
