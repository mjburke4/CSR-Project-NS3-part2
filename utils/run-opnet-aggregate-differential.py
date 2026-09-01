#!/usr/bin/env python3
"""Run a historical OPNET-output-vector aggregate differential workflow.

The workflow is intentionally fail-closed.  It corroborates scenario, seed,
and full-duration identities, decodes the supplied Modeler ``.ov`` file in
strict mode, verifies one complete time axis, runs the imported scenario,
requires exact application-sequence correlation, and compares an explicit
source-equivalent statistic set over the complete bucket-end window.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from typing import Any, Iterable


SCENARIO_SCHEMA = "csr-opnet-scenario-v1"
RUN_SCHEMA = "csr-opnet-aggregate-differential-run-v1"
NS3_PROVENANCE_SCHEMA = "csr-ns3-aggregate-provenance-v1"
APP_ADMISSION_DIAGNOSTICS_SCHEMA = "csr-app-admission-diagnostics-v1"
APPLICATION_PROFILES = {
    "current-send-only",
    "legacy-send-only-no-dscp",
    "legacy-send-to-from-no-dscp",
    "unspecified",
}
MAC_PROFILES = {
    "current-fine-free-slot",
    "hist-2014-coarse-inclusive-no-avoid",
    "hist-2014-zero-based-rebuild-list",
    "hist-2015-fine-one-based-table-no-avoid",
    "hist-2014-next-tslot-modulo-probe",
    "unspecified",
}

# These statistics are derived from the same application send/delivery events
# and remain the canonical default surface.  Source-equivalent HOP/MAC queue
# samples are available as explicit diagnostic selections; broadening the
# default would change the meaning of existing certifying runs.
DEFAULT_STATISTICS = (
    "Generator.Traffic Sent (packets/sec)",
    "Generator.Traffic Sent (bits/sec)",
    "Generator.Packet Size (bits)",
    "Sink.Traffic Received (packets/sec)",
    "Sink.Traffic Received (packets)",
    "Sink.Traffic Received (bits/sec)",
    "Sink.Traffic Received (bits)",
    "Sink.End-to-End Delay (seconds)",
)

ARTIFACT_PATHS = {
    "opnet_extract_manifest": "opnet-extract.json",
    "opnet_aggregates": "opnet-aggregates.csv",
    "opnet_extract_log": "opnet-extract.log",
    "ns3_trace": "ns3-trace.csv",
    "app_admission_diagnostics": "app-admission-diagnostics.csv",
    "ns3_run_log": "ns3-run.log",
    "ns3_aggregates": "ns3-aggregates.csv",
    "ns3_aggregate_provenance": "ns3-aggregates.provenance.json",
    "ns3_aggregate_log": "ns3-aggregate.log",
    "comparison_report": "aggregate-report.json",
    "normalized_aggregates": "aggregate-normalized.csv",
    "comparison_log": "aggregate-compare.log",
}
MANIFEST_NAME = "aggregate-run-manifest.json"
EVIDENCE_SCOPE = "historical_bucket_aggregates_not_event_parity"
NS3_RNG_SEED_MAX = 4294944442


class WorkflowError(ValueError):
    """A workflow precondition or generated artifact is not trustworthy."""

    def __init__(self, message: str, *, stage: str = "preflight", exit_code: int = 2):
        super().__init__(message)
        self.stage = stage
        self.exit_code = exit_code


def digest(path: Path) -> str:
    """Return a streaming SHA-256 digest for *path*."""

    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def file_record(path: Path) -> dict[str, Any]:
    """Build an absolute, content-addressed manifest record."""

    return {
        "path": str(path.resolve()),
        "sha256": digest(path),
        "size_bytes": path.stat().st_size,
    }


def _finite_nonnegative(raw: str) -> float:
    try:
        value = float(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and nonnegative")
    return value


def _finite_positive(raw: str) -> float:
    value = _finite_nonnegative(raw)
    if value == 0.0:
        raise argparse.ArgumentTypeError("value must be positive")
    return value


def _sha256_argument(raw: str) -> str:
    value = raw.strip().lower()
    if not re.fullmatch(r"[0-9a-f]{64}", value):
        raise argparse.ArgumentTypeError("value must be a 64-digit SHA-256 digest")
    return value


def _number(value: float) -> str:
    return format(value, ".17g")


def load_scenario_run(path: Path) -> dict[str, Any]:
    """Read the one canonical run row needed before invoking the C++ runner."""

    try:
        stream = path.open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise WorkflowError(str(error)) from error
    with stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise WorkflowError(f"{path}: scenario CSV has no header")
        if len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise WorkflowError(f"{path}: scenario CSV has duplicate columns")
        required = {
            "record",
            "schema",
            "scenario",
            "source_sha256",
            "duration_s",
            "seed",
        }
        missing = sorted(required - set(reader.fieldnames))
        if missing:
            raise WorkflowError(
                f"{path}: scenario CSV is missing columns: {', '.join(missing)}"
            )
        run_rows: list[tuple[int, dict[str, str]]] = []
        flow_count = 0
        for row_number, row in enumerate(reader, start=2):
            if None in row or any(value is None for value in row.values()):
                raise WorkflowError(f"{path}:{row_number}: malformed CSV row")
            if row["schema"].strip() != SCENARIO_SCHEMA:
                raise WorkflowError(
                    f"{path}:{row_number}: unsupported scenario schema "
                    f"{row['schema'].strip()!r}"
                )
            if row["record"].strip() == "run":
                run_rows.append((row_number, row))
            elif row["record"].strip() == "flow":
                flow_count += 1
        if len(run_rows) != 1:
            raise WorkflowError(
                f"{path}: expected exactly one run row, found {len(run_rows)}"
            )
        row_number, row = run_rows[0]
        scenario = row["scenario"].strip()
        if not scenario:
            raise WorkflowError(f"{path}:{row_number}: scenario is empty")
        application_profile = (
            row.get("application_profile", "").strip() or "unspecified"
        )
        if application_profile not in APPLICATION_PROFILES:
            raise WorkflowError(
                f"{path}:{row_number}: unsupported application_profile "
                f"{application_profile!r}"
            )
        mac_profile = row.get("mac_profile", "").strip() or "unspecified"
        if mac_profile not in MAC_PROFILES:
            raise WorkflowError(
                f"{path}:{row_number}: unsupported mac_profile {mac_profile!r}"
            )
        source_sha256 = row["source_sha256"].strip().lower()
        if not re.fullmatch(r"[0-9a-f]{64}", source_sha256):
            raise WorkflowError(
                f"{path}:{row_number}: source_sha256 is not a SHA-256 digest"
            )
        try:
            duration_s = float(row["duration_s"])
        except ValueError as error:
            raise WorkflowError(
                f"{path}:{row_number}: duration_s is not numeric"
            ) from error
        if not math.isfinite(duration_s) or duration_s <= 0.0:
            raise WorkflowError(
                f"{path}:{row_number}: duration_s must be finite and positive"
            )
        try:
            seed_value = float(row["seed"])
        except ValueError as error:
            raise WorkflowError(
                f"{path}:{row_number}: seed is not numeric"
            ) from error
        if (
            not math.isfinite(seed_value)
            or seed_value < 1.0
            or seed_value > float(NS3_RNG_SEED_MAX)
            or not seed_value.is_integer()
        ):
            raise WorkflowError(
                f"{path}:{row_number}: seed must be a finite integer in "
                f"1..{NS3_RNG_SEED_MAX}"
            )
        return {
            "scenario": scenario,
            "application_profile": application_profile,
            "mac_profile": mac_profile,
            "source_sha256": source_sha256,
            "duration_s": duration_s,
            "seed": int(seed_value),
            "flow_count": flow_count,
        }


def validate_app_admission_diagnostics(
    path: Path,
    scenario: str,
    application_profile: str,
    expected_flow_count: int,
) -> dict[str, int]:
    """Validate compact per-flow gate counters emitted by the runner."""

    required = {
        "schema",
        "scenario",
        "application_profile",
        "flow_index",
        "attempts",
        "admitted",
        "blocked_discovery",
        "blocked_topology",
        "blocked_gateway_route",
        "blocked_destination",
        "blocked_nsdp",
        "first_admitted_s",
        "last_admitted_s",
    }
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise WorkflowError(
                "application admission diagnostics has no header", stage="run_ns3"
            )
        if len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise WorkflowError(
                "application admission diagnostics has duplicate columns",
                stage="run_ns3",
            )
        missing = sorted(required - set(reader.fieldnames))
        if missing:
            raise WorkflowError(
                "application admission diagnostics is missing columns: "
                + ", ".join(missing),
                stage="run_ns3",
            )
        rows = list(reader)

    if len(rows) != expected_flow_count:
        raise WorkflowError(
            "application admission diagnostics flow count does not match the "
            f"scenario: expected {expected_flow_count}, found {len(rows)}",
            stage="run_ns3",
        )
    totals = {
        "attempts": 0,
        "admitted": 0,
        "blocked_discovery": 0,
        "blocked_topology": 0,
        "blocked_gateway_route": 0,
        "blocked_destination": 0,
        "blocked_nsdp": 0,
    }
    seen_indexes: set[int] = set()
    blocked_fields = tuple(key for key in totals if key.startswith("blocked_"))
    for row_number, row in enumerate(rows, start=2):
        if None in row or any(value is None for value in row.values()):
            raise WorkflowError(
                f"application admission diagnostics row {row_number} is malformed",
                stage="run_ns3",
            )
        if row["schema"].strip() != APP_ADMISSION_DIAGNOSTICS_SCHEMA:
            raise WorkflowError(
                f"application admission diagnostics row {row_number} has an "
                "unsupported schema",
                stage="run_ns3",
            )
        if row["scenario"].strip() != scenario:
            raise WorkflowError(
                f"application admission diagnostics row {row_number} has the "
                "wrong scenario",
                stage="run_ns3",
            )
        if row["application_profile"].strip() != application_profile:
            raise WorkflowError(
                f"application admission diagnostics row {row_number} has the "
                "wrong application profile",
                stage="run_ns3",
            )
        try:
            flow_index = int(row["flow_index"], 10)
            counts = {field: int(row[field], 10) for field in totals}
        except ValueError as error:
            raise WorkflowError(
                f"application admission diagnostics row {row_number} has a "
                "non-integer index or counter",
                stage="run_ns3",
            ) from error
        if flow_index < 0 or flow_index in seen_indexes:
            raise WorkflowError(
                f"application admission diagnostics row {row_number} has an "
                "invalid or duplicate flow_index",
                stage="run_ns3",
            )
        seen_indexes.add(flow_index)
        if any(value < 0 for value in counts.values()):
            raise WorkflowError(
                f"application admission diagnostics row {row_number} has a "
                "negative counter",
                stage="run_ns3",
            )
        if counts["attempts"] != counts["admitted"] + sum(
            counts[field] for field in blocked_fields
        ):
            raise WorkflowError(
                f"application admission diagnostics row {row_number} counters "
                "do not partition attempts",
                stage="run_ns3",
            )
        for time_field in ("first_admitted_s", "last_admitted_s"):
            raw_time = row[time_field].strip()
            if raw_time:
                try:
                    time_value = float(raw_time)
                except ValueError as error:
                    raise WorkflowError(
                        f"application admission diagnostics row {row_number} "
                        f"has nonnumeric {time_field}",
                        stage="run_ns3",
                    ) from error
                if not math.isfinite(time_value) or time_value < 0.0:
                    raise WorkflowError(
                        f"application admission diagnostics row {row_number} "
                        f"has invalid {time_field}",
                        stage="run_ns3",
                    )
        for field, value in counts.items():
            totals[field] += value

    if seen_indexes != set(range(expected_flow_count)):
        raise WorkflowError(
            "application admission diagnostics flow_index values are not "
            "contiguous from zero",
            stage="run_ns3",
        )
    return totals


def validate_vector_axes(extract: dict[str, Any]) -> dict[str, Any]:
    """Require one complete, identical bucket-end axis across every vector."""

    if extract.get("status") != "complete":
        raise WorkflowError(
            f"OPNET extraction is {extract.get('status', 'unknown')!r}, not complete",
            stage="extract_opnet",
            exit_code=3,
        )
    vectors = extract.get("vectors")
    if not isinstance(vectors, list) or not vectors:
        raise WorkflowError("OPNET extraction contains no vectors", stage="extract_opnet")

    reference: tuple[float, float, tuple[float, ...]] | None = None
    for index, vector in enumerate(vectors):
        if not isinstance(vector, dict) or vector.get("status") != "complete":
            raise WorkflowError(
                f"OPNET vector {index} is not complete", stage="extract_opnet"
            )
        axis = vector.get("time_axis")
        buckets = vector.get("buckets")
        if not isinstance(axis, dict) or not isinstance(buckets, list) or not buckets:
            raise WorkflowError(
                f"OPNET vector {index} has no decoded time axis",
                stage="extract_opnet",
            )
        try:
            start_s = float(axis["encoded_start_s"])
            width_s = float(axis["bucket_width_s"])
            times = tuple(float(bucket["time_s"]) for bucket in buckets)
        except (KeyError, TypeError, ValueError) as error:
            raise WorkflowError(
                f"OPNET vector {index} has a malformed time axis",
                stage="extract_opnet",
            ) from error
        if (
            not math.isfinite(start_s)
            or not math.isfinite(width_s)
            or width_s <= 0.0
            or any(not math.isfinite(time_s) for time_s in times)
        ):
            raise WorkflowError(
                f"OPNET vector {index} has a non-finite or nonpositive time axis",
                stage="extract_opnet",
            )
        for bucket_index, time_s in enumerate(times, start=1):
            expected = start_s + bucket_index * width_s
            if not math.isclose(time_s, expected, rel_tol=1.0e-12, abs_tol=1.0e-9):
                raise WorkflowError(
                    f"OPNET vector {index} bucket {bucket_index} is not on its "
                    "encoded regular axis",
                    stage="extract_opnet",
                )
        candidate = (start_s, width_s, times)
        if reference is None:
            reference = candidate
            continue
        same_scalars = math.isclose(
            candidate[0], reference[0], rel_tol=1.0e-12, abs_tol=1.0e-9
        ) and math.isclose(
            candidate[1], reference[1], rel_tol=1.0e-12, abs_tol=1.0e-9
        )
        same_times = len(candidate[2]) == len(reference[2]) and all(
            math.isclose(actual, expected, rel_tol=1.0e-12, abs_tol=1.0e-9)
            for actual, expected in zip(candidate[2], reference[2])
        )
        if not same_scalars or not same_times:
            statistic = vector.get("statistic", f"vector {index}")
            raise WorkflowError(
                f"OPNET vector axes are inconsistent at {statistic!r}",
                stage="extract_opnet",
            )

    assert reference is not None
    return {
        "encoded_start_s": reference[0],
        "bucket_width_s": reference[1],
        "bucket_count": len(reference[2]),
        "first_bucket_end_s": reference[2][0],
        "last_bucket_end_s": reference[2][-1],
        "bucket_ends_s": list(reference[2]),
    }


def _on_boundary(value: float, start_s: float, width_s: float) -> bool:
    quotient = (value - start_s) / width_s
    return quotient >= 0.0 and math.isclose(
        quotient, round(quotient), rel_tol=1.0e-12, abs_tol=1.0e-12
    )


def choose_window(
    axis: dict[str, Any],
    scenario_duration_s: float,
    requested_stop_s: float | None,
    requested_start_s: float | None,
) -> tuple[float, float]:
    """Resolve a closed comparison window aligned to recovered bucket ends."""

    start_axis = float(axis["encoded_start_s"])
    width_s = float(axis["bucket_width_s"])
    first_s = float(axis["first_bucket_end_s"])
    last_s = float(axis["last_bucket_end_s"])
    stop_s = scenario_duration_s if requested_stop_s is None else requested_stop_s
    if stop_s > scenario_duration_s and not math.isclose(
        stop_s, scenario_duration_s, rel_tol=1.0e-12, abs_tol=1.0e-9
    ):
        raise WorkflowError("--stop must not exceed the imported scenario duration")
    if stop_s > last_s and not math.isclose(
        stop_s, last_s, rel_tol=1.0e-12, abs_tol=1.0e-9
    ):
        raise WorkflowError(
            "chosen stop exceeds the last recovered OPNET bucket end"
        )
    if stop_s < first_s or not _on_boundary(stop_s, start_axis, width_s):
        raise WorkflowError(
            "chosen stop must be an exact recovered OPNET bucket boundary"
        )
    start_s = first_s if requested_start_s is None else requested_start_s
    if start_s < start_axis or not _on_boundary(start_s, start_axis, width_s):
        raise WorkflowError(
            "--time-start must be the encoded axis start or an exact bucket boundary"
        )
    if start_s > stop_s:
        raise WorkflowError("--time-start must not exceed the chosen stop")
    return start_s, stop_s


def _selected_statistics(
    requested: Iterable[str], extract: dict[str, Any]
) -> tuple[str, ...]:
    selected = tuple(dict.fromkeys(value.strip() for value in requested if value.strip()))
    if not selected:
        selected = DEFAULT_STATISTICS
    available = {
        vector.get("statistic")
        for vector in extract.get("vectors", [])
        if isinstance(vector, dict)
    }
    missing = [statistic for statistic in selected if statistic not in available]
    if missing:
        raise WorkflowError(
            "selected statistics are absent from the OPNET vectors: "
            + ", ".join(repr(value) for value in missing),
            stage="extract_opnet",
        )
    return selected


def _comparison_profile(
    selected: tuple[str, ...], arguments: argparse.Namespace
) -> tuple[str, list[str]]:
    """Classify semantic overrides without confusing a selected pass with parity."""

    overrides: list[str] = []
    if len(selected) != len(DEFAULT_STATISTICS) or set(selected) != set(
        DEFAULT_STATISTICS
    ):
        overrides.append("statistics_not_complete_core_set")
    if arguments.abs_tolerance != 0.0:
        overrides.append("nonzero_absolute_tolerance")
    if arguments.rel_tolerance != 0.0:
        overrides.append("nonzero_relative_tolerance")
    if arguments.time_tolerance != 0.0:
        overrides.append("nonzero_time_tolerance")
    if arguments.flow_limit != 0:
        overrides.append("flow_limit")
    if arguments.no_duty_cycling:
        overrides.append("duty_cycling_disabled")
    if arguments.no_gateway_discovery:
        overrides.append("gateway_discovery_disabled")
    if arguments.no_opnet_app_gating:
        overrides.append("opnet_app_gating_disabled")
    if arguments.legacy_trace_size_exclusion_bits != 0:
        overrides.append("legacy_trace_size_adjustment")
    return ("canonical" if not overrides else "diagnostic"), overrides


def _validate_probe_crosscheck(
    extract: dict[str, Any], scenario: str, selected: Iterable[str]
) -> None:
    definition = extract.get("probe_definition")
    if not isinstance(definition, dict):
        raise WorkflowError(
            "extractor did not return the requested probe definition",
            stage="extract_opnet",
        )
    if definition.get("scenario") != scenario:
        raise WorkflowError(
            "probe-definition scenario does not match the canonical scenario",
            stage="extract_opnet",
        )
    if definition.get("warnings"):
        raise WorkflowError(
            "probe-definition cross-check produced warnings",
            stage="extract_opnet",
        )
    wanted = set(selected)
    for vector in extract.get("vectors", []):
        if vector.get("statistic") not in wanted:
            continue
        match = vector.get("probe_definition_match")
        if not isinstance(match, dict) or match.get("status") not in {
            "exact",
            "name_and_aggregation_only",
        }:
            raise WorkflowError(
                f"probe definition did not corroborate {vector.get('statistic')!r}",
                stage="extract_opnet",
            )


def _opnet_source_identity(path: Path, has_probe_definition: bool) -> dict[str, Any]:
    """Derive, rather than assign, the scenario carried by a Modeler result name."""

    name = path.name
    if not name.lower().endswith(".ov"):
        raise WorkflowError("OPNET vector input must have a .ov filename")
    stem = name[:-3]
    match = re.fullmatch(r"(.+)-DES-(\d+)", stem)
    if match is not None:
        return {
            "scenario": match.group(1),
            "basis": "modeler_des_result_filename",
            "design_run": int(match.group(2)),
            "probe_definition_required": False,
        }
    if not has_probe_definition:
        raise WorkflowError(
            "OPNET vector filename does not use the authoritative "
            "<scenario>-DES-<run>.ov convention; --probe-definition is required"
        )
    if not stem:
        raise WorkflowError("OPNET vector filename has an empty scenario identity")
    return {
        "scenario": stem,
        "basis": "plain_ov_filename_corroborated_by_probe_definition",
        "design_run": None,
        "probe_definition_required": True,
    }


def _validate_extract_identity(
    extract: dict[str, Any],
    scenario_run: dict[str, Any],
    opnet_ov: Path,
    source_identity: dict[str, Any],
) -> dict[str, Any]:
    """Cross-check all scenario/run identities available in decoded evidence."""

    scenario = scenario_run["scenario"]
    if source_identity["scenario"] != scenario:
        raise WorkflowError(
            f"OPNET source filename identifies scenario "
            f"{source_identity['scenario']!r}, not canonical scenario {scenario!r}",
            stage="extract_opnet",
        )
    if extract.get("scenario") != scenario:
        raise WorkflowError(
            "extractor source-derived scenario does not match the canonical scenario",
            stage="extract_opnet",
        )

    provenance = extract.get("provenance")
    if not isinstance(provenance, dict):
        raise WorkflowError(
            "extractor manifest has no provenance object", stage="extract_opnet"
        )
    expected_digest = digest(opnet_ov)
    if (
        provenance.get("source_file") != opnet_ov.name
        or provenance.get("input_path") != str(opnet_ov.resolve())
        or provenance.get("source_file_sha256") != expected_digest
    ):
        raise WorkflowError(
            "extractor provenance does not identify the supplied OPNET vector",
            stage="extract_opnet",
        )

    execution = extract.get("execution")
    if not isinstance(execution, dict):
        raise WorkflowError(
            "extractor manifest has no execution identity", stage="extract_opnet"
        )
    try:
        duration_s = float(execution["duration_s"])
        seed_value = float(execution["seed"])
    except (KeyError, TypeError, ValueError) as error:
        raise WorkflowError(
            "OPNET execution duration/seed identity is absent or malformed",
            stage="extract_opnet",
        ) from error
    if not math.isfinite(duration_s) or duration_s != scenario_run["duration_s"]:
        raise WorkflowError(
            "OPNET execution duration does not exactly match the canonical scenario",
            stage="extract_opnet",
        )
    if (
        not math.isfinite(seed_value)
        or seed_value < 0.0
        or not seed_value.is_integer()
        or int(seed_value) != scenario_run["seed"]
    ):
        raise WorkflowError(
            "OPNET execution seed does not exactly match the canonical scenario",
            stage="extract_opnet",
        )
    return {
        "scenario": scenario,
        "duration_s": duration_s,
        "seed": int(seed_value),
        "source_file_sha256": expected_digest,
        "source_identity": source_identity,
    }


def _statistics_in_csv(path: Path, source: str, scenario: str) -> set[str]:
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"scenario", "statistic", "source"}
        if reader.fieldnames is None or not required.issubset(reader.fieldnames):
            raise WorkflowError(
                f"{path}: aggregate CSV lacks canonical identity columns",
                stage="aggregate_ns3",
            )
        statistics: set[str] = set()
        for row_number, row in enumerate(reader, start=2):
            if None in row or any(value is None for value in row.values()):
                raise WorkflowError(
                    f"{path}:{row_number}: malformed aggregate row",
                    stage="aggregate_ns3",
                )
            if row["source"].strip() != source or row["scenario"].strip() != scenario:
                raise WorkflowError(
                    f"{path}:{row_number}: unexpected source/scenario identity",
                    stage="aggregate_ns3",
                )
            statistics.add(row["statistic"].strip())
        return statistics


def _load_and_validate_ns3_provenance(
    path: Path, trace: Path, aggregates: Path, scenario: str
) -> dict[str, Any]:
    """Require exact delivery correlation and matching send/delivery sizes."""

    try:
        provenance = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise WorkflowError(
            f"cannot read ns-3 aggregate provenance: {error}",
            stage="aggregate_ns3",
        ) from error
    if not isinstance(provenance, dict):
        raise WorkflowError(
            "ns-3 aggregate provenance root is not an object",
            stage="aggregate_ns3",
        )
    if (
        provenance.get("schema") != NS3_PROVENANCE_SCHEMA
        or provenance.get("source") != "ns3"
        or provenance.get("scenario") != scenario
    ):
        raise WorkflowError(
            "ns-3 aggregate provenance has an unexpected schema/source/scenario",
            stage="aggregate_ns3",
        )

    input_record = provenance.get("input")
    output_record = provenance.get("output")
    if not isinstance(input_record, dict) or not isinstance(output_record, dict):
        raise WorkflowError(
            "ns-3 aggregate provenance lacks input/output records",
            stage="aggregate_ns3",
        )
    if (
        input_record.get("path") != str(trace.resolve())
        or input_record.get("sha256") != digest(trace)
        or input_record.get("size_bytes") != trace.stat().st_size
        or output_record.get("sha256") != digest(aggregates)
        or output_record.get("size_bytes") != aggregates.stat().st_size
    ):
        raise WorkflowError(
            "ns-3 aggregate provenance does not match generated artifacts",
            stage="aggregate_ns3",
        )

    delay_matching = provenance.get("delay_matching")
    if not isinstance(delay_matching, dict):
        raise WorkflowError(
            "ns-3 aggregate provenance lacks delay-matching evidence",
            stage="aggregate_ns3",
        )
    matched_count = delay_matching.get("matched_count")
    unmatched_delivery_count = delay_matching.get("unmatched_delivery_count")
    methods = delay_matching.get("matched_by_method")
    if (
        not isinstance(matched_count, int)
        or isinstance(matched_count, bool)
        or matched_count < 0
        or not isinstance(unmatched_delivery_count, int)
        or isinstance(unmatched_delivery_count, bool)
        or unmatched_delivery_count < 0
        or not isinstance(methods, dict)
        or any(
            not isinstance(count, int) or isinstance(count, bool) or count < 0
            for count in methods.values()
        )
    ):
        raise WorkflowError(
            "ns-3 delay-matching counts are malformed", stage="aggregate_ns3"
        )
    source_exact_method = "source_exact_dack_retry_duplicate"
    unsupported = {
        method: count
        for method, count in methods.items()
        if method not in {"exact_sequence", source_exact_method} and count != 0
    }
    exact_count = methods.get("exact_sequence", 0)
    duplicate_count = methods.get(source_exact_method, 0)
    if (
        unsupported
        or not isinstance(exact_count, int)
        or isinstance(exact_count, bool)
        or exact_count < 0
        or not isinstance(duplicate_count, int)
        or isinstance(duplicate_count, bool)
        or duplicate_count < 0
        or exact_count + duplicate_count != matched_count
    ):
        raise WorkflowError(
            "workflow-generated trace used unsupported delay matching",
            stage="aggregate_ns3",
        )
    duplicate_evidence = provenance.get("source_exact_dack_retry_duplicates")
    if not isinstance(duplicate_evidence, dict):
        raise WorkflowError(
            "ns-3 aggregate provenance lacks DACK-retry duplicate evidence",
            stage="aggregate_ns3",
        )
    proof_count = duplicate_evidence.get("repeated_feedback_proof_count")
    evidence_matched_count = duplicate_evidence.get(
        "matched_duplicate_delivery_count"
    )
    unused_proof_count = duplicate_evidence.get("unused_proof_count")
    lineages = duplicate_evidence.get("lineages")
    if (
        not isinstance(proof_count, int)
        or isinstance(proof_count, bool)
        or proof_count < 0
        or not isinstance(evidence_matched_count, int)
        or isinstance(evidence_matched_count, bool)
        or evidence_matched_count != duplicate_count
        or not isinstance(unused_proof_count, int)
        or isinstance(unused_proof_count, bool)
        or unused_proof_count < 0
        or proof_count != duplicate_count + unused_proof_count
        or not isinstance(lineages, list)
    ):
        raise WorkflowError(
            "ns-3 DACK-retry duplicate evidence is inconsistent",
            stage="aggregate_ns3",
        )
    lineage_budget = 0
    lineage_identities: set[tuple[str, ...]] = set()
    evidence_event_indexes: set[int] = set()
    identity_fields = (
        "src",
        "dst",
        "sequence",
        "relay",
        "ingress_peer",
        "hop_sequence",
    )
    for lineage in lineages:
        if not isinstance(lineage, dict):
            raise WorkflowError(
                "ns-3 DACK-retry duplicate lineage is malformed",
                stage="aggregate_ns3",
            )
        identity = tuple(lineage.get(field) for field in identity_fields)
        qualifying = lineage.get("qualifying_feedback_count")
        dack_count = lineage.get("dack_feedback_count")
        budget = lineage.get("proven_extra_delivery_budget")
        pairs = lineage.get("source_ordered_pairs")
        if (
            any(not isinstance(value, str) or not value.isdigit() for value in identity)
            or identity in lineage_identities
            or not isinstance(qualifying, int)
            or isinstance(qualifying, bool)
            or qualifying < 2
            or not isinstance(dack_count, int)
            or isinstance(dack_count, bool)
            or dack_count < 1
            or dack_count > qualifying
            or not isinstance(budget, int)
            or isinstance(budget, bool)
            or budget != qualifying - 1
            or dack_count not in {qualifying - 1, qualifying}
            or not isinstance(pairs, list)
            or len(pairs) != qualifying
        ):
            raise WorkflowError(
                "ns-3 DACK-retry duplicate lineage is malformed",
                stage="aggregate_ns3",
            )
        previous_feedback_index = -1
        for pair in pairs:
            if not isinstance(pair, dict):
                raise WorkflowError(
                    "ns-3 DACK-retry duplicate lineage pair is malformed",
                    stage="aggregate_ns3",
                )
            enqueue_index = pair.get("enqueue_event_index")
            feedback_index = pair.get("feedback_event_index")
            if (
                not isinstance(enqueue_index, int)
                or isinstance(enqueue_index, bool)
                or enqueue_index < 0
                or not isinstance(feedback_index, int)
                or isinstance(feedback_index, bool)
                or feedback_index <= enqueue_index
                or enqueue_index <= previous_feedback_index
                or enqueue_index in evidence_event_indexes
                or feedback_index in evidence_event_indexes
            ):
                raise WorkflowError(
                    "ns-3 DACK-retry duplicate lineage pair is malformed",
                    stage="aggregate_ns3",
                )
            evidence_event_indexes.add(enqueue_index)
            evidence_event_indexes.add(feedback_index)
            previous_feedback_index = feedback_index
        lineage_identities.add(identity)
        lineage_budget += budget
    if lineage_budget != proof_count:
        raise WorkflowError(
            "ns-3 DACK-retry duplicate lineage budget is inconsistent",
            stage="aggregate_ns3",
        )
    if unmatched_delivery_count != 0:
        raise WorkflowError(
            "workflow-generated trace contains unmatched deliveries",
            stage="aggregate_ns3",
        )
    window = provenance.get("window")
    event_counts = (
        window.get("in_window_event_counts")
        if isinstance(window, dict)
        else None
    )
    delivery_count = (
        event_counts.get("nwk_delivery", 0) if isinstance(event_counts, dict) else None
    )
    if (
        not isinstance(delivery_count, int)
        or isinstance(delivery_count, bool)
        or delivery_count != matched_count
    ):
        raise WorkflowError(
            "ns-3 delivery count is not fully accounted for by exact or "
            "source-exact DACK-retry matches",
            stage="aggregate_ns3",
        )
    size_integrity = provenance.get("matched_packet_size_integrity")
    if not isinstance(size_integrity, dict):
        raise WorkflowError(
            "ns-3 aggregate provenance lacks matched packet-size evidence",
            stage="aggregate_ns3",
        )
    size_compared_count = size_integrity.get("compared_count")
    size_mismatch_count = size_integrity.get("mismatch_count")
    if (
        not isinstance(size_compared_count, int)
        or isinstance(size_compared_count, bool)
        or size_compared_count != matched_count
        or not isinstance(size_mismatch_count, int)
        or isinstance(size_mismatch_count, bool)
        or size_mismatch_count != 0
    ):
        raise WorkflowError(
            "ns-3 matched app_send/nwk_delivery packet sizes are inconsistent",
            stage="aggregate_ns3",
        )
    return {
        "matched_count": matched_count,
        "matched_by_method": {
            method: count for method, count in sorted(methods.items()) if count
        },
        "source_exact_dack_retry_duplicate_proof_count": proof_count,
        "source_exact_dack_retry_duplicate_delivery_count": duplicate_count,
        "unmatched_delivery_count": unmatched_delivery_count,
        "matched_size_compared_count": size_compared_count,
        "matched_size_mismatch_count": size_mismatch_count,
    }


def run_stage(
    name: str,
    command: list[str],
    log_path: Path,
    stages: dict[str, Any],
) -> int:
    """Run one subprocess and retain merged stdout/stderr as evidence."""

    record = {
        "command": command,
        "exit_code": None,
        "log": log_path.name,
    }
    stages[name] = record
    try:
        with log_path.open("w", encoding="utf-8") as log:
            completed = subprocess.run(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                text=True,
                check=False,
            )
    except OSError as error:
        log_path.write_text(f"error: {error}\n", encoding="utf-8")
        record["launch_error"] = str(error)
        raise WorkflowError(str(error), stage=name) from error
    record["exit_code"] = completed.returncode
    return completed.returncode


def _artifact_records(output_dir: Path) -> dict[str, Any]:
    records: dict[str, Any] = {}
    for identity, filename in ARTIFACT_PATHS.items():
        path = output_dir / filename
        records[identity] = None if not path.is_file() else {
            "path": filename,
            "sha256": digest(path),
            "size_bytes": path.stat().st_size,
        }
    return records


def _write_manifest(output_dir: Path, manifest: dict[str, Any]) -> None:
    manifest["artifacts"] = _artifact_records(output_dir)
    (output_dir / MANIFEST_NAME).write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def _managed_filenames() -> tuple[str, ...]:
    return (*ARTIFACT_PATHS.values(), MANIFEST_NAME)


def _check_output_collisions(
    output_dir: Path, source_paths: dict[str, Path]
) -> None:
    """Reject an input/tool that would be replaced as a workflow artifact."""

    targets = {
        (output_dir / filename).resolve(): filename
        for filename in _managed_filenames()
    }
    for label, path in source_paths.items():
        target_name = targets.get(path.resolve())
        if target_name is not None:
            raise WorkflowError(
                f"{label} input collides with managed output target {target_name!r}"
            )


def _classify_output_directory(output_dir: Path) -> str:
    """Return new/empty/owned; reject every ambiguous destructive target."""

    if output_dir.is_symlink():
        raise WorkflowError("--output-dir must not be a symbolic link")
    if not output_dir.exists():
        return "new"
    if not output_dir.is_dir():
        raise WorkflowError("--output-dir exists but is not a directory")
    entries = list(output_dir.iterdir())
    if not entries:
        return "empty"

    marker = output_dir / MANIFEST_NAME
    if marker.is_symlink() or not marker.is_file():
        raise WorkflowError(
            "nonempty output directory is not workflow-owned; expected a regular "
            f"{MANIFEST_NAME} ownership manifest"
        )
    try:
        previous = json.loads(marker.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise WorkflowError(
            f"cannot validate output-directory ownership manifest: {error}"
        ) from error
    if not isinstance(previous, dict) or previous.get("schema") != RUN_SCHEMA:
        raise WorkflowError(
            "nonempty output directory has no valid workflow ownership manifest"
        )
    for filename in _managed_filenames():
        target = output_dir / filename
        if target.exists() and not target.is_file() and not target.is_symlink():
            raise WorkflowError(
                f"managed output target {filename!r} exists but is not a file"
            )
    return "owned"


def _clear_managed_artifacts(output_dir: Path) -> None:
    """Prevent a failed rerun from presenting artifacts left by an older run."""

    for filename in _managed_filenames():
        path = output_dir / filename
        if path.is_file() or path.is_symlink():
            path.unlink()


def _prepare_output_directory(output_dir: Path, classification: str) -> None:
    if classification == "new":
        output_dir.mkdir(parents=True, exist_ok=False)
    elif classification == "owned":
        _clear_managed_artifacts(output_dir)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", required=True, type=Path)
    parser.add_argument("--runner", required=True, type=Path)
    parser.add_argument("--opnet-ov", required=True, type=Path)
    parser.add_argument(
        "--probe-definition",
        required=True,
        type=Path,
        help="required paired Modeler .pb.m identity/label corroboration",
    )
    parser.add_argument(
        "--expected-opnet-ov-sha256",
        required=True,
        type=_sha256_argument,
        help="required trusted SHA-256 of --opnet-ov",
    )
    parser.add_argument(
        "--expected-probe-definition-sha256",
        required=True,
        type=_sha256_argument,
        help="required trusted SHA-256 of --probe-definition",
    )
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument(
        "--stop",
        type=_finite_positive,
        help="must equal the canonical full duration (default: full duration)",
    )
    parser.add_argument(
        "--time-start",
        type=_finite_nonnegative,
        help="must equal the first recovered bucket end (default: that bucket)",
    )
    parser.add_argument("--flow-limit", type=int, default=0)
    parser.add_argument("--no-duty-cycling", action="store_true")
    parser.add_argument("--no-gateway-discovery", action="store_true")
    parser.add_argument("--no-opnet-app-gating", action="store_true")
    parser.add_argument(
        "--statistic",
        action="append",
        default=[],
        help="exact source-equivalent statistic (repeatable; defaults to core set)",
    )
    parser.add_argument(
        "--abs-tolerance", type=_finite_nonnegative, default=0.0
    )
    parser.add_argument(
        "--rel-tolerance", type=_finite_nonnegative, default=0.0
    )
    parser.add_argument(
        "--time-tolerance", type=_finite_nonnegative, default=0.0
    )
    parser.add_argument("--max-differences", type=int, default=200)
    parser.add_argument(
        "--legacy-trace-size-exclusion-bits",
        "--opnet-size-exclusion-bits",
        dest="legacy_trace_size_exclusion_bits",
        type=int,
        default=0,
        help=(
            "compatibility-only bits subtracted from legacy ns-3 trace packet "
            "sizes (default: 0; --opnet-size-exclusion-bits is a retained alias)"
        ),
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    arguments = build_parser().parse_args(argv)
    stages: dict[str, Any] = {}
    output_ready = False
    manifest: dict[str, Any] = {
        "schema": RUN_SCHEMA,
        "status": "running",
        "evidence_scope": EVIDENCE_SCOPE,
        "profile": "pending",
        "noncertifying_overrides": [],
        "failure": None,
        "inputs": {},
        "tools": {},
        "configuration": {},
        "stages": stages,
        "artifacts": {},
    }

    extractor = Path(__file__).resolve().with_name("extract-opnet-ov.py")
    aggregator = Path(__file__).resolve().with_name("aggregate-ns3-trace.py")
    comparator = Path(__file__).resolve().with_name(
        "compare-opnet-ns3-aggregates.py"
    )
    try:
        if arguments.flow_limit < 0:
            raise WorkflowError("--flow-limit must be nonnegative")
        if arguments.max_differences <= 0:
            raise WorkflowError("--max-differences must be positive")
        if arguments.legacy_trace_size_exclusion_bits < 0:
            raise WorkflowError(
                "--legacy-trace-size-exclusion-bits must be nonnegative"
            )

        inputs = {
            "scenario": arguments.scenario,
            "runner": arguments.runner,
            "opnet_ov": arguments.opnet_ov,
            "probe_definition": arguments.probe_definition,
        }
        for label, path in inputs.items():
            if not path.is_file():
                raise WorkflowError(f"{label} does not exist: {path}")
        if not os.access(arguments.runner, os.X_OK):
            raise WorkflowError(f"runner is not executable: {arguments.runner}")
        tools = {
            "extractor": extractor,
            "aggregator": aggregator,
            "comparator": comparator,
        }
        for label, path in tools.items():
            if not path.is_file():
                raise WorkflowError(f"{label} utility does not exist: {path}")
            manifest["tools"][label] = file_record(path)
        manifest["inputs"] = {
            label: file_record(path) for label, path in inputs.items()
        }
        actual_ov_sha256 = manifest["inputs"]["opnet_ov"]["sha256"]
        actual_probe_sha256 = manifest["inputs"]["probe_definition"]["sha256"]
        manifest["evidence_identity"] = {
            "opnet_ov": {
                "expected_sha256": arguments.expected_opnet_ov_sha256,
                "actual_sha256": actual_ov_sha256,
                "match": actual_ov_sha256 == arguments.expected_opnet_ov_sha256,
            },
            "probe_definition": {
                "expected_sha256": arguments.expected_probe_definition_sha256,
                "actual_sha256": actual_probe_sha256,
                "match": (
                    actual_probe_sha256
                    == arguments.expected_probe_definition_sha256
                ),
            },
        }
        if actual_ov_sha256 != arguments.expected_opnet_ov_sha256:
            raise WorkflowError(
                "OPNET vector SHA-256 mismatch: expected "
                f"{arguments.expected_opnet_ov_sha256}, actual {actual_ov_sha256}"
            )
        if actual_probe_sha256 != arguments.expected_probe_definition_sha256:
            raise WorkflowError(
                "probe-definition SHA-256 mismatch: expected "
                f"{arguments.expected_probe_definition_sha256}, actual "
                f"{actual_probe_sha256}"
            )

        scenario_run = load_scenario_run(arguments.scenario)
        if (
            arguments.stop is not None
            and arguments.stop != scenario_run["duration_s"]
        ):
            raise WorkflowError(
                "--stop must exactly equal the canonical scenario duration; "
                "authoritative prefix runs are not supported"
            )
        source_identity = _opnet_source_identity(
            arguments.opnet_ov, True
        )
        if source_identity["scenario"] != scenario_run["scenario"]:
            raise WorkflowError(
                f"OPNET source filename identifies scenario "
                f"{source_identity['scenario']!r}, not canonical scenario "
                f"{scenario_run['scenario']!r}"
            )

        _check_output_collisions(arguments.output_dir, {**inputs, **tools})
        output_classification = _classify_output_directory(arguments.output_dir)
        _prepare_output_directory(arguments.output_dir, output_classification)
        output_ready = True
        _write_manifest(arguments.output_dir, manifest)

        opnet_extract_json = arguments.output_dir / ARTIFACT_PATHS[
            "opnet_extract_manifest"
        ]
        opnet_aggregates = arguments.output_dir / ARTIFACT_PATHS["opnet_aggregates"]
        extract_command = [
            sys.executable,
            str(extractor),
            str(arguments.opnet_ov),
            "--json",
            str(opnet_extract_json),
            "--csv",
            str(opnet_aggregates),
            "--strict",
        ]
        extract_command.extend(
            ("--probe-definition", str(arguments.probe_definition))
        )
        extract_code = run_stage(
            "extract_opnet",
            extract_command,
            arguments.output_dir / ARTIFACT_PATHS["opnet_extract_log"],
            stages,
        )
        if extract_code != 0:
            raise WorkflowError(
                f"OPNET extractor failed with status {extract_code}",
                stage="extract_opnet",
                exit_code=extract_code,
            )
        try:
            extract = json.loads(opnet_extract_json.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as error:
            raise WorkflowError(
                f"cannot read extractor manifest: {error}", stage="extract_opnet"
            ) from error
        if not isinstance(extract, dict):
            raise WorkflowError(
                "extractor manifest root is not an object", stage="extract_opnet"
            )
        opnet_identity = _validate_extract_identity(
            extract, scenario_run, arguments.opnet_ov, source_identity
        )
        axis = validate_vector_axes(extract)
        selected = _selected_statistics(arguments.statistic, extract)
        profile, noncertifying_overrides = _comparison_profile(selected, arguments)
        if scenario_run["application_profile"] == "unspecified":
            profile = "diagnostic"
            noncertifying_overrides.append("application_profile_unspecified")
        if scenario_run["mac_profile"] == "unspecified":
            profile = "diagnostic"
            noncertifying_overrides.append("mac_profile_unspecified")
        manifest["profile"] = profile
        manifest["noncertifying_overrides"] = noncertifying_overrides
        _validate_probe_crosscheck(extract, scenario_run["scenario"], selected)
        if (
            arguments.time_start is not None
            and arguments.time_start != float(axis["first_bucket_end_s"])
        ):
            raise WorkflowError(
                "--time-start must equal the first recovered bucket end; "
                "authoritative suffix comparisons are not supported"
            )
        time_start_s, stop_s = choose_window(
            axis,
            scenario_run["duration_s"],
            arguments.stop,
            arguments.time_start,
        )
        manifest["configuration"] = {
            "scenario": scenario_run["scenario"],
            "scenario_duration_s": scenario_run["duration_s"],
            "identities": {
                "canonical_scenario": {
                    "scenario": scenario_run["scenario"],
                    "application_profile": scenario_run["application_profile"],
                    "mac_profile": scenario_run["mac_profile"],
                    "duration_s": scenario_run["duration_s"],
                    "seed": scenario_run["seed"],
                    "source_sha256": scenario_run["source_sha256"],
                },
                "opnet_execution": opnet_identity,
            },
            "stop_s": stop_s,
            "comparison_time_start_s": time_start_s,
            "comparison_time_stop_s": stop_s,
            "opnet_axis": {
                key: value for key, value in axis.items() if key != "bucket_ends_s"
            },
            "statistics": list(selected),
            "tolerances": {
                "absolute": arguments.abs_tolerance,
                "relative": arguments.rel_tolerance,
                "time_s": arguments.time_tolerance,
            },
            "runner": {
                "flow_limit": arguments.flow_limit,
                "duty_cycling": not arguments.no_duty_cycling,
                "opnet_aligned_duty_cycle": True,
                "gateway_discovery": not arguments.no_gateway_discovery,
                "opnet_app_gating": not arguments.no_opnet_app_gating,
                "application_profile": scenario_run["application_profile"],
                "mac_profile": scenario_run["mac_profile"],
                "aggregate_trace_only": True,
                "quiet_model_logs": True,
                "app_admission_diagnostics_schema": (
                    APP_ADMISSION_DIAGNOSTICS_SCHEMA
                ),
            },
            "legacy_trace_size_exclusion_bits": (
                arguments.legacy_trace_size_exclusion_bits
            ),
            "require_zero_matched_packet_size_mismatches": True,
        }

        ns3_trace = arguments.output_dir / ARTIFACT_PATHS["ns3_trace"]
        app_diagnostics = arguments.output_dir / ARTIFACT_PATHS[
            "app_admission_diagnostics"
        ]
        runner_command = [
            str(arguments.runner.resolve()),
            f"--scenario={arguments.scenario}",
            f"--trace={ns3_trace}",
            f"--appDiagnostics={app_diagnostics}",
            f"--stop={_number(stop_s)}",
            f"--flowLimit={arguments.flow_limit}",
            f"--dutyCycling={0 if arguments.no_duty_cycling else 1}",
            "--opnetAlignedDutyCycle=1",
            f"--gatewayDiscovery={0 if arguments.no_gateway_discovery else 1}",
            f"--opnetAppGating={0 if arguments.no_opnet_app_gating else 1}",
            "--aggregateTraceOnly=1",
            "--quietModelLogs=1",
        ]
        runner_code = run_stage(
            "run_ns3",
            runner_command,
            arguments.output_dir / ARTIFACT_PATHS["ns3_run_log"],
            stages,
        )
        if runner_code != 0:
            raise WorkflowError(
                f"ns-3 runner failed with status {runner_code}",
                stage="run_ns3",
                exit_code=runner_code,
            )
        if not ns3_trace.is_file():
            raise WorkflowError("ns-3 runner produced no trace", stage="run_ns3")
        if not app_diagnostics.is_file():
            raise WorkflowError(
                "ns-3 runner produced no application admission diagnostics",
                stage="run_ns3",
            )
        stages["run_ns3"]["application_admission_totals"] = (
            validate_app_admission_diagnostics(
                app_diagnostics,
                scenario_run["scenario"],
                scenario_run["application_profile"],
                scenario_run["flow_count"],
            )
        )

        ns3_aggregates = arguments.output_dir / ARTIFACT_PATHS["ns3_aggregates"]
        ns3_provenance = arguments.output_dir / ARTIFACT_PATHS[
            "ns3_aggregate_provenance"
        ]
        aggregate_command = [
            sys.executable,
            str(aggregator),
            str(ns3_trace),
            str(ns3_aggregates),
            "--scenario",
            scenario_run["scenario"],
            "--bucket-width",
            _number(float(axis["bucket_width_s"])),
            "--stop-time",
            _number(stop_s),
            "--legacy-trace-size-exclusion-bits",
            str(arguments.legacy_trace_size_exclusion_bits),
            "--require-zero-size-mismatches",
            "--provenance",
            str(ns3_provenance),
        ]
        aggregate_code = run_stage(
            "aggregate_ns3",
            aggregate_command,
            arguments.output_dir / ARTIFACT_PATHS["ns3_aggregate_log"],
            stages,
        )
        if aggregate_code != 0:
            raise WorkflowError(
                f"ns-3 aggregate derivation failed with status {aggregate_code}",
                stage="aggregate_ns3",
                exit_code=aggregate_code,
            )
        exact_matching = _load_and_validate_ns3_provenance(
            ns3_provenance,
            ns3_trace,
            ns3_aggregates,
            scenario_run["scenario"],
        )
        stages["aggregate_ns3"]["exact_sequence_validation"] = exact_matching
        available_ns3 = _statistics_in_csv(
            ns3_aggregates, "ns3", scenario_run["scenario"]
        )
        missing_ns3 = [value for value in selected if value not in available_ns3]
        if missing_ns3:
            raise WorkflowError(
                "selected statistics are absent from ns-3 aggregates: "
                + ", ".join(repr(value) for value in missing_ns3),
                stage="aggregate_ns3",
            )

        report = arguments.output_dir / ARTIFACT_PATHS["comparison_report"]
        normalized = arguments.output_dir / ARTIFACT_PATHS[
            "normalized_aggregates"
        ]
        compare_command = [
            sys.executable,
            str(comparator),
            str(opnet_aggregates),
            str(ns3_aggregates),
            "--scenario",
            scenario_run["scenario"],
            "--time-start",
            _number(time_start_s),
            "--time-stop",
            _number(stop_s),
            "--abs-tolerance",
            _number(arguments.abs_tolerance),
            "--rel-tolerance",
            _number(arguments.rel_tolerance),
            "--time-tolerance",
            _number(arguments.time_tolerance),
            "--max-differences",
            str(arguments.max_differences),
            "--report",
            str(report),
            "--normalized",
            str(normalized),
        ]
        for statistic in selected:
            compare_command.extend(("--statistic", statistic))
        compare_code = run_stage(
            "compare",
            compare_command,
            arguments.output_dir / ARTIFACT_PATHS["comparison_log"],
            stages,
        )
        manifest["status"] = (
            "selected_comparison_passed"
            if compare_code == 0
            else "selected_comparison_failed"
        )
        if compare_code != 0:
            manifest["failure"] = {
                "stage": "compare",
                "message": f"aggregate comparator returned status {compare_code}",
                "exit_code": compare_code,
            }
        if output_ready:
            _write_manifest(arguments.output_dir, manifest)
        return compare_code
    except WorkflowError as error:
        manifest["status"] = "failed"
        manifest["failure"] = {
            "stage": error.stage,
            "message": str(error),
            "exit_code": error.exit_code,
        }
        if output_ready:
            _write_manifest(arguments.output_dir, manifest)
        print(f"error: {error}", file=sys.stderr)
        return error.exit_code
    except OSError as error:
        manifest["status"] = "failed"
        manifest["failure"] = {
            "stage": "workflow",
            "message": str(error),
            "exit_code": 2,
        }
        if output_ready:
            _write_manifest(arguments.output_dir, manifest)
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
