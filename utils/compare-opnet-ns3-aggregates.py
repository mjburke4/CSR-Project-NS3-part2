#!/usr/bin/env python3
"""Compare canonical OPNET and ns-3 aggregate time-series CSV rows.

The canonical input schema is ``csr-aggregate-series-v1`` with these core
columns::

    schema,scenario,statistic,time_s,value,source

``source`` must identify either ``opnet`` or ``ns3``.  Multiple input files
may be supplied and extra provenance columns are preserved when recognized.
Rows do not have to be ordered.  Series identity is the exact
``(scenario, statistic, unit, aggregation)`` tuple, and point identity adds a
simulation time aligned within the configured time tolerance.  The optional
``unit`` and ``aggregation`` columns are intentionally part of identity: two
series with different units or aggregation semantics are reported as
noncomparable and are never subjected to a numeric comparison.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Iterable


AGGREGATE_SCHEMA = "csr-aggregate-series-v1"
REPORT_SCHEMA = "csr-aggregate-differential-report-v1"
CORE_COLUMNS = (
    "schema",
    "scenario",
    "statistic",
    "time_s",
    "value",
    "source",
)
SEMANTIC_COLUMNS = ("unit", "aggregation")
VALUE_METADATA_COLUMNS = ("raw_value", "value_status")
PROVENANCE_COLUMNS = ("source_file", "source_file_sha256")
OUTPUT_COLUMNS = (
    CORE_COLUMNS + SEMANTIC_COLUMNS + VALUE_METADATA_COLUMNS + PROVENANCE_COLUMNS
)
ALIASES = {
    "schema": ("schema",),
    "scenario": ("scenario", "scenario_name"),
    "statistic": ("statistic", "statistic_name", "metric", "vector"),
    "time_s": ("time_s", "time", "simulation_time", "simulation_time_s"),
    "value": ("value", "statistic_value", "metric_value"),
    "source": ("source", "simulator", "origin"),
    "unit": ("unit", "units"),
    "aggregation": (
        "aggregation",
        "aggregation_semantics",
        "aggregation_method",
    ),
    "raw_value": ("raw_value",),
    "value_status": ("value_status", "status"),
    "source_file": ("source_file",),
    "source_file_sha256": ("source_file_sha256",),
}
SOURCE_ALIASES = {
    "opnet": "opnet",
    "modeler": "opnet",
    "riverbed_modeler": "opnet",
    "ns3": "ns3",
    "ns_3": "ns3",
}
MAX_FINITE_FLOAT = sys.float_info.max


class AggregateError(ValueError):
    """An aggregate input cannot be normalized or aligned safely."""


@dataclass(frozen=True)
class Point:
    """One normalized aggregate sample and its input provenance."""

    scenario: str
    statistic: str
    time_s: float
    value: float | None
    source: str
    input_path: Path
    input_row: int
    unit: str = ""
    aggregation: str = ""
    raw_value: str = ""
    value_status: str = "observed"
    source_file: str = ""
    source_file_sha256: str = ""


def _normalized_header(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.strip().lower()).strip("_")


def _clean_text(value: str | None) -> str:
    return "" if value is None else value.strip()


def _finite_number(path: Path, row: int, field: str, value: str) -> float:
    try:
        number = float(value)
    except ValueError as error:
        raise AggregateError(f"{path}:{row}: {field} is not numeric: {value!r}") from error
    if not math.isfinite(number):
        raise AggregateError(f"{path}:{row}: {field} must be finite: {value!r}")
    return number


def _canonical_source(path: Path, row: int, value: str) -> str:
    normalized = _normalized_header(value)
    source = SOURCE_ALIASES.get(normalized)
    if source is None:
        raise AggregateError(
            f"{path}:{row}: source must be 'opnet' or 'ns3', got {value!r}"
        )
    return source


def read_aggregates(paths: Iterable[Path]) -> list[Point]:
    """Read, normalize, and validate one or more aggregate CSV files."""
    points: list[Point] = []
    identities: dict[
        tuple[str, str, str, str, str, float], tuple[Path, int]
    ] = {}
    for path in paths:
        try:
            stream = path.open("r", encoding="utf-8-sig", newline="")
        except OSError as error:
            raise AggregateError(str(error)) from error
        with stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames is None:
                raise AggregateError(f"{path}: aggregate CSV has no header")
            normalized_headers = [
                _normalized_header(name) for name in reader.fieldnames
            ]
            if "" in normalized_headers:
                raise AggregateError(f"{path}: aggregate CSV has an empty column name")
            if len(set(normalized_headers)) != len(normalized_headers):
                raise AggregateError(
                    f"{path}: aggregate CSV has duplicate normalized columns"
                )
            source_headers = {
                _normalized_header(name): name for name in reader.fieldnames
            }
            column_map: dict[str, str] = {}
            for canonical, aliases in ALIASES.items():
                for alias in aliases:
                    source_header = source_headers.get(_normalized_header(alias))
                    if source_header is not None:
                        column_map[canonical] = source_header
                        break
            missing = [
                field
                for field in CORE_COLUMNS
                if field not in column_map
            ]
            if missing:
                raise AggregateError(
                    f"{path}: missing required columns: {', '.join(missing)}"
                )

            for row_number, row in enumerate(reader, start=2):
                if None in row:
                    raise AggregateError(
                        f"{path}:{row_number}: contains fields beyond the CSV header"
                    )
                if any(value is None for value in row.values()):
                    raise AggregateError(
                        f"{path}:{row_number}: has fewer fields than the CSV header"
                    )

                def get(field: str) -> str:
                    header = column_map.get(field)
                    return "" if header is None else _clean_text(row.get(header))

                schema = get("schema")
                if schema != AGGREGATE_SCHEMA:
                    raise AggregateError(
                        f"{path}:{row_number}: unsupported schema {schema!r}"
                    )
                scenario = get("scenario")
                statistic = get("statistic")
                if not scenario:
                    raise AggregateError(f"{path}:{row_number}: scenario is empty")
                if not statistic:
                    raise AggregateError(f"{path}:{row_number}: statistic is empty")
                time_s = _finite_number(path, row_number, "time_s", get("time_s"))
                if time_s < 0.0:
                    raise AggregateError(
                        f"{path}:{row_number}: time_s must be nonnegative"
                    )
                raw_value = get("value")
                value_status = _normalized_header(get("value_status"))
                if raw_value:
                    if value_status not in {"", "observed"}:
                        raise AggregateError(
                            f"{path}:{row_number}: numeric value has incompatible "
                            f"value_status {value_status!r}"
                        )
                    value: float | None = _finite_number(
                        path, row_number, "value", raw_value
                    )
                    value_status = "observed"
                else:
                    if value_status not in {"missing", "no_sample"}:
                        raise AggregateError(
                            f"{path}:{row_number}: value is empty without "
                            "value_status=missing"
                        )
                    value = None
                    value_status = "missing"
                source = _canonical_source(path, row_number, get("source"))
                unit = get("unit")
                aggregation = get("aggregation")
                identity = (
                    source,
                    scenario,
                    statistic,
                    unit,
                    aggregation,
                    time_s,
                )
                prior = identities.get(identity)
                if prior is not None:
                    raise AggregateError(
                        f"{path}:{row_number}: duplicate point identity also present at "
                        f"{prior[0]}:{prior[1]}: {identity!r}"
                    )
                identities[identity] = (path, row_number)
                points.append(
                    Point(
                        scenario=scenario,
                        statistic=statistic,
                        time_s=time_s,
                        value=value,
                        source=source,
                        input_path=path,
                        input_row=row_number,
                        unit=unit,
                        aggregation=aggregation,
                        raw_value=get("raw_value"),
                        value_status=value_status,
                        source_file=get("source_file"),
                        source_file_sha256=get("source_file_sha256"),
                    )
                )
    if not points:
        raise AggregateError("aggregate input contains no data rows")
    return points


def _point_sort_key(point: Point) -> tuple[str, str, str, str, float, int]:
    return (
        point.scenario,
        point.statistic,
        point.unit,
        point.aggregation,
        point.time_s,
        0 if point.source == "opnet" else 1,
    )


def write_normalized(path: Path, points: Iterable[Point]) -> None:
    """Write canonical, deterministically ordered aggregate rows."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for point in sorted(points, key=_point_sort_key):
            writer.writerow(
                {
                    "schema": AGGREGATE_SCHEMA,
                    "scenario": point.scenario,
                    "statistic": point.statistic,
                    "time_s": repr(point.time_s),
                    "value": "" if point.value is None else repr(point.value),
                    "source": point.source,
                    "unit": point.unit,
                    "aggregation": point.aggregation,
                    "raw_value": point.raw_value,
                    "value_status": point.value_status,
                    "source_file": point.source_file,
                    "source_file_sha256": point.source_file_sha256,
                }
            )


def filter_points(
    points: Iterable[Point],
    scenarios: set[str],
    statistics: set[str],
    units: set[str] | None = None,
    aggregations: set[str] | None = None,
    time_start_s: float | None = None,
    time_stop_s: float | None = None,
) -> list[Point]:
    """Apply exact series selectors without changing values."""
    units = set() if units is None else units
    aggregations = set() if aggregations is None else aggregations
    return [
        point
        for point in points
        if (not scenarios or point.scenario in scenarios)
        and (not statistics or point.statistic in statistics)
        and (not units or point.unit in units)
        and (not aggregations or point.aggregation in aggregations)
        and (time_start_s is None or point.time_s >= time_start_s)
        and (time_stop_s is None or point.time_s <= time_stop_s)
    ]


def _better_score(
    candidate: tuple[int, float, int], current: tuple[int, float, int]
) -> bool:
    """Prefer more matches, then less time error, then deterministic action."""
    return (candidate[0], -candidate[1], candidate[2]) > (
        current[0],
        -current[1],
        current[2],
    )


def align_series(
    opnet: list[Point], ns3: list[Point], time_tolerance_s: float
) -> tuple[list[tuple[Point, Point]], list[Point], list[Point]]:
    """Find a maximum-cardinality, minimum-time-error monotonic alignment."""
    opnet = sorted(opnet, key=lambda point: point.time_s)
    ns3 = sorted(ns3, key=lambda point: point.time_s)
    n = len(opnet)
    m = len(ns3)
    matches = [[0] * (m + 1) for _ in range(n + 1)]
    costs = [[0.0] * (m + 1) for _ in range(n + 1)]
    actions = [[""] * (m + 1) for _ in range(n + 1)]

    for i in range(n, -1, -1):
        for j in range(m, -1, -1):
            if i == n and j == m:
                continue
            choices: list[tuple[tuple[int, float, int], str]] = []
            if i < n:
                choices.append(((matches[i + 1][j], costs[i + 1][j], 0), "opnet"))
            if j < m:
                choices.append(((matches[i][j + 1], costs[i][j + 1], 1), "ns3"))
            if i < n and j < m:
                delta = abs(opnet[i].time_s - ns3[j].time_s)
                if delta <= time_tolerance_s:
                    choices.append(
                        (
                            (
                                1 + matches[i + 1][j + 1],
                                delta + costs[i + 1][j + 1],
                                2,
                            ),
                            "match",
                        )
                    )
            best_score, best_action = choices[0]
            for score, action in choices[1:]:
                if _better_score(score, best_score):
                    best_score, best_action = score, action
            matches[i][j] = best_score[0]
            costs[i][j] = best_score[1]
            actions[i][j] = best_action

    aligned: list[tuple[Point, Point]] = []
    missing: list[Point] = []
    extra: list[Point] = []
    i = 0
    j = 0
    while i < n or j < m:
        action = actions[i][j]
        if action == "match":
            aligned.append((opnet[i], ns3[j]))
            i += 1
            j += 1
        elif action == "opnet":
            missing.append(opnet[i])
            i += 1
        elif action == "ns3":
            extra.append(ns3[j])
            j += 1
        else:  # pragma: no cover - protects against an internal DP regression.
            raise AssertionError(f"invalid alignment action at {i},{j}: {action!r}")
    return aligned, missing, extra


def _value_deltas(expected: float, actual: float) -> tuple[float, float, bool]:
    """Return finite absolute/relative deltas and whether absolute was capped."""
    scale = max(abs(expected), abs(actual))
    if scale == 0.0:
        return 0.0, 0.0, False

    # Subtraction cannot overflow when both operands have the same sign and
    # retains more precision for nearby values.  For opposite signs, normalize
    # first so that the comparison remains finite even near +/-DBL_MAX.
    if (expected >= 0.0) == (actual >= 0.0):
        absolute_delta = abs(expected - actual)
        return absolute_delta, absolute_delta / scale, False

    relative_delta = abs(expected / scale - actual / scale)
    if relative_delta > MAX_FINITE_FLOAT / scale:
        return MAX_FINITE_FLOAT, relative_delta, True
    return relative_delta * scale, relative_delta, False


def _saturating_product(left: float, right: float) -> tuple[float, bool]:
    """Multiply nonnegative finite values without returning infinity."""
    if left == 0.0 or right == 0.0:
        return 0.0, False
    if left > MAX_FINITE_FLOAT / right:
        return MAX_FINITE_FLOAT, True
    return left * right, False


def _point_location(point: Point) -> dict[str, object]:
    location: dict[str, object] = {
        "input": str(point.input_path),
        "row": point.input_row,
    }
    if point.source_file:
        location["source_file"] = point.source_file
    if point.source_file_sha256:
        location["source_file_sha256"] = point.source_file_sha256
    return location


def compare(
    points: Iterable[Point],
    absolute_tolerance: float,
    relative_tolerance: float,
    time_tolerance_s: float,
    max_differences: int,
) -> dict[str, object]:
    """Align semantically identical series and return a comparison report."""
    point_list = list(points)
    grouped: dict[tuple[str, str, str, str, str], list[Point]] = {}
    for point in point_list:
        grouped.setdefault(
            (
                point.scenario,
                point.statistic,
                point.unit,
                point.aggregation,
                point.source,
            ),
            [],
        ).append(point)

    base_keys = sorted({(key[0], key[1]) for key in grouped})
    differences: list[dict[str, object]] = []
    skipped: list[dict[str, object]] = []
    series_reports: list[dict[str, object]] = []
    total_differences = 0
    total_skipped = 0
    counts = {
        "base_series": len(base_keys),
        "comparable_series": 0,
        "unmatched_opnet_series": 0,
        "unmatched_ns3_series": 0,
        "noncomparable_series": 0,
        "matched_points": 0,
        "numeric_points_compared": 0,
        "missing_in_ns3": 0,
        "extra_in_ns3": 0,
        "numeric_mismatches": 0,
        "missing_ns3_values": 0,
        "indeterminate_series": 0,
        "skipped_missing_values": 0,
        "skipped_noncomparable_points": 0,
    }

    def append_difference(difference: dict[str, object]) -> None:
        nonlocal total_differences
        total_differences += 1
        if len(differences) < max_differences:
            differences.append(difference)

    def append_skipped(item: dict[str, object]) -> None:
        nonlocal total_skipped
        total_skipped += 1
        if len(skipped) < max_differences:
            skipped.append(item)

    for scenario, statistic in base_keys:
        opnet_semantics = {
            (key[2], key[3])
            for key in grouped
            if key[0] == scenario and key[1] == statistic and key[4] == "opnet"
        }
        ns3_semantics = {
            (key[2], key[3])
            for key in grouped
            if key[0] == scenario and key[1] == statistic and key[4] == "ns3"
        }
        if not ns3_semantics:
            for unit, aggregation in sorted(opnet_semantics):
                opnet = grouped[(scenario, statistic, unit, aggregation, "opnet")]
                counts["unmatched_opnet_series"] += 1
                counts["skipped_noncomparable_points"] += len(opnet)
                append_difference(
                    {
                        "kind": "unmatched_opnet_series",
                        "scenario": scenario,
                        "statistic": statistic,
                        "unit": unit,
                        "aggregation": aggregation,
                        "opnet_points": len(opnet),
                    }
                )
                series_reports.append(
                    {
                        "scenario": scenario,
                        "statistic": statistic,
                        "unit": unit,
                        "aggregation": aggregation,
                        "status": "unmatched_opnet",
                        "opnet_points": len(opnet),
                        "ns3_points": 0,
                        "pass": False,
                    }
                )
            continue
        if not opnet_semantics:
            for unit, aggregation in sorted(ns3_semantics):
                ns3 = grouped[(scenario, statistic, unit, aggregation, "ns3")]
                counts["unmatched_ns3_series"] += 1
                counts["skipped_noncomparable_points"] += len(ns3)
                append_difference(
                    {
                        "kind": "unmatched_ns3_series",
                        "scenario": scenario,
                        "statistic": statistic,
                        "unit": unit,
                        "aggregation": aggregation,
                        "ns3_points": len(ns3),
                    }
                )
                series_reports.append(
                    {
                        "scenario": scenario,
                        "statistic": statistic,
                        "unit": unit,
                        "aggregation": aggregation,
                        "status": "unmatched_ns3",
                        "opnet_points": 0,
                        "ns3_points": len(ns3),
                        "pass": False,
                    }
                )
            continue

        common_semantics = opnet_semantics & ns3_semantics
        opnet_only = opnet_semantics - common_semantics
        ns3_only = ns3_semantics - common_semantics
        if opnet_only and ns3_only:
            skipped_points = sum(
                len(grouped[(scenario, statistic, unit, aggregation, "opnet")])
                for unit, aggregation in opnet_only
            ) + sum(
                len(grouped[(scenario, statistic, unit, aggregation, "ns3")])
                for unit, aggregation in ns3_only
            )
            counts["noncomparable_series"] += 1
            counts["skipped_noncomparable_points"] += skipped_points
            append_difference(
                {
                    "kind": "noncomparable_series",
                    "scenario": scenario,
                    "statistic": statistic,
                    "opnet_semantics": [
                        {"unit": unit, "aggregation": aggregation}
                        for unit, aggregation in sorted(opnet_only)
                    ],
                    "ns3_semantics": [
                        {"unit": unit, "aggregation": aggregation}
                        for unit, aggregation in sorted(ns3_only)
                    ],
                    "skipped_points": skipped_points,
                }
            )
            series_reports.append(
                {
                    "scenario": scenario,
                    "statistic": statistic,
                    "status": "noncomparable",
                    "opnet_semantics": [
                        {"unit": unit, "aggregation": aggregation}
                        for unit, aggregation in sorted(opnet_only)
                    ],
                    "ns3_semantics": [
                        {"unit": unit, "aggregation": aggregation}
                        for unit, aggregation in sorted(ns3_only)
                    ],
                    "skipped_points": skipped_points,
                    "pass": False,
                }
            )
        elif opnet_only:
            for unit, aggregation in sorted(opnet_only):
                opnet = grouped[(scenario, statistic, unit, aggregation, "opnet")]
                counts["unmatched_opnet_series"] += 1
                counts["skipped_noncomparable_points"] += len(opnet)
                append_difference(
                    {
                        "kind": "unmatched_opnet_series",
                        "scenario": scenario,
                        "statistic": statistic,
                        "unit": unit,
                        "aggregation": aggregation,
                        "opnet_points": len(opnet),
                    }
                )
                series_reports.append(
                    {
                        "scenario": scenario,
                        "statistic": statistic,
                        "unit": unit,
                        "aggregation": aggregation,
                        "status": "unmatched_opnet",
                        "opnet_points": len(opnet),
                        "ns3_points": 0,
                        "pass": False,
                    }
                )
        elif ns3_only:
            for unit, aggregation in sorted(ns3_only):
                ns3 = grouped[(scenario, statistic, unit, aggregation, "ns3")]
                counts["unmatched_ns3_series"] += 1
                counts["skipped_noncomparable_points"] += len(ns3)
                append_difference(
                    {
                        "kind": "unmatched_ns3_series",
                        "scenario": scenario,
                        "statistic": statistic,
                        "unit": unit,
                        "aggregation": aggregation,
                        "ns3_points": len(ns3),
                    }
                )
                series_reports.append(
                    {
                        "scenario": scenario,
                        "statistic": statistic,
                        "unit": unit,
                        "aggregation": aggregation,
                        "status": "unmatched_ns3",
                        "opnet_points": 0,
                        "ns3_points": len(ns3),
                        "pass": False,
                    }
                )

        for unit, aggregation in sorted(common_semantics):
            counts["comparable_series"] += 1
            opnet = grouped[(scenario, statistic, unit, aggregation, "opnet")]
            ns3 = grouped[(scenario, statistic, unit, aggregation, "ns3")]
            aligned, missing, extra = align_series(opnet, ns3, time_tolerance_s)
            counts["matched_points"] += len(aligned)
            counts["missing_in_ns3"] += len(missing)
            counts["extra_in_ns3"] += len(extra)
            mismatch_count = 0
            missing_ns3_value_count = 0
            missing_value_count = 0
            numeric_point_count = 0
            authoritative_numeric_point_count = sum(
                point.value is not None for point in opnet
            )
            maximum_absolute_delta = 0.0
            maximum_relative_delta = 0.0
            maximum_time_delta = 0.0

            identity = {
                "scenario": scenario,
                "statistic": statistic,
                "unit": unit,
                "aggregation": aggregation,
            }
            for point in missing:
                append_difference(
                    {
                        "kind": "missing_in_ns3",
                        **identity,
                        "opnet_time_s": point.time_s,
                        "opnet_value": point.value,
                        "opnet_location": _point_location(point),
                    }
                )
            for point in extra:
                append_difference(
                    {
                        "kind": "extra_in_ns3",
                        **identity,
                        "ns3_time_s": point.time_s,
                        "ns3_value": point.value,
                        "ns3_location": _point_location(point),
                    }
                )
            for expected, actual in aligned:
                time_delta = abs(expected.time_s - actual.time_s)
                maximum_time_delta = max(maximum_time_delta, time_delta)
                # A missing Modeler bucket (including the 2e100 sentinel
                # normalized by the extractor) is not authoritative numeric
                # evidence, so the aligned ns-3 value cannot be judged and is
                # reported as skipped.  The converse is a real ns-3 coverage
                # failure: an observed Modeler value must never disappear just
                # because ns-3 marked its aligned bucket missing.
                if expected.value is None:
                    missing_value_count += 1
                    counts["skipped_missing_values"] += 1
                    append_skipped(
                        {
                            "kind": "missing_value",
                            "reason": "opnet_reference_missing",
                            **identity,
                            "opnet_time_s": expected.time_s,
                            "ns3_time_s": actual.time_s,
                            "opnet_status": expected.value_status,
                            "ns3_status": actual.value_status,
                            "opnet_raw_value": expected.raw_value,
                            "ns3_raw_value": actual.raw_value,
                        }
                    )
                    continue
                if actual.value is None:
                    missing_ns3_value_count += 1
                    counts["missing_ns3_values"] += 1
                    append_difference(
                        {
                            "kind": "missing_ns3_value",
                            **identity,
                            "opnet_time_s": expected.time_s,
                            "ns3_time_s": actual.time_s,
                            "time_delta_s": time_delta,
                            "opnet_value": expected.value,
                            "ns3_status": actual.value_status,
                            "ns3_raw_value": actual.raw_value,
                            "opnet_location": _point_location(expected),
                            "ns3_location": _point_location(actual),
                        }
                    )
                    continue
                absolute_delta, relative_delta, absolute_delta_capped = _value_deltas(
                    expected.value, actual.value
                )
                numeric_point_count += 1
                counts["numeric_points_compared"] += 1
                relative_allowance, relative_allowance_capped = _saturating_product(
                    relative_tolerance,
                    max(abs(expected.value), abs(actual.value)),
                )
                allowed_delta = max(absolute_tolerance, relative_allowance)
                allowed_delta_capped = (
                    relative_allowance_capped
                    and relative_allowance >= absolute_tolerance
                )
                maximum_absolute_delta = max(maximum_absolute_delta, absolute_delta)
                maximum_relative_delta = max(maximum_relative_delta, relative_delta)
                within_absolute_tolerance = (
                    not absolute_delta_capped
                    and absolute_delta <= absolute_tolerance
                )
                within_relative_tolerance = relative_delta <= relative_tolerance
                if within_absolute_tolerance or within_relative_tolerance:
                    continue
                mismatch_count += 1
                counts["numeric_mismatches"] += 1
                append_difference(
                    {
                        "kind": "numeric_mismatch",
                        **identity,
                        "opnet_time_s": expected.time_s,
                        "ns3_time_s": actual.time_s,
                        "time_delta_s": time_delta,
                        "opnet_value": expected.value,
                        "ns3_value": actual.value,
                        "absolute_delta": absolute_delta,
                        "absolute_delta_capped": absolute_delta_capped,
                        "relative_delta": relative_delta,
                        "allowed_absolute_delta": allowed_delta,
                        "allowed_absolute_delta_capped": allowed_delta_capped,
                        "opnet_location": _point_location(expected),
                        "ns3_location": _point_location(actual),
                    }
                )

            indeterminate = authoritative_numeric_point_count == 0
            if indeterminate:
                counts["indeterminate_series"] += 1
                append_difference(
                    {
                        "kind": "indeterminate_series",
                        **identity,
                        "reason": "no_authoritative_numeric_points",
                        "opnet_points": len(opnet),
                        "ns3_points": len(ns3),
                        "matched_points": len(aligned),
                    }
                )

            series_reports.append(
                {
                    **identity,
                    "status": "indeterminate" if indeterminate else "compared",
                    "opnet_points": len(opnet),
                    "ns3_points": len(ns3),
                    "matched_points": len(aligned),
                    "authoritative_numeric_points": authoritative_numeric_point_count,
                    "numeric_points_compared": numeric_point_count,
                    "missing_in_ns3": len(missing),
                    "extra_in_ns3": len(extra),
                    "numeric_mismatches": mismatch_count,
                    "missing_ns3_values": missing_ns3_value_count,
                    "skipped_missing_values": missing_value_count,
                    "maximum_time_delta_s": maximum_time_delta,
                    "maximum_absolute_delta": maximum_absolute_delta,
                    "maximum_relative_delta": maximum_relative_delta,
                    "pass": (
                        not indeterminate
                        and not missing
                        and not extra
                        and mismatch_count == 0
                        and missing_ns3_value_count == 0
                    ),
                }
            )

    failure_count = (
        counts["unmatched_opnet_series"]
        + counts["unmatched_ns3_series"]
        + counts["noncomparable_series"]
        + counts["missing_in_ns3"]
        + counts["extra_in_ns3"]
        + counts["numeric_mismatches"]
        + counts["missing_ns3_values"]
        + counts["indeterminate_series"]
    )
    return {
        "schema": REPORT_SCHEMA,
        "pass": failure_count == 0,
        "opnet_point_count": sum(point.source == "opnet" for point in point_list),
        "ns3_point_count": sum(point.source == "ns3" for point in point_list),
        "series_identity_fields": [
            "scenario",
            "statistic",
            "unit",
            "aggregation",
        ],
        "point_identity_field": "time_s",
        "tolerances": {
            "absolute": absolute_tolerance,
            "relative": relative_tolerance,
            "time_s": time_tolerance_s,
        },
        "counts": counts,
        "reported_difference_count": len(differences),
        "total_difference_count": total_differences,
        "differences_truncated": total_differences > len(differences),
        "differences": differences,
        "reported_skipped_count": len(skipped),
        "total_skipped_count": total_skipped,
        "skipped_truncated": total_skipped > len(skipped),
        "skipped": skipped,
        "series": series_reports,
    }


def _nonnegative_finite(raw: str) -> float:
    try:
        value = float(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("value must be finite and nonnegative")
    return value


def _digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_output_paths(
    inputs: Iterable[Path], report: Path | None, normalized: Path | None
) -> None:
    """Reject output paths that alias an input or each other."""

    def resolved(path: Path) -> Path:
        try:
            return path.resolve()
        except (OSError, RuntimeError) as error:
            raise AggregateError(f"cannot resolve path {path}: {error}") from error

    def same_existing_file(left: Path, right: Path) -> bool:
        try:
            return left.samefile(right)
        except FileNotFoundError:
            return False
        except OSError as error:
            raise AggregateError(
                f"cannot compare file identity for {left} and {right}: {error}"
            ) from error

    input_paths = list(inputs)
    resolved_inputs: dict[Path, Path] = {}
    for input_path in input_paths:
        resolved_inputs.setdefault(resolved(input_path), input_path)

    resolved_outputs: dict[str, tuple[Path, Path]] = {}
    for option, output_path in (("--report", report), ("--normalized", normalized)):
        if output_path is None:
            continue
        output_resolved = resolved(output_path)
        aliased_input = resolved_inputs.get(output_resolved)
        if aliased_input is None:
            aliased_input = next(
                (
                    input_path
                    for input_path in input_paths
                    if same_existing_file(output_path, input_path)
                ),
                None,
            )
        if aliased_input is not None:
            raise AggregateError(
                f"{option} output {output_path} aliases aggregate input "
                f"{aliased_input}"
            )
        resolved_outputs[option] = (output_path, output_resolved)

    if report is not None and normalized is not None:
        report_resolved = resolved_outputs["--report"][1]
        normalized_resolved = resolved_outputs["--normalized"][1]
        if report_resolved == normalized_resolved:
            raise AggregateError(
                f"--report output {report} and --normalized output {normalized} "
                "resolve to the same path"
            )
        if same_existing_file(report, normalized):
            raise AggregateError(
                f"--report output {report} and --normalized output {normalized} "
                "alias the same file"
            )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="canonical aggregate CSV file(s), each row identifying its source",
    )
    parser.add_argument("--report", type=Path, help="write the summary JSON report")
    parser.add_argument(
        "--normalized", type=Path, help="write merged canonical rows in stable order"
    )
    parser.add_argument(
        "--abs-tolerance",
        type=_nonnegative_finite,
        default=0.0,
        help="global absolute value tolerance (default: exact)",
    )
    parser.add_argument(
        "--rel-tolerance",
        type=_nonnegative_finite,
        default=0.0,
        help="global symmetric relative value tolerance (default: exact)",
    )
    parser.add_argument(
        "--time-tolerance",
        type=_nonnegative_finite,
        default=0.0,
        help="maximum timestamp separation for point identity alignment",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help="include this exact scenario identity (repeatable)",
    )
    parser.add_argument(
        "--statistic",
        action="append",
        default=[],
        help="include this exact statistic identity (repeatable)",
    )
    parser.add_argument(
        "--unit",
        action="append",
        default=[],
        help="include this exact unit identity (repeatable)",
    )
    parser.add_argument(
        "--aggregation",
        action="append",
        default=[],
        help="include this exact aggregation identity (repeatable)",
    )
    parser.add_argument(
        "--time-start",
        type=_nonnegative_finite,
        help="include buckets at or after this simulation time",
    )
    parser.add_argument(
        "--time-stop",
        type=_nonnegative_finite,
        help="include buckets at or before this simulation time",
    )
    parser.add_argument(
        "--max-differences",
        type=int,
        default=200,
        help="maximum detailed missing/extra/mismatch records",
    )
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    if arguments.max_differences <= 0:
        raise SystemExit("error: --max-differences must be positive")
    if (
        arguments.time_start is not None
        and arguments.time_stop is not None
        and arguments.time_start > arguments.time_stop
    ):
        raise SystemExit("error: --time-start must not exceed --time-stop")
    try:
        validate_output_paths(
            arguments.inputs, arguments.report, arguments.normalized
        )
        all_points = read_aggregates(arguments.inputs)
        points = filter_points(
            all_points,
            set(arguments.scenario),
            set(arguments.statistic),
            set(arguments.unit),
            set(arguments.aggregation),
            arguments.time_start,
            arguments.time_stop,
        )
        if not points:
            raise AggregateError("aggregate filters selected no data rows")
        report = compare(
            points,
            arguments.abs_tolerance,
            arguments.rel_tolerance,
            arguments.time_tolerance,
            arguments.max_differences,
        )
        report["filters"] = {
            "scenarios": sorted(set(arguments.scenario)),
            "statistics": sorted(set(arguments.statistic)),
            "units": sorted(set(arguments.unit)),
            "aggregations": sorted(set(arguments.aggregation)),
            "time_start_s": arguments.time_start,
            "time_stop_s": arguments.time_stop,
            "input_point_count": len(all_points),
            "selected_point_count": len(points),
            "filtered_point_count": len(all_points) - len(points),
        }
        report["inputs"] = [
            {
                "path": str(path.resolve()),
                "sha256": _digest(path),
            }
            for path in arguments.inputs
        ]
        if arguments.normalized:
            write_normalized(arguments.normalized, points)
        if arguments.report:
            arguments.report.parent.mkdir(parents=True, exist_ok=True)
            arguments.report.write_text(
                json.dumps(report, indent=2, sort_keys=True, allow_nan=False) + "\n",
                encoding="utf-8",
            )
    except (AggregateError, OSError, csv.Error) as error:
        raise SystemExit(f"error: {error}") from error

    counts = report["counts"]
    print(
        f"{'PASS' if report['pass'] else 'FAIL'}: "
        f"OPNET={report['opnet_point_count']} ns-3={report['ns3_point_count']} "
        f"matched={counts['matched_points']} "
        f"missing={counts['missing_in_ns3']} extra={counts['extra_in_ns3']} "
        f"numeric={counts['numeric_mismatches']} "
        f"missing_ns3_values={counts['missing_ns3_values']} "
        f"indeterminate={counts['indeterminate_series']} "
        f"unmatched_series="
        f"{counts['unmatched_opnet_series'] + counts['unmatched_ns3_series']} "
        f"noncomparable={counts['noncomparable_series']} "
        f"skipped_values={counts['skipped_missing_values']}"
    )
    if not report["pass"]:
        for difference in report["differences"][:10]:
            print(json.dumps(difference, sort_keys=True, allow_nan=False))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
