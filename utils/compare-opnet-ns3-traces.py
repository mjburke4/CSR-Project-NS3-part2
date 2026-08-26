#!/usr/bin/env python3
"""Normalize and compare ordered OPNET and ns-3 CSR event traces."""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from difflib import SequenceMatcher
import json
import math
from pathlib import Path
import re
import sys
from typing import Iterable


TRACE_SCHEMA = "csr-differential-trace-v1"
TRACE_COLUMNS = (
    "schema",
    "event_index",
    "time_s",
    "event",
    "node",
    "peer",
    "packet_type",
    "src",
    "dst",
    "sequence",
    "rate_kbps",
    "size_bytes",
    "success",
    "reason",
    "pathloss_db",
    "rx_power_dbm",
    "noise_dbm",
    "snr_db",
    "jsr_db",
    "header_errors",
    "payload_errors",
    "total_errors",
    "route_cost",
    "next_hop",
    "security_count",
    "detail",
)

ALIASES = {
    "event_index": ("event_index", "index", "event_id"),
    "time_s": ("time_s", "time", "t", "sim_time", "simulation_time"),
    "event": ("event", "event_type", "action", "stage"),
    "node": ("node", "node_id", "receiver", "rx_node"),
    "peer": ("peer", "peer_id", "transmitter", "tx_node"),
    "packet_type": ("packet_type", "pkt_type", "type"),
    "src": ("src", "source", "source_node"),
    "dst": ("dst", "destination", "destination_node"),
    "sequence": ("sequence", "seq", "sequence_number"),
    "rate_kbps": ("rate_kbps", "rate", "speed", "speed_kbps"),
    "size_bytes": ("size_bytes", "bytes", "packet_bytes", "size"),
    "success": ("success", "accepted", "accept", "delivered"),
    "reason": ("reason", "drop_reason", "result"),
    "pathloss_db": ("pathloss_db", "path_loss_db", "pathloss"),
    "rx_power_dbm": ("rx_power_dbm", "received_power_dbm", "rx_power"),
    "noise_dbm": ("noise_dbm", "noise_power_dbm", "noise"),
    "snr_db": ("snr_db", "snr"),
    "jsr_db": ("jsr_db", "jsr"),
    "header_errors": ("header_errors", "header_error_count"),
    "payload_errors": ("payload_errors", "payload_error_count"),
    "total_errors": ("total_errors", "error_count", "errors"),
    "route_cost": ("route_cost", "cost"),
    "next_hop": ("next_hop", "nexthop"),
    "security_count": ("security_count", "security_epoch"),
    "detail": ("detail", "message", "description"),
}

EVENT_ALIASES = {
    "tx": "tx_start",
    "transmit": "tx_start",
    "mac_tx": "tx_start",
    "rx": "rx_accept",
    "receive": "rx_accept",
    "accept": "rx_accept",
    "accepted": "rx_accept",
    "drop": "rx_drop",
    "dropped": "rx_drop",
    "deliver": "nwk_delivery",
    "delivery": "nwk_delivery",
    "route": "route_change",
    "state": "mac_state",
}

NUMERIC_FIELDS = {
    "event_index",
    "time_s",
    "node",
    "peer",
    "src",
    "dst",
    "sequence",
    "rate_kbps",
    "size_bytes",
    "pathloss_db",
    "rx_power_dbm",
    "noise_dbm",
    "snr_db",
    "jsr_db",
    "header_errors",
    "payload_errors",
    "total_errors",
    "route_cost",
    "next_hop",
    "security_count",
}

DEFAULT_TOLERANCES = {
    "time_s": 1.0e-6,
    "pathloss_db": 1.0e-3,
    "rx_power_dbm": 1.0e-3,
    "noise_dbm": 1.0e-3,
    "snr_db": 1.0e-3,
    "jsr_db": 1.0e-3,
}

DEFAULT_KEY_FIELDS = (
    "event",
    "node",
    "peer",
    "packet_type",
    "src",
    "dst",
    "sequence",
)

IGNORABLE_VALUES = {"", "n/a", "na", "none", "null", "-"}


class TraceError(ValueError):
    """The input trace cannot be normalized safely."""


@dataclass(frozen=True)
class Event:
    """One canonical trace row and its original input position."""

    values: dict[str, str]
    input_row: int


def _normalized_header(value: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", value.strip().lower()).strip("_")


def _clean_value(value: str | None) -> str:
    if value is None:
        return ""
    cleaned = value.strip()
    return "" if cleaned.lower() in IGNORABLE_VALUES else cleaned


def _boolean(value: str) -> str:
    if not value:
        return ""
    lowered = value.lower()
    if lowered in {"1", "true", "yes", "accept", "accepted", "pass", "delivered"}:
        return "1"
    if lowered in {"0", "false", "no", "drop", "dropped", "fail", "rejected"}:
        return "0"
    raise TraceError(f"invalid success value {value!r}")


def _canonical_number(field: str, value: str) -> str:
    if not value:
        return ""
    try:
        number = float(value.replace(",", ""))
    except ValueError as error:
        raise TraceError(f"{field} is not numeric: {value!r}") from error
    if not math.isfinite(number):
        # Preserve signed infinities emitted by the PHY diagnostics.
        lowered = value.lower()
        if lowered in {"inf", "+inf", "infinity", "+infinity"}:
            return "inf"
        if lowered in {"-inf", "-infinity"}:
            return "-inf"
        raise TraceError(f"{field} is not finite: {value!r}")
    if field in {
        "event_index",
        "node",
        "peer",
        "src",
        "dst",
        "sequence",
        "rate_kbps",
        "size_bytes",
        "header_errors",
        "payload_errors",
        "total_errors",
        "next_hop",
        "security_count",
    }:
        integer = int(number)
        if number != integer:
            raise TraceError(f"{field} is not an integer: {value!r}")
        return str(integer)
    return repr(number)


def normalize_trace(path: Path) -> list[Event]:
    """Map a canonical or alias-column CSV into ordered canonical events."""
    try:
        stream = path.open("r", encoding="utf-8-sig", newline="")
    except OSError as error:
        raise TraceError(str(error)) from error
    with stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise TraceError(f"{path}: trace has no CSV header")
        source_headers = {_normalized_header(name): name for name in reader.fieldnames}
        column_map: dict[str, str] = {}
        for canonical, aliases in ALIASES.items():
            for alias in aliases:
                source = source_headers.get(_normalized_header(alias))
                if source is not None:
                    column_map[canonical] = source
                    break
        missing = [field for field in ("time_s", "event", "node") if field not in column_map]
        if missing:
            raise TraceError(f"{path}: missing required columns: {', '.join(missing)}")

        events: list[Event] = []
        last_time = -math.inf
        for row_number, row in enumerate(reader, start=2):
            values = {field: "" for field in TRACE_COLUMNS}
            values["schema"] = TRACE_SCHEMA
            for field, source in column_map.items():
                values[field] = _clean_value(row.get(source))
            if not values["event_index"]:
                values["event_index"] = str(len(events))
            values["event"] = re.sub(
                r"[^a-z0-9]+", "_", values["event"].strip().lower()
            ).strip("_")
            values["event"] = EVENT_ALIASES.get(values["event"], values["event"])
            if not values["event"]:
                raise TraceError(f"{path}:{row_number}: event is empty")
            values["success"] = _boolean(values["success"])
            for field in NUMERIC_FIELDS:
                values[field] = _canonical_number(field, values[field])
            time_s = float(values["time_s"])
            if time_s + 1.0e-15 < last_time:
                raise TraceError(
                    f"{path}:{row_number}: trace time decreased from {last_time} to {time_s}"
                )
            last_time = time_s
            events.append(Event(values=values, input_row=row_number))
    return events


def write_normalized(path: Path, events: Iterable[Event]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for event in events:
            writer.writerow(event.values)


def parse_tolerance(specification: str) -> tuple[str, float]:
    if "=" not in specification:
        raise argparse.ArgumentTypeError("tolerance must be FIELD=ABSOLUTE_VALUE")
    field, raw = specification.split("=", 1)
    field = _normalized_header(field)
    if field not in TRACE_COLUMNS or field not in NUMERIC_FIELDS:
        raise argparse.ArgumentTypeError(f"unknown numeric trace field {field!r}")
    try:
        value = float(raw)
    except ValueError as error:
        raise argparse.ArgumentTypeError(str(error)) from error
    if not math.isfinite(value) or value < 0.0:
        raise argparse.ArgumentTypeError("tolerance must be finite and nonnegative")
    return field, value


def _active_key_fields(
    opnet: list[Event], ns3: list[Event], requested: tuple[str, ...]
) -> tuple[str, ...]:
    active: list[str] = []
    for field in requested:
        if field not in TRACE_COLUMNS:
            raise TraceError(f"unknown key field {field!r}")
        if field in {"event", "node"}:
            active.append(field)
            continue
        if any(event.values[field] for event in opnet) and any(
            event.values[field] for event in ns3
        ):
            active.append(field)
    return tuple(active)


def _event_key(event: Event, fields: tuple[str, ...]) -> tuple[str, ...]:
    return tuple(event.values[field] for field in fields)


def _field_difference(
    field: str,
    expected: str,
    actual: str,
    tolerances: dict[str, float],
) -> tuple[bool, float | None]:
    if field in NUMERIC_FIELDS:
        expected_number = float(expected)
        actual_number = float(actual)
        if math.isinf(expected_number) or math.isinf(actual_number):
            return expected_number != actual_number, None
        delta = abs(expected_number - actual_number)
        return delta > tolerances.get(field, 0.0), delta
    return expected != actual, None


def compare(
    opnet: list[Event],
    ns3: list[Event],
    key_fields: tuple[str, ...],
    tolerances: dict[str, float],
    ignored_fields: set[str],
    max_differences: int,
) -> dict[str, object]:
    """Align event identities and compare every mutually observed field."""
    active_keys = _active_key_fields(opnet, ns3, key_fields)
    matcher = SequenceMatcher(
        None,
        [_event_key(event, active_keys) for event in opnet],
        [_event_key(event, active_keys) for event in ns3],
        autojunk=False,
    )
    differences: list[dict[str, object]] = []
    counts = {
        "matched_events": 0,
        "missing_in_ns3": 0,
        "extra_in_ns3": 0,
        "replaced_events": 0,
        "field_mismatches": 0,
        "coverage_gaps": 0,
    }

    def append_difference(item: dict[str, object]) -> None:
        if len(differences) < max_differences:
            differences.append(item)

    def compare_pair(expected: Event, actual: Event) -> None:
        counts["matched_events"] += 1
        for field in TRACE_COLUMNS:
            if field in {"schema", "event_index"} or field in ignored_fields:
                continue
            expected_value = expected.values[field]
            actual_value = actual.values[field]
            if not expected_value or not actual_value:
                if expected_value != actual_value:
                    counts["coverage_gaps"] += 1
                continue
            different, delta = _field_difference(
                field, expected_value, actual_value, tolerances
            )
            if different:
                counts["field_mismatches"] += 1
                item: dict[str, object] = {
                    "kind": "field",
                    "field": field,
                    "opnet_row": expected.input_row,
                    "ns3_row": actual.input_row,
                    "opnet": expected_value,
                    "ns3": actual_value,
                }
                if delta is not None:
                    item["absolute_delta"] = delta
                    item["tolerance"] = tolerances.get(field, 0.0)
                append_difference(item)

    for tag, i1, i2, j1, j2 in matcher.get_opcodes():
        if tag == "equal":
            for expected, actual in zip(opnet[i1:i2], ns3[j1:j2]):
                compare_pair(expected, actual)
            continue
        if tag == "delete":
            for expected in opnet[i1:i2]:
                counts["missing_in_ns3"] += 1
                append_difference(
                    {
                        "kind": "missing_in_ns3",
                        "opnet_row": expected.input_row,
                        "key": dict(zip(active_keys, _event_key(expected, active_keys))),
                    }
                )
            continue
        if tag == "insert":
            for actual in ns3[j1:j2]:
                counts["extra_in_ns3"] += 1
                append_difference(
                    {
                        "kind": "extra_in_ns3",
                        "ns3_row": actual.input_row,
                        "key": dict(zip(active_keys, _event_key(actual, active_keys))),
                    }
                )
            continue

        expected_block = opnet[i1:i2]
        actual_block = ns3[j1:j2]
        paired = min(len(expected_block), len(actual_block))
        for index in range(paired):
            expected = expected_block[index]
            actual = actual_block[index]
            counts["replaced_events"] += 1
            append_difference(
                {
                    "kind": "replaced_event",
                    "opnet_row": expected.input_row,
                    "ns3_row": actual.input_row,
                    "opnet_key": dict(zip(active_keys, _event_key(expected, active_keys))),
                    "ns3_key": dict(zip(active_keys, _event_key(actual, active_keys))),
                }
            )
            compare_pair(expected, actual)
        for expected in expected_block[paired:]:
            counts["missing_in_ns3"] += 1
            append_difference(
                {
                    "kind": "missing_in_ns3",
                    "opnet_row": expected.input_row,
                    "key": dict(zip(active_keys, _event_key(expected, active_keys))),
                }
            )
        for actual in actual_block[paired:]:
            counts["extra_in_ns3"] += 1
            append_difference(
                {
                    "kind": "extra_in_ns3",
                    "ns3_row": actual.input_row,
                    "key": dict(zip(active_keys, _event_key(actual, active_keys))),
                }
            )

    failure_count = (
        counts["missing_in_ns3"]
        + counts["extra_in_ns3"]
        + counts["replaced_events"]
        + counts["field_mismatches"]
    )
    return {
        "schema": "csr-differential-report-v1",
        "pass": failure_count == 0,
        "opnet_event_count": len(opnet),
        "ns3_event_count": len(ns3),
        "key_fields": list(active_keys),
        "tolerances": tolerances,
        "ignored_fields": sorted(ignored_fields),
        "counts": counts,
        "reported_difference_count": len(differences),
        "differences_truncated": failure_count > len(differences),
        "differences": differences,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("opnet_trace", type=Path, help="OPNET canonical/alias CSV")
    parser.add_argument("ns3_trace", type=Path, help="ns-3 canonical/alias CSV")
    parser.add_argument("--report", type=Path, help="write machine-readable JSON report")
    parser.add_argument("--normalized-opnet", type=Path, help="write normalized OPNET CSV")
    parser.add_argument("--normalized-ns3", type=Path, help="write normalized ns-3 CSV")
    parser.add_argument(
        "--key-fields",
        default=",".join(DEFAULT_KEY_FIELDS),
        help="comma-separated event identity fields",
    )
    parser.add_argument(
        "--tolerance",
        action="append",
        default=[],
        type=parse_tolerance,
        metavar="FIELD=VALUE",
        help="override an absolute numeric tolerance",
    )
    parser.add_argument(
        "--ignore-field",
        action="append",
        default=[],
        help="canonical field to exclude from matched-row comparison",
    )
    parser.add_argument(
        "--max-differences", type=int, default=200, help="maximum detailed differences"
    )
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    try:
        opnet = normalize_trace(arguments.opnet_trace)
        ns3 = normalize_trace(arguments.ns3_trace)
        if arguments.normalized_opnet:
            write_normalized(arguments.normalized_opnet, opnet)
        if arguments.normalized_ns3:
            write_normalized(arguments.normalized_ns3, ns3)
        tolerances = dict(DEFAULT_TOLERANCES)
        tolerances.update(arguments.tolerance)
        key_fields = tuple(
            _normalized_header(field)
            for field in arguments.key_fields.split(",")
            if field.strip()
        )
        ignored_fields = {_normalized_header(field) for field in arguments.ignore_field}
        unknown_ignored = ignored_fields - set(TRACE_COLUMNS)
        if unknown_ignored:
            raise TraceError(f"unknown ignored fields: {', '.join(sorted(unknown_ignored))}")
        if arguments.max_differences <= 0:
            raise TraceError("--max-differences must be positive")
        report = compare(
            opnet,
            ns3,
            key_fields,
            tolerances,
            ignored_fields,
            arguments.max_differences,
        )
    except (OSError, TraceError, csv.Error) as error:
        raise SystemExit(f"error: {error}") from error

    if arguments.report:
        arguments.report.parent.mkdir(parents=True, exist_ok=True)
        arguments.report.write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    counts = report["counts"]
    print(
        f"{'PASS' if report['pass'] else 'FAIL'}: "
        f"OPNET={report['opnet_event_count']} ns-3={report['ns3_event_count']} "
        f"missing={counts['missing_in_ns3']} extra={counts['extra_in_ns3']} "
        f"replaced={counts['replaced_events']} fields={counts['field_mismatches']} "
        f"coverage_gaps={counts['coverage_gaps']}"
    )
    if not report["pass"]:
        for difference in report["differences"][:10]:
            print(json.dumps(difference, sort_keys=True))
        raise SystemExit(1)


if __name__ == "__main__":
    main()
