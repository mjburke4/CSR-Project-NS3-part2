#!/usr/bin/env python3
"""Derive OPNET-comparable aggregate series from a CSR ns-3 event trace.

New traces carry an observation-only application packet sequence and use it
for exact delay correlation.  Legacy traces are matched FIFO within each
``(src, dst)`` pair.  Provenance records the method and every unmatched event
instead of hiding that limitation.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict, deque
import csv
import hashlib
import json
import math
from pathlib import Path
import re
from typing import Iterable, Iterator


TRACE_SCHEMA = "csr-differential-trace-v1"
SERIES_SCHEMA = "csr-aggregate-series-v1"
PROVENANCE_SCHEMA = "csr-ns3-aggregate-provenance-v1"

CORE_COLUMNS = (
    "schema",
    "scenario",
    "statistic",
    "time_s",
    "value",
    "source",
)
EXTRA_COLUMNS = (
    "unit",
    "aggregation",
    "raw_value",
    "value_status",
    "source_file",
    "source_file_sha256",
)
OUTPUT_COLUMNS = CORE_COLUMNS + EXTRA_COLUMNS

SENT_PACKETS_RATE = "Generator.Traffic Sent (packets/sec)"
SENT_BITS_RATE = "Generator.Traffic Sent (bits/sec)"
PACKET_SIZE = "Generator.Packet Size (bits)"
RECEIVED_PACKETS_RATE = "Sink.Traffic Received (packets/sec)"
RECEIVED_PACKETS = "Sink.Traffic Received (packets)"
RECEIVED_BITS_RATE = "Sink.Traffic Received (bits/sec)"
RECEIVED_BITS = "Sink.Traffic Received (bits)"
END_TO_END_DELAY = "Sink.End-to-End Delay (seconds)"
ECC_DROPS_RATE = "ECC.Traffic Dropped (packets/sec)"
HOP_RESEND_QUEUE_SIZE = "HOP.Resend Queue Size (packets)"
MAC_ACK_QUEUE_SIZE = "MAC.ACK Queue Size (packets)"
MAC_TX_QUEUE_SIZE = "MAC.Tx Queue Size (packets)"
MAC_TX_QUEUE_DELAY = "MAC.Tx Queuing Delay (sec)"
NWK_QUEUE_SIZE = "NWK.Network Queue Size (packets)"
NWK_QUEUE_DELAY = "NWK.Network Queuing Delay (sec)"

STATISTIC_SAMPLE_EVENT = "statistic_sample"
SOURCE_EXACT_DACK_RETRY_METHOD = "source_exact_dack_retry_duplicate"

STATISTICS = (
    (SENT_PACKETS_RATE, "packets/s", "bucket_sum_per_second"),
    (SENT_BITS_RATE, "bits/s", "bucket_sum_per_second"),
    (PACKET_SIZE, "bits", "bucket_sample_mean"),
    (RECEIVED_PACKETS_RATE, "packets/s", "bucket_sum_per_second"),
    (RECEIVED_PACKETS, "packets", "bucket_sum"),
    (RECEIVED_BITS_RATE, "bits/s", "bucket_sum_per_second"),
    (RECEIVED_BITS, "bits", "bucket_sum"),
    (END_TO_END_DELAY, "s", "bucket_sample_mean"),
    (ECC_DROPS_RATE, "packets/s", "bucket_sum_per_second"),
)

# These vectors are activated dynamically when a trace contains at least one
# matching statistic_sample event.  This keeps legacy traces byte-shape
# compatible (nine derived series) while making an explicitly instrumented
# trace fail closed if a sample identity or value is malformed.
STATISTIC_SAMPLE_STATISTICS = (
    (HOP_RESEND_QUEUE_SIZE, "packets", "bucket_sample_mean", True),
    (MAC_ACK_QUEUE_SIZE, "packets", "bucket_sample_mean", True),
    (MAC_TX_QUEUE_SIZE, "packets", "bucket_sample_mean", True),
    (MAC_TX_QUEUE_DELAY, "s", "bucket_sample_mean", False),
    (NWK_QUEUE_SIZE, "packets", "bucket_sample_mean", True),
    (NWK_QUEUE_DELAY, "s", "bucket_sample_mean", False),
)
STATISTIC_SAMPLE_SEMANTICS = {
    name: (unit, aggregation, integral)
    for name, unit, aggregation, integral in STATISTIC_SAMPLE_STATISTICS
}

REQUIRED_COLUMNS = {
    "schema",
    "event_index",
    "time_s",
    "event",
    "src",
    "dst",
    "size_bytes",
    "reason",
}


class TraceAggregationError(ValueError):
    """The event trace cannot be aggregated without guessing."""


def digest(path: Path) -> str:
    """Return the SHA-256 digest of *path*."""

    value = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def _paths_alias(first: Path, second: Path) -> bool:
    """Return whether two resolved paths identify the same filesystem entry."""

    if first == second:
        return True
    try:
        return first.samefile(second)
    except OSError:
        # One or both outputs commonly do not exist yet. Their resolved names
        # were already compared above, including existing symlink targets.
        return False


def _validate_cli_paths(trace: Path, output: Path, provenance: Path) -> None:
    """Protect the input trace and keep the two generated artifacts distinct."""

    if _paths_alias(trace, output):
        raise TraceAggregationError(
            "aggregate output must not alias the input trace"
        )
    if _paths_alias(trace, provenance):
        raise TraceAggregationError(
            "provenance output must not alias the input trace"
        )
    if _paths_alias(output, provenance):
        raise TraceAggregationError(
            "aggregate output must not alias the provenance output"
        )


def _format_number(value: float) -> str:
    if not math.isfinite(value):
        raise TraceAggregationError("refusing to write a non-finite aggregate value")
    if value == 0.0:
        return "0"
    return format(value, ".17g")


def _parse_nonnegative_float(value: str, label: str, row_number: int) -> float:
    try:
        parsed = float(value)
    except ValueError as error:
        raise TraceAggregationError(
            f"row {row_number}: {label} is not numeric: {value!r}"
        ) from error
    if not math.isfinite(parsed) or parsed < 0.0:
        raise TraceAggregationError(
            f"row {row_number}: {label} must be finite and nonnegative"
        )
    return parsed


def _parse_index(value: str, row_number: int) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as error:
        raise TraceAggregationError(
            f"row {row_number}: event_index is not an integer: {value!r}"
        ) from error
    if parsed < 0:
        raise TraceAggregationError(
            f"row {row_number}: event_index must be nonnegative"
        )
    return parsed


def _parse_nonnegative_integer(value: str, label: str, row_number: int) -> int:
    try:
        parsed = int(value, 10)
    except ValueError as error:
        raise TraceAggregationError(
            f"row {row_number}: {label} is not an integer: {value!r}"
        ) from error
    if parsed < 0:
        raise TraceAggregationError(
            f"row {row_number}: {label} must be nonnegative"
        )
    return parsed


def _statistic_sample(
    row: dict[str, str], row_number: int
) -> tuple[str, str, float]:
    """Validate and return one explicit source-equivalent statistic sample."""

    missing_columns = [
        field for field in ("node", "statistic", "value") if field not in row
    ]
    if missing_columns:
        raise TraceAggregationError(
            f"row {row_number}: {STATISTIC_SAMPLE_EVENT} requires columns: "
            + ", ".join(missing_columns)
        )

    node_raw = (row.get("node") or "").strip()
    statistic = (row.get("statistic") or "").strip()
    value_raw = (row.get("value") or "").strip()
    if not node_raw:
        raise TraceAggregationError(
            f"row {row_number}: {STATISTIC_SAMPLE_EVENT} requires node"
        )
    if not statistic:
        raise TraceAggregationError(
            f"row {row_number}: {STATISTIC_SAMPLE_EVENT} requires statistic"
        )
    if not value_raw:
        raise TraceAggregationError(
            f"row {row_number}: {STATISTIC_SAMPLE_EVENT} requires value"
        )
    semantics = STATISTIC_SAMPLE_SEMANTICS.get(statistic)
    if semantics is None:
        raise TraceAggregationError(
            f"row {row_number}: unsupported {STATISTIC_SAMPLE_EVENT} statistic "
            f"{statistic!r}"
        )

    node = _parse_nonnegative_integer(node_raw, "node", row_number)
    value = _parse_nonnegative_float(value_raw, "value", row_number)
    if semantics[2] and not value.is_integer():
        raise TraceAggregationError(
            f"row {row_number}: {statistic} sample must be an integer packet count"
        )
    return statistic, str(node), value


def _trace_packet_bits(
    row: dict[str, str], row_number: int, size_exclusion_bits: int
) -> int:
    value = row["size_bytes"].strip()
    if not value:
        raise TraceAggregationError(
            f"row {row_number}: {row['event']} requires size_bytes"
        )
    try:
        size_bytes = int(value, 10)
    except ValueError as error:
        raise TraceAggregationError(
            f"row {row_number}: size_bytes is not an integer: {value!r}"
        ) from error
    if size_bytes < 0:
        raise TraceAggregationError(
            f"row {row_number}: size_bytes must be nonnegative"
        )
    trace_bits = size_bytes * 8
    if trace_bits < size_exclusion_bits:
        raise TraceAggregationError(
            f"row {row_number}: size_bytes={size_bytes} is too small for "
            f"the {size_exclusion_bits}-bit legacy trace adjustment"
        )
    return trace_bits - size_exclusion_bits


def _flow_key(row: dict[str, str], row_number: int) -> tuple[str, str]:
    source = row["src"].strip()
    destination = row["dst"].strip()
    if not source or not destination:
        raise TraceAggregationError(
            f"row {row_number}: {row['event']} requires both src and dst"
        )
    return source, destination


def _detail_fields(text: str, row_number: int) -> dict[str, str]:
    """Parse one fail-closed semicolon-delimited trace detail field."""

    parsed: dict[str, str] = {}
    for token in text.strip().split(";"):
        if not token or "=" not in token:
            raise TraceAggregationError(
                f"row {row_number}: detail must contain semicolon-separated "
                "key=value pairs"
            )
        key, value = (part.strip() for part in token.split("=", 1))
        if not key or not re.fullmatch(r"[a-z][a-z0-9_]*", key):
            raise TraceAggregationError(
                f"row {row_number}: invalid detail key {key!r}"
            )
        if key in parsed:
            raise TraceAggregationError(
                f"row {row_number}: duplicate detail key {key!r}"
            )
        if not value:
            raise TraceAggregationError(
                f"row {row_number}: detail value for {key!r} is empty"
            )
        parsed[key] = value
    return parsed


def _retry_feedback_lineage(
    row: dict[str, str], row_number: int
) -> tuple[tuple[tuple[str, str, str], str, str, str], str] | None:
    """Return one qualifying ACK/DACK reception and its exact lineage.

    A later qualifying reception proves a source-exact retry only when an
    earlier row on the same lineage selected DACK.  The later reassessment may
    select ACK if congestion has cleared.
    """

    reason = row["reason"].strip().lower()
    if reason not in {"ack", "dack"}:
        return None
    missing_columns = [name for name in ("detail",) if name not in row]
    if missing_columns:
        raise TraceAggregationError(
            f"row {row_number}: hop_feedback {reason.upper()} requires columns: "
            + ", ".join(missing_columns)
        )

    detail_raw = row["detail"].strip()
    if not detail_raw:
        raise TraceAggregationError(
            f"row {row_number}: hop_feedback {reason.upper()} requires detail"
        )
    detail = _detail_fields(detail_raw, row_number)
    required = {
        "hop_sequence",
        "first_reception",
        "ackable",
        "nsdp_count_before",
        "nsdp_count_after",
        "nsdp_state_valid",
        "nsdp_limit",
    }
    missing = sorted(required - set(detail))
    if missing:
        raise TraceAggregationError(
            f"row {row_number}: hop_feedback {reason.upper()} detail omits "
            + ", ".join(missing)
        )
    if (
        detail["first_reception"] != "1"
        or detail["ackable"] != "1"
        or detail["nsdp_state_valid"] != "1"
    ):
        return None
    flow = _flow_key(row, row_number)
    sequence_raw = row["sequence"].strip()
    node_raw = row["node"].strip()
    peer_raw = row["peer"].strip()
    for label, value in (
        ("sequence", sequence_raw),
        ("node", node_raw),
        ("peer", peer_raw),
    ):
        if not value:
            raise TraceAggregationError(
                f"row {row_number}: hop_feedback {reason.upper()} requires {label}"
            )
    sequence = str(
        _parse_nonnegative_integer(sequence_raw, "sequence", row_number)
    )
    node = str(_parse_nonnegative_integer(node_raw, "node", row_number))
    peer = str(_parse_nonnegative_integer(peer_raw, "peer", row_number))
    hop_sequence = str(
        _parse_nonnegative_integer(
            detail["hop_sequence"], "hop_sequence", row_number
        )
    )
    before = _parse_nonnegative_integer(
        detail["nsdp_count_before"], "nsdp_count_before", row_number
    )
    after = _parse_nonnegative_integer(
        detail["nsdp_count_after"], "nsdp_count_after", row_number
    )
    limit = _parse_nonnegative_integer(
        detail["nsdp_limit"], "nsdp_limit", row_number
    )
    success = (row.get("success") or "").strip()
    if success and success != "1":
        return None
    if (
        limit != 16
        or after != before + 1
    ):
        return None
    if (reason == "dack" and before < limit) or (
        reason == "ack" and before >= limit
    ):
        return None
    return (((flow[0], flow[1], sequence), node, peer, hop_sequence), reason)


def _feedback_lineage_identity(
    row: dict[str, str], row_number: int
) -> tuple[tuple[str, str, str], str, str, str] | None:
    """Return a carried feedback lineage even when its proof state is invalid."""

    if row["reason"].strip().lower() not in {"ack", "dack"}:
        return None
    raw = {
        name: (row.get(name) or "").strip()
        for name in ("src", "dst", "sequence", "node", "peer")
    }
    detail_raw = (row.get("detail") or "").strip()
    if any(not value for value in raw.values()) or not detail_raw:
        return None
    detail = _detail_fields(detail_raw, row_number)
    hop_sequence_raw = detail.get("hop_sequence", "").strip()
    if not hop_sequence_raw:
        return None
    return (
        (
            str(_parse_nonnegative_integer(raw["src"], "src", row_number)),
            str(_parse_nonnegative_integer(raw["dst"], "dst", row_number)),
            str(
                _parse_nonnegative_integer(
                    raw["sequence"], "sequence", row_number
                )
            ),
        ),
        str(_parse_nonnegative_integer(raw["node"], "node", row_number)),
        str(_parse_nonnegative_integer(raw["peer"], "peer", row_number)),
        str(
            _parse_nonnegative_integer(
                hop_sequence_raw, "hop_sequence", row_number
            )
        ),
    )


def _relay_enqueue_key(
    row: dict[str, str], row_number: int
) -> tuple[tuple[str, str, str], str, str] | None:
    """Return the exact packet/relay/ingress identity for a relay enqueue."""

    if row["reason"].strip().lower() != "relay":
        return None
    success = (row.get("success") or "").strip()
    if success and success != "1":
        return None
    flow = _flow_key(row, row_number)
    values = {name: row[name].strip() for name in ("sequence", "node", "peer")}
    for name, value in values.items():
        if not value:
            raise TraceAggregationError(
                f"row {row_number}: relay nwk_enqueue requires {name}"
            )
        values[name] = str(_parse_nonnegative_integer(value, name, row_number))
    return (
        (flow[0], flow[1], values["sequence"]),
        values["node"],
        values["peer"],
    )


def _feedback_enqueue_key(
    row: dict[str, str], row_number: int
) -> tuple[tuple[str, str, str], str, str] | None:
    """Return feedback identity when carried, including non-proof feedback."""

    if row["reason"].strip().lower() not in {"ack", "dack"}:
        return None
    raw = {name: row[name].strip() for name in ("src", "dst", "sequence", "node", "peer")}
    if any(not value for value in raw.values()):
        return None
    return (
        (
            str(_parse_nonnegative_integer(raw["src"], "src", row_number)),
            str(_parse_nonnegative_integer(raw["dst"], "dst", row_number)),
            str(_parse_nonnegative_integer(raw["sequence"], "sequence", row_number)),
        ),
        str(_parse_nonnegative_integer(raw["node"], "node", row_number)),
        str(_parse_nonnegative_integer(raw["peer"], "peer", row_number)),
    )


def _bucket_index(time_s: float, bucket_width_s: float, bucket_count: int) -> int:
    """Map an in-window time to [previous bucket end, bucket end)."""

    quotient = time_s / bucket_width_s
    nearest = round(quotient)
    if math.isclose(quotient, nearest, rel_tol=0.0, abs_tol=1.0e-12):
        quotient = float(nearest)
    index = math.floor(quotient)
    if index < 0 or index >= bucket_count:
        stop_time_s = bucket_width_s * bucket_count
        raise TraceAggregationError(
            f"event time {time_s!r} is outside the half-open aggregation "
            f"window [0, {stop_time_s!r})"
        )
    return index


def _is_stop_boundary(
    time_s: float, bucket_width_s: float, bucket_count: int
) -> bool:
    """Use the bucket mapper's quotient tolerance at the exclusive stop."""

    return math.isclose(
        time_s / bucket_width_s,
        float(bucket_count),
        rel_tol=0.0,
        abs_tol=1.0e-12,
    )


def _validate_window(bucket_width_s: float, stop_time_s: float) -> int:
    if not math.isfinite(bucket_width_s) or bucket_width_s <= 0.0:
        raise TraceAggregationError("--bucket-width must be finite and positive")
    if not math.isfinite(stop_time_s) or stop_time_s <= 0.0:
        raise TraceAggregationError("--stop-time must be finite and positive")
    quotient = stop_time_s / bucket_width_s
    rounded = round(quotient)
    if rounded <= 0 or not math.isclose(
        quotient, rounded, rel_tol=1.0e-12, abs_tol=1.0e-12
    ):
        raise TraceAggregationError(
            "--stop-time must be an integer multiple of --bucket-width"
        )
    return int(rounded)


def _event_rows(path: Path) -> Iterator[tuple[int, float, dict[str, str]]]:
    """Yield validated rows without retaining a potentially large trace."""

    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        if reader.fieldnames is None:
            raise TraceAggregationError("event trace has no CSV header")
        if len(set(reader.fieldnames)) != len(reader.fieldnames):
            raise TraceAggregationError("event trace has duplicate CSV columns")
        missing = sorted(REQUIRED_COLUMNS - set(reader.fieldnames))
        if missing:
            raise TraceAggregationError(
                "event trace is missing required columns: " + ", ".join(missing)
            )

        previous_index = -1
        previous_time = -math.inf
        event_count = 0
        for row_number, row in enumerate(reader, start=2):
            if None in row:
                raise TraceAggregationError(
                    f"row {row_number}: contains fields beyond the CSV header"
                )
            if any(value is None for value in row.values()):
                raise TraceAggregationError(
                    f"row {row_number}: has fewer fields than the CSV header"
                )
            if not any((value or "").strip() for value in row.values()):
                continue
            schema = row["schema"].strip()
            if schema != TRACE_SCHEMA:
                raise TraceAggregationError(
                    f"row {row_number}: unsupported trace schema {schema!r}"
                )
            event_index = _parse_index(row["event_index"].strip(), row_number)
            time_s = _parse_nonnegative_float(
                row["time_s"].strip(), "time_s", row_number
            )
            if event_index <= previous_index:
                raise TraceAggregationError(
                    f"row {row_number}: event_index is not strictly increasing"
                )
            if time_s < previous_time:
                raise TraceAggregationError(
                    f"row {row_number}: time_s decreases from the previous row"
                )
            previous_index = event_index
            previous_time = time_s
            event = row["event"].strip()
            if not event:
                raise TraceAggregationError(f"row {row_number}: event is empty")
            event_count += 1
            yield row_number, time_s, row
        if event_count == 0:
            raise TraceAggregationError("event trace contains no event rows")


def derive_series(
    path: Path,
    scenario: str,
    bucket_width_s: float,
    stop_time_s: float,
    size_exclusion_bits: int = 0,
) -> tuple[list[dict[str, str]], dict[str, object]]:
    """Derive canonical rows and provenance details from one trace."""

    if not scenario.strip():
        raise TraceAggregationError("--scenario must not be empty")
    if (
        not isinstance(size_exclusion_bits, int)
        or isinstance(size_exclusion_bits, bool)
        or size_exclusion_bits < 0
    ):
        raise TraceAggregationError(
            "legacy trace size adjustment must be a nonnegative integer"
        )
    bucket_count = _validate_window(bucket_width_s, stop_time_s)
    input_sha256 = digest(path)

    sent_packets = [0] * bucket_count
    sent_bits = [0] * bucket_count
    packet_sizes: list[list[float]] = [[] for _ in range(bucket_count)]
    received_packets = [0] * bucket_count
    received_bits = [0] * bucket_count
    ecc_drops = [0] * bucket_count
    delays: list[list[float]] = [[] for _ in range(bucket_count)]
    statistic_samples: dict[str, list[list[float]]] = {
        name: [[] for _ in range(bucket_count)]
        for name in STATISTIC_SAMPLE_SEMANTICS
    }
    active_sample_statistics: set[str] = set()
    statistic_sample_counts: Counter[str] = Counter()
    in_window_statistic_sample_counts: Counter[str] = Counter()
    statistic_samples_by_node: Counter[tuple[str, str]] = Counter()
    in_window_statistic_samples_by_node: Counter[tuple[str, str]] = Counter()
    statistic_samples_excluded_at_stop: Counter[str] = Counter()
    statistic_samples_excluded_after_stop: Counter[str] = Counter()
    pending_exact: dict[
        tuple[str, str], dict[str, tuple[float, int, int]]
    ] = defaultdict(dict)
    exact_send_history: dict[
        tuple[str, str, str], tuple[float, int, int]
    ] = {}
    pending_fifo: dict[
        tuple[str, str], deque[tuple[float, int, int]]
    ] = defaultdict(deque)
    all_event_counts: Counter[str] = Counter()
    in_window_event_counts: Counter[str] = Counter()
    excluded_at_stop: Counter[str] = Counter()
    excluded_after_stop: Counter[str] = Counter()
    ignored_rx_drop_reasons: Counter[str] = Counter()
    unmatched_deliveries: Counter[tuple[str, str]] = Counter()
    matched_by_flow: Counter[tuple[str, str]] = Counter()
    matched_by_method: Counter[str] = Counter()
    dack_feedback_by_lineage: Counter[
        tuple[tuple[str, str, str], str, str, str]
    ] = Counter()
    qualifying_feedback_by_lineage: Counter[
        tuple[tuple[str, str, str], str, str, str]
    ] = Counter()
    source_ordered_pairs_by_lineage: dict[
        tuple[tuple[str, str, str], str, str, str],
        list[dict[str, int]],
    ] = defaultdict(list)
    dack_seen_by_lineage: set[
        tuple[tuple[str, str, str], str, str, str]
    ] = set()
    ack_closed_lineages: set[
        tuple[tuple[str, str, str], str, str, str]
    ] = set()
    invalidated_lineages: set[
        tuple[tuple[str, str, str], str, str, str]
    ] = set()
    proven_duplicate_budget: Counter[tuple[str, str, str]] = Counter()
    proven_duplicate_by_lineage: Counter[
        tuple[tuple[str, str, str], str, str, str]
    ] = Counter()
    consumed_duplicate_budget: Counter[tuple[str, str, str]] = Counter()
    pending_relay_enqueues: dict[
        tuple[tuple[str, str, str], str, str], tuple[int, int]
    ] = {}
    conflicting_relay_enqueues: set[
        tuple[tuple[str, str, str], str, str]
    ] = set()
    size_mismatches_by_flow: Counter[tuple[str, str]] = Counter()
    size_mismatch_examples: list[dict[str, object]] = []
    input_row_count = 0

    for row_number, time_s, row in _event_rows(path):
        input_row_count += 1
        event = row["event"].strip()
        all_event_counts[event] += 1
        parsed_statistic_sample: tuple[str, str, float] | None = None
        if event == STATISTIC_SAMPLE_EVENT:
            parsed_statistic_sample = _statistic_sample(row, row_number)
            statistic, node, _ = parsed_statistic_sample
            active_sample_statistics.add(statistic)
            statistic_sample_counts[statistic] += 1
            statistic_samples_by_node[(statistic, node)] += 1
        at_stop_boundary = _is_stop_boundary(
            time_s, bucket_width_s, bucket_count
        )
        if at_stop_boundary or time_s > stop_time_s:
            if event in {
                "app_send",
                "nwk_delivery",
                "hop_feedback",
                "rx_drop",
                STATISTIC_SAMPLE_EVENT,
            }:
                if at_stop_boundary:
                    excluded_at_stop[event] += 1
                    if parsed_statistic_sample is not None:
                        statistic_samples_excluded_at_stop[
                            parsed_statistic_sample[0]
                        ] += 1
                else:
                    excluded_after_stop[event] += 1
                    if parsed_statistic_sample is not None:
                        statistic_samples_excluded_after_stop[
                            parsed_statistic_sample[0]
                        ] += 1
            continue
        in_window_event_counts[event] += 1
        bucket = _bucket_index(time_s, bucket_width_s, bucket_count)

        if event == "app_send":
            flow = _flow_key(row, row_number)
            app_bits = _trace_packet_bits(row, row_number, size_exclusion_bits)
            sent_packets[bucket] += 1
            sent_bits[bucket] += app_bits
            packet_sizes[bucket].append(float(app_bits))
            sequence = (row.get("sequence") or "").strip()
            if sequence:
                packet_key = (flow[0], flow[1], sequence)
                if packet_key in exact_send_history:
                    raise TraceAggregationError(
                        f"row {row_number}: reused application sequence "
                        f"{sequence!r} for src={flow[0]} dst={flow[1]}"
                    )
                if sequence in pending_exact[flow]:
                    raise TraceAggregationError(
                        f"row {row_number}: duplicate pending application sequence "
                        f"{sequence!r} for src={flow[0]} dst={flow[1]}"
                    )
                send_record = (time_s, row_number, app_bits)
                pending_exact[flow][sequence] = send_record
                exact_send_history[packet_key] = send_record
            else:
                pending_fifo[flow].append((time_s, row_number, app_bits))
        elif event == "nwk_enqueue":
            relay_enqueue = _relay_enqueue_key(row, row_number)
            if relay_enqueue is not None:
                if relay_enqueue in pending_relay_enqueues:
                    conflicting_relay_enqueues.add(relay_enqueue)
                pending_relay_enqueues[relay_enqueue] = (
                    _parse_nonnegative_integer(
                        row["event_index"], "event_index", row_number
                    ),
                    row_number,
                )
        elif event == "nwk_delivery":
            flow = _flow_key(row, row_number)
            app_bits = _trace_packet_bits(row, row_number, size_exclusion_bits)
            received_packets[bucket] += 1
            received_bits[bucket] += app_bits
            sequence = (row.get("sequence") or "").strip()
            matched: tuple[float, int, int] | None = None
            method = ""
            if sequence:
                matched = pending_exact[flow].pop(sequence, None)
                if matched is not None:
                    method = "exact_sequence"
                else:
                    packet_key = (flow[0], flow[1], sequence)
                    if (
                        consumed_duplicate_budget[packet_key]
                        < proven_duplicate_budget[packet_key]
                        and packet_key in exact_send_history
                    ):
                        matched = exact_send_history[packet_key]
                        method = SOURCE_EXACT_DACK_RETRY_METHOD
                        consumed_duplicate_budget[packet_key] += 1
                if matched is None and not pending_exact[flow] and pending_fifo[flow]:
                    # A mixed/legacy producer may lack the key even when the
                    # delivery-side instrumentation provides one.
                    matched = pending_fifo[flow].popleft()
                    method = "fifo_fallback"
            else:
                exact_first: tuple[str, tuple[float, int, int]] | None = None
                if pending_exact[flow]:
                    first_sequence = next(iter(pending_exact[flow]))
                    exact_first = (
                        first_sequence,
                        pending_exact[flow][first_sequence],
                    )
                fifo_first = pending_fifo[flow][0] if pending_fifo[flow] else None
                if exact_first is not None and (
                    fifo_first is None or exact_first[1][0] <= fifo_first[0]
                ):
                    matched = pending_exact[flow].pop(exact_first[0])
                    method = "fifo_fallback"
                elif fifo_first is not None:
                    matched = pending_fifo[flow].popleft()
                    method = "fifo_fallback"

            if matched is not None:
                sent_time, sent_row_number, matched_send_bits = matched
                delay = time_s - sent_time
                if delay < 0.0:
                    raise TraceAggregationError(
                        f"row {row_number}: FIFO match produced a negative delay"
                    )
                delays[bucket].append(delay)
                matched_by_flow[flow] += 1
                matched_by_method[method] += 1
                if matched_send_bits != app_bits:
                    size_mismatches_by_flow[flow] += 1
                    if len(size_mismatch_examples) < 20:
                        size_mismatch_examples.append(
                            {
                                "src": flow[0],
                                "dst": flow[1],
                                "sequence": sequence or None,
                                "matching_method": method,
                                "send_row": sent_row_number,
                                "delivery_row": row_number,
                                "send_bits": matched_send_bits,
                                "delivery_bits": app_bits,
                            }
                        )
            else:
                unmatched_deliveries[flow] += 1
        elif event == "hop_feedback":
            feedback_enqueue_key = _feedback_enqueue_key(row, row_number)
            feedback = _retry_feedback_lineage(row, row_number)
            carried_lineage = _feedback_lineage_identity(row, row_number)
            if feedback is not None:
                lineage, reason = feedback
                enqueue_key = (lineage[0], lineage[1], lineage[2])
                matching_enqueue = pending_relay_enqueues.pop(enqueue_key, None)
                conflicted = enqueue_key in conflicting_relay_enqueues
                conflicting_relay_enqueues.discard(enqueue_key)
                if (
                    lineage[0] not in exact_send_history
                    or matching_enqueue is None
                    or conflicted
                ):
                    dack_seen_by_lineage.discard(lineage)
                    invalidated_lineages.add(lineage)
                    continue
                if (
                    lineage in ack_closed_lineages
                    or lineage in invalidated_lineages
                ):
                    continue
                source_ordered_pairs_by_lineage[lineage].append(
                    {
                        "enqueue_event_index": matching_enqueue[0],
                        "feedback_event_index": _parse_nonnegative_integer(
                            row["event_index"], "event_index", row_number
                        ),
                    }
                )
                if (
                    qualifying_feedback_by_lineage[lineage] > 0
                    and lineage in dack_seen_by_lineage
                ):
                    proven_duplicate_budget[lineage[0]] += 1
                    proven_duplicate_by_lineage[lineage] += 1
                qualifying_feedback_by_lineage[lineage] += 1
                if reason == "dack":
                    dack_feedback_by_lineage[lineage] += 1
                    dack_seen_by_lineage.add(lineage)
                else:
                    dack_seen_by_lineage.discard(lineage)
                    ack_closed_lineages.add(lineage)
            elif carried_lineage is not None:
                enqueue_key = (
                    carried_lineage[0],
                    carried_lineage[1],
                    carried_lineage[2],
                )
                pending_relay_enqueues.pop(enqueue_key, None)
                conflicting_relay_enqueues.discard(enqueue_key)
                dack_seen_by_lineage.discard(carried_lineage)
                invalidated_lineages.add(carried_lineage)
            elif feedback_enqueue_key is not None:
                pending_relay_enqueues.pop(feedback_enqueue_key, None)
                conflicting_relay_enqueues.discard(feedback_enqueue_key)
        elif event == "rx_drop":
            reason = row["reason"].strip().lower()
            # The historical statistic is registered by the OPNET ECC stage.
            # Earlier pipeline rejections explicitly do not update that counter.
            if reason == "ecc":
                ecc_drops[bucket] += 1
            else:
                ignored_rx_drop_reasons[reason or "<empty>"] += 1
        elif event == STATISTIC_SAMPLE_EVENT:
            if parsed_statistic_sample is None:  # pragma: no cover - invariant.
                raise AssertionError("statistic sample was not parsed")
            statistic, node, value = parsed_statistic_sample
            statistic_samples[statistic][bucket].append(value)
            in_window_statistic_sample_counts[statistic] += 1
            in_window_statistic_samples_by_node[(statistic, node)] += 1

    unmatched_sends: Counter[tuple[str, str]] = Counter()
    for flow in set(pending_exact) | set(pending_fifo):
        count = len(pending_exact[flow]) + len(pending_fifo[flow])
        if count:
            unmatched_sends[flow] = count

    source_file = path.name
    series_values: dict[str, list[float | None]] = {
        SENT_PACKETS_RATE: [value / bucket_width_s for value in sent_packets],
        SENT_BITS_RATE: [value / bucket_width_s for value in sent_bits],
        PACKET_SIZE: [
            (sum(samples) / len(samples)) if samples else None
            for samples in packet_sizes
        ],
        RECEIVED_PACKETS_RATE: [
            value / bucket_width_s for value in received_packets
        ],
        RECEIVED_PACKETS: [float(value) for value in received_packets],
        RECEIVED_BITS_RATE: [
            value / bucket_width_s for value in received_bits
        ],
        RECEIVED_BITS: [float(value) for value in received_bits],
        END_TO_END_DELAY: [
            (sum(samples) / len(samples)) if samples else None for samples in delays
        ],
        ECC_DROPS_RATE: [value / bucket_width_s for value in ecc_drops],
    }
    active_sample_semantics = tuple(
        (name, unit, aggregation)
        for name, unit, aggregation, _ in STATISTIC_SAMPLE_STATISTICS
        if name in active_sample_statistics
    )
    for statistic, _, _ in active_sample_semantics:
        series_values[statistic] = [
            (sum(samples) / len(samples)) if samples else None
            for samples in statistic_samples[statistic]
        ]
    output_statistics = STATISTICS + active_sample_semantics
    semantics = {
        name: (unit, aggregation)
        for name, unit, aggregation in output_statistics
    }
    rows: list[dict[str, str]] = []
    missing_sample_buckets: dict[str, list[float]] = defaultdict(list)
    for statistic, _, _ in output_statistics:
        unit, aggregation = semantics[statistic]
        for bucket, value in enumerate(series_values[statistic]):
            bucket_end_s = (bucket + 1) * bucket_width_s
            if value is None:
                missing_sample_buckets[statistic].append(bucket_end_s)
                formatted_value = ""
                value_status = "missing"
            else:
                formatted_value = _format_number(value)
                value_status = "observed"
            rows.append(
                {
                    "schema": SERIES_SCHEMA,
                    "scenario": scenario.strip(),
                    "statistic": statistic,
                    "time_s": _format_number(bucket_end_s),
                    "value": formatted_value,
                    "source": "ns3",
                    "unit": unit,
                    "aggregation": aggregation,
                    "raw_value": "",
                    "value_status": value_status,
                    "source_file": source_file,
                    "source_file_sha256": input_sha256,
                }
            )

    def flow_counts(counter: Counter[tuple[str, str]]) -> list[dict[str, object]]:
        return [
            {"src": source, "dst": destination, "count": count}
            for (source, destination), count in sorted(counter.items())
        ]

    def node_counts(
        counter: Counter[tuple[str, str]], statistic: str
    ) -> dict[str, int]:
        return {
            node: counter[(sample_statistic, node)]
            for sample_statistic, node in sorted(counter)
            if sample_statistic == statistic
        }

    repeated_dack_lineages = [
        {
            "src": packet[0],
            "dst": packet[1],
            "sequence": packet[2],
            "relay": relay,
            "ingress_peer": ingress_peer,
            "hop_sequence": hop_sequence,
            "qualifying_feedback_count": count,
            "dack_feedback_count": dack_feedback_by_lineage[lineage],
            "proven_extra_delivery_budget": proven_duplicate_by_lineage[lineage],
            "source_ordered_pairs": source_ordered_pairs_by_lineage[lineage],
        }
        for lineage, count in sorted(
            qualifying_feedback_by_lineage.items()
        )
        for packet, relay, ingress_peer, hop_sequence in [lineage]
        if proven_duplicate_by_lineage[lineage] > 0
    ]
    proven_duplicate_count = sum(proven_duplicate_budget.values())
    matched_duplicate_count = sum(consumed_duplicate_budget.values())

    statistic_sample_records = []
    for statistic, unit, aggregation in active_sample_semantics:
        integral = STATISTIC_SAMPLE_SEMANTICS[statistic][2]
        statistic_sample_records.append(
            {
                "statistic": statistic,
                "unit": unit,
                "aggregation": aggregation,
                "integral_samples_required": integral,
                "sample_count": statistic_sample_counts[statistic],
                "in_window_sample_count": in_window_statistic_sample_counts[
                    statistic
                ],
                "samples_by_node": node_counts(
                    statistic_samples_by_node, statistic
                ),
                "in_window_samples_by_node": node_counts(
                    in_window_statistic_samples_by_node, statistic
                ),
                "excluded_at_stop_count": statistic_samples_excluded_at_stop[
                    statistic
                ],
                "excluded_after_stop_count": statistic_samples_excluded_after_stop[
                    statistic
                ],
                "missing_sample_mean_bucket_ends_s": missing_sample_buckets.get(
                    statistic, []
                ),
            }
        )

    provenance: dict[str, object] = {
        "schema": PROVENANCE_SCHEMA,
        "output_schema": SERIES_SCHEMA,
        "source": "ns3",
        "scenario": scenario.strip(),
        "input": {
            "path": str(path.resolve()),
            "sha256": input_sha256,
            "size_bytes": path.stat().st_size,
            "trace_schema": TRACE_SCHEMA,
            "event_row_count": input_row_count,
            "event_counts": dict(sorted(all_event_counts.items())),
        },
        "window": {
            "start_time_s": 0.0,
            "stop_time_s": stop_time_s,
            "bucket_width_s": bucket_width_s,
            "bucket_count": bucket_count,
            "timestamp": "bucket_end",
            "interval": (
                "[previous_bucket_end, bucket_end); t=0 is included in the "
                "first bucket and t=stop_time_s is excluded"
            ),
            "start_endpoint": "inclusive",
            "stop_endpoint": "exclusive",
            "in_window_event_counts": dict(sorted(in_window_event_counts.items())),
            "excluded_at_stop": dict(sorted(excluded_at_stop.items())),
            "excluded_after_stop": dict(sorted(excluded_after_stop.items())),
        },
        "delay_matching": {
            "method": (
                "exact application sequence when present; FIFO per exact "
                "(src, dst) pair otherwise; an exact send timestamp is reused "
                "only for a source-exact duplicate proved by repeated "
                "first-reception DACK feedback"
            ),
            "matched_count": sum(matched_by_flow.values()),
            "matched_by_method": dict(sorted(matched_by_method.items())),
            "matched_by_flow": flow_counts(matched_by_flow),
            "unmatched_delivery_count": sum(unmatched_deliveries.values()),
            "unmatched_deliveries": flow_counts(unmatched_deliveries),
            "unmatched_send_count": sum(unmatched_sends.values()),
            "unmatched_sends": flow_counts(unmatched_sends),
            "missing_sample_mean_bucket_ends_s": missing_sample_buckets.get(
                END_TO_END_DELAY, []
            ),
        },
        "source_exact_dack_retry_duplicates": {
            "proof": (
                "a prior reason=dack, first_reception=1 feedback row followed "
                "by another qualifying first-reception ACK/DACK with equal "
                "(src,dst,app sequence,relay,ingress peer,HOP sequence); each "
                "row follows a matching relay enqueue and has a valid pre/post "
                "relay NSDP increment"
            ),
            "repeated_feedback_proof_count": proven_duplicate_count,
            "matched_duplicate_delivery_count": matched_duplicate_count,
            "unused_proof_count": (
                proven_duplicate_count - matched_duplicate_count
            ),
            "lineages": repeated_dack_lineages,
        },
        "matched_packet_size_integrity": {
            "compared_count": sum(matched_by_flow.values()),
            "mismatch_count": sum(size_mismatches_by_flow.values()),
            "mismatches_by_flow": flow_counts(size_mismatches_by_flow),
            "mismatch_examples": size_mismatch_examples,
            "example_limit": 20,
        },
        "ecc_drop_derivation": {
            "included_rx_drop_reason": "ecc",
            "ignored_rx_drop_reasons": dict(sorted(ignored_rx_drop_reasons.items())),
        },
        "statistic_sample_derivation": {
            "event": STATISTIC_SAMPLE_EVENT,
            "encoding": {
                "node_field": "node",
                "statistic_field": "statistic",
                "value_field": "value",
            },
            "allowlisted_statistics": [
                name for name, _, _, _ in STATISTIC_SAMPLE_STATISTICS
            ],
            "aggregation_semantics": (
                "arithmetic mean of every discrete source-ordered sample from "
                "all nodes in [previous_bucket_end, bucket_end); samples are "
                "not time weighted and per-node means are not averaged"
            ),
            "active_statistics": statistic_sample_records,
        },
        "statistics": [
            {"statistic": name, "unit": unit, "aggregation": aggregation}
            for name, unit, aggregation in output_statistics
        ],
        "opnet_app_size_semantics": {
            "trace_size_source": (
                "event size_bytes is the actual total br_Network packet size"
            ),
            "derived_statistic_bits": "size_bytes * 8 - size_exclusion_bits",
            "size_exclusion_bits": size_exclusion_bits,
            "size_exclusion_purpose": (
                "explicit compatibility adjustment for legacy traces that "
                "recorded the configured source size instead of the modeled "
                "br_Network size"
            ),
            "missing_sample_mean_bucket_ends_s": {
                statistic: bucket_ends
                for statistic, bucket_ends in sorted(missing_sample_buckets.items())
                if statistic in {PACKET_SIZE, END_TO_END_DELAY}
            },
        },
        "semantic_limits": [
            (
                "Legacy rows without a common application sequence use "
                "deterministic FIFO matching per exact source/destination pair."
            ),
            (
                "Runner-scheduled app_send flows are comparable to OPNET "
                "SEND_ONLY_TO_GATEWAY traffic only when scenario flow and "
                "gateway-discovery semantics are aligned."
            ),
            (
                "A missing delay sample is marked missing, never replaced with "
                "a fabricated zero."
            ),
            (
                "Current runner traces record the actual total br_Network "
                "packet size, so no post-hoc size exclusion is applied by "
                "default; any legacy adjustment is explicit in provenance."
            ),
        ],
        "output": {"row_count": len(rows)},
    }
    return rows, provenance


def write_outputs(
    rows: Iterable[dict[str, str]],
    provenance: dict[str, object],
    output_path: Path,
    provenance_path: Path,
) -> None:
    """Write canonical CSV and its machine-readable provenance record."""

    output_path.parent.mkdir(parents=True, exist_ok=True)
    provenance_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=OUTPUT_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    provenance["output"] = {
        **dict(provenance.get("output", {})),
        "path": str(output_path),
        "sha256": digest(output_path),
        "size_bytes": output_path.stat().st_size,
    }
    provenance_path.write_text(
        json.dumps(provenance, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trace", type=Path, help="csr-differential-trace-v1 CSV")
    parser.add_argument("output", type=Path, help="canonical aggregate-series CSV")
    parser.add_argument("--scenario", required=True, help="stable scenario identity")
    parser.add_argument(
        "--bucket-width",
        required=True,
        type=float,
        help="bucket width in simulation seconds",
    )
    parser.add_argument(
        "--stop-time",
        required=True,
        type=float,
        help="exclusive aggregate-window stop time in simulation seconds",
    )
    parser.add_argument(
        "--legacy-trace-size-exclusion-bits",
        "--opnet-size-exclusion-bits",
        dest="size_exclusion_bits",
        type=int,
        default=0,
        help=(
            "compatibility-only bits subtracted from legacy trace size_bytes "
            "(default: 0; --opnet-size-exclusion-bits is a retained alias)"
        ),
    )
    parser.add_argument(
        "--provenance",
        type=Path,
        help="provenance JSON (default: OUTPUT.provenance.json)",
    )
    parser.add_argument(
        "--require-zero-size-mismatches",
        action="store_true",
        help=(
            "fail after writing evidence if a matched app_send/nwk_delivery "
            "pair reports different packet sizes"
        ),
    )
    return parser


def main() -> None:
    arguments = build_parser().parse_args()
    raw_provenance_path = arguments.provenance or Path(
        str(arguments.output) + ".provenance.json"
    )
    trace_path = arguments.trace.resolve()
    output_path = arguments.output.resolve()
    provenance_path = raw_provenance_path.resolve()
    try:
        _validate_cli_paths(trace_path, output_path, provenance_path)
        rows, provenance = derive_series(
            trace_path,
            arguments.scenario,
            arguments.bucket_width,
            arguments.stop_time,
            arguments.size_exclusion_bits,
        )
        write_outputs(rows, provenance, output_path, provenance_path)
        mismatch_count = provenance["matched_packet_size_integrity"][
            "mismatch_count"
        ]
        if arguments.require_zero_size_mismatches and mismatch_count:
            raise TraceAggregationError(
                f"{mismatch_count} matched app_send/nwk_delivery packet-size "
                "mismatch(es); see provenance"
            )
    except (OSError, TraceAggregationError) as error:
        raise SystemExit(f"error: {error}") from error
    print(
        f"wrote {len(rows)} aggregate rows to {output_path} "
        f"with provenance {provenance_path}"
    )


if __name__ == "__main__":
    main()
