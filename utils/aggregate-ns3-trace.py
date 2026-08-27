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


def _bucket_index(time_s: float, bucket_width_s: float, bucket_count: int) -> int:
    """Map time to (previous bucket end, bucket end], with zero in bucket 0."""

    quotient = time_s / bucket_width_s
    nearest = round(quotient)
    if math.isclose(quotient, nearest, rel_tol=0.0, abs_tol=1.0e-12):
        quotient = float(nearest)
    index = max(0, math.ceil(quotient) - 1)
    return min(index, bucket_count - 1)


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
    pending_exact: dict[
        tuple[str, str], dict[str, tuple[float, int, int]]
    ] = defaultdict(dict)
    pending_fifo: dict[
        tuple[str, str], deque[tuple[float, int, int]]
    ] = defaultdict(deque)
    all_event_counts: Counter[str] = Counter()
    in_window_event_counts: Counter[str] = Counter()
    excluded_after_stop: Counter[str] = Counter()
    ignored_rx_drop_reasons: Counter[str] = Counter()
    unmatched_deliveries: Counter[tuple[str, str]] = Counter()
    matched_by_flow: Counter[tuple[str, str]] = Counter()
    matched_by_method: Counter[str] = Counter()
    size_mismatches_by_flow: Counter[tuple[str, str]] = Counter()
    size_mismatch_examples: list[dict[str, object]] = []
    input_row_count = 0

    for row_number, time_s, row in _event_rows(path):
        input_row_count += 1
        event = row["event"].strip()
        all_event_counts[event] += 1
        if time_s > stop_time_s:
            if event in {"app_send", "nwk_delivery", "rx_drop"}:
                excluded_after_stop[event] += 1
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
                if sequence in pending_exact[flow]:
                    raise TraceAggregationError(
                        f"row {row_number}: duplicate pending application sequence "
                        f"{sequence!r} for src={flow[0]} dst={flow[1]}"
                    )
                pending_exact[flow][sequence] = (time_s, row_number, app_bits)
            else:
                pending_fifo[flow].append((time_s, row_number, app_bits))
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
                elif not pending_exact[flow] and pending_fifo[flow]:
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
        elif event == "rx_drop":
            reason = row["reason"].strip().lower()
            # The historical statistic is registered by the OPNET ECC stage.
            # Earlier pipeline rejections explicitly do not update that counter.
            if reason == "ecc":
                ecc_drops[bucket] += 1
            else:
                ignored_rx_drop_reasons[reason or "<empty>"] += 1

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
    semantics = {
        name: (unit, aggregation) for name, unit, aggregation in STATISTICS
    }
    rows: list[dict[str, str]] = []
    missing_sample_buckets: dict[str, list[float]] = defaultdict(list)
    for statistic, _, _ in STATISTICS:
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
            "interval": "(previous_bucket_end, bucket_end], with t=0 in first bucket",
            "in_window_event_counts": dict(sorted(in_window_event_counts.items())),
            "excluded_after_stop": dict(sorted(excluded_after_stop.items())),
        },
        "delay_matching": {
            "method": (
                "exact application sequence when present; FIFO per exact "
                "(src, dst) pair otherwise"
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
        "statistics": [
            {"statistic": name, "unit": unit, "aggregation": aggregation}
            for name, unit, aggregation in STATISTICS
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
        help="inclusive simulation stop time in seconds",
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
