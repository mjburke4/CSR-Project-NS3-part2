#!/usr/bin/env python3
"""Focused tests for aggregate-ns3-trace.py."""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


UTILS = Path(__file__).resolve().parents[1]
SCRIPT = UTILS / "aggregate-ns3-trace.py"


def load_script():
    spec = importlib.util.spec_from_file_location("aggregate_ns3_trace", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


aggregate = load_script()

TRACE_COLUMNS = (
    "schema",
    "event_index",
    "time_s",
    "event",
    "src",
    "dst",
    "sequence",
    "size_bytes",
    "reason",
    "node",
    "statistic",
    "value",
)

LEGACY_TRACE_COLUMNS = TRACE_COLUMNS[:-3]


def trace_row(index: int, time_s: float, event: str, **values: str) -> dict[str, str]:
    row = {
        "schema": aggregate.TRACE_SCHEMA,
        "event_index": str(index),
        "time_s": str(time_s),
        "event": event,
        "src": "",
        "dst": "",
        "sequence": "",
        "size_bytes": "",
        "reason": "",
        "node": "",
        "statistic": "",
        "value": "",
    }
    row.update(values)
    return row


def write_trace(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_legacy_trace(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=LEGACY_TRACE_COLUMNS, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(
            {column: row[column] for column in LEGACY_TRACE_COLUMNS} for row in rows
        )


def run_cli(trace: Path, output: Path, provenance: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            str(trace),
            str(output),
            "--scenario",
            "path-safety",
            "--bucket-width",
            "5",
            "--stop-time",
            "5",
            "--provenance",
            str(provenance),
        ],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


class AggregateNs3TraceTests(unittest.TestCase):
    def test_pools_allowlisted_statistic_samples_and_preserves_missing_buckets(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(
                        0,
                        0,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.HOP_RESEND_QUEUE_SIZE,
                        value="0",
                    ),
                    trace_row(
                        1,
                        1,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.MAC_ACK_QUEUE_SIZE,
                        value="1",
                    ),
                    trace_row(
                        2,
                        1,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.MAC_TX_QUEUE_DELAY,
                        value="0.25",
                    ),
                    trace_row(
                        3,
                        2,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="2",
                        statistic=aggregate.HOP_RESEND_QUEUE_SIZE,
                        value="2",
                    ),
                    # A duplicate value/time remains an independent source sample.
                    trace_row(
                        4,
                        2,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.HOP_RESEND_QUEUE_SIZE,
                        value="4",
                    ),
                    trace_row(
                        5,
                        4,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="2",
                        statistic=aggregate.MAC_ACK_QUEUE_SIZE,
                        value="3",
                    ),
                    trace_row(
                        6,
                        4,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="2",
                        statistic=aggregate.MAC_TX_QUEUE_DELAY,
                        value="0.75",
                    ),
                    # An exact boundary belongs to the bucket starting there.
                    trace_row(
                        7,
                        5,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.HOP_RESEND_QUEUE_SIZE,
                        value="8",
                    ),
                    trace_row(
                        8,
                        6,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="3",
                        statistic=aggregate.MAC_TX_QUEUE_SIZE,
                        value="7",
                    ),
                    # The explicit stop and later samples activate their series but
                    # do not contribute to a bucket.
                    trace_row(
                        9,
                        10,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.MAC_TX_QUEUE_DELAY,
                        value="99",
                    ),
                    trace_row(
                        10,
                        11,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="3",
                        statistic=aggregate.MAC_TX_QUEUE_SIZE,
                        value="9",
                    ),
                ],
            )

            rows, provenance = aggregate.derive_series(
                trace, "queue-samples", 5.0, 10.0
            )
            values = {
                (row["statistic"], float(row["time_s"])): row for row in rows
            }

            self.assertEqual(len(rows), 26)
            self.assertEqual(
                values[(aggregate.HOP_RESEND_QUEUE_SIZE, 5.0)]["value"], "2"
            )
            self.assertEqual(
                values[(aggregate.HOP_RESEND_QUEUE_SIZE, 10.0)]["value"], "8"
            )
            self.assertEqual(
                values[(aggregate.MAC_ACK_QUEUE_SIZE, 5.0)]["value"], "2"
            )
            self.assertEqual(
                values[(aggregate.MAC_ACK_QUEUE_SIZE, 10.0)]["value_status"],
                "missing",
            )
            self.assertEqual(
                values[(aggregate.MAC_TX_QUEUE_SIZE, 5.0)]["value_status"],
                "missing",
            )
            self.assertEqual(
                values[(aggregate.MAC_TX_QUEUE_SIZE, 10.0)]["value"], "7"
            )
            self.assertEqual(
                values[(aggregate.MAC_TX_QUEUE_DELAY, 5.0)]["value"], "0.5"
            )
            self.assertEqual(
                values[(aggregate.MAC_TX_QUEUE_DELAY, 10.0)]["value_status"],
                "missing",
            )
            for statistic in (
                aggregate.HOP_RESEND_QUEUE_SIZE,
                aggregate.MAC_ACK_QUEUE_SIZE,
                aggregate.MAC_TX_QUEUE_SIZE,
            ):
                self.assertEqual(values[(statistic, 5.0)]["unit"], "packets")
                self.assertEqual(
                    values[(statistic, 5.0)]["aggregation"],
                    "bucket_sample_mean",
                )
            self.assertEqual(
                values[(aggregate.MAC_TX_QUEUE_DELAY, 5.0)]["unit"], "s"
            )

            details = {
                record["statistic"]: record
                for record in provenance["statistic_sample_derivation"][
                    "active_statistics"
                ]
            }
            self.assertEqual(
                details[aggregate.HOP_RESEND_QUEUE_SIZE]["sample_count"], 4
            )
            self.assertEqual(
                details[aggregate.HOP_RESEND_QUEUE_SIZE]["samples_by_node"],
                {"1": 3, "2": 1},
            )
            self.assertEqual(
                details[aggregate.MAC_ACK_QUEUE_SIZE][
                    "missing_sample_mean_bucket_ends_s"
                ],
                [10.0],
            )
            self.assertEqual(
                details[aggregate.MAC_TX_QUEUE_DELAY]["sample_count"], 3
            )
            self.assertEqual(
                details[aggregate.MAC_TX_QUEUE_DELAY]["in_window_sample_count"],
                2,
            )
            self.assertEqual(
                details[aggregate.MAC_TX_QUEUE_DELAY]["excluded_at_stop_count"],
                1,
            )
            self.assertEqual(
                details[aggregate.MAC_TX_QUEUE_SIZE]["excluded_after_stop_count"],
                1,
            )
            self.assertEqual(
                provenance["window"]["excluded_at_stop"],
                {aggregate.STATISTIC_SAMPLE_EVENT: 1},
            )
            self.assertEqual(
                provenance["window"]["excluded_after_stop"],
                {aggregate.STATISTIC_SAMPLE_EVENT: 1},
            )

    def test_out_of_window_sample_activates_an_all_missing_series(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(
                        0,
                        5,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="7",
                        statistic=aggregate.MAC_ACK_QUEUE_SIZE,
                        value="1",
                    )
                ],
            )

            rows, provenance = aggregate.derive_series(
                trace, "outside-only", 5.0, 5.0
            )
            ack = [
                row
                for row in rows
                if row["statistic"] == aggregate.MAC_ACK_QUEUE_SIZE
            ]

            self.assertEqual(len(rows), 10)
            self.assertEqual(len(ack), 1)
            self.assertEqual(ack[0]["value"], "")
            self.assertEqual(ack[0]["value_status"], "missing")
            detail = provenance["statistic_sample_derivation"][
                "active_statistics"
            ][0]
            self.assertEqual(detail["sample_count"], 1)
            self.assertEqual(detail["in_window_sample_count"], 0)
            self.assertEqual(detail["excluded_at_stop_count"], 1)

    def test_pools_source_exact_nwk_queue_size_and_delay_samples(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(
                        0,
                        0,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.NWK_QUEUE_SIZE,
                        value="0",
                    ),
                    trace_row(
                        1,
                        1,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.NWK_QUEUE_DELAY,
                        value="0.25",
                    ),
                    trace_row(
                        2,
                        2,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="2",
                        statistic=aggregate.NWK_QUEUE_SIZE,
                        value="4",
                    ),
                    trace_row(
                        3,
                        4,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="2",
                        statistic=aggregate.NWK_QUEUE_DELAY,
                        value="0.75",
                    ),
                    trace_row(
                        4,
                        5,
                        aggregate.STATISTIC_SAMPLE_EVENT,
                        node="1",
                        statistic=aggregate.NWK_QUEUE_SIZE,
                        value="3",
                    ),
                ],
            )

            rows, provenance = aggregate.derive_series(
                trace, "nwk-queue-samples", 5.0, 10.0
            )
            values = {
                (row["statistic"], float(row["time_s"])): row for row in rows
            }

            self.assertEqual(len(rows), 22)
            self.assertEqual(values[(aggregate.NWK_QUEUE_SIZE, 5.0)]["value"], "2")
            self.assertEqual(values[(aggregate.NWK_QUEUE_SIZE, 10.0)]["value"], "3")
            self.assertEqual(values[(aggregate.NWK_QUEUE_DELAY, 5.0)]["value"], "0.5")
            self.assertEqual(
                values[(aggregate.NWK_QUEUE_DELAY, 10.0)]["value_status"],
                "missing",
            )
            self.assertEqual(
                values[(aggregate.NWK_QUEUE_SIZE, 5.0)]["unit"], "packets"
            )
            self.assertEqual(values[(aggregate.NWK_QUEUE_DELAY, 5.0)]["unit"], "s")
            self.assertEqual(
                values[(aggregate.NWK_QUEUE_SIZE, 5.0)]["aggregation"],
                "bucket_sample_mean",
            )
            self.assertEqual(
                values[(aggregate.NWK_QUEUE_DELAY, 5.0)]["aggregation"],
                "bucket_sample_mean",
            )

            details = {
                record["statistic"]: record
                for record in provenance["statistic_sample_derivation"][
                    "active_statistics"
                ]
            }
            self.assertTrue(
                details[aggregate.NWK_QUEUE_SIZE]["integral_samples_required"]
            )
            self.assertFalse(
                details[aggregate.NWK_QUEUE_DELAY]["integral_samples_required"]
            )
            self.assertEqual(
                details[aggregate.NWK_QUEUE_SIZE]["samples_by_node"],
                {"1": 2, "2": 1},
            )
            self.assertEqual(
                details[aggregate.NWK_QUEUE_DELAY][
                    "missing_sample_mean_bucket_ends_s"
                ],
                [10.0],
            )

    def test_rejects_malformed_or_unsupported_statistic_samples(self) -> None:
        cases = (
            ({"node": "1", "statistic": "MAC.Unknown", "value": "1"}, "unsupported"),
            (
                {
                    "node": "1.5",
                    "statistic": aggregate.MAC_ACK_QUEUE_SIZE,
                    "value": "1",
                },
                "node is not an integer",
            ),
            (
                {
                    "node": "1",
                    "statistic": aggregate.MAC_ACK_QUEUE_SIZE,
                    "value": "1.5",
                },
                "integer packet count",
            ),
            (
                {
                    "node": "1",
                    "statistic": aggregate.MAC_TX_QUEUE_DELAY,
                    "value": "-0.1",
                },
                "finite and nonnegative",
            ),
            (
                {
                    "node": "1",
                    "statistic": aggregate.MAC_TX_QUEUE_DELAY,
                    "value": "nan",
                },
                "finite and nonnegative",
            ),
            (
                {
                    "node": "1",
                    "statistic": aggregate.NWK_QUEUE_SIZE,
                    "value": "1.5",
                },
                "integer packet count",
            ),
            (
                {
                    "node": "1",
                    "statistic": aggregate.NWK_QUEUE_DELAY,
                    "value": "-0.1",
                },
                "finite and nonnegative",
            ),
            (
                {
                    "node": "1",
                    "statistic": aggregate.NWK_QUEUE_DELAY,
                    "value": "inf",
                },
                "finite and nonnegative",
            ),
        )
        for fields, message in cases:
            with self.subTest(fields=fields):
                with tempfile.TemporaryDirectory() as temporary:
                    trace = Path(temporary) / "trace.csv"
                    write_trace(
                        trace,
                        [
                            trace_row(
                                0,
                                1,
                                aggregate.STATISTIC_SAMPLE_EVENT,
                                **fields,
                            )
                        ],
                    )
                    with self.assertRaisesRegex(
                        aggregate.TraceAggregationError, message
                    ):
                        aggregate.derive_series(trace, "malformed", 5.0, 5.0)

    def test_legacy_header_without_statistic_columns_remains_supported(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "legacy.csv"
            write_legacy_trace(
                trace,
                [trace_row(0, 1, "app_send", src="1", dst="2", size_bytes="10")],
            )

            rows, provenance = aggregate.derive_series(
                trace, "legacy-header", 5.0, 5.0
            )

            self.assertEqual(len(rows), 9)
            self.assertEqual(
                provenance["statistic_sample_derivation"]["active_statistics"], []
            )

    def test_derives_bucketed_counts_fifo_delay_and_ecc_drops(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(0, 0, "app_send", src="1", dst="2", size_bytes="10"),
                    # This delivery has no matching send but still contributes to
                    # the received-traffic counter.
                    trace_row(1, 3, "nwk_delivery", src="9", dst="8", size_bytes="10"),
                    trace_row(2, 5, "app_send", src="1", dst="2", size_bytes="20"),
                    trace_row(3, 7, "nwk_delivery", src="1", dst="2", size_bytes="10"),
                    # Exact bucket boundaries belong to the bucket starting there.
                    trace_row(4, 10, "app_send", src="1", dst="2", size_bytes="30"),
                    trace_row(5, 10, "rx_drop", src="1", dst="2", reason="ecc"),
                    trace_row(6, 11, "rx_drop", src="1", dst="2", reason="collision"),
                    trace_row(7, 12, "nwk_delivery", src="1", dst="2", size_bytes="20"),
                    # Explicit stop excludes later relevant events and records them.
                    trace_row(8, 21, "app_send", src="1", dst="2", size_bytes="40"),
                ],
            )

            rows, provenance = aggregate.derive_series(trace, "two-node", 10.0, 20.0)
            values = {
                (row["statistic"], float(row["time_s"])): row for row in rows
            }

            self.assertEqual(len(rows), 18)
            self.assertEqual(
                values[(aggregate.SENT_PACKETS_RATE, 10.0)]["value"],
                "0.20000000000000001",
            )
            self.assertEqual(
                values[(aggregate.SENT_PACKETS_RATE, 20.0)]["value"],
                "0.10000000000000001",
            )
            self.assertEqual(
                values[(aggregate.RECEIVED_PACKETS_RATE, 10.0)]["value"],
                "0.20000000000000001",
            )
            self.assertEqual(
                values[(aggregate.RECEIVED_PACKETS, 10.0)]["value"], "2"
            )
            self.assertEqual(
                values[(aggregate.RECEIVED_PACKETS, 20.0)]["value"], "1"
            )
            self.assertEqual(
                values[(aggregate.SENT_BITS_RATE, 10.0)]["value"],
                "24",
            )
            self.assertEqual(
                values[(aggregate.SENT_BITS_RATE, 20.0)]["value"],
                "24",
            )
            self.assertEqual(values[(aggregate.PACKET_SIZE, 10.0)]["value"], "120")
            self.assertEqual(values[(aggregate.PACKET_SIZE, 20.0)]["value"], "240")
            self.assertEqual(
                values[(aggregate.RECEIVED_BITS_RATE, 10.0)]["value"],
                "16",
            )
            self.assertEqual(
                values[(aggregate.RECEIVED_BITS, 10.0)]["value"], "160"
            )
            self.assertEqual(
                values[(aggregate.RECEIVED_BITS, 20.0)]["value"], "160"
            )
            self.assertEqual(values[(aggregate.END_TO_END_DELAY, 10.0)]["value"], "7")
            self.assertEqual(values[(aggregate.END_TO_END_DELAY, 20.0)]["value"], "7")
            self.assertEqual(
                values[(aggregate.ECC_DROPS_RATE, 10.0)]["value"],
                "0",
            )
            self.assertEqual(
                values[(aggregate.ECC_DROPS_RATE, 20.0)]["value"],
                "0.10000000000000001",
            )

            self.assertEqual(provenance["delay_matching"]["matched_count"], 2)
            self.assertEqual(
                provenance["delay_matching"]["unmatched_delivery_count"], 1
            )
            self.assertEqual(provenance["delay_matching"]["unmatched_send_count"], 1)
            self.assertEqual(
                provenance["ecc_drop_derivation"]["ignored_rx_drop_reasons"],
                {"collision": 1},
            )
            self.assertEqual(
                provenance["window"]["excluded_after_stop"], {"app_send": 1}
            )
            self.assertEqual(
                provenance["matched_packet_size_integrity"],
                {
                    "compared_count": 2,
                    "example_limit": 20,
                    "mismatch_count": 0,
                    "mismatch_examples": [],
                    "mismatches_by_flow": [],
                },
            )
            self.assertEqual(
                provenance["opnet_app_size_semantics"]["size_exclusion_bits"], 0
            )

    def test_modeler_bucket_endpoint_is_left_closed_and_stop_is_excluded(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(0, 0, "app_send", src="1", dst="2", size_bytes="10"),
                    trace_row(1, 300, "app_send", src="1", dst="2", size_bytes="20"),
                    trace_row(2, 600, "app_send", src="1", dst="2", size_bytes="30"),
                    trace_row(3, 601, "app_send", src="1", dst="2", size_bytes="40"),
                ],
            )

            rows, provenance = aggregate.derive_series(
                trace, "bucket-endpoints", 60.0, 600.0
            )
            sent_rates = {
                float(row["time_s"]): row["value"]
                for row in rows
                if row["statistic"] == aggregate.SENT_PACKETS_RATE
            }
            packet_sizes = {
                float(row["time_s"]): row["value"]
                for row in rows
                if row["statistic"] == aggregate.PACKET_SIZE
                and row["value_status"] == "observed"
            }

            self.assertEqual(sent_rates[60.0], "0.016666666666666666")
            self.assertEqual(sent_rates[300.0], "0")
            self.assertEqual(sent_rates[360.0], "0.016666666666666666")
            self.assertEqual(packet_sizes, {60.0: "80", 360.0: "160"})
            self.assertEqual(
                provenance["window"]["interval"],
                "[previous_bucket_end, bucket_end); t=0 is included in the "
                "first bucket and t=stop_time_s is excluded",
            )
            self.assertEqual(provenance["window"]["start_endpoint"], "inclusive")
            self.assertEqual(provenance["window"]["stop_endpoint"], "exclusive")
            self.assertEqual(
                provenance["window"]["excluded_at_stop"], {"app_send": 1}
            )
            self.assertEqual(
                provenance["window"]["excluded_after_stop"], {"app_send": 1}
            )
            self.assertEqual(aggregate._bucket_index(0.0, 60.0, 10), 0)
            self.assertEqual(aggregate._bucket_index(300.0, 60.0, 10), 5)
            with self.assertRaisesRegex(
                aggregate.TraceAggregationError, "outside the half-open"
            ):
                aggregate._bucket_index(600.0, 60.0, 10)

    def test_prefers_exact_application_sequence_over_fifo(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(
                        0,
                        1,
                        "app_send",
                        src="1",
                        dst="2",
                        sequence="100",
                        size_bytes="10",
                    ),
                    trace_row(
                        1,
                        2,
                        "app_send",
                        src="1",
                        dst="2",
                        sequence="101",
                        size_bytes="10",
                    ),
                    # Delivery order differs from generation order.  Exact
                    # keys put delays 2 s and 11 s in their arrival buckets.
                    trace_row(
                        2,
                        4,
                        "nwk_delivery",
                        src="1",
                        dst="2",
                        sequence="101",
                        size_bytes="10",
                    ),
                    trace_row(
                        3,
                        12,
                        "nwk_delivery",
                        src="1",
                        dst="2",
                        sequence="100",
                        size_bytes="10",
                    ),
                ],
            )

            rows, provenance = aggregate.derive_series(trace, "sequenced", 10.0, 20.0)
            delays = {
                float(row["time_s"]): row["value"]
                for row in rows
                if row["statistic"] == aggregate.END_TO_END_DELAY
            }

            self.assertEqual(delays, {10.0: "2", 20.0: "11"})
            self.assertEqual(
                provenance["delay_matching"]["matched_by_method"],
                {"exact_sequence": 2},
            )

    def test_marks_empty_delay_bucket_missing_instead_of_zero(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [trace_row(0, 1, "app_send", src="1", dst="2", size_bytes="10")],
            )

            rows, provenance = aggregate.derive_series(trace, "no-delivery", 5.0, 10.0)
            delays = [row for row in rows if row["statistic"] == aggregate.END_TO_END_DELAY]

            self.assertEqual(len(delays), 2)
            self.assertEqual([row["value"] for row in delays], ["", ""])
            self.assertEqual(
                [row["value_status"] for row in delays], ["missing", "missing"]
            )
            self.assertEqual(
                provenance["delay_matching"]["missing_sample_mean_bucket_ends_s"],
                [5.0, 10.0],
            )

    def test_legacy_trace_size_adjustment_is_explicit_and_validated(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [trace_row(0, 1, "app_send", src="1", dst="2", size_bytes="10")],
            )

            rows, provenance = aggregate.derive_series(
                trace, "current-trace", 5.0, 5.0
            )
            packet_size = next(
                row for row in rows if row["statistic"] == aggregate.PACKET_SIZE
            )
            self.assertEqual(packet_size["value"], "80")
            self.assertEqual(
                provenance["opnet_app_size_semantics"]["size_exclusion_bits"], 0
            )

            legacy_rows, legacy_provenance = aggregate.derive_series(
                trace, "legacy-trace", 5.0, 5.0, size_exclusion_bits=64
            )
            legacy_packet_size = next(
                row
                for row in legacy_rows
                if row["statistic"] == aggregate.PACKET_SIZE
            )
            self.assertEqual(legacy_packet_size["value"], "16")
            self.assertEqual(
                legacy_provenance["opnet_app_size_semantics"][
                    "size_exclusion_bits"
                ],
                64,
            )
            self.assertIn(
                "compatibility adjustment",
                legacy_provenance["opnet_app_size_semantics"][
                    "size_exclusion_purpose"
                ],
            )

            with self.assertRaisesRegex(
                aggregate.TraceAggregationError, "too small"
            ):
                aggregate.derive_series(
                    trace, "bad-adjustment", 5.0, 5.0, size_exclusion_bits=81
                )

    def test_matched_packet_size_mismatch_is_evident_and_can_fail_cli(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            output = root / "series.csv"
            provenance_path = root / "provenance.json"
            write_trace(
                trace,
                [
                    trace_row(
                        0,
                        1,
                        "app_send",
                        src="1",
                        dst="2",
                        sequence="7",
                        size_bytes="10",
                    ),
                    trace_row(
                        1,
                        2,
                        "nwk_delivery",
                        src="1",
                        dst="2",
                        sequence="7",
                        size_bytes="9",
                    ),
                ],
            )

            rows, provenance = aggregate.derive_series(
                trace, "size-mismatch", 5.0, 5.0
            )
            self.assertEqual(len(rows), 9)
            integrity = provenance["matched_packet_size_integrity"]
            self.assertEqual(integrity["compared_count"], 1)
            self.assertEqual(integrity["mismatch_count"], 1)
            self.assertEqual(
                integrity["mismatches_by_flow"],
                [{"src": "1", "dst": "2", "count": 1}],
            )
            self.assertEqual(integrity["mismatch_examples"][0]["send_bits"], 80)
            self.assertEqual(
                integrity["mismatch_examples"][0]["delivery_bits"], 72
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(trace),
                    str(output),
                    "--scenario",
                    "size-mismatch",
                    "--bucket-width",
                    "5",
                    "--stop-time",
                    "5",
                    "--provenance",
                    str(provenance_path),
                    "--require-zero-size-mismatches",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn("packet-size mismatch", completed.stderr)
            self.assertTrue(output.is_file())
            written = json.loads(provenance_path.read_text(encoding="utf-8"))
            self.assertEqual(
                written["matched_packet_size_integrity"]["mismatch_count"], 1
            )

    def test_rejects_non_integral_window_and_disordered_trace(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(1, 2, "scenario_start"),
                    trace_row(0, 3, "scenario_stop"),
                ],
            )

            with self.assertRaisesRegex(
                aggregate.TraceAggregationError, "integer multiple"
            ):
                aggregate.derive_series(trace, "bad-window", 6.0, 10.0)
            with self.assertRaisesRegex(
                aggregate.TraceAggregationError, "strictly increasing"
            ):
                aggregate.derive_series(trace, "bad-order", 5.0, 10.0)

    def test_cli_writes_canonical_csv_and_hashed_provenance(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            output = root / "series.csv"
            provenance_path = root / "provenance.json"
            write_trace(
                trace,
                [
                    trace_row(0, 1, "app_send", src="1", dst="2", size_bytes="10"),
                    trace_row(1, 2, "nwk_delivery", src="1", dst="2", size_bytes="10"),
                ],
            )

            completed = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(trace),
                    str(output),
                    "--scenario",
                    "fixture",
                    "--bucket-width",
                    "5",
                    "--stop-time",
                    "10",
                    "--provenance",
                    str(provenance_path),
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            self.assertEqual(completed.returncode, 0, completed.stderr)

            with output.open("r", encoding="utf-8", newline="") as stream:
                reader = csv.DictReader(stream)
                output_rows = list(reader)
                self.assertEqual(tuple(reader.fieldnames or ()), aggregate.OUTPUT_COLUMNS)
            self.assertEqual(len(output_rows), 18)
            self.assertTrue(
                all(row["schema"] == aggregate.SERIES_SCHEMA for row in output_rows)
            )
            self.assertTrue(all(row["source"] == "ns3" for row in output_rows))

            record = json.loads(provenance_path.read_text(encoding="utf-8"))
            expected_hash = hashlib.sha256(output.read_bytes()).hexdigest()
            self.assertEqual(record["output"]["sha256"], expected_hash)
            self.assertEqual(
                record["input"]["sha256"],
                hashlib.sha256(trace.read_bytes()).hexdigest(),
            )

    def test_cli_rejects_colliding_paths_before_writes(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            write_trace(trace, [trace_row(0, 1, "scenario_start")])
            original = trace.read_bytes()

            aggregate_is_trace = run_cli(
                trace, trace, root / "aggregate-is-trace.provenance.json"
            )
            self.assertNotEqual(aggregate_is_trace.returncode, 0)
            self.assertIn(
                "aggregate output must not alias the input trace",
                aggregate_is_trace.stderr,
            )
            self.assertEqual(trace.read_bytes(), original)
            self.assertFalse(
                (root / "aggregate-is-trace.provenance.json").exists()
            )

            provenance_is_trace_output = root / "provenance-is-trace.csv"
            provenance_is_trace = run_cli(
                trace, provenance_is_trace_output, trace
            )
            self.assertNotEqual(provenance_is_trace.returncode, 0)
            self.assertIn(
                "provenance output must not alias the input trace",
                provenance_is_trace.stderr,
            )
            self.assertEqual(trace.read_bytes(), original)
            self.assertFalse(provenance_is_trace_output.exists())

            shared_output = root / "shared-output"
            outputs_collide = run_cli(trace, shared_output, shared_output)
            self.assertNotEqual(outputs_collide.returncode, 0)
            self.assertIn(
                "aggregate output must not alias the provenance output",
                outputs_collide.stderr,
            )
            self.assertEqual(trace.read_bytes(), original)
            self.assertFalse(shared_output.exists())

    def test_cli_rejects_symlink_alias_of_trace(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            write_trace(trace, [trace_row(0, 1, "scenario_start")])
            original = trace.read_bytes()
            alias = root / "trace-alias.csv"
            try:
                alias.symlink_to(trace)
            except OSError as error:
                self.skipTest(f"symlinks are unavailable: {error}")

            completed = run_cli(
                trace, alias, root / "symlink.provenance.json"
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn(
                "aggregate output must not alias the input trace",
                completed.stderr,
            )
            self.assertEqual(trace.read_bytes(), original)
            self.assertFalse((root / "symlink.provenance.json").exists())

    def test_cli_rejects_hardlink_alias_of_trace(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            write_trace(trace, [trace_row(0, 1, "scenario_start")])
            original = trace.read_bytes()
            alias = root / "trace-hardlink.csv"
            try:
                alias.hardlink_to(trace)
            except OSError as error:
                self.skipTest(f"hard links are unavailable: {error}")

            completed = run_cli(
                trace, alias, root / "hardlink.provenance.json"
            )
            self.assertNotEqual(completed.returncode, 0)
            self.assertIn(
                "aggregate output must not alias the input trace",
                completed.stderr,
            )
            self.assertEqual(trace.read_bytes(), original)
            self.assertFalse((root / "hardlink.provenance.json").exists())


if __name__ == "__main__":
    unittest.main()
