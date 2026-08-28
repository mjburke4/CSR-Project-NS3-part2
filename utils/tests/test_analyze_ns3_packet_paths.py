#!/usr/bin/env python3
"""Focused tests for analyze-ns3-packet-paths.py."""

from __future__ import annotations

import csv
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


UTILS = Path(__file__).resolve().parents[1]
SCRIPT = UTILS / "analyze-ns3-packet-paths.py"
AGGREGATE_SCRIPT = UTILS / "aggregate-ns3-trace.py"


def load_script():
    specification = importlib.util.spec_from_file_location(
        "analyze_ns3_packet_paths", SCRIPT
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {SCRIPT}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


analyzer = load_script()


def load_aggregate_script():
    specification = importlib.util.spec_from_file_location(
        "aggregate_ns3_trace_for_path_test", AGGREGATE_SCRIPT
    )
    if specification is None or specification.loader is None:
        raise RuntimeError(f"cannot load {AGGREGATE_SCRIPT}")
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


aggregate = load_aggregate_script()

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
    "reservation_slot",
    "reservation_counter",
    "detail",
    "statistic",
    "value",
)

COMPACT_TRACE_COLUMNS = (
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
    "peer",
    "next_hop",
    "route_cost",
    "detail",
)


def trace_row(index: int, time_s: float, event: str, **values: str) -> dict[str, str]:
    row = {column: "" for column in TRACE_COLUMNS}
    row.update(
        {
            "schema": analyzer.TRACE_SCHEMA,
            "event_index": str(index),
            "time_s": str(time_s),
            "event": event,
        }
    )
    row.update(values)
    return row


def packet_row(
    index: int,
    time_s: float,
    event: str,
    node: int,
    source: int = 0,
    destination: int = 2,
    sequence: int = 42,
    next_hop: int | None = None,
    detail: str = "",
) -> dict[str, str]:
    return trace_row(
        index,
        time_s,
        event,
        node=str(node),
        src=str(source),
        dst=str(destination),
        sequence=str(sequence),
        next_hop="" if next_hop is None else str(next_hop),
        detail=detail,
    )


def write_trace(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def write_compact_trace(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=COMPACT_TRACE_COLUMNS, lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(
            {column: row.get(column, "") for column in COMPACT_TRACE_COLUMNS}
            for row in rows
        )


class AnalyzePacketPathsTests(unittest.TestCase):
    def test_accepts_compact_aggregate_only_trace_header(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_compact_trace(
                trace,
                [
                    trace_row(
                        0,
                        0.0,
                        "route_change",
                        node="0",
                        dst="2",
                        next_hop="2",
                    ),
                    packet_row(1, 1.0, "app_send", 0),
                    packet_row(2, 1.1, "nwk_enqueue", 0),
                    packet_row(3, 1.2, "nwk_forward", 0, next_hop=2),
                    packet_row(4, 1.3, "nwk_delivery", 2),
                ],
            )

            packets, report = analyzer.analyze_trace(trace)

            self.assertTrue(report["pass"])
            self.assertEqual(packets[0].path, [0, 2])
            self.assertEqual(packets[0].route_context_mismatch_count, 0)

    def test_reconstructs_path_residence_transit_and_distributions(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(
                        0,
                        1.0,
                        "route_change",
                        node="0",
                        dst="2",
                        next_hop="1",
                        success="1",
                    ),
                    packet_row(1, 10.0, "app_send", 0),
                    packet_row(2, 10.1, "nwk_enqueue", 0),
                    packet_row(3, 11.0, "nwk_forward", 0, next_hop=1),
                    trace_row(
                        4,
                        11.1,
                        "route_change",
                        node="1",
                        dst="2",
                        next_hop="2",
                        success="1",
                    ),
                    # Same-destination churn at an unrelated node must not be
                    # attributed to this packet's path.
                    trace_row(
                        5,
                        11.2,
                        "route_change",
                        node="99",
                        dst="2",
                        next_hop="2",
                        success="1",
                    ),
                    packet_row(6, 11.5, "nwk_enqueue", 1),
                    packet_row(7, 12.0, "nwk_forward", 1, next_hop=2),
                    packet_row(8, 13.0, "nwk_delivery", 2),
                ],
            )

            packets, report = analyzer.analyze_trace(
                trace, startup_time_s=12.0, bucket_width_s=5.0
            )

            self.assertTrue(report["pass"])
            self.assertEqual(len(packets), 1)
            packet = packets[0]
            self.assertTrue(packet.valid)
            self.assertTrue(packet.complete)
            self.assertEqual(packet.path, [0, 1, 2])
            self.assertEqual(packet.hop_count, 2)
            self.assertAlmostEqual(packet.app_to_local_enqueue_s, 0.1)
            self.assertAlmostEqual(packet.total_nwk_residence_s, 1.4)
            self.assertAlmostEqual(packet.total_leg_transit_s, 1.5)
            self.assertAlmostEqual(packet.end_to_end_s, 3.0)
            self.assertAlmostEqual(packet.decomposition_error_s, 0.0)
            self.assertEqual(packet.route_change_count, 1)
            self.assertEqual(packet.route_context_mismatch_count, 0)
            self.assertEqual(report["overall"]["packet_count"], 1)
            self.assertEqual(report["post_startup"]["packet_count"], 1)
            self.assertEqual(
                report["path_distribution"],
                [
                    {
                        "path": [0, 1, 2],
                        "hop_count": 2,
                        "count": 1,
                        "post_startup_count": 1,
                    }
                ],
            )
            self.assertEqual(report["flow_distribution"][0]["src"], 0)
            self.assertEqual(report["flow_distribution"][0]["dst"], 2)
            self.assertEqual(report["end_to_end_buckets"][2]["packet_count"], 1)
            self.assertAlmostEqual(
                report["end_to_end_buckets"][2]["mean_end_to_end_s"], 3.0
            )

    def test_route_context_mismatch_is_a_strict_validation_error(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(
                        0,
                        1.0,
                        "route_change",
                        node="0",
                        dst="2",
                        next_hop="1",
                        success="1",
                    ),
                    packet_row(1, 2.0, "app_send", 0),
                    packet_row(2, 2.1, "nwk_enqueue", 0),
                    packet_row(3, 2.2, "nwk_forward", 0, next_hop=2),
                    packet_row(4, 2.3, "nwk_delivery", 2),
                ],
            )

            packets, report = analyzer.analyze_trace(trace)

            self.assertFalse(report["pass"])
            self.assertFalse(packets[0].valid)
            self.assertEqual(packets[0].route_context_mismatch_count, 1)
            self.assertEqual(report["issue_counts"]["route_context_mismatch"], 1)
            self.assertEqual(report["counts"]["route_context_mismatch_packets"], 1)
            self.assertEqual(report["counts"]["route_context_mismatches"], 1)

    def test_reverse_route_forward_is_not_compared_to_forward_route_state(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    trace_row(
                        0,
                        1.0,
                        "route_change",
                        node="0",
                        dst="2",
                        next_hop="1",
                        success="1",
                    ),
                    packet_row(1, 2.0, "app_send", 0),
                    packet_row(2, 2.1, "nwk_enqueue", 0),
                    packet_row(
                        3,
                        2.2,
                        "nwk_forward",
                        0,
                        next_hop=2,
                        detail="route=reverse",
                    ),
                    packet_row(4, 2.3, "nwk_delivery", 2),
                ],
            )

            packets, report = analyzer.analyze_trace(trace)

            self.assertTrue(report["pass"])
            self.assertTrue(packets[0].valid)
            self.assertEqual(packets[0].route_context_mismatch_count, 0)
            self.assertEqual(report["counts"]["route_context_mismatches"], 0)

    def test_flags_loop_duplicate_admission_and_next_hop_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    packet_row(0, 0.0, "app_send", 0),
                    packet_row(1, 0.1, "nwk_enqueue", 0),
                    packet_row(2, 0.2, "nwk_forward", 0, next_hop=9),
                    packet_row(3, 0.3, "nwk_enqueue", 1),
                    packet_row(4, 0.4, "nwk_forward", 1, next_hop=0),
                    packet_row(5, 0.5, "nwk_enqueue", 0),
                    packet_row(6, 0.6, "nwk_forward", 0, next_hop=2),
                    packet_row(7, 0.7, "nwk_delivery", 2),
                ],
            )

            packets, report = analyzer.analyze_trace(trace)

            self.assertFalse(report["pass"])
            self.assertFalse(packets[0].valid)
            self.assertTrue(packets[0].complete)
            self.assertEqual(packets[0].path, [0, 1, 0, 2])
            codes = {issue["code"] for issue in packets[0].issues}
            self.assertIn("duplicate_local_admission", codes)
            self.assertIn("routing_loop", codes)
            self.assertIn("next_hop_mismatch", codes)
            self.assertEqual(report["counts"]["invalid_packets"], 1)
            self.assertEqual(report["counts"]["incomplete_delivered_packets"], 0)

    def test_delivered_chain_with_missing_forward_is_incomplete_and_invalid(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    packet_row(0, 1.0, "app_send", 0),
                    packet_row(1, 1.1, "nwk_enqueue", 0),
                    packet_row(2, 1.2, "nwk_delivery", 2),
                ],
            )

            packets, report = analyzer.analyze_trace(trace)

            self.assertFalse(report["pass"])
            self.assertFalse(packets[0].valid)
            self.assertFalse(packets[0].complete)
            self.assertEqual(report["counts"]["incomplete_delivered_packets"], 1)
            self.assertEqual(report["overall"]["packet_count"], 0)

    def test_valid_undelivered_prefix_is_reported_but_passes_strict_acceptance(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            summary = root / "summary.json"
            packets_csv = root / "packets.csv"
            write_trace(
                trace,
                [
                    packet_row(0, 1.0, "app_send", 0),
                    packet_row(1, 1.1, "nwk_enqueue", 0),
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(trace),
                    "--summary-json",
                    str(summary),
                    "--packet-csv",
                    str(packets_csv),
                    "--strict",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            report = json.loads(summary.read_text(encoding="utf-8"))
            self.assertTrue(report["pass"])
            self.assertEqual(report["counts"]["incomplete_packets"], 1)
            self.assertEqual(report["counts"]["undelivered_packets"], 1)
            self.assertEqual(report["counts"]["invalid_packets"], 0)
            with packets_csv.open("r", encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual(rows[0]["status"], "incomplete_undelivered")
            self.assertEqual(rows[0]["valid"], "1")

    def test_inconsistent_undelivered_prefix_fails_validation(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    packet_row(0, 1.0, "app_send", 0),
                    packet_row(1, 1.1, "nwk_enqueue", 0),
                    packet_row(2, 1.2, "nwk_forward", 0, next_hop=9),
                    packet_row(3, 1.3, "nwk_enqueue", 1),
                ],
            )

            packets, report = analyzer.analyze_trace(trace)

            self.assertFalse(report["pass"])
            self.assertFalse(packets[0].valid)
            self.assertFalse(packets[0].complete)
            self.assertEqual(packets[0].path, [0, 1])
            self.assertIn(
                "next_hop_mismatch",
                {issue["code"] for issue in packets[0].issues},
            )

    def test_destination_enqueue_cannot_replace_synchronous_delivery(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    packet_row(0, 1.0, "app_send", 0),
                    packet_row(1, 1.1, "nwk_enqueue", 0),
                    packet_row(2, 1.2, "nwk_forward", 0, next_hop=2),
                    packet_row(3, 1.3, "nwk_enqueue", 2),
                ],
            )

            packets, report = analyzer.analyze_trace(trace)

            self.assertFalse(report["pass"])
            self.assertIn(
                "destination_queue_admission",
                {issue["code"] for issue in packets[0].issues},
            )

    def test_bucket_boundary_matches_aggregate_and_stop_is_exclusive(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    packet_row(0, 55.0, "app_send", 0),
                    packet_row(1, 55.0, "nwk_enqueue", 0),
                    packet_row(2, 56.0, "nwk_forward", 0, next_hop=2),
                    packet_row(3, 60.0, "nwk_delivery", 2),
                ],
            )

            _, unbounded = analyzer.analyze_trace(trace, bucket_width_s=60.0)
            self.assertEqual(len(unbounded["end_to_end_buckets"]), 2)
            self.assertEqual(unbounded["end_to_end_buckets"][0]["packet_count"], 0)
            self.assertEqual(unbounded["end_to_end_buckets"][1]["packet_count"], 1)

            _, stopped = analyzer.analyze_trace(
                trace, bucket_width_s=60.0, stop_time_s=60.0
            )
            self.assertEqual(len(stopped["end_to_end_buckets"]), 1)
            self.assertEqual(stopped["end_to_end_buckets"][0]["packet_count"], 0)
            self.assertEqual(stopped["counts"]["end_to_end_bucket_packets"], 0)
            self.assertEqual(
                stopped["counts"]["end_to_end_bucket_excluded_at_stop_packets"],
                1,
            )
            self.assertEqual(stopped["overall"]["packet_count"], 1)

            # The aggregate mapper snaps quotient noise at a boundary.  The
            # stop classifier must use the same rule instead of silently
            # counting this packet in-window and dropping it from all buckets.
            rows = [
                packet_row(0, 55.0, "app_send", 0),
                packet_row(1, 55.0, "nwk_enqueue", 0),
                packet_row(2, 56.0, "nwk_forward", 0, next_hop=2),
                packet_row(3, 59.99999999999999, "nwk_delivery", 2),
            ]
            rows[0]["size_bytes"] = "10"
            rows[-1]["size_bytes"] = "10"
            write_trace(trace, rows)
            _, near_boundary = analyzer.analyze_trace(
                trace, bucket_width_s=60.0, stop_time_s=60.0
            )
            self.assertEqual(
                near_boundary["counts"][
                    "end_to_end_bucket_excluded_at_stop_packets"
                ],
                1,
            )
            self.assertEqual(
                sum(
                    bucket["packet_count"]
                    for bucket in near_boundary["end_to_end_buckets"]
                ),
                near_boundary["counts"]["end_to_end_bucket_packets"],
            )
            _, aggregate_provenance = aggregate.derive_series(
                trace, "near-stop-boundary", 60.0, 60.0
            )
            self.assertEqual(
                aggregate_provenance["window"]["excluded_at_stop"],
                {"nwk_delivery": 1},
            )

            rows[-1] = packet_row(3, 61.0, "nwk_delivery", 2)
            write_trace(trace, rows)
            _, after_boundary = analyzer.analyze_trace(
                trace, bucket_width_s=60.0, stop_time_s=60.0
            )
            self.assertEqual(
                after_boundary["counts"][
                    "end_to_end_bucket_excluded_at_stop_packets"
                ],
                0,
            )
            self.assertEqual(
                after_boundary["counts"][
                    "end_to_end_bucket_excluded_after_stop_packets"
                ],
                1,
            )

    def test_strict_cli_writes_evidence_then_fails_orphan_delivery(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            summary = root / "summary.json"
            packets_csv = root / "packets.csv"
            write_trace(trace, [packet_row(0, 2.0, "nwk_delivery", 2)])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(trace),
                    "--summary-json",
                    str(summary),
                    "--packet-csv",
                    str(packets_csv),
                    "--strict",
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

            self.assertEqual(result.returncode, 1)
            self.assertTrue(summary.is_file())
            self.assertTrue(packets_csv.is_file())
            report = json.loads(summary.read_text(encoding="utf-8"))
            self.assertFalse(report["pass"])
            self.assertEqual(report["counts"]["incomplete_delivered_packets"], 1)
            self.assertIn("missing_app_send", report["issue_counts"])

    def test_rejects_global_timestamp_regression(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                [
                    packet_row(0, 2.0, "app_send", 0),
                    packet_row(1, 1.0, "nwk_enqueue", 0),
                ],
            )

            with self.assertRaisesRegex(
                analyzer.PathAnalysisError, "precedes prior source time"
            ):
                analyzer.analyze_trace(trace)

    def test_rejects_physically_truncated_csv_row(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            row = packet_row(0, 1.0, "app_send", 0)
            with trace.open("w", encoding="utf-8", newline="") as stream:
                writer = csv.writer(stream, lineterminator="\n")
                writer.writerow(TRACE_COLUMNS)
                writer.writerow([row[column] for column in TRACE_COLUMNS[:-2]])

            with self.assertRaisesRegex(analyzer.PathAnalysisError, "too few"):
                analyzer.analyze_trace(trace)

    def test_cli_rejects_hard_link_output_alias(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            trace = root / "trace.csv"
            summary_alias = root / "summary.json"
            packets_csv = root / "packets.csv"
            write_trace(
                trace,
                [
                    packet_row(0, 1.0, "app_send", 0),
                    packet_row(1, 1.1, "nwk_enqueue", 0),
                ],
            )
            original = trace.read_bytes()
            os.link(trace, summary_alias)

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT),
                    str(trace),
                    "--summary-json",
                    str(summary_alias),
                    "--packet-csv",
                    str(packets_csv),
                ],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("must not alias", result.stderr)
            self.assertEqual(trace.read_bytes(), original)
            self.assertFalse(packets_csv.exists())


if __name__ == "__main__":
    unittest.main()
