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
    peer: int | None = None,
    next_hop: int | None = None,
    reason: str = "",
    detail: str = "",
) -> dict[str, str]:
    return trace_row(
        index,
        time_s,
        event,
        node=str(node),
        peer="" if peer is None else str(peer),
        src=str(source),
        dst=str(destination),
        sequence=str(sequence),
        next_hop="" if next_hop is None else str(next_hop),
        reason=reason,
        detail=detail,
    )


def hop_admission_row(
    index: int,
    time_s: float,
    node: int,
    peer: int,
    hop_sequence: int,
    source: int = 0,
    destination: int = 3,
    sequence: int = 42,
) -> dict[str, str]:
    return trace_row(
        index,
        time_s,
        "hop_admission",
        node=str(node),
        peer=str(peer),
        next_hop=str(peer),
        src=str(source),
        dst=str(destination),
        sequence=str(sequence),
        reason="admitted",
        detail=f"hop_sequence={hop_sequence}",
    )


def hop_feedback_row(
    index: int,
    time_s: float,
    node: int,
    peer: int,
    hop_sequence: int,
    reason: str,
    nsdp_before: int,
    nsdp_after: int,
    nsdp_state_valid: bool = True,
    source: int = 0,
    destination: int = 3,
    sequence: int = 42,
) -> dict[str, str]:
    return trace_row(
        index,
        time_s,
        "hop_feedback",
        node=str(node),
        peer=str(peer),
        src=str(source),
        dst=str(destination),
        sequence=str(sequence),
        reason=reason,
        detail=(
            f"hop_sequence={hop_sequence};first_reception=1;ackable=1;"
            f"nsdp_count_before={nsdp_before};nsdp_count_after={nsdp_after};"
            f"nsdp_state_valid={1 if nsdp_state_valid else 0};nsdp_limit=16"
        ),
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


def repeated_reordered_branch_rows(
    first_feedback_reason: str = "dack",
    queue_delay_fault: str | None = None,
) -> list[dict[str, str]]:
    """Build two overlapping suffixes whose HOP feedback arrives out of order."""

    first_before = 16 if first_feedback_reason == "dack" else 0
    rows = [
        packet_row(0, 1.0, "app_send", 0, destination=3),
        packet_row(1, 1.0, "nwk_enqueue", 0, destination=3, reason="local"),
        packet_row(
            2,
            2.0,
            "nwk_forward",
            0,
            destination=3,
            next_hop=1,
            reason="local",
        ),
        hop_admission_row(3, 2.0, 0, 1, 7),
        packet_row(
            4,
            3.0,
            "nwk_enqueue",
            1,
            destination=3,
            peer=0,
            reason="relay",
        ),
        hop_feedback_row(
            5,
            3.0,
            1,
            0,
            7,
            first_feedback_reason,
            first_before,
            first_before + 1,
        ),
        packet_row(
            6,
            3.2,
            "nwk_enqueue",
            1,
            destination=3,
            peer=0,
            reason="relay",
        ),
        hop_feedback_row(
            7,
            3.2,
            1,
            0,
            7,
            "dack" if first_feedback_reason == "dack" else "ack",
            17 if first_feedback_reason == "dack" else 1,
            18 if first_feedback_reason == "dack" else 2,
        ),
        packet_row(
            8,
            4.0,
            "nwk_forward",
            1,
            destination=3,
            peer=0,
            next_hop=2,
            reason="relay",
        ),
        hop_admission_row(9, 4.0, 1, 2, 989),
        packet_row(
            10,
            4.1,
            "nwk_forward",
            1,
            destination=3,
            peer=0,
            next_hop=2,
            reason="relay",
        ),
        hop_admission_row(11, 4.1, 1, 2, 990),
        # ACK for HOP sequence 990 precedes 989.  Exact directed identity,
        # rather than FIFO arrival order, must bind these receptions.
        packet_row(
            12,
            5.0,
            "nwk_enqueue",
            2,
            destination=3,
            peer=1,
            reason="relay",
        ),
        hop_feedback_row(13, 5.0, 2, 1, 990, "ack", 0, 1),
        packet_row(
            14,
            6.0,
            "nwk_enqueue",
            2,
            destination=3,
            peer=1,
            reason="relay",
        ),
        hop_feedback_row(15, 6.0, 2, 1, 989, "ack", 1, 2),
        packet_row(
            16,
            7.0,
            "nwk_forward",
            2,
            destination=3,
            peer=1,
            next_hop=3,
            reason="relay",
        ),
        hop_admission_row(17, 7.0, 2, 3, 110),
        packet_row(
            18,
            7.1,
            "nwk_forward",
            2,
            destination=3,
            peer=1,
            next_hop=3,
            reason="relay",
        ),
        hop_admission_row(19, 7.1, 2, 3, 111),
        hop_feedback_row(
            20, 8.0, 3, 2, 111, "ack", 0, 0, nsdp_state_valid=False
        ),
        packet_row(
            21,
            8.0,
            "nwk_delivery",
            3,
            destination=3,
            peer=2,
            reason="delivered",
        ),
        hop_feedback_row(
            22, 9.0, 3, 2, 110, "ack", 0, 0, nsdp_state_valid=False
        ),
        packet_row(
            23,
            9.0,
            "nwk_delivery",
            3,
            destination=3,
            peer=2,
            reason="delivered",
        ),
    ]
    # The frozen trace emits this direct statistic immediately before every
    # nwk_forward.  Crossed residence values deliberately select non-FIFO
    # copies at both overlapping relay stages.
    delay_by_forward_index = {
        2: 1.0,
        8: 0.8,
        10: 1.1,
        16: 1.0,
        18: 2.1,
    }
    expanded: list[dict[str, str]] = []
    for row in rows:
        if row["event"] == "nwk_forward":
            old_forward_index = int(row["event_index"])
            faulted = old_forward_index == 8
            if not (faulted and queue_delay_fault == "missing"):
                delay = delay_by_forward_index[old_forward_index]
                sample_node = row["node"]
                if faulted and queue_delay_fault == "wrong-node":
                    sample_node = "99"
                if faulted and queue_delay_fault == "wrong-delay":
                    delay += 0.25
                expanded.append(
                    trace_row(
                        0,
                        float(row["time_s"]),
                        "statistic_sample",
                        node=sample_node,
                        statistic=analyzer.NWK_QUEUE_DELAY_STATISTIC,
                        value=str(delay),
                    )
                )
                if faulted and queue_delay_fault == "non-adjacent":
                    expanded.append(
                        trace_row(
                            0,
                            float(row["time_s"]),
                            "statistic_sample",
                            node=row["node"],
                            statistic="NWK.Network Queue Size (packets)",
                            value="0",
                        )
                    )
        expanded.append(row)
    for event_index, row in enumerate(expanded):
        row["event_index"] = str(event_index)
    return expanded


class AnalyzePacketPathsTests(unittest.TestCase):
    def test_reconstructs_reordered_repeated_dack_suffixes_by_exact_hop_identity(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(trace, repeated_reordered_branch_rows())

            packets, report = analyzer.analyze_trace(
                trace, startup_time_s=0.0, bucket_width_s=10.0
            )

            self.assertTrue(report["pass"])
            self.assertEqual(report["counts"]["packet_keys"], 1)
            self.assertEqual(report["counts"]["packet_lifecycles"], 2)
            self.assertEqual(report["counts"]["delivered_packets"], 2)
            self.assertEqual(
                report["counts"]["reconstructed_branch_packet_keys"], 1
            )
            self.assertEqual(
                report["counts"]["ambiguous_repeated_dack_branch_packet_keys"],
                0,
            )
            self.assertEqual(report["counts"]["source_exact_dack_retry_forks"], 1)
            self.assertEqual(report["counts"]["invalid_packets"], 0)
            self.assertEqual(report["overall"]["packet_count"], 2)
            self.assertEqual(
                sum(bucket["packet_count"] for bucket in report["end_to_end_buckets"]),
                2,
            )
            self.assertEqual({tuple(packet.path) for packet in packets}, {(0, 1, 2, 3)})
            self.assertEqual({packet.copy_index for packet in packets}, {0, 1})
            self.assertEqual(
                report["configuration"]["repeated_dack_branch_matching"],
                "exact directed-HOP identity plus queue-delay-constrained unique "
                "bipartite queue-copy matching; never FIFO",
            )
            self.assertEqual(
                report["configuration"]["queue_delay_match_abs_tolerance_s"],
                1.0e-9,
            )

            observed_transits: dict[int, float] = {}
            observed_residences: dict[int, tuple[float, ...]] = {}
            for packet in packets:
                self.assertTrue(packet.valid)
                self.assertTrue(packet.complete)
                self.assertEqual(len(packet.hop_lineage), 3)
                for position, leg in enumerate(packet.hop_lineage):
                    if leg.hop_sequence in {989, 990}:
                        observed_transits[leg.hop_sequence] = float(
                            packet.leg_transits[position]["transit_s"]
                        )
                observed_residences[packet.hop_lineage[1].hop_sequence] = tuple(
                    float(residence["residence_s"])
                    for residence in packet.nwk_residences
                )
            # HOP 990 is received first even though 989 was admitted first.
            # Exact tuple matching preserves 990 -> 5.0 and 989 -> 6.0.
            self.assertAlmostEqual(observed_transits[990], 0.9)
            self.assertAlmostEqual(observed_transits[989], 2.0)
            for observed, expected in zip(
                observed_residences[990], (1.0, 1.1, 2.1)
            ):
                self.assertAlmostEqual(observed, expected)
            for observed, expected in zip(
                observed_residences[989], (1.0, 0.8, 1.0)
            ):
                self.assertAlmostEqual(observed, expected)

    def test_repeated_dack_reconstruction_is_candidate_order_independent(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(trace, repeated_reordered_branch_rows())
            loaded = analyzer.load_trace(trace)
            key = next(iter(loaded.packets))

            normal = analyzer._reconstruct_repeated_dack_branches(
                key,
                loaded.packets[key],
                loaded.hop_evidence[key],
                loaded.queue_delay_by_forward,
            )
            reversed_candidates = analyzer._reconstruct_repeated_dack_branches(
                key,
                loaded.packets[key],
                loaded.hop_evidence[key],
                loaded.queue_delay_by_forward,
                reverse_candidate_order=True,
            )

            self.assertIsNotNone(normal)
            self.assertIsNotNone(reversed_candidates)

            def snapshot(reconstruction):
                return [
                    (
                        tuple(event.event_index for event in lifecycle.events),
                        tuple(
                            (
                                leg.from_node,
                                leg.to_node,
                                leg.hop_sequence,
                            )
                            for leg in lifecycle.hop_lineage
                        ),
                    )
                    for lifecycle in reconstruction.lifecycles
                ]

            self.assertEqual(snapshot(normal), snapshot(reversed_candidates))

    def test_repeated_dack_queue_delay_evidence_fails_closed(self) -> None:
        for fault in ("missing", "non-adjacent", "wrong-node", "wrong-delay"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as temporary:
                trace = Path(temporary) / "trace.csv"
                write_trace(
                    trace,
                    repeated_reordered_branch_rows(queue_delay_fault=fault),
                )

                packets, report = analyzer.analyze_trace(trace)

                self.assertFalse(report["pass"])
                self.assertEqual(len(packets), 1)
                self.assertEqual(
                    report["counts"]["reconstructed_branch_packet_keys"], 0
                )
                self.assertEqual(
                    report["counts"][
                        "ambiguous_repeated_dack_branch_packet_keys"
                    ],
                    0,
                )
                self.assertIn("duplicate_delivery", report["issue_counts"])

    def test_repeated_dack_queue_matching_must_be_unique(self) -> None:
        rows = repeated_reordered_branch_rows()
        node_one_receptions = [
            row
            for row in rows
            if row["event"] in {"nwk_enqueue", "hop_feedback"}
            and row["node"] == "1"
        ]
        node_one_receptions[2]["time_s"] = "3.0000000005"
        node_one_receptions[3]["time_s"] = "3.0000000005"
        for position, row in enumerate(rows):
            if row["event"] == "nwk_forward" and row["time_s"] == "4.0":
                rows[position - 1]["value"] = "1.0"
                break

        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(trace, rows)

            packets, report = analyzer.analyze_trace(trace)

            self.assertFalse(report["pass"])
            self.assertEqual(len(packets), 1)
            self.assertEqual(report["counts"]["reconstructed_branch_packet_keys"], 0)
            self.assertEqual(
                report["counts"]["ambiguous_repeated_dack_branch_packet_keys"],
                1,
            )
            self.assertIn("duplicate_delivery", report["issue_counts"])

    def test_duplicate_suffixes_without_prior_dack_remain_strict_failure(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            trace = Path(temporary) / "trace.csv"
            write_trace(
                trace,
                repeated_reordered_branch_rows(first_feedback_reason="ack"),
            )

            packets, report = analyzer.analyze_trace(trace)

            self.assertFalse(report["pass"])
            self.assertEqual(len(packets), 1)
            self.assertEqual(report["counts"]["packet_lifecycles"], 1)
            self.assertEqual(report["counts"]["reconstructed_branch_packet_keys"], 0)
            self.assertIn("duplicate_delivery", report["issue_counts"])

    def test_compact_trace_requires_admissions_only_for_repeated_branches(
        self,
    ) -> None:
        with (
            self.subTest(shape="ordinary"),
            tempfile.TemporaryDirectory() as temporary,
        ):
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

        with (
            self.subTest(shape="repeated"),
            tempfile.TemporaryDirectory() as temporary,
        ):
            trace = Path(temporary) / "trace.csv"
            write_compact_trace(
                trace,
                [
                    row
                    for row in repeated_reordered_branch_rows()
                    if row["event"] != "hop_admission"
                ],
            )

            with self.assertRaisesRegex(
                analyzer.PathAnalysisError,
                r"1 packet key\(s\).*--admissionTrace=1",
            ) as raised:
                analyzer.analyze_trace(trace)

            self.assertIn("refusing to infer HOP identity", str(raised.exception))

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
